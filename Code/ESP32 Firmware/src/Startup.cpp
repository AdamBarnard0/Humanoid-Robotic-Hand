#include "Startup.h"
#include "Config.h"
#include "HardwareIO.h"
#include "JointMap.h"
#include "Movement.h"
#include "Calibration.h"
#include "Diagnostics.h"
#include "Feedback.h"
#include <Wire.h>

// Boot without moving anything.
//
// Every joint is read from its feedback pot and then commanded to the
// position it is already in, so the first pulse it receives matches where it
// is standing. Previously boot called centerAllJoints(), which drove all 19
// joints to 1500us — a jump from whatever pose the hand was left in.
//
// A joint that cannot be read (rail off, never homed, feedback uncalibrated)
// falls back to neutral, since without a reading there is nothing better to
// aim at.
static void holdMeasuredPositions() {
  uint8_t held = 0, centred = 0, offCount = 0;

  Serial.println(F("\nHolding current position (reading feedback)..."));

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    JointId id = (JointId)i;
    Joint &j = joints[id];

    if (!j.attached) { offCount++; continue; }

    if (jointFeedbackTrustworthy(id)) {
      float deg = readJointAngleDeg(id, FEEDBACK_SAMPLES);
      float loDeg, hiDeg;
      jointAngleLimits(id, loDeg, hiDeg);
      if (deg < loDeg) deg = loDeg;
      if (deg > hiDeg) deg = hiDeg;

      setJointAngleDegImmediate(id, deg);
      held++;
    } else {
      // No usable reading — neutral, written directly so the wrist's rate
      // limiter doesn't ramp from an assumed position.
      hwSetJointPulseUs(j, SERVO_CENTER_US);
      j.currentPulseUs = SERVO_CENTER_US;
      centred++;
    }
  }

  Serial.print(F("  "));
  Serial.print(held);
  Serial.print(F(" held at measured position, "));
  Serial.print(centred);
  Serial.print(F(" centred (no usable feedback), "));
  Serial.print(offCount);
  Serial.println(F(" motor(s) off."));
  if (centred > 0) {
    Serial.println(F("  ('fb' shows which joints have a calibrated channel)"));
  }
}

void startupSequence() {
  // Buffer sizes must be set BEFORE begin() on the ESP32.
  Serial.setRxBufferSize(SERIAL_RX_BUFFER);
  Serial.setTxBufferSize(SERIAL_TX_BUFFER);
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_READ_TIMEOUT_MS);
  delay(300);
  Serial.println(F("\nHRH1 hand - firmware starting"));

  hwInitPins();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  hwI2cScan();
  hwInitIna219();   // whole-rail current sensor; wrist joints report this

  // Bounds first — neither of these touches hardware, so they run even if
  // the PCA9685 is missing (keeps 'list' meaningful while debugging wiring).
  // Order matters: MANUAL_BOUNDS must win over whatever was in flash.
  calibrationLoadAll();
  applyManualBoundsOverride();

  if (pca9685Present == true) {
    hwInitPCA9685();
    holdMeasuredPositions();
  } else {
    Serial.println(F("WARNING: PCA9685 not found on I2C bus. Check wiring/power before continuing."));
  }

  printMenu();
}

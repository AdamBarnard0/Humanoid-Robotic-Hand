#include "Safety.h"
#include "Config.h"
#include "JointMap.h"
#include "HardwareIO.h"
#include "Movement.h"
#include "Feedback.h"
#include "CurrentMonitor.h"
#include "Protocol.h"   // controlModeGet(): the PC link never gets unprompted text
#include <math.h>
#include <stdio.h>

// ===========================================================================
// State
// ===========================================================================
static bool        estop = false;
static uint32_t    faultMask = 0;
static FaultReason faultWhy[NUM_JOINTS];
static uint8_t     pauseDepth = 0;

// Current limit: millis() when the joint first read over the limit, 0 = not
// over. The wrist motors share one rail reading, so they share one timer.
static uint32_t overSinceMs[NUM_JOINTS];
static uint32_t railOverSinceMs = 0;
static uint32_t lastRailMs = 0;

// Deviation: the reference point the "has it stopped moving?" test measures
// from. Reset whenever the measured OR commanded angle moves out of the band.
static bool     devRefValid[NUM_JOINTS];
static float    devRefMeas[NUM_JOINTS];
static float    devRefCmd[NUM_JOINTS];
static uint32_t devRefMs[NUM_JOINTS];

static bool terminalMode() { return controlModeGet() == MODE_TERMINAL; }

static void clearTracking() {
  railOverSinceMs = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    overSinceMs[i] = 0;
    devRefValid[i] = false;
  }
}

// ===========================================================================
// E-stop
// ===========================================================================
void safetyEstop(const char *source) {
  if (estop) return;
  estop = true;
  hwOutputsLockAll();
  if (terminalMode()) {
    Serial.print(F("\n*** E-STOP ("));
    Serial.print(source);
    Serial.println(F(") — all motor outputs off. Type 'reset' to re-enable. ***"));
  }
}

bool safetyEstopActive() { return estop; }

// Commands one joint to where it is now, so re-enabling it doesn't move it.
static void holdWhereItIs(JointId id) {
  Joint &j = joints[id];
  if (!j.attached) return;
  if (jointFeedbackTrustworthy(id)) {
    float deg = readJointAngleDeg(id, FEEDBACK_SAMPLES);
    float lo, hi;
    jointAngleLimits(id, lo, hi);
    if (deg < lo) deg = lo;
    if (deg > hi) deg = hi;
    setJointAngleDegImmediate(id, deg);
  } else if (j.currentPulseUs != 0) {
    hwSetJointPulseUs(j, j.currentPulseUs);   // best guess: its last command
  }
}

void safetyReset() {
  const bool     wasEstop = estop;
  const uint32_t wasFaulted = faultMask;

  estop     = false;
  faultMask = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if ((wasFaulted >> i) & 1UL) joints[i].currentMA = 0.0f;   // stale reading
    faultWhy[i] = FAULT_NONE;
  }
  clearTracking();

  // Unlock in software but keep the PCA9685 outputs physically off while
  // the new pulses are written, then switch them on together.
  hwOutputsUnlockAll();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (wasEstop || ((wasFaulted >> i) & 1UL)) holdWhereItIs((JointId)i);
  }
  hwOutputEnablePin(true);

  if (terminalMode()) {
    Serial.println(wasEstop || wasFaulted ? F("Reset: outputs re-enabled, joints holding where they are.")
                                          : F("Reset: nothing was stopped."));
  }
}

// ===========================================================================
// Faults
// ===========================================================================
uint32_t    safetyFaultMask() { return faultMask; }
FaultReason safetyFaultReason(JointId id) { return (id < NUM_JOINTS) ? faultWhy[id] : FAULT_NONE; }

const char *safetyFaultName(FaultReason r) {
  switch (r) {
    case FAULT_CURRENT:   return "current";
    case FAULT_DEVIATION: return "deviation";
    default:              return "none";
  }
}

static void tripJoint(uint8_t i, FaultReason why, float value) {
  if ((faultMask >> i) & 1UL) return;
  faultMask  |= (1UL << i);
  faultWhy[i] = why;
  hwLockJoint(i);
  if (terminalMode()) {
    Serial.print(F("\n*** PROTECTION: "));
    Serial.print(joints[i].name);
    if (why == FAULT_CURRENT) {
      Serial.print(F(" over current ("));
      Serial.print(value, 0);
      Serial.print(F(" mA for > "));
      Serial.print(PROT_CURRENT_HOLD_MS);
      Serial.print(F(" ms)"));
    } else {
      Serial.print(F(" stopped "));
      Serial.print(value, 1);
      Serial.print(F(" deg from its target"));
    }
    Serial.println(F(" — motor disabled. 'faults' lists, 'reset' clears. ***"));
  }
}

// ===========================================================================
// Protection checks
// ===========================================================================

// Over-limit for longer than the hold time? 'since' is the timer to update.
static bool overForTooLong(float mA, float limit, uint32_t &since, uint32_t now) {
  if (!(mA > limit)) { since = 0; return false; }
  // 0 means "not over", so a start time of exactly 0 is stored as 1. (Not
  // now|1, which can sit 1 ms in the FUTURE and make now - since underflow
  // into an instant trip.)
  if (since == 0) { since = (now != 0) ? now : 1UL; return false; }
  return (uint32_t)(now - since) >= (uint32_t)PROT_CURRENT_HOLD_MS &&
         (int32_t)(now - since) >= 0;
}

static void checkCurrent(uint32_t now) {
#if PROT_CURRENT_ENABLE
  bool wristChecked = false;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    if (!j.attached || ((faultMask >> i) & 1UL)) continue;

    if (j.driver == DRV_PCA9685) {
      if (overForTooLong(j.currentMA, PROT_FINGER_CURRENT_MA, overSinceMs[i], now)) {
        tripJoint(i, FAULT_CURRENT, j.currentMA);
      }
    } else if (!wristChecked && ina219Present) {
      // One rail reading for all three wrist motors: there's no telling which
      // of them is drawing it, so all three are stopped together.
      wristChecked = true;
      if (overForTooLong(j.currentMA, PROT_WRIST_RAIL_MA, railOverSinceMs, now)) {
        const float mA = j.currentMA;
        for (uint8_t k = 0; k < NUM_JOINTS; k++) {
          if (joints[k].driver == DRV_LEDC && joints[k].attached) tripJoint(k, FAULT_CURRENT, mA);
        }
      }
    }
  }
#else
  (void)now;
#endif
}

static void checkDeviation(uint32_t now) {
#if PROT_DEV_ENABLE
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if ((faultMask >> i) & 1UL) continue;
    float meas;
    if (!feedbackFilteredAngle((JointId)i, PROT_FB_MAX_AGE_MS, meas)) {
      devRefValid[i] = false;          // can't see it, can't judge it
      continue;
    }
    const float cmd = jointCommandedAngleDeg((JointId)i);

    // Moving — either the joint itself, or the command it's following?
    // Then it's on its way somewhere: restart the stillness clock.
    if (!devRefValid[i] ||
        fabsf(meas - devRefMeas[i]) > PROT_STILL_BAND_DEG ||
        fabsf(cmd  - devRefCmd[i])  > PROT_STILL_BAND_DEG) {
      devRefValid[i] = true;
      devRefMeas[i]  = meas;
      devRefCmd[i]   = cmd;
      devRefMs[i]    = now;
      continue;
    }

    // Settled. Only now is a large error a fault rather than a joint in
    // transit.
    const float err = fabsf(meas - cmd);
    if ((now - devRefMs[i]) >= (uint32_t)PROT_STILL_MS && err > PROT_DEV_LIMIT_DEG) {
      tripJoint(i, FAULT_DEVIATION, err);
    }
  }
#else
  (void)now;
#endif
}

void safetyService() {
  if (pauseDepth > 0) return;          // a routine owns the hardware right now

  feedbackServiceTick();               // one joint's pot, round-robin
  currentMonitorStep();                // LTC2497: one transaction per 160 ms at most

  const uint32_t now = millis();
  if (now - lastRailMs >= (uint32_t)PROT_RAIL_SAMPLE_MS) {
    lastRailMs = now;
    currentMonitorSampleRail();        // INA219 -> the wrist joints' currentMA
  }

  if (estop) return;                   // everything is off already
  checkCurrent(now);
  checkDeviation(now);
}

void safetyPause() { if (pauseDepth < 255) pauseDepth++; }

void safetyResume() {
  if (pauseDepth == 0) return;
  if (--pauseDepth == 0) clearTracking();   // judge afresh from here on
}

// ===========================================================================
// Serial helpers for blocking routines
// ===========================================================================
bool safetyDrainSerial() {
  bool other = false;
  while (Serial.available()) {
    int c = Serial.read();
    if (c == ESTOP_CHAR) safetyEstop("serial");
    else if (c >= 0)     other = true;
  }
  return other;
}

bool safetyRoutinePoll(bool anyByteAborts) {
  safetyService();
  if (anyByteAborts) {
    if (safetyDrainSerial()) return true;
  } else {
    while (Serial.available() && Serial.peek() == ESTOP_CHAR) {
      Serial.read();
      safetyEstop("serial");
    }
  }
  return estop;
}

bool safetyWait(uint32_t ms, bool anyByteAborts) {
  const uint32_t start = millis();
  do {
    if (safetyRoutinePoll(anyByteAborts)) return false;
    delay(1);
  } while (millis() - start < ms);
  return true;
}

// ===========================================================================
// Reporting
// ===========================================================================
void safetyPrintStatus() {
  Serial.print(F("\nE-stop: "));
  Serial.println(estop ? F("ACTIVE") : F("off"));
  if (faultMask == 0) {
    Serial.println(F("Faults: none"));
  } else {
    Serial.println(F("Faults (motor disabled until 'reset'):"));
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      if (!((faultMask >> i) & 1UL)) continue;
      Serial.print(F("  "));
      Serial.print(joints[i].name);
      Serial.print(F(": "));
      Serial.println(safetyFaultName(faultWhy[i]));
    }
  }
  if (pauseDepth > 0) Serial.println(F("(protection paused)"));
  Serial.println();
}

int safetyFormatFaults(char *buf, size_t cap) {
  int n = snprintf(buf, cap, "estop=%d", estop ? 1 : 0);
  for (uint8_t i = 0; i < NUM_JOINTS && n > 0 && (size_t)n < cap; i++) {
    if (!((faultMask >> i) & 1UL)) continue;
    n += snprintf(buf + n, cap - (size_t)n, " %s=%s", joints[i].name, safetyFaultName(faultWhy[i]));
  }
  return n;
}

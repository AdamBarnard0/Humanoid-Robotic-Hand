#include "HardwareIO.h"
#include "Config.h"
#include "JointMap.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

static Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);
bool pca9685Present = false;

const uint8_t LTC2497_CH_CMD[16] = {
  0xB0, 0xB8, 0xB1, 0xB9, 0xB2, 0xBA, 0xB3, 0xBB,
  0xB4, 0xBC, 0xB5, 0xBD, 0xB6, 0xBE, 0xB7, 0xBF
};

static uint16_t usToTicks(uint16_t us) {
  float ticks = (us * 4096.0f) / (1000000.0f / SERVO_FREQ_HZ);
  if (ticks < 0) ticks = 0;
  if (ticks > 4095) ticks = 4095;
  return (uint16_t)ticks;
}

// Sets up one LEDC channel for servo-style PWM on a GPIO. The Arduino core
// changed this API at 3.0 — ledcAttach() replaced ledcSetup()+ledcAttachPin()
// and now picks the channel itself, so the explicit channel number only
// matters on 2.x.
static void ledcInitChannel(uint8_t ch, uint8_t pin) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcAttach(pin, LEDC_FREQ_HZ, LEDC_RES_BITS);
#else
  ledcSetup(ch, LEDC_FREQ_HZ, LEDC_RES_BITS);
  ledcAttachPin(pin, ch);
#endif
}

static void ledcWriteDuty(const Joint &j, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(j.pwmPin, duty);   // 3.x addresses the channel by pin
#else
  ledcWrite(j.ledcCh, duty);
#endif
}

// Pulse width -> LEDC duty. One PWM period is 1/LEDC_FREQ_HZ seconds, so a
// pulse of 'us' is that fraction of the full-scale duty count.
static uint32_t usToDuty(uint16_t us) {
  const uint32_t full = (1UL << LEDC_RES_BITS) - 1UL;
  const uint32_t periodUs = 1000000UL / LEDC_FREQ_HZ;
  uint32_t duty = ((uint32_t)us * full) / periodUs;
  if (duty > full) duty = full;
  return duty;
}

void hwInitPins() {
  // Allocate an LEDC channel to each directly-driven joint, but emit NO
  // pulses yet — duty 0 leaves the line low and the servo limp.
  //
  // Deliberately not neutral: startupSequence() decides where each joint
  // should go only after reading its feedback pot, and writing 1500us here
  // would jerk the wrist to centre before that reading ever happened.
  uint8_t nextCh = LEDC_FIRST_CH;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    if (j.driver != DRV_LEDC) continue;
    j.ledcCh = nextCh++;
    ledcInitChannel(j.ledcCh, j.pwmPin);
    ledcWriteDuty(j, 0);
  }

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);
  pinMode(MUX_EN, OUTPUT);
  digitalWrite(MUX_EN, MUX_EN_ACTIVE_LOW ? LOW : HIGH); // enabled

  analogReadResolution(12);
  analogSetPinAttenuation(MUX_SIG, ADC_11db); // full 0-3.3V range

  // Wrist pots bypass the mux and land on their own ADC1 pins — they need
  // the same attenuation or they'd saturate above ~1.1V.
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].fbSrc == FB_ADC) analogSetPinAttenuation(joints[i].adcPin, ADC_11db);
  }

  pinMode(PCA9685_OE_PIN, OUTPUT);
  digitalWrite(PCA9685_OE_PIN, LOW); // LOW = outputs enabled on this board
}

bool hwInitPCA9685() {
  if (!pca9685Present) return false;
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ_HZ);
  delay(10);
  return true;
}

// ---------------------------------------------------------------------------
// Output lock — the single point every motor command passes through.
//
// While the whole hand is locked (E-stop), or one joint is locked (that joint
// has a protection fault), pulse writes are silently dropped. Gating here
// rather than in each caller means no routine, gesture, demo or PC frame can
// move a stopped motor, including code paths that bypass Movement.cpp.
// ---------------------------------------------------------------------------
static bool     outLockedAll  = false;
static uint32_t outLockedMask = 0;

void hwSetChannelPulseUs(uint8_t pcaChannel, uint16_t us) {
  if (outLockedAll) return;
  pca.setPWM(pcaChannel, 0, usToTicks(us));
}

void hwDisableChannel(uint8_t pcaChannel) {
  // off = 4096 sets the channel's FULL_OFF bit — output held low, no pulse
  // train, so the servo on this channel alone loses holding torque.
  pca.setPWM(pcaChannel, 0, 4096);
}

void hwSetJointPulseUs(const Joint &j, uint16_t us) {
  const uint8_t idx = (uint8_t)(&j - joints);
  if (outLockedAll || ((outLockedMask >> idx) & 1UL)) return;
  if (j.driver == DRV_LEDC) ledcWriteDuty(j, usToDuty(us));
  else                      hwSetChannelPulseUs(j.pcaChannel, us);
}

void hwOutputsLockAll() {
  outLockedAll = true;
  // 1. Output-enable pin: one GPIO write turns off all 16 PCA9685 outputs in
  //    hardware, instantly, without depending on the I2C bus.
  digitalWrite(PCA9685_OE_PIN, HIGH);
  // 2. Wrist: duty 0, no pulses.
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].driver == DRV_LEDC) ledcWriteDuty(joints[i], 0);
  }
  // 3. Also set every PCA channel to FULL_OFF, so the outputs stay off even
  //    if the enable pin is re-asserted before new pulses are written.
  if (pca9685Present) {
    for (uint8_t ch = 0; ch < 16; ch++) hwDisableChannel(ch);
  }
}

void hwOutputsUnlockAll() {
  outLockedAll  = false;
  outLockedMask = 0;
}

void hwOutputEnablePin(bool enable) {
  digitalWrite(PCA9685_OE_PIN, enable ? LOW : HIGH);   // LOW = enabled
}

void hwLockJoint(uint8_t idx) {
  if (idx >= NUM_JOINTS) return;
  outLockedMask |= (1UL << idx);
  hwDisableJoint(joints[idx]);
}

bool hwOutputsLocked() { return outLockedAll; }

void hwDisableJoint(const Joint &j) {
  // Duty 0 leaves the GPIO low with no pulses — same effect as the
  // PCA9685's FULL_OFF bit: the servo goes limp.
  if (j.driver == DRV_LEDC) ledcWriteDuty(j, 0);
  else                      hwDisableChannel(j.pcaChannel);
}

void hwMuxSelect(uint8_t ch) {
  digitalWrite(MUX_S0, (ch >> 0) & 0x01);
  digitalWrite(MUX_S1, (ch >> 1) & 0x01);
  digitalWrite(MUX_S2, (ch >> 2) & 0x01);
  digitalWrite(MUX_S3, (ch >> 3) & 0x01);
}

int hwReadMuxRaw(uint8_t ch) {
  hwMuxSelect(ch);
  delayMicroseconds(MUX_SETTLE_US);
  return analogRead(MUX_SIG);
}

float hwRawToVolts(int raw) {
  return ((float)raw / (float)ADC_MAX_COUNTS) * ADC_VREF_VOLTS;
}

// Sorts a small batch and averages the middle half: the top and bottom
// quarter are discarded (n=5 drops 1 each end, n=8 drops 2). A median would
// throw away more information; a plain mean lets one spike through.
static float trimmedMeanMv(uint16_t *v, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {                 // insertion sort, n <= 16
    uint16_t x = v[i];
    int8_t k = (int8_t)i - 1;
    while (k >= 0 && v[k] > x) { v[k + 1] = v[k]; k--; }
    v[k + 1] = x;
  }
  uint8_t drop = n / 4;
  uint32_t acc = 0;
  for (uint8_t i = drop; i < n - drop; i++) acc += v[i];
  return (float)acc / (float)(n - 2 * drop);
}

// One batch from an ADC1 pin, in volts.
//
// analogReadMilliVolts() rather than analogRead(): it applies the chip's
// factory eFuse calibration, which corrects the ESP32 ADC's offset and its
// non-linearity at 11 dB (worst above ~2.5 V). analogRead() scaled by
// 3.3/4095 assumes a straight line that the ADC doesn't have. This makes
// readings more ACCURATE — it doesn't reduce noise; the trimmed mean and the
// filter in Feedback.cpp do that.
static float readPinVolts(uint8_t pin, uint8_t samples) {
  if (samples == 0) samples = 1;
  if (samples > FEEDBACK_MAX_BATCH) samples = FEEDBACK_MAX_BATCH;
  uint16_t buf[FEEDBACK_MAX_BATCH];
  for (uint8_t i = 0; i < samples; i++) buf[i] = (uint16_t)analogReadMilliVolts(pin);
  return trimmedMeanMv(buf, samples) / 1000.0f;
}

// The mux needs MUX_SETTLE_US once after switching channel; the batch is
// then taken back to back.
float hwReadMuxVolts(uint8_t ch, uint8_t samples) {
  hwMuxSelect(ch);
  delayMicroseconds(MUX_SETTLE_US);
  return readPinVolts(MUX_SIG, samples);
}

float hwReadAdcVolts(uint8_t gpio, uint8_t samples) {
  return readPinVolts(gpio, samples);   // direct pin, nothing to settle
}

float hwReadJointVolts(const Joint &j, uint8_t samples) {
  if (j.fbSrc == FB_ADC) return hwReadAdcVolts(j.adcPin, samples);
  if (j.muxChannel < 0)  return 0.0f;
  return hwReadMuxVolts((uint8_t)j.muxChannel, samples);
}

// The ESP32 Arduino core has a known, occasional bug where a repeated-start
// read throws "i2cWriteReadNonStop Error -1" even on an otherwise healthy
// bus (core issue #9617). Empirically here it's an isolated, single-
// transaction hiccup, not a stuck bus — a fresh retry of the exact same
// transaction almost always succeeds immediately after. So the mitigation
// is just that: try again a couple of times before giving up, no bus reset
// involved.
uint32_t hwLtc2497Transact(uint8_t cmd) {
  for (uint8_t attempt = 0; attempt < LTC2497_MAX_ATTEMPTS; attempt++) {
    // Repeated start (endTransmission(false)) is required here, not
    // optional: the LTC2497 NAKs any read that arrives while it's
    // mid-conversion, and holding the bus with a repeated start between the
    // channel-select write and the following read is what lets this
    // pipelined write-then-read-previous-result sequence work reliably.
    Wire.beginTransmission(SERVO_CLICK_ADC_ADDR);
    Wire.write(cmd);
    uint8_t err = Wire.endTransmission(false);
    if (err != 0) continue;

    uint8_t got = Wire.requestFrom((uint8_t)SERVO_CLICK_ADC_ADDR, (uint8_t)3);
    if (got != 3) continue;

    uint32_t raw = 0;
    for (uint8_t i = 0; i < 3; i++) {
      raw = (raw << 8) | Wire.read();
    }
    return raw;
  }
  return LTC2497_READ_FAILED;
}

// ===========================================================================
// INA219 — whole-rail current
// ===========================================================================
bool ina219Present = false;

#define INA219_REG_CONFIG   0x00
#define INA219_REG_SHUNT    0x01

// Reset, then 32V range / +-320mV shunt / 12-bit, continuous shunt+bus.
#define INA219_CONFIG_VALUE 0x399F

static bool ina219Write16(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

static bool ina219Read16(uint8_t reg, uint16_t &out) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)INA219_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = (uint16_t)Wire.read();
  uint16_t lo = (uint16_t)Wire.read();
  out = (uint16_t)((hi << 8) | lo);
  return true;
}

bool hwInitIna219() {
  ina219Present = ina219Write16(INA219_REG_CONFIG, INA219_CONFIG_VALUE);
  Serial.print(F("INA219 rail sensor: "));
  Serial.println(ina219Present ? F("ready") : F("NOT FOUND (check address 0x45)"));
  return ina219Present;
}

float hwReadRailMilliamps() {
  if (!ina219Present) return NAN;
  uint16_t raw;
  if (!ina219Read16(INA219_REG_SHUNT, raw)) return NAN;

  // Shunt register is signed, LSB = 10uV.
  int16_t signedRaw = (int16_t)raw;
  float shuntMv = (float)signedRaw * 0.01f;          // 10uV steps -> mV
  return shuntMv / INA219_SHUNT_OHMS;                // mV / ohm = mA
}

bool hwI2cScan() {
  Serial.println(F("\n--- I2C bus scan ---"));
  uint8_t found = 0;
  pca9685Present = false;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      found++;
      Serial.print(F("  Found device at 0x"));
      if (addr < 16) Serial.print('0');
      Serial.print(addr, HEX);
      if (addr == PCA9685_ADDR) {
        Serial.print(F("  <- PCA9685 servo driver (expected)"));
        pca9685Present = true;
      } else if (addr == SERVO_CLICK_ADC_ADDR) {
        Serial.print(F("  <- Servo Click onboard current-sense ADC (expected)"));
      } else if (addr == INA219_ADDR) {
        Serial.print(F("  <- INA219 motor-rail current sensor (expected)"));
      } else {
        Serial.print(F("  <- unexpected/unknown device"));
      }
      Serial.println();
    }
  }
  if (found == 0) {
    Serial.println(F("  No I2C devices found. Check power to driver board."));
  }
  Serial.println(F("--- scan complete ---"));
  return pca9685Present;
}

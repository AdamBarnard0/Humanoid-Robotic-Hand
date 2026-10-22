#include "JointMap.h"
#include "Config.h"

// Fields common to both kinds of joint.
static void initCommon(Joint &j, const char* name, bool invert) {
  j.name = name;
  j.invert = invert;
  j.attached = true;

  j.centerPulseUs = SERVO_CENTER_US;
  j.minPulseUs = 0;
  j.maxPulseUs = 0;

  j.minBoundVolts = 0.0f;
  j.maxBoundVolts = 0.0f;

  j.targetAngleDeg = 0.0f;
  j.targetSpeedDegSec = 0.0f;
  j.commandedAngleDeg = 0.0f;

  j.currentPulseUs = SERVO_CENTER_US;
  j.currentMA = 0.0f;
  j.lastFeedbackVolts = 0.0f;
  j.currentAngleDeg = 0.0f;
}

// Servo Click joint: PWM from the PCA9685, feedback through the mux.
static Joint makePcaJoint(const char* name, uint8_t pcaChannel, bool invert) {
  Joint j;
  initCommon(j, name, invert);
  j.driver     = DRV_PCA9685;
  j.pcaChannel = pcaChannel;
  j.pwmPin     = 0;
  j.ledcCh     = 0;
  j.fbSrc      = FB_MUX;
  j.muxChannel = -1;        // discovered at runtime by 'fbfind'
  j.adcPin     = 0;
  return j;
}

// Wrist joint: PWM straight off a GPIO via LEDC, feedback on its own ADC1
// pin. No mux, and no per-motor current sensing.
static Joint makeWristJoint(const char* name, uint8_t pwmPin, uint8_t adcPin,
                            bool invert) {
  Joint j;
  initCommon(j, name, invert);
  j.driver     = DRV_LEDC;
  j.pcaChannel = 0;
  j.pwmPin     = pwmPin;
  j.ledcCh     = 0;        // assigned in hwInitPins()
  j.fbSrc      = FB_ADC;
  j.muxChannel = -1;
  j.adcPin     = adcPin;
  return j;
}

// The wiring itself lives in JOINT_WIRING_TABLE in Config.h — that's the
// one place to edit after an assembly. This just expands it into joints[].
Joint joints[NUM_JOINTS] = {
  #define XP(nm, ch, inv)          makePcaJoint(nm, ch, inv),
  #define XW(nm, pwm, adc, inv)    makeWristJoint(nm, pwm, adc, inv),
  JOINT_WIRING_TABLE
  #undef XP
  #undef XW
};

// ===========================================================================
// MANUAL BOUNDS OVERRIDE — optional, per-joint hard-set center/min/max.
//
// If you've worked out good bounds by hand and want to lock them in without
// re-running 'homeall' every boot, add a line here. Applied at boot from
// Startup.cpp. Leave a joint OUT of this list to keep whatever homing
// gives it. The list is empty by default.
//
// NOTE: centerPulseUs must sit between minPulseUs and maxPulseUs, otherwise
// Movement.cpp clamps it to the nearest bound and 'center' will land in the
// same place as 'zero'.
//
// Example:
//   { "P_IP",  1500, 1120, 1890 },
// ===========================================================================
struct ManualBounds {
  const char* name;
  uint16_t centerPulseUs;
  uint16_t minPulseUs;
  uint16_t maxPulseUs;
};

static const ManualBounds MANUAL_BOUNDS[] = {
  /*
  { "P_MP",  1803, 1463, 2144 },
  { "P_P",   1805, 1166, 2444 },
  { "P_IP",  1660, 1514, 1957 },
  { "R_MP",  1758, 1391, 2126 },
  { "R_P",   1628, 1007, 2249 },
  { "R_IP",  1638, 1372, 1904 },
  { "M_MP",  1713, 1202, 2225 },
  { "M_P",   1363, 1600, 1827 },
  { "M_IP",  1725, 1202, 2249 },
  { "I_MP",  1500, 1234, 1766 },
  { "I_P",   1454, 1783, 2126 },
  { "I_IP",  1298,  751, 1845 },
  { "T_CMC", 1955, 1838, 2072 },
  { "T_ABD", 1616, 1223, 2009 },
  { "T_MCP", 2124, 1823, 2426 },
  { "T_IP",  1400, 1528, 1800 },
   */
};
static const uint8_t MANUAL_BOUNDS_COUNT = sizeof(MANUAL_BOUNDS) / sizeof(MANUAL_BOUNDS[0]);

void applyManualBoundsOverride() {
  for (uint8_t i = 0; i < MANUAL_BOUNDS_COUNT; i++) {
    const ManualBounds &m = MANUAL_BOUNDS[i];
    JointId id = findJointByName(m.name);
    if (id == NUM_JOINTS) {
      Serial.print(F("Manual bounds: unknown joint name '"));
      Serial.print(m.name);
      Serial.println(F("' — skipped."));
      continue;
    }
    Joint &j = joints[id];
    j.centerPulseUs = m.centerPulseUs;
    j.minPulseUs = m.minPulseUs;
    j.maxPulseUs = m.maxPulseUs;
    Serial.print(F("Manual bounds applied: "));
    Serial.println(j.name);
  }
}

static bool namesEqualIgnoreCase(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return false;
    a++; b++;
  }
  return *a == *b;
}

JointId findJointByName(const char* name) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (namesEqualIgnoreCase(joints[i].name, name)) return (JointId)i;
  }
  return NUM_JOINTS;
}

JointId findJointByChannel(uint8_t pcaChannel) {
  // Only PCA-driven joints have a meaningful channel number. Without this
  // guard a wrist joint (whose pcaChannel is a placeholder 0) would shadow
  // whichever finger is actually on channel 0.
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].driver != DRV_PCA9685) continue;
    if (joints[i].pcaChannel == pcaChannel) return (JointId)i;
  }
  return NUM_JOINTS;
}

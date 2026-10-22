#include "Movement.h"
#include "Config.h"
#include "HardwareIO.h"
#include "JointMap.h"
#include "Feedback.h"   // measured-vs-commanded reporting at each extreme
#include "Protocol.h"   // controlModeGet() — the ramp defers to PC mode's own limiter
#include "Safety.h"

// Current effective [lo, hi] logical bounds for a joint.
static void effectiveBounds(const Joint &j, uint16_t &lo, uint16_t &hi) {
  lo = j.minPulseUs;
  hi = j.maxPulseUs;
  if (lo > hi) {
    uint16_t t = lo;
    lo = hi;
    hi = t;
  }
}

static uint16_t clamp(uint16_t us, uint16_t lo, uint16_t hi) {
  if (us < lo) return lo;
  if (us > hi) return hi;
  return us;
}

float jointSpeedScale(JointId id) {
  switch (id) {
    case W_1: return WRIST_SPEED_W_1;
    case W_2: return WRIST_SPEED_W_2;
    case W_3: return WRIST_SPEED_W_3;
    default:  return 1.0f;   // fingers and thumb move at whatever the servo does
  }
}

// Walks the output from where it currently is to 'target' at this joint's
// rate limit, rather than stepping there in one go and letting the servo
// slew at full speed.
//
// Blocking, which is fine for terminal commands, gestures and demos — they
// block anyway. Skipped entirely when:
//   * the joint has no limit (scale >= 1.0), or
//   * PC mode is driving, because Protocol.cpp already runs its own slew
//     limiter at 50Hz and a nested blocking ramp would stall that loop.
//
// Homing is unaffected either way: it writes through hwSetJointPulseUs()
// directly and already steps in small increments of its own.
static void rampJointTo(Joint &j, JointId id, uint16_t target) {
  float scale = jointSpeedScale(id);

  if (scale >= 1.0f || controlModeGet() == MODE_PC) {
    hwSetJointPulseUs(j, target);
    j.currentPulseUs = target;
    return;
  }

  int stepUs = (int)((WRIST_FULL_RATE_US_PER_S * scale * WRIST_RAMP_TICK_MS) / 1000.0f);
  if (stepUs < 1) stepUs = 1;

  int from = (int)j.currentPulseUs;
  int to   = (int)target;
  int dir  = (to > from) ? 1 : -1;

  while (((to - from) * dir) > stepUs) {
    from += dir * stepUs;
    hwSetJointPulseUs(j, (uint16_t)from);
    j.currentPulseUs = (uint16_t)from;   // the deviation check reads this
    // A ramp can take a couple of seconds; an E-stop mustn't wait for it.
    if (!safetyWait(WRIST_RAMP_TICK_MS, false)) return;
  }

  hwSetJointPulseUs(j, target);
  j.currentPulseUs = target;
}

void setJointPulseUs(JointId id, uint16_t us) {
  if (id >= NUM_JOINTS) return;
  Joint &j = joints[id];
  if (!j.attached) return;

  uint16_t lo, hi;
  effectiveBounds(j, lo, hi);
  us = clamp(us, lo, hi);

  // Invert by reflecting within [lo, hi] rather than mirroring around
  // center, so the result always stays inside bounds even if center isn't
  // exactly midway between them.
  uint16_t commanded = j.invert ? (uint16_t)(lo + hi - us) : us;

  // Rate-limited for the wrist, immediate for everything else.
  rampJointTo(j, id, commanded);
}

// Drives the motor to the SERVO's own mechanical neutral (SERVO_CENTER_US,
// 1500us = 0 deg on the horn) — this is the assembly reference position,
// NOT the middle of the joint's homed travel.
//
// Deliberately bypasses setJointPulseUs(): no bounds clamping (centering
// has to work before any bounds exist — on a cold boot min/max are 0, and
// clamping would pin the output to 0us), and no invert reflection (1500us
// is the same physical horn position whichever way the joint is mounted).
void centerJoint(JointId id) {
  if (id >= NUM_JOINTS) return;
  Joint &j = joints[id];
  if (!j.attached) return;
  // Rate-limited too: centring from a far position is exactly the move
  // that looks alarming on the wrist.
  rampJointTo(j, id, SERVO_CENTER_US);
}

void centerAllJoints() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].attached) centerJoint((JointId)i);
  }
  Serial.print(F("All attached joints commanded to servo neutral ("));
  Serial.print(SERVO_CENTER_US);
  Serial.println(F("us)."));
}

// ===========================================================================
// ANGLE MAPPING — degrees to logical pulse width.
//
// Two reference conventions, per joint:
//
//   ZERO_AT_MIN_BOUND: 0 deg = fully extended/straight = minPulseUs. The
//   joint only moves one way from there (flexion), reaching 'romDeg' at
//   maxPulseUs. Used by the finger IP/P joints.
//
//   ZERO_AT_CENTER: 0 deg = centerPulseUs (neutral/rest position). Positive
//   angle moves toward maxPulseUs, reaching 'posMaxDeg' there; negative
//   angle moves toward minPulseUs, reaching -'negMaxDeg' there. Used by the
//   abduction (MP) joints, the whole thumb and the wrist. posMaxDeg and
//   negMaxDeg are NOT assumed equal — several of these ranges are
//   deliberately asymmetric (e.g. T_CMC: +82/-37.5), and center isn't
//   assumed to be pulse-width-symmetric either (min/max don't have to be
//   equal distances from center) — each half of the range is interpolated
//   independently between its own two reference points.
//
// This assumes MAX pulse width = flexion/closing/"+", MIN pulse width =
// extension/"-". VERIFY EMPIRICALLY before trusting large commanded
// angles: command a small angle (e.g. 10 deg) on one joint of each type
// and confirm visually it moves the expected way. If a specific joint is
// backwards, flip its 'invert' flag in JointMap.cpp rather than editing
// the table below — invert is applied after this mapping (in
// setJointPulseUs()) so it corrects direction without disturbing the
// angle numbers.
// ===========================================================================
enum AngleZeroMode : uint8_t {
  ZERO_AT_MIN_BOUND,
  ZERO_AT_CENTER,
};

struct JointAngleCal {
  AngleZeroMode mode;
  float romDeg;     // ZERO_AT_MIN_BOUND: angle (deg) at maxPulseUs
  float posMaxDeg;  // ZERO_AT_CENTER: angle (deg) at maxPulseUs
  float negMaxDeg;  // ZERO_AT_CENTER: angle magnitude (deg) at minPulseUs
};

// Indexed by JointId — MUST stay in JointTypes.h enum order.
// All ROM figures come from Config.h; edit them there, not here.
#define P_CAL   { ZERO_AT_MIN_BOUND, ROM_FINGER_P_DEG,  0.0f, 0.0f }
#define IP_CAL  { ZERO_AT_MIN_BOUND, ROM_FINGER_IP_DEG, 0.0f, 0.0f }

static const JointAngleCal JOINT_ANGLE_CAL[NUM_JOINTS] = {
  /* P_MP  */ { ZERO_AT_CENTER, 0.0f, ROM_P_MP_POS_DEG, ROM_P_MP_NEG_DEG },
  /* P_P   */ P_CAL,
  /* P_IP  */ IP_CAL,

  /* R_MP  */ { ZERO_AT_CENTER, 0.0f, ROM_R_MP_POS_DEG, ROM_R_MP_NEG_DEG },
  /* R_P   */ P_CAL,
  /* R_IP  */ IP_CAL,

  /* M_MP  */ { ZERO_AT_CENTER, 0.0f, ROM_M_MP_POS_DEG, ROM_M_MP_NEG_DEG },
  /* M_P   */ P_CAL,
  /* M_IP  */ IP_CAL,

  /* I_MP  */ { ZERO_AT_CENTER, 0.0f, ROM_I_MP_POS_DEG, ROM_I_MP_NEG_DEG },
  /* I_P   */ P_CAL,
  /* I_IP  */ IP_CAL,

  /* T_CMC */ { ZERO_AT_CENTER,    0.0f, ROM_T_CMC_POS_DEG, ROM_T_CMC_NEG_DEG },
  /* T_ABD */ { ZERO_AT_CENTER,    0.0f, ROM_T_ABD_POS_DEG, ROM_T_ABD_NEG_DEG },
  /* T_MCP */ { ZERO_AT_CENTER,    0.0f, ROM_T_MCP_POS_DEG, ROM_T_MCP_NEG_DEG },
  /* T_IP  */ { ZERO_AT_CENTER,    0.0f, ROM_T_IP_POS_DEG,  ROM_T_IP_NEG_DEG  },

  // Wrist — all centre-referenced with symmetric travel.
  /* W_1   */ { ZERO_AT_CENTER,    0.0f, ROM_WRIST_1_DEG, ROM_WRIST_1_DEG },
  /* W_2   */ { ZERO_AT_CENTER,    0.0f, ROM_WRIST_2_DEG, ROM_WRIST_2_DEG },
  /* W_3   */ { ZERO_AT_CENTER,    0.0f, ROM_WRIST_3_DEG, ROM_WRIST_3_DEG },
};

#undef P_CAL
#undef IP_CAL

bool jointZeroIsCenter(JointId id) {
  if (id >= NUM_JOINTS) return false;
  return JOINT_ANGLE_CAL[id].mode == ZERO_AT_CENTER;
}

void jointAngleLimits(JointId id, float &minDeg, float &maxDeg) {
  if (id >= NUM_JOINTS) { minDeg = 0.0f; maxDeg = 0.0f; return; }
  const JointAngleCal &cal = JOINT_ANGLE_CAL[id];
  if (cal.mode == ZERO_AT_MIN_BOUND) {
    minDeg = 0.0f;
    maxDeg = cal.romDeg;
  } else {
    minDeg = -cal.negMaxDeg;
    maxDeg = cal.posMaxDeg;
  }
}

// Angle -> LOGICAL pulse width (pre-invert, pre-clamp). Split out so the
// ramping and non-ramping paths cannot drift apart.
static uint16_t angleToLogicalPulseUs(JointId id, float angleDeg) {
  const Joint &j = joints[id];
  const JointAngleCal &cal = JOINT_ANGLE_CAL[id];
  float pulseUs;

  if (cal.mode == ZERO_AT_MIN_BOUND) {
    float frac = (cal.romDeg > 0.0f) ? (angleDeg / cal.romDeg) : 0.0f;
    pulseUs = (float)j.minPulseUs + frac * (float)((int32_t)j.maxPulseUs - (int32_t)j.minPulseUs);
  } else {
    if (angleDeg >= 0.0f) {
      float frac = (cal.posMaxDeg > 0.0f) ? (angleDeg / cal.posMaxDeg) : 0.0f;
      pulseUs = (float)j.centerPulseUs + frac * (float)((int32_t)j.maxPulseUs - (int32_t)j.centerPulseUs);
    } else {
      float frac = (cal.negMaxDeg > 0.0f) ? ((-angleDeg) / cal.negMaxDeg) : 0.0f;
      pulseUs = (float)j.centerPulseUs - frac * (float)((int32_t)j.centerPulseUs - (int32_t)j.minPulseUs);
    }
  }
  if (pulseUs < 0.0f) pulseUs = 0.0f;
  return (uint16_t)(pulseUs + 0.5f);
}

// The inverse of angleToLogicalPulseUs(): the angle the joint is being
// commanded to RIGHT NOW, worked out from the pulse last written to it. Works
// whichever path wrote that pulse (PC slew limiter, gesture, demo, terminal),
// which is what the deviation check needs.
float jointCommandedAngleDeg(JointId id) {
  if (id >= NUM_JOINTS) return 0.0f;
  const Joint &j = joints[id];
  const JointAngleCal &cal = JOINT_ANGLE_CAL[id];
  uint16_t lo, hi;
  effectiveBounds(j, lo, hi);
  float us = j.invert ? (float)((int32_t)lo + (int32_t)hi - (int32_t)j.currentPulseUs)
                      : (float)j.currentPulseUs;

  if (cal.mode == ZERO_AT_MIN_BOUND) {
    float span = (float)((int32_t)j.maxPulseUs - (int32_t)j.minPulseUs);
    return (span != 0.0f) ? (us - (float)j.minPulseUs) / span * cal.romDeg : 0.0f;
  }
  if (us >= (float)j.centerPulseUs) {
    float span = (float)((int32_t)j.maxPulseUs - (int32_t)j.centerPulseUs);
    return (span != 0.0f) ? (us - (float)j.centerPulseUs) / span * cal.posMaxDeg : 0.0f;
  }
  float span = (float)((int32_t)j.centerPulseUs - (int32_t)j.minPulseUs);
  return (span != 0.0f) ? -((float)j.centerPulseUs - us) / span * cal.negMaxDeg : 0.0f;
}

void setJointAngleDeg(JointId id, float angleDeg) {
  if (id >= NUM_JOINTS) return;
  if (!joints[id].attached) return;

  // setJointPulseUs() clamps to this joint's current min/max bounds and
  // applies invert, same as every other movement function in this file.
  setJointPulseUs(id, angleToLogicalPulseUs(id, angleDeg));
}

// Same destination as setJointAngleDeg(), but written in one go with no rate
// limiting, and currentPulseUs synced to match.
//
// Needed at boot. The rate limiter walks from currentPulseUs, which before
// anything has been commanded is only an assumption (SERVO_CENTER_US) rather
// than where the joint really is — so ramping would drive the joint to that
// assumed position first, which is exactly the jump this avoids.
void setJointAngleDegImmediate(JointId id, float angleDeg) {
  if (id >= NUM_JOINTS) return;
  Joint &j = joints[id];
  if (!j.attached) return;

  uint16_t lo, hi;
  effectiveBounds(j, lo, hi);
  uint16_t us = clamp(angleToLogicalPulseUs(id, angleDeg), lo, hi);
  uint16_t commanded = j.invert ? (uint16_t)(lo + hi - us) : us;

  hwSetJointPulseUs(j, commanded);
  j.currentPulseUs = commanded;
}

void zeroAllJoints() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (joints[i].attached) setJointAngleDeg((JointId)i, 0.0f);
  }
  Serial.println(F("All attached joints commanded to their 0 deg (anatomical zero) position."));
}

void sweepJoint(JointId id) {
  if (id >= NUM_JOINTS) return;
  Joint &j = joints[id];
  if (!j.attached) {
    Serial.println(F("Joint not marked attached, skipping."));
    return;
  }
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }

  // A small fixed excursion either side of neutral, not the full travel.
  // This is the post-assembly "which motor is this?" sweep: it runs before
  // homing, when min/max are still 0/0, and a full-ROM swing on an
  // abduction joint would drive it straight into its mechanical stop.
  const bool homed = !(j.minPulseUs == 0 && j.maxPulseUs == 0);

  int centre = homed ? (int)j.centerPulseUs : SERVO_CENTER_US;
  int lo = centre - SWEEP_RANGE_US;
  int hi = centre + SWEEP_RANGE_US;

  // Stay inside whatever envelope we actually know about: the joint's own
  // bounds once homed, otherwise the global safety limits.
  int envLo = homed ? (int)j.minPulseUs : SERVO_MIN_US;
  int envHi = homed ? (int)j.maxPulseUs : SERVO_MAX_US;
  if (lo < envLo) lo = envLo;
  if (hi > envHi) hi = envHi;

  Serial.print(F("\nSweeping "));
  Serial.print(j.name);
  Serial.print(F(": "));
  Serial.print(lo);
  Serial.print(F("us -> "));
  Serial.print(hi);
  Serial.print(F("us -> "));
  Serial.print(centre);
  Serial.println(homed ? F("us") : F("us  (not homed — sweeping around 1500us)"));

  // Raw writes rather than setJointPulseUs(): that clamps to the joint's
  // bounds, which are 0/0 before homing and would pin the output to 0us.
  hwSetJointPulseUs(j, (uint16_t)lo);
  delay(MOVE_SETTLE_MS);
  hwSetJointPulseUs(j, (uint16_t)hi);
  delay(MOVE_SETTLE_MS);
  hwSetJointPulseUs(j, (uint16_t)centre);
  delay(MOVE_SETTLE_MS);
  j.currentPulseUs = (uint16_t)centre;

  Serial.println(F("Sweep done."));
}

// ===========================================================================
// FULL RANGE-OF-MOTION SWEEP ('sweep_r')
//
// Walks a joint smoothly through its entire commandable angular range and
// back, reporting commanded against measured at each extreme.
//
// Distinct from sweepJoint(), which nudges +/-SWEEP_RANGE_US around neutral
// purely to identify which motor drives which joint. This one exercises the
// REAL homed travel, so it needs bounds — and because it drives by angle
// rather than pulse width it also exercises the angle mapping, which makes
// it the command for checking that a joint's ROM figures and centre
// placement are actually right.
// ===========================================================================

static bool sweepRangeGlide(JointId id, float fromDeg, float toDeg) {
  for (uint16_t step = 1; step <= SWEEP_R_STEPS; step++) {
    float frac = (float)step / (float)SWEEP_R_STEPS;
    setJointAngleDeg(id, fromDeg + (toDeg - fromDeg) * frac);
    if (!safetyWait(SWEEP_R_STEP_MS, true)) return false;   // any key / E-stop
  }
  return true;
}

static void sweepRangeReport(JointId id, float commandedDeg) {
  Serial.print(F("    commanded "));
  Serial.print(commandedDeg, 1);
  Serial.print(F(" deg"));
  if (jointFeedbackTrustworthy(id)) {
    float measured = readJointAngleDeg(id, FEEDBACK_SAMPLES);
    Serial.print(F("   measured "));
    Serial.print(measured, 1);
    Serial.print(F(" deg   error "));
    Serial.print(measured - commandedDeg, 1);
    Serial.println(F(" deg"));
  } else {
    Serial.println(F("   (no usable feedback on this joint)"));
  }
}

void sweepJointRange(JointId id) {
  if (id >= NUM_JOINTS) return;
  Joint &j = joints[id];

  if (!j.attached) {
    Serial.println(F("Joint not marked attached, skipping."));
    return;
  }
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }
  if (j.minPulseUs == 0 && j.maxPulseUs == 0) {
    Serial.print(F("  "));
    Serial.print(j.name);
    Serial.println(F(": no bounds yet — home it first, or use 'sweep' to identify it."));
    return;
  }

  float loDeg, hiDeg;
  jointAngleLimits(id, loDeg, hiDeg);

  Serial.print(F("\nFull-range sweep of "));
  Serial.print(j.name);
  Serial.print(F(":  "));
  Serial.print(loDeg, 1);
  Serial.print(F(" to +"));
  Serial.print(hiDeg, 1);
  Serial.print(F(" deg   ("));
  Serial.print(j.minPulseUs);
  Serial.print(F(" - "));
  Serial.print(j.centerPulseUs);
  Serial.print(F(" - "));
  Serial.print(j.maxPulseUs);
  Serial.println(F("us)"));
  Serial.println(F("Send any character to abort."));

  SafetyPauseGuard pause;   // the extremes are where a joint may legitimately lag

  // Start from zero so the sweep is symmetric about a known reference.
  setJointAngleDeg(id, 0.0f);

  if (safetyWait(SWEEP_R_HOLD_MS, true) && sweepRangeGlide(id, 0.0f, loDeg)) {
    Serial.println(F("  minimum:"));
    sweepRangeReport(id, loDeg);

    if (safetyWait(SWEEP_R_HOLD_MS, true) && sweepRangeGlide(id, loDeg, hiDeg)) {
      Serial.println(F("  maximum:"));
      sweepRangeReport(id, hiDeg);

      if (safetyWait(SWEEP_R_HOLD_MS, true) && sweepRangeGlide(id, hiDeg, 0.0f)) {
        Serial.println(F("  back at zero:"));
        sweepRangeReport(id, 0.0f);
        Serial.println(F("Sweep done."));
        return;
      }
    }
  }

  // Any abort lands here. A user abort puts the joint somewhere sane; an
  // E-stop leaves everything off.
  if (safetyEstopActive()) {
    Serial.println(F("\n  E-stop — sweep stopped."));
    return;
  }
  Serial.println(F("\n  Aborted — returning to zero."));
  setJointAngleDeg(id, 0.0f);
}

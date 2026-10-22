#include "Homing.h"
#include "Config.h"
#include "HardwareIO.h"
#include "JointMap.h"
#include "Movement.h"
#include "CurrentMonitor.h"
#include "Calibration.h"
#include "Diagnostics.h"   // serialWaitForLine()
#include "Safety.h"

// Each step of a homing sequence is followed by an E-stop check. Carrying on
// would record bounds for joints that never moved and save them to flash.
#define HOME_STEP(call)                                                    \
  do {                                                                     \
    call;                                                                  \
    if (safetyEstopActive()) {                                             \
      Serial.println(F("\n  E-stop — homing abandoned, nothing saved.")); \
      return;                                                              \
    }                                                                      \
  } while (0)

// Total travel span for a joint, used to derive the opposite bound from the
// one actually found. Keyed directly off the JointId rather than a separate
// "joint type" code, so the automatic and manual sequences cannot disagree
// and adding a per-finger figure is a one-line change.
static int homingOffsetFor(JointId id) {
  switch (id) {
    case I_MP: return HOMING_OFFSET_I_MP;      // abduction is per-finger now
    case M_MP: return HOMING_OFFSET_M_MP;
    case R_MP: return HOMING_OFFSET_R_MP;
    case P_MP: return HOMING_OFFSET_P_MP;
    case P_IP: case R_IP: case M_IP: case I_IP: return HOMING_OFFSET_FINGER_IP;
    case P_P:  case R_P:  case M_P:  case I_P:  return HOMING_OFFSET_FINGER_P;
    case T_MCP: return HOMING_OFFSET_T_MCP;
    case T_CMC: return HOMING_OFFSET_T_CMC;
    case T_ABD: return HOMING_OFFSET_T_ABD;
    case T_IP:  return HOMING_OFFSET_T_IP;
    case W_1:   return HOMING_OFFSET_W_1;
    case W_2:   return HOMING_OFFSET_W_2;
    case W_3:   return HOMING_OFFSET_W_3;
    default:    return 0;
  }
}

// Records the found bound and derives the opposite one. 'direction' here is
// the POST-invert direction actually travelled.
//
// The centre is placed at the joint's TRUE 0 deg position, which is only the
// midpoint of travel when the range happens to be symmetric. setJointAngleDeg()
// interpolates each half of the range independently between centre and its
// own bound, so a centre sitting at the midpoint of an ASYMMETRIC joint gives
// the two halves different degrees-per-microsecond and every commanded angle
// in between comes out wrong.
static void recordBounds(JointId id, int foundUs, bool direction) {
  Joint &j = joints[id];
  int offsetUS = homingOffsetFor(id);

  // Derive both bounds in int. The previous version assigned
  // (uint16_t)(foundUs - offsetUS) straight into the field, so a negative
  // result wrapped to ~65000 instead of being caught. Only a joint whose
  // offset exceeds the pulse width homing actually found can reach that, and
  // since manual homing starts every joint at 1500us, that means only an
  // offset above ~1500 — which is why T_IP alone misbehaved while T_CMC,
  // T_MCP and the abduction joints (all <= 1000) were fine.
  int lo, hi;
  if (direction) { hi = foundUs; lo = foundUs - offsetUS; }
  else           { lo = foundUs; hi = foundUs + offsetUS; }

  // A derived bound outside the safety envelope means this joint's
  // HOMING_OFFSET_* is larger than the travel the mechanism really has, or
  // the stop was found in the wrong place. Clamp so nothing downstream ever
  // sees a nonsense pulse width, and say so loudly — the angle mapping will
  // be compressed until the offset is corrected.
  bool clamped = false;
  if (lo < SERVO_MIN_US) { lo = SERVO_MIN_US; clamped = true; }
  if (hi > SERVO_MAX_US) { hi = SERVO_MAX_US; clamped = true; }

  j.minPulseUs = (uint16_t)lo;
  j.maxPulseUs = (uint16_t)hi;

  // Split the span in the same ratio as the joint's negative and positive
  // travel. Joints whose zero is the min bound never consult centre when
  // mapping angles, so the midpoint is harmless for them.
  int span = hi - lo;
  int fromMin = span / 2;
  if (jointZeroIsCenter(id)) {
    float loDeg, hiDeg;
    jointAngleLimits(id, loDeg, hiDeg);
    float neg = -loDeg, pos = hiDeg;
    if (neg + pos > 0.0f) fromMin = (int)((float)span * neg / (neg + pos) + 0.5f);
  }
  j.centerPulseUs = (uint16_t)(lo + fromMin);

  if (clamped) {
    Serial.print(F("  !! "));
    Serial.print(j.name);
    Serial.print(F(": offset "));
    Serial.print(offsetUS);
    Serial.print(F("us does not fit from "));
    Serial.print(foundUs);
    Serial.print(F("us — travel clamped to "));
    Serial.print(span);
    Serial.println(F("us."));
    Serial.println(F("     Angles for this joint will be wrong until its"));
    Serial.println(F("     HOMING_OFFSET_* is corrected in Config.h."));
  }

  Serial.print(F("  "));
  Serial.print(j.name);
  Serial.print(F(" bounds: min="));
  Serial.print(j.minPulseUs);
  Serial.print(F("us center="));
  Serial.print(j.centerPulseUs);
  Serial.print(F("us max="));
  Serial.print(j.maxPulseUs);
  Serial.println(F("us"));
}

// Sends one joint to its anatomical zero.
//
// Goes through setJointAngleDeg() rather than writing centerPulseUs raw.
// For an INVERTED joint the stored centre is a LOGICAL pulse width and only
// becomes the correct physical position once setJointPulseUs() applies the
// invert reflection — a raw write would land it on the mirror image of its
// true zero. That was harmless while every centre-referenced joint had
// symmetric travel (mirror of the midpoint is the midpoint), but the finger
// abduction joints are no longer symmetric.
//
// Skips motors switched off with 'off', and joints with no bounds yet.
static void zeroOne(JointId id) {
  Joint &j = joints[id];
  if (!j.attached) return;
  if (j.minPulseUs == 0 && j.maxPulseUs == 0) return;
  setJointAngleDeg(id, 0.0f);
}

// Sends one joint to zero and waits for it to physically arrive. A commanded
// move returns immediately, so without the wait the next joint could start
// homing while this one is still travelling — and it would then stall against
// a neighbour that hasn't finished getting out of the way, recording a false
// bound. Automatic homing needs this most, since nothing else paces it.
static void zeroOneAndSettle(JointId id) {
  zeroOne(id);
  delay(MOVE_SETTLE_MS);
}

static void parkAtZero(JointId a, JointId b, JointId c, JointId d) {
  zeroOne(a);
  zeroOne(b);
  zeroOne(c);
  zeroOne(d);
  delay(MOVE_SETTLE_MS);   // all four travel in parallel, so one wait covers them
}

// ===========================================================================
// AUTOMATIC HOMING — drives until a stall current is seen.
// ===========================================================================
void homeAllJoints() {
  SafetyPauseGuard pause;   // stalling into the stops is the whole point
  if (safetyEstopActive()) return;
  centerAllJoints();

  // ***************** Home abduction, index to pinky ******************
  HOME_STEP(StallHome(I_MP, 1));
  HOME_STEP(StallHome(M_MP, 1));
  HOME_STEP(StallHome(R_MP, 1));
  HOME_STEP(StallHome(P_MP, 1));
  HOME_STEP(parkAtZero(P_MP, R_MP, M_MP, I_MP));

  // ***************** Home distal/intermediate phalanges ******************
  HOME_STEP(StallHome(P_IP, 1));
  HOME_STEP(StallHome(R_IP, 1));
  HOME_STEP(StallHome(M_IP, 1));
  HOME_STEP(StallHome(I_IP, 1));
  HOME_STEP(parkAtZero(P_IP, R_IP, M_IP, I_IP));

  // ***************** Home proximal phalanges ******************
  HOME_STEP(StallHome(P_P, 1));
  HOME_STEP(StallHome(R_P, 1));
  HOME_STEP(StallHome(M_P, 1));
  HOME_STEP(StallHome(I_P, 1));
  HOME_STEP(parkAtZero(P_P, R_P, M_P, I_P));

  // ***************** Home thumb ******************
  // Each joint is cleared to zero before the next one homes. Left at its
  // homed limit a thumb joint fouls the travel of the one that follows, and
  // stall detection cannot tell "hit my own stop" from "hit my neighbour" —
  // it would simply record the false bound and move on. Mirrors the same
  // sequence in manualHomeAllJoints(). T_IP is last, so nothing follows it.
  HOME_STEP(StallHome(T_MCP, 1));
  HOME_STEP(zeroOneAndSettle(T_MCP));

  HOME_STEP(StallHome(T_CMC, 1));
  HOME_STEP(zeroOneAndSettle(T_CMC));

  HOME_STEP(StallHome(T_ABD, 1));
  HOME_STEP(zeroOneAndSettle(T_ABD));

  HOME_STEP(StallHome(T_IP, 1));

  // The wrist motors are not on the Servo Click and have no per-motor
  // current sensing, so stall detection can't see them. Use 'manualhome'
  // for the wrist — it keeps whatever bounds it already had here.
  Serial.println(F("\n  Wrist (W_1/W_2/W_3) skipped — no current sensing."));
  Serial.println(F("  Use 'manualhome' or 'manualhome W_1' to set wrist bounds."));

  calibrationSaveAll(); // persist the bounds we just found, so they survive a reboot
  centerAllJoints();
  return;
}

void StallHome(JointId motorID, bool direction) {
  Joint &j = joints[motorID];
  if (safetyEstopActive()) return;

  if (!j.attached) {
    Serial.print(F("  "));
    Serial.print(j.name);
    Serial.println(F(": motor is OFF — skipped."));
    return;
  }

  if (j.invert) {
    direction = !direction; // invert direction if joint is mechanically inverted
  }

  // Always start from the servo's mechanical neutral — homeAllJoints()
  // calls centerAllJoints() first, which physically parks every motor at
  // SERVO_CENTER_US regardless of invert, so the sweep has to start
  // counting from the same place or the first write jumps the joint.
  int us = SERVO_CENTER_US;
  int step = direction ? HOMING_STEP_US : -HOMING_STEP_US; // direction 1 = forwards/tensor, 0 = backwards/extensor

  j.currentMA = 0.0f; // clear any stale value so the while-loop below can't exit immediately
  unsigned long lastStepMs = millis();
  bool slowed = false;

  // Current is polled on just this joint's channel, so it refreshes every
  // ~LTC2497_CONV_MS (~160ms) rather than waiting on a full 16-joint
  // rotation. Safe because homeAllJoints() is a single blocking call —
  // nothing else touches the LTC2497 while it runs.
  while (j.currentMA < HOMING_STALL_CURRENT_MA) {
    // With the outputs cut the stall never comes — leave without a bound.
    if (safetyRoutinePoll(false)) return;
    currentMonitorSingleStep(motorID);

    if (millis() - lastStepMs >= HOMING_STEP_DELAY_MS) {
      lastStepMs = millis();
      us += step;

      if ((us < SERVO_MIN_US || us > SERVO_MAX_US) && !slowed) {
        Serial.print(F("  "));
        Serial.print(j.name);
        Serial.println(F(": hit safety travel limit without a stall reading — slowing down."));
        step = direction ? HOMING_SLOW_STEP_US : -HOMING_SLOW_STEP_US; // reduce step size to home slower
        slowed = true;
      }
      hwSetJointPulseUs(j, (uint16_t)us);
    }
  }

  recordBounds(motorID, us, direction);
  return;
}

// ===========================================================================
// MANUAL HOMING — user jogs each joint and says when it's at the limit.
// ===========================================================================

// Jogs one joint from SERVO_CENTER_US in its homing direction until the
// user types 'd'. Mirrors StallHome()'s direction/invert handling and uses
// the same offset to derive the second bound.
static void manualHomeJoint(JointId motorID, bool direction) {
  Joint &j = joints[motorID];

  if (!j.attached) {
    Serial.print(F("  "));
    Serial.print(j.name);
    Serial.println(F(": motor is OFF — skipped."));
    return;
  }

  if (j.invert) {
    direction = !direction; // same invert handling as StallHome()
  }

  // A positive step from the user always means "further in the homing
  // direction", which for an inverted joint is a DECREASE in raw us.
  int sign = direction ? 1 : -1;

  int us = SERVO_CENTER_US;
  hwSetJointPulseUs(j, (uint16_t)us);

  Serial.print(F("\n--- Manual home: "));
  Serial.print(j.name);
  Serial.print(F("  (homing direction: "));
  Serial.print(direction ? F("+us") : F("-us"));
  Serial.println(F(") ---"));
  Serial.println(F("  Type a step in us to move further (e.g. 10), a negative"));
  Serial.println(F("  value to back off, or 'd' when the joint is at its limit."));

  while (true) {
    Serial.print(F("  "));
    Serial.print(j.name);
    Serial.print(F(" @ "));
    Serial.print(us);
    Serial.print(F("us > "));

    String line = serialWaitForLine();
    if (safetyEstopActive()) return;          // no bound recorded
    if (line.length() == 0) continue;
    if (line.equalsIgnoreCase("d")) break;

    int stepUs = line.toInt();
    if (stepUs == 0) {
      Serial.println(F("  Enter a step size in us (e.g. 10 or -10), or 'd' to finish."));
      continue;
    }

    us += sign * stepUs;

    // Hard safety envelope — never command outside it, however much the
    // user asks for.
    if (us < SERVO_MIN_US) {
      us = SERVO_MIN_US;
      Serial.println(F("  (clamped to SERVO_MIN_US)"));
    }
    if (us > SERVO_MAX_US) {
      us = SERVO_MAX_US;
      Serial.println(F("  (clamped to SERVO_MAX_US)"));
    }

    hwSetJointPulseUs(j, (uint16_t)us);
  }

  recordBounds(motorID, us, direction);
}

void manualHomeAllJoints() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }

  SafetyPauseGuard pause;
  if (safetyEstopActive()) return;

  // Drop anything already buffered so a stray character doesn't get eaten
  // as the first joint's step value.
  safetyDrainSerial();
  if (safetyEstopActive()) return;

  Serial.println(F("\n===== MANUAL HOMING ====="));
  Serial.println(F("Same joint order and direction as 'homeall'. Each joint starts"));
  Serial.println(F("at servo neutral; jog it to its mechanical limit, then type 'd'."));

  centerAllJoints();

  // ***************** Home abduction, index to pinky ******************
  HOME_STEP(manualHomeJoint(I_MP, 1));
  HOME_STEP(manualHomeJoint(M_MP, 1));
  HOME_STEP(manualHomeJoint(R_MP, 1));
  HOME_STEP(manualHomeJoint(P_MP, 1));
  HOME_STEP(parkAtZero(P_MP, R_MP, M_MP, I_MP));

  // ***************** Home distal/intermediate phalanges ******************
  HOME_STEP(manualHomeJoint(P_IP, 1));
  HOME_STEP(manualHomeJoint(R_IP, 1));
  HOME_STEP(manualHomeJoint(M_IP, 1));
  HOME_STEP(manualHomeJoint(I_IP, 1));
  HOME_STEP(parkAtZero(P_IP, R_IP, M_IP, I_IP));

  // ***************** Home proximal phalanges ******************
  HOME_STEP(manualHomeJoint(P_P, 1));
  HOME_STEP(manualHomeJoint(R_P, 1));
  HOME_STEP(manualHomeJoint(M_P, 1));
  HOME_STEP(manualHomeJoint(I_P, 1));
  HOME_STEP(parkAtZero(P_P, R_P, M_P, I_P));

  // ***************** Home thumb ******************
  // Each of these is sent to zero before the next one starts. Left sitting
  // at its homed limit a thumb joint fouls the travel of the one that
  // follows, so the operator would be jogging the next joint into its
  // neighbour rather than its own mechanical stop. T_IP is last and has
  // nothing following it, so it needs no clearing move.
  HOME_STEP(manualHomeJoint(T_MCP, 1));
  HOME_STEP(zeroOneAndSettle(T_MCP));

  HOME_STEP(manualHomeJoint(T_CMC, 1));
  HOME_STEP(zeroOneAndSettle(T_CMC));

  HOME_STEP(manualHomeJoint(T_ABD, 1));
  HOME_STEP(zeroOneAndSettle(T_ABD));

  HOME_STEP(manualHomeJoint(T_IP, 1));

  // ***************** Home wrist ******************
  // The wrist has no per-motor current sensing, so it can only be homed
  // by hand — 'homeall' skips it entirely.
  HOME_STEP(manualHomeJoint(W_1, 1));
  HOME_STEP(manualHomeJoint(W_2, 1));
  HOME_STEP(manualHomeJoint(W_3, 1));

  calibrationSaveAll();
  centerAllJoints();
  Serial.println(F("===== MANUAL HOMING COMPLETE =====\n"));
  return;
}

void manualHomeSingleJoint(JointId id) {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }
  if (id >= NUM_JOINTS) return;

  Joint &j = joints[id];
  if (!j.attached) {
    Serial.print(F("  "));
    Serial.print(j.name);
    Serial.println(F(": motor is OFF — 'on' it first."));
    return;
  }

  SafetyPauseGuard pause;
  if (safetyEstopActive()) return;

  // Drop anything already buffered so a stray character isn't eaten as the
  // first step value.
  safetyDrainSerial();
  if (safetyEstopActive()) return;

  Serial.print(F("\n===== RE-HOME "));
  Serial.print(j.name);
  Serial.println(F(" ====="));

  manualHomeJoint(id, 1);
  if (safetyEstopActive()) {
    Serial.println(F("\n  E-stop — re-home abandoned, nothing saved."));
    return;
  }

  calibrationSaveJoint((uint8_t)id);
  zeroOne(id);

  Serial.println(F("Saved to flash.\n"));
}

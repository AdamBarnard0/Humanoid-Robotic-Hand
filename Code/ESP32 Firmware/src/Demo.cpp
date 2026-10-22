#include "Demo.h"
#include "Config.h"
#include "JointMap.h"
#include "Movement.h"
#include "HardwareIO.h"
#include "Feedback.h"
#include "Safety.h"
#include <math.h>

static const float TWO_PI_F = 6.28318531f;

// Abduction joints, index side -> pinky side.
static const JointId ABD_JOINTS[4] = { I_MP, M_MP, R_MP, P_MP };
// Finger flexion joints, same finger order.
static const JointId P_JOINTS[4]   = { I_P,  M_P,  R_P,  P_P  };
static const JointId IP_JOINTS[4]  = { I_IP, M_IP, R_IP, P_IP };

static bool hasBounds(const Joint &j) {
  return !(j.minPulseUs == 0 && j.maxPulseUs == 0);
}

// Commands a joint only if it's usable, so a joint that's switched off or
// never homed simply sits out rather than being driven to 0us.
static void demoSet(JointId id, float deg) {
  Joint &j = joints[id];
  if (!j.attached || !hasBounds(j)) return;
  setJointAngleDeg(id, deg);
}

static void setGroup(const JointId *group, float deg) {
  for (uint8_t i = 0; i < 4; i++) demoSet(group[i], deg);
}

// Splay angle for one finger at 'frac' of its available travel, where -1 is
// that finger's negative limit and +1 its positive one. Each direction is
// scaled against its own limit, since none of these joints is symmetric.
static float abdAngleFor(JointId id, float frac) {
  float loDeg, hiDeg;
  jointAngleLimits(id, loDeg, hiDeg);
  float reach = (frac >= 0.0f) ? hiDeg : -loDeg;   // magnitude available that way
  return frac * reach * DEMO_ABD_FRACTION;
}

// Drives all four abduction joints to the same fraction of their own travel,
// so they arrive at their limits together despite the ranges differing.
static void setAbdFraction(float frac) {
  for (uint8_t i = 0; i < 4; i++) {
    demoSet(ABD_JOINTS[i], abdAngleFor(ABD_JOINTS[i], frac));
  }
}

// Swings the thumb away from the palm so the fingers can close, or so the
// index can point, without fouling it.
static void setThumbClear() {
  demoSet(T_CMC, DEMO_THUMB_CLEAR_CMC);
  demoSet(T_ABD, DEMO_THUMB_CLEAR_ABD);
  demoSet(T_MCP, DEMO_THUMB_CLEAR_MCP);
  demoSet(T_IP,  DEMO_THUMB_CLEAR_IP);
}

// Waits, but bails out early if the user sends anything or an E-stop
// arrives. Protection keeps running throughout. Returns false if the demo
// should stop.
static bool demoPause(uint16_t ms) {
  return safetyWait(ms, true);
}

// After an abort: a user abort returns the hand to zero; an E-stop leaves
// every output off.
static void demoAbortToZero() {
  if (safetyEstopActive()) {
    Serial.println(F("\n  E-stop — demo stopped."));
    return;
  }
  Serial.println(F("\n  Demo aborted — returning to zero."));
  zeroAllJoints();
}

#define DEMO_WAIT(ms)                                   \
  if (!demoPause(ms)) {                                 \
    demoAbortToZero();                                  \
    return;                                             \
  }

static void demoStep(const char* label) {
  Serial.print(F("  "));
  Serial.println(label);
}

void runMovementDemo() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }

  // Warn about anything that will sit the demo out, but don't refuse —
  // a partly-built or partly-faulty hand should still be demonstrable.
  uint8_t unusable = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (!joints[i].attached || !hasBounds(joints[i])) unusable++;
  }
  if (unusable > 0) {
    Serial.print(F("  NOTE: "));
    Serial.print(unusable);
    Serial.println(F(" joint(s) are off or unhomed and will not take part."));
  }

  Serial.println(F("\n===== MOVEMENT DEMO ====="));
  Serial.println(F("Send any character to abort.\n"));

  zeroAllJoints();
  DEMO_WAIT(DEMO_HOLD_MS);

  // --- 1. all abduction, together -------------------------------------
  demoStep("1/5  abduction, all together");
  for (uint8_t rep = 0; rep < 2; rep++) {
    setAbdFraction(+1.0f);
    DEMO_WAIT(DEMO_STEP_MS);
    setAbdFraction(-1.0f);
    DEMO_WAIT(DEMO_STEP_MS);
  }
  setAbdFraction(0.0f);
  DEMO_WAIT(DEMO_STEP_MS);

  // --- 2. abduction wave, finger by finger ----------------------------
  // A ripple rather than one-finger-at-a-time: each finger swings across
  // and STAYS there while the next follows, so the motion travels down the
  // hand. The wave then reverses — index->pinky going right, then
  // pinky->index going left, alternating.
  demoStep("2/5  abduction wave, rippling across the hand");
  {
    float frac = +1.0f;
    for (uint8_t pass = 0; pass < DEMO_ABD_WAVE_PASSES; pass++) {
      bool indexFirst = (pass % 2 == 0);
      for (uint8_t k = 0; k < 4; k++) {
        JointId id = ABD_JOINTS[indexFirst ? k : (3 - k)];
        demoSet(id, abdAngleFor(id, frac));
        DEMO_WAIT(DEMO_WAVE_MS);
      }
      frac = -frac;
    }
    // Settle back to neutral, continuing the same alternation.
    bool indexFirst = (DEMO_ABD_WAVE_PASSES % 2 == 0);
    for (uint8_t k = 0; k < 4; k++) {
      demoSet(ABD_JOINTS[indexFirst ? k : (3 - k)], 0.0f);
      DEMO_WAIT(DEMO_WAVE_MS);
    }
  }

  zeroAllJoints();
  DEMO_WAIT(DEMO_HOLD_MS);

  // --- 3. P/IP wave, twice ---------------------------------------------
  // The bend passes back and forth along the fingers: proximals bend, then
  // hand off to the IP joints, then back to the proximals, then flat. Each
  // hand-off is a single move with both groups changing together, which is
  // what makes it read as one travelling bend rather than two separate
  // curls.
  demoStep("3/5  P/IP wave");
  for (uint8_t rep = 0; rep < DEMO_PIP_REPS; rep++) {
    // all P curl
    setGroup(P_JOINTS,  DEMO_CURL_P_DEG);
    setGroup(IP_JOINTS, 0.0f);
    DEMO_WAIT(DEMO_STEP_MS);

    // P straightens as IP curls
    setGroup(P_JOINTS,  0.0f);
    setGroup(IP_JOINTS, DEMO_CURL_IP_DEG);
    DEMO_WAIT(DEMO_STEP_MS);

    // IP extends as P curls again
    setGroup(IP_JOINTS, 0.0f);
    setGroup(P_JOINTS,  DEMO_CURL_P_DEG);
    DEMO_WAIT(DEMO_STEP_MS);

    // flat
    setGroup(P_JOINTS,  0.0f);
    setGroup(IP_JOINTS, 0.0f);
    DEMO_WAIT(DEMO_STEP_MS);
  }

  zeroAllJoints();
  DEMO_WAIT(DEMO_HOLD_MS);

  // --- 4. sweeping close, pinky to index ------------------------------
  // Same end pose as the 'close' gesture, but rolled on finger by finger
  // from the pinky inwards instead of snapping shut all at once.
  //
  // The thumb swings clear FIRST: at zero it sits where the closing pinky
  // wants to go. It only folds back over once all four fingers are in.
  demoStep("4/5  thumb clear, close pinky->index, thumb back over");
  setThumbClear();
  DEMO_WAIT(DEMO_STEP_MS);

  for (int8_t k = 3; k >= 0; k--) {          // P_JOINTS run index->pinky, so walk back
    demoSet(P_JOINTS[k],  DEMO_CURL_P_DEG);
    demoSet(IP_JOINTS[k], DEMO_CURL_IP_DEG);
    DEMO_WAIT(DEMO_CLOSE_WAVE_MS);
  }
  // Thumb folds over the closed fingers last.
  demoSet(T_CMC, GEST_THUMB_CMC);
  demoSet(T_ABD, GEST_THUMB_ABD);
  demoSet(T_MCP, GEST_THUMB_MCP);
  demoSet(T_IP,  GEST_THUMB_IP);
  DEMO_WAIT(DEMO_HOLD_MS);

  // --- 5. point the index and circle it -------------------------------
  // Thumb has to get out of the way again — it is folded across the fist,
  // directly over the index that is about to come out.
  demoStep("5/5  thumb clear, index point + circle");
  setThumbClear();
  DEMO_WAIT(DEMO_STEP_MS);

  demoSet(I_P,  0.0f);   // straighten the index out of the fist
  demoSet(I_IP, 0.0f);
  demoSet(I_MP, 0.0f);
  DEMO_WAIT(DEMO_HOLD_MS);

  for (uint8_t rev = 0; rev < DEMO_CIRCLE_REVS; rev++) {
    for (uint8_t s = 0; s < DEMO_CIRCLE_STEPS; s++) {
      float th = (TWO_PI_F * (float)s) / (float)DEMO_CIRCLE_STEPS;
      // Sine on the side-to-side axis, cosine on the up-down axis: 90 deg
      // out of phase, so the fingertip sweeps a loop rather than a line.
      demoSet(I_MP, DEMO_CIRCLE_MP_AMP * sinf(th));
      demoSet(I_P,  DEMO_CIRCLE_P_MID + DEMO_CIRCLE_P_AMP * cosf(th));
      DEMO_WAIT(DEMO_CIRCLE_MS);
    }
  }

  zeroAllJoints();
  Serial.println(F("\n===== DEMO COMPLETE =====\n"));
}

// ===========================================================================
// DEMO 2 — thumb touches each fingertip in turn, PINKY to INDEX
//
// The four poses below were captured on the real hand with the PC app's
// sliders (new hardware), not derived.
// Values are in JointId order:
//   P_MP P_P P_IP | R_MP R_P R_IP | M_MP M_P M_IP | I_MP I_P I_IP
//   | T_CMC T_ABD T_MCP T_IP | W_1 W_2 W_3
//
// Note the fingers already touched keep a slight residual curl in later
// poses — that is how they were captured, and it keeps them clear of the
// thumb as it works inward. Don't "tidy" them to zero.
// ===========================================================================
static const uint8_t TOUCH_COUNT = 4;

static const char* const TOUCH_NAME[TOUCH_COUNT] = {
  "pinky", "ring", "middle", "index"
};

static const float TOUCH_POSE[TOUCH_COUNT][NUM_JOINTS] = {
  /* pinky  */ { -35.0f, 78.9f, 39.5f,
                  -0.3f, 14.9f, 18.0f,
                   0.3f, 15.9f, 15.4f,
                   1.1f, 14.9f, 17.0f,
                 -14.9f, 59.5f, 52.1f, 29.9f,
                   0.0f,  0.0f,  0.0f },   // wrist left at 0 (capture read 4.6 / 1.6 / -1.2)

  /* ring   */ {  -4.1f,  5.6f, 16.7f,
                   0.0f, 87.2f, 67.0f,
                   0.0f, 15.0f, 15.0f,
                   0.0f, 15.0f,  7.2f,
                 -11.2f, 33.2f, 60.4f, 33.3f,
                   0.0f,  0.0f,  0.0f },

  /* middle */ {  -4.1f,  5.6f, 16.7f,
                   0.0f,  7.4f, 29.9f,
                   0.0f, 71.4f, 78.9f,
                   0.0f, 15.0f,  7.2f,
                   2.3f, 20.6f, 73.6f,  5.1f,
                   0.0f,  0.0f,  0.0f },

  /* index  */ {  -4.1f,  5.6f, 16.7f,
                   0.0f,  7.4f, 29.9f,
                   0.0f,  7.4f, 37.1f,
                   0.0f, 71.4f, 41.9f,
                  24.3f, 17.4f, 37.3f, 28.3f,
                   0.0f,  0.0f,  0.0f },
};

static const float TOUCH_ZERO[NUM_JOINTS] = {
  0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f,
  0.0f, 0.0f, 0.0f,  0.0f, 0.0f, 0.0f,  0.0f,
  0.0f, 0.0f, 0.0f
};

// Pose the hand is currently holding, so a transition can start from it.
static float touchCurrent[NUM_JOINTS];

// Walks every joint from touchCurrent to 'target' together, so the whole
// hand arrives at once instead of joint-by-joint. Returns false if the
// user aborted.
static bool glideToPose(const float *target) {
  float start[NUM_JOINTS];
  for (uint8_t i = 0; i < NUM_JOINTS; i++) start[i] = touchCurrent[i];

  for (uint8_t s = 1; s <= DEMO2_STEPS; s++) {
    float t = (float)s / (float)DEMO2_STEPS;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      demoSet((JointId)i, start[i] + (target[i] - start[i]) * t);
    }
    if (!demoPause(DEMO2_STEP_MS)) return false;
  }

  for (uint8_t i = 0; i < NUM_JOINTS; i++) touchCurrent[i] = target[i];
  return true;
}

#define TOUCH_GLIDE(p)                                          \
  if (!glideToPose(p)) {                                        \
    demoAbortToZero();                                          \
    return;                                                     \
  }

void runTouchDemo() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }

  uint8_t unusable = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (!joints[i].attached || !hasBounds(joints[i])) unusable++;
  }
  if (unusable > 0) {
    Serial.print(F("  NOTE: "));
    Serial.print(unusable);
    Serial.println(F(" joint(s) are off or unhomed — the touch may not land."));
  }

  Serial.println(F("\n===== THUMB / FINGERTIP DEMO ====="));
  Serial.println(F("Send any character to abort.\n"));

  // Start from a known zero rather than wherever the hand was left.
  zeroAllJoints();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) touchCurrent[i] = 0.0f;
  DEMO_WAIT(DEMO2_TOUCH_MS);

  for (uint8_t p = 0; p < TOUCH_COUNT; p++) {
    Serial.print(F("  thumb -> "));
    Serial.println(TOUCH_NAME[p]);

    TOUCH_GLIDE(TOUCH_POSE[p]);
    DEMO_WAIT(DEMO2_TOUCH_MS);
  }

  Serial.println(F("  back to zero"));
  TOUCH_GLIDE(TOUCH_ZERO);

  zeroAllJoints();
  Serial.println(F("\n===== TOUCH DEMO COMPLETE =====\n"));
}

// ===========================================================================
// DEMO 3 — wrist pick-and-place
//
// Neutral, a wrist twirl, then rotate to one side and dip, grab, lift, carry
// across while lowering, release, return to neutral. Poses were set up on the real hand with the
// PC app's sliders. Values in JointId order:
//   P_MP P_P P_IP | R_MP R_P R_IP | M_MP M_P M_IP | I_MP I_P I_IP
//   | T_CMC T_ABD T_MCP T_IP | W_1 W_2 W_3
//
// Provenance matters here, because three of these poses weren't captured
// with numbers:
//   side, grab, release  exact slider readouts
//   lift, carry          grab's grip (so the hold never changes mid-carry)
//                        with their own exact wrist positions
//   neutral              ESTIMATED from slider positions, +/- a few degrees
// Correct any of them here directly — nothing else depends on the numbers.
// ===========================================================================
enum { W3_NEUTRAL, W3_SIDE, W3_GRAB, W3_LIFT, W3_CARRY, W3_RELEASE, W3_COUNT };

static const float WRIST_POSE[W3_COUNT][NUM_JOINTS] = {
  /* neutral — relaxed, slight curl (ESTIMATED) */
  {   0.0f,  15.0f,  16.0f,     0.0f,  15.0f,  16.0f,
      0.0f,  15.0f,  16.0f,     0.0f,  15.0f,  16.0f,
     60.0f,   3.0f, -12.0f,  10.0f,     0.0f,   0.0f,  0.0f },

  /* side — rotated clockwise and dipped, hand opening */
  {  -2.9f,  18.6f,   0.0f,     0.0f,  16.7f,   0.0f,
      0.0f,  20.4f,   0.0f,     0.0f,  30.6f,   0.0f,
     29.2f,  33.2f,  24.1f,  29.9f,   -82.6f, -43.7f,  0.0f },

  /* grab — pinch the object at the bottom of the dip (exact) */
  { -35.0f,  90.0f,  25.1f,   -23.5f,  90.0f,  29.9f,
     -0.2f,  75.2f,  69.4f,    23.5f,  72.4f,  43.1f,
    -12.5f,  31.1f,  52.1f,  23.3f,   -78.9f, -55.0f,  0.0f },

  /* lift — same grip, wrist raised */
  { -35.0f,  90.0f,  25.1f,   -23.5f,  90.0f,  29.9f,
     -0.2f,  75.2f,  69.4f,    23.5f,  72.4f,  43.1f,
    -12.5f,  31.1f,  52.1f,  23.3f,   -80.7f,   5.1f,  0.0f },

  /* carry — same grip, rotated across and lowered */
  { -35.0f,  90.0f,  25.1f,   -23.5f,  90.0f,  29.9f,
     -0.2f,  75.2f,  69.4f,    23.5f,  72.4f,  43.1f,
    -12.5f,  31.1f,  52.1f,  23.3f,    54.7f, -55.0f,  0.0f },

  /* release — fingers open and splay, thumb swings clear */
  { -35.0f,  33.4f,  22.7f,   -24.0f,  48.2f,  31.1f,
      0.0f,  38.0f,  28.7f,    22.2f,  57.5f,  26.3f,
      7.2f,  70.0f, -53.5f,  44.9f,    54.7f, -55.0f,  0.0f },
};

static float wristCurrent[NUM_JOINTS];

// Eased glide: every joint moves together from wristCurrent to 'target'
// over durationMs, following a half-cosine (starts and ends at zero speed).
// Returns false if the user aborted.
static bool wristGlide(const float *target, uint16_t durationMs) {
  float start[NUM_JOINTS];
  for (uint8_t i = 0; i < NUM_JOINTS; i++) start[i] = wristCurrent[i];

  uint16_t steps = durationMs / DEMO3_STEP_MS;
  if (steps < 1) steps = 1;
  for (uint16_t s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;
    float e = 0.5f - 0.5f * cosf(3.14159265f * t);
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      demoSet((JointId)i, start[i] + (target[i] - start[i]) * e);
    }
    if (!demoPause(DEMO3_STEP_MS)) return false;
  }
  for (uint8_t i = 0; i < NUM_JOINTS; i++) wristCurrent[i] = target[i];
  return true;
}

// On abort, ease back to neutral rather than snapping — the hand may be
// holding something, and an instant zero with the wrist at -55 deg would
// fling it. The return is itself uninterruptible, so it always completes.
static void wristAbortReturn() {
  if (safetyEstopActive()) {
    Serial.println(F("\n  E-stop — demo stopped."));
    return;
  }
  Serial.println(F("\n  Demo aborted — easing back to neutral."));
  safetyDrainSerial();
  float start[NUM_JOINTS];
  for (uint8_t i = 0; i < NUM_JOINTS; i++) start[i] = wristCurrent[i];
  uint16_t steps = DEMO3_T_RETURN_MS / DEMO3_STEP_MS;
  for (uint16_t s = 1; s <= steps; s++) {
    float e = 0.5f - 0.5f * cosf(3.14159265f * (float)s / (float)steps);
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      demoSet((JointId)i, start[i] + (WRIST_POSE[W3_NEUTRAL][i] - start[i]) * e);
    }
    // Not abortable by a keypress, but an E-stop still stops it.
    if (!safetyWait(DEMO3_STEP_MS, false)) return;
  }
}

// Wrist twirl about the current W_2/W_3 position: pitch follows sin and yaw
// follows cos, so the hand traces a circle. The radius eases from 0 up to
// full over the first half turn and back to 0 over the last, so it leaves
// and returns to the centre smoothly. Only W_2 and W_3 move. Returns false
// if the user aborted (wristCurrent is kept up to date for the abort glide).
static bool wristTwirl() {
  const float c2 = wristCurrent[W_2], c3 = wristCurrent[W_3];
  const uint32_t totalMs = (uint32_t)DEMO3_TWIRL_REVS * DEMO3_TWIRL_REV_MS;
  const uint32_t rampMs  = DEMO3_TWIRL_REV_MS / 2;
  const unsigned long t0 = millis();

  // The angle comes from the real elapsed time, not a count of ticks, so
  // the turn rate is exact however long each write-and-wait actually takes.
  while (true) {
    uint32_t tMs = (uint32_t)(millis() - t0);
    if (tMs >= totalMs) break;
    float theta = 6.2831853f * (float)tMs / (float)DEMO3_TWIRL_REV_MS;

    float r = 1.0f;                                  // radius envelope
    if (tMs < rampMs)                 r = 0.5f - 0.5f * cosf(3.14159265f * (float)tMs / (float)rampMs);
    else if (tMs > totalMs - rampMs)  r = 0.5f - 0.5f * cosf(3.14159265f * (float)(totalMs - tMs) / (float)rampMs);

    wristCurrent[W_2] = c2 + r * DEMO3_TWIRL_W2_AMP * sinf(theta);
    wristCurrent[W_3] = c3 + r * DEMO3_TWIRL_W3_AMP * cosf(theta);
    demoSet(W_2, wristCurrent[W_2]);
    demoSet(W_3, wristCurrent[W_3]);
    if (!demoPause(DEMO3_STEP_MS)) return false;
  }
  wristCurrent[W_2] = c2;
  wristCurrent[W_3] = c3;
  demoSet(W_2, c2);
  demoSet(W_3, c3);
  return true;
}

#define WRIST_STEP(label, pose, ms)                        \
  demoStep(label);                                         \
  if (!wristGlide(WRIST_POSE[pose], ms)) { wristAbortReturn(); return; } \
  if (!demoPause(DEMO3_HOLD_MS))         { wristAbortReturn(); return; }

void runWristDemo() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }

  uint8_t unusable = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (!joints[i].attached || !hasBounds(joints[i])) unusable++;
  }
  if (unusable > 0) {
    Serial.print(F("  NOTE: "));
    Serial.print(unusable);
    Serial.println(F(" joint(s) are off or unhomed and will not take part."));
  }

  Serial.println(F("\n===== WRIST PICK-AND-PLACE DEMO ====="));
  Serial.println(F("Send any character to abort.\n"));

  // Start the first glide from where the hand actually is, so reaching
  // neutral is a smooth move rather than a snap. Joints without trusted
  // feedback have to be assumed at zero.
  uint32_t trusted = updateAllJointAngles();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    wristCurrent[i] = ((trusted >> i) & 1UL) ? joints[i].currentAngleDeg : 0.0f;
  }

  WRIST_STEP("1/8  to neutral",                    W3_NEUTRAL, DEMO3_T_NEUTRAL_MS);

  demoStep("2/8  wrist twirl (ends back at neutral)");
  if (!wristTwirl())             { wristAbortReturn(); return; }
  if (!demoPause(DEMO3_HOLD_MS)) { wristAbortReturn(); return; }

  WRIST_STEP("3/8  rotate to the side and dip",    W3_SIDE,    DEMO3_T_SWING_MS);
  WRIST_STEP("4/8  grab",                          W3_GRAB,    DEMO3_T_GRAB_MS);
  WRIST_STEP("5/8  lift",                          W3_LIFT,    DEMO3_T_LIFT_MS);
  WRIST_STEP("6/8  carry across and lower",        W3_CARRY,   DEMO3_T_CARRY_MS);
  WRIST_STEP("7/8  release",                       W3_RELEASE, DEMO3_T_RELEASE_MS);
  WRIST_STEP("8/8  back to neutral",               W3_NEUTRAL, DEMO3_T_RETURN_MS);

  Serial.println(F("\n===== WRIST DEMO COMPLETE =====\n"));
}

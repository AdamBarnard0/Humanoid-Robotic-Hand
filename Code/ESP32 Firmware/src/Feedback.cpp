#include "Feedback.h"
#include "Config.h"
#include "HardwareIO.h"
#include "JointMap.h"
#include "Movement.h"
#include "Calibration.h"
#include "Safety.h"
#include <math.h>

// A joint with bounds still at 0/0 has never been homed.
// True once this joint can actually produce a reading: a muxed joint needs
// its channel discovered, a direct-ADC joint is always ready.
static bool jointHasFeedback(const Joint &j) {
  return (j.fbSrc == FB_ADC) || (j.muxChannel >= 0);
}

static bool hasBounds(const Joint &j) {
  return !(j.minPulseUs == 0 && j.maxPulseUs == 0);
}

// Position of an abduction joint across the hand, pinky(0) -> index(3).
// Returns -1 for anything that isn't an MP joint.
static int mpOrderIndex(JointId id) {
  switch (id) {
    case P_MP: return 0;
    case R_MP: return 1;
    case M_MP: return 2;
    case I_MP: return 3;
    default:   return -1;
  }
}

static const JointId MP_JOINTS[4] = { P_MP, R_MP, M_MP, I_MP };

// Poses the hand so 'id' can swing freely and unambiguously:
//   - every other joint sits at its 0 deg position
//   - if 'id' is an abduction joint, the other abduction joints are splayed
//     AWAY from it (fingers on the index side go +, pinky side go -) so
//     neighbouring fingers don't collide or drag on the one being moved.
static void poseForJoint(JointId id) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    if (!j.attached || !hasBounds(j)) continue;
    setJointAngleDeg((JointId)i, 0.0f);
  }

  int k = mpOrderIndex(id);
  if (k < 0) return;

  for (uint8_t m = 0; m < 4; m++) {
    JointId other = MP_JOINTS[m];
    if (other == id) continue;
    Joint &j = joints[other];
    if (!j.attached || !hasBounds(j)) continue;

    float loDeg, hiDeg;
    jointAngleLimits(other, loDeg, hiDeg);
    // Further toward the index side than the joint under test -> spread +,
    // otherwise spread -. e.g. testing M_MP gives I_MP +20, R_MP/P_MP -20.
    setJointAngleDeg(other, (mpOrderIndex(other) > k) ? hiDeg : loDeg);
  }
}

static void readAllChannels(float *out) {
  for (uint8_t ch = 0; ch < 16; ch++) {
    out[ch] = hwReadMuxVolts(ch, FEEDBACK_SAMPLES);
  }
}

// Records the pot voltage at this joint's two bounds. Assumes the hand is
// ALREADY posed for it (poseForJoint) and that it has a channel assigned.
// Returns false (having changed and saved nothing) on an E-stop.
static bool calibrateJointBounds(JointId id) {
  Joint &j = joints[id];

  setJointPulseUs(id, j.minPulseUs);
  if (!safetyWait(FEEDBACK_SETTLE_MS, false)) return false;
  float vMin = hwReadJointVolts(j, FEEDBACK_SAMPLES);

  setJointPulseUs(id, j.maxPulseUs);
  if (!safetyWait(FEEDBACK_SETTLE_MS, false)) return false;
  float vMax = hwReadJointVolts(j, FEEDBACK_SAMPLES);

  j.minBoundVolts = vMin;
  j.maxBoundVolts = vMax;

  Serial.print(F("    cal: "));
  Serial.print(j.minBoundVolts, 3);
  Serial.print(F("V @ min -> "));
  Serial.print(j.maxBoundVolts, 3);
  Serial.print(F("V @ max   span "));
  Serial.print(fabsf(j.maxBoundVolts - j.minBoundVolts), 3);
  Serial.println(F("V"));

  if (fabsf(j.maxBoundVolts - j.minBoundVolts) < FEEDBACK_MIN_DELTA_V) {
    Serial.println(F("    WARNING: span too small to resolve angle reliably."));
  }

  calibrationSaveJoint((uint8_t)id);
  return true;
}

// Waits during discovery; on an E-stop, puts back the channel assignments of
// every joint not yet done (they were cleared at the start) and says so.
#define FB_WAIT_OR_ABORT(ms)                                          \
  if (!safetyWait((ms), false)) { fbAbort(prevCh, i); return; }

static void fbAbort(const int8_t *prevCh, uint8_t from) {
  for (uint8_t k = from; k < NUM_JOINTS; k++) {
    if (joints[k].fbSrc == FB_MUX) joints[k].muxChannel = prevCh[k];
  }
  Serial.println(F("\n  E-stop — discovery abandoned. Joints not yet done keep their old channels."));
}

void discoverFeedbackChannels() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }
  SafetyPauseGuard pause;   // joints are driven hard to their bounds on purpose
  if (safetyEstopActive()) return;

  Serial.println(F("\n===== FEEDBACK CHANNEL DISCOVERY ====="));
  Serial.print(F("Jogging each joint in turn; a channel must move at least "));
  Serial.print(FEEDBACK_MIN_DELTA_V, 2);
  Serial.println(F("V to be claimed."));

  bool channelTaken[16];
  for (uint8_t c = 0; c < 16; c++) channelTaken[c] = false;
  int8_t prevCh[NUM_JOINTS];
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    prevCh[i] = joints[i].muxChannel;
    if (joints[i].fbSrc == FB_MUX) joints[i].muxChannel = -1;
  }

  float base[16], moved[16];

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    JointId id = (JointId)i;
    Joint &j = joints[id];

    Serial.print(F("\n  "));
    Serial.print(j.name);
    Serial.print(F(": "));

    if (!j.attached)   { Serial.println(F("motor OFF — skipped."));            continue; }
    if (!hasBounds(j)) { Serial.println(F("no bounds — home it first."));      continue; }

    // Wrist joints read a dedicated ADC pin fixed by wiring, so there is
    // nothing to discover — go straight to calibrating their bounds.
    if (j.fbSrc == FB_ADC) {
      Serial.print(F("direct ADC on GPIO"));
      Serial.print(j.adcPin);
      Serial.println(F(" — no search needed."));
      poseForJoint(id);
      FB_WAIT_OR_ABORT(FEEDBACK_SETTLE_MS);
      if (!calibrateJointBounds(id)) { fbAbort(prevCh, i); return; }
      setJointAngleDeg(id, 0.0f);
      FB_WAIT_OR_ABORT(FEEDBACK_SETTLE_MS);
      continue;
    }

    // Sweep the joint across its FULL travel, min bound to max bound, not
    // from its zero. For centre-referenced joints (the abductions, thumb
    // CMC/ABD/MCP) zero sits mid-range, so starting there would only ever
    // exercise half the span — e.g. an MP joint would swing 0..+20 instead
    // of -20..+20, halving the signal we're trying to detect.
    poseForJoint(id);
    setJointPulseUs(id, j.minPulseUs);
    FB_WAIT_OR_ABORT(FEEDBACK_SETTLE_MS);
    readAllChannels(base);

    setJointPulseUs(id, j.maxPulseUs);
    FB_WAIT_OR_ABORT(FEEDBACK_SETTLE_MS);
    readAllChannels(moved);

    // Largest response among channels not already claimed.
    int   best = -1;
    float bestD = 0.0f, secondD = 0.0f;
    for (uint8_t ch = 0; ch < 16; ch++) {
      if (channelTaken[ch]) continue;
      float d = fabsf(moved[ch] - base[ch]);
      if (d > bestD)        { secondD = bestD; bestD = d; best = (int)ch; }
      else if (d > secondD) { secondD = d; }
    }

    if (best >= 0 && bestD >= FEEDBACK_MIN_DELTA_V) {
      j.muxChannel = (int8_t)best;
      channelTaken[best] = true;
      Serial.print(F("mux ch "));
      Serial.print(best);
      Serial.print(F("  (moved "));
      Serial.print(bestD, 2);
      Serial.print(F("V, next best "));
      Serial.print(secondD, 2);
      Serial.println(F("V)"));
      if (secondD >= FEEDBACK_MIN_DELTA_V) {
        Serial.println(F("    WARNING: another channel also moved a lot — check for mechanical coupling."));
      }

      // The sweep above already sampled this channel at both bounds, so
      // calibration is free — base[] was taken at minPulseUs and moved[]
      // at maxPulseUs. No need to drive the joint there a second time.
      j.minBoundVolts = base[best];
      j.maxBoundVolts = moved[best];
      calibrationSaveJoint(i);

      Serial.print(F("    cal: "));
      Serial.print(j.minBoundVolts, 3);
      Serial.print(F("V @ min -> "));
      Serial.print(j.maxBoundVolts, 3);
      Serial.print(F("V @ max   span "));
      Serial.print(fabsf(j.maxBoundVolts - j.minBoundVolts), 3);
      Serial.println(F("V"));
    } else {
      Serial.print(F("no clear response (best "));
      Serial.print(bestD, 2);
      Serial.println(F("V) — left unassigned."));
      Serial.println(F("    Motor may be faulty, or its pot is disconnected."));
    }

    setJointAngleDeg(id, 0.0f);
    // This joint is finished (and saved) — on an abort, restore from the next.
    if (!safetyWait(FEEDBACK_SETTLE_MS, false)) { fbAbort(prevCh, (uint8_t)(i + 1)); return; }
  }

  // If exactly one eligible joint and exactly one channel are left over,
  // the answer is forced — assign it by elimination. This is what rescues
  // a joint whose motor is too broken to produce a readable swing.
  int lastJoint = -1, freeCh = -1;
  uint8_t unassignedJoints = 0, freeChannels = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    if (j.fbSrc != FB_MUX) continue;
    if (j.attached && hasBounds(j) && j.muxChannel < 0) { unassignedJoints++; lastJoint = (int)i; }
  }
  for (uint8_t c = 0; c < 16; c++) {
    if (!channelTaken[c]) { freeChannels++; freeCh = (int)c; }
  }

  if (unassignedJoints == 1 && freeChannels == 1) {
    joints[lastJoint].muxChannel = (int8_t)freeCh;
    Serial.print(F("\n  By elimination: "));
    Serial.print(joints[lastJoint].name);
    Serial.print(F(" -> mux ch "));
    Serial.println(freeCh);
  } else if (unassignedJoints > 0) {
    Serial.print(F("\n  "));
    Serial.print(unassignedJoints);
    Serial.print(F(" joint(s) left unassigned, "));
    Serial.print(freeChannels);
    Serial.println(F(" channel(s) free — too ambiguous to guess."));
  }

  // A joint assigned by elimination never got calibrated in the loop.
  if (unassignedJoints == 1 && freeChannels == 1) {
    JointId id = (JointId)lastJoint;
    poseForJoint(id);
    if (!safetyWait(FEEDBACK_SETTLE_MS, false) || !calibrateJointBounds(id)) {
      Serial.println(F("\n  E-stop — the last joint was left uncalibrated."));
      return;
    }
    setJointAngleDeg(id, 0.0f);
  }

  Serial.println(F("\n===== DISCOVERY COMPLETE ====="));
}

// readJointAngleDeg() falls back to the last known angle when it cannot
// actually measure, which is fine for telemetry but useless at boot — a
// stale 0.0 would read as a real position. This is the explicit test.
//
// The band check is what catches an unpowered pot: the pots are excited from
// the motor rail, so with that switched off (USB-only development) they read
// near 0V, which lands far outside any calibrated band.
// Whether a reading 'v' from this joint can be believed: homed, channel
// assigned, bound voltages calibrated, and v inside that calibrated band.
// Split out so the per-frame trust mask can reuse the reading it has just
// taken instead of reading the pot a second time.
static bool readingTrusted(const Joint &j, float v) {
  if (j.minPulseUs == 0 && j.maxPulseUs == 0) return false;   // never homed
  if (!jointHasFeedback(j)) return false;
  float span = j.maxBoundVolts - j.minBoundVolts;
  if (fabsf(span) < FEEDBACK_MIN_SPAN_V) return false;        // never calibrated
  float frac = (v - j.minBoundVolts) / span;
  return (frac >= -FEEDBACK_TRUST_MARGIN) && (frac <= 1.0f + FEEDBACK_TRUST_MARGIN);
}

bool jointFeedbackTrustworthy(JointId id) {
  if (id >= NUM_JOINTS) return false;
  Joint &j = joints[id];
  if (!j.attached) return false;
  if (!jointHasFeedback(j)) return false;
  float v = hwReadJointVolts(j, FEEDBACK_SAMPLES);
  j.lastFeedbackVolts = v;
  return readingTrusted(j, v);
}

// Volts -> degrees. minBoundVolts corresponds to minPulseUs, and
// jointAngleLimits() returns exactly the angles at min and max, so this is
// a straight remap. Returns false (leaving currentAngleDeg alone) if the
// joint's bound voltages were never calibrated.
static bool voltsToAngle(JointId id, float v) {
  Joint &j = joints[id];
  float span = j.maxBoundVolts - j.minBoundVolts;
  if (fabsf(span) < FEEDBACK_MIN_SPAN_V) return false;
  float loDeg, hiDeg;
  jointAngleLimits(id, loDeg, hiDeg);
  j.currentAngleDeg = loDeg + ((v - j.minBoundVolts) / span) * (hiDeg - loDeg);
  return true;
}

// ===========================================================================
// BACKGROUND SAMPLING + FILTERING
//
// feedbackServiceTick() runs every loop() pass and reads ONE joint's pot,
// round-robin, folding the result into that joint's low-pass filter. Each
// joint is therefore refreshed a couple of hundred times a second from
// readings spread out in time — which is what averaging needs to actually
// remove noise. Samples taken microseconds apart share the same noise and
// average away almost nothing.
//
// updateAllJointAngles() then just converts the filtered voltages: no ADC
// reads, so an S reply is built instantly.
// ===========================================================================
static float    filtV[NUM_JOINTS];
static uint32_t filtAtUs[NUM_JOINTS];
static bool     filtValid[NUM_JOINTS];
static uint8_t  bgNext = 0;

// First-order low-pass using the REAL time since this joint's last update:
// alpha = dt / (tau + dt). Behaves the same at any loop speed, and after a
// long gap (dt >> tau) alpha -> 1, so a stale filter snaps to the new value.
static void filterUpdate(uint8_t i, float v, uint32_t nowUs) {
  if (!filtValid[i]) {
    filtV[i] = v;
    filtValid[i] = true;
  } else {
    float dtMs = (float)(uint32_t)(nowUs - filtAtUs[i]) / 1000.0f;
    float a = dtMs / (FEEDBACK_FILTER_TAU_MS + dtMs);
    filtV[i] += a * (v - filtV[i]);
  }
  filtAtUs[i] = nowUs;
}

// Discards filter history and starts again from a fresh reading.
static void filterReset(uint8_t i, float v) {
  filtValid[i] = false;
  filterUpdate(i, v, micros());
}

void feedbackServiceTick() {
  for (uint8_t k = 0; k < NUM_JOINTS; k++) {
    uint8_t i = (uint8_t)((bgNext + k) % NUM_JOINTS);
    if (!jointHasFeedback(joints[i])) continue;
    float v = hwReadJointVolts(joints[i], FEEDBACK_BG_BATCH);
    filterUpdate(i, v, micros());
    bgNext = (uint8_t)((i + 1) % NUM_JOINTS);
    return;
  }
}

// A one-off synchronous read, used by routines and diagnostics that need a
// fresh measurement right now. Also re-seeds the background filter, since a
// fresh batched reading is the best starting point it could have.
float readJointAngleDeg(JointId id, uint8_t samples) {
  if (id >= NUM_JOINTS) return 0.0f;
  Joint &j = joints[id];
  if (!jointHasFeedback(j)) return j.currentAngleDeg;

  float v = hwReadJointVolts(j, samples);
  j.lastFeedbackVolts = v;
  filterReset(id, v);
  voltsToAngle(id, v);
  return j.currentAngleDeg;
}

uint32_t updateAllJointAngles() {
  // Normally no ADC work at all: the background sampler keeps every filter
  // fresh. A filter goes stale only while something blocking has stopped
  // loop() — a routine, or boot — and is then re-read here before use.
  const uint32_t now = micros();
  const uint32_t staleUs = (uint32_t)FEEDBACK_STALE_MS * 1000UL;
  uint32_t trusted = 0;
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    if (!jointHasFeedback(j)) continue;
    if (!filtValid[i] || (uint32_t)(now - filtAtUs[i]) > staleUs) {
      filterReset(i, hwReadJointVolts(j, FEEDBACK_BG_BATCH));
    }
    j.lastFeedbackVolts = filtV[i];
    voltsToAngle((JointId)i, filtV[i]);
    if (readingTrusted(j, filtV[i])) trusted |= (1UL << i);
  }
  return trusted;
}

bool feedbackFilteredAngle(JointId id, uint32_t maxAgeMs, float &deg) {
  if (id >= NUM_JOINTS) return false;
  const Joint &j = joints[id];
  if (!j.attached || !jointHasFeedback(j) || !filtValid[id]) return false;
  if ((uint32_t)(micros() - filtAtUs[id]) > maxAgeMs * 1000UL) return false;
  if (!readingTrusted(j, filtV[id])) return false;
  float span = j.maxBoundVolts - j.minBoundVolts;
  float loDeg, hiDeg;
  jointAngleLimits(id, loDeg, hiDeg);
  deg = loDeg + ((filtV[id] - j.minBoundVolts) / span) * (hiDeg - loDeg);
  return true;
}

void printJointFeedback() {
  Serial.println(F("\n--- Joint feedback ---"));
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    Serial.print(F("  "));
    Serial.print(j.name);
    if (!jointHasFeedback(j)) {
      Serial.println(F(": no feedback channel assigned (run 'fbfind')"));
      continue;
    }
    readJointAngleDeg((JointId)i);
    Serial.print(F(": ch "));
    Serial.print(j.muxChannel);
    Serial.print(F("  "));
    Serial.print(j.lastFeedbackVolts, 3);
    Serial.print(F("V  measured "));
    Serial.print(j.currentAngleDeg, 1);
    Serial.print(F(" deg   (commanded "));
    Serial.print(j.currentPulseUs);
    Serial.println(F("us)"));
  }
  Serial.println();
}

// ===========================================================================
// CONNECTIVITY TEST
//
// Centres the whole hand, then takes each joint on its own through
// min -> max -> centre while watching that joint's own feedback pot. A
// joint whose pot barely moves has either a dead motor, a disconnected
// signal wire, or an uncalibrated feedback channel — the point is to tell
// you which joints are actually alive before you trust anything else.
// ===========================================================================
static bool testAbort() {
  if (!safetyRoutinePoll(true)) return false;
  Serial.println(safetyEstopActive() ? F("\n  E-stop — test stopped.") : F("\n  Test aborted."));
  return true;
}

void runConnectivityTest() {
  if (!pca9685Present) {
    Serial.println(F("PCA9685 not detected, skipping."));
    return;
  }

  Serial.println(F("\n===== CONNECTIVITY TEST ====="));
  Serial.println(F("Centring everything, then sweeping each motor in turn."));
  Serial.println(F("Send any character to abort.\n"));

  SafetyPauseGuard pause;   // every joint is driven to both bounds on purpose
  centerAllJoints();
  if (!safetyWait(TEST_SETTLE_MS, false)) { testAbort(); return; }

  uint8_t passed = 0, failed = 0, skipped = 0;

  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    JointId id = (JointId)i;
    Joint &j = joints[id];

    Serial.print(F("  "));
    Serial.print(j.name);
    Serial.print(j.driver == DRV_LEDC ? F(" (gpio ") : F(" (pca "));
    Serial.print(j.driver == DRV_LEDC ? j.pwmPin : j.pcaChannel);
    Serial.print(F("): "));

    if (!j.attached)   { Serial.println(F("motor OFF — skipped."));       skipped++; continue; }
    if (!hasBounds(j)) { Serial.println(F("no bounds — home it first.")); skipped++; continue; }

    // Sweep the full travel and watch this joint's own pot at each end.
    setJointPulseUs(id, j.minPulseUs);
    if (!safetyWait(TEST_SETTLE_MS, false)) { testAbort(); return; }
    float vMin = hwReadJointVolts(j, FEEDBACK_SAMPLES);

    setJointPulseUs(id, j.maxPulseUs);
    if (!safetyWait(TEST_SETTLE_MS, false)) { testAbort(); return; }
    float vMax = hwReadJointVolts(j, FEEDBACK_SAMPLES);

    centerJoint(id);

    float swing = fabsf(vMax - vMin);
    bool  ok    = jointHasFeedback(j) && (swing >= TEST_MIN_SWING_V);

    Serial.print(ok ? F("PASS") : F("FAIL"));
    Serial.print(F("  swing "));
    Serial.print(swing, 3);
    Serial.print(F("V"));

    if (j.driver == DRV_PCA9685) {
      Serial.print(F("  ("));
      Serial.print(j.currentMA, 1);
      Serial.print(F(" mA)"));
    }

    if (!jointHasFeedback(j)) {
      Serial.print(F("  <- no feedback channel, run 'fbfind'"));
    } else if (!ok) {
      Serial.print(F("  <- motor not moving, or pot disconnected"));
    }
    Serial.println();

    ok ? passed++ : failed++;
    if (testAbort()) return;
  }

  delay(TEST_SETTLE_MS);

  Serial.print(F("\n  "));
  Serial.print(passed);
  Serial.print(F(" passed, "));
  Serial.print(failed);
  Serial.print(F(" failed, "));
  Serial.print(skipped);
  Serial.println(F(" skipped."));

  float rail = hwReadRailMilliamps();
  if (!isnan(rail)) {
    Serial.print(F("  Rail current at rest: "));
    Serial.print(rail, 1);
    Serial.println(F(" mA"));
  }
  Serial.println(F("===== TEST COMPLETE =====\n"));
}

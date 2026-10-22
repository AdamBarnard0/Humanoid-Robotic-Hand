#include "Protocol.h"
#include "Config.h"
#include "JointMap.h"
#include "Movement.h"
#include "Feedback.h"
#include "CurrentMonitor.h"
#include "HardwareIO.h"
#include "Diagnostics.h"
#include "Homing.h"
#include "Demo.h"
#include "Safety.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ===========================================================================
// PC CONTROL PROTOCOL v3 — see PROTOCOL.md for the wire specification.
//
// Lockstep: the PC is the only clock. Every line received in PC mode gets
// exactly one reply line; the hand never transmits unprompted. Motion runs
// on its own 50 Hz slew limiter so it stays smooth between 30 Hz targets.
// ===========================================================================

static ControlMode mode = MODE_TERMINAL;

// ---- session state --------------------------------------------------------
static unsigned long lastRxMs     = 0;     // last valid T frame
static unsigned long lastMotionMs = 0;
static bool  wdArmed   = false;            // armed by the first valid T
static bool  wdFired   = false;            // fired since the last S reply
static int   lastSeq   = -1;               // -1 = nothing seen this session
static bool  routineJustRan = false;       // exclude that pass from loop stats

// Last angle actually written to each joint by the slew limiter. Lets the
// motion tick skip joints whose setpoint hasn't moved — otherwise all 16
// PCA9685 channels are rewritten over I2C every 20 ms even when the hand is
// standing still. NAN forces a write on the first tick after (re)seeding.
static float lastWrittenDeg[NUM_JOINTS];

static struct {
  uint32_t rx, ok, cs, fmt, lng, unk, wd;
  uint32_t es;                             // commands refused because of the E-stop
  uint32_t loopMaxUs;
} st;

static const uint8_t FLAG_WDOG  = 0x01;
static const uint8_t FLAG_GAP   = 0x02;
static const uint8_t FLAG_NODRV = 0x04;
static const uint8_t FLAG_ESTOP = 0x08;   // E-stop latched: outputs off, targets ignored
static const uint8_t FLAG_FAULT = 0x10;   // >= 1 joint stopped by protection ('C faults')

static bool hasBounds(const Joint &j) {
  return !(j.minPulseUs == 0 && j.maxPulseUs == 0);
}

// ---------------------------------------------------------------------------
// Output. Every protocol frame is built into one buffer, checksummed, and
// written with a single call.
// ---------------------------------------------------------------------------
static char txBuf[PROTO_TX_MAX];

static uint8_t xorOf(const char *s, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= (uint8_t)s[i];
  return x;
}

static void sendFrame(size_t len) {
  if (len > sizeof(txBuf) - 6) len = sizeof(txBuf) - 6;   // never overrun
  uint8_t cs = xorOf(txBuf, len);
  len += (size_t)snprintf(txBuf + len, sizeof(txBuf) - len, "*%02X\r\n", cs);
  Serial.write((const uint8_t *)txBuf, len);
}

// "<type> <seq> <text>" or "<type> - <text>" when seq < 0.
static void sendSimple(char type, long seq, const char *text) {
  int n = (seq < 0) ? snprintf(txBuf, sizeof(txBuf), "%c - %s", type, text)
                    : snprintf(txBuf, sizeof(txBuf), "%c %ld %s", type, seq, text);
  sendFrame((size_t)n);
}

// ---------------------------------------------------------------------------
// Strict tokenising. Every token is preceded by exactly one space; the
// caller checks the cursor lands exactly on the end afterwards, so double,
// leading and trailing spaces are all rejected.
// ---------------------------------------------------------------------------

// " <int>" — optional '-', 1..5 digits.
static bool nextInt(const char *&p, const char *end, long &out) {
  if (p >= end || *p != ' ') return false;
  const char *s = p + 1;
  bool neg = false;
  if (s < end && *s == '-') { neg = true; s++; }
  const char *digits = s;
  long v = 0;
  while (s < end && *s >= '0' && *s <= '9') {
    if (s - digits >= 5) return false;
    v = v * 10 + (*s - '0');
    s++;
  }
  if (s == digits) return false;             // no digits at all
  if (s < end && *s != ' ') return false;    // junk inside the token
  out = neg ? -v : v;
  p = s;
  return true;
}

// " <word>" — letters, digits and '_' only.
static bool nextWord(const char *&p, const char *end, char *out, size_t cap) {
  if (p >= end || *p != ' ') return false;
  const char *s = p + 1;
  size_t n = 0;
  while (s < end && *s != ' ') {
    char c = *s;
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '_';
    if (!ok || n + 1 >= cap) return false;
    out[n++] = c;
    s++;
  }
  if (n == 0) return false;
  out[n] = '\0';
  p = s;
  return true;
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// Joint lookup by exact, case-sensitive name. Channel numbers are a wiring
// detail the PC has no business knowing, so they are deliberately refused.
static JointId jointByExactName(const char *name) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (strcmp(joints[i].name, name) == 0) return (JointId)i;
  }
  return NUM_JOINTS;
}

// ---------------------------------------------------------------------------
// Motion
// ---------------------------------------------------------------------------

// Re-seeds the slew limiter from where each joint actually IS. Called on
// entering PC mode and after any routine that drove the joints directly —
// without it the next T frame would slew from a stale position and jump.
static void seedFromMeasured() {
  uint32_t trusted = updateAllJointAngles();
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    // Trusted measurement if we have one; otherwise the last commanded
    // angle, which is our best estimate. (v2 used muxChannel >= 0 as the
    // test, which is never true for the wrist's direct-ADC pots, so the
    // wrist was seeded at 0 deg and jumped there on entering PC mode.)
    float start = ((trusted >> i) & 1UL) ? j.currentAngleDeg : j.commandedAngleDeg;

    float loDeg, hiDeg;
    jointAngleLimits((JointId)i, loDeg, hiDeg);
    if (start < loDeg) start = loDeg;
    if (start > hiDeg) start = hiDeg;

    j.commandedAngleDeg = start;
    j.targetAngleDeg    = start;
    lastWrittenDeg[i]   = NAN;
  }
}

static void resetSpeeds() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    joints[i].targetSpeedDegSec = PROTO_MAX_SPEED * jointSpeedScale((JointId)i);
  }
}

// Walks each joint's commanded angle toward its target no faster than its
// speed. The servos close their own position loop, so ramping the setpoint
// is the only thing "speed" can mean.
static void motionTick(float dtSec) {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    if (!j.attached || !hasBounds(j)) continue;

    float stepDeg = j.targetSpeedDegSec * dtSec;
    float diff    = j.targetAngleDeg - j.commandedAngleDeg;

    if (fabsf(diff) <= stepDeg) j.commandedAngleDeg = j.targetAngleDeg;
    else                        j.commandedAngleDeg += (diff > 0.0f) ? stepDeg : -stepDeg;

    if (isnan(lastWrittenDeg[i]) || fabsf(j.commandedAngleDeg - lastWrittenDeg[i]) > 0.001f) {
      setJointAngleDeg((JointId)i, j.commandedAngleDeg);
      lastWrittenDeg[i] = j.commandedAngleDeg;
    }
  }
}

// Clamps to the joint's range in TENTHS so the comparison is exact —
// a target sitting precisely on a limit must not be reported as clamped.
static bool applyTarget(JointId id, long tenths) {
  float loDeg, hiDeg;
  jointAngleLimits(id, loDeg, hiDeg);
  long lo = lroundf(loDeg * 10.0f), hi = lroundf(hiDeg * 10.0f);
  bool clamped = false;
  if (tenths < lo) { tenths = lo; clamped = true; }
  if (tenths > hi) { tenths = hi; clamped = true; }
  joints[id].targetAngleDeg = (float)tenths / 10.0f;
  return clamped;
}

// ---------------------------------------------------------------------------
// S — the reply to a valid T
// ---------------------------------------------------------------------------
static long clampL(long v, long lo, long hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void sendState(long seq, uint8_t flags, uint32_t clamp) {
  uint32_t fb = updateAllJointAngles();   // converts the filtered readings; no ADC work
                                          // (the wrist rail current is sampled in safetyService())

  int n = snprintf(txBuf, sizeof(txBuf), "S %ld %02X %05lX %05lX",
                   seq, flags, (unsigned long)clamp, (unsigned long)fb);
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    long a = clampL(lroundf(joints[i].currentAngleDeg * 10.0f), -32768, 32767);
    long m = clampL(lroundf(joints[i].currentMA), -99999, 99999);
    n += snprintf(txBuf + n, sizeof(txBuf) - n, " %ld %ld", a, m);
  }
  sendFrame((size_t)n);
}

// ---------------------------------------------------------------------------
// Frame handlers
// ---------------------------------------------------------------------------
static void reject(uint32_t &counter, long seq, const char *code) {
  counter++;
  sendSimple('E', seq, code);
}

static void handleTarget(const char *line, const char *end) {
  const char *p = line + 1;
  long seq;
  if (!nextInt(p, end, seq) || seq < 0 || seq > 255) { reject(st.fmt, -1, "format"); return; }

  // Parse the whole frame into scratch first — a frame that goes bad half
  // way through must not leave half the hand on new targets.
  long v[NUM_JOINTS];
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    if (!nextInt(p, end, v[i]) || v[i] < -32768 || v[i] > 32767) { reject(st.fmt, seq, "format"); return; }
  }
  if (p != end) { reject(st.fmt, seq, "format"); return; }

  st.ok++;
  uint8_t flags = 0;
  if (lastSeq >= 0 && seq != ((lastSeq + 1) & 0xFF)) flags |= FLAG_GAP;
  lastSeq = (int)seq;
  if (wdFired)         { flags |= FLAG_WDOG; wdFired = false; }
  if (!pca9685Present) flags |= FLAG_NODRV;
  if (safetyFaultMask()) flags |= FLAG_FAULT;

  // While E-stopped the frame is still answered — the PC keeps seeing where
  // the (limp) hand is — but its targets are not taken.
  uint32_t clamp = 0;
  if (safetyEstopActive()) {
    flags |= FLAG_ESTOP;
  } else {
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      if (applyTarget((JointId)i, v[i])) clamp |= (1UL << i);
    }
  }

  lastRxMs = millis();
  wdArmed  = true;
  sendState(seq, flags, clamp);
}

// Routines drive joints directly and block, so afterwards: throw away
// anything the PC sent meanwhile (acting on it now would chase stale
// targets), re-seed the slew limiter, and leave the watchdog disarmed until
// the next T.
static void routineBegin(long seq, const char *name) { sendSimple('K', seq, name); }

static void routineEnd(long seq, const char *name) {
  safetyDrainSerial();   // discards, but still honours an E-stop sent meanwhile
  seedFromMeasured();
  lastMotionMs   = millis();
  wdArmed        = false;
  routineJustRan = true;
  sendSimple('D', seq, name);
}

// Instant commands (speed, stats) don't move anything, so they skip the
// drain and re-seed — calling one mid-teleop must not disturb targets.
static void instantDone(long seq, const char *name) { sendSimple('D', seq, name); }

static void handleCmdFrame(const char *line, const char *end) {
  const char *p = line + 1;
  long seq;
  if (!nextInt(p, end, seq) || seq < 0 || seq > 255) { reject(st.fmt, -1, "format"); return; }

  char name[16], a1[16] = "", a2[16] = "";
  int argc = 0;
  if (!nextWord(p, end, name, sizeof(name))) { reject(st.fmt, seq, "format"); return; }
  if (p < end) { if (!nextWord(p, end, a1, sizeof(a1))) { reject(st.fmt, seq, "format"); return; } argc = 1; }
  if (p < end) { if (!nextWord(p, end, a2, sizeof(a2))) { reject(st.fmt, seq, "format"); return; } argc = 2; }
  if (p != end) { reject(st.fmt, seq, "badarg"); return; }

  lastSeq = (int)seq;   // C frames share the PC's sequence counter

  // --- safety: always accepted, E-stopped or not ---
  if (strcmp(name, "estop") == 0) {
    if (argc != 0) { reject(st.fmt, seq, "badarg"); return; }
    st.ok++;
    safetyEstop("pc");
    routineBegin(seq, name); instantDone(seq, name);
    return;
  }
  if (strcmp(name, "reset") == 0) {
    if (argc != 0) { reject(st.fmt, seq, "badarg"); return; }
    st.ok++;
    routineBegin(seq, name);
    safetyReset();
    // Same re-sync as after a routine: the slew limiter restarts from where
    // the joints now are, and the watchdog waits for the next T.
    seedFromMeasured();
    lastMotionMs = millis();
    wdArmed      = false;
    instantDone(seq, name);
    return;
  }
  if (strcmp(name, "faults") == 0) {
    if (argc != 0) { reject(st.fmt, seq, "badarg"); return; }
    st.ok++;
    routineBegin(seq, name);
    int n = snprintf(txBuf, sizeof(txBuf), "I ");
    n += safetyFormatFaults(txBuf + n, sizeof(txBuf) - (size_t)n);
    sendFrame((size_t)n);
    instantDone(seq, name);
    return;
  }

  // Anything that moves the hand is refused while E-stopped. 'speed' and
  // 'stats' don't move anything and stay available.
  if (safetyEstopActive() && strcmp(name, "speed") != 0 && strcmp(name, "stats") != 0) {
    reject(st.es, seq, "estop");
    return;
  }

  // --- routines without arguments ---
  struct Routine { const char *name; void (*fn)(); };
  static const Routine plain[] = {
    { "demo1",   runMovementDemo },
    { "demo2",   runTouchDemo },
    { "demo3",   runWristDemo },
    { "homeall", homeAllJoints },
    { "fbfind",  discoverFeedbackChannels },
    { "test",    runConnectivityTest },
    { "zero",    zeroAllJoints },
    { "center",  centerAllJoints },
  };
  for (const Routine &r : plain) {
    if (strcmp(name, r.name) != 0) continue;
    if (argc != 0) { reject(st.fmt, seq, "badarg"); return; }
    st.ok++;
    routineBegin(seq, name); r.fn(); routineEnd(seq, name);
    return;
  }

  if (strcmp(name, "manualhome") == 0) {
    if (argc > 1) { reject(st.fmt, seq, "badarg"); return; }
    if (argc == 1) {
      JointId id = jointByExactName(a1);
      if (id == NUM_JOINTS) { reject(st.fmt, seq, "badjoint"); return; }
      st.ok++;
      routineBegin(seq, name); manualHomeSingleJoint(id); routineEnd(seq, name);
    } else {
      st.ok++;
      routineBegin(seq, name); manualHomeAllJoints(); routineEnd(seq, name);
    }
    return;
  }

  if (strcmp(name, "sweep_r") == 0) {
    if (argc != 1) { reject(st.fmt, seq, "badarg"); return; }
    JointId id = jointByExactName(a1);
    if (id == NUM_JOINTS) { reject(st.fmt, seq, "badjoint"); return; }
    st.ok++;
    routineBegin(seq, name); sweepJointRange(id); routineEnd(seq, name);
    return;
  }

  if (strcmp(name, "speed") == 0) {
    if (argc != 2) { reject(st.fmt, seq, "badarg"); return; }
    bool all = (strcmp(a1, "all") == 0);
    JointId id = all ? NUM_JOINTS : jointByExactName(a1);
    if (!all && id == NUM_JOINTS) { reject(st.fmt, seq, "badjoint"); return; }

    // a2 was accepted by nextWord; it must also be a plain positive integer.
    long dps = 0;
    size_t len = strlen(a2);
    if (len == 0 || len > 5) { reject(st.fmt, seq, "badarg"); return; }
    for (size_t k = 0; k < len; k++) {
      if (a2[k] < '0' || a2[k] > '9') { reject(st.fmt, seq, "badarg"); return; }
      dps = dps * 10 + (a2[k] - '0');
    }

    st.ok++;
    routineBegin(seq, name);
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      if (!all && i != (uint8_t)id) continue;
      float ceiling = PROTO_MAX_SPEED * jointSpeedScale((JointId)i);
      float s = (float)dps;
      if (s < PROTO_MIN_SPEED) s = PROTO_MIN_SPEED;
      if (s > ceiling)         s = ceiling;
      joints[i].targetSpeedDegSec = s;
    }
    instantDone(seq, name);
    return;
  }

  if (strcmp(name, "stats") == 0) {
    if (argc != 0) { reject(st.fmt, seq, "badarg"); return; }
    st.ok++;
    routineBegin(seq, name);
    int n = snprintf(txBuf, sizeof(txBuf),
                     "I rx=%lu ok=%lu cs=%lu fmt=%lu long=%lu unk=%lu wd=%lu loop=%lu",
                     (unsigned long)st.rx, (unsigned long)st.ok, (unsigned long)st.cs,
                     (unsigned long)st.fmt, (unsigned long)st.lng, (unsigned long)st.unk,
                     (unsigned long)st.wd, (unsigned long)st.loopMaxUs);
    sendFrame((size_t)n);
    st.loopMaxUs = 0;
    instantDone(seq, name);
    return;
  }

  reject(st.fmt, seq, "badcmd");
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
ControlMode controlModeGet() { return mode; }

void controlModeSet(ControlMode m) {
  mode = m;
  if (m == MODE_PC) {
    memset(&st, 0, sizeof(st));
    lastSeq  = -1;
    wdArmed  = false;
    wdFired  = false;
    resetSpeeds();
    seedFromMeasured();
    lastMotionMs = millis();
    Serial.print(F("MODE PC\r\n"));
  } else {
    Serial.print(F("MODE TERM\r\n"));
    printMenu();
  }
}

void protocolHandleLine(const char *line, size_t len) {
  if (len == 0) return;                       // blank lines are ignored

  // Plain handshake words: no seq, no checksum, so they can be typed.
  if (len == 2 && memcmp(line, "pc", 2) == 0)   { controlModeSet(MODE_PC);       return; }
  if (len == 4 && memcmp(line, "term", 4) == 0) { controlModeSet(MODE_TERMINAL); return; }

  st.rx++;

  // Checksum: '*' followed by exactly two hex digits at the very end.
  if (len < 4 || line[len - 3] != '*') { reject(st.cs, -1, "checksum"); return; }
  int hi = hexVal(line[len - 2]), lo = hexVal(line[len - 1]);
  if (hi < 0 || lo < 0)                 { reject(st.cs, -1, "checksum"); return; }
  const char *end = line + len - 3;
  if (xorOf(line, (size_t)(end - line)) != (uint8_t)(hi * 16 + lo)) {
    reject(st.cs, -1, "checksum");
    return;
  }

  switch (line[0]) {
    case 'T': handleTarget(line, end);  break;
    case 'C': handleCmdFrame(line, end); break;
    default:  reject(st.unk, -1, "unknown");
  }
}

void protocolLineTooLong() {
  st.rx++;
  reject(st.lng, -1, "toolong");
}

void protocolNoteLoopUs(uint32_t us) {
  if (routineJustRan) { routineJustRan = false; return; }
  if (us > st.loopMaxUs) st.loopMaxUs = us;
}

void protocolTick() {
  if (mode != MODE_PC) return;
  unsigned long now = millis();

  // Watchdog: if the PC goes quiet, freeze targets rather than keep chasing
  // a stale one. Silent — reported via FLAG_WDOG on the next reply.
  if (wdArmed && (now - lastRxMs) > PROTO_TIMEOUT_MS) {
    wdArmed = false;
    wdFired = true;
    st.wd++;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      joints[i].targetAngleDeg = joints[i].commandedAngleDeg;
    }
  }

  if (now - lastMotionMs >= PROTO_MOTION_MS) {
    float dt = (float)(now - lastMotionMs) / 1000.0f;
    lastMotionMs = now;
    // E-stopped: the setpoints stay frozen where they were. 'C reset'
    // re-seeds them from the measured positions before motion resumes.
    if (!safetyEstopActive()) motionTick(dt);
  }
  // Current sensing now runs from safetyService() in both modes.
}

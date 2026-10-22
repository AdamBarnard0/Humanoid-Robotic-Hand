#include "Gestures.h"
#include "Config.h"
#include "JointMap.h"
#include "Movement.h"
#include "HardwareIO.h"

// Pose building blocks all come from Config.h (section 9) — tune them
// there. Short aliases keep the table below readable.
#define FINGER_UP_P    GEST_FINGER_UP_P
#define FINGER_UP_IP   GEST_FINGER_UP_IP
#define FINGER_CURL_P  GEST_FINGER_CURL_P
#define FINGER_CURL_IP GEST_FINGER_CURL_IP
#define THUMB_CMC      GEST_THUMB_CMC
#define THUMB_ABD      GEST_THUMB_ABD
#define THUMB_MCP      GEST_THUMB_MCP
#define THUMB_IP       GEST_THUMB_IP
#define SPLAY_INDEX    GEST_SPLAY_INDEX
#define SPLAY_MIDDLE   GEST_SPLAY_MIDDLE
#define MP_NEUTRAL     GEST_MP_NEUTRAL
// Gestures are hand shapes — the wrist stays neutral in all of them.
#define WRIST_NEUTRAL  0.0f

struct Gesture {
  const char* name;
  const char* description;
  float deg[NUM_JOINTS];   // MUST stay in JointId enum order
};

// Column order, per JointTypes.h:
//   P_MP P_P P_IP | R_MP R_P R_IP | M_MP M_P M_IP | I_MP I_P I_IP
//   | T_CMC T_ABD T_MCP T_IP | W_1 W_2 W_3
static const Gesture GESTURES[] = {
  // Captured on the real hand from the PC app's sliders — exact per-joint
  // values, not built from the shared blocks above.
  { "peace", "index + middle up and splayed, ring + pinky curled",
    {   3.0f,  80.0f, 100.0f,                          // pinky  - down
       -5.0f,  80.0f, 100.0f,                          // ring   - down
       -8.0f,   0.0f,   0.0f,                          // middle - up, splayed
        8.0f,   0.0f,   0.0f,                          // index  - up, splayed
        1.0f,  65.0f,  48.8f,  36.7f,                  // thumb  - across the palm
      WRIST_NEUTRAL, WRIST_NEUTRAL, WRIST_NEUTRAL } },

  { "rock", "index + pinky up, middle + ring curled",
    { MP_NEUTRAL,   FINGER_UP_P,   FINGER_UP_IP,      // pinky  - up
      MP_NEUTRAL,   FINGER_CURL_P, FINGER_CURL_IP,    // ring   - down
      MP_NEUTRAL,   FINGER_CURL_P, FINGER_CURL_IP,    // middle - down
      MP_NEUTRAL,   FINGER_UP_P,   FINGER_UP_IP,      // index  - up
      THUMB_CMC,    THUMB_ABD,     THUMB_MCP, THUMB_IP,
      WRIST_NEUTRAL, WRIST_NEUTRAL, WRIST_NEUTRAL } },

  // Captured on the real hand from the PC app's sliders. P and IP sit
  // exactly on their limits (90 / 116 deg).
  { "close", "fist — every finger curled, thumb folded over",
    {   0.0f,  90.0f, 116.0f,                          // pinky  - down
        0.0f,  90.0f, 116.0f,                          // ring   - down
        0.0f,  90.0f, 116.0f,                          // middle - down
        0.0f,  90.0f, 116.0f,                          // index  - down
       56.2f,   6.9f,  22.4f,  60.0f,                  // thumb  - folded over
      WRIST_NEUTRAL, WRIST_NEUTRAL, WRIST_NEUTRAL } },

  { "flip", "middle up, everything else curled",
    { MP_NEUTRAL,   FINGER_CURL_P, FINGER_CURL_IP,    // pinky  - down
      MP_NEUTRAL,   FINGER_CURL_P, FINGER_CURL_IP,    // ring   - down
      MP_NEUTRAL,   FINGER_UP_P,   FINGER_UP_IP,      // middle - up
      MP_NEUTRAL,   FINGER_CURL_P, FINGER_CURL_IP,    // index  - down
      THUMB_CMC,    THUMB_ABD,     THUMB_MCP, THUMB_IP,
      WRIST_NEUTRAL, WRIST_NEUTRAL, WRIST_NEUTRAL } },

  // Captured on the real hand from the PC app's sliders.
  { "phone", "'call me' — thumb and pinky out, other three curled",
    { -29.9f,   0.0f,   0.0f,                          // pinky  - out, splayed away
        0.0f,  80.0f,  84.9f,                          // ring   - down
        0.0f,  80.0f,  90.9f,                          // middle - down
        0.0f,  78.9f, 106.4f,                          // index  - down
       40.3f,  -5.7f, -13.9f,   0.1f,                  // thumb  - out, straight
      WRIST_NEUTRAL, WRIST_NEUTRAL, WRIST_NEUTRAL } },
};
static const uint8_t GESTURE_COUNT = sizeof(GESTURES) / sizeof(GESTURES[0]);

static bool eqIgnoreCase(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a, cb = *b;
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return false;
    a++; b++;
  }
  return *a == *b;
}

bool applyGesture(const char* name) {
  for (uint8_t g = 0; g < GESTURE_COUNT; g++) {
    const Gesture &ges = GESTURES[g];
    if (!eqIgnoreCase(ges.name, name)) continue;

    if (!pca9685Present) {
      Serial.println(F("PCA9685 not detected, skipping."));
      return true;
    }

    uint8_t noBounds = 0;
    for (uint8_t i = 0; i < NUM_JOINTS; i++) {
      Joint &j = joints[i];
      if (j.minPulseUs == 0 && j.maxPulseUs == 0) { noBounds++; continue; }
      setJointAngleDeg((JointId)i, ges.deg[i]);
    }

    Serial.print(F("Gesture: "));
    Serial.print(ges.name);
    Serial.print(F(" — "));
    Serial.println(ges.description);
    if (noBounds > 0) {
      Serial.print(F("  "));
      Serial.print(noBounds);
      Serial.println(F(" joint(s) skipped — no bounds yet, run 'homeall' or 'manualhome'."));
    }
    return true;
  }
  return false;
}

void listGestures() {
  Serial.println(F("\n--- Gestures ---"));
  for (uint8_t g = 0; g < GESTURE_COUNT; g++) {
    Serial.print(F("  "));
    Serial.print(GESTURES[g].name);
    Serial.print(F("  - "));
    Serial.println(GESTURES[g].description);
  }
  Serial.println();
}

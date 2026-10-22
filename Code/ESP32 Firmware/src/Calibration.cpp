#include "Calibration.h"
#include "JointMap.h"
#include <Preferences.h>
#include <stdio.h>

static Preferences prefs;

// NVS keys must be <=15 chars. Longest joint name here is 5 chars
// (T_MCP/T_CMC/T_ABD), so name + a 3-char suffix fits comfortably.
static void keyFor(const char* name, const char* suffix, char* out, size_t outLen) {
  snprintf(out, outLen, "%s%s", name, suffix);
}

void calibrationLoadAll() {
  prefs.begin("calibration", true); // read-only
  char key[24];
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    // Defaults are the CURRENT in-memory values, so a joint with nothing
    // saved yet is simply left alone rather than being zeroed.
    keyFor(j.name, "_c", key, sizeof(key));
    j.centerPulseUs = prefs.getUShort(key, j.centerPulseUs);
    keyFor(j.name, "_mn", key, sizeof(key));
    j.minPulseUs = prefs.getUShort(key, j.minPulseUs);
    keyFor(j.name, "_mx", key, sizeof(key));
    j.maxPulseUs = prefs.getUShort(key, j.maxPulseUs);
    keyFor(j.name, "_fbc", key, sizeof(key));
    j.muxChannel = (int8_t)prefs.getChar(key, j.muxChannel);
    keyFor(j.name, "_fvn", key, sizeof(key));
    j.minBoundVolts = prefs.getFloat(key, j.minBoundVolts);
    keyFor(j.name, "_fvx", key, sizeof(key));
    j.maxBoundVolts = prefs.getFloat(key, j.maxBoundVolts);
  }
  prefs.end();
  Serial.println(F("Calibration loaded from flash (where present)."));
}

void calibrationSaveJoint(uint8_t i) {
  if (i >= NUM_JOINTS) return;
  Joint &j = joints[i];
  prefs.begin("calibration", false); // read-write
  char key[24];
  keyFor(j.name, "_c", key, sizeof(key));   prefs.putUShort(key, j.centerPulseUs);
  keyFor(j.name, "_mn", key, sizeof(key));  prefs.putUShort(key, j.minPulseUs);
  keyFor(j.name, "_mx", key, sizeof(key));  prefs.putUShort(key, j.maxPulseUs);
  keyFor(j.name, "_fbc", key, sizeof(key)); prefs.putChar(key, (int8_t)j.muxChannel);
  keyFor(j.name, "_fvn", key, sizeof(key)); prefs.putFloat(key, j.minBoundVolts);
  keyFor(j.name, "_fvx", key, sizeof(key)); prefs.putFloat(key, j.maxBoundVolts);
  prefs.end();
}

void calibrationSaveAll() {
  for (uint8_t i = 0; i < NUM_JOINTS; i++) calibrationSaveJoint(i);
  Serial.println(F("Calibration saved to flash."));
}

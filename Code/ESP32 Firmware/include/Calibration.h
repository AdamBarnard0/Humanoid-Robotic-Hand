#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// Persists per-joint calibration to NVS flash via the Preferences library,
// so homing and feedback-discovery results survive a reboot. Namespace
// "calibration", keys derived from each joint's name:
//   _c/_mn/_mx    center/min/max pulse widths
//   _fbc          discovered feedback mux channel
//   _fvn/_fvx     pot voltage at the min/max bound
//
// Flow:
//   boot     -> calibrationLoadAll()  (Startup.cpp)
//   homeall  -> calibrationSaveAll()  (end of homeAllJoints(), Homing.cpp)
//
// MANUAL_BOUNDS in JointMap.cpp is applied AFTER loading, so anything
// listed there still overrides whatever was in flash.
// ---------------------------------------------------------------------

// Call once at boot. Any joint with no saved entry keeps its current
// in-memory value, so this is safe on a fresh/erased flash.
void calibrationLoadAll();

void calibrationSaveJoint(uint8_t idx);  // persist one joint's center/min/max
void calibrationSaveAll();

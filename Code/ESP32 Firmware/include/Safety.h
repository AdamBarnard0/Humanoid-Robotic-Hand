#pragma once
#include <Arduino.h>
#include "JointTypes.h"

// ---------------------------------------------------------------------
// E-stop and joint protection. Settings and the full description are in
// Config.h section 12.
//
//   E-stop  — latched, whole hand. Every output stops pulsing (servos go
//             limp) and the hardware layer refuses all further writes.
//   Faults  — latched, per joint. A joint over its continuous current limit,
//             or settled too far from its commanded angle, goes limp on its
//             own; the rest of the hand keeps working.
// Both are cleared only by safetyReset().
// ---------------------------------------------------------------------

enum FaultReason : uint8_t {
  FAULT_NONE      = 0,
  FAULT_CURRENT   = 1,
  FAULT_DEVIATION = 2,
};

// ---- E-stop ---------------------------------------------------------------
void safetyEstop(const char *source);   // latch + cut every output
bool safetyEstopActive();

// Clears the E-stop and every fault. Each joint that was stopped is first
// re-commanded to where it actually IS (measured, or its last command if it
// has no usable feedback), and only then are outputs re-enabled — so
// nothing jumps back to a target it was heading for before the stop.
void safetyReset();

// ---- Faults ---------------------------------------------------------------
uint32_t    safetyFaultMask();          // bit n = joint n faulted
FaultReason safetyFaultReason(JointId id);
const char *safetyFaultName(FaultReason r);   // "current" / "deviation"

// ---- Background service ---------------------------------------------------
// Called every loop() pass, and from inside routine waits so protection keeps
// running during demos. Refreshes one joint's position filter, advances the
// current sensing, and runs both protection checks.
void safetyService();

// Pauses the protection checks (not the E-stop) for routines that drive
// joints into their stops on purpose: homing, fbfind, sweep_r, test. Nests.
void safetyPause();
void safetyResume();
struct SafetyPauseGuard {
  SafetyPauseGuard()  { safetyPause(); }
  ~SafetyPauseGuard() { safetyResume(); }
};

// ---- Serial helpers for blocking routines ---------------------------------
// The main loop catches ESTOP_CHAR as it reads bytes. Routines block that
// loop, so they call these instead.

// Services the background work and checks for an E-stop. When anyByteAborts
// is true every waiting byte is consumed and any of them asks the routine to
// stop (the demos' "send anything to abort"); when false only an ESTOP_CHAR
// at the head of the buffer is taken, and other input is left alone.
// Returns true if the routine must stop now.
bool safetyRoutinePoll(bool anyByteAborts);

// delay() replacement for routines: waits ms while polling as above.
// Returns false if the routine must stop.
bool safetyWait(uint32_t ms, bool anyByteAborts);

// Throws away everything waiting on the serial line, but still honours an
// ESTOP_CHAR anywhere in it. Returns true if anything else was discarded.
bool safetyDrainSerial();

// ---- Reporting --------------------------------------------------------------
void safetyPrintStatus();                     // terminal 'faults'
int  safetyFormatFaults(char *buf, size_t cap);   // "estop=0 P_IP=current ..."

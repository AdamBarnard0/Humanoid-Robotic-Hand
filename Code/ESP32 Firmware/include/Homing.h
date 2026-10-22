#pragma once
#include "JointTypes.h"
#include "Config.h"

// ---------------------------------------------------------------------
// Homing: drives a joint until it stalls against a mechanical limit
// (detected as a current spike), records that as one bound, then derives
// the opposite bound from a fixed per-jointType travel offset.
//
// NOT run automatically anywhere — trigger manually via 'homeall' once
// you've confirmed it's safe and someone is watching the hand.
//
// HOMING_STALL_CURRENT_MA WILL need tuning against this hand's actual
// MG996R stall current before it's trustworthy.
// ---------------------------------------------------------------------
// Tunables (stall current, step size, per-jointType travel offsets) live
// in Config.h.

// The travel offset used to derive the second bound is looked up from the
// JointId itself (homingOffsetFor() in Homing.cpp), so every caller — the
// automatic sequence, the manual one and the single-joint re-home — is
// guaranteed to use the same figure for a given joint.
void StallHome(JointId motorID, bool direction);

void homeAllJoints();         // automatic: stall-detected, no user input

// Manual equivalent of homeAllJoints(): same joint order, same starting
// point (SERVO_CENTER_US), same direction handling and same offset-derived
// second bound — but instead of watching for a current spike, the user
// jogs each joint from the terminal and types 'd' when it's at its limit.
// Unlike homeAllJoints() this also covers the wrist, which has no per-motor
// current sensing and so cannot be stall-homed.
void manualHomeAllJoints();

// Re-homes ONE joint manually, for fixing up a joint that came out wrong
// without redoing the whole hand. Identical jog behaviour to
// manualHomeAllJoints(). Saves to flash and
// parks the joint at its new centre when done.
void manualHomeSingleJoint(JointId id);

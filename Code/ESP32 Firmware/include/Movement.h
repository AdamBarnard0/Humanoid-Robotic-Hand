#pragma once
#include "JointTypes.h"

// ---------------------------------------------------------------------
// Joint-level movement. Handles per-joint invert + bounds clamping so
// callers always work in a consistent logical pulse-width space,
// regardless of how a given joint happens to be mechanically mounted.
// ---------------------------------------------------------------------

// Commands a joint to an absolute LOGICAL pulse width (us) — pre-invert,
// clamped to the joint's current min/max bounds.
void setJointPulseUs(JointId id, uint16_t us);

// Fraction of full speed this joint is allowed to move at (see Config.h
// section 4c). 1.0 for everything except the wrist axes, which are
// deliberately slowed because they swing the whole hand.
float jointSpeedScale(JointId id);

void centerJoint(JointId id);
void centerAllJoints();

// Commands a joint by angle in degrees rather than raw pulse width. 0 deg
// means: fully extended (straight) for finger IP/P joints and the thumb
// IP joint, or the neutral/center position for abduction (MP) and the
// other thumb joints (CMC/ABD/MCP) — see JOINT_ANGLE_CAL in Movement.cpp
// for the exact per-joint reference points and signed ranges. Maps
// directly onto the joint's current center/min/max bounds, whatever they
// currently hold.
void setJointAngleDeg(JointId id, float angleDeg);

// As above but immediate — no rate limiting, and currentPulseUs is synced.
// For boot, where the joint's real position isn't known until it's measured
// and a ramp would move it before it got there.
void setJointAngleDegImmediate(JointId id, float angleDeg);

// The angle this joint is being commanded to right now, recovered from the
// last pulse written to it (whichever code wrote it). Only meaningful once
// the joint has bounds.
float jointCommandedAngleDeg(JointId id);

// True if this joint's 0 deg reference is its centre of travel (abduction
// joints, thumb CMC/ABD/MCP); false if 0 deg is its min bound, i.e. fully
// extended (finger P/IP joints, thumb IP).
bool jointZeroIsCenter(JointId id);

// The commandable angular range for this joint: 0..romDeg for joints whose
// zero is the min bound, or -negMaxDeg..+posMaxDeg for centre-referenced
// ones. Angles outside this are clamped by setJointAngleDeg().
void jointAngleLimits(JointId id, float &minDeg, float &maxDeg);

// Moves every attached joint to its 0 degree position (see
// setJointAngleDeg()). This is NOT the same as centerAllJoints(), which
// just sends every joint to the middle of its raw pulse-width range for
// assembly purposes — zeroAllJoints() sends each joint to its actual
// anatomical zero (straight fingers, thumb at rest), which for several
// joints is NOT the middle of the pulse-width range.
void zeroAllJoints();

// Identification sweep: moves one joint +/-SWEEP_RANGE_US either side of
// neutral and back, so you can see which motor drives which joint after an
// assembly. Deliberately NOT the full travel — it works before homing and
// won't drive a short-throw joint into its stop. For a full-travel check
// use 'test' (Feedback.h), which requires homed bounds.
void sweepJoint(JointId id);

// Full range-of-motion sweep: glides the joint through its entire homed
// angular range (0 -> min -> max -> 0) and reports commanded against
// measured at each extreme. Needs bounds, and drives by ANGLE, so it
// exercises the ROM figures and centre placement as well as the mechanism.
void sweepJointRange(JointId id);

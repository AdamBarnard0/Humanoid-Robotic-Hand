#pragma once
#include "JointTypes.h"

// ---------------------------------------------------------------------
// Baked-in hand poses, each defined as a full set of per-joint angles in
// degrees. Angles follow the same conventions as setJointAngleDeg() — see
// JOINT_ANGLE_CAL in Movement.cpp for each joint's zero reference and
// range — so a pose stays valid across re-homing.
//
// There is no motion profile: every joint is commanded at once and the
// hand snaps to the pose. Smooth transitions belong on the PC side later.
// ---------------------------------------------------------------------

// Applies a gesture by name (case-insensitive). Returns false if the name
// isn't recognised.
bool applyGesture(const char* name);

// Prints the available gesture names + descriptions.
void listGestures();

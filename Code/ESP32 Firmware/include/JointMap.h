#pragma once
#include "JointTypes.h"

// The single source of truth for joint wiring + bounds + live state.
// Defined in JointMap.cpp — that's the file to edit after every assembly.
extern Joint joints[NUM_JOINTS];

// Case-insensitive lookup by name (e.g. "p_ip" or "P_IP"). Returns
// NUM_JOINTS if no joint matches.
JointId findJointByName(const char* name);

// Lookup by PCA9685 channel number (0-15). Returns NUM_JOINTS if no joint
// is wired to that channel.
JointId findJointByChannel(uint8_t pcaChannel);

// Applies the MANUAL_BOUNDS table in JointMap.cpp, if any entries are
// present. Called at boot from Startup.cpp — this is the place to hard-set
// a joint's center/min/max if you've determined good values by hand and
// don't want to re-run homing every boot.
void applyManualBoundsOverride();

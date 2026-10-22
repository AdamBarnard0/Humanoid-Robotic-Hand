#pragma once
#include <Arduino.h>
#include "JointTypes.h"

// ---------------------------------------------------------------------
// Current sensing via the Servo Click's LTC2497, used by homing to detect
// stalls. Non-blocking: issues at most one I2C transaction per call, gated
// by LTC2497_CONV_MS.
//
// The LTC2497 is pipelined — each transaction selects a channel for the
// NEXT conversion and returns the result of whatever was selected on the
// PREVIOUS one. So the first call after switching joints is a throwaway
// priming read; every call after that updates joints[id].currentMA every
// ~LTC2497_CONV_MS.
//
// There is only one conversion pipeline on the chip, so only one joint may
// be polled at a time — call this for a single joint until you're done
// with it before moving to the next.
// ---------------------------------------------------------------------
void currentMonitorSingleStep(JointId id);

// Round-robin variant: advances one channel per call, so every joint's
// currentMA refreshes once per NUM_JOINTS * LTC2497_CONV_MS (~2.6s). Used
// in PC mode, where telemetry needs a value for all 16 joints rather than
// a fast reading for one.
//
// Shares the same pipeline state as currentMonitorSingleStep(), so only
// one of the two may be driven at a time — homing uses the single-channel
// version and never runs while PC mode is active.
void currentMonitorStep();

// One INA219 read, copied into the three wrist joints' currentMA. Called
// once per S reply.
void currentMonitorSampleRail();

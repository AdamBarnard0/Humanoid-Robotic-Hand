#pragma once
#include "JointTypes.h"
#include "Config.h"

// ---------------------------------------------------------------------
// Analogue joint feedback: potentiometer per joint, read through the
// CD74HC4067 mux.
//
// A joint's motor channel and its feedback channel are unrelated, and the
// feedback wiring isn't recorded anywhere — so the mapping is DISCOVERED
// at runtime: jog one joint at a time and watch which mux channel moves.
//
// Typical order of operations after an assembly:
//   homeall / manualhome   -> establishes pulse-width bounds
//   fbfind                 -> works out which mux channel belongs to which
//                             joint AND calibrates the voltage at each bound
//   fb                     -> live readout
// ---------------------------------------------------------------------

// Tunables (detection threshold, sample count, settle time) live in Config.h.

// Sweeps each joint across its FULL travel (min bound to max bound) in
// turn and assigns the mux channel that responds. Because the sweep sits
// at both bounds anyway, the bound voltages are captured at the same time
// — discovery and calibration in a single pass, one visit per joint.
// Joints whose motor never produces a conclusive response (e.g. a faulty
// servo) are left unassigned rather than being given a wrong channel.
void discoverFeedbackChannels();

// Discovery and calibration are one operation — there is no separate
// calibrate step. Re-run discoverFeedbackChannels() after any re-homing.

// ONE-OFF synchronous read of one joint's pot (routines and diagnostics
// that need a fresh measurement now); also re-seeds that joint's filter.
// Updates lastFeedbackVolts + currentAngleDeg.
// Returns the measured angle in degrees. Joints with no assigned channel
// or no voltage calibration are left untouched. Fewer samples trade noise
// rejection for speed.
float readJointAngleDeg(JointId id, uint8_t samples = FEEDBACK_SAMPLES);

// Background sampler: reads ONE joint's pot and updates its low-pass
// filter. Call every loop() pass; cycles through all joints round-robin.
void feedbackServiceTick();

// Refreshes currentAngleDeg for every joint that has a usable channel from
// its FILTERED reading, and returns a bitmask (bit n = joint n) of the
// joints whose reading was trustworthy — the 'fb' field of the S reply.
// Does no ADC reads unless a joint's filter has gone stale.
uint32_t updateAllJointAngles();

// The background filter's current angle for one joint, WITHOUT any ADC read
// and without touching the joint table. False if the joint has no trusted
// reading, or the filter hasn't been updated within maxAgeMs. Used by the
// protection checks, which run every loop pass and must stay cheap.
bool feedbackFilteredAngle(JointId id, uint32_t maxAgeMs, float &deg);

// True if this joint's feedback can be believed right now: homed, channel
// assigned, bound voltages calibrated, and the live reading sitting inside
// that calibrated band.
bool jointFeedbackTrustworthy(JointId id);

// Live readout: channel, voltage, measured angle vs commanded pulse.
void printJointFeedback();

// Connectivity test: centres everything, then sweeps each joint on its own
// and watches its feedback. Proves every motor moves and every pot is
// wired, in one pass. Send any character to abort.
void runConnectivityTest();

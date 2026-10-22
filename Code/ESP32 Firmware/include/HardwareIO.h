#pragma once
#include <Arduino.h>
#include "JointTypes.h"

// ---------------------------------------------------------------------
// Low-level, channel-indexed hardware primitives. No knowledge of joints
// — callers pass raw PCA9685/LTC2497 channel numbers (0-15). Joint-aware
// code lives in Movement.cpp / CurrentMonitor.cpp.
// ---------------------------------------------------------------------

// True once the I2C scan in hwI2cScan() has confirmed the PCA9685 acks.
extern bool pca9685Present;

// LTC2497 single-ended channel-select command bytes, indexed 0-15. Same
// values used by MikroE's official Servo Click driver.
extern const uint8_t LTC2497_CH_CMD[16];

void hwInitPins();                 // PCA9685 output-enable pin mode/level
bool hwInitPCA9685();              // begin() the PCA9685 driver (only meaningful if pca9685Present)

void hwSetChannelPulseUs(uint8_t pcaChannel, uint16_t us);

// Cuts the PWM pulse train to a single channel using the PCA9685's
// per-channel FULL_OFF bit. A hobby servo with no pulses stops driving and
// goes limp — every other channel is unaffected. Undo by simply writing a
// normal pulse to the channel again with hwSetChannelPulseUs().
void hwDisableChannel(uint8_t pcaChannel);

// --- Joint-aware output: routes to the PCA9685 or straight to LEDC ---
// Everything above the hardware layer should use these rather than the
// channel-level calls, so a joint's driver stays an implementation detail.
void hwSetJointPulseUs(const Joint &j, uint16_t us);
void hwDisableJoint(const Joint &j);

// --- Output lock (E-stop / protection faults) — driven by Safety.cpp ---
// While locked, hwSetJointPulseUs() drops every write, so nothing above this
// layer can move a stopped motor.
void hwOutputsLockAll();             // OE pin high + wrist duty 0 + all PCA FULL_OFF
void hwOutputsUnlockAll();           // clears both locks; OE pin left as it is
void hwOutputEnablePin(bool enable); // PCA9685 /OE: true = outputs on
void hwLockJoint(uint8_t idx);       // one joint limp and locked (fault)
bool hwOutputsLocked();

// --- Feedback pot mux (CD74HC4067) ---
void  hwMuxSelect(uint8_t muxChannel);
int   hwReadMuxRaw(uint8_t muxChannel);            // 0-4095, single sample
float hwReadMuxVolts(uint8_t muxChannel, uint8_t samples); // averaged, in volts
float hwRawToVolts(int raw);

// Averaged read straight off an ADC1 GPIO — used by the wrist joints, whose
// pots bypass the mux entirely.
float hwReadAdcVolts(uint8_t gpio, uint8_t samples);

// Reads whichever source this joint uses (mux or direct pin).
float hwReadJointVolts(const Joint &j, uint8_t samples);

// Low-level pipelined LTC2497 transaction: writes 'cmd' (selects a channel
// for the NEXT conversion) and reads back the 3-byte result of whatever
// channel was selected on the PREVIOUS transaction. Retries a couple of
// times internally (see HardwareIO.cpp) since isolated read failures are a
// known, occasional ESP32 driver quirk. Returns LTC2497_READ_FAILED if all
// attempts fail — callers must not treat that as a real reading of 0.
#define LTC2497_READ_FAILED  0xFFFFFFFFUL
uint32_t hwLtc2497Transact(uint8_t cmd);

// --- INA219: TOTAL current through the whole motor rail ---
// Not per-motor. Reads the shunt-voltage register directly and divides by
// the known shunt resistance, which sidesteps the calibration register (and
// its LSB maths) entirely. Returns NAN if the sensor doesn't respond.
extern bool ina219Present;
bool  hwInitIna219();      // probes the sensor, sets ina219Present
float hwReadRailMilliamps();

// Scans the I2C bus 0x08-0x77, prints results to Serial, updates and
// returns pca9685Present.
bool hwI2cScan();

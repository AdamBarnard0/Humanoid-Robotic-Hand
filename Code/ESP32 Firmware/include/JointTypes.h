#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// Joint identity + naming convention
// ---------------------------------------------------------------------
// Each of the 4 fingers (Pinky/Ring/Middle/Index) has 3 independently
// controlled DOF:
//   MP  - metacarpophalangeal joint (abduction / side-to-side spread)
//   P   - proximal phalanx flexion
//   IP  - interphalangeal flexion of the fused distal+intermediate phalange
//
// The thumb has 4 DOF, named to match the ORCA hand convention:
//   CMC - carpometacarpal flexion (the thumb's base opposition swing)
//   ABD - abduction / rotation at the base
//   MCP - metacarpophalangeal flexion
//   IP  - interphalangeal flexion
//
// Naming = <finger prefix>_<joint>, e.g. P_IP, R_MP, T_CMC.
//   P = Pinky, R = Ring, M = Middle, I = Index, T = Thumb
//
// The wrist adds 3 more DOF (pitch/roll/yaw). They are named generically
// W_1..W_3 because which axis sits on which motor is not fixed — rename
// them here once the mechanics are confirmed.
//
// Array order below (and in Config.h's JOINT_WIRING_TABLE) IS the index
// used everywhere else — entries must stay in this order.
// ---------------------------------------------------------------------
enum JointId : uint8_t {
  P_MP = 0, P_P, P_IP,            // Pinky
  R_MP, R_P, R_IP,                // Ring
  M_MP, M_P, M_IP,                // Middle
  I_MP, I_P, I_IP,                // Index
  T_CMC, T_ABD, T_MCP, T_IP,      // Thumb (4 DOF, ORCA-style naming)
  W_1, W_2, W_3,                  // Wrist (3 DOF, driven straight off the ESP32)
  NUM_JOINTS
};

// The 16 finger/thumb joints hang off the Servo Click; the wrist motors are
// wired straight to the ESP32. Everything above this index is PCA-driven.
#define NUM_PCA_JOINTS  16

// How a joint's PWM is produced.
enum JointDriver : uint8_t {
  DRV_PCA9685 = 0,   // Servo Click channel, current-sensed by the LTC2497
  DRV_LEDC,          // ESP32 GPIO via the LEDC peripheral, no per-motor sensing
};

// Where a joint's position feedback comes from.
enum FeedbackSrc : uint8_t {
  FB_MUX = 0,        // through the CD74HC4067; channel discovered by 'fbfind'
  FB_ADC,            // straight off a dedicated ADC1 pin, fixed by wiring
};

// Per-joint configuration + live state. One instance per joint, indexed by
// JointId via the global joints[] array declared in JointMap.h.
struct Joint {
  const char* name;             // human-readable id, e.g. "P_IP"

  // --- Wiring: set per assembly in Config.h (changes every rebuild) ---
  JointDriver driver;           // how this joint's PWM is generated
  uint8_t pcaChannel;           // DRV_PCA9685: PCA9685 / LTC2497 channel (0-15)
  uint8_t pwmPin;               // DRV_LEDC: ESP32 GPIO carrying the pulse
  uint8_t ledcCh;               // DRV_LEDC: LEDC channel, assigned at init
  bool    invert;               // true = reverse commanded direction for this joint's mounting
  bool    attached;             // false = motor not physically connected yet, skip it

  // --- Travel bounds: set by homing, or hard-set via MANUAL_BOUNDS ---
  uint16_t centerPulseUs;       // neutral/center position, pre-invert (logical) pulse width
  uint16_t minPulseUs;          // lower travel bound, logical pulse width
  uint16_t maxPulseUs;          // upper travel bound, logical pulse width

  // --- Feedback pot: calibrated by 'fbfind' ---
  FeedbackSrc fbSrc;            // muxed (channel discovered) or direct ADC pin
  int8_t   muxChannel;          // FB_MUX: CD74HC4067 channel (0-15), -1 = not assigned
  uint8_t  adcPin;              // FB_ADC: ESP32 ADC1 GPIO, fixed by wiring
  float    minBoundVolts;       // pot voltage measured at minPulseUs
  float    maxBoundVolts;       // pot voltage measured at maxPulseUs

  // --- PC control mode: see Protocol.cpp ---
  float    targetAngleDeg;      // where the PC wants this joint (already clamped to ROM)
  float    targetSpeedDegSec;   // how fast to get there (already clamped)
  float    commandedAngleDeg;   // the slew-limited setpoint actually being driven

  // --- Live state ---
  uint16_t currentPulseUs;      // last pulse width actually sent to the PCA9685 (post-invert)
  float    currentMA;           // live current draw
  float    lastFeedbackVolts;   // most recent pot reading
  float    currentAngleDeg;     // MEASURED position in degrees, from the pot (for the PC link)
};

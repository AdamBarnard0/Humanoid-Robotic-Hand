#pragma once
#include <Arduino.h>

// ===========================================================================
//                        HRH1 — ALL TUNABLE PARAMETERS
//
// Every number you might want to change lives in this file. Nothing else in
// the firmware should contain a magic constant; if you find one, move it
// here.
//
// Sections:
//   1. Serial / I2C
//   2. Hardware addresses + pin assignments
//   3. Per-joint wiring          <- EDIT AFTER EVERY ASSEMBLY
//   4. Servo travel + timing
//   5. Current sensing (LTC2497)
//   6. Homing
//   7. Feedback pots
//   8. Joint range of motion (degrees)
//   9. Gesture poses
// ===========================================================================


// ---------------------------------------------------------------------
// 1. SERIAL / I2C
// ---------------------------------------------------------------------
// 921600 baud. Protocol v3 at 30 Hz needs ~11 kB/s, about 12% of the link,
// so there's ample headroom. Set your serial monitor and the PC client to
// match.
#define SERIAL_BAUD    921600

// Only used by serialWaitForLine() (interactive prompts inside routines).
// The main loop's line reader is non-blocking and doesn't use it.
#define SERIAL_READ_TIMEOUT_MS  10

// UART driver buffers, set before Serial.begin(). The RX side holds several
// frames if the loop is briefly busy; the TX side lets a ~240-byte S reply
// be queued without blocking the loop while it drains (without it, write()
// stalls until the bytes fit the 128-byte hardware FIFO).
#define SERIAL_RX_BUFFER   1024
#define SERIAL_TX_BUFFER    512

#define I2C_SDA        21
#define I2C_SCL        22
// ESP32 Arduino core defaults to 100kHz if unset. Lower = more noise margin
// but slower transactions; raise again if the bus is clean and homing feels
// sluggish.
#define I2C_CLOCK_HZ   400000

// Upper bound on any single I2C transaction. The core's default is 50 ms,
// so one wedged transaction could stall the whole loop for that long. Every
// device here completes in well under 1 ms.
#define I2C_TIMEOUT_MS  10


// ---------------------------------------------------------------------
// 2. HARDWARE ADDRESSES + PINS
// ---------------------------------------------------------------------
#define PCA9685_ADDR          0x40
#define PCA9685_OE_PIN        19     // LOW = outputs enabled, HIGH = disabled
#define SERVO_CLICK_ADC_ADDR  0x16   // Servo Click onboard LTC2497

// INA219 on the shared I2C bus, measuring TOTAL current through the whole
// motor rail (not per-motor). A0+A1 tied to VCC gives 0x45 — must be set
// explicitly, most libraries default to 0x40 and silently find nothing.
#define INA219_ADDR           0x45
#define INA219_SHUNT_OHMS     0.1f   // breakout board shunt resistor

// CD74HC4067 analogue mux (feedback pots). Per the datasheet S0 is the LSB
// and E is active LOW — both already handled in HardwareIO.cpp.
#define MUX_S0         13
#define MUX_S1         14
#define MUX_S2         27
#define MUX_S3         26
#define MUX_EN         25
#define MUX_SIG        34            // ADC1_CH6, input-only
#define MUX_EN_ACTIVE_LOW true
#define MUX_SETTLE_US  50            // settle time after switching channel

// ESP32 ADC. 12-bit resolution and 11dB attenuation are set in hwInitPins();
// change these together if you change the attenuation.
#define ADC_MAX_COUNTS   4095
#define ADC_VREF_VOLTS   3.3f


// ---------------------------------------------------------------------
// 3. PER-JOINT WIRING — EDIT THIS AFTER EVERY ASSEMBLY
//
// Two kinds of joint, two macros:
//
//   XP(name, pcaChannel, invert)        driven by the Servo Click PCA9685
//   XW(name, pwmPin, adcPin, invert)    driven directly by the ESP32
//
//   pcaChannel : PCA9685 channel (0-15) the motor is plugged into. Doubles
//                as the LTC2497 current-sense channel (same header).
//   pwmPin     : ESP32 GPIO producing the servo pulse via LEDC.
//   adcPin     : ESP32 GPIO reading the feedback pot directly. MUST be an
//                ADC1 pin (32-39) — ADC2 is unusable while the radio is on.
//   invert     : true to reverse commanded direction for this joint's
//                mechanical mounting. Determine empirically — command a
//                small angle and see which way it goes.
//
// Entries MUST stay in JointId order (see JointTypes.h).
//
// PCA joints find their feedback pot at runtime via 'fbfind' (mux channel
// is not fixed by wiring). Wrist joints read a dedicated ADC pin, so their
// feedback channel IS fixed here and needs no discovery.
// ---------------------------------------------------------------------
#define JOINT_WIRING_TABLE                                                    \
  XP("P_MP",   5, true )    /* Pinky  - MCP abduction                     */  \
  XP("P_P",   8, false )    /* Pinky  - proximal flexion                  */  \
  XP("P_IP",   11, false )    /* Pinky  - IP flexion (fused distal/interm.) */  \
                                                                              \
  XP("R_MP",  9, false)    /* Ring   - MCP abduction                     */  \
  XP("R_P",    1, false)    /* Ring   - proximal flexion                  */  \
  XP("R_IP",  6, false)    /* Ring   - IP flexion                        */  \
                                                                              \
  XP("M_MP",  4, true)    /* Middle - MCP abduction                     */  \
  XP("M_P",    7, false )    /* Middle - proximal flexion                  */  \
  XP("M_IP",   0,  false )    /* Middle - IP flexion                        */  \
                                                                              \
  XP("I_MP",  2, true)    /* Index  - MCP abduction                     */  \
  XP("I_P",    14, false)    /* Index  - proximal flexion                  */  \
  XP("I_IP",  13, false)    /* Index  - IP flexion                        */  \
                                                                              \
  XP("T_CMC",  12,  false )    /* Thumb  - CMC flexion (opposition swing)    */  \
  XP("T_ABD",  10, true )    /* Thumb  - abduction/rotation at the base    */  \
  XP("T_MCP",  15, false)    /* Thumb  - MCP flexion                       */  \
  XP("T_IP",   3, false)    /* Thumb  - IP flexion                        */  \
                                                                              \
  XW("W_1",   16, 35, false) /* roll  Wrist - GPIO16 PWM (RX2), GPIO35 feedback */  \
  XW("W_2",   17, 33, false) /* pitch Wrist - GPIO17 PWM (TX2), GPIO33 feedback */  \
  XW("W_3",    4, 32, false) /* yaw   Wrist - GPIO4  PWM,       GPIO32 feedback */

// NOTE: W_2 uses a DIFFERENT servo control board from W_1/W_3. An earlier
// unit latched its driver off under load — powered, pot still reading, but
// limp and back-drivable, and only a motor-rail power cycle cleared it.
// Replacing it fixed the fault, but don't assume the three wrist servos are
// interchangeable: if W_2 ever needs a different pulse range, neutral or
// frame rate from the other two, SERVO_MIN_US / SERVO_MAX_US /
// SERVO_CENTER_US / LEDC_FREQ_HZ are GLOBAL and would have to be made
// per-joint first. The ROM and homing-offset constants are already
// per-axis, so those are safe to tune independently.


// ---------------------------------------------------------------------
// 4. SERVO TRAVEL + TIMING
//
// SERVO_MIN_US/SERVO_MAX_US are the safety envelope for pulse widths the
// firmware will generate.
//
// MEASURED, not assumed: on a scope these servos respond from roughly
// 400us to 2600us, which is just over their rated 180 deg. The 1000-2000us
// range normally quoted for hobby servos only exercised about 110 deg of
// that. 600-2400us is set deliberately inside the measured limits so the
// horn is never commanded against its own internal end stops.
//
// 1500us remains the midpoint of this range as well as the servo's own
// mechanical neutral, so SERVO_CENTER_US is unchanged.
//
// NOTE: these bound homing searches and manual jogging, but per-joint
// bounds found by homing are NOT re-clamped to them afterwards — see
// setJointPulseUs(), which clamps to the joint's own min/max only.
// ---------------------------------------------------------------------
#define SERVO_FREQ_HZ    50
#define SERVO_MIN_US     600
#define SERVO_MAX_US     2400
#define SERVO_CENTER_US  1500        // servo's mechanical neutral, used by 'center'
#define MOVE_SETTLE_MS   400         // time allowed for a servo to arrive

// LEDC (ESP32 hardware PWM) for the directly-driven wrist motors. 16-bit
// resolution at 50Hz gives ~0.3us of pulse-width precision — far finer than
// the servos can resolve.
#define LEDC_FREQ_HZ     SERVO_FREQ_HZ
#define LEDC_RES_BITS    16
#define LEDC_FIRST_CH    0           // LEDC channels are allocated sequentially
                                     // from here, one per direct-driven joint


// Identification sweep ('sweep <j>'). A small fixed excursion either side
// of neutral, NOT the joint's full travel — this is used after assembly to
// see which motor moves which joint, typically before anything is homed.
// Small enough that an abduction joint won't hit its stop.
#define SWEEP_RANGE_US   150

// Full range-of-motion sweep ('sweep_r'). Steps smoothly through the joint's
// whole homed travel rather than jumping between the bounds, so you can
// watch for binding and see where it actually stops.
#define SWEEP_R_STEPS      40      // interpolation points per leg
#define SWEEP_R_STEP_MS    25      // dwell per point -> ~1s per leg
#define SWEEP_R_HOLD_MS    500     // pause at each extreme


// ---------------------------------------------------------------------
// 4c. WRIST SPEED LIMIT
//
// The wrist carries the whole hand, so at full servo speed it snaps around
// alarmingly. These scale each axis's maximum slew rate as a fraction of
// full speed: 1.0 = unrestricted, 0.6 = 60%.
//
// MUST be float — an integer type would truncate 0.6 to 0 and the joint
// would never move at all.
//
// Applied in both directions of travel:
//   * terminal commands, gestures and demos ramp the setpoint in steps
//   * PC mode clamps the deg/s the PC asks for (see Protocol.cpp)
// ---------------------------------------------------------------------
#define WRIST_SPEED_W_1   0.6f      // roll
#define WRIST_SPEED_W_2   0.6f      // pitch
#define WRIST_SPEED_W_3   0.6f      // yaw

// Pulse-width slew that counts as 100% speed, and how often the ramp
// updates. 3000us/s crosses the full 1800us envelope in about 0.6s, which
// is roughly what these servos manage unloaded.
#define WRIST_FULL_RATE_US_PER_S  3000.0f
#define WRIST_RAMP_TICK_MS        10


// ---------------------------------------------------------------------
// 4b. CONNECTIVITY TEST ('test')
// ---------------------------------------------------------------------
#define TEST_SETTLE_MS      450      // time at each end of the sweep
#define TEST_MIN_SWING_V    0.30f    // feedback swing below this = FAIL


// ---------------------------------------------------------------------
// 5. CURRENT SENSING (LTC2497 on the Servo Click)
// ---------------------------------------------------------------------
#define LTC2497_VREF_MV       3300   // matches the Servo Click's VCC SEL jumper
#define LTC2497_CONV_MS       160    // datasheet conversion time (~149ms) + margin
#define LTC2497_MAX_ATTEMPTS  3      // retries for the ESP32 repeated-start driver bug


// ---------------------------------------------------------------------
// 6. HOMING
//
// A joint is driven until its current spikes (stall), which fixes ONE
// bound; the opposite bound is derived by adding/subtracting the travel
// offset for that joint type below. Tune the offsets against real measured
// travel — they're what sets each joint's range of motion in microseconds.
// ---------------------------------------------------------------------
#define HOMING_STALL_CURRENT_MA   80.0f   // TODO: tune against real stall current
#define HOMING_STEP_US            6       // pulse-width increment per step
#define HOMING_STEP_DELAY_MS      40      // dwell between steps
#define HOMING_SLOW_STEP_US       2       // reduced step after hitting the safety limit

// Travel span per joint type, in microseconds.
// Abduction travel is now different for every finger. These are ESTIMATES
// scaled from the old figure of 390us over a 40 deg range, i.e. 9.75us per
// degree, applied to each finger's own total ROM. That assumes all four
// abduction joints share a pulley ratio — measure and replace these once
// the new hand has been homed for real.
#define HOMING_OFFSET_I_MP        429     // 44 deg total (24 + 20)
#define HOMING_OFFSET_M_MP        468     // 48 deg total (24 + 24)
#define HOMING_OFFSET_R_MP        429     // 44 deg total (20 + 24)
#define HOMING_OFFSET_P_MP        439     // 45 deg total (10 + 35)
#define HOMING_OFFSET_FINGER_IP   750     // type 1
#define HOMING_OFFSET_FINGER_P    800     // type 2
#define HOMING_OFFSET_T_MCP       1450    // type 3
#define HOMING_OFFSET_T_CMC       900     // type 4
#define HOMING_OFFSET_T_ABD      900     // type 5
#define HOMING_OFFSET_T_IP        1806
#define HOMING_OFFSET_W_1         1000     // type 7 — roll
#define HOMING_OFFSET_W_2         1100     // type 8 — pitch
#define HOMING_OFFSET_W_3         460     // type 9 — yaw

// Step size used by 'manualhome' if the user just presses enter.
#define MANUAL_HOME_DEFAULT_STEP  10


// ---------------------------------------------------------------------
// 7. FEEDBACK POTS
// ---------------------------------------------------------------------
#define FEEDBACK_MIN_DELTA_V   0.30f  // change that counts as "this joint moved it"
#define FEEDBACK_SAMPLES       8      // samples per one-off read (discovery, calibration,
                                      // boot hold, sweep_r) — trimmed mean, see below

// Background sampling. Every loop() pass reads ONE joint's pot (a small
// batch, trimmed mean) and folds it into that joint's low-pass filter, round-
// robin over all 19. The PC reply then just reports the filtered values with
// no ADC reads of its own — instant, and far quieter than the two back-to-
// back samples v3.0 took per reply.
//
// A batch is sorted and the top and bottom quarter discarded before
// averaging, so a single spike can't drag the result.
#define FEEDBACK_BG_BATCH     20      // samples per background read — sorted, the
                                      // top and bottom 5 dropped, middle 10 averaged.
                                      // Each sample costs ~30 us, so one joint's read
                                      // takes ~650 us and a full 19-joint sweep ~12 ms
                                      // (each joint refreshed ~80 times a second).
#define FEEDBACK_MAX_BATCH    32      // hard cap on any batch size (sets the stack buffer)

// Low-pass time constant. Larger = smoother but the reported angle lags the
// real one by about this much. 20 ms is under one 30 Hz frame. The filter
// uses the real time between updates, so it behaves the same however fast
// the loop happens to run.
#define FEEDBACK_FILTER_TAU_MS 20.0f

// A joint's filter older than this is re-read synchronously before use.
// Blocking routines (homing, demos, sweeps) stop the background sampler, so
// straight after one every filter is stale — this refreshes them instead of
// reporting where the hand was before the routine started.
#define FEEDBACK_STALE_MS      50
#define FEEDBACK_SETTLE_MS     400    // time allowed for a servo to arrive
#define FEEDBACK_MIN_SPAN_V    0.05f  // below this, volts->degrees is not resolvable

// How far outside its calibrated band a reading may sit and still be
// believed, as a fraction of the band. Covers normal overshoot past a homed
// bound; anything wilder (an unpowered pot reading ~0V) is rejected.
#define FEEDBACK_TRUST_MARGIN  0.15f


// ---------------------------------------------------------------------
// 8. JOINT RANGE OF MOTION (degrees)
//
// Feeds JOINT_ANGLE_CAL in Movement.cpp. Two conventions:
//   - finger P/IP: 0 deg = fully extended, ROM is one-directional
//   - abduction + thumb CMC/ABD/MCP: 0 deg = neutral, signed range
// ---------------------------------------------------------------------
#define ROM_FINGER_IP_DEG      116.0f
#define ROM_FINGER_P_DEG        90.0f
// Finger abduction is PER-FINGER and ASYMMETRIC in the new mechanism.
// Positive is the direction the joint homes in (to the right, viewed from
// the palm side); negative is to the left. 0 deg is the finger straight up.
#define ROM_I_MP_POS_DEG        24.0f
#define ROM_I_MP_NEG_DEG        20.0f
#define ROM_M_MP_POS_DEG        24.0f
#define ROM_M_MP_NEG_DEG        24.0f
#define ROM_R_MP_POS_DEG        20.0f
#define ROM_R_MP_NEG_DEG        24.0f
#define ROM_P_MP_POS_DEG         3.0f
#define ROM_P_MP_NEG_DEG        35.0f

#define ROM_T_CMC_POS_DEG       82.0f
#define ROM_T_CMC_NEG_DEG       37.0f   // backwards
// T_ABD is NOT symmetric — the CAD gives +70/-32, and unlike T_CMC/T_MCP/T_IP
// this is a genuine range difference rather than a mirrored sign convention.
#define ROM_T_ABD_POS_DEG       70.0f
#define ROM_T_ABD_NEG_DEG       32.0f
#define ROM_T_MCP_POS_DEG       95.0f   // closing
#define ROM_T_MCP_NEG_DEG       65.0f   // backwards
// T_IP is now centre-referenced like the rest of the thumb: it still drives
// the same distance forward into flexion, but can also come back past 0 deg
// into extension.
#define ROM_T_IP_POS_DEG        98.0f   // forward / flexion
#define ROM_T_IP_NEG_DEG        63.0f   // back past 0 / extension

// Wrist — all three are centre-referenced (0 deg = neutral) with symmetric
// travel. From the CAD/URDF export, no longer placeholders.
//
// These were previously rotated by one position: the original placeholders
// were given as "pitch 45, yaw 25, roll 90" and got assigned to W_1/W_2/W_3
// in that order, but W_1 is ROLL, not pitch. The CAD's roll figure is 90,
// which is what the old W_3 constant was holding.
#define ROM_WRIST_1_DEG         90.0f   // roll  — rotation about the forearm
#define ROM_WRIST_2_DEG         55.0f   // pitch — flexion / extension
#define ROM_WRIST_3_DEG         16.5f   // yaw   — radial / ulnar deviation


// ---------------------------------------------------------------------
// 9. GESTURE POSES (degrees, feeds the table in Gestures.cpp)
//
// Curl values sit short of full ROM so a folded finger isn't driven hard
// into its mechanical stop and left stalling there.
// ---------------------------------------------------------------------
#define GEST_FINGER_UP_P      0.0f
#define GEST_FINGER_UP_IP     0.0f
#define GEST_FINGER_CURL_P   90.0f    // ROM is ROM_FINGER_P_DEG
#define GEST_FINGER_CURL_IP 105.0f    // ROM is ROM_FINGER_IP_DEG
#define GEST_MP_NEUTRAL       0.0f

// Thumb folded across the palm — shared by all gestures.
#define GEST_THUMB_CMC       60.0f
#define GEST_THUMB_ABD        0.0f
#define GEST_THUMB_MCP       70.0f
#define GEST_THUMB_IP        60.0f

// Peace-sign spread. If index and middle CROSS instead of splaying apart,
// negate both of these.
#define GEST_SPLAY_INDEX     15.0f
#define GEST_SPLAY_MIDDLE   -15.0f


// ---------------------------------------------------------------------
// 10. MOVEMENT DEMO ('demo')
// ---------------------------------------------------------------------
#define DEMO_STEP_MS        500       // pause between poses within a section
#define DEMO_HOLD_MS        800       // pause at zero between sections
#define DEMO_WAVE_MS        180       // gap between fingers in a rippling wave
#define DEMO_CLOSE_WAVE_MS  260       // gap between fingers when sweeping closed
                                      // (longer: a full curl is a bigger move
                                      // than an abduction flick)
#define DEMO_ABD_WAVE_PASSES 4        // side-to-side passes in the abduction wave
// Splay amplitude, as a FRACTION of each finger's own travel rather than one
// shared angle in degrees.
//
// The four abduction joints are all different and all asymmetric — the pinky
// runs -35/+3, the middle -24/+24 — so a single shared figure is limited by
// the most restricted joint in its most restricted direction, which means +3
// deg for the whole hand and a wave you can barely see. Driving each finger
// to the same fraction of its OWN limit uses the full mechanism and looks
// like a hand splaying rather than twitching.
//
// Short of 1.0 so a servo is never left sitting against its mechanical stop.
#define DEMO_ABD_FRACTION    0.85f
#define DEMO_CURL_P_DEG     GEST_FINGER_CURL_P
#define DEMO_CURL_IP_DEG    GEST_FINGER_CURL_IP

// Index-finger circle at the end of the demo. Horizontal motion comes from
// the MP (abduction) joint, vertical from the P joint, driven 90 deg out of
// phase so the fingertip traces a loop.
#define DEMO_PIP_REPS       1         // repeats of the P/IP wave

// Thumb parked clear of the palm so the fingers can close, and so the index
// can point without fouling it. Reaching poses elsewhere use POSITIVE T_ABD
// to come across the palm, so negative swings away — if the thumb moves the
// wrong way here, negate DEMO_THUMB_CLEAR_ABD.
#define DEMO_THUMB_CLEAR_CMC    0.0f
#define DEMO_THUMB_CLEAR_ABD  -32.0f   // was -40, beyond the CAD limit of -32
#define DEMO_THUMB_CLEAR_MCP    0.0f
#define DEMO_THUMB_CLEAR_IP     0.0f

#define DEMO_CIRCLE_REVS    2
#define DEMO_CIRCLE_STEPS   24        // points per revolution
#define DEMO_CIRCLE_MS      45        // dwell per point
// Deliberately well under the MP joint's +-20 deg ROM: the index is circling
// while the other fingers are still closed in a fist, and a full-amplitude
// swing catches the middle finger.
#define DEMO_CIRCLE_MP_AMP  10.0f     // horizontal half-amplitude
#define DEMO_CIRCLE_P_MID   25.0f     // P flexion at the centre of the circle
#define DEMO_CIRCLE_P_AMP   18.0f     // vertical half-amplitude

// --- demo2: thumb-to-fingertip touches -------------------------------
// Transitions are interpolated rather than snapped, both so it reads as a
// deliberate movement and so the thumb approaches each fingertip under
// control instead of arriving at full speed.
#define DEMO2_STEPS         20        // interpolation points per transition
#define DEMO2_STEP_MS       25        // -> 500 ms to move between poses
#define DEMO2_TOUCH_MS     600        // dwell at each fingertip contact

// --- demo3: wrist pick-and-place ------------------------------------
// Every transition is eased in and out (slow-fast-slow) rather than moved at
// a constant rate, so the hand doesn't jerk an object it's carrying. Each
// segment's duration is set separately because the wrist moves are much
// larger than the finger moves: the carry swings W_1 through ~135 deg.
// Peak wrist speed stays under ~120 deg/s at these settings, well inside
// the wrist's 240 deg/s ceiling.
#define DEMO3_STEP_MS         20      // interpolation tick (50 Hz)
#define DEMO3_T_NEUTRAL_MS  1200      // wherever the hand is -> neutral
#define DEMO3_T_SWING_MS    1500      // rotate to the side and dip
#define DEMO3_T_GRAB_MS      300      // close on the object — about as fast as the servos go
#define DEMO3_T_LIFT_MS     1000      // raise the wrist
#define DEMO3_T_CARRY_MS    1800      // rotate across and lower
#define DEMO3_T_RELEASE_MS   400      // open and let go
#define DEMO3_T_RETURN_MS   1500      // back to neutral
#define DEMO3_HOLD_MS        500      // pause after each segment

// Wrist twirl, run once the hand has reached neutral: W_2 (pitch) and W_3
// (yaw) move 90 deg out of phase so the hand traces a circle. It spirals out
// from neutral over the first half turn and back in over the last, so it
// starts and ends exactly at neutral with no jump.
//
// Unequal amplitudes make it an ellipse: tall in pitch (+/-55 of room) and
// narrower in yaw, which only travels +/-16.5. Equal amplitudes would give a
// true circle, capped at ~14 deg by yaw.
#define DEMO3_TWIRL_W2_AMP   25.0f    // pitch amplitude, deg (of +/-55)
#define DEMO3_TWIRL_W3_AMP   11.0f    // yaw amplitude, deg (of +/-16.5)
#define DEMO3_TWIRL_REVS        2     // turns in total, incl. the spiral in/out
#define DEMO3_TWIRL_REV_MS   2000     // time per turn (W_2 peaks ~79 deg/s at 25 deg)


// ---------------------------------------------------------------------
// 11. PC CONTROL PROTOCOL v3 — wire specification in PROTOCOL.md
//
// Lockstep and PC-clocked: the PC sends one T frame per PROTO_PERIOD_MS and
// the hand answers every line with exactly one reply. The hand never sends
// on a timer, so it has no rate constant of its own — PROTO_PERIOD_MS is
// here only so the two sides are documented in one place.
// ---------------------------------------------------------------------
#define PROTO_PERIOD_MS       33      // PC's T period (30 Hz) — informational
#define PROTO_MOTION_MS       20      // slew limiter, independent of the link (50 Hz)
#define PROTO_TIMEOUT_MS     250      // no valid T for this long -> freeze targets
                                      // (~7 missed frames at 30 Hz)

#define PROTO_LINE_MAX       255      // longest accepted line, excluding the ending
#define PROTO_TX_MAX         320      // reply buffer; a full S frame is ~240 bytes

#define PROTO_MIN_SPEED        1.0f   // deg/s — slower than this would never arrive
#define PROTO_MAX_SPEED      400.0f   // deg/s — roughly an MG996R's no-load rate
                                      // (the wrist is capped lower, section 4c)


// ---------------------------------------------------------------------
// 12. SAFETY — E-stop and protection (Safety.cpp)
//
// E-STOP: a single ESTOP_CHAR byte on the serial line, from the PC or typed
// in the terminal, stops the hand at once — in either mode and even in the
// middle of a routine. All 19 outputs stop pulsing (servos go limp) and stay
// that way until 'reset' (terminal) or 'C <seq> reset' (PC).
//
// PROTECTION: each joint is watched for two faults. A tripped joint goes
// limp on its own; the rest of the hand carries on. Cleared by 'reset'.
//   1. Continuous current — over the limit for longer than the hold time.
//      Short spikes (fast moves, a brief grip) are ignored. Finger current
//      is refreshed only every ~2.6 s (one LTC2497 conversion per 160 ms,
//      16 channels), so the hold must cover at least two readings.
//   2. Position deviation — the joint has STOPPED MOVING but is sitting
//      further than the limit from where it is commanded. A joint still on
//      its way to a new target is never flagged, however far it has to go.
//      "Stopped" = neither the measured nor the commanded angle has moved
//      more than PROT_STILL_BAND_DEG for PROT_STILL_MS.
// Both are paused during homing, fbfind, sweep_r and 'test', which drive
// joints into their stops on purpose.
// ---------------------------------------------------------------------
#define ESTOP_CHAR                 '!'

#define PROT_CURRENT_ENABLE         1       // 0 = current limit off
#define PROT_FINGER_CURRENT_MA     80.0f    // per motor (LTC2497). TODO: tune —
                                            // same uncalibrated scale as
                                            // HOMING_STALL_CURRENT_MA
#define PROT_WRIST_RAIL_MA       3000.0f    // all three wrist motors together (INA219)
#define PROT_CURRENT_HOLD_MS     3000       // over the limit this long -> fault
#define PROT_RAIL_SAMPLE_MS        50       // INA219 read interval

#define PROT_DEV_ENABLE             1       // 0 = deviation check off
#define PROT_DEV_LIMIT_DEG         20.0f    // settled error larger than this -> fault
#define PROT_STILL_BAND_DEG         2.0f    // movement smaller than this = "not moving"
#define PROT_STILL_MS             500       // must be still this long before judging
#define PROT_FB_MAX_AGE_MS        250       // ignore filtered readings older than this

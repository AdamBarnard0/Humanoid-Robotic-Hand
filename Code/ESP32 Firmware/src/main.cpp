/*
 * HRH1 Humanoid Hand — Firmware
 * ------------------------------------------------------------------
 * 19 joints: 16 finger/thumb joints on the Servo Click's PCA9685, and a
 * 3-DOF wrist driven directly from ESP32 GPIOs. Wiring lives in the
 * JOINT_WIRING_TABLE in Config.h (edit it after every reassembly).
 *
 * Code is split by responsibility:
 *   Config              - every tunable constant, plus the wiring table
 *   JointTypes/JointMap - joint naming + wiring/bounds/live-state table
 *   HardwareIO          - the only module touching I2C, PWM and the ADC
 *   CurrentMonitor      - LTC2497 per-motor current + INA219 wrist rail
 *   Feedback            - pot readings, volts -> degrees, channel discovery
 *   Movement            - joint-level moves, angle mapping, bounds, sweeps
 *   Homing/Calibration  - homing sequences and their flash persistence
 *   Gestures/Demo       - preset poses and demonstration sequences
 *   Diagnostics         - interactive terminal menu
 *   Protocol            - PC link (see PROTOCOL.md)
 *   Safety              - E-stop, current + position-deviation protection
 *   Startup             - boot sequence
 *
 * Serial @ 921600. Type 'h' for the command menu.
 */

#include <Arduino.h>
#include "Config.h"
#include "Startup.h"
#include "Diagnostics.h"
#include "Protocol.h"
#include "Safety.h"

// Non-blocking line reader into a fixed buffer.
//
// Replaces Serial.readStringUntil(), which allocated a heap String for
// every line (fragmenting the heap over a long session at 30 lines/s) and
// blocked for up to SERIAL_READ_TIMEOUT_MS whenever a line arrived split
// across USB packets. Bytes are now collected as they arrive, and a line is
// only acted on once its '\n' is in. An over-long line is discarded whole
// and reported, rather than being split into two garbage lines.
static char   lineBuf[PROTO_LINE_MAX + 2];   // + room for '\r' and '\0'
static size_t lineLen      = 0;
static bool   lineOverflow = false;

static void dispatchLine() {
  if (lineLen > 0 && lineBuf[lineLen - 1] == '\r') lineLen--;
  const bool tooLong = lineOverflow || lineLen > PROTO_LINE_MAX;
  lineBuf[lineLen < sizeof(lineBuf) ? lineLen : sizeof(lineBuf) - 1] = '\0';

  if (controlModeGet() == MODE_PC) {
    if (tooLong) protocolLineTooLong();
    else         protocolHandleLine(lineBuf, lineLen);
  } else {
    if (tooLong)          Serial.println(F("Line too long, ignored."));
    else if (lineLen > 0) handleCommand(String(lineBuf));
  }
  lineLen = 0;
  lineOverflow = false;
}

void setup() {
  startupSequence();
}

void loop() {
  const uint32_t t0 = micros();

  protocolTick();   // no-op outside PC mode

  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    // E-stop acts on the byte itself — no newline, no checksum, either
    // mode. It never appears inside a valid frame or command, so it can't
    // be confused with one.
    if (c == ESTOP_CHAR) { safetyEstop("serial"); continue; }
    if (c == '\n') {
      dispatchLine();
    } else if (!lineOverflow) {
      if (lineLen < PROTO_LINE_MAX + 1) lineBuf[lineLen++] = (char)c;
      else                              lineOverflow = true;
    }
  }

  // Background work, both modes: one joint's pot per pass (keeps every
  // filtered position fresh so a PC reply never has to sample), current
  // sensing, and the protection checks.
  safetyService();

  protocolNoteLoopUs(micros() - t0);
}

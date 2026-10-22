#pragma once
#include <Arduino.h>

// ---------------------------------------------------------------------
// PC CONTROL PROTOCOL v3 — full wire specification in PROTOCOL.md.
//
// Two control modes:
//   TERMINAL  the interactive menu in Diagnostics.cpp (default at boot)
//   PC        the line protocol below; the menu is not parsed
//
// Enter PC mode with 'pc', leave with 'term' (plain words, no checksum).
//
// Lockstep, PC-clocked at 30 Hz. Every line received in PC mode gets
// exactly ONE reply line; the hand never transmits unprompted.
//
//   PC -> hand   T <seq> <a0> ... <a18>*CS       targets, tenths of a degree
//   hand -> PC   S <seq> <flags> <clamp> <fb> <a0> <m0> ... <a18> <m18>*CS
//   hand -> PC   E <seq|-> <code>*CS             rejected line
//
//   PC -> hand   C <seq> <name> [args]*CS        routine / command
//   hand -> PC   K <seq> <name>*CS  ...log...  D <seq> <name>*CS
//
// CS = XOR of every byte before '*', two hex digits.
// ---------------------------------------------------------------------

enum ControlMode : uint8_t {
  MODE_TERMINAL = 0,
  MODE_PC,
};

ControlMode controlModeGet();
void controlModeSet(ControlMode m);

// One complete line received while in PC mode, line ending already removed.
void protocolHandleLine(const char *line, size_t len);

// The line reader discarded a line longer than PROTO_LINE_MAX.
void protocolLineTooLong();

// Main-loop pass duration, for the 'stats' report.
void protocolNoteLoopUs(uint32_t us);

// Watchdog, slew limiter and current sampling. Call every loop() pass;
// does nothing unless PC mode is active.
void protocolTick();

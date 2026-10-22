#pragma once
#include <Arduino.h>

// Interactive Serial command menu + dispatcher. See printMenu() for the
// full command list.
void printMenu();
void handleCommand(String cmd);

// Blocks until a full line arrives on Serial, then returns it trimmed.
// Shared by any command that needs follow-up input (manual homing, edit).
String serialWaitForLine();

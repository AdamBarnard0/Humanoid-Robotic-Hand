#include "Diagnostics.h"
#include "Config.h"
#include "HardwareIO.h"
#include "JointMap.h"
#include "Movement.h"
#include "Homing.h"
#include "Calibration.h"
#include "Gestures.h"
#include "Feedback.h"
#include "Demo.h"
#include "Protocol.h"
#include "Safety.h"

void printMenu() {
  Serial.println(F("\n===== HRH1 Hand Control Menu ====="));
  Serial.println(F("  list          - list all joints (wiring + bounds)"));
  Serial.println(F("  homeall       - run automatic homing on all joints"));
  Serial.println(F("  manualhome    - jog every joint to its limit by hand"));
  Serial.println(F("  manualhome <j>- re-home just one joint by hand"));
  Serial.println(F("  edit <j>      - live-adjust one joint's zero point (range follows)"));
  Serial.println(F("  off <j>       - cut PWM to one motor (it goes limp)"));
  Serial.println(F("  on <j>        - re-enable a motor"));
  Serial.println(F("  center        - move all joints to servo neutral (1500us)"));
  Serial.println(F("  zero          - move all joints to their 0 deg anatomical position"));
  Serial.println(F("  angle <j> <d> - move one joint to <d> degrees"));
  Serial.println(F("  gesture <g>   - strike a preset pose (bare 'gesture' lists them)"));
  Serial.println(F("  demo1         - movement demonstration (any key aborts)"));
  Serial.println(F("  demo2         - thumb touches each fingertip in turn"));
  Serial.println(F("  demo3         - wrist pick-and-place (any key aborts)"));
  Serial.println(F("  fbfind        - discover + calibrate feedback channels (one pass)"));
  Serial.println(F("  fb            - feedback readout (volts + measured degrees)"));
  Serial.println(F("  test          - centre all, then sweep each motor to prove it's wired"));
  Serial.println(F("  sweep <j>     - nudge one joint +/-150us to identify it (works unhomed)"));
  Serial.println(F("  sweep_r <j>   - sweep one joint through its full homed range"));
  Serial.println(F("  !  / estop    - E-STOP: every motor stops at once (works mid-routine)"));
  Serial.println(F("  reset         - clear the E-stop and any protection faults"));
  Serial.println(F("  faults        - E-stop state + joints stopped by protection"));
  Serial.println(F("  pc            - hand control over to the PC link ('term' returns)"));
  Serial.println(F("  h / ?         - show this menu"));
  Serial.println(F("  <j> = joint name (e.g. P_IP) or PCA channel number (0-15)"));
  Serial.println(F("==================================\n"));
}

// Interactive prompts (manual homing, edit) block here, so this is also
// where an E-stop typed at a prompt is caught. Callers check
// safetyEstopActive() after an empty return.
String serialWaitForLine() {
  while (!Serial.available()) {
    delay(10);
  }
  String s = Serial.readStringUntil('\n');
  if (s.indexOf(ESTOP_CHAR) >= 0) {
    safetyEstop("serial");
    return String("");
  }
  s.trim();
  return s;
}

static void printJointList() {
  Serial.println(F("\n--- Joints ---"));
  for (uint8_t i = 0; i < NUM_JOINTS; i++) {
    Joint &j = joints[i];
    Serial.print(F("  "));
    Serial.print(j.name);
    // Wrist joints have no PCA channel — show the GPIO that actually
    // drives them instead, otherwise the column is meaningless.
    if (j.driver == DRV_LEDC) {
      Serial.print(F(": gpio="));
      Serial.print(j.pwmPin);
    } else {
      Serial.print(F(": pca="));
      Serial.print(j.pcaChannel);
    }
    Serial.print(F(" invert="));
    Serial.print(j.invert ? F("Y") : F("N"));
    Serial.print(F(" motor="));
    Serial.print(j.attached ? F("ON ") : F("OFF"));
    Serial.print(F(" fb="));
    if (j.fbSrc == FB_ADC) {
      Serial.print(F("a"));            // direct ADC pin, fixed by wiring
      Serial.print(j.adcPin);
    } else if (j.muxChannel < 0) {
      Serial.print(F("--"));
    } else {
      Serial.print(j.muxChannel);
    }
    Serial.print(F("  min="));
    Serial.print(j.minPulseUs);
    Serial.print(F("us center="));
    Serial.print(j.centerPulseUs);
    Serial.print(F("us max="));
    Serial.print(j.maxPulseUs);
    Serial.println(F("us"));
  }
  Serial.println();
}

// Pulls the argument out of a two-word command like "sweep P_IP" or
// "off 11", and resolves it to a joint by NAME or by PCA CHANNEL number.
// Returns NUM_JOINTS if there's no argument or nothing matches.
static JointId parseJointArg(const String &cmd) {
  int sp = cmd.indexOf(' ');
  if (sp < 0) return NUM_JOINTS;
  String arg = cmd.substring(sp + 1);
  arg.trim();
  if (arg.length() == 0) return NUM_JOINTS;

  char c = arg.charAt(0);
  if (c >= '0' && c <= '9') {
    long ch = arg.toInt();
    if (ch < 0 || ch > 15) return NUM_JOINTS;
    return findJointByChannel((uint8_t)ch);
  }
  return findJointByName(arg.c_str());
}

// Parses "angle <name|channel> <deg>", e.g. "angle P_IP 45" or
// "angle 4 -12.5". Returns false if malformed or the joint is unknown.
static bool parseAngleCmd(const String &cmd, JointId &id, float &deg) {
  int sp1 = cmd.indexOf(' ');
  if (sp1 < 0) return false;
  String rest = cmd.substring(sp1 + 1);
  rest.trim();

  int sp2 = rest.indexOf(' ');
  if (sp2 < 0) return false;
  String jArg = rest.substring(0, sp2);
  String dArg = rest.substring(sp2 + 1);
  jArg.trim();
  dArg.trim();
  if (jArg.length() == 0 || dArg.length() == 0) return false;

  char c = jArg.charAt(0);
  if (c >= '0' && c <= '9') {
    long ch = jArg.toInt();
    if (ch < 0 || ch > 15) return false;
    id = findJointByChannel((uint8_t)ch);
  } else {
    id = findJointByName(jArg.c_str());
  }
  if (id == NUM_JOINTS) return false;

  char d0 = dArg.charAt(0);
  if (!((d0 >= '0' && d0 <= '9') || d0 == '-' || d0 == '+' || d0 == '.')) return false;
  deg = dArg.toFloat();
  return true;
}

static void commandJointAngle(JointId id, float deg) {
  Joint &j = joints[id];

  if (j.minPulseUs == 0 && j.maxPulseUs == 0) {
    Serial.println(F("  No bounds yet — run 'homeall' or 'manualhome' first."));
    return;
  }

  float loDeg, hiDeg;
  jointAngleLimits(id, loDeg, hiDeg);

  if (deg < loDeg || deg > hiDeg) {
    Serial.print(F("  Out of range: "));
    Serial.print(j.name);
    Serial.print(F(" accepts "));
    Serial.print(loDeg, 1);
    Serial.print(F(" to "));
    Serial.print(hiDeg, 1);
    Serial.println(F(" deg — clamping."));
    deg = (deg < loDeg) ? loDeg : hiDeg;
  }

  setJointAngleDeg(id, deg);

  if ((safetyFaultMask() >> id) & 1UL) {
    Serial.print(F("  NOTE: "));
    Serial.print(j.name);
    Serial.print(F(" is stopped by protection ("));
    Serial.print(safetyFaultName(safetyFaultReason(id)));
    Serial.println(F(") — it won't move until 'reset'."));
  }

  Serial.print(F("  "));
  Serial.print(j.name);
  Serial.print(F(" -> "));
  Serial.print(deg, 1);
  Serial.print(F(" deg  ("));
  Serial.print(j.currentPulseUs);
  Serial.println(F("us)"));

  if (!j.attached) {
    Serial.println(F("  (motor is OFF — 'on' it to see the change)"));
  }
}

// The pulse width that currently corresponds to 0 deg for this joint.
static uint16_t zeroPulseUs(JointId id) {
  const Joint &j = joints[id];
  return jointZeroIsCenter(id) ? j.centerPulseUs : j.minPulseUs;
}

static void printZeroPrompt(JointId id) {
  const Joint &j = joints[id];
  Serial.print(F("  "));
  Serial.print(j.name);
  Serial.print(F(" zero @ "));
  Serial.print(zeroPulseUs(id));
  Serial.print(F("us  [min "));
  Serial.print(j.minPulseUs);
  Serial.print(F(" / ctr "));
  Serial.print(j.centerPulseUs);
  Serial.print(F(" / max "));
  Serial.print(j.maxPulseUs);
  Serial.print(F("] > "));
}

// Live adjustment of a joint's 0 deg reference. The joint is physically
// held at 0 deg the whole time and re-commanded after every step, so you
// tune it by eye exactly like manual homing.
//
// Only the zero point is entered by hand — min/center/max all shift with
// it by the same amount, so the range of motion is preserved and stays
// consistent with the span homing derived from the joint-type offset. This
// is the right correction for homing having stopped slightly early or late:
// the ROM was fine, the reference was off.
static void editJoint(JointId id) {
  Joint &j = joints[id];

  if (j.minPulseUs == 0 && j.maxPulseUs == 0) {
    Serial.println(F("  No bounds yet — run 'homeall' or 'manualhome' first."));
    return;
  }

  Serial.print(F("\n--- Edit zero: "));
  Serial.print(j.name);
  Serial.print(F(" (pca "));
  Serial.print(j.pcaChannel);
  Serial.println(F(") ---"));

  Serial.print(F("  0 deg reference: "));
  Serial.println(jointZeroIsCenter(id) ? F("centre of travel")
                                       : F("min bound (fully extended)"));
  Serial.print(F("  Range of motion: "));
  Serial.print((uint16_t)(j.maxPulseUs - j.minPulseUs));
  Serial.println(F("us (preserved — the whole range shifts with the zero)"));
  Serial.println(F("  Type +/- us to move the zero point, 'd' to save, 'q' to abort."));

  if (!j.attached) {
    Serial.println(F("  NOTE: motor is OFF — values will update but it won't move."));
  }

  const uint16_t mn0 = j.minPulseUs, ctr0 = j.centerPulseUs, mx0 = j.maxPulseUs;

  setJointAngleDeg(id, 0.0f); // park at zero so the starting point is visible

  while (true) {
    printZeroPrompt(id);
    String line = serialWaitForLine();

    if (safetyEstopActive()) {
      // Don't save a zero point the joint was never seen at.
      j.minPulseUs = mn0;
      j.centerPulseUs = ctr0;
      j.maxPulseUs = mx0;
      Serial.println(F("  E-stop — edit abandoned, original values kept."));
      return;
    }
    if (line.length() == 0) continue;
    if (line.equalsIgnoreCase("d")) break;
    if (line.equalsIgnoreCase("q")) {
      j.minPulseUs = mn0;
      j.centerPulseUs = ctr0;
      j.maxPulseUs = mx0;
      setJointAngleDeg(id, 0.0f);
      Serial.println(F("  Aborted, original values restored."));
      return;
    }

    long d = line.toInt();
    if (d == 0) {
      Serial.println(F("  Enter a shift in us (e.g. 10 or -10), 'd' to save, 'q' to abort."));
      continue;
    }

    // Clamp the shift so the whole window stays inside the safety envelope.
    if ((long)j.minPulseUs + d < SERVO_MIN_US) d = (long)SERVO_MIN_US - (long)j.minPulseUs;
    if ((long)j.maxPulseUs + d > SERVO_MAX_US) d = (long)SERVO_MAX_US - (long)j.maxPulseUs;
    if (d == 0) {
      Serial.println(F("  Already against the safety envelope — can't shift further that way."));
      continue;
    }

    j.minPulseUs    = (uint16_t)((long)j.minPulseUs + d);
    j.centerPulseUs = (uint16_t)((long)j.centerPulseUs + d);
    j.maxPulseUs    = (uint16_t)((long)j.maxPulseUs + d);

    setJointAngleDeg(id, 0.0f); // follow the new zero live
  }

  calibrationSaveJoint((uint8_t)id);

  Serial.print(F("  Saved: zero="));
  Serial.print(zeroPulseUs(id));
  Serial.print(F("us  min="));
  Serial.print(j.minPulseUs);
  Serial.print(F("us center="));
  Serial.print(j.centerPulseUs);
  Serial.print(F("us max="));
  Serial.print(j.maxPulseUs);
  Serial.println(F("us"));
}

static void setMotorEnabled(JointId id, bool enable) {
  Joint &j = joints[id];
  if (enable) {
    j.attached = true;                     // must come first, movement funcs check it
    setJointPulseUs(id, j.centerPulseUs);  // restore the pulse train
    Serial.print(F("Motor ON: "));
  } else {
    hwDisableJoint(j);
    j.attached = false;
    Serial.print(F("Motor OFF (limp): "));
  }
  Serial.print(j.name);
  Serial.print(F(" (pca "));
  Serial.print(j.pcaChannel);
  Serial.println(F(")"));
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    printMenu();
    return;
  }

  // --- safety, always available ---
  if (cmd.equalsIgnoreCase("estop"))  { safetyEstop("terminal"); return; }
  if (cmd.equalsIgnoreCase("reset"))  { safetyReset();           return; }
  if (cmd.equalsIgnoreCase("faults")) { safetyPrintStatus();     return; }

  // While E-stopped, only commands that don't move anything are accepted.
  if (safetyEstopActive()) {
    const bool passive = cmd.equalsIgnoreCase("list") || cmd.equalsIgnoreCase("fb") ||
                         cmd.equalsIgnoreCase("h")    || cmd.equalsIgnoreCase("?") ||
                         cmd.equalsIgnoreCase("pc")   || cmd.equalsIgnoreCase("gesture");
    if (!passive) {
      Serial.println(F("E-STOP is active — motors are off. Type 'reset' to re-enable."));
      return;
    }
  }

  if (cmd.equalsIgnoreCase("list"))       { printJointList();      return; }
  if (cmd.equalsIgnoreCase("homeall"))    { homeAllJoints();       return; }
  if (cmd.equalsIgnoreCase("manualhome")) { manualHomeAllJoints(); return; }
  if (cmd.equalsIgnoreCase("center"))     { centerAllJoints();     return; }
  if (cmd.equalsIgnoreCase("zero"))       { zeroAllJoints();       return; }
  if (cmd.equalsIgnoreCase("gesture")) { listGestures();   return; }
  if (cmd.equalsIgnoreCase("demo1"))   { runMovementDemo(); return; }
  if (cmd.equalsIgnoreCase("demo2"))   { runTouchDemo();    return; }
  if (cmd.equalsIgnoreCase("demo3"))   { runWristDemo();    return; }
  if (cmd.equalsIgnoreCase("pc"))      { controlModeSet(MODE_PC); return; }
  if (cmd.equalsIgnoreCase("fbfind"))  { discoverFeedbackChannels(); return; }
  if (cmd.equalsIgnoreCase("fb"))       { printJointFeedback();   return; }
  if (cmd.equalsIgnoreCase("test"))     { runConnectivityTest();  return; }
  if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("?")) { printMenu(); return; }

  // --- gesture <name> ---
  if (cmd.startsWith("gesture ") || cmd.startsWith("Gesture ") || cmd.startsWith("GESTURE ")) {
    String arg = cmd.substring(cmd.indexOf(' ') + 1);
    arg.trim();
    if (arg.length() == 0 || !applyGesture(arg.c_str())) {
      Serial.println(F("Unknown gesture."));
      listGestures();
    }
    return;
  }

  // --- three-word: angle <joint> <deg> ---
  if (cmd.startsWith("angle ") || cmd.startsWith("Angle ") || cmd.startsWith("ANGLE ")) {
    JointId id;
    float deg;
    if (!parseAngleCmd(cmd, id, deg)) {
      Serial.println(F("Usage: angle <name|channel> <degrees>   e.g. 'angle P_IP 45'"));
      return;
    }
    commandJointAngle(id, deg);
    return;
  }

  // --- two-word commands, all taking a joint name or PCA channel ---
  struct { const char* verb; uint8_t action; } verbs[] = {
    { "sweep", 0 }, { "edit", 1 }, { "off", 2 }, { "on", 3 }, { "manualhome", 4 },
    { "sweep_r", 5 },
  };

  for (auto &v : verbs) {
    String prefix = String(v.verb) + " ";
    if (!cmd.startsWith(prefix.c_str())) continue;

    JointId id = parseJointArg(cmd);
    if (id == NUM_JOINTS) {
      Serial.print(F("Unknown joint. Usage: "));
      Serial.print(v.verb);
      Serial.println(F(" <name|channel>   (try 'list')"));
      return;
    }
    switch (v.action) {
      case 0: sweepJoint(id);           break;
      case 1: editJoint(id);            break;
      case 2: setMotorEnabled(id, false); break;
      case 3: setMotorEnabled(id, true);  break;
      case 4: manualHomeSingleJoint(id);  break;
      case 5: sweepJointRange(id);        break;
    }
    return;
  }

  Serial.println(F("Unknown command. Type 'h' for menu."));
}

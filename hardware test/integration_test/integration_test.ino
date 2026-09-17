// Phase 2 integration test — the real Robot class (ToF + IMU + motors
// together), see ../HARDWARE_TEST_FLOW.md sections 2.1-2.6.
//
// Robot.h/.cpp, Tof.h/.cpp, IMU.h/.cpp, motor.h/.cpp are all symlinked into
// this folder, so this exercises the exact production integration code
// (Robot::begin/move/turn/snapToCardinal/isWallFront-Left-Right), not a
// reimplementation. Flashing this sketch temporarily replaces
// micromouse.ino on the board -- reflash the main firmware afterwards.
//
// *** WHEELS OFF THE GROUND unless you are specifically testing move()/
//     turn() distance or heading, per the test plan. ***
//
// ---------------------------------------------------------------------------
// WIRING — same full pin set as the Phase 1 tests combined (ToF + IMU +
// motors + encoders all active at once). Board: ESP32 Dev Module.
// ---------------------------------------------------------------------------
//   I2C bus (ToF x3 + IMU, shared):
//     GPIO21 = SDA, GPIO22 = SCL, 3.3V, GND
//   ToF XSHUT: Right=GPIO4  Center=GPIO18  Left=GPIO19
//   IMU (BNO08x) address 0x4B on the same I2C bus
//   Left motor:  IN1=GPIO33 IN2=GPIO25 PWM=GPIO32
//   Right motor: IN1=GPIO27 IN2=GPIO26 PWM=GPIO14
//   Left encoder: A=GPIO39 B=GPIO36     Right encoder: A=GPIO34 B=GPIO35
// ---------------------------------------------------------------------------
//
// Serial commands (type a full line + Enter, 115200 baud):
//   sensors        -- one-shot print of Left/Center/Right ToF mm
//   walls          -- one-shot print of isWallFront/Left/Right
//   move <N>       -- robot.move(N) -- drives N cells forward (blocking)
//   turn <deg>     -- robot.turn(deg) -- relative turn, e.g. "turn 90" (blocking)
//   snap           -- robot.snapToCardinal() -- snap heading to nearest 90 deg
//   drift CONFIRM  -- robot.calibrateDriftFactor() -- ONLY run wheels off the
//                     ground: drives BOTH motors at PWM 255 (max, unthrottled)
//                     for 5 seconds with no obstacle/abort check (see RD-07).
//                     Requires the literal extra word CONFIRM as a safety gate.
//   spin           -- infinite loop: turn(90), wait 3s, repeat, forever.
//                     For untethered PID testing -- send this while still
//                     connected, then disconnect. Only a reset/reflash stops it.
//   help           -- show this list
//
// A background heartbeat line (ToF distances + wall booleans) prints once a
// second whenever no command is being typed.

#include "Robot.h"

// Set to 1 while doing untethered PID tuning (spin starts automatically on
// every power-up/reset, no serial connection needed). Set to 0 to go back to
// the plain interactive command mode (sensors/walls/move/turn/snap/drift).
#define AUTO_SPIN_ON_BOOT 0

Robot robot;
String lineBuf;
unsigned long lastHeartbeat = 0;

void printHelp() {
  Serial.println();
  Serial.println(F("=== Robot integration test ==="));
  Serial.println(F("Commands: sensors | walls | move <N> | turn <deg> | spin | snap | drift CONFIRM | help"));
  Serial.println();
}

void printWalls() {
  Serial.print(F("wallFront=")); Serial.print(robot.isWallFront());
  Serial.print(F(" wallLeft="));  Serial.print(robot.isWallLeft());
  Serial.print(F(" wallRight=")); Serial.println(robot.isWallRight());
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "help") {
    printHelp();
  } else if (line == "sensors") {
    robot.print_all_sensors();
  } else if (line == "walls") {
    printWalls();
  } else if (line.startsWith("move ")) {
    int cells = line.substring(5).toInt();
    Serial.print(F("move(")); Serial.print(cells); Serial.println(F(") ..."));
    robot.move(cells);
    Serial.println(F("move() done."));
  } else if (line.startsWith("turn ")) {
    int deg = line.substring(5).toInt();
    Serial.print(F("turn(")); Serial.print(deg); Serial.println(F(") ..."));
    robot.turn(deg);
    Serial.println(F("turn() done."));
  } else if (line == "spin") {
    Serial.println(F("spin: infinite loop of turn(90) + 3s pause, starting now. Reset/reflash to stop."));
    delay(500);
    while (true) {
      robot.turn(90);
      delay(3000);
    }
  } else if (line == "snap") {
    Serial.println(F("snapToCardinal() ..."));
    robot.snapToCardinal();
    Serial.println(F("snapToCardinal() done."));
  } else if (line == "drift CONFIRM") {
    Serial.println(F("calibrateDriftFactor(): BOTH motors at PWM 255 for 5s, no abort condition."));
    float factor = robot.calibrateDriftFactor();
    Serial.print(F("Drift factor (leftDist/rightDist): "));
    Serial.println(factor);
  } else if (line == "drift") {
    Serial.println(F("Refusing: type 'drift CONFIRM' (wheels off the ground -- runs both motors at max PWM for 5s)."));
  } else {
    Serial.print(F("Unknown command: "));
    Serial.println(line);
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  printHelp();

  Serial.println(F("robot.begin() -- this includes a 5s stabilization delay,"));
  Serial.println(F("and will hang here if the IMU fails to initialize."));
  robot.begin();
  Serial.println(F("robot.begin() done."));

#if AUTO_SPIN_ON_BOOT
  Serial.println(F("AUTO_SPIN_ON_BOOT=1: starting spin (turn 90 + 3s pause, forever) in 2s..."));
  delay(2000);
  while (true) {
    robot.turn(90);
    delay(3000);
  }
#endif
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuf.length() > 0) {
        handleLine(lineBuf);
        lineBuf = "";
      }
    } else {
      lineBuf += c;
    }
  }

  if (millis() - lastHeartbeat > 1000) {
    lastHeartbeat = millis();
    robot.print_all_sensors();
    printWalls();
  }
}

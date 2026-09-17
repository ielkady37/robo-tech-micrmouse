// Phase 1 unit test — encoders + motors (motor.h/motor.cpp), see
// ../HARDWARE_TEST_FLOW.md sections 1.2 and 1.3.
//
// Exercises the real MotorDriver class from the main firmware (motor.h/
// motor.cpp are symlinked into this folder) in isolation, with no IMU/ToF/
// algorithm code involved. Flashing this sketch temporarily replaces
// micromouse.ino on the board — reflash the main firmware afterwards.
//
// *** KEEP THE WHEELS OFF THE GROUND FOR THIS TEST ***
//
// ---------------------------------------------------------------------------
// WIRING — connect before powering on. Board: ESP32 Dev Module.
// ---------------------------------------------------------------------------
//   Left motor:
//     ESP32 GPIO 33 (IN1L) -> Left H-bridge IN1
//     ESP32 GPIO 25 (IN2L) -> Left H-bridge IN2
//     ESP32 GPIO 32 (speedL, PWM) -> Left H-bridge ENA/PWM
//   Right motor:
//     ESP32 GPIO 27 (IN1R) -> Right H-bridge IN1
//     ESP32 GPIO 26 (IN2R) -> Right H-bridge IN2
//     ESP32 GPIO 14 (speedR, PWM) -> Right H-bridge ENB/PWM
//   Left encoder:
//     ESP32 GPIO 39 (ENCAL) -> Left encoder channel A (yellow)
//     ESP32 GPIO 36 (ENCBL) -> Left encoder channel B (white)
//   Right encoder:
//     ESP32 GPIO 34 (ENCAR) -> Right encoder channel A (yellow)
//     ESP32 GPIO 35 (ENCBR) -> Right encoder channel B (white)
//   GPIO 34-39 are input-only pins on the ESP32 -- do not use them for
//   anything but encoder input.
//   Motor power (battery via H-bridge) is separate from ESP32 logic power;
//   make sure both share a common GND.
// ---------------------------------------------------------------------------
//
// Serial commands (type a single character + Enter, 115200 baud):
//   1 = left motor forward   2 = left motor reverse
//   3 = right motor forward  4 = right motor reverse
//   5 = both forward         6 = both reverse
//   7 = spin left (left rev, right fwd)   8 = spin right (left fwd, right rev)
//   0 = stop both
//   r = reset both encoders to 0
//   + / - = increase / decrease test speed (default 60, range 0-255)
//
// Tick counts and computed distances print continuously regardless of
// command state, so you can also just rotate a wheel by hand (motors
// stopped) and watch the ticks/distance change per test 1.2.1-1.2.5.

#include "motor.h"

MotorDriver motor_driver;
int16_t testSpeed = 60;

void printHelp() {
  Serial.println();
  Serial.println(F("=== Motor + Encoder unit test ==="));
  Serial.println(F("KEEP WHEELS OFF THE GROUND."));
  Serial.println(F("Commands: 1=Lfwd 2=Lrev 3=Rfwd 4=Rrev 5=bothFwd 6=bothRev"));
  Serial.println(F("          7=spinLeft 8=spinRight 0=stop r=resetEnc +/-=speed"));
  Serial.print(F("Test speed: "));
  Serial.println(testSpeed);
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  motor_driver.begin();
  printHelp();
  Serial.println(F("PosL\tPosR\tDistL_cm\tDistR_cm"));
}

void handleCommand(char c) {
  switch (c) {
    case '1': motor_driver.setMotors(testSpeed, 0); break;
    case '2': motor_driver.setMotors(-testSpeed, 0); break;
    case '3': motor_driver.setMotors(0, testSpeed); break;
    case '4': motor_driver.setMotors(0, -testSpeed); break;
    case '5': motor_driver.setMotors(testSpeed, testSpeed); break;
    case '6': motor_driver.setMotors(-testSpeed, -testSpeed); break;
    case '7': motor_driver.setMotors(-testSpeed, testSpeed); break;
    case '8': motor_driver.setMotors(testSpeed, -testSpeed); break;
    case '0': motor_driver.setMotors(0, 0); break;
    case 'r':
      motor_driver.resetEncoderL();
      motor_driver.resetEncoderR();
      Serial.println(F("Encoders reset."));
      break;
    case '+':
      testSpeed = min(255, testSpeed + 10);
      Serial.print(F("Test speed: "));
      Serial.println(testSpeed);
      break;
    case '-':
      testSpeed = max(0, testSpeed - 10);
      Serial.print(F("Test speed: "));
      Serial.println(testSpeed);
      break;
    case '\n':
    case '\r':
      break;
    default:
      printHelp();
      break;
  }
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.read());
  }

  Serial.print(motor_driver.getPosL());
  Serial.print(F("\t"));
  Serial.print(motor_driver.getPosR());
  Serial.print(F("\t"));
  Serial.print(motor_driver.getDistanceL());
  Serial.print(F("\t\t"));
  Serial.println(motor_driver.getDistanceR());

  delay(300);
}

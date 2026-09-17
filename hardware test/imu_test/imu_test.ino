// Phase 1 unit test — IMU (IMU.h/IMU.cpp), see ../HARDWARE_TEST_FLOW.md
// section 1.5.
//
// Exercises the real IMU class from the main firmware (IMU.h/IMU.cpp are
// symlinked into this folder) in isolation, with no motor/ToF/algorithm
// code involved. Flashing this sketch temporarily replaces micromouse.ino
// on the board — reflash the main firmware afterwards.
//
// ---------------------------------------------------------------------------
// WIRING — connect before powering on. Board: ESP32 Dev Module.
// ---------------------------------------------------------------------------
//   Shared I2C bus (same bus the ToF sensors use):
//     ESP32 GPIO 21 (SDA) -> BNO08x SDA
//     ESP32 GPIO 22 (SCL) -> BNO08x SCL
//     ESP32 3V3           -> BNO08x VIN -- 3.3V logic, do not use 5V
//     ESP32 GND           -> BNO08x GND
//   BNO08x is expected at I2C address 0x4B (IMU.cpp:24). If your breakout's
//   address-select pad/jumper is set differently, begin_I2C(0x4B) will fail
//   and the sketch will print "Failed to find BNO08x at 0x4B!" and hang.
// ---------------------------------------------------------------------------
//
// The first 5 seconds after boot are the yaw-offset calibration window
// (IMU.cpp:60-73) -- keep the sensor still and level during that time.
// getYaw() reads 0 the whole time it's calibrating; once it settles,
// yaw should read ~0 at rest and track physical rotation smoothly.
//
// Robot.cpp always reads "-imu.getYaw()" (note the negation) as its heading
// convention, so this test prints both the raw value and that negated
// value side by side.

#include "IMU.h"

IMU imu;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("=== IMU unit test ==="));
  Serial.println(F("Wiring expected: I2C SDA=GPIO21 SCL=GPIO22, BNO08x @ 0x4B"));
  Serial.println(F("Keep the board still and level for 5s after this line for calibration..."));

  imu.begin();  // hangs here (while(1)) if the IMU doesn't ack at 0x4B

  Serial.println(F("BNO08x ready."));
  Serial.println(F("rawYaw\t-rawYaw(Robot.cpp convention)\tPitch\tRoll"));
}

void loop() {
  imu.update();

  float yaw = imu.getYaw();
  float pitch = imu.getPitch();
  float roll = imu.getRoll();

  Serial.print(yaw);
  Serial.print(F("\t"));
  Serial.print(-yaw);
  Serial.print(F("\t\t\t\t"));
  Serial.print(pitch);
  Serial.print(F("\t"));
  Serial.println(roll);

  delay(100);
}

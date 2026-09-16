// Phase 1 unit test — ToF sensor suite (3x VL53L0X), see ../HARDWARE_TEST_FLOW.md section 1.4.
//
// Exercises the real TOF class from the main firmware (Tof.h/Tof.cpp are
// symlinked into this folder) in isolation, with no motor/IMU/algorithm
// code involved. Flashing this sketch temporarily replaces micromouse.ino
// on the board — reflash the main firmware afterwards.
//
// ---------------------------------------------------------------------------
// WIRING — connect before powering on. Board: ESP32 Dev Module.
// ---------------------------------------------------------------------------
//   Shared I2C bus (all three sensors + IMU share this):
//     ESP32 GPIO 21 (SDA) -> VL53L0X SDA (all three, same wire)
//     ESP32 GPIO 22 (SCL) -> VL53L0X SCL (all three, same wire)
//     ESP32 3V3           -> VL53L0X VIN (all three) -- 3.3V logic, do not use 5V
//     ESP32 GND           -> VL53L0X GND (all three)
//
//   XSHUT (shutdown) pins — one per sensor, used to bring them up one at a
//   time so each can be re-addressed on the shared I2C bus:
//     ESP32 GPIO 4  -> XSHUT on the RIGHT sensor   (becomes I2C address 0x30)
//     ESP32 GPIO 18 -> XSHUT on the CENTER sensor  (becomes I2C address 0x31)
//     ESP32 GPIO 19 -> XSHUT on the LEFT sensor    (becomes I2C address 0x32)
//
//   Mount each sensor physically on the side matching its label (Right/
//   Center/Left as seen facing forward from the robot's rear) — the
//   test below reports readings by that label, not by wire color.
// ---------------------------------------------------------------------------

#include "Tof.h"

#define THRESHOLD_FRONT 70   // mm, matches Robot.h — center "wall" cutoff
#define THRESHOLD_SIDE 170   // mm, matches Robot.h — left/right "wall" cutoff

TOF tof;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println(F("=== ToF unit test ==="));
  Serial.println(F("Wiring expected:"));
  Serial.println(F("  I2C  SDA=GPIO21  SCL=GPIO22  (shared bus, 3.3V)"));
  Serial.println(F("  XSHUT Right=GPIO4  Center=GPIO18  Left=GPIO19"));
  Serial.println();

  tof.begin();  // hangs here (while(1)) if any sensor fails to ACK its address — see Serial output for which one

  Serial.println(F("All three VL53L0X sensors initialized OK."));
  Serial.println(F("Left_mm  Center_mm  Right_mm  | wallLeft wallFront wallRight"));
}

void loop() {
  tof.updateReadings();

  uint16_t left = tof.getTofLeft();
  uint16_t center = tof.getTofCenter();
  uint16_t right = tof.getTofRight();

  bool wallLeft = left <= THRESHOLD_SIDE;
  bool wallFront = center <= THRESHOLD_FRONT;
  bool wallRight = right <= THRESHOLD_SIDE;

  Serial.print(left);
  Serial.print(F("\t "));
  Serial.print(center);
  Serial.print(F("\t  "));
  Serial.print(right);
  Serial.print(F("\t  | "));
  Serial.print(wallLeft);
  Serial.print(F("        "));
  Serial.print(wallFront);
  Serial.print(F("         "));
  Serial.println(wallRight);

  delay(200);
}

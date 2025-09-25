#include "Tof.h"
#include <Arduino.h>  // Needed for pinMode, digitalRead, etc.

// address we will assign for all sensors
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32  // Check compitability with imu address

// set the pins to shutdown
#define SHT_LOX1 4
#define SHT_LOX2 18
#define SHT_LOX3 19

// Static variables
SemaphoreHandle_t TOF::left_tof_Mutex;
SemaphoreHandle_t TOF::right_tof_Mutex;
SemaphoreHandle_t TOF::center_tof_Mutex;

void TOF::begin() {
  left_tof_Mutex = xSemaphoreCreateMutex();
  right_tof_Mutex = xSemaphoreCreateMutex();
  center_tof_Mutex = xSemaphoreCreateMutex();

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);
  pinMode(SHT_LOX3, OUTPUT);

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  Serial.println(F("All in reset mode...(pins are low)"));


  Serial.println(F("Starting..."));
  setID();
}

void TOF::setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);
  digitalWrite(SHT_LOX3, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while (1)
      ;
  }
  delay(10);

  // activating LOX3
  digitalWrite(SHT_LOX3, HIGH);
  delay(10);

  //initing LOX3
  if (!lox3.begin(LOX3_ADDRESS)) {
    Serial.println(F("Failed to boot third VL53L0X"));
    while (1)
      ;
  }
}

void TOF::updateReadings() {
  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);  // pass in 'true' to get debug data printout!
  lox3.rangingTest(&measure3, false);  // pass in 'true' to get debug data printout!

  if (xSemaphoreTake(left_tof_Mutex, portMAX_DELAY) == pdTRUE) {
    if (measure3.RangeStatus != 4)
      leftDistance = measure3.RangeMilliMeter;
    else
      leftDistance = 65535;
    xSemaphoreGive(left_tof_Mutex);
  }

  if (xSemaphoreTake(right_tof_Mutex, portMAX_DELAY) == pdTRUE) {
    if (measure1.RangeStatus != 4)
      rightDistance = measure1.RangeMilliMeter;
    else
      rightDistance = 65535;
    xSemaphoreGive(right_tof_Mutex);
  }

  if (xSemaphoreTake(center_tof_Mutex, portMAX_DELAY) == pdTRUE) {
    if (measure2.RangeStatus != 4)
      centerDistance = measure2.RangeMilliMeter;
    else
      centerDistance = 65535;
    xSemaphoreGive(center_tof_Mutex);
  }
}

uint16_t TOF::getTofRight() {
  int temp = 0;
  if (xSemaphoreTake(right_tof_Mutex, portMAX_DELAY) == pdTRUE) {
    temp = rightDistance;
    xSemaphoreGive(right_tof_Mutex);
  }
  return temp;
}

uint16_t TOF::getTofCenter() {
  int temp = 0;
  if (xSemaphoreTake(center_tof_Mutex, portMAX_DELAY) == pdTRUE) {
    temp = centerDistance;
    xSemaphoreGive(center_tof_Mutex);
  }
  return temp;
}

uint16_t TOF::getTofLeft() {
  int temp = 0;
  if (xSemaphoreTake(left_tof_Mutex, portMAX_DELAY) == pdTRUE) {
    temp = leftDistance;
    xSemaphoreGive(left_tof_Mutex);
  }
  return temp;
}

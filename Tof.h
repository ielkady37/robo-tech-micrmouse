#ifndef TOF_H
#define TOF_H

#include "Adafruit_VL53L0X.h"
#include <stdint.h>

class TOF {
  public:
    void begin();              // Initialization function
    uint16_t getTofRight();
    uint16_t getTofCenter();
    uint16_t getTofLeft();
    void updateReadings();

  private:
    // objects for the vl53l0x
    Adafruit_VL53L0X lox1;
    Adafruit_VL53L0X lox2;
    Adafruit_VL53L0X lox3;

    // this holds the measurement
    VL53L0X_RangingMeasurementData_t measure1;
    VL53L0X_RangingMeasurementData_t measure2;
    VL53L0X_RangingMeasurementData_t measure3;
    
    // Semaphores
    static SemaphoreHandle_t left_tof_Mutex;
    static SemaphoreHandle_t right_tof_Mutex;
    static SemaphoreHandle_t center_tof_Mutex;

    // Distances
    int leftDistance;
    int rightDistance;
    int centerDistance;

    void setID();

};

#endif

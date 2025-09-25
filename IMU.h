#ifndef IMU_H
#define IMU_H

#include <Adafruit_BNO08x.h>
#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

class IMU {
public:
  IMU();
  bool begin();
  void update();
  float getYaw();
  float getPitch();
  float getRoll();

private:
  Adafruit_BNO08x myIMU;
  // BNO080 myIMU;

  long double yaw, pitch, roll;
  long double continuousYaw;
  long double lastYaw;
  long double yawOffset;

  // calibration
  bool calibrationDone;
  long double sum;
  long long count;
  unsigned long calibStart;
  static SemaphoreHandle_t imu_Mutex;

  void setReports();
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 1000;

};

#endif

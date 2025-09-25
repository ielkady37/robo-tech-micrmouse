#include "IMU.h"
#include <math.h>

SemaphoreHandle_t IMU::imu_Mutex;

IMU::IMU() {
  yaw = pitch = roll = 0;
  continuousYaw = 0;
  lastYaw = 0;
  yawOffset = 0;

  calibrationDone = false;
  sum = 0;
  count = 0;
  calibStart = 0;
}

bool IMU::begin() {
  imu_Mutex = xSemaphoreCreateMutex();

  Serial.begin(115200);
  Wire.begin();

  if (!myIMU.begin_I2C(0x4B)) {   // use I2C addr 0x4B
    Serial.println("Failed to find BNO08x at 0x4B!");
    while (1);
  }
  Serial.println("BNO08x Found at 0x4B!");

  // Enable one of the stable reports
  if (!myIMU.enableReport(SH2_ARVR_STABILIZED_RV)) {
    Serial.println("Could not enable ARVR report!");
    while (1);
  }

  return true;
}

void IMU::update() {
  sh2_SensorValue_t sensorValue;
  if (myIMU.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      float qw = sensorValue.un.arvrStabilizedRV.real;
      float qx = sensorValue.un.arvrStabilizedRV.i;
      float qy = sensorValue.un.arvrStabilizedRV.j;
      float qz = sensorValue.un.arvrStabilizedRV.k;

      // quaternion → yaw/pitch/roll
      float yawNow = atan2f(2.0f * (qx * qy + qw * qz),
                            qw * qw + qx * qx - qy * qy - qz * qz)
                     * 180.0f / PI;
      float pitchNow = asinf(-2.0f * (qx * qz - qw * qy)) * 180.0f / PI;
      float rollNow  = atan2f(2.0f * (qw * qx + qy * qz),
                              qw * qw - qx * qx - qy * qy + qz * qz)
                       * 180.0f / PI;
      // Serial.println("------------ YAW: ");
      // // Serial.println(yawNow);U
      // --- calibration phase ---
      if (!calibrationDone) {
        if (millis() - calibStart < 5000) {
          sum += yawNow;
          count++;
        } else {
          if (count > 0) {
            yawOffset = sum / count;   // average yaw = offset
          } else {
            yawOffset = yawNow;        // fallback
          }
          continuousYaw = 0;           // reset unwrap
          lastYaw = yawNow - yawOffset;
          calibrationDone = true;
          // Serial.println("Calibration done → yaw starts at 0");
        }
      }

      // always apply offset after calibration
      float yawCorrected = yawNow - yawOffset;
      if (yawCorrected > 180) yawCorrected -= 360;
      else if (yawCorrected < -180) yawCorrected += 360;

      // unwrap
      float delta = yawCorrected - lastYaw;
      if (delta > 180) delta -= 360;
      else if (delta < -180) delta += 360;

      continuousYaw += delta;
      lastYaw = yawCorrected;

      // store safely
      if (xSemaphoreTake(imu_Mutex, portMAX_DELAY) == pdTRUE) {
        this->yaw   = calibrationDone ? continuousYaw : 0; // force 0 until calibration finishes
        this->pitch = pitchNow;
        this->roll  = rollNow;
        xSemaphoreGive(imu_Mutex);
      }
    }
  }
}


float IMU::getYaw() {
  float temp = 0;
  if (xSemaphoreTake(imu_Mutex, portMAX_DELAY) == pdTRUE) {
    temp = this->yaw;
    xSemaphoreGive(imu_Mutex);
  }
  return temp;
}
float IMU::getPitch() {
  return pitch;
}
float IMU::getRoll() {
  return roll;
}


// #include "IMU.h"
// #include <math.h>
// #include <Wire.h>
// #include "SparkFun_BNO080_Arduino_Library.h"

// BNO080 myIMU;  // SparkFun BNO080 object

// SemaphoreHandle_t IMU::imu_Mutex;

// IMU::IMU() {
//   yaw = pitch = roll = 0;
//   continuousYaw = 0;
//   lastYaw = 0;
//   yawOffset = 0;

//   calibrationDone = false;
//   sum = 0;
//   count = 0;
//   calibStart = 0;
// }

// bool IMU::begin() {
//   imu_Mutex = xSemaphoreCreateMutex();
//   Serial.begin(115200);
//   Wire.begin();
//   if (!myIMU.begin()) {
//     Serial.println("Failed to find BNO080!");
//     while (1);
//   }
//   Serial.println("BNO080 Found!");

//   // Enable rotation vector (quaternion + yaw/pitch/roll)
//   myIMU.enableRotationVector(50); // update rate 50Hz

//   calibStart = millis();
//   return true;
// }

// void IMU::update() {
//   if (myIMU.dataAvailable()) {
//     // Get Euler angles directly (from SparkFun library)
//     float yawNow   = myIMU.getYaw() * 180.0f / PI;
//     float pitchNow = myIMU.getPitch() * 180.0f / PI;
//     float rollNow  = myIMU.getRoll() * 180.0f / PI;
//     Serial.println(yawNow);
//     // --- calibration phase ---
//     if (!calibrationDone) {
//       if (millis() - calibStart < 5000) {
//         sum += yawNow;
//         count++;
//       } else {
//         if (count > 0) {
//           yawOffset = sum / count;   // average yaw = offset
//         } else {
//           yawOffset = yawNow;        // fallback
//         }
//         continuousYaw = 0;           // reset unwrap
//         lastYaw = yawNow - yawOffset;
//         calibrationDone = true;
//         Serial.println("Calibration done → yaw starts at 0");
//       }
//     }

//     // always apply offset after calibration
//     float yawCorrected = yawNow - yawOffset;
//     if (yawCorrected > 180) yawCorrected -= 360;
//     else if (yawCorrected < -180) yawCorrected += 360;

//     // unwrap
//     float delta = yawCorrected - lastYaw;
//     if (delta > 180) delta -= 360;
//     else if (delta < -180) delta += 360;

//     continuousYaw += delta;
//     lastYaw = yawCorrected;

//     // store safely
//     if (xSemaphoreTake(imu_Mutex, portMAX_DELAY) == pdTRUE) {
//       this->yaw   = calibrationDone ? continuousYaw : 0; // force 0 until calibration finishes
//       this->pitch = pitchNow;
//       this->roll  = rollNow;
//       xSemaphoreGive(imu_Mutex);
//     }
//   }
// }

// float IMU::getYaw() {
//   float temp = 0;
//   if (xSemaphoreTake(imu_Mutex, portMAX_DELAY) == pdTRUE) {
//     temp = this->yaw;
//     xSemaphoreGive(imu_Mutex);
//   }
//   return temp;
// }

// float IMU::getPitch() {
//   return pitch;
// }

// float IMU::getRoll() {
//   return roll;
// }

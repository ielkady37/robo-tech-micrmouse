#include "IMU.h"
#include <math.h>

SemaphoreHandle_t IMU::imu_Mutex;

namespace {
class ImuLock {
public:
  explicit ImuLock(SemaphoreHandle_t mutex) : mutex(mutex),
      locked(mutex && xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {}
  ~ImuLock() { if (locked) xSemaphoreGive(mutex); }
  explicit operator bool() const { return locked; }
  ImuLock(const ImuLock&) = delete;
  ImuLock& operator=(const ImuLock&) = delete;
private:
  SemaphoreHandle_t mutex;
  bool locked;
};

const char* streamStateName(IMUStreamState state) {
  switch (state) {
    case IMUStreamState::Starting: return "starting";
    case IMUStreamState::Calibrating: return "calibrating";
    case IMUStreamState::Ready: return "ready";
    case IMUStreamState::Recovering: return "recovering";
    case IMUStreamState::Failed: return "failed";
  }
  return "unknown";
}
}

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
  if (taskStarted) return true;  // Never create a second owner of the driver.
  imu_Mutex = xSemaphoreCreateMutex();
  if (!imu_Mutex) return false;

  Serial.begin(115200);
  Wire.begin();

  if (!myIMU.begin_I2C(0x4B)) {   // use I2C addr 0x4B
    Serial.println("Failed to find BNO08x at 0x4B!");
    return false;
  }
  Serial.println("BNO08x Found at 0x4B!");

  // Register a persistent callback destination before configuration can service events.
  myIMU.getSensorEvent(&sensorValue);
  if (!myIMU.enableReport(SH2_ARVR_STABILIZED_RV, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable ARVR report!");
    return false;
  }

  calibStart = millis();
  myIMU.wasReset();  // Consume the expected power-on reset, not a runtime outage.
  reading.calibrating = true;
  diagnostics.state = IMUStreamState::Calibrating;
  lastGoodReportMs = calibStart;
  // Poll independently of the blocking ToF reads and motion/settling delays.
  taskStarted = true;  // Set before the new task can be scheduled.
  if (xTaskCreatePinnedToCore(sampleTask, "IMU sampling", 4096, this,
                             1, nullptr, 1) != pdPASS) {
    taskStarted = false;
    return false;
  }
  return true;
}

void IMU::sampleTask(void* context) {
  IMU* self = static_cast<IMU*>(context);
  while (true) {
    self->update();
    vTaskDelay(1);
  }
}

bool IMU::update() {
  if (!taskStarted) return false;
  {
    ImuLock lock(imu_Mutex);
    diagnostics.lastPollMs = millis();
  }
  // No snapshot lock is held over I2C calls: motion can always read status/brake.
  sensorValue = {};
  bool gotReport = myIMU.getSensorEvent(&sensorValue);
  bool reset = myIMU.wasReset();
  if (reset) {
    startRecovery(true);
    gotReport = false;  // Wait for reports from the newly configured stream.
  } else if ((diagnostics.state == IMUStreamState::Ready ||
              diagnostics.state == IMUStreamState::Calibrating) &&
             uint32_t(millis() - lastGoodReportMs) >= STREAM_TIMEOUT_MS) {
    startRecovery(false);
  }

  bool wasCalibrating = !calibrationDone;
  IMUStreamState before = diagnostics.state;
  bool published = gotReport && processReport();
  if (wasCalibrating && calibrationDone) Serial.println("IMU calibration complete");
  if ((before == IMUStreamState::Recovering || before == IMUStreamState::Failed) &&
      (diagnostics.state == IMUStreamState::Ready || diagnostics.state == IMUStreamState::Calibrating)) {
    Serial.println("IMU stream recovered");
    printDiagnostics();
  }
  serviceRecovery();
  return published;
}

void IMU::startRecovery(bool sensorReset) {
  bool newOutage = false;
  {
    ImuLock lock(imu_Mutex);
    if (sensorReset) {
      ++diagnostics.resetCount;
      haveSensorTimestamp = false;
      confirmationReports = 0;
    }
    if (diagnostics.state != IMUStreamState::Recovering && diagnostics.state != IMUStreamState::Failed) {
      newOutage = true;
      diagnostics.state = IMUStreamState::Recovering;
      diagnostics.recoveryAttempts = 0;
      ++reading.generation;
      confirmationReports = 0;
      lastRestartMs = uint32_t(millis()) - RECOVERY_RETRY_MS;
    }
    reading.valid = false;
  }
  if (sensorReset) Serial.println("IMU reset detected");
  if (newOutage) {
    Serial.println("IMU stream lost");
    printDiagnostics();
  }
}

void IMU::serviceRecovery() {
  if (diagnostics.state != IMUStreamState::Recovering) return;
  if (uint32_t(millis() - lastRestartMs) < RECOVERY_RETRY_MS) return;

  if (diagnostics.recoveryAttempts >= MAX_RECOVERY_ATTEMPTS) {
    {
      ImuLock lock(imu_Mutex);
      diagnostics.state = IMUStreamState::Failed;
    }
    Serial.println("IMU recovery failed -- restart attempts exhausted; still listening");
    printDiagnostics();
    return;
  }
  {
    ImuLock lock(imu_Mutex);
    ++diagnostics.recoveryAttempts;
    ++diagnostics.restartRequests;
    lastRestartMs = millis();
    confirmationReports = 0;
  }
  Serial.print("IMU report restart: attempt ");
  Serial.print(diagnostics.recoveryAttempts);
  Serial.print("/");
  Serial.println(MAX_RECOVERY_ATTEMPTS);
  bool enabled = myIMU.enableReport(SH2_ARVR_STABILIZED_RV, REPORT_INTERVAL_US);
  {
    ImuLock lock(imu_Mutex);
    if (enabled) haveSensorTimestamp = false;
    else ++diagnostics.restartFailures;
  }
  Serial.println(enabled ? "IMU report request accepted; waiting for fresh reports"
                         : "IMU report request failed");
  // A successful configuration call is not proof that heading reports resumed.
}

bool IMU::processReport() {
  ImuLock lock(imu_Mutex);
  if (!lock) return false;
  ++diagnostics.receivedReports;
  if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
      // A driver timestamp can restart or jump on I2C. Reject a repeated report,
      // not every subsequent report whose timestamp is below an old high value.
      if (haveSensorTimestamp && sensorValue.sequence == lastSensorSequence &&
          sensorValue.timestamp == lastSensorTimestampUs) {
        ++diagnostics.duplicateReports;
        confirmationReports = 0;
        return false;
      }
      float qw = sensorValue.un.arvrStabilizedRV.real;
      float qx = sensorValue.un.arvrStabilizedRV.i;
      float qy = sensorValue.un.arvrStabilizedRV.j;
      float qz = sensorValue.un.arvrStabilizedRV.k;
      if (!isfinite(qw) || !isfinite(qx) || !isfinite(qy) || !isfinite(qz)) {
        ++diagnostics.invalidReports;
        confirmationReports = 0;
        return false;
      }
      float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
      if (!isfinite(norm) || norm < 0.001f) {
        ++diagnostics.invalidReports;
        confirmationReports = 0;
        return false;
      }
      qw /= norm;
      qx /= norm;
      qy /= norm;
      qz /= norm;
      // Timestamp fresh acquisitions, never repeated control-loop reads. Extend
      // the local 32-bit micros() clock so rollover cannot reverse sample time.
      uint32_t nowUs = micros();
      if (haveReportClock) {
        uint32_t elapsedUs = nowUs - lastReportUs;
        if (elapsedUs == 0) return false;
        reportTimeUs += elapsedUs;
      } else {
        reportTimeUs = nowUs;
        haveReportClock = true;
      }
      lastReportUs = nowUs;
      lastSensorTimestampUs = sensorValue.timestamp;
      lastSensorSequence = sensorValue.sequence;
      haveSensorTimestamp = true;
      uint32_t nowMs = millis();
      uint32_t reportGapMs = nowMs - lastGoodReportMs;
      if (reportGapMs > diagnostics.maxReportGapMs) diagnostics.maxReportGapMs = reportGapMs;
      lastGoodReportMs = diagnostics.lastAcceptedMs = nowMs;
      ++diagnostics.acceptedReports;

      if (diagnostics.state == IMUStreamState::Recovering || diagnostics.state == IMUStreamState::Failed) {
        if (confirmationReports == 0 || uint32_t(nowMs - confirmationLastMs) > RECOVERY_CONFIRM_GAP_MS) {
          confirmationReports = 0;
          confirmationStartMs = nowMs;
        }
        if (confirmationReports < RECOVERY_CONFIRM_REPORTS) ++confirmationReports;
        confirmationLastMs = nowMs;
        if (confirmationReports >= RECOVERY_CONFIRM_REPORTS &&
            uint32_t(nowMs - confirmationStartMs) >= RECOVERY_CONFIRM_SPAN_MS) {
          diagnostics.state = calibrationDone ? IMUStreamState::Ready : IMUStreamState::Calibrating;
        }
      }

      // quaternion → yaw/pitch/roll
      float yawNow = atan2f(2.0f * (qx * qy + qw * qz),
                            qw * qw + qx * qx - qy * qy - qz * qz)
                     * 180.0f / PI;
      float pitchNow = asinf(constrain(-2.0f * (qx * qz - qw * qy), -1.0f, 1.0f)) * 180.0f / PI;
      float rollNow  = atan2f(2.0f * (qw * qx + qy * qz),
                              qw * qw - qx * qx - qy * qy + qz * qz)
                       * 180.0f / PI;
      // Serial.println("------------ YAW: ");
      // // Serial.println(yawNow);U
      // --- calibration phase ---
      if (!calibrationDone) {
        if (millis() - calibStart < CALIBRATION_MS) {
          sum += yawNow;
          count++;
          return false;  // Calibration readings are not motion measurements.
        } else {
          if (count > 0) {
            yawOffset = sum / count;   // average yaw = offset
          } else {
            yawOffset = yawNow;        // fallback
          }
          continuousYaw = 0;           // reset unwrap
          lastYaw = yawNow - yawOffset;
          calibrationDone = true;
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

      // Report restart preserves yawOffset/continuousYaw and the original calibration.
      {
        this->yaw = continuousYaw;
        this->pitch = pitchNow;
        this->roll  = rollNow;
        reading.yaw = this->yaw;
        reading.timestampUs = reportTimeUs;
        reading.receivedMs = millis();
        ++reading.sequence;
        if (diagnostics.state == IMUStreamState::Calibrating) diagnostics.state = IMUStreamState::Ready;
        reading.valid = diagnostics.state == IMUStreamState::Ready;
        reading.calibrating = false;
        return reading.valid;
      }
  } else {
    ++diagnostics.otherReports;
  }
  return false;
}

IMUDiagnostics IMU::getDiagnostics() {
  ImuLock lock(imu_Mutex);
  return lock ? diagnostics : IMUDiagnostics{};
}

void IMU::printDiagnostics() {
  IMUReading sample;
  IMUDiagnostics stats;
  {
    ImuLock lock(imu_Mutex);
    if (!lock) return;
    sample = reading;
    stats = diagnostics;
  }
  uint32_t nowMs = millis();
  Serial.print("IMU status: state="); Serial.print(streamStateName(stats.state));
  Serial.print(" generation="); Serial.print(sample.generation);
  Serial.print(" sampleAgeMs=");
  if (sample.sequence == 0) Serial.print("none");
  else Serial.print(uint32_t(nowMs - sample.receivedMs));
  Serial.print(" pollAgeMs="); Serial.print(uint32_t(nowMs - stats.lastPollMs));
  Serial.print(" received="); Serial.print(stats.receivedReports);
  Serial.print(" accepted="); Serial.print(stats.acceptedReports);
  Serial.print(" duplicate="); Serial.print(stats.duplicateReports);
  Serial.print(" invalid="); Serial.print(stats.invalidReports);
  Serial.print(" other="); Serial.print(stats.otherReports);
  Serial.print(" resets="); Serial.print(stats.resetCount);
  Serial.print(" attempts="); Serial.print(stats.recoveryAttempts);
  Serial.print(" requestFailures="); Serial.print(stats.restartFailures);
  Serial.print(" maxGapMs="); Serial.println(stats.maxReportGapMs);
}

IMUReading IMU::getReading() {
  IMUReading snapshot;
  if (imu_Mutex && xSemaphoreTake(imu_Mutex, portMAX_DELAY) == pdTRUE) {
    snapshot = reading;
    xSemaphoreGive(imu_Mutex);
  }
  return snapshot;
}

float IMU::getYaw() {
  return getReading().yaw;
}
float IMU::getPitch() {
  float value = 0;
  if (imu_Mutex && xSemaphoreTake(imu_Mutex, portMAX_DELAY) == pdTRUE) {
    value = pitch;
    xSemaphoreGive(imu_Mutex);
  }
  return value;
}
float IMU::getRoll() {
  float value = 0;
  if (imu_Mutex && xSemaphoreTake(imu_Mutex, portMAX_DELAY) == pdTRUE) {
    value = roll;
    xSemaphoreGive(imu_Mutex);
  }
  return value;
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

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Wire.h>

enum class IMUStreamState { Starting, Calibrating, Ready, Recovering, Failed };

struct IMUDiagnostics {
  IMUStreamState state = IMUStreamState::Starting;
  uint32_t receivedReports = 0;
  uint32_t acceptedReports = 0;
  uint32_t duplicateReports = 0;
  uint32_t invalidReports = 0;
  uint32_t otherReports = 0;
  uint32_t resetCount = 0;
  uint32_t restartRequests = 0;
  uint32_t restartFailures = 0;
  uint32_t lastPollMs = 0;
  uint32_t lastAcceptedMs = 0;
  uint32_t maxReportGapMs = 0;
  uint8_t recoveryAttempts = 0;
};

struct IMUReading {
  float yaw = 0;
  uint64_t timestampUs = 0;  // Monotonic acquisition time of this fresh report
  uint32_t receivedMs = 0;
  uint32_t sequence = 0;
  uint32_t generation = 0;  // Changes on an outage/reset, even if recovery is quick.
  bool valid = false;
  bool calibrating = false;
};

class IMU {
public:
  static constexpr uint32_t CALIBRATION_MS = 5000;
  static constexpr uint32_t STREAM_TIMEOUT_MS = 500;
  static constexpr uint32_t RECOVERY_RETRY_MS = 500;
  static constexpr uint8_t MAX_RECOVERY_ATTEMPTS = 3;
  static constexpr uint8_t RECOVERY_CONFIRM_REPORTS = 5;
  static constexpr uint32_t RECOVERY_CONFIRM_GAP_MS = 100;
  static constexpr uint32_t RECOVERY_CONFIRM_SPAN_MS = 40;
  IMU();
  bool begin();
  // Only the sampling task polls the sensor. Consumers read atomic snapshots.
  bool update();
  IMUReading getReading();
  IMUDiagnostics getDiagnostics();
  void printDiagnostics();
  float getYaw();
  float getPitch();
  float getRoll();

private:
  Adafruit_BNO08x myIMU;
  // The driver retains this pointer and can write through it during enableReport().
  sh2_SensorValue_t sensorValue{};
  bool taskStarted = false;

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

  IMUReading reading;
  IMUDiagnostics diagnostics;
  uint64_t lastSensorTimestampUs = 0;
  bool haveSensorTimestamp = false;
  uint8_t lastSensorSequence = 0;
  uint32_t lastReportUs = 0;
  uint64_t reportTimeUs = 0;
  bool haveReportClock = false;
  uint32_t lastGoodReportMs = 0;
  uint32_t lastRestartMs = 0;
  uint32_t confirmationStartMs = 0;
  uint32_t confirmationLastMs = 0;
  uint8_t confirmationReports = 0;
  void startRecovery(bool sensorReset);
  void serviceRecovery();
  bool processReport();
  static void sampleTask(void* context);
  static constexpr uint32_t REPORT_INTERVAL_US = 10000;  // 100 Hz

};

#endif

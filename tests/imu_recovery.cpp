#include <cassert>
#include <iostream>
#include <limits>
#include "host/Arduino.h"
#define private public
#include "../IMU.h"
#undef private
#include "../IMU.cpp"

uint8_t sequence = 0;
sh2_SensorValue_t lastReport;

bool report(IMU& imu, float yaw, uint32_t elapsed = 10) {
  testNowMs += elapsed;
  sh2_SensorValue_t value;
  value.sequence = sequence++;
  value.timestamp = uint64_t(testNowMs) * 1000;
  value.un.arvrStabilizedRV.real = std::cos(yaw * PI / 360.0);
  value.un.arvrStabilizedRV.k = std::sin(yaw * PI / 360.0);
  lastReport = value;
  testImuEvents.push_back(value);
  return imu.update();
}

void silence(IMU& imu, uint32_t duration) {
  uint32_t start = testNowMs;
  while (uint32_t(testNowMs - start) < duration) {
    testNowMs += 10;
    imu.update();
  }
}

int messages(const char* text) {
  int count = 0;
  for (const auto& line : Serial.lines) if (line.find(text) != std::string::npos) ++count;
  return count;
}

int main() {
  IMU imu;
  testOnEnableReport = [&](sh2_SensorValue_t* destination) {
    assert(destination == &imu.sensorValue);
    assert(testMutexDepth == 0); // I2C configuration cannot hold the snapshot mutex.
    imu.getReading();
    *destination = sh2_SensorValue_t{}; // Model a callback during configuration.
  };
  assert(imu.begin());
  assert(!testImuReset && testReportRequests.size() == 1);
  assert(imu.begin());
  assert(testBnoBeginCalls == 1 && testReportRequests.size() == 1);
  while (testNowMs < IMU::CALIBRATION_MS) report(imu, 30);
  assert(imu.getReading().valid);
  assert(report(imu, 75));
  assert(std::fabs(imu.getReading().yaw - 45) < 0.01);
  uint64_t beforeResetTime = imu.getReading().timestampUs;
  long double originalOffset = imu.yawOffset;
  unsigned long originalCalibrationStart = imu.calibStart;

  // Runtime reset disables reports. Restore the stream without zeroing heading.
  testImuReset = true;
  testReportsEnabled = false;
  sequence = 0;
  testNowMs += 10;
  assert(!imu.update());
  assert(!imu.getReading().valid && imu.getReading().generation == 1);
  assert(imu.getDiagnostics().resetCount == 1 && testReportsEnabled);
  assert(testReportRequests.size() == 2);
  for (int i = 0; i < IMU::RECOVERY_CONFIRM_REPORTS - 1; ++i) {
    assert(!report(imu, 75));
    assert(!imu.getReading().valid);
  }
  assert(report(imu, 75));
  assert(imu.getReading().valid && !imu.getReading().calibrating);
  assert(imu.getReading().timestampUs > beforeResetTime);
  assert(std::fabs(imu.getReading().yaw - 45) < 0.01);
  assert(imu.yawOffset == originalOffset && imu.calibStart == originalCalibrationStart);
  assert(messages("IMU calibration complete") == 1);

  // The retained callback destination stays alive after update() returns.
  assert(testCallbackBuffer == &imu.sensorValue);
  testCallbackBuffer->timestamp = 999;

  // Accepted configuration commands with no data are still a failed recovery.
  size_t beforeRequests = testReportRequests.size();
  uint32_t lastGood = imu.getDiagnostics().lastAcceptedMs;
  silence(imu, IMU::STREAM_TIMEOUT_MS + IMU::MAX_RECOVERY_ATTEMPTS * IMU::RECOVERY_RETRY_MS);
  assert(imu.getDiagnostics().state == IMUStreamState::Failed);
  assert(!imu.getReading().valid && imu.getReading().generation == 2);
  assert(testReportRequests.size() == beforeRequests + IMU::MAX_RECOVERY_ATTEMPTS);
  for (int i = 0; i < IMU::MAX_RECOVERY_ATTEMPTS; ++i)
    assert(testReportRequests[beforeRequests + i] == lastGood + IMU::STREAM_TIMEOUT_MS + i * IMU::RECOVERY_RETRY_MS);
  assert(messages("IMU recovery failed") == 1);
  silence(imu, 2000);
  assert(testReportRequests.size() == beforeRequests + IMU::MAX_RECOVERY_ATTEMPTS);
  assert(messages("IMU recovery failed") == 1);

  // Sparse reports are not enough. A confirmed late stream can release Failed.
  for (int i = 0; i < 6; ++i) assert(!report(imu, 75, IMU::RECOVERY_CONFIRM_GAP_MS + 10));
  for (int i = 0; i < IMU::RECOVERY_CONFIRM_REPORTS - 2; ++i) assert(!report(imu, 75));
  // Five reports must also span 40 ms, not just arrive in a burst.
  assert(!report(imu, 75, 1));
  assert(report(imu, 75, 10));
  assert(imu.getDiagnostics().state == IMUStreamState::Ready);
  assert(messages("IMU stream recovered") == 2);

  // Diagnostics distinguish duplicated data, bad quaternions and other reports.
  IMUDiagnostics before = imu.getDiagnostics();
  testNowMs += 10;
  testImuEvents.push_back(lastReport);
  assert(!imu.update());
  auto invalid = lastReport;
  ++invalid.sequence;
  invalid.un.arvrStabilizedRV.real = std::numeric_limits<float>::quiet_NaN();
  testImuEvents.push_back(invalid);
  assert(!imu.update());
  invalid.sensorId = 99;
  testImuEvents.push_back(invalid);
  assert(!imu.update());
  IMUDiagnostics after = imu.getDiagnostics();
  assert(after.receivedReports == before.receivedReports + 3);
  assert(after.acceptedReports == before.acceptedReports);
  assert(after.duplicateReports == before.duplicateReports + 1);
  assert(after.invalidReports == before.invalidReports + 1);
  assert(after.otherReports == before.otherReports + 1);

  // Reset storms cannot replenish the retry budget of an ongoing outage.
  testEnableReportSucceeds = false;
  testReportsEnabled = false;
  beforeRequests = testReportRequests.size();
  for (int i = 0; i < 30; ++i) {
    testImuReset = true;
    testNowMs += 100;
    imu.update();
  }
  assert(imu.getDiagnostics().state == IMUStreamState::Failed);
  assert(imu.getDiagnostics().restartFailures == IMU::MAX_RECOVERY_ATTEMPTS);
  assert(testReportRequests.size() == beforeRequests + IMU::MAX_RECOVERY_ATTEMPTS);
  assert(testBnoBeginCalls == 1 && imu.calibStart == originalCalibrationStart);
  assert(testMutexDepth == 0);
  testOnEnableReport = {};
  std::cout << "PASS: runtime resets, persistent callback storage, heading preservation, bounded retries, confirmation, late recovery, counters, reset storms\n";
}

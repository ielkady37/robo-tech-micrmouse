#include <cassert>
#include <iostream>
#include <limits>
#include "../IMU.cpp"

uint8_t nextSequence = 0;

void event(float yaw, uint64_t timestamp, int sequence = -1) {
  sh2_SensorValue_t value;
  value.timestamp = timestamp;
  value.sequence = sequence < 0 ? nextSequence++ : sequence;
  value.un.arvrStabilizedRV.real = std::cos(yaw * PI / 360.0);
  value.un.arvrStabilizedRV.k = std::sin(yaw * PI / 360.0);
  testImuEvents.push_back(value);
}

int main() {
  IMU imu;
  testNowMs = 1000; // Calibration starts at begin(), not at system boot.
  assert(imu.begin());
  assert(testTask != nullptr && testReportIntervalUs == 10000);
  assert(!imu.getReading().valid);
  assert(imu.getReading().calibrating);
  assert(!imu.update());
  for (testNowMs = 1010; testNowMs < 6000; testNowMs += 10) {
    event(0, uint64_t(testNowMs) * 1000);
    assert(!imu.update());
  }
  assert(!imu.getReading().valid);
  testNowMs = 6000;
  event(0, 6000000);
  assert(imu.update());
  IMUReading first = imu.getReading();
  assert(first.valid && first.receivedMs == 6000 && first.timestampUs == 6000000);
  assert(!first.calibrating);
  assert(!imu.update());
  assert(imu.getReading().sequence == first.sequence);

  // An exact repeated report must not refresh the age or sequence.
  testNowMs = 6100;
  event(90, 6000000, nextSequence - 1);
  assert(!imu.update());
  assert(imu.getReading().receivedMs == 6000);

  // A new report whose driver clock restarted is accepted immediately.
  testNowMs = 6110;
  event(10, 1);
  assert(imu.update());
  assert(imu.getReading().sequence == first.sequence + 1);
  assert(imu.getReading().timestampUs == 6110000);
  testNowMs = 6120;
  event(179, 1); // Sequence changed even though the raw timestamp did not.
  assert(imu.update());
  testNowMs = 6130;
  event(-179, 2);
  assert(imu.update());
  IMUReading unwrapped = imu.getReading();
  assert(std::fabs(unwrapped.yaw - 181) < 0.01);
  assert(unwrapped.timestampUs == 6130000);
  assert(unwrapped.receivedMs == 6130);

  sh2_SensorValue_t invalid;
  invalid.timestamp = 6030000;
  invalid.un.arvrStabilizedRV.real = std::numeric_limits<float>::quiet_NaN();
  testImuEvents.push_back(invalid);
  assert(!imu.update());
  assert(imu.getReading().sequence == unwrapped.sequence);
  invalid.un.arvrStabilizedRV.real = 1;
  invalid.sensorId = 99;
  testImuEvents.push_back(invalid);
  assert(!imu.update());

  // micros() rollover cannot reverse the controller's acquisition clock.
  IMU wrapImu;
  testNowMs = 4289960;
  assert(wrapImu.begin());
  for (testNowMs += 10; testNowMs < 4294960; testNowMs += 10) {
    event(0, uint64_t(testNowMs) * 1000);
    assert(!wrapImu.update());
  }
  event(0, 3);
  assert(wrapImu.update());
  uint64_t beforeWrap = wrapImu.getReading().timestampUs;
  testNowMs += 10;
  event(0, 4);
  assert(wrapImu.update());
  assert(wrapImu.getReading().timestampUs - beforeWrap == 10000);
  std::cout << "PASS: calibration readiness, fresh reports, driver clock restart, duplicate rejection, invalid data, yaw unwrap, micros rollover\n";
}

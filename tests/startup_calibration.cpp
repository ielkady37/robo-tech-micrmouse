#include <cassert>
#include <iostream>
#include "host/Arduino.h"
#define private public
#include "../API.h"
#undef private
#include "../IMU.cpp"
#include "../Robot.cpp"
#include "../API.cpp"
#include "../algorithm.cpp"

// Run the actual IMU publisher and motion controller against the same reports.
uint32_t firstCalibratedReportMs;
bool emitReports;
uint8_t reportSequence;
int motorCommands;
int motorL, motorR;
float physicalHeading, wheelL, wheelR;
uint32_t resetAtMs;

void tick() {
  assert(testNowMs < 40000);
  physicalHeading += (motorL - motorR) * 0.00075f;
  wheelL += motorL * 0.0002f;
  wheelR += motorR * 0.0002f;
  if (resetAtMs && testNowMs == resetAtMs) {
    testImuReset = true;
    testReportsEnabled = false;
    reportSequence = 0;
  }
  if (emitReports && testReportsEnabled && testNowMs % 10 == 0 &&
      (testNowMs < IMU::CALIBRATION_MS || testNowMs >= firstCalibratedReportMs)) {
    sh2_SensorValue_t event;
    event.sequence = reportSequence++;
    // Reproduce a large driver timestamp followed by lower/fixed timestamps.
    event.timestamp = testNowMs < IMU::CALIBRATION_MS ? (1ULL << 60) + testNowMs : 12345;
    event.un.arvrStabilizedRV.real = std::cos(-physicalHeading * PI / 360.0);
    event.un.arvrStabilizedRV.k = std::sin(-physicalHeading * PI / 360.0);
    testImuEvents.push_back(event);
  }
  Robot::imu.update(); // Represents the one sampling task in this host test.
}

void TOF::begin() {}
void TOF::updateReadings() {}
uint16_t TOF::getTofCenter() { return 500; }
uint16_t TOF::getTofLeft() { return 500; }
uint16_t TOF::getTofRight() { return 500; }
void MotorDriver::begin() {}
int MotorDriver::getPosL() { return std::lround(wheelL / 0.2f); }
int MotorDriver::getPosR() { return std::lround(wheelR / 0.2f); }
float MotorDriver::getDistanceL() { return wheelL; }
float MotorDriver::getDistanceR() { return wheelR; }
void MotorDriver::resetEncoderL() { wheelL = 0; }
void MotorDriver::resetEncoderR() { wheelR = 0; }
void MotorDriver::brake() { motorL = motorR = 0; }
void MotorDriver::activeBrake() { brake(); }
void MotorDriver::setMotors(int16_t left, int16_t right) {
  if (left || right) ++motorCommands;
  motorL = left;
  motorR = right;
}

void reset(uint32_t readyAt) {
  testNowMs = 0;
  testTick = tick;
  testImuEvents.clear();
  testReportRequests.clear();
  testEnableReportSucceeds = true;
  Serial.lines.clear();
  Robot::imu = IMU();
  Robot::intendedHeading = 0;
  Robot::imuFaultReported = false;
  API::currentDirection = currentDirection = NORTH;
  API::currentX = API::currentY = currentRow = currentCol = 0;
  firstCalibratedReportMs = readyAt;
  emitReports = true;
  reportSequence = 0;
  motorCommands = 0;
  motorL = motorR = 0;
  physicalHeading = wheelL = wheelR = 0;
  resetAtMs = 0;
}

int messages(const char* text) {
  int count = 0;
  for (const auto& line : Serial.lines) if (line.find(text) != std::string::npos) ++count;
  return count;
}

int main() {
  Robot robot;
  for (uint32_t readyAt : {5000u, 5500u, 8000u}) {
    reset(readyAt);
    robot.begin();
    assert(testNowMs >= readyAt && testNowMs <= readyAt + IMU::RECOVERY_CONFIRM_SPAN_MS);
    assert(Robot::imu.getReading().valid && !Robot::imu.getReading().calibrating);
    assert(motorCommands == 0);
    for (const auto& line : Serial.lines) assert(line.find("ImuTimeout") == std::string::npos);
    assert(robot.turnCardinal(0) == MotionResult::Completed);
    assert(motorCommands == 0);
  }

  // An early motion call gets calibration time; its 3-second deadline starts later.
  reset(5500);
  assert(Robot::imu.begin());
  assert(robot.turnCardinal(0) == MotionResult::Completed);
  assert(testNowMs >= 5500 + 750 && testNowMs < 6500);
  assert(motorCommands == 0);

  // Accept a fresh cached baseline without waiting for another sequence number.
  IMUReading fresh;
  emitReports = false;
  uint32_t before = testNowMs;
  assert(robot.waitForFreshImu(fresh));
  assert(testNowMs == before);
  // But that one cached sample still cannot complete an entire turn.
  assert(robot.turnCardinal(0) == MotionResult::ImuTimeout);
  emitReports = true;
  assert(robot.turnCardinal(0) == MotionResult::Completed);

  // A missing sensor cannot make an individual readiness attempt unbounded.
  reset(5000);
  assert(Robot::imu.begin());
  emitReports = false;
  assert(!robot.waitForFreshImu(fresh));
  assert(testNowMs == IMU_STARTUP_TIMEOUT_MS && motorCommands == 0);

  // A real reset during a moving turn stops that command, even if the stream
  // recovers before the braking delay finishes. Heading/coordinates are not committed.
  reset(5000);
  robot.begin();
  resetAtMs = testNowMs + 100;
  assert(API::turnRight() == MotionResult::ImuTimeout);
  assert(motorCommands > 0 && motorL == 0 && motorR == 0);
  assert(Robot::intendedHeading == 0 && API::currentDirection == NORTH);
  assert(API::currentX == 0 && API::currentY == 0);
  assert(messages("IMU reset detected") == 1 && messages("IMU stream recovered") == 1);
  assert(messages("IMU calibration complete") == 1);
  assert(API::alignHeading() == MotionResult::Completed);
  assert(std::fabs(physicalHeading) <= ERROR_TOL);

  // Navigation retries alignment once after confirmed recovery, rather than
  // repeatedly calling a motion routine while reports are unavailable.
  reset(5000);
  robot.begin();
  targetRow = targetCol = 0; // Finish this test pass after its initial alignment.
  testFlashWrites = 0;
  resetAtMs = testNowMs + 20;
  API api;
  floodFill(api);
  assert(messages("ImuTimeout") == 1);
  floodFill(api);
  assert(messages("Retrying heading alignment") == 1);
  assert(messages("Navigation alignment failed: ImuTimeout") == 0);

  emitReports = false;
  delay(IMU::STREAM_TIMEOUT_MS + IMU::MAX_RECOVERY_ATTEMPTS * IMU::RECOVERY_RETRY_MS);
  assert(Robot::imu.getDiagnostics().state == IMUStreamState::Failed);
  size_t exhaustedRequests = testReportRequests.size();
  int oldFaults = messages("ImuTimeout");
  for (int i = 0; i < 100; ++i) floodFill(api);
  assert(messages("Navigation paused") == 1 && messages("ImuTimeout") == oldFaults);
  assert(testReportRequests.size() == exhaustedRequests);
  assert(motorL == 0 && motorR == 0 && currentRow == 0 && currentCol == 0);

  emitReports = true;
  int waits = 0;
  while (!API::imuReady()) { floodFill(api); assert(++waits <= 10); }
  assert(waits >= IMU::RECOVERY_CONFIRM_REPORTS);
  floodFill(api);
  assert(messages("Retrying heading alignment") == 2);
  assert(testFlashWrites == 0 && API::currentX == 0 && API::currentY == 0);
  assert(Robot::intendedHeading == 0 && messages("IMU calibration complete") == 1);
  std::cout << "PASS: startup/calibration, late reports, live reset during motion, heading preservation, bounded recovery, navigation pause/retry\n";
}

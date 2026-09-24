#include <cassert>
#include <iostream>
#include <tuple>
#include <vector>
#include <string>
#include <queue>
#include "host/Arduino.h"
// Inspect state without adding test-only accessors to the firmware API.
#define private public
#include "../API.h"
#undef private
#include "../Robot.cpp"
#include "../API.cpp"
#include "../algorithm.cpp"

enum class Model { Normal, FrozenYaw, Blocked, BlockForward, FreeOnReverse, FreezeSecondLeft };
Model model;
IMUReading sample;
float physicalHeading, wheelL, wheelR;
int commandL, commandR;
bool reversed, emitSamples, oneReportOnly, scriptedRate, instantStreamRestart;
uint32_t loseImuAt, firstReverseMs, lastDriveMs;
uint16_t frontReading;
std::vector<std::tuple<uint32_t, int, int>> commands;

void publish() {
  sample.yaw = -physicalHeading;
  sample.timestampUs = uint64_t(testNowMs) * 1000;
  sample.receivedMs = testNowMs;
  ++sample.sequence;
  sample.valid = true;
}

void tick() {
  assert(testNowMs < 60000); // Catch accidentally unbounded motion/recovery.
  bool blocked = model == Model::Blocked || (model == Model::FreeOnReverse && !reversed) ||
                 (model == Model::BlockForward && commandL > 0 && commandR > 0);
  if (!blocked) {
    wheelL += commandL * 0.0002f;
    wheelR += commandR * 0.0002f;
    bool frozen = model == Model::FrozenYaw ||
                  (model == Model::FreezeSecondLeft && physicalHeading < -89.0f);
    if (!frozen) physicalHeading += (commandL - commandR) * 0.00075f;
  }
  if (scriptedRate && testNowMs == 20) physicalHeading = 1;
  if (instantStreamRestart && testNowMs == 50) ++sample.generation;
  if (emitSamples && (!loseImuAt || testNowMs < loseImuAt) && testNowMs % 10 == 0 &&
      (!oneReportOnly || testNowMs == 10)) publish();
}

IMU::IMU() {}
bool IMU::begin() { return true; }
IMUReading IMU::getReading() { return sample; }
float IMU::getYaw() { return sample.yaw; }
void IMU::printDiagnostics() { Serial.println("IMU status: simulated"); }
void TOF::begin() {}
void TOF::updateReadings() {}
uint16_t TOF::getTofCenter() { return frontReading; }
uint16_t TOF::getTofLeft() { return 500; }
uint16_t TOF::getTofRight() { return 500; }
void MotorDriver::begin() {}
int MotorDriver::getPosL() { return std::lround(wheelL / 0.2f); }
int MotorDriver::getPosR() { return std::lround(wheelR / 0.2f); }
float MotorDriver::getDistanceL() { return wheelL; }
float MotorDriver::getDistanceR() { return wheelR; }
void MotorDriver::resetEncoderL() { wheelL = 0; }
void MotorDriver::resetEncoderR() { wheelR = 0; }
void MotorDriver::setMotors(int16_t left, int16_t right) {
  assert(std::abs(left) <= MAX_SPEED_FORWARD && std::abs(right) <= MAX_SPEED_FORWARD);
  commandL = left;
  commandR = right;
  commands.emplace_back(testNowMs, left, right);
  if (left || right) lastDriveMs = testNowMs;
  if (left < 0 && right < 0 && !reversed) {
    reversed = true;
    firstReverseMs = testNowMs;
  }
}
void MotorDriver::brake() {
  commands.emplace_back(testNowMs, 0, 0);
  commandL = commandR = 0;
}
void MotorDriver::activeBrake() { brake(); }

int messages(const char* text) {
  int count = 0;
  for (const auto& line : Serial.lines) if (line.find(text) != std::string::npos) ++count;
  return count;
}

void reset(Model next = Model::Normal, float heading = 0) {
  model = next;
  testNowMs = 0;
  testTick = tick;
  physicalHeading = heading;
  wheelL = wheelR = 0;
  commandL = commandR = 0;
  reversed = oneReportOnly = scriptedRate = instantStreamRestart = false;
  emitSamples = true;
  loseImuAt = firstReverseMs = lastDriveMs = 0;
  frontReading = 500;
  sample = IMUReading{};
  publish();
  commands.clear();
  Serial.lines.clear();
  Robot::intendedHeading = 0;
  Robot::imuFaultReported = false;
  API::currentDirection = currentDirection = NORTH;
  API::currentX = API::currentY = currentRow = currentCol = 0;
  for (int r = 0; r < MAZE_LENGTH; ++r)
    for (int c = 0; c < MAZE_WIDTH; ++c) hasNorthWall[r][c] = hasEastWall[r][c] = false;
  testFlashWrites = 0;
}

void stopped() { assert(commandL == 0 && commandR == 0); }

int main() {
  Robot robot;
  API api;

  // Reusing a cached reading must not count as multiple settled samples.
  reset();
  oneReportOnly = true;
  assert(robot.turnCardinal(0) == MotionResult::ImuTimeout);
  stopped();
  assert(!reversed && testNowMs < IMU_TIMEOUT_MS + BRAKE_MS + 20);

  reset();
  assert(robot.turnCardinal(0) == MotionResult::Completed);
  assert(testNowMs >= 800); // At least 50 ms of fresh settling + normal stop delay.

  // PID derivative uses the 10 ms fresh-report interval, not the 1 ms polling loop.
  reset();
  sample.valid = false; // First baseline arrives at 10 ms.
  scriptedRate = true;
  assert(robot.turnCardinal(1) == MotionResult::Completed);
  bool correctDerivative = false;
  for (auto [time, left, right] : commands)
    if (time == 20 && left == 55 && right == -55) correctDerivative = true;
  assert(correctDerivative);

  reset();
  loseImuAt = 80;
  assert(robot.turnCardinal(1) == MotionResult::ImuTimeout);
  stopped();
  assert(!reversed && testNowMs < loseImuAt + IMU_TIMEOUT_MS + BRAKE_MS);
  assert(lastDriveMs < loseImuAt + IMU_TIMEOUT_MS);

  // An outage can recover between two control iterations. Its generation still
  // invalidates the interrupted turn even though the latest snapshot is fresh.
  reset();
  instantStreamRestart = true;
  assert(api.turnRight() == MotionResult::ImuTimeout);
  assert(sample.valid && API::currentDirection == NORTH && Robot::intendedHeading == 0);
  assert(messages("stream restarted during turn") == 1);
  stopped();

  // A fine pulse stops at 20 ms even if reports disappear during that pulse.
  reset(Model::Normal, 5);
  loseImuAt = 70;
  assert(robot.turnCardinal(0) == MotionResult::ImuTimeout);
  bool sawDrive = false, sawBrake = false;
  uint32_t pulseTime = 0;
  for (auto [time, left, right] : commands) {
    if (left || right) { sawDrive = true; pulseTime = time; }
    else if (sawDrive && !sawBrake) {
      assert(time - pulseTime <= TURN_PULSE_MS);
      sawBrake = true;
    }
  }
  assert(sawDrive && sawBrake && !reversed);

  reset();
  emitSamples = false;
  assert(robot.turnCardinal(1) == MotionResult::ImuTimeout);
  assert(lastDriveMs == 0);

  // Encoder movement cannot defeat the absolute turn deadline.
  reset(Model::FrozenYaw);
  assert(api.turnRight() == MotionResult::TurnTimeout);
  assert(testNowMs >= TURN_TIMEOUT_MS && testNowMs < TURN_TIMEOUT_MS + 200);
  assert(API::currentDirection == NORTH && Robot::intendedHeading == 0);
  stopped();
  model = Model::Normal;
  assert(api.turnRight() == MotionResult::Completed);
  assert(std::fabs(physicalHeading - 90) <= ERROR_TOL);
  assert(API::currentDirection == EAST);

  // Drift is corrected against cardinal targets, not accumulated into each turn.
  reset(Model::Normal, 8);
  assert(api.turnRight() == MotionResult::Completed);
  assert(std::fabs(physicalHeading - 90) <= ERROR_TOL);
  physicalHeading = 98;
  publish();
  assert(api.turnRight() == MotionResult::Completed);
  assert(std::fabs(physicalHeading - 180) <= ERROR_TOL);
  assert(API::currentDirection == SOUTH);

  reset(Model::Normal, 8);
  assert(robot.turn(90) == MotionResult::Completed);
  assert(std::fabs(physicalHeading - 98) <= ERROR_TOL); // Relative API still exists.

  // Fine corrections use short PWM pulses and braking, in both directions.
  for (float heading : {5.0f, -5.0f, 359.0f}) {
    reset(Model::Normal, heading);
    assert(robot.snapToCardinal() == MotionResult::Completed);
    assert(std::fabs(physicalHeading - std::round(heading / 90) * 90) <= ERROR_TOL);
    uint32_t pulseStart = 0;
    bool driving = false, sawPulse = false;
    for (auto [time, left, right] : commands) {
      if (left || right) {
        assert(std::abs(left) == MIN_SPEED_ROT && left == -right);
        if (!driving) pulseStart = time;
        driving = sawPulse = true;
      } else if (driving) {
        assert(time - pulseStart <= TURN_PULSE_MS);
        driving = false;
      }
    }
    if (std::fabs(heading) < 10) assert(sawPulse);
  }

  // Stalls recover once but still return Stalled, with no heading commit.
  reset(Model::Blocked);
  assert(api.turnRight() == MotionResult::Stalled);
  assert(messages("backing up") == 1 && reversed);
  assert(API::currentDirection == NORTH && Robot::intendedHeading == 0);
  stopped();

  // An interrupted U-turn retains the first successful quarter-turn only.
  reset(Model::FreezeSecondLeft);
  assert(moveInDirection(SOUTH, api) == MotionResult::TurnTimeout);
  assert(currentDirection == WEST && API::currentDirection == WEST);
  assert(API::currentX == 0 && API::currentY == 0);
  assert(currentRow == 0 && currentCol == 0);

  reset(Model::BlockForward);
  floodFill(api);
  assert(messages("Stalled") >= 1);
  assert(currentRow == 0 && currentCol == 0 && API::currentX == 0 && API::currentY == 0);
  assert(testFlashWrites == 0);
  stopped();

  reset();
  frontReading = 20;
  assert(api.moveForward(1) == MotionResult::Blocked);
  assert(API::currentX == 0 && API::currentY == 0);
  stopped();

  reset();
  assert(api.moveForward(1) == MotionResult::Completed);
  assert(API::currentY == 1);
  stopped();
  std::cout << "PASS: fresh-sample PID/settling, stale IMU, turn deadlines, cardinal/relative targets, fine pulses, recovery, navigation outcomes\n";
}

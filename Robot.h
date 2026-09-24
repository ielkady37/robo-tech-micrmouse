#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include "IMU.h"
#include "motor.h"
#include "Tof.h"
#include "MotionResult.h"

#define THRESHOLD_SIDE 170
#define THRESHOLD_FRONT 145


class Robot {
public:
  void begin(); /* Initialization function were the begin methods of the child objects are called 
                                and the update task is attached to the other processor */

  bool isWallFront();
  bool isWallLeft();
  bool isWallRight();
  // Recovery never changes a failed command into Completed.
  MotionResult move(int cells);
  MotionResult turn(int target);  // Relative turn, for manual motion sequences
  MotionResult turnCardinal(int quarterTurns);  // Maze target: last confirmed heading + N * 90
  // void moveWithHeading(int target);
  void print_all_sensors();  // For debugging purposes
  float calibrateDriftFactor();
  void getDriftFactor();
  MotionResult snapToCardinal();
  bool isImuReady();
private:
  static TOF tof;
  static IMU imu;
  static MotorDriver motor_driver;

  static float intendedHeading;
  static bool imuFaultReported;
  bool waitForFreshImu(IMUReading& reading);
  MotionResult finishMotion(MotionResult result, bool allowRecovery, const char* reason = nullptr);
  MotionResult alignToCardinal(bool allowRecovery);
  MotionResult turnToHeading(float desiredHeading, bool allowRecovery);
  void recoverFromStall();

  static void update(void * parameters);  // An infinite loop for upadting all sensors
};

#endif

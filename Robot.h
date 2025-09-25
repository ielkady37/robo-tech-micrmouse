#ifndef ROBOT_H
#define ROBOT_H

#include <stdint.h>
#include "IMU.h"
#include "motor.h"
#include "Tof.h"

#define THRESHOLD_SIDE 170
#define THRESHOLD_FRONT 70


class Robot {
public:
  void begin(); /* Initialization function were the begin methods of the child objects are called 
                                and the update task is attached to the other processor */

  bool isWallFront();
  bool isWallLeft();
  bool isWallRight();
  void move(int cells);  // The function were the distance PID code and the align to zero codes are supposed to be
  void turn(int target);   // The function were the align to a specific angle is supposed to be
  // void moveWithHeading(int target);
  void print_all_sensors();  // For debugging purposes
  float calibrateDriftFactor();
  void getDriftFactor();
  void snapToCardinal(); 
private:
  static TOF tof;
  static IMU imu;
  static MotorDriver motor_driver;

  static void update(void * parameters);  // An infinite loop for upadting all sensors
  float heading;
};

#endif

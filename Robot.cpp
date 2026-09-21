#include "Arduino.h"
#include "Robot.h"
#include <math.h>
#include "Preferences.h"
#define MIN_SPEED_FORWARD 50
#define MAX_SPEED_FORWARD 65

#define MIN_SPEED_ROT 40
#define MAX_SPEED_ROT 65

#define MAX_ERROR 90
#define DEADZONE 0.7
#define CELL_SIZE 18
const int REQUIRED_STABLE = 2;   // must stay within tolerance 5 times in a row
const float ERROR_TOL = 0.3;
const float RATE_TOL = 10.0;     // deg/s -- must also be near-stationary to call a turn settled
// Preferences prefs;

//Static Variables
TOF Robot::tof;
IMU Robot::imu;
MotorDriver Robot::motor_driver;


void Robot::begin() {
  tof.begin();
  if (!imu.begin()) {
    Serial.println("IMU failed to initialize");
    while (1)
      ;
  }

  motor_driver.begin();
  motor_driver.setMotors(0, 0);

  xTaskCreatePinnedToCore(
    update,              // Function Name
    "Updating Sensors",  // Task Name for debugging
    10000,               // Stack Size
    NULL,                // Parameters
    1,                   // Priority
    NULL,                // Task Handle
    0                    // Core 0
  );
  delay(5000);  // Stabilize
}

// 0 means "no reading yet" (sensor cache starts at 0 before the first ranging
// pass) and 65535 is Tof.cpp's sentinel for an invalid/failed reading -- neither
// is a real distance, so treat both as "no data" rather than "wall right here".
static bool isValidTofReading(uint16_t d) {
  return d != 0 && d != 65535;
}

bool Robot::isWallFront() {
  uint16_t d = tof.getTofCenter();
  if (isValidTofReading(d) && d <= THRESHOLD_FRONT) {
    return true;
  }
  return false;
}

bool Robot::isWallLeft() {
  uint16_t d = tof.getTofLeft();
  if (isValidTofReading(d) && d <= THRESHOLD_SIDE) {
    return true;
  }
  return false;
}

bool Robot::isWallRight() {
  uint16_t d = tof.getTofRight();
  if (isValidTofReading(d) && d <= THRESHOLD_SIDE) {
    return true;
  }
  return false;
}
void Robot::snapToCardinal() {
  imu.update();
  float currentYaw = -imu.getYaw();  // use your convention

  // Find nearest multiple of 90
  int nearestCardinal = round(currentYaw / 90.0) * 90;

  // Compute difference
  float error = nearestCardinal - currentYaw;

  // Use your turn function with PID to correct heading
  if (fabs(error) > ERROR_TOL) {
    turn(error);  // reuse your turn() that accepts relative target
  }
}
void Robot::move(int cells) {
  snapToCardinal();
    motor_driver.resetEncoderL();
    motor_driver.resetEncoderR();
    int targetCm = cells*(CELL_SIZE);
    // Distance PID
    float kp_dist = 1.0;
    float kd_dist = 4.0;

    // Heading PID
    float kp_heading = 0.8;   // was 2, but that value was never actually exercised (heading
                              // correction was dead code until now) -- starting conservative
    float kd_heading = 0.02;  // dt-normalized (deg/s) derivative gain -- re-tune on hardware

    float eprev_dist = 0;
    float eprev_heading = 0;

    int stableCount = 0;
    bool wallDetected = false;
    int wallCloseCount = 0;

    // float startDistance = (motor_driver.getDistanceL() + motor_driver.getDistanceR())/2;
    float startDistance = 0;
    float desiredDistance = startDistance + targetCm;

    imu.update();
    float startYaw = -imu.getYaw();   // record initial heading
    unsigned long lastTime = micros();
    while (true) {
        // --- Update sensors ---
        imu.update();
        float currentDist =(motor_driver.getDistanceL() + motor_driver.getDistanceR())/2;
        float currentYaw  = -imu.getYaw();

        unsigned long now = micros();
        float dt = (now - lastTime) / 1000000.0f;
        lastTime = now;
        if (dt <= 0) dt = 0.001f;  // guard against a zero/degenerate sample

        // Wall safety stop: redefine the target as "here" so the distance PID
        // decelerates and settles smoothly instead of an abrupt motor cutoff.
        // eprev_dist is reset in the same pass so this isn't seen as a derivative spike.
        // Require 2 consecutive close readings so one noisy ToF sample can't trip it.
        if (!wallDetected) {
          uint16_t tofC = tof.getTofCenter();
          if (isValidTofReading(tofC) && tofC < 40) {
            wallCloseCount++;
            if (wallCloseCount >= REQUIRED_STABLE) {
              wallDetected = true;
              desiredDistance = currentDist;
              eprev_dist = 0;
            }
          } else {
            wallCloseCount = 0;
          }
        }

        // --- Distance PID ---
        float error_dist = desiredDistance - currentDist;
        float derv_dist  = error_dist - eprev_dist;
        float pid_dist   = kp_dist * error_dist + kd_dist * derv_dist;

        // Clamp forward speed
        float baseSpeed = constrain(pid_dist, -MAX_SPEED_FORWARD, MAX_SPEED_FORWARD);
        if (wallDetected) baseSpeed = constrain(baseSpeed, 0.0f, (float)MAX_SPEED_FORWARD);  // never back away from a detected wall

        // --- Heading PID ---
        float error_heading = startYaw - currentYaw;
        float derv_heading  = (error_heading - eprev_heading) / dt;  // deg/s
        float pid_heading   = constrain(kp_heading * error_heading + kd_heading * derv_heading,-MAX_SPEED_ROT, MAX_SPEED_ROT);

        // Mix heading correction into motor speeds
        float rightSpeed  = baseSpeed - pid_heading;
        float leftSpeed = baseSpeed + pid_heading;

        // Clamp to motor limits
        leftSpeed  = constrain(leftSpeed, -MAX_SPEED_FORWARD, MAX_SPEED_FORWARD);
        rightSpeed = constrain(rightSpeed, -MAX_SPEED_FORWARD, MAX_SPEED_FORWARD);
        if (wallDetected) {
          // Once stopping for a wall, heading correction may only ease speed toward
          // zero, never drive a wheel in reverse (that reads as "backing away").
          leftSpeed  = constrain(leftSpeed, 0.0f, (float)MAX_SPEED_FORWARD);
          rightSpeed = constrain(rightSpeed, 0.0f, (float)MAX_SPEED_FORWARD);
        }

        if (leftSpeed > 0.7 && leftSpeed <= MIN_SPEED_FORWARD) {
          leftSpeed = MIN_SPEED_FORWARD;
        } else if (leftSpeed < -0.7 && leftSpeed >= -MIN_SPEED_FORWARD) {
          leftSpeed = -MIN_SPEED_FORWARD;
        } else if (leftSpeed >= -0.7 && leftSpeed <= 0.7) {
          leftSpeed = 0;  // deadband zone
        }
        if (rightSpeed > 0.7 && rightSpeed <= MIN_SPEED_FORWARD) {
          rightSpeed = MIN_SPEED_FORWARD;
        } else if (rightSpeed < -0.7 && rightSpeed >= -MIN_SPEED_FORWARD) {
          rightSpeed = -MIN_SPEED_FORWARD;
        } else if (rightSpeed >= -0.7 && rightSpeed <= 0.7) {
          rightSpeed = 0;  // deadband zone
        }
        // Apply trimming / deadband if needed
        if (fabs(leftSpeed) < DEADZONE) leftSpeed = 0;
        if (fabs(rightSpeed) < DEADZONE) rightSpeed = 0;

        motor_driver.setMotors(leftSpeed, rightSpeed);

        // Save errors
        eprev_dist    = error_dist;
        eprev_heading = error_heading;

        // Debug
        Serial.print("|| DistErr: ");
        Serial.print(error_dist);
        Serial.print("|| HeadErr: ");
        Serial.print(error_heading);
        Serial.print("|| Lspeed: ");
        Serial.print(leftSpeed);
        Serial.print("|| Rspeed: ");
        Serial.print(rightSpeed);
        Serial.print("|| TofC: ");
        Serial.println(tof.getTofCenter());

        // Exit condition
        if (fabs(error_dist) < ERROR_TOL) {
            stableCount++;
            if (stableCount >= REQUIRED_STABLE) break;
        } else {
            stableCount = 0;
        }
    }
    motor_driver.resetEncoderL();
    motor_driver.resetEncoderR();
    motor_driver.setMotors(0, 0);
    delay(750);
}



void Robot::turn(int target) {
  float kp = 0.5;
  float kd = 0.06;  // dt-normalized (deg/s) derivative gain -- re-tune on hardware
  float eprev = 0;
  int stableCount = 0;

  float startAngle = -imu.getYaw();
  float desiredHeading = startAngle + target;

  float pidSignal;
  float speed = 0;
  float current;
  float error;
  float derv;
  float angularVelocity;

  unsigned long lastTime = micros();

  motor_driver.setMotors(0, 0);
  while (true) {
    imu.update();

    current = -imu.getYaw();
    error = desiredHeading - current;  // relative to desired heading

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;
    lastTime = now;
    if (dt <= 0) dt = 0.001f;  // guard against a zero/degenerate sample

    derv = (error - eprev) / dt;  // deg/s
    angularVelocity = -derv;      // deg/s, actual turn rate
    pidSignal = kp * error + kd * derv;

    // Scale PID output into motor speed range
    speed = (pidSignal / (kp * MAX_ERROR)) * MAX_SPEED_ROT;
    speed = constrain(speed, -MAX_SPEED_ROT, MAX_SPEED_ROT);

    // --- Trimming logic ---
    if (speed > 0.7 && speed <= MIN_SPEED_ROT) {
      speed = MIN_SPEED_ROT;
    } else if (speed < -0.7 && speed >= -MIN_SPEED_ROT) {
      speed = -MIN_SPEED_ROT;
    } else if (speed >= -0.7 && speed <= 0.7) {
      speed = 0;  // deadband zone
    }

    motor_driver.setMotors(speed, -speed);
    eprev = error;
    Serial.print(" || error: ");
    Serial.print(error);
    Serial.print(" || rate: ");
    Serial.print(angularVelocity);
    Serial.print(" || speed: ");
    Serial.print(speed);
    Serial.print(" || dt(ms): ");
    Serial.println(dt * 1000.0f);

    // Settle only once heading AND rotation rate are both near zero -- otherwise
    // leftover spin momentum coasts the heading past the target after motors cut.
    if (fabs(error) < ERROR_TOL && fabs(angularVelocity) < RATE_TOL) {
      stableCount++;
      if (stableCount >= REQUIRED_STABLE) break;
    } else {
      stableCount = 0;
    }
  }

  motor_driver.setMotors(0, 0);
  delay(750);
}


void Robot::update(void* parameters) {
  while (true) {
    tof.updateReadings();
    imu.getRoll();

    vTaskDelay(0);  // yield
  }
}

void Robot::print_all_sensors() {
  // Serial.print("Yaw: ");
  // Serial.print(imu.getYaw());

  Serial.print(" Left: ");
  Serial.print(tof.getTofLeft());

  Serial.print(" Center: ");
  Serial.print(tof.getTofCenter());

  Serial.print(" Right: ");
  Serial.println(tof.getTofRight());

  // Serial.print(" DistanceL: ");
  // Serial.println(motor_driver.getDistanceL());
  // Serial.print(" DistanceR: ");
  // Serial.println(motor_driver.getDistanceR());
  // Serial.print(" ticks L: ");
  // Serial.println(motor_driver.getPosL());
  // Serial.print(" ticks R: ");
  // Serial.println(motor_driver.getPosR());
  // Serial.println();
}

float Robot::calibrateDriftFactor() {
    // Reset encoders
    motor_driver.resetEncoderL();
    motor_driver.resetEncoderR();

    // Run motors at same speed for 1 second
    motor_driver.setMotors(255, 255);
    delay(5000);
    motor_driver.setMotors(0, 0);

    // Read distances
    float leftDist  = motor_driver.getDistanceL();
    float rightDist = motor_driver.getDistanceR();

    if (rightDist == 0) return 1.0; // avoid divide by zero

    // Factor to balance right vs left
    float factor = leftDist / rightDist;
    Serial.print("Drift factor: ");
    Serial.println(factor);
    
    // --- Save to ESP32 NVS (flash) ---
    // prefs.begin("robot", false);    // namespace = "robot"
    // prefs.putFloat("driftFactor", factor);
    // prefs.putFloat("leftDist", leftDist);
    // prefs.putFloat("rightDist", rightDist);
    // prefs.end();

    return factor; // >1 means left stronger, <1 means right stronger
}
void Robot::getDriftFactor()
{
  // prefs.begin("robot", true); // read-only
  // float driftFactor = prefs.getFloat("driftFactor", 1.0); // default = 1.0
  // float leftDist = prefs.getFloat("leftDist", 1.0); // default = 1.0
  // float rightDist = prefs.getFloat("rightDist", 1.0); // default = 1.0
  // prefs.end();
  
  // Serial.print("Loaded drift factor: ");
  // Serial.println(driftFactor);
  // Serial.println(leftDist);
  // Serial.println(rightDist);
}



// void Robot::move(int numCells) {
//   int target_distance = numCells * CELL_SIZE;

//   float kp_distance = 1.5;
//   float kd_distance = 0;
//   int eprev_distance = 0;  // I think mtb2ash 0 bs 8alban msh ht3ml far2

//   int MAX_CORRECTION = 50;
//   float kp_angle = 0.8;
//   float kd_angle = 0.1;
//   int eprev_angle = 0;

//   int pid_distance = 150;
//   int pid_angle = 0;

//   float starting_angle = -imu.getYaw();
//   motor_driver.resetEncoder();

//   float error_angle = 0;
//   float current_angle;
//   float derv_angle;
//   int correction;

//   int error_distance = target_distance;
//   int current_distance;
//   int derv_distance = 0;
//   int speed = 0;
//   int speed_prev = 0;

//   /*Error angle = zero && Error Distance = 0 && speed = zero*/
//   while (fabs(error_distance) > 1 || fabs(derv_distance) != 0) {

//     // Calculate Correction
//     current_angle = -imu.getYaw();
//     error_angle = starting_angle - current_angle;
//     derv_angle = error_angle - eprev_angle;

//     pid_angle = kp_angle * error_angle + kd_angle * derv_angle;
//     correction = pid_angle;

//     if (correction > MAX_CORRECTION) {
//       correction = MAX_CORRECTION;
//     } else if (correction < -MAX_CORRECTION) {
//       correction = -MAX_CORRECTION;
//     }

//     // Callculate Speed
//     current_distance = motor_driver.getPos();
//     error_distance = target_distance - current_distance;
//     derv_distance = error_distance - eprev_distance;

//     pid_distance = kp_distance * error_distance + kd_distance * derv_distance;

//     if (speed_prev < pid_distance) {
//       speed++;
//     } else {
//       speed--;
//     }

//     if (speed > MAX_SPEED) {
//       speed = MAX_SPEED;
//     } else if (speed < -MAX_SPEED) {
//       speed = -MAX_SPEED;
//     }

//     if (pid_distance > 0) {
//       speed = map(pid_distance, 0, MAX_SPEED * kp_distance, (MIN_SPEED - 5), MAX_SPEED);
//     } else if (pid_distance < 0) {
//       speed = map(pid_distance, -MAX_SPEED * kp_distance, 0, -MAX_SPEED, -(MIN_SPEED - 5));
//     } else {
//       speed = 0;
//     }

//     motor_driver.setMotors((speed + correction) * 0.96, (speed - correction) * 1.05);

//     eprev_angle = error_angle;  // store previous
//     eprev_distance = error_distance;
//     speed_prev = speed;
//   }
//   motor_driver.setMotors(0, 0);
//   delay(350);
//   // turn((int)(-error_angle));
// }


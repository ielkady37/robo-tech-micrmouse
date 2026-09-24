#include "Arduino.h"
#include "Robot.h"
#include <math.h>
#include "Preferences.h"
// Fixed motor speed limits (PWM).
#define MIN_SPEED_FORWARD 50
#define MAX_SPEED_FORWARD 65

#define MIN_SPEED_ROT 43
#define MAX_SPEED_ROT 65

#define MAX_ERROR 90
#define DEADZONE 0.7
#define CELL_SIZE 18
#define BRAKE_MS 120  // how long to hold the short-circuit brake before releasing
const int REQUIRED_STABLE = 2;  // distance / front-wall confirmation
const int TURN_STABLE_SAMPLES = 5;
const float ERROR_TOL = 1.0f;   // degrees
const float RATE_TOL = 5.0f;    // deg/s
const float DIST_TOL = 0.7;      // cm, for move(). Must be >= the PWM deadband below, or the
                                 // motors cut out before the exit condition can ever be met.
// Preferences prefs;

// --- Stall detection ---
#define STALL_SAMPLE_MS   50    // how often to compare tick counts
#define MOTION_TIMEOUT_MS 2000  // recover after this long without encoder motion
#define IMU_TIMEOUT_MS IMU::STREAM_TIMEOUT_MS
#define IMU_STARTUP_TIMEOUT_MS (IMU::CALIBRATION_MS + 2000)
#define TURN_TIMEOUT_MS 3000   // per 90 degrees, minimum one interval
#define TURN_SETTLE_US 50000   // stable for at least 50 ms of measurement time
#define TURN_FINE_ANGLE 10.0f
#define TURN_PULSE_MS 20
#define TURN_PULSE_BRAKE_MS 40
#define TURN_RATE_FILTER_SEC 0.04f

// One bounded recovery attempt before continuing to the next motion.
#define RECOVERY_BACKUP_CM 4.0f
#define RECOVERY_BACKUP_TIMEOUT_MS 1000
#define RECOVERY_ALIGN_TIMEOUT_MS 3000

struct StallMonitor {
  int lastPosL;
  int lastPosR;
  unsigned long lastSampleMs;
  unsigned long lastMotionMs;

  void begin(int posL, int posR, unsigned long nowMs) {
    lastPosL = posL;
    lastPosR = posR;
    lastSampleMs = nowMs;
    lastMotionMs = nowMs;
  }

  // Track encoder motion even when the PID commands zero: an unfinished motion
  // can otherwise wait forever in the deadband. Check each encoder separately
  // because the counters move in opposite directions during a turn.
  bool timedOut(int posL, int posR, unsigned long nowMs) {
    if (nowMs - lastSampleMs < STALL_SAMPLE_MS) return false;

    if (posL != lastPosL || posR != lastPosR) {
      lastMotionMs = nowMs;
    }
    lastPosL = posL;
    lastPosR = posR;
    lastSampleMs = nowMs;

    return (nowMs - lastMotionMs) >= MOTION_TIMEOUT_MS;
  }
};

//Static Variables
TOF Robot::tof;
IMU Robot::imu;
MotorDriver Robot::motor_driver;
float Robot::intendedHeading = 0;
bool Robot::imuFaultReported = false;


void Robot::begin() {
  intendedHeading = 0;  // IMU calibration defines the initial maze north.
  imuFaultReported = false;
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
  Serial.println("Waiting for IMU calibration and first heading...");
  IMUReading reading;
  while (!waitForFreshImu(reading)) {
    // No navigation starts until a calibrated report actually arrives. A late
    // report can release this wait; there is no latched startup failure.
    Serial.println("IMU not ready yet -- waiting for calibrated reports");
    delay(50);
  }
  motor_driver.setMotors(0, 0);
  Serial.println("IMU ready");
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
static bool isFreshImu(const IMUReading& reading, uint32_t nowMs) {
  return reading.valid && !reading.calibrating && isfinite(reading.yaw) &&
         uint32_t(nowMs - reading.receivedMs) < IMU_TIMEOUT_MS;
}

bool Robot::isImuReady() {
  return isFreshImu(imu.getReading(), millis());
}

bool Robot::waitForFreshImu(IMUReading& reading) {
  motor_driver.brake();
  reading = imu.getReading();
  unsigned long timeoutMs = reading.calibrating ? IMU_STARTUP_TIMEOUT_MS : IMU_TIMEOUT_MS;
  unsigned long startMs = millis();
  while (true) {
    reading = imu.getReading();
    // An existing fresh report is a usable baseline. Subsequent PID iterations
    // still require new sequence numbers before updating rate or settling.
    if (isFreshImu(reading, millis())) return true;
    if (millis() - startMs >= timeoutMs) return false;
    delay(1);
  }
}

MotionResult Robot::finishMotion(MotionResult result, bool allowRecovery, const char* reason) {
  motor_driver.brake();
  if (result == MotionResult::ImuTimeout && !imuFaultReported) {
    imuFaultReported = true;
    Serial.print("|| Motion result: ImuTimeout -- ");
    Serial.println(reason ? reason : "heading not ready");
    imu.printDiagnostics();
  }
  delay(BRAKE_MS);
  motor_driver.setMotors(0, 0);
  if (result == MotionResult::Completed) imuFaultReported = false;
  else if (result != MotionResult::ImuTimeout) {
    Serial.print("|| Motion result: ");
    Serial.println(motionResultName(result));
  }
  if (result == MotionResult::Stalled && allowRecovery) recoverFromStall();
  motor_driver.resetEncoderL();
  motor_driver.resetEncoderR();
  return result;
}

MotionResult Robot::snapToCardinal() {
  IMUReading reading;
  if (!waitForFreshImu(reading)) return finishMotion(MotionResult::ImuTimeout, false);
  float target = roundf(-reading.yaw / 90.0f) * 90.0f;
  MotionResult result = turnToHeading(target, true);
  if (result == MotionResult::Completed) intendedHeading = target;
  return result;
}

MotionResult Robot::alignToCardinal(bool allowRecovery) {
  IMUReading reading;
  if (!waitForFreshImu(reading)) return finishMotion(MotionResult::ImuTimeout, false);
  float currentYaw = -reading.yaw;

  // Yaw is continuous, so this also handles negative headings and full revolutions.
  float nearestCardinal = roundf(currentYaw / 90.0f) * 90.0f;

  // Even an already-aligned heading must have fresh, stable rate measurements.
  return turnToHeading(nearestCardinal, allowRecovery);
}

void Robot::recoverFromStall() {
  Serial.println("|| RECOVERY -- backing up");
  motor_driver.brake();
  delay(BRAKE_MS);

  float startL = motor_driver.getDistanceL();
  float startR = motor_driver.getDistanceR();
  unsigned long backupStartMs = millis();
  bool backedUp = false;

  while (millis() - backupStartMs < RECOVERY_BACKUP_TIMEOUT_MS) {
    // Reverse travel decreases the signed encoder distance. Stop each wheel at
    // its own target so one free wheel cannot keep reversing if the other jams.
    bool reverseL = startL - motor_driver.getDistanceL() < RECOVERY_BACKUP_CM;
    bool reverseR = startR - motor_driver.getDistanceR() < RECOVERY_BACKUP_CM;
    if (!reverseL && !reverseR) {
      backedUp = true;
      break;
    }
    motor_driver.setMotors(reverseL ? -MIN_SPEED_FORWARD : 0,
                           reverseR ? -MIN_SPEED_FORWARD : 0);
    delay(5);
  }

  motor_driver.activeBrake();
  delay(BRAKE_MS);
  motor_driver.setMotors(0, 0);
  if (!backedUp) Serial.println("|| RECOVERY -- backup timed out");

  Serial.println("|| RECOVERY -- aligning to nearest 90 degrees");
  // A failed recovery turn must return, not start another recovery recursively.
  MotionResult alignment = alignToCardinal(false);

  motor_driver.brake();
  delay(BRAKE_MS);
  motor_driver.setMotors(0, 0);
  motor_driver.resetEncoderL();
  motor_driver.resetEncoderR();
  Serial.println(backedUp && alignment == MotionResult::Completed ? "|| RECOVERY complete -- command still failed"
                                    : "|| RECOVERY incomplete -- continuing");
}

MotionResult Robot::move(int cells) {
  if (cells <= 0) return MotionResult::Completed;
  // If initial alignment stalls, its recovery replaces this command too.
  MotionResult alignment = turnToHeading(intendedHeading, true);
  if (alignment != MotionResult::Completed) return alignment;
    motor_driver.resetEncoderL();
    motor_driver.resetEncoderR();
    int targetCm = cells*(CELL_SIZE);
    // Distance PID
    float kp_dist = 1.0;
    float kd_dist = 4.0;

    // Heading PID
    float kp_heading = 1;   // was 2, but that value was never actually exercised (heading
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

    IMUReading previous = imu.getReading();
    float startYaw = intendedHeading;

    StallMonitor stall;
    stall.begin(motor_driver.getPosL(), motor_driver.getPosR(), millis());

    while (true) {
        // --- Update sensors ---
        IMUReading reading = imu.getReading();
        if (reading.generation != previous.generation) {
          return finishMotion(MotionResult::ImuTimeout, false, "IMU stream restarted during move");
        }
        if (!isFreshImu(reading, millis())) {
          return finishMotion(MotionResult::ImuTimeout, false, "stale heading during move");
        }
        if (stall.timedOut(motor_driver.getPosL(), motor_driver.getPosR(), millis())) {
          return finishMotion(MotionResult::Stalled, true);
        }
        if (reading.sequence == previous.sequence) {
          delay(1);
          continue;
        }
        if (reading.timestampUs <= previous.timestampUs ||
            reading.timestampUs - previous.timestampUs >= IMU_TIMEOUT_MS * 1000ULL) {
          return finishMotion(MotionResult::ImuTimeout, false, "invalid sample interval during move");
        }
        float currentDist =(motor_driver.getDistanceL() + motor_driver.getDistanceR())/2;
        float currentYaw  = -reading.yaw;

        float dt = (reading.timestampUs - previous.timestampUs) / 1000000.0f;
        previous = reading;

        // Wall safety stop: redefine the target as "here" so the distance PID
        // decelerates and settles smoothly instead of an abrupt motor cutoff.
        // eprev_dist is reset in the same pass so this isn't seen as a derivative spike.
        // Require 2 consecutive close readings so one noisy ToF sample can't trip it.
        if (!wallDetected && desiredDistance - currentDist > DIST_TOL) {
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

        // Clamp forward speed. A forward move never drives in reverse: because the
        // MIN_SPEED_FORWARD floor below snaps any small command up to full speed,
        // correcting an overshoot backwards turns the approach into a bang-bang
        // oscillation instead of a settle.
        float baseSpeed = constrain(pid_dist, 0.0f, (float)MAX_SPEED_FORWARD);

        // --- Heading PID ---
        float error_heading = startYaw - currentYaw;
        float derv_heading  = (error_heading - eprev_heading) / dt;  // deg/s
        float pid_heading   = constrain(kp_heading * error_heading + kd_heading * derv_heading,-MAX_SPEED_ROT, MAX_SPEED_ROT);

        // Mix heading correction into motor speeds
        float rightSpeed  = baseSpeed - pid_heading;
        float leftSpeed = baseSpeed + pid_heading;

        // Clamp to motor limits. Heading correction may slow a wheel to a stop but
        // never reverse it -- a reversed wheel pivots the robot in place instead of
        // steering it, which is what reads as the robot backing away.
        leftSpeed  = constrain(leftSpeed, 0.0f, (float)MAX_SPEED_FORWARD);
        rightSpeed = constrain(rightSpeed, 0.0f, (float)MAX_SPEED_FORWARD);

        if (leftSpeed > DEADZONE && leftSpeed <= MIN_SPEED_FORWARD) {
          leftSpeed = MIN_SPEED_FORWARD;
        } else if (leftSpeed <= DEADZONE) {
          leftSpeed = 0;  // deadband zone
        }
        if (rightSpeed > DEADZONE && rightSpeed <= MIN_SPEED_FORWARD) {
          rightSpeed = MIN_SPEED_FORWARD;
        } else if (rightSpeed <= DEADZONE) {
          rightSpeed = 0;  // deadband zone
        }

        motor_driver.setMotors(leftSpeed, rightSpeed);

        // Save errors
        eprev_dist    = error_dist;
        eprev_heading = error_heading;

        // Debug
        // Serial.print("|| DistErr: ");
        // Serial.print(error_dist);
        // Serial.print("|| HeadErr: ");
        // Serial.print(error_heading);
        // Serial.print("|| Lspeed: ");
        // Serial.print(leftSpeed);
        // Serial.print("|| Rspeed: ");
        // Serial.print(rightSpeed);
        // Serial.print("|| TofC: ");
        // Serial.println(tof.getTofCenter());

        // Done once the target is reached or passed. Testing the signed error (not
        // fabs) means an overshoot ends the move instead of provoking a reverse
        // correction, and DIST_TOL >= DEADZONE guarantees the band is reachable
        // while the motors are still allowed to run.
        if (error_dist <= DIST_TOL) {
            stableCount++;
            if (stableCount >= REQUIRED_STABLE) break;
        } else {
            stableCount = 0;
        }
    }

    // Brake rather than coast. Simply switching the motors off free-wheels the
    // robot past the target; the active brake drives against the remaining
    // momentum until the encoders show the wheels have actually stopped.
    motor_driver.activeBrake();
    delay(BRAKE_MS);
    motor_driver.setMotors(0, 0);  // release the short once the wheels are stopped

    // Reset after the wheels are actually stopped, so coast-down ticks are not
    // counted into the next move's baseline.
    motor_driver.resetEncoderL();
    motor_driver.resetEncoderR();
    delay(750 - BRAKE_MS);
    return wallDetected ? MotionResult::Blocked : MotionResult::Completed;
}



MotionResult Robot::turn(int target) {
  IMUReading reading;
  if (!waitForFreshImu(reading)) return finishMotion(MotionResult::ImuTimeout, false);
  float desiredHeading = -reading.yaw + target;
  MotionResult result = turnToHeading(desiredHeading, true);
  if (result == MotionResult::Completed) {
    intendedHeading = roundf(desiredHeading / 90.0f) * 90.0f;
  }
  return result;
}

MotionResult Robot::turnCardinal(int quarterTurns) {
  float target = intendedHeading + quarterTurns * 90.0f;
  MotionResult result = turnToHeading(target, true);
  if (result == MotionResult::Completed) intendedHeading = target;
  return result;
}

MotionResult Robot::turnToHeading(float desiredHeading, bool allowRecovery) {
  float kp = 0.5;
  float kd = 0.06;  // dt-normalized (deg/s) derivative gain
  IMUReading previous;
  if (!waitForFreshImu(previous)) return finishMotion(MotionResult::ImuTimeout, false);
  // Calibration/startup waiting is not part of the turn's motion deadline.
  unsigned long turnStartMs = millis();

  float initialError = desiredHeading - (-previous.yaw);
  unsigned long turnLimitMs = (unsigned long)(TURN_TIMEOUT_MS *
      fmaxf(1.0f, fabsf(initialError) / 90.0f));
  if (!allowRecovery) turnLimitMs = RECOVERY_ALIGN_TIMEOUT_MS;
  float angularVelocity = 0;
  bool haveRate = false;
  int stableCount = 0;
  uint64_t stableSinceUs = 0;
  enum class PulsePhase { Idle, Driving, Braking };
  PulsePhase pulse = PulsePhase::Idle;
  unsigned long pulseStartMs = 0;

  StallMonitor stall;
  stall.begin(motor_driver.getPosL(), motor_driver.getPosR(), millis());

  while (true) {
    IMUReading reading = imu.getReading();
    unsigned long nowMs = millis();
    // These checks run even when the sensor publishes no new measurements.
    if (reading.generation != previous.generation) {
      return finishMotion(MotionResult::ImuTimeout, false, "IMU stream restarted during turn");
    }
    if (!isFreshImu(reading, nowMs)) {
      return finishMotion(MotionResult::ImuTimeout, false, "stale heading during turn");
    }
    if (nowMs - turnStartMs >= turnLimitMs) return finishMotion(MotionResult::TurnTimeout, false);
    if (stall.timedOut(motor_driver.getPosL(), motor_driver.getPosR(), nowMs)) {
      return finishMotion(MotionResult::Stalled, allowRecovery);
    }

    // End a pulse on wall-clock time, even if the IMU stops reporting mid-pulse.
    if (pulse == PulsePhase::Driving && nowMs - pulseStartMs >= TURN_PULSE_MS) {
      motor_driver.brake();
      pulse = PulsePhase::Braking;
      pulseStartMs = nowMs;
    }
    if (reading.sequence == previous.sequence) {
      delay(1);
      continue;
    }
    if (reading.timestampUs <= previous.timestampUs ||
        reading.timestampUs - previous.timestampUs >= IMU_TIMEOUT_MS * 1000ULL) {
      return finishMotion(MotionResult::ImuTimeout, false, "invalid sample interval during turn");
    }
    float dt = (reading.timestampUs - previous.timestampUs) / 1000000.0f;
    float current = -reading.yaw;
    float error = desiredHeading - current;
    float rawRate = (current - (-previous.yaw)) / dt;
    float alpha = dt / (TURN_RATE_FILTER_SEC + dt);
    angularVelocity = haveRate ? angularVelocity + alpha * (rawRate - angularVelocity) : rawRate;
    haveRate = true;
    previous = reading;

    // Serial.print("|| Heading: ");
    // Serial.print(current);
    // Serial.print(" || Target: ");
    // Serial.print(desiredHeading);
    // Serial.print(" || error: ");
    // Serial.println(error);
    // Serial.print(" || rate: ");
    // Serial.println(angularVelocity);

    // Count distinct measurements, and require a real span of settled time.
    if (fabsf(error) <= ERROR_TOL && fabsf(rawRate) <= RATE_TOL && fabsf(angularVelocity) <= RATE_TOL) {
      if (stableCount == 0) stableSinceUs = reading.timestampUs;
      ++stableCount;
      if (stableCount >= TURN_STABLE_SAMPLES && reading.timestampUs - stableSinceUs >= TURN_SETTLE_US) {
        finishMotion(MotionResult::Completed, false);
        delay(750 - BRAKE_MS);
        return MotionResult::Completed;
      }
    } else {
      stableCount = 0;
    }

    if (fabsf(error) <= ERROR_TOL) {
      motor_driver.brake();
      pulse = PulsePhase::Braking;
      pulseStartMs = nowMs;
    } else if (fabsf(error) <= TURN_FINE_ANGLE) {
      // Brake the main turn before issuing a short fixed-PWM correction.
      if (pulse == PulsePhase::Idle) {
        motor_driver.brake();
        pulse = PulsePhase::Braking;
        pulseStartMs = nowMs;
      }
      if (pulse == PulsePhase::Braking && nowMs - pulseStartMs >= TURN_PULSE_BRAKE_MS &&
          fabsf(rawRate) <= RATE_TOL && fabsf(angularVelocity) <= RATE_TOL) {
        int speed = error > 0 ? MIN_SPEED_ROT : -MIN_SPEED_ROT;
        motor_driver.setMotors(speed, -speed);
        pulse = PulsePhase::Driving;
        pulseStartMs = nowMs;
      }
    } else {
      pulse = PulsePhase::Idle;
      float pidSignal = kp * error - kd * angularVelocity;
      float speed = constrain((pidSignal / (kp * MAX_ERROR)) * MAX_SPEED_ROT,
                              (float)-MAX_SPEED_ROT, (float)MAX_SPEED_ROT);
      if (fabsf(speed) <= DEADZONE) {
        motor_driver.brake();
      } else {
        if (speed > 0 && speed < MIN_SPEED_ROT) speed = MIN_SPEED_ROT;
        if (speed < 0 && speed > -MIN_SPEED_ROT) speed = -MIN_SPEED_ROT;
        motor_driver.setMotors(speed, -speed);
      }
    }
  }
}


void Robot::update(void* parameters) {
  while (true) {
    tof.updateReadings();

    vTaskDelay(1);  // IMU sampling runs independently of these blocking reads.
  }
}

void Robot::print_all_sensors() {
  Serial.print("Yaw: ");
  Serial.println(imu.getYaw() );

  // Serial.print(" Left: ");
  // Serial.print(tof.getTofLeft());

  // Serial.print(" Center: ");
  // Serial.print(tof.getTofCenter());

  // Serial.print(" Right: ");
  // Serial.println(tof.getTofRight());

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

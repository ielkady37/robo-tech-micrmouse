#include "esp32-hal.h"
#include "motor.h"
#include <Arduino.h>
#include <stdint.h>

#define TICKS_PER_REV 56
#define WHEEL_DIA 4

// Active brake tuning
#define BRAKE_PWM        70   // reverse drive strength while killing momentum
#define BRAKE_SAMPLE_MS  10   // encoder sampling period during the brake
#define BRAKE_STOPPED_N  3    // consecutive zero-tick samples that mean "stopped"
#define BRAKE_TIMEOUT_MS 250  // hard limit so a brake can never run away
volatile int MotorDriver::posi = 0;
volatile int MotorDriver::posiR = 0;

void IRAM_ATTR MotorDriver::readEncoderL() {
  int b = digitalRead(ENCBL);
  if (b == HIGH) {
    posi--;
  } else {
    posi++;
  }
}
void IRAM_ATTR MotorDriver::readEncoderR() {
  int b = digitalRead(ENCBR);
  if (b == HIGH) {
    posiR--;
  } else {
    posiR++;
  }
}

void MotorDriver::begin() {
  posiMutex = portMUX_INITIALIZER_UNLOCKED;
  posiRMutex = portMUX_INITIALIZER_UNLOCKED;

  pinMode(ENCAL, INPUT);
  pinMode(ENCBL, INPUT);

  pinMode(ENCAR, INPUT);
  pinMode(ENCBR, INPUT);
  posi = 0;
  attachInterrupt(digitalPinToInterrupt(ENCAL), readEncoderL, RISING);
  posiR = 0;
  attachInterrupt(digitalPinToInterrupt(ENCAR), readEncoderR, RISING);

  pinMode(IN1L, OUTPUT);
  pinMode(IN2L, OUTPUT);
  pinMode(speedL, OUTPUT);

  pinMode(IN1R, OUTPUT);
  pinMode(IN2R, OUTPUT);
  pinMode(speedR, OUTPUT);
}

int MotorDriver::getPosL() {
  int temp;
  portENTER_CRITICAL(&posiMutex);
  temp = posi;
  portEXIT_CRITICAL(&posiMutex);
  return temp;
}
int MotorDriver::getPosR() {
  int temp;
  portENTER_CRITICAL(&posiRMutex);
  temp = posiR;
  portEXIT_CRITICAL(&posiRMutex);
  return temp;
}
float MotorDriver::getDistanceL() {
  return (((float)getPosL() / TICKS_PER_REV) * (PI * WHEEL_DIA));
}
float MotorDriver::getDistanceR() {
  return (((float)getPosR() / TICKS_PER_REV) * (PI * WHEEL_DIA)) ;
}

void MotorDriver::resetEncoderL() {
  portENTER_CRITICAL(&posiMutex);
  posi = 0;
  portEXIT_CRITICAL(&posiMutex);
}
void MotorDriver::resetEncoderR() {
  portENTER_CRITICAL(&posiRMutex);
  posiR = 0;
  portEXIT_CRITICAL(&posiRMutex);
}


void MotorDriver::setDirection(uint8_t lowPin, uint8_t highPin) {
  digitalWrite(lowPin, LOW);
  digitalWrite(highPin, HIGH);
}


void MotorDriver::setMotors(int16_t leftSpeed, int16_t rightSpeed) {
  if (leftSpeed > 0) {
    setDirection(IN1L, IN2L);
  } else if (leftSpeed < 0) {
    setDirection(IN2L, IN1L);
  } else {
    digitalWrite(IN1L, LOW);
    digitalWrite(IN2L, LOW);
  }

  if (rightSpeed > 0) {
    setDirection(IN1R, IN2R);
  } else if (rightSpeed < 0) {
    setDirection(IN2R, IN1R);
  } else {
    digitalWrite(IN1R, LOW);
    digitalWrite(IN2R, LOW);
  }

  analogWrite(speedL, abs(leftSpeed)*0.98);
  analogWrite(speedR, abs(rightSpeed));
}

void MotorDriver::brake() {
  // TB6612FNG short brake: IN1 = IN2 = H pulls both outputs low, shorting the motor
  // through the bridge so its back-EMF is dissipated and the rotor is arrested.
  // PWM level does not matter in this state (datasheet lists it as H/L). Contrast
  // IN1 = IN2 = L, which puts the outputs high-Z and lets the motor free-wheel --
  // that coast is what setMotors(0, 0) does, and where overshoot comes from.
  digitalWrite(IN1L, HIGH);
  digitalWrite(IN2L, HIGH);
  digitalWrite(IN1R, HIGH);
  digitalWrite(IN2R, HIGH);

  analogWrite(speedL, 255);
  analogWrite(speedR, 255);
}

void MotorDriver::activeBrake() {
  // A short brake alone dissipates back-EMF, so its torque falls away as the motor
  // slows and the last few millimetres are barely braked at all. Driving against
  // the direction of travel keeps full torque down to zero -- at the cost that
  // holding it a moment too long drives the robot backwards. So the reverse pulse
  // is closed around the encoders and ends the moment the wheels actually stop.

  // Measure which way each wheel is really turning. The last commanded speed is
  // not usable here: the controller may already be in its deadband commanding
  // zero while the robot is still rolling.
  int prevL = getPosL();
  int prevR = getPosR();
  delay(BRAKE_SAMPLE_MS);
  long dirL = (long)getPosL() - prevL;
  long dirR = (long)getPosR() - prevR;

  if (dirL == 0 && dirR == 0) {  // already stopped, nothing to kill
    brake();
    return;
  }

  // Drive each wheel against its own direction, so this works for a spin (wheels
  // opposing) just as well as for a straight move.
  setMotors(dirL > 0 ? -BRAKE_PWM : (dirL < 0 ? BRAKE_PWM : 0),
            dirR > 0 ? -BRAKE_PWM : (dirR < 0 ? BRAKE_PWM : 0));

  prevL = getPosL();
  prevR = getPosR();
  unsigned long start = millis();
  int stoppedSamples = 0;

  while (millis() - start < BRAKE_TIMEOUT_MS) {
    delay(BRAKE_SAMPLE_MS);
    int nowL = getPosL();
    int nowR = getPosR();
    long dL = (long)nowL - prevL;
    long dR = (long)nowR - prevR;
    prevL = nowL;
    prevR = nowR;

    // A wheel now turning the other way has already given up its momentum; any
    // further reverse drive is just accelerating the robot backwards.
    if ((dirL > 0 && dL < 0) || (dirL < 0 && dL > 0) ||
        (dirR > 0 && dR < 0) || (dirR < 0 && dR > 0)) break;

    if (dL == 0 && dR == 0) {
      if (++stoppedSamples >= BRAKE_STOPPED_N) break;
    } else {
      stoppedSamples = 0;
    }
  }

  brake();  // hold the wheels with a short brake now that the momentum is gone
}

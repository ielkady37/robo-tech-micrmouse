#include "esp32-hal.h"
#include "motor.h"
#include <Arduino.h>
#include <stdint.h>

#define TICKS_PER_REV 60
#define WHEEL_DIA 4
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

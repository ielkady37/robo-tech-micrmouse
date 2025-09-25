#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <freertos/portmacro.h>

#define ENCAL 39   // YELLOW - Left encoder channel A
#define ENCBL 36   // WHITE - Left encoder channel B
#define IN2L 25   // Left Motor IN2 
#define IN1L 33   // Left Motor IN1 
#define speedL 32

#define ENCAR 34   // YELLOW - Right encoder channel A
#define ENCBR 35   // WHITE - Right encoder channel B
#define IN2R 26   // Right Motor IN2
#define IN1R 27   // Right Motor IN1 
#define speedR 14


class MotorDriver {
  public:
    void begin(); 
    void resetEncoderL();
    void resetEncoderR();
    int getPosL();
    int getPosR();
    float getDistanceL();
    float getDistanceR();
    static volatile int posi;
    static volatile int posiR;
    void setMotors(int16_t leftSpeed, int16_t rightSpeed);

  private:
  portMUX_TYPE posiMutex;
  portMUX_TYPE posiRMutex;
  void static IRAM_ATTR  readEncoderL();
  void static IRAM_ATTR  readEncoderR();
  void setDirection( uint8_t lowPin, uint8_t highPin);
};

#endif

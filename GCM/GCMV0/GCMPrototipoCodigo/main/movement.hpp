#include "ToF_Setup.hpp"

#define PIN_STBY 16
#define PIN_BIN1 18
#define PIN_BIN2 17
#define PIN_AIN1 4
#define PIN_AIN2 8
#define PIN_PWMA 9
#define PIN_PWMB 10

#define TURN_PWM 50
#define FORWARD_PWM 80

#define TURN_DELAY 90

extern float previousError;
extern uint32_t previousTime;

extern const float KP;
extern const float KD;

void moveForward(ToFSensor& tof);
void turnLeft();
void turnRight();
void turnBack();
void stopMotors();
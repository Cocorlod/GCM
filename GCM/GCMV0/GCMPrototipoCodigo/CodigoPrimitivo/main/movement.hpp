#pragma once

#include "ToF_Setup.hpp"

#define PIN_STBY 16

#define PIN_BIN1 18
#define PIN_BIN2 17

#define PIN_AIN1 4
#define PIN_AIN2 8

#define PIN_PWMA 9
#define PIN_PWMB 10

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define TURN_PWM 127
#define FORWARD_PWM 140

#define TURN_DELAY 200

extern float previousError;
extern uint32_t previousTime;

extern const float KP;
extern const float KD;

void moveForward(ToFSensor& tof);

void turnLeft();
void turnRight();
void turnBack();

void stopMotors();
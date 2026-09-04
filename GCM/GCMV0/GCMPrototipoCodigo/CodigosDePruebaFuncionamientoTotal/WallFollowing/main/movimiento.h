#pragma once

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define BASE_PWM_LEFT 30
#define BASE_PWM_RIGHT 30
#define TARGET_SPEED_MM_S 80.0f
#define SPEED_LOOP_PERIOD_MS 50
#define KP_LEFT 0.15f
#define KI_LEFT 0.40f
#define KP_RIGHT 0.15f
#define KI_RIGHT 0.40f  
#define MAX_INTEGRAL 300.0f
#define TURN_PWM 90
#define TURN_STOP_DELAY_MS 100

#define PIN_STBY 16
#define PIN_BIN1 18
#define PIN_BIN2 17
#define PIN_AIN1 4
#define PIN_AIN2 8
#define PIN_PWMA 9
#define PIN_PWMB 10

int leftPWM = BASE_PWM_LEFT;
int rightPWM = BASE_PWM_RIGHT;

void stopMotors();
void turn90toLeft();
void turn90toRight();
void updateSpeedControl();
#pragma once

// 1. ADD THE STRUCT DEFINITION HERE
struct StallRecovery {
    bool kicking = false;
    uint32_t kickStartTime = 0;
    uint32_t lowSpeedSince = 0;
};

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define BASE_PWM_LEFT 120
#define BASE_PWM_RIGHT 87
#define TARGET_SPEED_MM_S 80.0f
#define SPEED_LOOP_PERIOD_MS 50
#define KP_LEFT 0.15f
#define KI_LEFT 0.40f
#define KP_RIGHT 0.15f
#define KI_RIGHT 0.40f  
#define MAX_INTEGRAL 300.0f
#define TURN_PWM 140
#define TURN_STOP_DELAY_MS 100

#define LEFT_CAN_REVERSE   false   // left motor physically can't reverse — software works around it
#define STALL_PWM_MIN       60     // below this commanded PWM, don't bother checking for stall
#define STALL_SPEED_MM_S    15.0f  // measured speed below this = "not actually moving"
#define STALL_TIME_MS       200    // how long it must sit below STALL_SPEED_MM_S before we act
#define KICK_PWM             170 
#define KICK_DURATION_MS      60  // reverse PWM used to break the stall (right motor)
#define KICK_RELEASE_MS       30   // zero-PWM unload pulse before the forward slam (non-reversible motors)
#define KICK_FULL_PWM        255  

#define PIN_STBY 16
#define PIN_BIN1 18
#define PIN_BIN2 17
#define PIN_AIN1 4
#define PIN_AIN2 8
#define PIN_PWMA 9
#define PIN_PWMB 10

// Add these to make the variables available to updateSpeedControl()
extern StallRecovery leftStall;
extern StallRecovery rightStall;
extern int leftPWM;
extern int rightPWM;

void stopMotors();
void turn90toLeft();
void turn90toRight();
void updateSpeedControl();
int applyStallRecovery(StallRecovery& stall, int commandedPWM, float measuredSpeed, uint32_t now, bool canReverse);
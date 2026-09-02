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
#define TURN_PWM_SLOW 90 
#define FORWARD_PWM 100

#define TURN_DELAY 300
#define DELAY_STOP_MS 100
#define TURN_TIMEOUT_MS 2000

#define PIN_ENCODER_LEFT_A   6
#define PIN_ENCODER_LEFT_B   13
#define PIN_ENCODER_RIGHT_A  14
#define PIN_ENCODER_RIGHT_B  21

extern float previousError;
extern uint32_t previousTime;

extern const float KP;
extern const float KD;

enum Turn : uint8_t {
  LEFT,
  RIGHT,
  BACK
};

static constexpr float WHEEL_DIAMETER_MM = 20.0f;                
static constexpr float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;

static constexpr float ROBOT_LENGTH_MM = 100.0f;  
static constexpr float ROBOT_WIDTH_MM  = 80.0f;

static constexpr float WHEEL_TRACK_MM  = 75.0f;

static constexpr float ENCODER_CPR_MOTOR_SHAFT = 12.0f;
static constexpr float QUADRATURE_DECODE_MULTIPLIER = 4.0f;

static constexpr float GEAR_RATIO = 4.995f;

static constexpr float COUNTS_PER_WHEEL_REV = ENCODER_CPR_MOTOR_SHAFT * QUADRATURE_DECODE_MULTIPLIER * GEAR_RATIO;

static constexpr float MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / COUNTS_PER_WHEEL_REV;

class Encoder {
  public:
    void begin(uint8_t pinA, uint8_t pinB);

    long getCount() const;
    void reset();
  private:
    uint8_t _pinA = 0;
    uint8_t _pinB = 0;
    
    volatile long _count = 0;
    volatile uint8_t _lastState = 0;

    static void IRAM_ATTR isrHandler(void* arg);
    void IRAM_ATTR handleInterrupt();
};

extern Encoder leftEncoder;
extern Encoder rightEncoder;

void moveForward(ToFSensor& tof);
void turn(Turn dir);
void stopMotors();
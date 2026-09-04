#pragma once

#include "ToF_Setup.hpp"

#define PIN_STBY 16

#define PIN_BIN1 18
#define PIN_BIN2 17

#define PIN_AIN1 4
#define PIN_AIN2 8

#define PIN_PWMA 9
#define PIN_PWMB 10


// ============================================================
// PWM
// ============================================================

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define TURN_PWM 127
#define TURN_PWM_SLOW 90
#define FORWARD_PWM 100

#define TURN_DELAY 300
#define DELAY_STOP_MS 100
#define TURN_TIMEOUT_MS 2000


// ============================================================
// ENCODERS
// ============================================================

#define PIN_ENCODER_LEFT_A   14
#define PIN_ENCODER_LEFT_B   13

#define PIN_ENCODER_RIGHT_A  6
#define PIN_ENCODER_RIGHT_B 21

// Right B is intentionally NOT USED.
// GPIO 21 is not required by the encoder code.


// ============================================================
// WALL FOLLOWING
// ============================================================

extern float previousError;
extern uint32_t previousTime;

extern const float KP;
extern const float KD;


// ============================================================
// TURN ENUM
// ============================================================

enum Turn : uint8_t {
  LEFT,
  RIGHT,
  BACK
};


// ============================================================
// ROBOT GEOMETRY
// ============================================================

static constexpr float WHEEL_DIAMETER_MM = 19.0f;

static constexpr float WHEEL_CIRCUMFERENCE_MM =
    PI * WHEEL_DIAMETER_MM;

static constexpr float ROBOT_LENGTH_MM = 78.0f;

static constexpr float ROBOT_WIDTH_MM = 84.25f;

static constexpr float WHEEL_TRACK_MM = 109.5f;


// ============================================================
// ENCODER PARAMETERS
// ============================================================
//
// Pololu #5101:
//
// 12 CPR is already specified when counting both edges
// of both encoder channels.
//
// Therefore:
//
// LEFT:
//     full A+B quadrature
//     12 * 4.995 counts per wheel revolution
//
// RIGHT:
//     A only
//     half of the full quadrature count rate
//     (12 / 2) * 4.995
//
// Do NOT multiply the left encoder by another 4.
// ============================================================

static constexpr float ENCODER_CPR_MOTOR_SHAFT = 12.0f;

static constexpr float GEAR_RATIO = 4.995f;


// ------------------------------------------------------------
// LEFT ENCODER
// ------------------------------------------------------------

static constexpr float LEFT_COUNTS_PER_WHEEL_REV =
    ENCODER_CPR_MOTOR_SHAFT * GEAR_RATIO;

static constexpr float LEFT_MM_PER_COUNT =
    WHEEL_CIRCUMFERENCE_MM /
    LEFT_COUNTS_PER_WHEEL_REV;


// ------------------------------------------------------------
// RIGHT ENCODER
// ------------------------------------------------------------
//
// A only + CHANGE = half of full quadrature resolution.
// ------------------------------------------------------------

static constexpr float RIGHT_COUNTS_PER_WHEEL_REV =
    (ENCODER_CPR_MOTOR_SHAFT / 2.0f) * GEAR_RATIO;

static constexpr float RIGHT_MM_PER_COUNT =
    WHEEL_CIRCUMFERENCE_MM /
    RIGHT_COUNTS_PER_WHEEL_REV;


// ============================================================
// ENCODER CLASS
// ============================================================

class Encoder {

  public:

    // Full A+B quadrature encoder
    void begin(uint8_t pinA, uint8_t pinB);

    // A-only encoder
    void beginAOnly(uint8_t pinA);

    long getCount() const;

    void reset();


  private:

    uint8_t _pinA = 0;
    uint8_t _pinB = 0;

    bool _aOnly = false;

    volatile long _count = 0;

    volatile uint8_t _lastState = 0;


    static void IRAM_ATTR isrHandler(void* arg);

    void IRAM_ATTR handleInterrupt();

    void IRAM_ATTR handleAOnlyInterrupt();
};


extern Encoder leftEncoder;

extern Encoder rightEncoder;


// ============================================================
// MOVEMENT FUNCTIONS
// ============================================================

void moveForward(ToFSensor& tof);

void turn(Turn dir);

void stopMotors();
#pragma once

#include "tof.h"

#define PIN_ENCODER_LEFT_A 14
#define PIN_ENCODER_LEFT_B 13
#define PIN_ENCODER_RIGHT_A 6
#define PIN_ENCODER_RIGHT_B 21

static constexpr float WHEEL_DIAMETER_MM = 19.0f;
static constexpr float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;
static constexpr float WHEEL_TRACK_MM = 109.5f;
static constexpr float ENCODER_CPR_MOTOR_SHAFT = 12.0f;
static constexpr float GEAR_RATIO = 4.995f;
static constexpr float LEFT_COUNTS_PER_WHEEL_REV = ENCODER_CPR_MOTOR_SHAFT * GEAR_RATIO * 4.0f;
static constexpr float RIGHT_COUNTS_PER_WHEEL_REV = ENCODER_CPR_MOTOR_SHAFT * GEAR_RATIO * 2.0f;
static constexpr float LEFT_MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / LEFT_COUNTS_PER_WHEEL_REV;
static constexpr float RIGHT_MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / RIGHT_COUNTS_PER_WHEEL_REV;
static constexpr float TURN_DISTANCE_MM_180 = PI * WHEEL_TRACK_MM;
static constexpr float TURN_DISTANCE_MM_90 = (PI * WHEEL_TRACK_MM) / 2;
static constexpr long RIGHT_TURN_COUNTS_180 = (long)(TURN_DISTANCE_MM_180 / RIGHT_MM_PER_COUNT);
static constexpr long LEFT_TURN_COUNTS_90 = (long)(TURN_DISTANCE_MM_90 / LEFT_MM_PER_COUNT);
static constexpr long RIGHT_TURN_COUNTS_90 = (long)(TURN_DISTANCE_MM_90 / RIGHT_MM_PER_COUNT);

class Encoder {
public:
    void begin(uint8_t pinA, uint8_t pinB);
    void beginAOnly(uint8_t pinA);
    long getCount() const;

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

struct PIController {
    float kp;
    float ki;
    float integral = 0.0f;
};

PIController leftPI = { KP_LEFT, KI_LEFT };
PIController rightPI = { KP_RIGHT, KI_RIGHT };

long lastLeftControlCount = 0;
long lastRightControlCount = 0;
long lastLeftSerialCount = 0;
long lastRightSerialCount = 0;
long turnStartLeftCount = 0;
long turnStartRightCount = 0;

uint32_t lastSpeedControlTime = 0;

float leftSpeed = 0.0f;
float rightSpeed = 0.0f;

void resetController();

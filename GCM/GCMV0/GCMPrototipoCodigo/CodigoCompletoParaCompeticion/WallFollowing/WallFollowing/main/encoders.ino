#include "encoders.h"

void Encoder::begin(uint8_t pinA, uint8_t pinB) {
    _pinA = pinA;
    _pinB = pinB;
    _aOnly = false;

    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    _lastState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    _count = 0;

    attachInterruptArg(digitalPinToInterrupt(_pinA), isrHandler, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_pinB), isrHandler, this, CHANGE);
}

void Encoder::beginAOnly(uint8_t pinA) {
    _pinA = pinA;
    _pinB = 0;
    _aOnly = true;

    pinMode(_pinA, INPUT_PULLUP);
    _count = 0;

    attachInterruptArg(digitalPinToInterrupt(_pinA), isrHandler, this, CHANGE);
}

long Encoder::getCount() const {
    noInterrupts();
    long count = _count;
    interrupts();
    return count;
}

void IRAM_ATTR Encoder::isrHandler(void* arg) {
    Encoder* encoder = static_cast<Encoder*>(arg);

    if (encoder->_aOnly) encoder->handleAOnlyInterrupt();
    else encoder->handleInterrupt();
}

void IRAM_ATTR Encoder::handleInterrupt() {
    static const int8_t table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };

    uint8_t a = digitalRead(_pinA);
    uint8_t b = digitalRead(_pinB);
    uint8_t state = (a << 1) | b;
    uint8_t idx = ((_lastState << 2) | state) & 0x0F;

    _count += table[idx];
    _lastState = state;
}

void IRAM_ATTR Encoder::handleAOnlyInterrupt() {
    _count++;
}

float updatePI(PIController& pi, float targetSpeed, float measuredSpeed, float dt) {
    float error = targetSpeed - measuredSpeed;
    pi.integral += error * dt;
    pi.integral = constrain(pi.integral, -MAX_INTEGRAL, MAX_INTEGRAL);
    return pi.kp * error + pi.ki * pi.integral;
}

void resetController() {
    leftPI.integral = 0.0f;
    rightPI.integral = 0.0f;
    leftPWM = BASE_PWM_LEFT;
    rightPWM = BASE_PWM_RIGHT;
    resetTofController();
    lastLeftControlCount = leftEncoder.getCount();
    lastRightControlCount = rightEncoder.getCount();
    lastSpeedControlTime = millis();
    lastTofControlTime = millis();
}
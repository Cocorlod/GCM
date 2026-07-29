#include "movement.hpp"

float previousError = 0.0f;
uint32_t previousTime = 0;

const float KP = 1.2f;
const float KD = 0.08f;

static constexpr int16_t MAX_PLAUSIBLE_ALIGNMENT_ERROR = 150;

static bool wasWallFollowing = false;

void moveForward(ToFSensor& tof) {
    bool leftWall  = tof.isThereWall(WALL_LEFT);
    bool rightWall = tof.isThereWall(WALL_RIGHT);

    if (!leftWall && !rightWall) {
        digitalWrite(PIN_STBY, HIGH);

        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);

        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);

        ledcWrite(PIN_PWMA, FORWARD_PWM);
        ledcWrite(PIN_PWMB, FORWARD_PWM);

        wasWallFollowing = false;
        return;
    }

    float error;

    if (leftWall && rightWall) {
        error = (tof.alignmentError(WALL_LEFT) - tof.alignmentError(WALL_RIGHT)) * 0.5f;
    }
    else if (leftWall) {
        error = tof.alignmentError(WALL_LEFT);
    }
    else {
        error = -tof.alignmentError(WALL_RIGHT);
    }

    error = constrain(error, -MAX_PLAUSIBLE_ALIGNMENT_ERROR, MAX_PLAUSIBLE_ALIGNMENT_ERROR);

    uint32_t now = micros();

    if (!wasWallFollowing) {
        previousError = error;
        previousTime = now;
        wasWallFollowing = true;
    }

    float dt = max((now - previousTime) * 1e-6f, 0.001f);

    if (dt < 0.001f)
        dt = 0.001f;

    float derivative = (error - previousError) / dt;

    float correction = KP * error + KD * derivative;

    int leftPWM  = constrain((int)(FORWARD_PWM - correction), 0, 255);
    int rightPWM = constrain((int)(FORWARD_PWM + correction), 0, 255);

    digitalWrite(PIN_STBY, HIGH);

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);

    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);

    ledcWrite(PIN_PWMA, leftPWM);
    ledcWrite(PIN_PWMB, rightPWM);

    previousError = error;
    previousTime = now;
}

void turn(Turn turn) {
    switch(turn) {
        case LEFT: {
            digitalWrite(PIN_STBY, HIGH);
            digitalWrite(PIN_BIN1, HIGH);
            digitalWrite(PIN_BIN2, LOW);
            digitalWrite(PIN_AIN1, LOW);
            digitalWrite(PIN_AIN2, LOW);
            ledcWrite(PIN_PWMA, TURN_PWM);
            ledcWrite(PIN_PWMB, TURN_PWM);
            break;
        }
        case RIGHT: {
            digitalWrite(PIN_STBY, HIGH);
            digitalWrite(PIN_BIN1, LOW);
            digitalWrite(PIN_BIN2, LOW);
            digitalWrite(PIN_AIN1, HIGH);
            digitalWrite(PIN_AIN2, LOW);
            ledcWrite(PIN_PWMA, TURN_PWM + 10);
            ledcWrite(PIN_PWMB, TURN_PWM + 10);
            break;
        }
        case BACK: {
            digitalWrite(PIN_STBY, HIGH);
            digitalWrite(PIN_BIN1, LOW);
            digitalWrite(PIN_BIN2, LOW);
            digitalWrite(PIN_AIN1, HIGH);
            digitalWrite(PIN_AIN2, LOW);    
            ledcWrite(PIN_PWMA, TURN_PWM + 34);
            ledcWrite(PIN_PWMB, TURN_PWM + 34);
            break;
        }
    }
}

void stopMotors() {
    digitalWrite(PIN_STBY, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}
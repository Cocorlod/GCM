#include "FSM.hpp"

float previousError = 0.0f;
uint32_t previousTime = 0;

const float KP = 1.2f;
const float KD = 0.08f;

static constexpr int16_t MAX_PLAUSIBLE_ALIGNMENT_ERROR = 150;

static bool wasWallFollowing = false;

void moveForward(ToFSensor& tof) {
    bool leftWall = tof.isThereWall(LEFT);
    bool rightWall = tof.isThereWall(RIGHT);

    if(!leftWall && !rightWall) {
        digitalWrite(PIN_STBY, HIGH);
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);    
        ledcWrite(PIN_PWMA, FORWARD_PWM);
        ledcWrite(PIN_PWMB, FORWARD_PWM);

        if(wasWallFollowing) {
            previousError = 0.0f;
            previousTime = millis();
            wasWallFollowing = false;
        }

        return;
    }

    float error = 0.0f;

    if(leftWall && rightWall) {
        error = (float)(tof.alignmentError(LEFT) - tof.alignmentError(RIGHT)) / 2.0f;
    }
    else if(leftWall) {
        error = (float)tof.alignmentError(LEFT);
    }
    else {
        error = -(float)tof.alignmentError(RIGHT);
    }

    error = constrain(error, -MAX_PLAUSIBLE_ALIGNMENT_ERROR, MAX_PLAUSIBLE_ALIGNMENT_ERROR);

    uint32_t now = millis();
    float dt = (now - previousTime) / 1000.0f;

    if(dt <= 0.0f || !wasWallFollowing) dt = 0.001f;

    float derivative = (error - previousError) / dt;
    float correction = KP * error + KD * derivative;

    int leftPWM = constrain(FORWARD_PWM - correction, 0, 255);
    int rightPWM = constrain(FORWARD_PWM + correction, 0, 255);

    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PIN_PWMA, leftPWM);
    ledcWrite(PIN_PWMB, rightPWM);

    previousError = error;
    previousTime = now;
    wasWallFollowing = true;
}

void turnLeft() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PIN_PWMA, TURN_PWM);
    ledcWrite(PIN_PWMB, TURN_PWM);
}

void turnRight() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    ledcWrite(PIN_PWMA, TURN_PWM);
    ledcWrite(PIN_PWMB, TURN_PWM);
}

void turnBack() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);    
    ledcWrite(PIN_PWMA, TURN_PWM);
    ledcWrite(PIN_PWMB, TURN_PWM);
}

void stopMotors() {
    digitalWrite(PIN_STBY, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}

void executeMove(TurnDecision decision, Heading& heading) {
    switch(decision) {
        case GO_FORWARD:
            moveForward(tof);
            break;  
        case TURN_LEFT:
            stopMotors();
            turnLeft();
            delay(TURN_DELAY);
            stopMotors();
            heading = rotateLeft(heading);
            moveForward(tof);
            break;
        case TURN_RIGHT:
            stopMotors();
            turnRight();
            delay(TURN_DELAY);
            stopMotors();
            heading = rotateRight(heading);
            moveForward(tof);
            break;
        case TURN_BACK:
            stopMotors();
            turnBack();
            delay(2 * TURN_DELAY);
            stopMotors();
            heading = rotateBack(heading);
            moveForward(tof);
            break;
        case NO_MOVE:
            stopMotors();
            break;
    }
}
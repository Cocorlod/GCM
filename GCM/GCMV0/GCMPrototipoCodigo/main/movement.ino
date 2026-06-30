#include "movement.hpp"
#include "algorithmicResolution.hpp"
#include "ToF_Setup.hpp"

void moveForward(ToFSensor& tof) {
    float error = 0.0f;

    if(tof.isThereWall(LEFT) && tof.isThereWall(RIGHT)) {
        error = (float)(tof.alignmentError(LEFT) - tof.alignmentError(RIGHT)) / 2.0f;
    }
    else if(tof.isThereWall(LEFT)) {
        error = (float)tof.alignmentError(LEFT);
    }
    else if(tof.isThereWall(RIGHT)) {
        error = -(float)tof.alignmentError(RIGHT);
    } else {
        digitalWrite(PIN_STBY, HIGH);
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        analogWrite(PIN_PWMA, FORWARD_PWM);
        analogWrite(PIN_PWMB, FORWARD_PWM);
        
        previousError = 0;
        previousTime = millis();

        return;
    }

    uint32_t now = millis();

    float dt = (now - previousTime) / 1000.0f;

    if(dt <= 0.0f) dt = 0.001f;

    float derivative = (error - previousError) / dt;
    float correction = KP * error + KD * derivative;

    int leftPWM = constrain(FORWARD_PWM - correction, 0, 255);
    int rightPWM = constrain(FORWARD_PWM + correction, 0, 255);

    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, leftPWM);
    analogWrite(PIN_PWMB, rightPWM);

    previousError = error;
    previousTime = now;
}

void turnLeft() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, TURN_PWM);
    analogWrite(PIN_PWMB, TURN_PWM);
}

void turnRight() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, TURN_PWM);
    analogWrite(PIN_PWMB, TURN_PWM);
}

void turnBack() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, TURN_PWM);
    analogWrite(PIN_PWMB, TURN_PWM);
}

void stopMotors() {
    digitalWrite(PIN_STBY, LOW);
    analogWrite(PIN_PWMA, 0);
    analogWrite(PIN_PWMB, 0);
}

void executeMove(TurnDecision decision, Heading& heading) {
    switch(decision) {
        case GO_FORWARD:
            moveForward(tof);
            break;  
        case TURN_LEFT:
            turnLeft();
            delay(TURN_DELAY);
            stopMotors();
            heading = rotateLeft(heading);
            moveForward(tof);
            break;
        case TURN_RIGHT:
            turnRight();
            delay(TURN_DELAY);
            stopMotors();
            heading = rotateRight(heading);
            moveForward(tof);
            break;
        case TURN_BACK:
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


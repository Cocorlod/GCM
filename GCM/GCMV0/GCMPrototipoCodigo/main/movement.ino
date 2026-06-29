/*
Motor control
PD
*/

#include "movement.hpp"
#include "algorithmicResolution.hpp"

void moveForward() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, FORWARD_PWM);
    analogWrite(PIN_PWMB, FORWARD_PWM);
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
            moveForward();
            break;  
        case TURN_LEFT:
            turnLeft();
            delay(TURN_DELAY);
            stopMotors();
            heading = rotateLeft(heading);
            moveForward();
            break;
        case TURN_RIGHT:
            turnRight();
            delay(TURN_DELAY);
            stopMotors();
            heading = rotateRight(heading);
            moveForward();
            break;
        case TURN_BACK:
            turnBack();
            delay(2 * TURN_DELAY);
            stopMotors();
            heading = rotateBack(heading);
            moveForward();
            break;
        case NO_MOVE:
            stopMotors();
            break;
    }
}
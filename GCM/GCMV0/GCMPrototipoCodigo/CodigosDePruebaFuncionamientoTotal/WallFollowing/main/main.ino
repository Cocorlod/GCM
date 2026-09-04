#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include "tof.h"
#include "movimiento.h"
#include "encoders.h"
#include "bluetooth.h"

Encoder leftEncoder;
Encoder rightEncoder;

#define PIN_BUTTON 47

#define IR_PIN 15
#define IR_THRESHOLD 2500

#define TURNING_TO_MOVING 1000

enum RobotState { MOVING, TURNING, STOPPED };

RobotState robotState = STOPPED;

bool goalDetected() {
    static uint8_t consecutiveHits = 0;
    static constexpr uint8_t REQUIRED_CONSECUTIVE = 4;

    if (analogRead(IR_PIN) >= IR_THRESHOLD) {
        if (consecutiveHits < 255) consecutiveHits++;
    } else {
        consecutiveHits = 0;
    }

    return consecutiveHits >= REQUIRED_CONSECUTIVE;
}

void determineTurnDirection() {
  if(!isThereWall(WALL_LEFT)) {
    turn90toLeft();
    robotState = MOVING;
  } 
  else if(!isThereWall(WALL_RIGHT)) {
    turn90toRight();
    robotState = MOVING;
  } else {
      turnBack();
      robotState = MOVING;
    }

  if(isThereWall(WALL_LEFT) && isThereWall(WALL_RIGHT)) {
    turnBack();
    robotState = MOVING;
  }
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_BUTTON, INPUT_PULLUP);

    pinMode(PIN_STBY, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RESOLUTION);

    stopMotors();

    leftEncoder.begin(PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B);
    rightEncoder.beginAOnly(PIN_ENCODER_RIGHT_A);

    setupToF();
    resetController();
}

void loop() {
  if(robotState == STOPPED) {
    stopMotors();
    if(digitalRead(PIN_BUTTON) == LOW) {
      delay(30);
      if(digitalRead(PIN_BUTTON) == LOW) {
        resetController();
        robotState = MOVING;
      }
    }    
  }

  if(robotState == MOVING) {
    if(goalDetected()) {
      robotState = STOPPED;
      stopMotors();
      return;
    }

    if(frontWallDetected()) {
      stopMotors();
      robotState = TURNING;
      turnStartLeftCount = leftEncoder.getCount();
      turnStartRightCount = rightEncoder.getCount();
      return;
    }

    delay(TURNING_TO_MOVING);

    updateTofControl();
    updateSpeedControl();

    return;
  }

  if(robotState == TURNING) {
    stopMotors();
    determineTurnDirection();
    return;
  }
}
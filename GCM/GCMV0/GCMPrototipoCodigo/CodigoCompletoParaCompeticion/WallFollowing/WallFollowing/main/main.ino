//copia
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
#define IR_THRESHOLD 250

#define TURNING_TO_MOVING 1000

enum RobotState { MOVING, TURNING, STOPPED };

RobotState robotState = STOPPED;

enum TurnDecision { GO_STRAIGHT, TURN_LEFT_D, TURN_RIGHT_D, TURN_BACK_D };
TurnDecision pendingDecision = GO_STRAIGHT;

long cellStartLeftCount = 0;
long cellStartRightCount = 0;

#define CELL_LENGTH_MM 250.0f  // set to your maze's actual cell pitch — measure it, don't guess

bool goalDetected() {
    static uint8_t consecutiveHits = 0;
    static constexpr uint8_t REQUIRED_CONSECUTIVE = 4;

    if (analogRead(IR_PIN) <= IR_THRESHOLD) {
        if (consecutiveHits < 255) consecutiveHits++;
    } else {
        consecutiveHits = 0;
    }

    return consecutiveHits >= REQUIRED_CONSECUTIVE;
}

TurnDecision decideNextMove() {
    if (!isThereWall(WALL_LEFT))  return TURN_LEFT_D;   // left-hand rule: opening on the left always wins
    if (!isThereWall(WALL_FRONT)) return GO_STRAIGHT;
    if (!isThereWall(WALL_RIGHT)) return TURN_RIGHT_D;
    return TURN_BACK_D;                                  // boxed in on 3 sides
}

void executeTurn(TurnDecision decision) {
    switch (decision) {
        case TURN_LEFT_D:  turn90toLeft();  break;
        case TURN_RIGHT_D: turn90toRight(); break;
        case TURN_BACK_D:  turnBack();      break;
        case GO_STRAIGHT: break;
    }
    readToFSensors();
    resetController();
    cellStartLeftCount  = leftEncoder.getCount();
    cellStartRightCount = rightEncoder.getCount();
    robotState = MOVING;
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
        cellStartLeftCount  = leftEncoder.getCount();
        cellStartRightCount = rightEncoder.getCount();
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

    long dl = labs(leftEncoder.getCount()  - cellStartLeftCount);
    long dr = labs(rightEncoder.getCount() - cellStartRightCount);
    float traveledMM = (dl * LEFT_MM_PER_COUNT + dr * RIGHT_MM_PER_COUNT) * 0.5f;

    if (traveledMM >= CELL_LENGTH_MM || frontWallDetected()) {
        TurnDecision decision = decideNextMove();

        if (decision == GO_STRAIGHT) {
            cellStartLeftCount  = leftEncoder.getCount();
            cellStartRightCount = rightEncoder.getCount();
        } else {
            stopMotors();
            turnStartLeftCount  = leftEncoder.getCount();
            turnStartRightCount = rightEncoder.getCount();
            pendingDecision = decision;
            robotState = TURNING;
            return;
        }
    }

    updateTofControl();
    updateSpeedControl();
    return;
}

  if(robotState == TURNING) {
    stopMotors();
    executeTurn(pendingDecision);
    return;
}
}
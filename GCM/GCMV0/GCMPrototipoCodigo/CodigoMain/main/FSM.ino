#include "FSM.hpp"

Heading heading = NORTH;

void executeMove(TurnDecision decision, Heading& heading) {
    switch(decision) {
        case GO_FORWARD:
            moveForward(tof);
            break;  
        case TURN_LEFT:
            stopMotors();
            turn(LEFT_T);
            delay(TURN_DELAY);
            rotate(LEFT, heading);
            stopMotors();
            moveForward(tof);
            break;
        case TURN_RIGHT:
            stopMotors();
            turn(RIGHT_T);
            delay(TURN_DELAY);
            rotate(RIGHT, heading);
            stopMotors();
            moveForward(tof);
            break;
        case TURN_BACK:
            stopMotors();
            turn(BACK);
            delay(2 * TURN_DELAY);
            rotate(BACK, heading);
            stopMotors();
            moveForward(tof);
            break;
        case NO_MOVE:
            stopMotors();
            break;
    }
}

void RobotFSM::config() {
  maze.reset();
  currentCell = maze.visitCell(-1, NORTH);
  maze.recordWalls(tof, currentCell, NORTH);

  heading = NORTH;
  state = WAITING;
}

void RobotFSM::update() {
  if(cellComplete()) {
    currentCell = maze.visitCell(currentCell, heading);

    maze.recordWalls(tof, currentCell, heading);

    TurnDecision d = DFS(maze, currentCell, heading);

    executeMove(d, heading);

    beginCellTravel();
  }

  if(bluetoothStop) {
    bluetoothStop = false;

    stopMotors();

    state = WAITING;

    return;
  }

  switch(state) {
    case WAITING: {
      if(digitalRead(START_BUTTON_PIN) == LOW || bluetoothStart) {
        bluetoothStart = false;

        state = EXPLORATION;
      }
      break;
    }

    case EXPLORATION: {
      DFS(maze, currentCell, heading);
    }

    case RETURN: {

    }

    case SPEEDRUN: {

    }
  }
}


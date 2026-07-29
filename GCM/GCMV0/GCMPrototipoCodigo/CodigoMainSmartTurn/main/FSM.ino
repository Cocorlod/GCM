#include "FSM.hpp"

static void executeMove(TurnDecision decision, Heading& heading) {
    switch (decision) {
        case GO_FORWARD:
            moveForward(tof);
            break;

        case TURN_LEFT:
            stopMotors();
            delay(DELAY_STOP_MS);
            turn(LEFT, tof);
            delay(TURN_DELAY);
            heading = rotate(LEFT, heading);
            stopMotors();
            delay(DELAY_STOP_MS);
            moveForward(tof);
            break;

        case TURN_RIGHT:
            stopMotors();
            delay(DELAY_STOP_MS);
            turn(RIGHT, tof);
            delay(TURN_DELAY);
            heading = rotate(RIGHT, heading);
            stopMotors();
            delay(DELAY_STOP_MS);
            moveForward(tof);
            break;

        case TURN_BACK:
            stopMotors();
            delay(DELAY_STOP_MS);
            turn(BACK, tof);
            delay(2 * TURN_DELAY);
            heading = rotate(BACK, heading);
            stopMotors();
            delay(DELAY_STOP_MS);
            moveForward(tof);
            break;

        case NO_MOVE:
            stopMotors();
            break;
    }
}

static void correctHeadingInPlace(Heading& heading, Heading desired) {
    if (heading == desired) return;

    TurnDecision d = decisionForHeading(heading, desired);

    stopMotors();

    switch (d) {
        case TURN_LEFT:
            turn(LEFT, tof);
            delay(TURN_DELAY);
            heading = rotate(LEFT, heading);
            break;
        case TURN_RIGHT:
            turn(RIGHT, tof);
            delay(TURN_DELAY);
            heading = rotate(RIGHT, heading);
            break;
        case TURN_BACK:
            turn(BACK, tof);
            delay(2 * TURN_DELAY);
            heading = rotate(BACK, heading);
            break;
        default:
            break;
    }

    stopMotors();
}

static constexpr uint32_t CLEAR_HOLD_MS = 2000;

void RobotFSM::config() {
    pathKnown = loadPath();

    if (pathKnown) {
        position = AT_START;
        heading = NORTH;
        debugPrint("Boot: speedrun path loaded from flash, ready to run");
    } else {
        reinitializeForExploration();
        debugPrint("Boot: no saved path, ready for exploration");
    }

    state = WAITING;
}

void RobotFSM::reinitializeForExploration() {
    maze.reset();
    currentCell = maze.visitCell(-1, NORTH);
    maze.recordWalls(tof, currentCell, NORTH);

    heading = NORTH;
    position = AT_START;
    pathKnown = false;
    goalSignal = false;

    resetExploration();
}

void RobotFSM::beginExploration() {
    TurnDecision d = DFS(maze, currentCell, heading);
    executeMove(d, heading);
    beginCellTravel();
}

void RobotFSM::beginSpeedrun() {
    resetSpeedrunProgress();
    heading = NORTH;

    TurnDecision d = speedrunNext();
    executeMove(d, heading);
    beginCellTravel();
}

void RobotFSM::beginReturn() {
    buildReturnPath();

    TurnDecision d = returnPathNext();
    executeMove(d, heading);
    beginCellTravel();
}

void RobotFSM::pollClearButton() {
    bool held = (digitalRead(CLEAR_BUTTON_PIN) == LOW);

    if (!held) {
        clearHoldStart = 0;
        return;
    }

    if (clearHoldStart == 0) {
        clearHoldStart = millis();
        return;
    }

    if (millis() - clearHoldStart >= CLEAR_HOLD_MS) {
        clearHoldStart = 0;

        stopMotors();
        clearPath();
        reinitializeForExploration();
        state = WAITING;

        debugPrint("Memory cleared via button hold - ready for new exploration");
    }
}

void RobotFSM::update() {
    pollClearButton();

    if (bluetoothStop) {
        bluetoothStop = false;
        stopMotors();

        if (state == EXPLORATION) {
            reinitializeForExploration();
        } else {
            position = AT_START; 
        }

        state = WAITING;
        return;
    }

    switch (state) {
        case WAITING: {
            bool startSignal = (digitalRead(START_BUTTON_PIN) == LOW) || bluetoothStart;

            if (startSignal) {
                bluetoothStart = false;

                if (!pathKnown) {
                    beginExploration();
                    state = EXPLORATION;
                } else if (position == AT_START) {
                    beginSpeedrun();
                    state = SPEEDRUN;
                } else {
                    beginReturn();
                    state = RETURN;
                }
            }
            break;
        }

        case EXPLORATION: {
            if (goalDetected()) {
                goalSignal = true;
            }

            if (cellComplete()) {
                currentCell = maze.visitCell(currentCell, heading);
                maze.recordWalls(tof, currentCell, heading);

                if (goalSignal) {
                    goalSignal = false;
                    commitGoal(maze, currentCell);
                    pathKnown = true;
                    position = AT_GOAL;
                    debugPrint("Exploration finished: goal reached");
                    state = WAITING;
                    break;
                }

                TurnDecision d = DFS(maze, currentCell, heading);

                if (d == NO_MOVE && explorationComplete()) {
                    stopMotors();
                    debugPrint("Exploration finished: maze fully covered, goal not found");
                    state = WAITING;
                    break;
                }

                executeMove(d, heading);
                beginCellTravel();
            }
            break;
        }

        case SPEEDRUN: {
            if (cellComplete()) {
                if (speedrunFinished()) {
                    stopMotors();
                    position = AT_GOAL;
                    debugPrint("Speedrun finished: goal reached");
                    state = WAITING;
                    break;
                }

                TurnDecision d = speedrunNext();
                executeMove(d, heading);
                beginCellTravel();
            }
            break;
        }

        case RETURN: {
            if (cellComplete()) {
                if (returnPathFinished()) {
                    correctHeadingInPlace(heading, NORTH);
                    stopMotors();
                    position = AT_START;
                    debugPrint("Return finished: back at start");
                    state = WAITING;
                    break;
                }

                TurnDecision d = returnPathNext();
                executeMove(d, heading);
                beginCellTravel();
            }
            break;
        }
    }
}
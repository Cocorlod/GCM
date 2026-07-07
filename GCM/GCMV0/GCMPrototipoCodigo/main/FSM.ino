#include "FSM.hpp"

Heading heading = NORTH;

// Current physical cell occupied by the robot
static int16_t currentCell = 0;

// Used by the debounced cell detector
static CellBoundaryDetector explorationBoundary;
static CellBoundaryDetector returnBoundary;

void RobotFSM::config() {
    maze.reset();
    resetDFS();

    goalFound = false;

    currentCell = maze.visitCell(-1, NORTH);
    maze.recordWalls(tof, currentCell, NORTH);

    explorationBoundary.reset();
    returnBoundary.reset();

    heading = NORTH;

    state = WAITING;
}

void RobotFSM::update() {

    // Global emergency stop
    if(bluetoothStop) {
        bluetoothStop = false;

        stopMotors();

        state = WAITING;

        return;
    }

    switch(state) {

        case WAITING: {

            stopMotors();

            if(digitalRead(CLEAR_BUTTON_PIN) == LOW) {

                if(clearButtonPressedTime == 0)
                    clearButtonPressedTime = millis();

                if(millis() - clearButtonPressedTime >= 2000) {

                    clearPath();

                    maze.reset();

                    resetDFS();

                    goalFound = false;

                    currentCell = maze.visitCell(-1, NORTH);

                    maze.recordWalls(tof, currentCell, NORTH);

                    state = WAITING;

                    clearButtonPressedTime = 0;
                }
            }
            else {
                clearButtonPressedTime = 0;
            }

            if(digitalRead(START_BUTTON_PIN) == LOW || bluetoothStart) {

                bluetoothStart = false;

                heading = NORTH;

                loadPath();

                if(pathSaved) {

                    debugPrint("Starting speedrun");

                    explorationBoundary.reset();

                    state = SPEEDRUN;
                }
                else {

                    debugPrint("Starting exploration");

                    maze.reset();

                    resetDFS();

                    goalFound = false;

                    currentCell = maze.visitCell(-1, NORTH);

                    maze.recordWalls(tof, currentCell, heading);

                    explorationBoundary.reset();

                    state = EXPLORATION;
                }

                delay(200);
            }

            break;
        }

        case EXPLORATION: {

            bool front = tof.isThereWall(FRONT);
            bool left  = tof.isThereWall(LEFT);
            bool right = tof.isThereWall(RIGHT);

            bool centered = tof.isCentered();

            bool rawDecision = centered && (front || (!left || !right));

            if(explorationBoundary.check(rawDecision)) {

                stopMotors();

                currentCell = maze.visitCell(currentCell, heading);

                maze.recordWalls(tof, currentCell, heading);

                TurnDecision move = chooseDFS(maze, currentCell, heading);

                if(move == NO_MOVE) {

                    stopMotors();

                    if(goalFound) {

                        computeFloodFill(maze);

                        generateSpeedrunPath(maze);

                        savePath();

                        debugPrint("Goal found, path saved.");
                    }
                    else {

                        debugPrint("Maze exhausted. Goal not found.");
                    }

                    resetReturnToStart();

                    returnBoundary.reset();

                    state = RETURN;

                    break;
                }

                executeMove(move, heading);
            }
            else {

                moveForward(tof);
            }

            break;
        }

        case RETURN: {

            if(returnToStartStep(heading, returnBoundary)) {

                heading = NORTH;

                state = WAITING;
            }

            break;
        }

        case SPEEDRUN: {

            static uint16_t currentMove = 0;

            if(currentMove >= pathLength) {

                currentMove = 0;

                stopMotors();

                state = FINISHED;

                break;
            }

            bool front = tof.isThereWall(FRONT);
            bool left  = tof.isThereWall(LEFT);
            bool right = tof.isThereWall(RIGHT);

            bool centered = tof.isCentered();

            bool rawDecision = centered && (front || (!left || !right));

            if(explorationBoundary.check(rawDecision)) {

                executeMove(speedrunPath[currentMove], heading);

                currentMove++;
            }
            else {

                moveForward(tof);
            }

            break;
        }

        case FINISHED: {

            stopMotors();

            if(digitalRead(START_BUTTON_PIN) == LOW || bluetoothStart) {

                bluetoothStart = false;

                heading = NORTH;

                resetReturnToStart();

                returnBoundary.reset();

                state = RETURN;

                delay(200);
            }

            break;
        }
    }
}
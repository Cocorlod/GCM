#include "FSM.hpp"

Heading heading = NORTH;

void RobotFSM::config() {
    maze.addCell();

    state = WAITING;
}

void RobotFSM::update() {
    switch (state) {
        case WAITING: {
            stopMotors();

            if(digitalRead(START_BUTTON_PIN) == LOW || bluetoothStart) {
                debugPrint("ROBOT STARTED");
                loadPath();
                delay(200);
                heading = NORTH;
                if(pathSaved) {
                    debugPrint("LOADING SPEEDRUN");
                    state = SPEEDRUN;
                } else {
                    debugPrint("STARTING EXPLORATION");
                    state = EXPLORATION;
                }
            }
            
            if(digitalRead(CLEAR_BUTTON_PIN) == LOW){
                if(clearButtonPressedTime == 0) {
                    clearButtonPressedTime = millis();
                }
                if(millis() - clearButtonPressedTime >= 2000) {
                    clearPath();
                    clearButtonPressedTime = 0;
                    state = WAITING;
                }
                
            } else {
                clearButtonPressedTime = 0;
            }

            break;
        }

        case EXPLORATION: { 
            bool front = tof.isThereWall(FRONT);
            
            bool left = tof.isThereWall(LEFT);

            bool right = tof.isThereWall(RIGHT);

            bool centered = tof.isCentered();

            bool decision = centered && (front || (!left || !right));

            if(decision) {
                stopMotors();

                maze.addCell();
                maze.mazeUpdate(tof, heading);

                TurnDecision move = chooseDFS(maze, maze.cellCount() - 1, heading);

                if(move == NO_MOVE) {
                    stopMotors();
                    computeFloodFill(maze);
                    generateSpeedrunPath(maze);
                    savePath();
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
            returnToStart(heading);
            if(finishedReturnToStart) {
                finishedReturnToStart = false;
                state = WAITING;
            }
            
            break;
        }

        case SPEEDRUN: {
            static int currentMove = 0;

            if(currentMove >= pathLength) {
                currentMove = 0;

                stopMotors();

                state = FINISHED;

                break;
            }

            executeMove(speedrunPath[currentMove], heading);

            currentMove++;
            break;
        }    

        case FINISHED: {
            stopMotors();
            if(digitalRead(START_BUTTON_PIN) == LOW) {
                delay(200);
                state = RETURN;
            }

            break;
        }
    }
}
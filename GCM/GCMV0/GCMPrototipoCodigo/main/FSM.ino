#include "FSM.hpp"

ToFSensor tof;
Heading heading = NORTH;

void RobotFSM::setup() {
    maze.addCell();

    state = EXPLORATION;
}

void RobotFSM::update() {
    switch (state) {
        case WAITING:
            stopMotors();
            break;
        case LOAD_MAZE:
            state = EXPLORATION;  
            break;
        case EXPLORATION:
            tof.update();

            bool front = tof.isThereWall(FRONT);
            
            bool left = tof.isThereWall(LEFT);

            bool right = tof.isThereWall(RIGHT);

            bool centered = tof.isCentered();

            bool decision = centered && (front || (!left && !right));

            if(decision) {
                stopMotors();

                maze.addCell();
                maze.mazeUpdate(tof, heading);

                TurnDecision move = chooseDFS(maze, maze.cellCount() - 1, heading);

                if(move == NO_MOVE) {
                    stopMotors();
                    computeFloodFill(maze);
                    generateSpeedRunPath(maze);

                    break;
                }

                executeMove(move, heading);
            } 
            else {
                moveForward();
            }
            
            break;

        case RETURN:
            returnToStart(heading);
            break;
        case SPEEDRUN:
            generateSpeedrunPath(maze);
            for(int i = 0; i < pathLength; i++) {
                executeMove(speedrunPath[i], heading);
            }
            state = RETURN;
            break;
        case ERROR_STATE:
            stopMotors();
            break;
    }
}
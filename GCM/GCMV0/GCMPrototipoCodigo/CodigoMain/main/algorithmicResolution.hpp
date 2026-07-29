#pragma once

#include "movement.hpp"
#include "maze.hpp"

#define IR_PIN 15
#define IR_THRESHOLD 2500

enum TurnDecision : uint8_t {
    GO_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_BACK,
    NO_MOVE
};

TurnDecision DFS(Maze& maze, uint16_t current, Heading heading);
bool explorationComplete();

void floodFill(Maze& maze, uint16_t goalCell);

bool goalDetected();

void beginCellTravel();
bool cellComplete();

bool buildSpeedrunPath(Maze& maze, uint16_t startCell, uint16_t goalCell);
TurnDecision speedrunNext();
bool speedrunFinished();
bool checkGoalAndBuildPath(Maze& maze, uint16_t currentCell);

void savePath();
bool loadPath();
void clearPath();

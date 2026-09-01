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

TurnDecision decisionForHeading(Heading current, Heading desired);

void floodFill(Maze& maze, uint16_t goalCell);

bool goalDetected();
void commitGoal(Maze& maze, uint16_t currentCell);

bool buildSpeedrunPath(Maze& maze, uint16_t startCell, uint16_t goalCell);
TurnDecision speedrunNext();
bool speedrunFinished();
void resetSpeedrunProgress();

void buildReturnPath();
TurnDecision returnPathNext();
bool returnPathFinished();

Heading rotate(Turn t, Heading h);

void resetExploration();

void beginCellTravel();
bool cellComplete();

void savePath();
bool loadPath();
void clearPath();
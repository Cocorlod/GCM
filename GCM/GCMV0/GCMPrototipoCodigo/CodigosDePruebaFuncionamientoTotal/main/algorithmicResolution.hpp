#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <Preferences.h>
#include "maze.hpp"

#define IR_PIN 15
#define IR_THRESHOLD 2500

static uint16_t flood[MAZE_MAX_CELLS];
static Preferences prefs;

bool isGoalDetected();

extern bool pathSaved;  
extern uint16_t pathLength;

extern bool goalFound;

enum TurnDecision : uint8_t {
    GO_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_BACK,
    NO_MOVE
};

extern TurnDecision speedrunPath[MAZE_MAX_CELLS];

extern Maze maze;

Heading rotateLeft(Heading h);
Heading rotateRight(Heading h);
Heading rotateBack(Heading h);
WallDir headingToWall(Heading h);

struct CellBoundaryDetector {
    uint8_t stableCount = 0;
    unsigned long lastEventTime = 0;

    void reset();
    bool check(bool rawCondition);
};

void resetDFS();

TurnDecision chooseDFS(Maze& maze, uint16_t current, Heading heading);

void computeFloodFill(Maze& maze);
void generateSpeedrunPath(Maze& maze);

void resetReturnToStart();
bool returnToStartStep(Heading& heading, CellBoundaryDetector& boundary);

void savePath();
void loadPath();
void clearPath();
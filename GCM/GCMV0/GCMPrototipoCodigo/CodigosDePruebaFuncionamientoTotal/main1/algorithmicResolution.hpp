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
extern bool finishedReturnToStart;
extern uint16_t pathLength;

enum TurnDecision {
    GO_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_BACK,
    NO_MOVE
};

extern TurnDecision speedrunPath[MAZE_MAX_CELLS];

struct DFSNode {
    uint16_t cell;
    Heading heading;
};

extern Maze maze;

TurnDecision chooseDFS(Maze& maze, uint16_t current, Heading heading);
void generateSpeedrunPath(Maze& maze);

void savePath();
void loadPath();
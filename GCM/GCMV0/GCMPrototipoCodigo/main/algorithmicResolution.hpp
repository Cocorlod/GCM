#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <Preferences.h>
#include "maze.hpp"

#define MAX_STACK 256
#define IR_PIN 15

static uint16_t flood[MAX_STACK];
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

extern TurnDecision speedrunPath[MAX_STACK];

struct DFSNode {
    uint16_t cell;
    Heading heading;
};

Maze maze;

TurnDecision speedRunPath[MAZE_MAX_CELLS];
TurnDecision chooseDFS(Maze& maze, uint16_t current, Heading heading);
void generateSpeedrunPath(Maze& maze);

void savePath();
void loadPath();
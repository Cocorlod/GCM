#pragma once

Maze maze;

#include <Arduino.h>
#include <stdint.h>
#include "maze.hpp"

#define MAX_STACK 256
#define IR_PIN 15

static uint16_t flood[MAZE_MAX_CELLS];

bool isGoalDetected();

enum TurnDecision {
    GO_FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
    TURN_BACK,
    NO_MOVE
};

struct DFSNode {
    uint16_t cell;
    Heading heading;
};

TurnDecision chooseDFS(Maze& maze, uint16_t current, Heading heading);
void generateSpeedrunPath(Maze& maze);
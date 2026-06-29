#pragma once

#define OPEN_INTERSECTION_DELAY 100

#include <Arduino.h>
#include "maze.hpp"
#include "movement.hpp"
#include "algorithmicResolution.hpp"
#include "ToF_Setup.hpp"

enum RobotState {
    WAITING,
    LOAD_MAZE,
    EXPLORATION,
    RETURN,
    SPEEDRUN,
    ERASE_MEMORY,
    FINISHED,
    ERROR_STATE
};

class RobotFSM {
public:
    void setup();

    void update();
private:
    RobotState state = WAITING;
};
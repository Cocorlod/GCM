#pragma once

#include <Arduino.h>
#include "maze.hpp"
#include "movement.hpp"
#include "algorithmicResolution.hpp"
#include "ToF_Setup.hpp"

static unsigned long clearButtonPressedTime = 0;

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
    void config();

    void update();
private:
    RobotState state = WAITING;
};
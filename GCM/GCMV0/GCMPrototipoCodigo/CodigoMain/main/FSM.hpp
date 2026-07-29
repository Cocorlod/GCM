#pragma once

#include <Arduino.h>
#include "maze.hpp"
#include "movement.hpp"
#include "algorithmicResolution.hpp"

static unsigned long clearButtonPressedTime = 0;

enum RobotState {
    WAITING,
    EXPLORATION,
    RETURN,
    SPEEDRUN
};

class RobotFSM {
    public:
        void config();

        void update();
    private:
        RobotState state = WAITING;
};
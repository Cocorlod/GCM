#pragma once

#include <Arduino.h>
#include "maze.hpp"
#include "movement.hpp"
#include "algorithmicResolution.hpp"
#include "ToF_Setup.hpp"

static unsigned long clearButtonPressedTime = 0;

enum RobotState {
    WAITING,
    EXPLORATION,
    RETURN,
    SPEEDRUN,
    FINISHED
};

class RobotFSM {
    public:
        void config();

        void update();
    private:
        RobotState state = WAITING;

        ToFSensor tof;
};
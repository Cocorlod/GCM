#pragma once

#include <Arduino.h>
#include "maze.hpp"
#include "movement.hpp"
#include "algorithmicResolution.hpp"
#include "bluetooth.hpp"

#define START_BUTTON_PIN 47
#define CLEAR_BUTTON_PIN 48

enum RobotState {
    WAITING,
    EXPLORATION,
    RETURN,
    SPEEDRUN
};

enum RobotPosition {
    AT_START,
    AT_GOAL
};

class RobotFSM {
    public:
        void config();
        void update();

    private:
        RobotState state = WAITING;

        Maze maze;
        uint16_t currentCell = 0;
        Heading heading = NORTH;

        bool pathKnown = false;
        RobotPosition position = AT_START;

        bool goalSignal = false;
        uint32_t clearHoldStart = 0;

        void reinitializeForExploration();

        void beginExploration();
        void beginSpeedrun();
        void beginReturn();

        void pollClearButton();
};
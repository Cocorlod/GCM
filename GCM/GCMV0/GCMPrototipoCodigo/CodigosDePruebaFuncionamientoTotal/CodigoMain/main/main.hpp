#pragma once

#include "ToF_Setup.hpp"
#include "movement.hpp"
#include "bluetooth.hpp"
#include "FSM.hpp"

#define SERIAL_SPEED 115200

#define LED_DEBUG_LASER 41
#define LED_TEST 39
#define LED_MOUNTED 38
#define LED_MODE 42

extern RobotFSM FSM;
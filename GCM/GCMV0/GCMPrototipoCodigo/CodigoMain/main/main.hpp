#pragma once

#include "ToF_Setup.hpp"
#include "movement.hpp"
#include "bluetooth.hpp"
#include "FSM.hpp"

#define START_BUTTON_PIN 47
#define CLEAR_BUTTON_PIN 48

#define SERIAL_SPEED 115200

#define LED_DEBUG_LASER 41
#define LED_TEST 39
#define LED_MOUNTED 38
#define LED_MODE 42

extern RobotFSM FSM;
ToFSensor tof;
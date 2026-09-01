#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern volatile bool bluetoothStart;
extern volatile bool bluetoothStop; 

enum RobotCommand {
    CMD_NONE,
    CMD_FORWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_BACK,
    CMD_STOP
};

extern volatile RobotCommand bluetoothCommand;

void beginBluetooth();

void debugPrint(const char *msg);

void debugPrintf(const char *format, ...);
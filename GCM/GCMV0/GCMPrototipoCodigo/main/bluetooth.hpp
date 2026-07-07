#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern volatile bool bluetoothStart;
extern volatile bool bluetoothStop; 

void beginBluetooth();

void debugPrint(const char *msg);

void debugPrintf(const char *format, ...);
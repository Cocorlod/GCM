#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdarg.h>

extern BLECharacteristic* pTxCharacteristic;
extern bool deviceConnected;

void bluetoothInit();
void debugPrint(const char* format, ...);
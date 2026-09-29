#pragma once

#include <Arduino.h>

#define TCA9555_ADDR 0x20

#define REG_INPUT_0 0x00
#define REG_OUTPUT_0 0x02
#define REG_CONFIG_0 0x06
#define REG_INPUT_1 0x01
#define REG_OUTPUT_1 0x03
#define REG_CONFIG_1 0x07

bool tca9555_digitalWrite(uint8_t pin, uint8_t level);
bool tca9555_digitalRead(uint8_t pin);
bool tca9555_pinMode(uint8_t pin, uint8_t mode);
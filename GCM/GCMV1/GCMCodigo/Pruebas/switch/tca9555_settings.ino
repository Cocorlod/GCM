#include "tca9555_settings.h"
#include <Wire.h>

uint8_t tca9555_read(uint8_t reg) {
  uint8_t data = 0;
  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) == 0) { 
    if (Wire.requestFrom(TCA9555_ADDR, 1) == 1) {
      data = Wire.read();
    }
  }
  return data;
}

bool tca9555_digitalWrite(uint8_t pin, uint8_t level) {
  uint8_t reg;
  uint8_t pin_mask;

  if (pin < 8) {
    reg = REG_OUTPUT_0;
    pin_mask = (1 << pin);
  } else {
    reg = REG_OUTPUT_1;
    pin_mask = (1 << (pin - 8));
  }

  uint8_t current_port_state = tca9555_read(reg);

  if(level == HIGH) {
    current_port_state |= pin_mask;
  } else {
    current_port_state &= ~(pin_mask);
  }

  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(reg);
  Wire.write(current_port_state);
  return (Wire.endTransmission() == 0);
}

bool tca9555_digitalRead(uint8_t pin) {
  uint8_t reg;

  if (pin < 8) {
    reg = REG_INPUT_0;
  } else {
    reg = REG_INPUT_1;
    pin -= 8;
  }

  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) { 
    return 0; 
  }

  uint8_t bytesReceived = Wire.requestFrom(TCA9555_ADDR, 1);
  if(bytesReceived == 1) {
    uint8_t data = Wire.read();

    return (data >> pin) & 0x01;
  }
}

bool tca9555_pinMode(uint8_t pin, uint8_t mode) {
  uint8_t reg;
  uint8_t pin_mask;

  if (pin < 8) {
    reg = REG_CONFIG_0;
    pin_mask = (1 << pin);
  } else {
    reg = REG_CONFIG_1;
    pin_mask = (1 << (pin - 8));
  }

  uint8_t current_config = tca9555_read(reg);

  if(mode == INPUT) {
    current_config |= pin_mask;
  } else {
    current_config &= ~(pin_mask);
  }

  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(reg);
  Wire.write(current_config);
  return (Wire.endTransmission() == 0);
}
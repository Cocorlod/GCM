#include "tca9555_global_setup.hpp"

void tcaWriteRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(TCA9555_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t tcaReadRegister(uint8_t reg) {
    Wire.beginTransmission(TCA9555_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(TCA9555_ADDR, 1);

    if (Wire.available())
        return Wire.read();

    return 0;
}
#include "ToF_Setup.hpp"

const uint8_t ToFSensor::XSHUTPIN[SENSOR_COUNT] = {
    PIN_XSHUT1,
    PIN_XSHUT2,
    PIN_XSHUT3,
    PIN_XSHUT4,
    PIN_XSHUT5,
    PIN_XSHUT6
};

bool ToFSensor::beginToF() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_CLOCK);

    for(uint8_t i = 0; i < SENSOR_COUNT; i++) {
        pinMode(XSHUTPIN[i], OUTPUT);
        digitalWrite(XSHUTPIN[i], LOW);
    }

    bool allOk = true;
    for(uint8_t i = 0; i < SENSOR_COUNT; i++) {
        digitalWrite(XSHUTPIN[i], HIGH);
        delay(10);

        sensor[i].setTimeout(500);
        if(!sensor[i].init()) {
            ok[i] = false;
            allOk = false;
            continue;
        }

        sensor[i].setAddress(I2C_DEFAULT_ADDRESS + i);

        sensor[i].setDistanceMode(VL53L1X::Short);
        sensor[i].setMeasurementTimingBudget(20000);
        sensor[i].startContinuous(20); 
        ok[i] = true;
    }
    return allOk;
}


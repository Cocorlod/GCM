#pragma once
#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h> 

#define PIN_SCL 11 
#define PIN_SDA 12
#define I2C_CLOCK 900000UL
#define I2C_DEFAULT_ADDRESS 0x29

/*
XSHUT1-> FR
XSHUT2-> FL
XSHUT3-> RF
XSHUT4-> RB
XSHUT5-> LB
XSHUT6-> RB
*/

#define PIN_XSHUT1 1
#define PIN_XSHUT2 5
#define PIN_XSHUT3 2
#define PIN_XSHUT4 37
#define PIN_XSHUT5 36
#define PIN_XSHUT6 7
#define SENSOR_COUNT 6

class ToFSensor {
    public:
        bool beginToF();

        uint16_t frontLeftReading();
        uint16_t frontRightReading();

        uint16_t rightFrontReading();
        uint16_t rightBackReading();

        uint16_t leftFrontReading();
        uint16_t leftBackReading();

    private:
        VL53L1X sensor[SENSOR_COUNT];
        bool ok[SENSOR_COUNT] = {false}

        uint16_t distance[SENSOR_COUNT] = {0};
}

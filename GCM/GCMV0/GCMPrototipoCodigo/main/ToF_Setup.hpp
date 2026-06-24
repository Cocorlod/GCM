#pragma once

#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h> 

#define PIN_SCL 11 
#define PIN_SDA 12
#define I2C_CLOCK 400000UL
#define I2C_DEFAULT_ADDRESS 0x29

/*
XSHUT1-> FR 0
XSHUT2-> FL 1
XSHUT3-> RF 2
XSHUT4-> RB 3 
XSHUT5-> LB 4
XSHUT6-> LF 5
*/

#define PIN_XSHUT1 1
#define PIN_XSHUT2 5
#define PIN_XSHUT3 2
#define PIN_XSHUT4 37
#define PIN_XSHUT5 36
#define PIN_XSHUT6 7
#define SENSOR_COUNT 6

static constexpr float MAX_ALLOWED_DIFF = 95.0f;
static constexpr float FRONT_WALL_THRESHOLD = 94.0f;
static constexpr float SIDE_WALL_THRESHOLD  = 89.0f;

enum SensorID : uint8_t {

    FRONT_R = 0,
    FRONT_L,

    RIGHT_F,
    RIGHT_B,

    LEFT_B,
    LEFT_F
};

enum WallSides : int8_t {
    FRONT = 0,
    RIGHT = 1,
    LEFT = -1
};

class ToFSensor {
    public:
        bool beginToF();

        void update();
    
        bool allSensorsOk() const;
        bool isThereWall(WallSides side) const;

        int16_t wallDistance(WallSides side) const; 
        
        int16_t alignmentError(WallSides side) const;

        uint16_t getDistance(SensorID id) const;
    private:
        VL53L1X sensor[SENSOR_COUNT];
        bool ok[SENSOR_COUNT] = {false};

        uint16_t distance[SENSOR_COUNT] = {0};
        static const uint8_t XSHUTPIN[SENSOR_COUNT];
};
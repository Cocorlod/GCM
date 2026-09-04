#pragma once

#include <Wire.h>
#include <Arduino.h>
#include <VL53L1X.h>

#define SENSOR_COUNT 6
#define FRONT_R 0
#define FRONT_L 1
#define RIGHT_F 2
#define RIGHT_B 3
#define LEFT_B 4
#define LEFT_F 5

#define XSHUT_FRONT_R 1
#define XSHUT_FRONT_L 5
#define XSHUT_RIGHT_F 2
#define XSHUT_RIGHT_B 37
#define XSHUT_LEFT_B 36
#define XSHUT_LEFT_F 7

#define TOF_CONTROL_PERIOD_MS 20
#define TOF_SIDE_KP 0.8f
#define TOF_SIDE_KD 0.08f
#define TOF_FRONT_BACK_KP 0.6f
#define TOF_FRONT_BACK_KD 0.06f
#define TOF_DIAGONAL_KP 0.4f
#define TOF_DIAGONAL_KD 0.04f
#define TOF_DERIVATIVE_ALPHA 0.75f
#define MAX_TOF_CORRECTION 20.0f
#define SENSOR_INVALID_DISTANCE 60
#define FRONT_WALL_THRESHOLD 120
#define SIDE_WALL_THRESHOLD 120

#define I2C_SDA 12
#define I2C_SCL 11

enum ErrorSection { LEFT_ERROR, RIGHT_ERROR, BACK_ERROR, FRONT_ERROR, LB_RF_DIAGONAL_ERROR, LF_RB_DIAGONAL_ERROR };

enum WallSides : int8_t {
    WALL_FRONT = 0,
    WALL_RIGHT = 1,
    WALL_LEFT = -1
};

VL53L1X sensors[SENSOR_COUNT];
uint8_t sensorPins[SENSOR_COUNT] = { XSHUT_FRONT_R, XSHUT_FRONT_L, XSHUT_RIGHT_F, XSHUT_RIGHT_B, XSHUT_LEFT_B, XSHUT_LEFT_F };
uint8_t sensorAddresses[SENSOR_COUNT] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35 };
uint16_t distance[SENSOR_COUNT];
bool ok[SENSOR_COUNT];

struct PDController {
    float kp;
    float kd;
    float previousError = 0.0f;
    float filteredDerivative = 0.0f;
};

PDController leftTofPD = { TOF_SIDE_KP, TOF_SIDE_KD };
PDController rightTofPD = { TOF_SIDE_KP, TOF_SIDE_KD };

PDController frontTofPD = { TOF_FRONT_BACK_KP, TOF_FRONT_BACK_KD };
PDController backTofPD = { TOF_FRONT_BACK_KP, TOF_FRONT_BACK_KD };

PDController diagonalLB_RF_PD = { TOF_DIAGONAL_KP, TOF_DIAGONAL_KD };
PDController diagonalLF_RB_PD = { TOF_DIAGONAL_KP, TOF_DIAGONAL_KD };

float tofCorrection = 0.0f;
uint32_t lastTofControlTime = 0;

void setupToF();
bool frontWallDetected();
void updateTofControl();
bool isThereWall(WallSides side);
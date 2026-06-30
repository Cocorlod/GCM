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

void ToFSensor::update() {
    for(uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if(!ok[i]) continue;
        if(sensor[i].dataReady()) {
            if(!sensor[i].timeoutOccurred()) {
                distance[i] = sensor[i].read(false);
            }
        }
    }
}

bool ToFSensor::allSensorsOk() const {
    for(uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if(!ok[i]) return false;
    }
    return true;
}

bool ToFSensor::isThereWall(WallSides side) const {
    switch(side) {
        case FRONT:
            return wallDistance(side) < FRONT_WALL_THRESHOLD;
        case LEFT:
            return wallDistance(side) < SIDE_WALL_THRESHOLD;
        case RIGHT:
            return wallDistance(side) < SIDE_WALL_THRESHOLD;
        default:
            return false;
    }
}

bool ToFSensor::isCentered() const {
    bool frontCentered = abs(wallDistance(FRONT) - FRONT_WALL_THRESHOLD_CENTER) <= OFFSET_CENTER;
    bool leftCentered = abs(wallDistance(LEFT) - SIDE_WALL_THRESHOLD_CENTER) <= OFFSET_CENTER;
    bool rightCentered = abs(wallDistance(RIGHT) - SIDE_WALL_THRESHOLD_CENTER) <= OFFSET_CENTER;

    if (isThereWall(FRONT)) {
        return frontCentered && ((leftCentered && isThereWall(LEFT)) || (rightCentered && isThereWall(RIGHT)));
    }

    if(isThereWall(LEFT) && isThereWall(RIGHT)) {
        return leftCentered && rightCentered;
    }

    if(isThereWall(LEFT)) {
        return leftCentered;
    }
    
    if(isThereWall(RIGHT)) {
        return rightCentered;
    }
    
    return true;
}

float ToFSensor::wallDistance(WallSides side) const {
    switch(side) {
        case FRONT:
            if(abs((int)distance[FRONT_L] - (int)distance[FRONT_R]) < MAX_ALLOWED_DIFF) {
                return (distance[FRONT_L] + distance[FRONT_R])/2.0f;
            } 
            return min(distance[FRONT_L], distance[FRONT_R]);
        case LEFT:
            if(abs((int)distance[LEFT_F] - (int)distance[LEFT_B]) < MAX_ALLOWED_DIFF) {
                return (distance[LEFT_F] + distance[LEFT_B])/2.0f;
            }
            return min(distance[LEFT_F], distance[LEFT_B]);
        case RIGHT:
            if(abs((int)distance[RIGHT_F] - (int)distance[RIGHT_B]) < MAX_ALLOWED_DIFF) {
                return (distance[RIGHT_F] + distance[RIGHT_B])/2.0f;
            }
            return min(distance[RIGHT_F], distance[RIGHT_B]);
        default:
            return 0.0f;
    }
}

int16_t ToFSensor::alignmentError(WallSides side) const {
    if(!isThereWall(side)) return 0;

    switch(side) {
        case LEFT:
            return (int16_t)distance[LEFT_F] - (int16_t)distance[LEFT_B];
        case RIGHT:
            return (int16_t)distance[RIGHT_F] - (int16_t)distance[RIGHT_B];
        default:
            return 0;
    }  
}

uint16_t ToFSensor::getDistance(SensorID id) const {
    if(!ok[id]) return 0;
    return distance[id];
}
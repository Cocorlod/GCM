/*
Setupeo de los sensores ToF VL53L1X en GCM V0:

-Se inicializan los sensores
-Se leen los valores y se almacenan en una variable
-Verificacion de que los sensores esten en funcionamiento
-Tres funciones que determinan la existencia de paredes en el frente, a la izquierda o derecha
-Tres funciones que determinan la distancia a cada una de las paredes (promedio entre dos sensores en cada direccion)
-Error en izquierda y derecha
-Distancia especifica de un sensor
*/

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
            if(!sensor[i].timeOutOccurred()) {
                distance[i] = sensor[i].read(false);
                sensor[i].clearInterrupt();
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

bool ToFSensor::isThereFrontWall() const {
    return frontWallDistance() < FRONT_WALL_THRESHOLD;
}

bool ToFSensor::isThereLeftWall() const {
    return leftWallDistance() < SIDE_WALL_THRESHOLD;
}

bool ToFSensor::isThereRightWall() const {
    return rightWallDistance() < SIDE_WALL_THRESHOLD;
}

float ToFSensor::frontWallDistance() const   {
    if(abs((int)distance[FRONT_L] - (int)distance[FRONT_R]) < MAX_ALLOWED_DIFF) {
        return (distance[FRONT_L] + distance[FRONT_R])/2.0f;
    } 
    return min(distance[FRONT_L], distance[FRONT_R]);
}

float ToFSensor::rightWallDistance() const {
    if(abs((int)distance[RIGHT_F] - (int)distance[RIGHT_B]) < MAX_ALLOWED_DIFF) {
        return (distance[RIGHT_F] + distance[RIGHT_B])/2.0f;
    }
    return min(distance[RIGHT_F], distance[RIGHT_B]);
}

float ToFSensor::leftWallDistance() const {
    if(abs((int)distance[LEFT_F] - (int)distance[LEFT_B]) < MAX_ALLOWED_DIFF) {
        return (distance[LEFT_F] + distance[LEFT_B])/2.0f;
    }
    return min(distance[LEFT_F], distance[LEFT_B]);
}

float ToFSensor::leftAlignmentError() const {
    if(!isThereLeftWall()) return 0.0f;
    return (float)distance[LEFT_F] - (float)distance[LEFT_B];
}

float ToFSensor::rightAlignmentError() const {
    if(!isThereRightWall()) return 0.0f;
    return (float)distance[RIGHT_F] - (float)distance[RIGHT_B];
}

uint16_t ToFSensor::getDistance(SensorID id) const {
    if(!ok[id]) return 0;
    return distance[id];
}
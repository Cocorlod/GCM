#include "ToF_Setup.hpp"

const uint8_t ToFSensor::XSHUTPIN[SENSOR_COUNT] = {
    PIN_XSHUT1,
    PIN_XSHUT2,
    PIN_XSHUT3,
    PIN_XSHUT4,
    PIN_XSHUT5,
    PIN_XSHUT6
};

bool ToFSensor::initSensor(uint8_t i) {
    digitalWrite(XSHUTPIN[i], LOW);
    delay(5);
    digitalWrite(XSHUTPIN[i], HIGH);
    delay(10);

    sensor[i].setTimeout(500);

    if (!sensor[i].init()) {
        digitalWrite(XSHUTPIN[i], LOW);
        return false;
    }

    sensor[i].setAddress(I2C_DEFAULT_ADDRESS + i);
    sensor[i].setDistanceMode(VL53L1X::Short);
    sensor[i].setMeasurementTimingBudget(20000);
    sensor[i].startContinuous(20);

    return true;
}

bool ToFSensor::beginToF() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(I2C_CLOCK);

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        pinMode(XSHUTPIN[i], OUTPUT);
        digitalWrite(XSHUTPIN[i], LOW);
    }

    bool allOk = true;

    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        ok[i] = initSensor(i);

        if (!ok[i]) {
            allOk = false;
        }
    }

    if (allOk) {
        debugPrint("All ToF sensors initialized successfully.");
    } else {
        debugPrint("ToF Failed");
    }

    return allOk;
}

void ToFSensor::tryRecoverSensor(uint8_t i) {
    uint32_t now = millis();

    if (now - lastRecoveryAttempt[i] < SENSOR_RECOVERY_INTERVAL_MS)
        return;

    lastRecoveryAttempt[i] = now;

    // Only the flaky pair members really need this, but it's harmless
    // to allow any sensor to reconnect if its cable comes back.
    if (initSensor(i)) {
        ok[i] = true;
        debugPrintf("Sensor %d recovered", i);
    }
}

void ToFSensor::update() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!ok[i]) {
            tryRecoverSensor(i);
            continue;
        }

        if (sensor[i].dataReady()) {
            if (!sensor[i].timeoutOccurred()) {
                uint16_t raw = sensor[i].read(false);

                // Apply tilt correction only to the front sensors
                if (i == FRONT_R || i == FRONT_L) {
                    distance[i] = (uint16_t)(raw * TOF_TILT_CORRECTION);
                } else {
                    distance[i] = raw;
                }
            } else {
                ok[i] = false;
                lastRecoveryAttempt[i] = millis();
            }
        }
    }
}

bool ToFSensor::allSensorsOk() const {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!ok[i])
            return false;
    }

    return true;
}

bool ToFSensor::sensorOk(SensorID id) const {
    return ok[id];
}

bool ToFSensor::isThereWall(WallSides side) const {
    switch (side) {
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

// primary is preferred as the "trusted" lone reading when only one is available.
// For FRONT: primary=FRONT_R, secondary=FRONT_L (FRONT_L is the flaky one, "sensor 2")
// For LEFT:  primary=LEFT_B,  secondary=LEFT_F  (LEFT_F is the flaky one, "sensor 6")
float ToFSensor::pairDistance(SensorID primary, SensorID secondary) const {
    bool pOk = ok[primary];
    bool sOk = ok[secondary];

    if (!pOk && !sOk)
        return SENSOR_INVALID_DISTANCE;

    // Only the trusted sensor is online -> use it purely.
    if (!sOk)
        return distance[primary];

    // Only the flaky sensor is online -> use it purely (better than nothing).
    if (!pOk)
        return distance[secondary];

    // Both online -> average, same disagreement-guard as before.
    if (abs((int)distance[primary] - (int)distance[secondary]) < MAX_ALLOWED_DIFF) {
        return (distance[primary] + distance[secondary]) / 2.0f;
    }

    return min(distance[primary], distance[secondary]);
}

float ToFSensor::wallDistance(WallSides side) const {
    switch (side) {
        case FRONT:
            // FRONT_R = sensor 1 (trusted), FRONT_L = sensor 2 (flaky cable)
            return pairDistance(FRONT_R, FRONT_L);

        case LEFT:
            // LEFT_B = sensor 5 (trusted), LEFT_F = sensor 6 (flaky cable)
            return pairDistance(LEFT_B, LEFT_F);

        case RIGHT: {
            bool fOk = ok[RIGHT_F];
            bool bOk = ok[RIGHT_B];

            if (!fOk && !bOk)
                return SENSOR_INVALID_DISTANCE;
            if (!fOk)
                return distance[RIGHT_B];
            if (!bOk)
                return distance[RIGHT_F];

            if (abs((int)distance[RIGHT_F] - (int)distance[RIGHT_B]) < MAX_ALLOWED_DIFF) {
                return (distance[RIGHT_F] + distance[RIGHT_B]) / 2.0f;
            }

            return min(distance[RIGHT_F], distance[RIGHT_B]);
        }

        default:
            return 0.0f;
    }
}

int16_t ToFSensor::alignmentError(WallSides side) const {
    if (!isThereWall(side))
        return 0;

    switch (side) {
        case LEFT:
            if (!ok[LEFT_F] || !ok[LEFT_B])
                return 0;

            return (int16_t)distance[LEFT_F] - (int16_t)distance[LEFT_B];

        case RIGHT:
            if (!ok[RIGHT_F] || !ok[RIGHT_B])
                return 0;

            return (int16_t)distance[RIGHT_F] - (int16_t)distance[RIGHT_B];

        default:
            return 0;
    }
}

uint16_t ToFSensor::getDistance(SensorID id) const {
    if (!ok[id])
        return 0;

    return distance[id];
}
#include "tof.h"

float updatePD(PDController& pd, float error, float dt) {
    float derivative = (error - pd.previousError) / dt;
    pd.filteredDerivative = TOF_DERIVATIVE_ALPHA * pd.filteredDerivative + (1.0f - TOF_DERIVATIVE_ALPHA) * derivative;
    float output = pd.kp * error + pd.kd * pd.filteredDerivative;
    pd.previousError = error;
    return output;
}

void setupToF() {
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        pinMode(sensorPins[i], OUTPUT);
        digitalWrite(sensorPins[i], LOW);
    }

    delay(10);

    for (int i = 0; i < SENSOR_COUNT; i++) {
        digitalWrite(sensorPins[i], HIGH);
        delay(10);

        sensors[i].setTimeout(500);

        if (!sensors[i].init()) {
            ok[i] = false;
            continue;
        }

        sensors[i].setAddress(sensorAddresses[i]);
        sensors[i].setDistanceMode(VL53L1X::Short);
        sensors[i].setMeasurementTimingBudget(20000);
        sensors[i].startContinuous(20);

        ok[i] = true;
    }
}

bool validSideWall(uint8_t frontSensor, uint8_t backSensor) {
    return ok[frontSensor] && ok[backSensor] && distance[frontSensor] > SENSOR_INVALID_DISTANCE && distance[backSensor] > SENSOR_INVALID_DISTANCE && distance[frontSensor] < SIDE_WALL_THRESHOLD && distance[backSensor] < SIDE_WALL_THRESHOLD;
}

bool validCrossPair(uint8_t sensorA, uint8_t sensorB) {
    return ok[sensorA] && ok[sensorB] && distance[sensorA] > SENSOR_INVALID_DISTANCE && distance[sensorB] > SENSOR_INVALID_DISTANCE;
}

int16_t alignmentError(ErrorSection errorSection) {
    switch (errorSection) {
        case LEFT_ERROR:
            if (!ok[LEFT_F] || !ok[LEFT_B]) return 0;
            return (int16_t)distance[LEFT_F] - (int16_t)distance[LEFT_B];

        case RIGHT_ERROR:
            if (!ok[RIGHT_F] || !ok[RIGHT_B]) return 0;
            return (int16_t)distance[RIGHT_F] - (int16_t)distance[RIGHT_B];

        case BACK_ERROR:
            if (!ok[LEFT_B] || !ok[RIGHT_B]) return 0;
            return (int16_t)distance[LEFT_B] - (int16_t)distance[RIGHT_B];

        case FRONT_ERROR:
            if (!ok[LEFT_F] || !ok[RIGHT_F]) return 0;
            return (int16_t)distance[LEFT_F] - (int16_t)distance[RIGHT_F];

        case LB_RF_DIAGONAL_ERROR:
            if (!ok[LEFT_B] || !ok[RIGHT_F]) return 0;
            return (int16_t)distance[LEFT_B] - (int16_t)distance[RIGHT_F];

        case LF_RB_DIAGONAL_ERROR:
            if (!ok[LEFT_F] || !ok[RIGHT_B]) return 0;
            return (int16_t)distance[LEFT_F] - (int16_t)distance[RIGHT_B];

        default:
            return 0;
    }
}

bool frontWallDetected() {
    if (!ok[FRONT_L] || !ok[FRONT_R]) return false;

    if (distance[FRONT_L] <= SENSOR_INVALID_DISTANCE || distance[FRONT_R] <= SENSOR_INVALID_DISTANCE) return false;

    if (abs((int)distance[FRONT_L] - (int)distance[FRONT_R]) >= 30) return false;

    float averageDistance = ((float)distance[FRONT_L] + (float)distance[FRONT_R]) * 0.5f;
    return averageDistance <= FRONT_WALL_THRESHOLD;
}

float calculateTofCorrection(float dt) {
    bool leftValid = validSideWall(LEFT_F, LEFT_B);
    bool rightValid = validSideWall(RIGHT_F, RIGHT_B);
    bool backValid = validCrossPair(LEFT_B, RIGHT_B);
    bool frontValid = validCrossPair(LEFT_F, RIGHT_F);
    bool diagonal1Valid = validCrossPair(LEFT_B, RIGHT_F);
    bool diagonal2Valid = validCrossPair(LEFT_F, RIGHT_B);

    float correctionSum = 0.0f;
    int correctionCount = 0;

    if (leftValid) {
        float error = alignmentError(LEFT_ERROR);
        correctionSum += updatePD(leftTofPD, error, dt);
        correctionCount++;
    }

    if (rightValid) {
        float error = alignmentError(RIGHT_ERROR);
        correctionSum += updatePD(rightTofPD, error, dt);
        correctionCount++;
    }

    if (backValid) {
        float error = alignmentError(BACK_ERROR);
        correctionSum += updatePD(backTofPD, error, dt);
        correctionCount++;
    }

    if (frontValid) {
        float error = alignmentError(FRONT_ERROR);
        correctionSum += updatePD(frontTofPD, error, dt);
        correctionCount++;
    }

    if (diagonal1Valid) {
        float error = alignmentError(LB_RF_DIAGONAL_ERROR);
        correctionSum += updatePD(diagonalLB_RF_PD, error, dt);
        correctionCount++;
    }

    if (diagonal2Valid) {
        float error = alignmentError(LF_RB_DIAGONAL_ERROR);
        correctionSum += updatePD(diagonalLF_RB_PD, error, dt);
        correctionCount++;
    }

    if (correctionCount == 0) {
        return 0.0f;
    }

    float correction = correctionSum / (float)correctionCount;

    return constrain(correctionSum / correctionCount, -MAX_TOF_CORRECTION, MAX_TOF_CORRECTION);
}

void readToFSensors() {
    for (int i = 0; i < SENSOR_COUNT; i++) {
        uint16_t reading = sensors[i].read();

        if (sensors[i].timeoutOccurred()) {
            ok[i] = false;
            distance[i] = 0;
        } else if (reading <= SENSOR_INVALID_DISTANCE) {
            ok[i] = false;
            distance[i] = reading;
        } else {
            ok[i] = true;
            distance[i] = reading;
        }
    }
}

float pairDistance(uint16_t primary, uint16_t secondary) {
    bool pOk = ok[primary];
    bool sOk = ok[secondary];

    if (!pOk && !sOk)
        return SENSOR_INVALID_DISTANCE;

    if (!sOk)
        return distance[primary];

    if (!pOk)
        return distance[secondary];

    if (abs((int)distance[primary] - (int)distance[secondary]) < 150) {
        return (distance[primary] + distance[secondary]) / 2.0f;
    }

    return min(distance[primary], distance[secondary]);
}

float wallDistance(WallSides side) {
    switch (side) {
        case WALL_FRONT:
            return pairDistance(FRONT_R, FRONT_L);

        case WALL_LEFT:
            return pairDistance(LEFT_B, LEFT_F);

        case WALL_RIGHT: {
            return pairDistance(RIGHT_F, RIGHT_B);
        }

        default:
            return 0.0f;
    }
}

bool isThereWall(WallSides side) {
    switch (side) {
        case WALL_FRONT:
            return wallDistance(side) < FRONT_WALL_THRESHOLD;

        case WALL_LEFT:
            return wallDistance(side) < SIDE_WALL_THRESHOLD;

        case WALL_RIGHT:
            return wallDistance(side) < SIDE_WALL_THRESHOLD;

        default:
            return false;
    }
}

void resetTofController() {
    leftTofPD.previousError = 0.0f;
    leftTofPD.filteredDerivative = 0.0f;
    rightTofPD.previousError = 0.0f;
    rightTofPD.filteredDerivative = 0.0f;
    backTofPD.previousError = 0.0f;
    backTofPD.filteredDerivative = 0.0f;
    frontTofPD.previousError = 0.0f;
    frontTofPD.filteredDerivative = 0.0f;
    diagonalLB_RF_PD.previousError = 0.0f;
    diagonalLB_RF_PD.filteredDerivative = 0.0f;
    diagonalLF_RB_PD.previousError = 0.0f;
    diagonalLF_RB_PD.filteredDerivative = 0.0f;
    tofCorrection = 0.0f;
}

void updateTofControl() {
    uint32_t now = millis();

    if (now - lastTofControlTime < TOF_CONTROL_PERIOD_MS) return;

    float dt = (now - lastTofControlTime) * 0.001f;
    lastTofControlTime = now;

    readToFSensors();

    if (frontWallDetected()) {
        stopMotors();

        resetTofController();
        return;
    }

    tofCorrection = calculateTofCorrection(dt);
}
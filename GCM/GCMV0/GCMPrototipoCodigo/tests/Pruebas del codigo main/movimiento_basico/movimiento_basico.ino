#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdarg.h>
#include <string.h>

#define PIN_BUTTON 47
#define PIN_STBY 16
#define PIN_BIN1 18
#define PIN_BIN2 17
#define PIN_AIN1 4
#define PIN_AIN2 8
#define PIN_PWMA 9
#define PIN_PWMB 10
#define PIN_ENCODER_LEFT_A 14
#define PIN_ENCODER_LEFT_B 13
#define PIN_ENCODER_RIGHT_A 6
#define PIN_ENCODER_RIGHT_B 21
#define I2C_SDA 12
#define I2C_SCL 11

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

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define TARGET_SPEED_MM_S 80.0f
#define SPEED_LOOP_PERIOD_MS 50
#define TOF_CONTROL_PERIOD_MS 20
#define SERIAL_PERIOD_MS 1000
#define BASE_PWM_LEFT 61
#define BASE_PWM_RIGHT 57
#define KP_LEFT 0.15f
#define KI_LEFT 0.40f
#define KP_RIGHT 0.15f
#define KI_RIGHT 0.40f
#define MAX_INTEGRAL 300.0f
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
#define TURN_PWM 90
#define TURN_STOP_DELAY_MS 100

#define BLE_DEVICE_NAME "Micromouse"
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

static constexpr float WHEEL_DIAMETER_MM = 19.0f;
static constexpr float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;
static constexpr float WHEEL_TRACK_MM = 109.5f;
static constexpr float ENCODER_CPR_MOTOR_SHAFT = 12.0f;
static constexpr float GEAR_RATIO = 4.995f;
static constexpr float LEFT_COUNTS_PER_WHEEL_REV = ENCODER_CPR_MOTOR_SHAFT * GEAR_RATIO * 4.0f;
static constexpr float RIGHT_COUNTS_PER_WHEEL_REV = ENCODER_CPR_MOTOR_SHAFT * GEAR_RATIO * 2.0f;
static constexpr float LEFT_MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / LEFT_COUNTS_PER_WHEEL_REV;
static constexpr float RIGHT_MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / RIGHT_COUNTS_PER_WHEEL_REV;
static constexpr float TURN_DISTANCE_MM = PI * WHEEL_TRACK_MM;
static constexpr long LEFT_TURN_COUNTS = (long)(TURN_DISTANCE_MM / LEFT_MM_PER_COUNT);
static constexpr long RIGHT_TURN_COUNTS = (long)(TURN_DISTANCE_MM / RIGHT_MM_PER_COUNT);

enum RobotState { WAITING, DRIVING, TURNING, FINISHED };
enum ErrorSection { LEFT_ERROR, RIGHT_ERROR, BACK_ERROR, FRONT_ERROR, LB_RF_DIAGONAL_ERROR, LF_RB_DIAGONAL_ERROR };

RobotState robotState = WAITING;

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        BLEDevice::startAdvertising();
    }
};

class Encoder {
public:
    void begin(uint8_t pinA, uint8_t pinB);
    void beginAOnly(uint8_t pinA);
    long getCount() const;

private:
    uint8_t _pinA = 0;
    uint8_t _pinB = 0;
    bool _aOnly = false;
    volatile long _count = 0;
    volatile uint8_t _lastState = 0;
    static void IRAM_ATTR isrHandler(void* arg);
    void IRAM_ATTR handleInterrupt();
    void IRAM_ATTR handleAOnlyInterrupt();
};

Encoder leftEncoder;
Encoder rightEncoder;
VL53L1X sensors[SENSOR_COUNT];

uint8_t sensorPins[SENSOR_COUNT] = { XSHUT_FRONT_R, XSHUT_FRONT_L, XSHUT_RIGHT_F, XSHUT_RIGHT_B, XSHUT_LEFT_B, XSHUT_LEFT_F };
uint8_t sensorAddresses[SENSOR_COUNT] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35 };
uint16_t distance[SENSOR_COUNT];
bool ok[SENSOR_COUNT];

struct PIController {
    float kp;
    float ki;
    float integral = 0.0f;
};

PIController leftPI = { KP_LEFT, KI_LEFT };
PIController rightPI = { KP_RIGHT, KI_RIGHT };

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

long lastLeftControlCount = 0;
long lastRightControlCount = 0;
long lastLeftSerialCount = 0;
long lastRightSerialCount = 0;
long turnStartLeftCount = 0;
long turnStartRightCount = 0;

uint32_t lastSpeedControlTime = 0;
uint32_t lastTofControlTime = 0;
uint32_t lastSerialTime = 0;

float leftSpeed = 0.0f;
float rightSpeed = 0.0f;
float tofCorrection = 0.0f;

int leftPWM = BASE_PWM_LEFT;
int rightPWM = BASE_PWM_RIGHT;

void bluetoothPrint(const char* text) {
    if (!deviceConnected) return;

    const size_t length = strlen(text);
    const size_t chunkSize = 20;

    for (size_t i = 0; i < length; i += chunkSize) {
        size_t remaining = length - i;
        size_t currentSize = remaining < chunkSize ? remaining : chunkSize;
        pCharacteristic->setValue((uint8_t*)&text[i], currentSize);
        pCharacteristic->notify();
        delay(2);
    }
}

void bluetoothPrintln(const char* text) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s\n", text);
    bluetoothPrint(buffer);
}

void bluetoothPrintf(const char* format, ...) {
    if (!deviceConnected) return;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    bluetoothPrint(buffer);
}

float updatePI(PIController& pi, float targetSpeed, float measuredSpeed, float dt) {
    float error = targetSpeed - measuredSpeed;
    pi.integral += error * dt;
    pi.integral = constrain(pi.integral, -MAX_INTEGRAL, MAX_INTEGRAL);
    return pi.kp * error + pi.ki * pi.integral;
}

float updatePD(PDController& pd, float error, float dt) {
    float derivative = (error - pd.previousError) / dt;
    pd.filteredDerivative = TOF_DERIVATIVE_ALPHA * pd.filteredDerivative + (1.0f - TOF_DERIVATIVE_ALPHA) * derivative;
    float output = pd.kp * error + pd.kd * pd.filteredDerivative;
    pd.previousError = error;
    return output;
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
            if (!ok[FRONT_L] || !ok[FRONT_R]) return 0;
            return (int16_t)distance[FRONT_L] - (int16_t)distance[FRONT_R];

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

void setMotors(int left, int right) {
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    digitalWrite(PIN_STBY, HIGH);

    if (left >= 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
    } else {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        left = -left;
    }

    if (right >= 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
    } else {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        right = -right;
    }

    ledcWrite(PIN_PWMA, left);
    ledcWrite(PIN_PWMB, right);
}

void stopMotors() {
    digitalWrite(PIN_STBY, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
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

void resetController() {
    leftPI.integral = 0.0f;
    rightPI.integral = 0.0f;
    leftPWM = BASE_PWM_LEFT;
    rightPWM = BASE_PWM_RIGHT;
    resetTofController();
    lastLeftControlCount = leftEncoder.getCount();
    lastRightControlCount = rightEncoder.getCount();
    lastLeftSerialCount = leftEncoder.getCount();
    lastRightSerialCount = rightEncoder.getCount();
    lastSpeedControlTime = millis();
    lastTofControlTime = millis();
    lastSerialTime = millis();
}

void updateSpeedControl() {
    uint32_t now = millis();

    if (now - lastSpeedControlTime < SPEED_LOOP_PERIOD_MS) return;

    float dt = (now - lastSpeedControlTime) * 0.001f;
    lastSpeedControlTime = now;

    long leftCount = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();

    long deltaLeft = leftCount - lastLeftControlCount;
    long deltaRight = rightCount - lastRightControlCount;

    lastLeftControlCount = leftCount;
    lastRightControlCount = rightCount;

    float measuredLeftSpeed = fabs(deltaLeft * LEFT_MM_PER_COUNT / dt);
    float measuredRightSpeed = fabs(deltaRight * RIGHT_MM_PER_COUNT / dt);

    float leftCorrection = updatePI(leftPI, TARGET_SPEED_MM_S, measuredLeftSpeed, dt);
    float rightCorrection = updatePI(rightPI, TARGET_SPEED_MM_S, measuredRightSpeed, dt);

    leftPWM = constrain((int)(BASE_PWM_LEFT + leftCorrection - tofCorrection), 0, 255);
    rightPWM = constrain((int)(BASE_PWM_RIGHT + rightCorrection + tofCorrection), 0, 255);

    setMotors(leftPWM, rightPWM);
}

void updateTofControl() {
    uint32_t now = millis();

    if (now - lastTofControlTime < TOF_CONTROL_PERIOD_MS) return;

    float dt = (now - lastTofControlTime) * 0.001f;
    lastTofControlTime = now;

    readToFSensors();

    if (frontWallDetected()) {
        stopMotors();

        robotState = TURNING;

        turnStartLeftCount = leftEncoder.getCount();
        turnStartRightCount = rightEncoder.getCount();

        resetTofController();
        return;
    }

    tofCorrection = calculateTofCorrection(dt);
}

void updateTurn() {
    digitalWrite(PIN_STBY, HIGH);

    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);

    while(true) {
        long currentCount;

        noInterrupts();

        currentCount = rightEncoder.getCount();

        interrupts();

        long counts = labs(currentCount - turnStartRightCount); 

        if(counts >= RIGHT_TURN_COUNTS) {
            break;
        }

        long remaining = RIGHT_TURN_COUNTS - counts;

        int pwm = TURN_PWM;

        if (remaining < RIGHT_TURN_COUNTS / 4) {
            pwm = 70;
        }

        ledcWrite(PIN_PWMA, 0);
        ledcWrite(PIN_PWMB, pwm);

    }
    stopMotors();

    while(true) delay(1000);
}

void printDebug() {
    uint32_t now = millis();

    if (now - lastSerialTime < SERIAL_PERIOD_MS) return;

    float dt = (now - lastSerialTime) * 0.001f;

    long leftCount = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();

    long deltaLeft = leftCount - lastLeftSerialCount;
    long deltaRight = rightCount - lastRightSerialCount;

    leftSpeed = fabs(deltaLeft * LEFT_MM_PER_COUNT / dt);
    rightSpeed = fabs(deltaRight * RIGHT_MM_PER_COUNT / dt);

    lastLeftSerialCount = leftCount;
    lastRightSerialCount = rightCount;
    lastSerialTime = now;

    const char* state;

    if (robotState == WAITING) state = "WAITING";
    else if (robotState == DRIVING) state = "DRIVING";
    else if (robotState == TURNING) state = "TURNING";
    else state = "FINISHED";

    bluetoothPrintf("STATE: %s | L: %.1f | R: %.1f | DIFF: %.1f | TOF: %.2f | PWM L: %d | PWM R: %d | F: %u,%u\n", state, leftSpeed, rightSpeed, fabs(leftSpeed - rightSpeed), tofCorrection, leftPWM, rightPWM, distance[FRONT_L], distance[FRONT_R]);
}

void Encoder::begin(uint8_t pinA, uint8_t pinB) {
    _pinA = pinA;
    _pinB = pinB;
    _aOnly = false;

    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);

    _lastState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    _count = 0;

    attachInterruptArg(digitalPinToInterrupt(_pinA), isrHandler, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_pinB), isrHandler, this, CHANGE);
}

void Encoder::beginAOnly(uint8_t pinA) {
    _pinA = pinA;
    _pinB = 0;
    _aOnly = true;

    pinMode(_pinA, INPUT_PULLUP);
    _count = 0;

    attachInterruptArg(digitalPinToInterrupt(_pinA), isrHandler, this, CHANGE);
}

long Encoder::getCount() const {
    noInterrupts();
    long count = _count;
    interrupts();
    return count;
}

void IRAM_ATTR Encoder::isrHandler(void* arg) {
    Encoder* encoder = static_cast<Encoder*>(arg);

    if (encoder->_aOnly) encoder->handleAOnlyInterrupt();
    else encoder->handleInterrupt();
}

void IRAM_ATTR Encoder::handleInterrupt() {
    static const int8_t table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };

    uint8_t a = digitalRead(_pinA);
    uint8_t b = digitalRead(_pinB);
    uint8_t state = (a << 1) | b;
    uint8_t idx = ((_lastState << 2) | state) & 0x0F;

    _count += table[idx];
    _lastState = state;
}

void IRAM_ATTR Encoder::handleAOnlyInterrupt() {
    _count++;
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

void setupBluetooth() {
    BLEDevice::init(BLE_DEVICE_NAME);

    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);

    BLEDevice::startAdvertising();
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_BUTTON, INPUT_PULLUP);

    pinMode(PIN_STBY, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RESOLUTION);

    stopMotors();

    leftEncoder.begin(PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B);
    rightEncoder.beginAOnly(PIN_ENCODER_RIGHT_A);

    setupToF();
    resetController();
    setupBluetooth();

    bluetoothPrintf("==============================\n");
    bluetoothPrintf("Micromouse BLE Debug\n");
    bluetoothPrintf("==============================\n");
    bluetoothPrintf("Press button to start\n");
}

void loop() {
    if (robotState == WAITING) {
        stopMotors();

        if (digitalRead(PIN_BUTTON) == LOW) {
            delay(30);

            if (digitalRead(PIN_BUTTON) == LOW) {
                resetController();
                robotState = DRIVING;

                bluetoothPrintf("Robot started\n");

                while (digitalRead(PIN_BUTTON) == LOW) delay(1);
            }
        }

        return;
    }

    if (robotState == DRIVING) {
        updateTofControl();
        updateSpeedControl();
        printDebug();
        return;
    }

    if (robotState == TURNING) {
        stopMotors();
        delay(TURN_STOP_DELAY_MS);
        updateTurn();
        printDebug();
        return;
    }
}
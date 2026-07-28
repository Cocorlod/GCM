ToFSensor tof;

bool started = false;

void setup() {
    Serial.begin(115200);
    beginBluetooth();

    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RESOLUTION);

    pinMode(47, INPUT_PULLUP);

    pinMode(PIN_STBY, OUTPUT);

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);

    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    tof.beginToF();

    Serial.println("Ready");
}

void loop() {
    if(bluetoothStop) {

    bluetoothStop = false;
    started = false;

    stopMotors();

    debugPrint("Robot stopped");
    }
    if(bluetoothCommand != CMD_NONE) {

    switch(bluetoothCommand) {

        case CMD_FORWARD:
            moveForward(tof);
            delay(300);
            stopMotors();
            break;

        case CMD_LEFT:
            turnLeft();
            delay(300);
            stopMotors();
            break;

        case CMD_RIGHT:
            turnRight();
            delay(300);
            stopMotors();
            break;

        case CMD_BACK:
            turnBack();
            delay(600);
            stopMotors();
            break;

        case CMD_STOP:
            stopMotors();
            break;

        default:
            break;
    }

    bluetoothCommand = CMD_NONE;
    return;
}

    if (!started && (digitalRead(47) == LOW || bluetoothStart)) {
        started = true;
        delay(200);
        Serial.println("Started");
    }

    if (!started)
        return;

    tof.update();

    bool frontWall = tof.wallDistance(FRONT) < 110;
    bool leftWall  = tof.isThereWall(LEFT);
    bool rightWall = tof.isThereWall(RIGHT);

    if (frontWall) {

        stopMotors();
        delay(20);

        // Refresh sensor readings before deciding
        tof.update();

        leftWall  = tof.isThereWall(LEFT);
        rightWall = tof.isThereWall(RIGHT);

        if (!leftWall) {

            debugPrint("Turn Left");

            turnLeft();
            delay(300);

        }
        else if (!rightWall) {

            debugPrint("Turn Right");

            turnRight();
            delay(300);

        }
        else {

            debugPrint("Turn Back");

            turnBack();
            delay(600);

        }

        stopMotors();
        delay(30);

        return;
    }

    
    moveForward(tof);

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 300) {
        debugPrint("Forward");
        lastPrint = millis();

        Serial.print("F:");
        Serial.print(tof.wallDistance(FRONT));

        Serial.print("  L:");
        Serial.print(tof.wallDistance(LEFT));

        Serial.print("  R:");
        Serial.println(tof.wallDistance(RIGHT));
    }
}


#define PIN_STBY 16
#define PIN_BIN1 18
#define PIN_BIN2 17
#define PIN_AIN1 4
#define PIN_AIN2 8
#define PIN_PWMA 9
#define PIN_PWMB 10

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define TURN_PWM 127
#define FORWARD_PWM 110

#define TURN_DELAY 200

extern float previousError;
extern uint32_t previousTime;

extern const float KP;
extern const float KD;

void moveForward(ToFSensor& tof);
void turnLeft();
void turnRight();
void turnBack();
void stopMotors();



float previousError = 0.0f;
uint32_t previousTime = 0;

const float KP = 1.2f;
const float KD = 0.08f;

static constexpr int16_t MAX_PLAUSIBLE_ALIGNMENT_ERROR = 150;

static bool wasWallFollowing = false;

void moveForward(ToFSensor& tof) {
    bool leftWall = tof.isThereWall(LEFT);
    bool rightWall = tof.isThereWall(RIGHT);

    if(!leftWall && !rightWall) {
        digitalWrite(PIN_STBY, HIGH);
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);    
        ledcWrite(PIN_PWMA, FORWARD_PWM);
        ledcWrite(PIN_PWMB, FORWARD_PWM);

        if(wasWallFollowing) {
            previousError = 0.0f;
            previousTime = millis();
            wasWallFollowing = false;
        }

        return;
    }

    float error = 0.0f;

    if(leftWall && rightWall) {
        error = (float)(tof.alignmentError(LEFT) - tof.alignmentError(RIGHT)) / 2.0f;
    }
    else if(leftWall) {
        error = (float)tof.alignmentError(LEFT);
    }
    else {
        error = -(float)tof.alignmentError(RIGHT);
    }

    error = constrain(error, -MAX_PLAUSIBLE_ALIGNMENT_ERROR, MAX_PLAUSIBLE_ALIGNMENT_ERROR);

    uint32_t now = millis();
    float dt = (now - previousTime) / 1000.0f;

    if(dt <= 0.0f || !wasWallFollowing) dt = 0.001f;

    float derivative = (error - previousError) / dt;
    float correction = KP * error + KD * derivative;

    int leftPWM = constrain(FORWARD_PWM - correction, 0, 255);
    int rightPWM = constrain(FORWARD_PWM + correction, 0, 255);

    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PIN_PWMA, leftPWM);
    ledcWrite(PIN_PWMB, rightPWM);

    previousError = error;
    previousTime = now;
    wasWallFollowing = true;
}

void turnLeft() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PIN_PWMA, TURN_PWM);
    ledcWrite(PIN_PWMB, TURN_PWM);
}

void turnRight() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    ledcWrite(PIN_PWMA, TURN_PWM);
    ledcWrite(PIN_PWMB, TURN_PWM);
}

void turnBack() {
    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);    
    ledcWrite(PIN_PWMA, TURN_PWM + 34);
    ledcWrite(PIN_PWMB, TURN_PWM + 34);
}

void stopMotors() {
    digitalWrite(PIN_STBY, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}

#include <Arduino.h>
#include <VL53L1X.h>
#include <Wire.h> 

#define PIN_SCL 11 
#define PIN_SDA 12
#define I2C_CLOCK 400000UL
#define I2C_DEFAULT_ADDRESS 0x30

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
#define OFFSET_CENTER 10

static constexpr int16_t MAX_ALLOWED_DIFF = 90;
static constexpr int16_t FRONT_WALL_THRESHOLD = 250;
static constexpr int16_t SIDE_WALL_THRESHOLD = 250;
static constexpr int16_t FRONT_WALL_THRESHOLD_CENTER = 93;
static constexpr int16_t SIDE_WALL_THRESHOLD_CENTER = 87;

static constexpr uint16_t SENSOR_INVALID_DISTANCE = 9999;
static constexpr float TOF_TILT_CORRECTION = 1.9;

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
        bool sensorOk(SensorID id) const;

        bool isThereWall(WallSides side) const;

        float wallDistance(WallSides side) const; 
        
        int16_t alignmentError(WallSides side) const;

        uint16_t getDistance(SensorID id) const;
    private:
        VL53L1X sensor[SENSOR_COUNT];
        bool ok[SENSOR_COUNT] = {false};

        uint16_t distance[SENSOR_COUNT] = {0};
        static const uint8_t XSHUTPIN[SENSOR_COUNT];
};


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
            digitalWrite(XSHUTPIN[i], LOW);
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

    if(allOk) {
        Serial.println("All ToF sensors initialized successfully.");
    } else {
        Serial.println("ToF Failed");
    }

    return allOk;
}

void ToFSensor::update() {
    for(uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if(!ok[i]) continue;

        if(sensor[i].dataReady()) {
            if(!sensor[i].timeoutOccurred()) {

                uint16_t raw = sensor[i].read(false);

                // Apply tilt correction only to the front sensors
                if(i == FRONT_R || i == FRONT_L) {
                    distance[i] = (uint16_t)(raw * TOF_TILT_CORRECTION);
                } else {
                    distance[i] = raw;
                }

            } else {
                ok[i] = false;
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

bool ToFSensor::sensorOk(SensorID id) const {
    return ok[id];
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

float ToFSensor::wallDistance(WallSides side) const {
    switch(side) {

        case FRONT: {
            bool lOk = ok[FRONT_L];
            bool rOk = ok[FRONT_R];

            if(!lOk && !rOk) return SENSOR_INVALID_DISTANCE;
            if(!lOk) return distance[FRONT_R];
            if(!rOk) return distance[FRONT_L];

            if(abs((int)distance[FRONT_L] - (int)distance[FRONT_R]) < MAX_ALLOWED_DIFF) {
                return (distance[FRONT_L] + distance[FRONT_R]) / 2.0f;
            }

            return min(distance[FRONT_L], distance[FRONT_R]);
        }

        case LEFT: {
            bool fOk = ok[LEFT_F];
            bool bOk = ok[LEFT_B];

            if(!fOk && !bOk) return SENSOR_INVALID_DISTANCE;
            if(!fOk) return distance[LEFT_B];
            if(!bOk) return distance[LEFT_F];

            if(abs((int)distance[LEFT_F] - (int)distance[LEFT_B]) < MAX_ALLOWED_DIFF) {
                return (distance[LEFT_F] + distance[LEFT_B]) / 2.0f;
            }

            return min(distance[LEFT_F], distance[LEFT_B]);
        }

        case RIGHT: {
            bool fOk = ok[RIGHT_F];
            bool bOk = ok[RIGHT_B];

            if(!fOk && !bOk) return SENSOR_INVALID_DISTANCE;
            if(!fOk) return distance[RIGHT_B];
            if(!bOk) return distance[RIGHT_F];

            if(abs((int)distance[RIGHT_F] - (int)distance[RIGHT_B]) < MAX_ALLOWED_DIFF) {
                return (distance[RIGHT_F] + distance[RIGHT_B]) / 2.0f;
            }

            return min(distance[RIGHT_F], distance[RIGHT_B]);
        }

        default:
            return 0.0f;
    }
}

int16_t ToFSensor::alignmentError(WallSides side) const {
    if(!isThereWall(side)) return 0;

    switch(side) {

        case LEFT:
            if(!ok[LEFT_F] || !ok[LEFT_B]) return 0;
            return (int16_t)distance[LEFT_F] - (int16_t)distance[LEFT_B];

        case RIGHT:
            if(!ok[RIGHT_F] || !ok[RIGHT_B]) return 0;
            return (int16_t)distance[RIGHT_F] - (int16_t)distance[RIGHT_B];

        default:
            return 0;
    }
}

uint16_t ToFSensor::getDistance(SensorID id) const {
    if(!ok[id]) return 0;
    return distance[id];
}
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

extern volatile bool bluetoothStart;
extern volatile bool bluetoothStop; 

enum RobotCommand {
    CMD_NONE,
    CMD_FORWARD,
    CMD_LEFT,
    CMD_RIGHT,
    CMD_BACK,
    CMD_STOP
};

extern volatile RobotCommand bluetoothCommand;

void beginBluetooth();

void debugPrint(const char *msg);

void debugPrintf(const char *format, ...);

volatile bool bluetoothStart = false;
volatile bool bluetoothStop = false;
volatile RobotCommand bluetoothCommand = CMD_NONE;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* server = nullptr;
BLEService* service = nullptr;

BLECharacteristic* txCharacteristic = nullptr;
BLECharacteristic* rxCharacteristic = nullptr;

bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {

    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("BLE Connected");
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("BLE Disconnected");

        delay(100);

        BLEDevice::startAdvertising();
    }
};

class RXCallbacks : public BLECharacteristicCallbacks {

    void onWrite(BLECharacteristic* characteristic) override {

    String cmd = characteristic->getValue().c_str();

    cmd.trim();
    cmd.toUpperCase();

    debugPrintf("RX: %s", cmd.c_str());

    if(cmd == "START") {
        bluetoothStart = true;
    }

    else if(cmd == "STOP") {
        bluetoothStop = true;
        bluetoothCommand = CMD_STOP;
    }

    else if(cmd == "FORWARD") {
        bluetoothCommand = CMD_FORWARD;
    }

    else if(cmd == "LEFT") {
        bluetoothCommand = CMD_LEFT;
    }

    else if(cmd == "RIGHT") {
        bluetoothCommand = CMD_RIGHT;
    }

    else if(cmd == "BACK") {
        bluetoothCommand = CMD_BACK;
    }

    else if(cmd == "RESET") {

        debugPrint("Restarting...");
        delay(100);
        ESP.restart();
    }
}
};

void beginBluetooth() {

    BLEDevice::init("GCM Micromouse");

    server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    service = server->createService(SERVICE_UUID);

    txCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );

    txCharacteristic->addDescriptor(new BLE2902());

    rxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );

    rxCharacteristic->setCallbacks(new RXCallbacks());

    service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();

    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();

    Serial.println("BLE Advertising Started");
}

void debugPrint(const char* msg) {

    Serial.println(msg);

    if(deviceConnected && txCharacteristic) {
        txCharacteristic->setValue((uint8_t*)msg, strlen(msg));
        txCharacteristic->notify();
    }
}

void debugPrintf(const char* format, ...) {

    char buffer[256];

    va_list args;

    va_start(args, format);

    vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    debugPrint(buffer);
}
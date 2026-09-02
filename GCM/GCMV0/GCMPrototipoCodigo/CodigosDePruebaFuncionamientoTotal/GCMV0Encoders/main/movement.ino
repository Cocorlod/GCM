#include "movement.hpp"

float previousError = 0.0f;
uint32_t previousTime = 0;

const float KP = 1.2f;
const float KD = 0.08f;

static constexpr int16_t MAX_PLAUSIBLE_ALIGNMENT_ERROR = 150;

static bool wasWallFollowing = false;

Encoder leftEncoder;
Encoder rightEncoder;

void Encoder::begin(uint8_t pinA, uint8_t pinB) {
    _pinA = pinA;
    _pinB = pinB;
 
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
 
    _lastState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    _count = 0;
 
    attachInterruptArg(digitalPinToInterrupt(_pinA), isrHandler, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_pinB), isrHandler, this, CHANGE);
}

long Encoder::getCount() const {
    noInterrupts();
    long c = _count;
    interrupts();
    return c;
}

void Encoder::reset() {
    noInterrupts();
    _count = 0;
    interrupts();
}

void IRAM_ATTR Encoder::isrHandler(void* arg) {
    static_cast<Encoder*>(arg)->handleInterrupt();
}
 
void IRAM_ATTR Encoder::handleInterrupt() {
    static const int8_t table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };
 
    uint8_t a = digitalRead(_pinA);
    uint8_t b = digitalRead(_pinB);
    uint8_t state = (a << 1) | b;
 
    uint8_t idx = (uint8_t)((_lastState << 2) | state) & 0x0F;
    _count += table[idx];
 
    _lastState = state;
}

static constexpr float KP_SPEED = 0.6f;  
static constexpr float KD_SPEED = 0.02f;  

static long lastLeftCountSpeed  = 0;
static long lastRightCountSpeed = 0;
static uint32_t lastSpeedTime = 0;
static float prevSpeedError = 0.0f;
static bool speedLoopInitialized = false;
 
static void resetSpeedSyncLoop() {
    speedLoopInitialized = false;
}
 
static float computeSpeedSyncCorrection() {
    long leftCount  = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();
    uint32_t now = micros();
 
    if (!speedLoopInitialized) {
        lastLeftCountSpeed  = leftCount;
        lastRightCountSpeed = rightCount;
        lastSpeedTime       = now;
        prevSpeedError      = 0.0f;
        speedLoopInitialized = true;
        return 0.0f;
    }
 
    float dt = max((now - lastSpeedTime) * 1e-6f, 0.001f);
 
    float leftSpeed  = (leftCount  - lastLeftCountSpeed)  * MM_PER_COUNT / dt;
    float rightSpeed = (rightCount - lastRightCountSpeed) * MM_PER_COUNT / dt;
 
    float error = leftSpeed - rightSpeed;
    float derivative = (error - prevSpeedError) / dt;
    float correction = KP_SPEED * error + KD_SPEED * derivative;
 
    lastLeftCountSpeed  = leftCount;
    lastRightCountSpeed = rightCount;
    lastSpeedTime = now;
    prevSpeedError = error;
 
    return correction; 
}

void moveForward(ToFSensor& tof) {
    bool leftWall  = tof.isThereWall(WALL_LEFT);
    bool rightWall = tof.isThereWall(WALL_RIGHT);
 
    float wallCorrection = 0.0f;
 
    if (leftWall || rightWall) {
        float error;
 
        if (leftWall && rightWall) {
            error = (tof.alignmentError(WALL_LEFT) - tof.alignmentError(WALL_RIGHT)) * 0.5f;
        } else if (leftWall) {
            error = tof.alignmentError(WALL_LEFT);
        } else {
            error = -tof.alignmentError(WALL_RIGHT);
        }
 
        error = constrain(error, -MAX_PLAUSIBLE_ALIGNMENT_ERROR, MAX_PLAUSIBLE_ALIGNMENT_ERROR);
 
        uint32_t now = micros();
 
        if (!wasWallFollowing) {
            previousError = error;
            previousTime = now;
            wasWallFollowing = true;
        }
 
        float dt = max((now - previousTime) * 1e-6f, 0.001f);
        float derivative = (error - previousError) / dt;
 
        wallCorrection = KP * error + KD * derivative;
 
        previousError = error;
        previousTime = now;
    } else {
        wasWallFollowing = false;
    }
 
    float speedCorrection = computeSpeedSyncCorrection();
    float totalCorrection = wallCorrection + speedCorrection;
 
    int leftPWM  = constrain((int)(FORWARD_PWM - totalCorrection), 0, 255);
    int rightPWM = constrain((int)(FORWARD_PWM + totalCorrection), 0, 255);
 
    digitalWrite(PIN_STBY, HIGH);
 
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
 
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
 
    ledcWrite(PIN_PWMA, leftPWM);
    ledcWrite(PIN_PWMB, rightPWM);
}

static float arcLengthForAngleDeg(float angleDeg) {
    float angleRad = angleDeg * PI / 180.0f;
    return angleRad * WHEEL_TRACK_MM;
}

static void setTurnDirection(bool clockwise) {
    if (clockwise) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, LOW);
    } else {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, LOW);
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
    }
}

static void performTurn(float angleDeg, bool clockwise) {
    long startLeft = leftEncoder.getCount();
    long startRight = rightEncoder.getCount();

    float arcMM = arcLengthForAngleDeg(angleDeg);
    long targetCounts = (long)(arcMM / MM_PER_COUNT);

    digitalWrite(PIN_STBY, HIGH);
    setTurnDirection(clockwise);

    uint32_t start = millis();
 
    while (millis() - start < TURN_TIMEOUT_MS) {
        long counts;

        if(clockwise) {
            counts = abs(leftEncoder.getCount() - startLeft);
        } else {
            counts = abs(rightEncoder.getCount() - startRight);
        }
 
        if (counts >= targetCounts) break;
 
        long remaining = targetCounts - counts;
        uint8_t pwm = (remaining < counts / 4) ? TURN_PWM_SLOW : TURN_PWM;
 
        ledcWrite(PIN_PWMA, clockwise ? pwm : 0);
        ledcWrite(PIN_PWMB, clockwise ? 0 : pwm);
    }
 
    stopMotors();
    resetSpeedSyncLoop();
}

void turn(Turn dir) {
    switch (dir) {
        case LEFT:
            performTurn(90.0f, false);
            break;
        case RIGHT:
            performTurn(90.0f, true);
            break;
        case BACK:
            performTurn(180.0f, true);
            break;
    }
}

void stopMotors() {
    digitalWrite(PIN_STBY, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}
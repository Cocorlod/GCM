#include <Arduino.h>

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

#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

#define TARGET_SPEED_MM_S 80.0f

#define SPEED_LOOP_PERIOD_MS 50
#define SERIAL_PERIOD_MS 1000

#define BASE_PWM_LEFT 61
#define BASE_PWM_RIGHT 57

#define KP_LEFT 0.15f
#define KI_LEFT 0.40f

#define KP_RIGHT 0.15f
#define KI_RIGHT 0.40f

#define MAX_INTEGRAL 300.0f

static constexpr float WHEEL_DIAMETER_MM = 19.0f;
static constexpr float WHEEL_CIRCUMFERENCE_MM = PI * WHEEL_DIAMETER_MM;

static constexpr float ENCODER_CPR_MOTOR_SHAFT = 12.0f;
static constexpr float GEAR_RATIO = 4.995f;

static constexpr float LEFT_COUNTS_PER_WHEEL_REV = ENCODER_CPR_MOTOR_SHAFT * GEAR_RATIO * 4.0f;
static constexpr float RIGHT_COUNTS_PER_WHEEL_REV = ENCODER_CPR_MOTOR_SHAFT * GEAR_RATIO * 2.0f;

static constexpr float LEFT_MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / LEFT_COUNTS_PER_WHEEL_REV;
static constexpr float RIGHT_MM_PER_COUNT = WHEEL_CIRCUMFERENCE_MM / RIGHT_COUNTS_PER_WHEEL_REV;

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

struct PIController {
    float kp;
    float ki;

    float integral = 0.0f;
};

PIController leftPI = {
    KP_LEFT,
    KI_LEFT
};

PIController rightPI = {
    KP_RIGHT,
    KI_RIGHT
};

long lastLeftControlCount = 0;
long lastRightControlCount = 0;

long lastLeftSerialCount = 0;
long lastRightSerialCount = 0;

uint32_t lastSpeedControlTime = 0;
uint32_t lastSerialTime = 0;

float leftSpeed = 0.0f;
float rightSpeed = 0.0f;

int leftPWM = BASE_PWM_LEFT;
int rightPWM = BASE_PWM_RIGHT;

bool started = false;

float updatePI(PIController& pi, float targetSpeed, float measuredSpeed, float dt)
{
    float error = targetSpeed - measuredSpeed;

    pi.integral += error * dt;
    pi.integral = constrain(pi.integral, -MAX_INTEGRAL, MAX_INTEGRAL);

    return pi.kp * error + pi.ki * pi.integral;
}

void setMotors(int left, int right)
{
    left = constrain(left, 0, 255);
    right = constrain(right, 0, 255);

    digitalWrite(PIN_STBY, HIGH);

    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);

    ledcWrite(PIN_PWMA, left);
    ledcWrite(PIN_PWMB, right);
}

void stopMotors()
{
    digitalWrite(PIN_STBY, LOW);

    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}

void resetController()
{
    leftPI.integral = 0.0f;
    rightPI.integral = 0.0f;

    leftPWM = BASE_PWM_LEFT;
    rightPWM = BASE_PWM_RIGHT;

    lastLeftControlCount = leftEncoder.getCount();
    lastRightControlCount = rightEncoder.getCount();

    lastLeftSerialCount = leftEncoder.getCount();
    lastRightSerialCount = rightEncoder.getCount();

    lastSpeedControlTime = millis();
    lastSerialTime = millis();
}

void updateSpeedControl()
{
    uint32_t now = millis();

    if (now - lastSpeedControlTime < SPEED_LOOP_PERIOD_MS) {
        return;
    }

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

    leftPWM = constrain((int)(BASE_PWM_LEFT + leftCorrection), 0, 255);
    rightPWM = constrain((int)(BASE_PWM_RIGHT + rightCorrection), 0, 255);

    setMotors(leftPWM, rightPWM);
}

void printDebug()
{
    uint32_t now = millis();

    if (now - lastSerialTime < SERIAL_PERIOD_MS) {
        return;
    }

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

    Serial.print("TARGET: ");
    Serial.print(TARGET_SPEED_MM_S, 1);

    Serial.print(" mm/s | L: ");
    Serial.print(leftSpeed, 1);

    Serial.print(" mm/s | R: ");
    Serial.print(rightSpeed, 1);

    Serial.print(" mm/s | DIFF: ");
    Serial.print(fabs(leftSpeed - rightSpeed), 1);

    Serial.print(" mm/s | PWM L: ");
    Serial.print(leftPWM);

    Serial.print(" | PWM R: ");
    Serial.println(rightPWM);
}

void Encoder::begin(uint8_t pinA, uint8_t pinB)
{
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

void Encoder::beginAOnly(uint8_t pinA)
{
    _pinA = pinA;
    _pinB = 0;
    _aOnly = true;

    pinMode(_pinA, INPUT_PULLUP);

    _count = 0;

    attachInterruptArg(digitalPinToInterrupt(_pinA), isrHandler, this, CHANGE);
}

long Encoder::getCount() const
{
    noInterrupts();

    long count = _count;

    interrupts();

    return count;
}

void IRAM_ATTR Encoder::isrHandler(void* arg)
{
    Encoder* encoder = static_cast<Encoder*>(arg);

    if (encoder->_aOnly) {
        encoder->handleAOnlyInterrupt();
    } else {
        encoder->handleInterrupt();
    }
}

void IRAM_ATTR Encoder::handleInterrupt()
{
    static const int8_t table[16] = {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };

    uint8_t a = digitalRead(_pinA);
    uint8_t b = digitalRead(_pinB);

    uint8_t state = (a << 1) | b;

    uint8_t idx = ((_lastState << 2) | state) & 0x0F;

    _count += table[idx];

    _lastState = state;
}

void IRAM_ATTR Encoder::handleAOnlyInterrupt()
{
    _count++;
}

void setup()
{
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

    resetController();

    Serial.println("Press button to start");
}

void loop()
{
    if (!started) {
        stopMotors();

        if (digitalRead(PIN_BUTTON) == LOW) {
            delay(30);

            if (digitalRead(PIN_BUTTON) == LOW) {
                started = true;

                resetController();

                Serial.println("Speed controller started");

                while (digitalRead(PIN_BUTTON) == LOW) {
                    delay(1);
                }
            }
        }

        return;
    }

    updateSpeedControl();
    printDebug();
}
#include "main.hpp"
#include "bluetooth.hpp"

RobotFSM robot;

void setup() {
    Serial.begin(SERIAL_SPEED);
    beginBluetooth();

    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RESOLUTION);

    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CLEAR_BUTTON_PIN, INPUT_PULLUP);

    pinMode(LED_DEBUG_LASER, OUTPUT);
    pinMode(LED_TEST, OUTPUT);
    pinMode(LED_MOUNTED, OUTPUT);
    pinMode(LED_MODE, OUTPUT);

    pinMode(PIN_STBY, OUTPUT);

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);

    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    tof.beginToF();

    robot.config();
}

void loop() {
    tof.update();
    
    robot.update();
}
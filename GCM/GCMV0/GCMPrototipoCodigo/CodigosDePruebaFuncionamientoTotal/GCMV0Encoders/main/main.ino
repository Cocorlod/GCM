#include "main.hpp"

RobotFSM FSM;
ToFSensor tof;

void setup() {
    Serial.begin(SERIAL_SPEED);
    beginBluetooth();

    pinMode(START_BUTTON_PIN, INPUT_PULLUP);
    pinMode(CLEAR_BUTTON_PIN, INPUT_PULLUP);

    ledcAttach(PIN_PWMA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_PWMB, PWM_FREQ, PWM_RESOLUTION);

    pinMode(PIN_STBY, OUTPUT);

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);

    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    pinMode(LED_DEBUG_LASER, OUTPUT);
    pinMode(LED_TEST, OUTPUT);
    pinMode(LED_MOUNTED, OUTPUT);
    pinMode(LED_MODE, OUTPUT);

    leftEncoder.begin(PIN_ENCODER_LEFT_A, PIN_ENCODER_LEFT_B);
    rightEncoder.begin(PIN_ENCODER_RIGHT_A, PIN_ENCODER_RIGHT_B);

    tof.beginToF();
    
    FSM.config();

    debugPrint("Setup finished");
}

void loop() {
    tof.update();

    FSM.update();
}
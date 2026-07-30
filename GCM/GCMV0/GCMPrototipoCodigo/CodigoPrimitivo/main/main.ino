#include "ToF_Setup.hpp"
#include "movement.hpp"
#include "bluetooth.hpp"

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
    if (bluetoothStop) {
        bluetoothStop = false;
        started = false;

        stopMotors();
        debugPrint("Robot stopped");
    }

    if (bluetoothCommand != CMD_NONE) {

        switch (bluetoothCommand) {

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
        bluetoothStart = false;

        delay(200);

        Serial.println("Started");
    }

    if (!started) return;

    tof.update();

    bool frontWall = tof.isThereWall(FRONT);
    bool leftWall  = tof.isThereWall(LEFT);
    bool rightWall = tof.isThereWall(RIGHT);

    if (frontWall) {

        stopMotors();
        delay(2000);

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

        Serial.print("F: ");
        Serial.print(tof.wallDistance(FRONT));

        Serial.print("  L: ");
        Serial.print(tof.wallDistance(LEFT));

        Serial.print("  R: ");
        Serial.println(tof.wallDistance(RIGHT));

        lastPrint = millis();
    }
}
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
                turn(LEFT, tof);
                delay(300);
                stopMotors();
                break;

            case CMD_RIGHT:
                turn(RIGHT, tof);
                delay(300);
                stopMotors();
                break;

            case CMD_BACK:
                turn(BACK, tof);
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

        while (true) {

            tof.update();

            bool ready = true;

            for (int i = 0; i < SENSOR_COUNT; i++) {
                ready &= tof.sensorOk((SensorID)i) && tof.getDistance((SensorID)i) != 0;
            }

            if (ready)
            break;

            delay(5);
        }

    Serial.println("Started");

}  

    if (!started) return;

    tof.update();

    bool frontWall = tof.isThereWall(WALL_FRONT);
    bool leftWall  = tof.isThereWall(WALL_LEFT);
    bool rightWall = tof.isThereWall(WALL_RIGHT);

    if (frontWall) {

        stopMotors();
        delay(2000);

        tof.update();

        leftWall  = tof.isThereWall(WALL_LEFT);
        rightWall = tof.isThereWall(WALL_RIGHT);

        if (!leftWall) {

            debugPrint("Turn Left");

            turn(LEFT, tof);
            delay(3000);

        }
        else if (!rightWall) {

            debugPrint("Turn Right");

            turn(RIGHT, tof);
            delay(3000);

        }
        else {

            debugPrint("Turn Back");
            
            turn(BACK, tof);
            delay(6000);

        }

        stopMotors();
        delay(30);

        return;
    }

    moveForward(tof);

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 300) {

        Serial.print("F: ");
        Serial.print(tof.wallDistance(WALL_FRONT));

        Serial.print("  L: ");
        Serial.print(tof.wallDistance(WALL_LEFT));

        Serial.print("  R: ");
        Serial.println(tof.wallDistance(WALL_RIGHT));

        lastPrint = millis();
    }
}
/*
Serial: 115200

LEDS:
-Debug Laser: GPIO41
-Test LED: GPIO39
-Robot mounted and ready: GPIO38
-Mode: GPIO42, (Modes indicates exploration, speedrun or clearing memory)

SWITCHES:
Start: GPIO47, when pressed it checks whether there has a maze saved in memory, if so it 
loads it and starts the speedrun, if not it starts the exploration
Clear: GPIO48, erases memory so that the robot can explore a new maze

The robot must keep the maze after its turned off and only erases the memory when clear is pressed

Board: ESP32 S3 1N16R8

*/

#include "FSM.hpp"

RobotFSM robot;

void setup() {
    Serial.begin(115200);
    robot.setup();
}

void loop() {
    robot.update();
}
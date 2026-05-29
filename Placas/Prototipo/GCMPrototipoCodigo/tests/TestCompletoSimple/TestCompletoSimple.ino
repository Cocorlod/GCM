/*
════════════════════════════════════════════════════════════
 GCM Interactive Beta Test
 ESP32-S3 + TB6612FNG + 6x VL53L1X
════════════════════════════════════════════════════════════

INTERACTIVE BEHAVIOR

- If front sensors detect obstacle:
    -> robot stops

- If left side is closer:
    -> turn right

- If right side is closer:
    -> turn left

- If path is clear:
    -> move forward

- If IR detects white:
    -> FULL STOP

This is NOT the final robot firmware.
This is only hardware + sensor integration validation.
*/

#include <Wire.h>
#include <VL53L1X.h>

// =========================================================
// MOTOR DRIVER
// =========================================================

#define AIN1 4
#define AIN2 8
#define BIN1 18
#define BIN2 17

#define PWMA 9
#define PWMB 10

#define STBY 16

// =========================================================
// IR SENSOR
// =========================================================

#define IR_PIN 15

// =========================================================
// BUTTONS
// =========================================================

#define START_BTN 13
#define CLEAR_BTN 14

// =========================================================
// LEDS
// =========================================================

#define LED_START 47
#define LED_CLEAR 48

// =========================================================
// I2C
// =========================================================

#define SDA_PIN 12
#define SCL_PIN 11

// =========================================================
// VL53L1X XSHUT
// =========================================================

#define XSHUT1 1
#define XSHUT2 2
#define XSHUT3 38
#define XSHUT4 37
#define XSHUT5 36
#define XSHUT6 35

const int xshutPins[6] = {
  XSHUT1,
  XSHUT2,
  XSHUT3,
  XSHUT4,
  XSHUT5,
  XSHUT6
};

// =========================================================
// SENSOR ADDRESSES
// =========================================================

const uint8_t sensorAddresses[6] = {
  0x30,
  0x31,
  0x32,
  0x33,
  0x34,
  0x35
};

// =========================================================
// SENSOR OBJECTS
// =========================================================

VL53L1X sensors[6];

int dist[6];

// =========================================================
// MOTOR CONTROL
// =========================================================

void motorA(int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  }
  else
  {
    analogWrite(PWMA, 0);
  }
}

void motorB(int speed)
{
  speed = constrain(speed, -255, 255);

  if (speed > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  }
  else
  {
    analogWrite(PWMB, 0);
  }
}

void moveForward(int speed)
{
  motorA(speed);
  motorB(speed);
}

void moveBackward(int speed)
{
  motorA(-speed);
  motorB(-speed);
}

void turnLeft(int speed)
{
  motorA(-speed);
  motorB(speed);
}

void turnRight(int speed)
{
  motorA(speed);
  motorB(-speed);
}

void stopMotors()
{
  motorA(0);
  motorB(0);
}

// =========================================================
// VL53L1X INIT
// =========================================================

void initVL53()
{
  Serial.println("Initializing VL53L1X...");

  for (int i = 0; i < 6; i++)
  {
    pinMode(xshutPins[i], OUTPUT);
    digitalWrite(xshutPins[i], LOW);
  }

  delay(100);

  for (int i = 0; i < 6; i++)
  {
    digitalWrite(xshutPins[i], HIGH);

    delay(100);

    if (!sensors[i].init())
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.println(" FAILED");
      continue;
    }

    sensors[i].setAddress(sensorAddresses[i]);

    sensors[i].setDistanceMode(VL53L1X::Short);

    sensors[i].setMeasurementTimingBudget(20000);

    sensors[i].startContinuous(20);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.println(" OK");
  }
}

// =========================================================
// READ SENSORS
// =========================================================

void readSensors()
{
  for (int i = 0; i < 6; i++)
  {
    dist[i] = sensors[i].read();

    if (sensors[i].timeoutOccurred())
    {
      dist[i] = 9999;
    }
  }
}

// =========================================================
// SETUP
// =========================================================

void setup()
{
  Serial.begin(115200);

  // Motors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  // IR
  pinMode(IR_PIN, INPUT);

  // Buttons
  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(CLEAR_BTN, INPUT_PULLUP);

  // LEDs
  pinMode(LED_START, OUTPUT);
  pinMode(LED_CLEAR, OUTPUT);

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Sensors
  initVL53();

  Serial.println("READY");
}

// =========================================================
// LOOP
// =========================================================

void loop()
{
  // =====================================================
  // BUTTONS
  // =====================================================

  bool startPressed = !digitalRead(START_BTN);
  bool clearPressed = !digitalRead(CLEAR_BTN);

  digitalWrite(LED_START, startPressed);
  digitalWrite(LED_CLEAR, clearPressed);

  // =====================================================
  // CLEAR BUTTON
  // =====================================================

  if (clearPressed)
  {
    Serial.println("CLEAR -> STOP");

    stopMotors();

    delay(200);
    return;
  }

  // =====================================================
  // WAIT FOR START
  // =====================================================

  if (!startPressed)
  {
    stopMotors();
    return;
  }

  // =====================================================
  // READ TOF
  // =====================================================

  readSensors();

  // Example layout:
  //
  // [0] left
  // [1] front-left
  // [2] front
  // [3] front-right
  // [4] right
  // [5] rear

  int left      = dist[0];
  int frontLeft = dist[1];
  int front     = dist[2];
  int frontRight= dist[3];
  int right     = dist[4];

  // =====================================================
  // IR SENSOR
  // =====================================================

  int ir = digitalRead(IR_PIN);

  // WHITE DETECTED
  if (ir == HIGH)
  {
    Serial.println("WHITE DETECTED -> STOP");

    stopMotors();

    delay(50);

    return;
  }

  // =====================================================
  // DEBUG PRINT
  // =====================================================

  Serial.print("L:");
  Serial.print(left);

  Serial.print(" FL:");
  Serial.print(frontLeft);

  Serial.print(" F:");
  Serial.print(front);

  Serial.print(" FR:");
  Serial.print(frontRight);

  Serial.print(" R:");
  Serial.println(right);

  // =====================================================
  // BASIC REACTIVE NAVIGATION
  // =====================================================

  // Obstacle directly ahead
  if (front < 120)
  {
    Serial.println("FRONT BLOCKED");

    // Choose freer side
    if (left > right)
    {
      Serial.println("TURN LEFT");
      turnLeft(120);
    }
    else
    {
      Serial.println("TURN RIGHT");
      turnRight(120);
    }
  }

  // Too close left wall
  else if (left < 80)
  {
    Serial.println("TOO CLOSE LEFT");
    turnRight(100);
  }

  // Too close right wall
  else if (right < 80)
  {
    Serial.println("TOO CLOSE RIGHT");
    turnLeft(100);
  }

  // Clear path
  else
  {
    Serial.println("FORWARD");
    moveForward(140);
  }

  delay(30);
}
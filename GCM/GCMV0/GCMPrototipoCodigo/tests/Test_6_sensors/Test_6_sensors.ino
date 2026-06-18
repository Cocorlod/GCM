#include <Wire.h>
#include <VL53L1X.h>

#define XSHUT_1 1
#define XSHUT_2 5
#define XSHUT_3 2
#define XSHUT_4 37
#define XSHUT_5 36
#define XSHUT_6 7

VL53L1X sensor1;
VL53L1X sensor2;
VL53L1X sensor3;
VL53L1X sensor4;
VL53L1X sensor5;
VL53L1X sensor6;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(12, 11);

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_5, OUTPUT);
  pinMode(XSHUT_6, OUTPUT);

  // Turn off all sensors
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_5, LOW);
  digitalWrite(XSHUT_6, LOW);

  delay(100);

  // Sensor 1
  digitalWrite(XSHUT_1, HIGH);
  delay(100);

  sensor1.setTimeout(500);
  if (!sensor1.init()) {
    Serial.println("Error sensor 1");
    while (1);
  }
  sensor1.setAddress(0x30);

  // Sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(100);

  sensor2.setTimeout(500);
  if (!sensor2.init()) {
    Serial.println("Error sensor 2");
    while (1);
  }
  sensor2.setAddress(0x31);

  // Sensor 3
  digitalWrite(XSHUT_3, HIGH);
  delay(100);

  sensor3.setTimeout(500);
  if (!sensor3.init()) {
    Serial.println("Error sensor 3");
    while (1);
  }
  sensor3.setAddress(0x32);

  // Sensor 4
  digitalWrite(XSHUT_4, HIGH);
  delay(100);

  sensor4.setTimeout(500);
  if (!sensor4.init()) {
    Serial.println("Error sensor 4");
    while (1);
  }
  sensor4.setAddress(0x33);

  // Sensor 5
  digitalWrite(XSHUT_5, HIGH);
  delay(100);

  sensor5.setTimeout(500);
  if (!sensor5.init()) {
    Serial.println("Error sensor 5");
    while (1);
  }
  sensor5.setAddress(0x34);

  // Sensor 6
  digitalWrite(XSHUT_6, HIGH);
  delay(100);

  sensor6.setTimeout(500);
  if (!sensor6.init()) {
    Serial.println("Error sensor 6");
    while (1);
  }
  sensor6.setAddress(0x35);

  // Start ranging
  sensor1.startContinuous(50);
  sensor2.startContinuous(50);
  sensor3.startContinuous(50);
  sensor4.startContinuous(50);
  sensor5.startContinuous(50);
  sensor6.startContinuous(50);

  Serial.println("All 6 sensors initialized");
}

void loop() {
  int d1 = sensor1.read();
  int d2 = sensor2.read();
  int d3 = sensor3.read();
  int d4 = sensor4.read();
  int d5 = sensor5.read();
  int d6 = sensor6.read();

  Serial.print("S1: ");
  Serial.print(d1);
  Serial.print(" mm\t");

  Serial.print("S2: ");
  Serial.print(d2);
  Serial.print(" mm\t");

  Serial.print("S3: ");
  Serial.print(d3);
  Serial.print(" mm\t");

  Serial.print("S4: ");
  Serial.print(d4);
  Serial.print(" mm\t");

  Serial.print("S5: ");
  Serial.print(d5);
  Serial.print(" mm\t");

  Serial.print("S6: ");
  Serial.print(d6);
  Serial.println(" mm");

  delay(100);
}
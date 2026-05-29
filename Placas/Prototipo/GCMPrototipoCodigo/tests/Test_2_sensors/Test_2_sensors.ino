#include <Wire.h>
#include <VL53L1X.h>

#define XSHUT_1 1
#define XSHUT_2 2

VL53L1X sensor1;
VL53L1X sensor2;

void setup() {

  Serial.begin(115200);
  delay(1000);

  Wire.begin(12, 11);

  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  // Apagar ambos
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);

  delay(100);

  // =====================
  // SENSOR 1
  // =====================

  digitalWrite(XSHUT_1, HIGH);
  delay(100);

  sensor1.setTimeout(500);

  if (!sensor1.init()) {
    Serial.println("Error sensor 1");
    while (1);
  }

  sensor1.setAddress(0x30);

  // =====================
  // SENSOR 2
  // =====================

  digitalWrite(XSHUT_2, HIGH);
  delay(100);

  sensor2.setTimeout(500);

  if (!sensor2.init()) {
    Serial.println("Error sensor 2");
    while (1);
  }

  // sensor2 queda en 0x29

  sensor1.startContinuous(50);
  sensor2.startContinuous(50);

  Serial.println("Sensores OK");
}

void loop() {

  int d1 = sensor1.read();
  int d2 = sensor2.read();

  Serial.print("S1: ");
  Serial.print(d1);

  Serial.print(" mm   ");

  Serial.print("S2: ");
  Serial.print(d2);

  Serial.println(" mm");

  delay(100);
}
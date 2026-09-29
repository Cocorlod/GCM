#include "tca9555_settings.h"

// Si no anda añadir Wire.begin()

void setup() {
  tca9555_pinMode(6, OUTPUT);
}

void loop() {
  tca9555_digitalWrite(6, HIGH);
  delay(1000);
  tca9555_digitalWrite(6, LOW);
  delay(1000);
}

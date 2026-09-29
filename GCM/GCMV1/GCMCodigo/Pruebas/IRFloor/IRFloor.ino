#include "tca9555_settings.h"

void setup() {
  Serial.begin(115200);

  tca9555_pinMode(0, OUTPUT); 
  tca9555_digitalWrite(0, HIGH);
}

void serial() {
  Serial.println(analogRead(14));
  delay(1000);
}
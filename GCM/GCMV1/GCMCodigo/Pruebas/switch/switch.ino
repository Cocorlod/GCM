#include "tca9555_settings.h"

bool state = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  tca9555_pinMode(7, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  bool lastState = tca9555_digitalRead(7);

  if(!lastState && lastState != state) {
    Serial.println("Switch pressed");
    state = lastState;
  }

  if(lastState && lastState != state) {
    state = lastState;
  }

  delay(50);
}

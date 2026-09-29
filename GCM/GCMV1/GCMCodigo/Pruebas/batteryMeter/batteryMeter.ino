void setup() {
  Serial.begin(115200);
}

void loop() {
  float inputVoltage = (analogRead(7) * 3.3) / 4095.0 * (147.0 / 47.0);

  Serial.println(inputVoltage);
  delay(1000);
}
#define LEFT_ENC_A   14
#define LEFT_ENC_B   13

#define RIGHT_ENC_A   6
#define RIGHT_ENC_B  21

void setup()
{
    Serial.begin(115200);

    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);

    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
}

void loop()
{
    Serial.print("L: ");
    Serial.print(digitalRead(LEFT_ENC_A));
    Serial.print(" ");
    Serial.print(digitalRead(LEFT_ENC_B));

    Serial.print("    R: ");
    Serial.println(digitalRead(RIGHT_ENC_A));

    delay(20);
}
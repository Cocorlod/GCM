#define LEFT_ENC_A 14
#define LEFT_ENC_B 13

#define RIGHT_ENC_A 6
#define RIGHT_ENC_B 21  

#define ENCODER_CPR 12

#define GEAR_RATIO 4.995

#define WHEEL_DIAMETER_MM 19.0

#define LEFT_COUNTS_PER_WHEEL_REV (ENCODER_CPR * GEAR_RATIO)

#define RIGHT_COUNTS_PER_WHEEL_REV ((ENCODER_CPR / 2.0) * GEAR_RATIO)

#define WHEEL_CIRCUMFERENCE_MM (PI * WHEEL_DIAMETER_MM)

#define LEFT_MM_PER_COUNT (WHEEL_CIRCUMFERENCE_MM / LEFT_COUNTS_PER_WHEEL_REV)

#define RIGHT_MM_PER_COUNT (WHEEL_CIRCUMFERENCE_MM / RIGHT_COUNTS_PER_WHEEL_REV)

volatile long leftCount = 0;
volatile long rightCount = 0;

volatile uint8_t leftLastState;

void IRAM_ATTR leftEncoderISR() {
    uint8_t A = digitalRead(LEFT_ENC_A);
    uint8_t B = digitalRead(LEFT_ENC_B);

    uint8_t state = (A << 1) | B;

    uint8_t transition = (leftLastState << 2) | state;

    switch (transition) {
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            leftCount++;
            break;

        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:
            leftCount--;
            break;
    }

    leftLastState = state;
}


// --------------------------------------------------
// RIGHT ENCODER
// A ONLY
// --------------------------------------------------

void IRAM_ATTR rightEncoderISR()
{
    rightCount++;
}


// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup()
{
    Serial.begin(115200);

    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);

    pinMode(RIGHT_ENC_A, INPUT_PULLUP);


    leftLastState = (digitalRead(LEFT_ENC_A) << 1) | digitalRead(LEFT_ENC_B);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),
        leftEncoderISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENC_B),
        leftEncoderISR,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENC_A),
        rightEncoderISR,
        CHANGE
    );

    Serial.println();
    Serial.println("=================================");
    Serial.println("       GCM ENCODER TEST");
    Serial.println("=================================");

    Serial.println();

    Serial.print("Motor encoder CPR: ");
    Serial.println(ENCODER_CPR);

    Serial.print("Gear ratio: ");
    Serial.println(GEAR_RATIO, 3);

    Serial.println();

    Serial.print("Left counts / wheel revolution: ");
    Serial.println(LEFT_COUNTS_PER_WHEEL_REV, 2);

    Serial.print("Left mm / count: ");
    Serial.print(LEFT_MM_PER_COUNT, 4);
    Serial.println(" mm");

    Serial.println();

    Serial.print("Right counts / wheel revolution: ");
    Serial.println(RIGHT_COUNTS_PER_WHEEL_REV, 2);

    Serial.print("Right mm / count: ");
    Serial.print(RIGHT_MM_PER_COUNT, 4);
    Serial.println(" mm");

    Serial.println();

    Serial.println("Rotate the wheels by hand...");
    Serial.println();
}

void loop()
{
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 200)
    {
        lastPrint = millis();

        noInterrupts();

        long L = leftCount;
        long R = rightCount;

        interrupts();

        double leftDistance =
            L * LEFT_MM_PER_COUNT;

        double rightDistance =
            R * RIGHT_MM_PER_COUNT;

        double leftRevolutions =
            L / LEFT_COUNTS_PER_WHEEL_REV;

        double rightRevolutions =
            R / RIGHT_COUNTS_PER_WHEEL_REV;


        Serial.print("L: ");
        Serial.print(L);

        Serial.print("  ");
        Serial.print(leftDistance, 2);
        Serial.print(" mm");

        Serial.print("    |    ");

        Serial.print("R: ");
        Serial.print(R);

        Serial.print("  ");
        Serial.print(rightDistance, 2);
        Serial.print(" mm");

        Serial.print("    |    ");

        Serial.print("Delta L-R: ");
        Serial.print(leftDistance - rightDistance, 2);

        Serial.println(" mm");
    }
}
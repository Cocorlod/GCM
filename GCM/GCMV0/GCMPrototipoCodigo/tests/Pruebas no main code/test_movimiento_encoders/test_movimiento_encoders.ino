// ============================================================
// GCM STRAIGHT MOVEMENT + ENCODER TEST
//
// LEFT ENCODER:
//   A + B
//   CHANGE on both channels
//   Full quadrature decoding
//
// RIGHT ENCODER:
//   A only
//   CHANGE
//
// CONTROL:
//   Closed-loop wheel speed matching
//   PD controller
//
// TEST SEQUENCE:
//   1. Move forward for 3 seconds
//   2. Stop
//   3. Pivot left 90 degrees
//   4. Stop
//   5. Pivot 180 degrees
//   6. Stop forever
//
// NO TOF
// NO IMU
// NO EXTERNAL LIBRARIES
// ============================================================


// ============================================================
// MOTOR PINS
// ============================================================

#define PIN_STBY  16

#define PIN_BIN1  18
#define PIN_BIN2  17

#define PIN_AIN1  4
#define PIN_AIN2  8

#define PIN_PWMA  9
#define PIN_PWMB  10


// ============================================================
// ENCODER PINS
// ============================================================

#define LEFT_ENC_A   14
#define LEFT_ENC_B   13

#define RIGHT_ENC_A   6

// Right B is intentionally NOT used.
// #define RIGHT_ENC_B 21


// ============================================================
// PWM
// ============================================================

#define PWM_FREQ        20000
#define PWM_RESOLUTION  8

#define FORWARD_PWM     100

#define TURN_PWM        127
#define TURN_PWM_SLOW    90


// ============================================================
// ROBOT GEOMETRY
// ============================================================

#define WHEEL_DIAMETER_MM  19.0f

#define WHEEL_TRACK_MM     109.5f

#define WHEEL_CIRCUMFERENCE_MM \
    (PI * WHEEL_DIAMETER_MM)


// ============================================================
// ENCODER PARAMETERS
// ============================================================
//
// Pololu #5101:
//
// 12 CPR is already the count obtained when counting
// both edges of both encoder channels.
//
// Therefore:
//
// LEFT:
//     12 * gearbox ratio
//
// RIGHT:
//     12 / 2 * gearbox ratio
//
// because the right encoder uses only channel A.
//
// ============================================================

#define ENCODER_CPR  12.0f

#define GEAR_RATIO   4.995f


// Left = full quadrature

#define LEFT_COUNTS_PER_WHEEL_REV \
    (ENCODER_CPR * GEAR_RATIO)


// Right = A only

#define RIGHT_COUNTS_PER_WHEEL_REV \
    ((ENCODER_CPR / 2.0f) * GEAR_RATIO)


// ============================================================
// DISTANCE RESOLUTION
// ============================================================

#define LEFT_MM_PER_COUNT \
    (WHEEL_CIRCUMFERENCE_MM / LEFT_COUNTS_PER_WHEEL_REV)

#define RIGHT_MM_PER_COUNT \
    (WHEEL_CIRCUMFERENCE_MM / RIGHT_COUNTS_PER_WHEEL_REV)


// ============================================================
// SPEED CONTROLLER
// ============================================================
//
// Positive error:
//
//     left faster than right
//
// Therefore:
//
//     left PWM decreases
//     right PWM increases
//
// ============================================================

#define KP_SPEED  1.0f
#define KD_SPEED  0.03f


// Controller update period

#define SPEED_LOOP_PERIOD_MS  20


// Maximum PWM correction

#define MAX_SPEED_CORRECTION  40.0f


// ============================================================
// OPTIONAL PWM RAMP
// ============================================================
//
// Prevents huge instantaneous PWM changes.
//
// This is useful because the derivative controller can react
// strongly when a wheel encoder suddenly produces a count.
//
// ============================================================

#define MAX_PWM_CHANGE  5


// ============================================================
// ENCODER VARIABLES
// ============================================================

volatile long leftCount  = 0;
volatile long rightCount = 0;


// Previous state of left A/B

volatile uint8_t leftLastState = 0;


// ============================================================
// LEFT ENCODER ISR
//
// TRUE QUADRATURE
//
// Both A and B trigger the interrupt.
// Every valid transition produces one count.
//
// ============================================================

void IRAM_ATTR leftEncoderISR()
{
    uint8_t A = digitalRead(LEFT_ENC_A);
    uint8_t B = digitalRead(LEFT_ENC_B);

    uint8_t state =
        (A << 1) | B;


    uint8_t transition =
        (leftLastState << 2) | state;


    switch (transition)
    {
        // ----------------------------------------------------
        // Forward
        // ----------------------------------------------------

        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:

            leftCount++;

            break;


        // ----------------------------------------------------
        // Reverse
        // ----------------------------------------------------

        case 0b0010:
        case 0b1011:
        case 0b1101:
        case 0b0100:

            leftCount--;

            break;


        // ----------------------------------------------------
        // Invalid / no movement
        // ----------------------------------------------------

        default:

            break;
    }


    leftLastState = state;
}


// ============================================================
// RIGHT ENCODER ISR
//
// A ONLY
//
// CHANGE means:
//
//     rising edge -> count
//     falling edge -> count
//
// Direction is NOT obtained from the encoder.
//
// Since we already know that the motor is commanded forward,
// the magnitude of the count rate is sufficient for speed
// synchronization.
// ============================================================

void IRAM_ATTR rightEncoderISR()
{
    rightCount++;
}


// ============================================================
// SPEED CONTROLLER VARIABLES
// ============================================================

long previousLeftCount  = 0;
long previousRightCount = 0;

float previousSpeedError = 0.0f;

bool speedControllerInitialized = false;

uint32_t previousSpeedTime = 0;


// ============================================================
// PWM VARIABLES
// ============================================================

int currentLeftPWM  = 0;
int currentRightPWM = 0;


// ============================================================
// RESET SPEED CONTROLLER
// ============================================================

void resetSpeedController()
{
    noInterrupts();

    previousLeftCount  = leftCount;
    previousRightCount = rightCount;

    interrupts();


    previousSpeedError = 0.0f;

    previousSpeedTime = millis();

    speedControllerInitialized = false;
}


// ============================================================
// RESET PWM
// ============================================================

void resetPWM()
{
    currentLeftPWM  = 0;
    currentRightPWM = 0;
}


// ============================================================
// LIMIT PWM CHANGE
// ============================================================

int rampPWM(int currentPWM, int targetPWM)
{
    if (targetPWM > currentPWM)
    {
        currentPWM += MAX_PWM_CHANGE;

        if (currentPWM > targetPWM)
            currentPWM = targetPWM;
    }
    else if (targetPWM < currentPWM)
    {
        currentPWM -= MAX_PWM_CHANGE;

        if (currentPWM < targetPWM)
            currentPWM = targetPWM;
    }


    return currentPWM;
}


// ============================================================
// CALCULATE SPEED CORRECTION
// ============================================================
//
// Returns:
//
//     positive:
//         left wheel is faster
//
//     negative:
//         right wheel is faster
//
// ============================================================

float calculateSpeedCorrection()
{
    static uint32_t lastLoopTime = 0;


    uint32_t now = millis();


    // --------------------------------------------------------
    // Fixed controller frequency
    // --------------------------------------------------------

    if (now - lastLoopTime < SPEED_LOOP_PERIOD_MS)
    {
        return 0.0f;
    }


    lastLoopTime = now;


    // --------------------------------------------------------
    // Read encoders atomically
    // --------------------------------------------------------

    long left;
    long right;


    noInterrupts();

    left  = leftCount;
    right = rightCount;

    interrupts();


    // --------------------------------------------------------
    // First iteration
    // --------------------------------------------------------

    if (!speedControllerInitialized)
    {
        previousLeftCount  = left;
        previousRightCount = right;

        previousSpeedError = 0.0f;

        speedControllerInitialized = true;

        return 0.0f;
    }


    // --------------------------------------------------------
    // Time interval
    // --------------------------------------------------------

    float dt =
        SPEED_LOOP_PERIOD_MS / 1000.0f;


    // --------------------------------------------------------
    // Encoder displacement
    // --------------------------------------------------------

    long deltaLeft =
        left - previousLeftCount;


    long deltaRight =
        right - previousRightCount;


    // --------------------------------------------------------
    // Wheel speeds
    //
    // We use absolute values because the right encoder
    // doesn't contain direction information.
    //
    // Both motors are commanded forward here.
    // --------------------------------------------------------

    float leftSpeed =
        fabs(
            deltaLeft *
            LEFT_MM_PER_COUNT /
            dt
        );


    float rightSpeed =
        fabs(
            deltaRight *
            RIGHT_MM_PER_COUNT /
            dt
        );


    // --------------------------------------------------------
    // Speed error
    //
    // Positive:
    //     left faster
    //
    // Negative:
    //     right faster
    // --------------------------------------------------------

    float error =
        leftSpeed - rightSpeed;


    // --------------------------------------------------------
    // Derivative
    // --------------------------------------------------------

    float derivative =
        (error - previousSpeedError) /
        dt;


    // --------------------------------------------------------
    // PD controller
    // --------------------------------------------------------

    float correction =
        KP_SPEED * error +
        KD_SPEED * derivative;


    // --------------------------------------------------------
    // Limit correction
    // --------------------------------------------------------

    correction =
        constrain(
            correction,
            -MAX_SPEED_CORRECTION,
             MAX_SPEED_CORRECTION
        );


    // --------------------------------------------------------
    // Save state
    // --------------------------------------------------------

    previousLeftCount  = left;
    previousRightCount = right;

    previousSpeedError = error;


    return correction;
}


// ============================================================
// SET FORWARD MOTOR DIRECTION
// ============================================================

void setMotorsForward()
{
    digitalWrite(PIN_STBY, HIGH);


    // --------------------------------------------------------
    // Left motor
    // --------------------------------------------------------

    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);


    // --------------------------------------------------------
    // Right motor
    // --------------------------------------------------------

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors()
{
    digitalWrite(PIN_STBY, LOW);


    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);


    currentLeftPWM  = 0;
    currentRightPWM = 0;
}


// ============================================================
// MOVE FORWARD
//
// Closed-loop wheel speed synchronization.
//
// Base:
//
//     left  = FORWARD_PWM
//     right = FORWARD_PWM
//
// Correction:
//
//     leftPWM  = base - correction
//     rightPWM = base + correction
//
// ============================================================

void moveForward()
{
    setMotorsForward();


    float correction =
        calculateSpeedCorrection();


    // --------------------------------------------------------
    // Calculate target PWM
    // --------------------------------------------------------

    int targetLeftPWM =
        constrain(
            (int)(FORWARD_PWM - correction),
            0,
            255
        );


    int targetRightPWM =
        constrain(
            (int)(FORWARD_PWM + correction),
            0,
            255
        );


    // --------------------------------------------------------
    // Smooth PWM changes
    // --------------------------------------------------------

    currentLeftPWM =
        rampPWM(
            currentLeftPWM,
            targetLeftPWM
        );


    currentRightPWM =
        rampPWM(
            currentRightPWM,
            targetRightPWM
        );


    // --------------------------------------------------------
    // Apply PWM
    // --------------------------------------------------------

    ledcWrite(
        PIN_PWMA,
        currentLeftPWM
    );


    ledcWrite(
        PIN_PWMB,
        currentRightPWM
    );
}


// ============================================================
// TURN LEFT
//
// Pivot around the LEFT wheel.
//
// Therefore:
//
//     LEFT  = stopped
//     RIGHT = powered
//
// For a pivot:
//
//     arc length = angle * wheel track
//
// ============================================================

void turnLeft(float angleDeg)
{
    long startCount;


    // --------------------------------------------------------
    // Get initial right encoder count
    // --------------------------------------------------------

    noInterrupts();

    startCount = rightCount;

    interrupts();


    // --------------------------------------------------------
    // Calculate wheel travel
    // --------------------------------------------------------

    float angleRad =
        angleDeg * PI / 180.0f;


    float wheelTravel =
        angleRad * WHEEL_TRACK_MM;


    // --------------------------------------------------------
    // Calculate required encoder counts
    // --------------------------------------------------------

    long targetCounts =
        (long)(
            wheelTravel /
            RIGHT_MM_PER_COUNT
        );


    Serial.println();
    Serial.print("Turning LEFT ");
    Serial.print(angleDeg);
    Serial.println(" degrees");


    Serial.print("Target counts: ");
    Serial.println(targetCounts);


    // --------------------------------------------------------
    // Motor directions
    // --------------------------------------------------------

    digitalWrite(PIN_STBY, HIGH);


    // Left stopped

    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);


    // Right forward

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);


    // --------------------------------------------------------
    // Turn
    // --------------------------------------------------------

    uint32_t startTime = millis();


    while (true)
    {
        long currentCount;


        noInterrupts();

        currentCount = rightCount;

        interrupts();


        long counts =
            labs(
                currentCount -
                startCount
            );


        // ----------------------------------------------------
        // Target reached
        // ----------------------------------------------------

        if (counts >= targetCounts)
        {
            break;
        }


        // ----------------------------------------------------
        // Remaining distance
        // ----------------------------------------------------

        long remaining =
            targetCounts - counts;


        // ----------------------------------------------------
        // Slow down near target
        // ----------------------------------------------------

        int pwm = TURN_PWM;


        if (remaining < targetCounts / 4)
        {
            pwm = TURN_PWM_SLOW;
        }


        // ----------------------------------------------------
        // Apply motor PWM
        // ----------------------------------------------------

        ledcWrite(
            PIN_PWMA,
            0
        );


        ledcWrite(
            PIN_PWMB,
            pwm
        );


        // ----------------------------------------------------
        // Safety timeout
        // ----------------------------------------------------

        if (millis() - startTime > 3000)
        {
            Serial.println("TURN TIMEOUT");

            break;
        }
    }


    // --------------------------------------------------------
    // Stop
    // --------------------------------------------------------

    stopMotors();


    Serial.println("Turn finished.");

    delay(500);
}


// ============================================================
// TURN 180 DEGREES
//
// Same pivot mechanism.
//
// LEFT wheel stopped.
// RIGHT wheel powered.
//
// ============================================================

void turn180() {
    long startCount;


    noInterrupts();

    startCount = rightCount;

    interrupts();


    float angleDeg = 180.0f;


    // --------------------------------------------------------
    // Calculate arc length
    // --------------------------------------------------------

    float angleRad =
        angleDeg * PI / 180.0f;


    float wheelTravel =
        angleRad * WHEEL_TRACK_MM;


    // --------------------------------------------------------
    // Calculate target counts
    // --------------------------------------------------------

    long targetCounts =
        (long)(
            wheelTravel /
            RIGHT_MM_PER_COUNT
        );


    Serial.println();
    Serial.println("Turning 180 degrees");


    Serial.print("Target counts: ");
    Serial.println(targetCounts);


    // --------------------------------------------------------
    // Motor direction
    // --------------------------------------------------------

    digitalWrite(PIN_STBY, HIGH);


    // Left stopped

    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);


    // Right forward

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);


    // --------------------------------------------------------
    // Turn
    // --------------------------------------------------------

    uint32_t startTime = millis();


    while (true)
    {
        long currentCount;


        noInterrupts();

        currentCount = rightCount;

        interrupts();


        long counts =
            labs(
                currentCount -
                startCount
            );


        // ----------------------------------------------------
        // Target reached
        // ----------------------------------------------------

        if (counts >= targetCounts)
        {
            break;
        }


        // ----------------------------------------------------
        // Remaining counts
        // ----------------------------------------------------

        long remaining =
            targetCounts - counts;


        // ----------------------------------------------------
        // Slow down near target
        // ----------------------------------------------------

        int pwm = TURN_PWM;


        if (remaining < targetCounts / 4)
        {
            pwm = TURN_PWM_SLOW;
        }


        // ----------------------------------------------------
        // Apply PWM
        // ----------------------------------------------------

        ledcWrite(
            PIN_PWMA,
            0
        );


        ledcWrite(
            PIN_PWMB,
            pwm
        );


        // ----------------------------------------------------
        // Safety timeout
        // ----------------------------------------------------

        if (millis() - startTime > 5000)
        {
            Serial.println("180 TURN TIMEOUT");

            break;
        }
    }


    stopMotors();


    Serial.println("180 turn finished.");

    delay(500);
}


// ============================================================
// PRINT ENCODER INFORMATION
// ============================================================

void printEncoderInfo()
{
    static uint32_t lastPrint = 0;


    if (millis() - lastPrint < 200)
        return;


    lastPrint = millis();


    long L;
    long R;


    noInterrupts();

    L = leftCount;
    R = rightCount;

    interrupts();


    // --------------------------------------------------------
    // Convert counts to distance
    // --------------------------------------------------------

    float leftDistance =
        L * LEFT_MM_PER_COUNT;


    float rightDistance =
        R * RIGHT_MM_PER_COUNT;


    // --------------------------------------------------------
    // Print
    // --------------------------------------------------------

    Serial.print("L: ");
    Serial.print(L);

    Serial.print("  ");

    Serial.print(leftDistance, 1);

    Serial.print(" mm");


    Serial.print("    |    R: ");

    Serial.print(R);

    Serial.print("  ");

    Serial.print(rightDistance, 1);

    Serial.print(" mm");


    Serial.print("    |    PWM: ");

    Serial.print(currentLeftPWM);

    Serial.print("/");

    Serial.println(currentRightPWM);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);


    // ========================================================
    // MOTOR PINS
    // ========================================================

    pinMode(PIN_STBY, OUTPUT);

    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);

    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);


    // ========================================================
    // PWM
    // ========================================================

    ledcAttach(
        PIN_PWMA,
        PWM_FREQ,
        PWM_RESOLUTION
    );


    ledcAttach(
        PIN_PWMB,
        PWM_FREQ,
        PWM_RESOLUTION
    );


    // ========================================================
    // ENCODER PINS
    // ========================================================

    pinMode(
        LEFT_ENC_A,
        INPUT_PULLUP
    );


    pinMode(
        LEFT_ENC_B,
        INPUT_PULLUP
    );


    pinMode(
        RIGHT_ENC_A,
        INPUT_PULLUP
    );


    // ========================================================
    // INITIAL LEFT ENCODER STATE
    // ========================================================

    leftLastState =
        (digitalRead(LEFT_ENC_A) << 1) |
        digitalRead(LEFT_ENC_B);


    // ========================================================
    // LEFT ENCODER
    //
    // A + B
    // ========================================================

    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENC_A),
        leftEncoderISR,
        CHANGE
    );


    attachInterrupt(
        digitalPinToInterrupt(LEFT_ENC_B),
        leftEncoderISR,
        CHANGE
    );


    // ========================================================
    // RIGHT ENCODER
    //
    // A ONLY
    // ========================================================

    attachInterrupt(
        digitalPinToInterrupt(RIGHT_ENC_A),
        rightEncoderISR,
        CHANGE
    );


    // ========================================================
    // STOP MOTORS
    // ========================================================

    stopMotors();


    // ========================================================
    // PRINT PARAMETERS
    // ========================================================

    Serial.println();
    Serial.println("========================================");
    Serial.println("       GCM STRAIGHT MOVEMENT TEST");
    Serial.println("========================================");


    Serial.println();


    Serial.print("Wheel diameter: ");
    Serial.print(WHEEL_DIAMETER_MM);
    Serial.println(" mm");


    Serial.print("Wheel track: ");
    Serial.print(WHEEL_TRACK_MM);
    Serial.println(" mm");


    Serial.println();


    Serial.print("Left counts / wheel rev: ");
    Serial.println(
        LEFT_COUNTS_PER_WHEEL_REV,
        2
    );


    Serial.print("Left mm / count: ");
    Serial.println(
        LEFT_MM_PER_COUNT,
        4
    );


    Serial.println();


    Serial.print("Right counts / wheel rev: ");
    Serial.println(
        RIGHT_COUNTS_PER_WHEEL_REV,
        2
    );


    Serial.print("Right mm / count: ");
    Serial.println(
        RIGHT_MM_PER_COUNT,
        4
    );


    Serial.println();


    Serial.print("Speed KP: ");
    Serial.println(KP_SPEED);


    Serial.print("Speed KD: ");
    Serial.println(KD_SPEED);


    Serial.println();


    Serial.println("Robot starting in 2 seconds...");


    delay(2000);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
    // ========================================================
    // 1. FORWARD FOR 3 SECONDS
    // ========================================================

    Serial.println();
    Serial.println("========================================");
    Serial.println("       FORWARD - 3 SECONDS");
    Serial.println("========================================");


    // --------------------------------------------------------
    // Reset encoders
    // --------------------------------------------------------

    noInterrupts();

    leftCount  = 0;
    rightCount = 0;

    interrupts();


    // --------------------------------------------------------
    // Reset controller
    // --------------------------------------------------------

    resetSpeedController();

    resetPWM();


    // --------------------------------------------------------
    // Forward
    // --------------------------------------------------------

    uint32_t forwardStart =
        millis();


    while (
        millis() - forwardStart < 3000
    )
    {
        moveForward();

        printEncoderInfo();
    }


    // ========================================================
    // STOP
    // ========================================================

    stopMotors();


    Serial.println();
    Serial.println("Forward finished.");


    // --------------------------------------------------------
    // Print final distance
    // --------------------------------------------------------

    long L;
    long R;


    noInterrupts();

    L = leftCount;
    R = rightCount;

    interrupts();


    Serial.print("Final left count: ");
    Serial.println(L);


    Serial.print("Final right count: ");
    Serial.println(R);


    Serial.print("Left distance: ");
    Serial.print(
        L * LEFT_MM_PER_COUNT,
        2
    );

    Serial.println(" mm");


    Serial.print("Right distance: ");
    Serial.print(
        R * RIGHT_MM_PER_COUNT,
        2
    );

    Serial.println(" mm");


    delay(500);


    // ========================================================
    // 2. LEFT 90°
    // ========================================================

    turnLeft(90.0f);


    // ========================================================
    // 3. 180°
    // ========================================================

    turn180();


    // ========================================================
    // FINISHED
    // ========================================================

    Serial.println();
    Serial.println("========================================");
    Serial.println("             TEST FINISHED");
    Serial.println("========================================");


    // --------------------------------------------------------
    // Do not repeat
    // --------------------------------------------------------

    stopMotors();


    while (true)
    {
        delay(1000);
    }
}
#include "movement.hpp"


// ============================================================
// WALL FOLLOWING PD
// ============================================================

float previousError = 0.0f;

uint32_t previousTime = 0;

const float KP = 1.2f;

const float KD = 0.08f;


// Maximum allowed wall alignment error

static constexpr int16_t MAX_PLAUSIBLE_ALIGNMENT_ERROR = 150;


// Track whether wall following was active

static bool wasWallFollowing = false;


// ============================================================
// ENCODERS
// ============================================================

Encoder leftEncoder;

Encoder rightEncoder;


// ============================================================
// LEFT ENCODER
//
// A+B FULL QUADRATURE
// ============================================================

void Encoder::begin(uint8_t pinA, uint8_t pinB)
{
    _pinA = pinA;

    _pinB = pinB;

    _aOnly = false;


    pinMode(
        _pinA,
        INPUT_PULLUP
    );


    pinMode(
        _pinB,
        INPUT_PULLUP
    );


    // Initial quadrature state

    _lastState =
        (digitalRead(_pinA) << 1) |
        digitalRead(_pinB);


    _count = 0;


    // Interrupt on A

    attachInterruptArg(
        digitalPinToInterrupt(_pinA),
        isrHandler,
        this,
        CHANGE
    );


    // Interrupt on B

    attachInterruptArg(
        digitalPinToInterrupt(_pinB),
        isrHandler,
        this,
        CHANGE
    );
}


// ============================================================
// A-ONLY ENCODER
//
// Used by RIGHT encoder.
//
// CHANGE = rising + falling edges.
// Direction is intentionally not decoded.
// ============================================================

void Encoder::beginAOnly(uint8_t pinA)
{
    _pinA = pinA;

    _pinB = 0;

    _aOnly = true;


    pinMode(
        _pinA,
        INPUT_PULLUP
    );


    _count = 0;


    attachInterruptArg(
        digitalPinToInterrupt(_pinA),
        isrHandler,
        this,
        CHANGE
    );
}


// ============================================================
// GET COUNT
// ============================================================

long Encoder::getCount() const
{
    noInterrupts();

    long c = _count;

    interrupts();

    return c;
}


// ============================================================
// RESET
// ============================================================

void Encoder::reset()
{
    noInterrupts();

    _count = 0;

    interrupts();
}


// ============================================================
// ISR DISPATCHER
// ============================================================

void IRAM_ATTR Encoder::isrHandler(void* arg)
{
    Encoder* encoder =
        static_cast<Encoder*>(arg);


    if (encoder->_aOnly)
    {
        encoder->handleAOnlyInterrupt();
    }
    else
    {
        encoder->handleInterrupt();
    }
}


// ============================================================
// QUADRATURE INTERRUPT
//
// Full A+B decoding.
// ============================================================

void IRAM_ATTR Encoder::handleInterrupt()
{
    static const int8_t table[16] =
    {
         0, -1,  1,  0,
         1,  0,  0, -1,
        -1,  0,  0,  1,
         0,  1, -1,  0
    };


    uint8_t a =
        digitalRead(_pinA);


    uint8_t b =
        digitalRead(_pinB);


    uint8_t state =
        (a << 1) | b;


    uint8_t idx =
        (uint8_t)(
            (_lastState << 2) |
            state
        ) & 0x0F;


    _count += table[idx];


    _lastState = state;
}


// ============================================================
// A-ONLY INTERRUPT
//
// Every A transition increments the count.
//
// Direction is not determined here.
// This is fine because the speed controller only needs
// wheel-speed magnitude during forward movement.
// ============================================================

void IRAM_ATTR Encoder::handleAOnlyInterrupt()
{
    _count++;
}


// ============================================================
// SPEED SYNCHRONIZATION
// ============================================================
//
// The two encoders have different resolutions:
//
// LEFT:
//     0.9958 mm/count
//
// RIGHT:
//     1.9915 mm/count
//
// Therefore raw encoder counts cannot be compared directly.
//
// We convert both to mm/s first.
// ============================================================

static constexpr float KP_SPEED = 1.0f;

static constexpr float KD_SPEED = 0.03f;


// Controller update period

static constexpr uint32_t SPEED_LOOP_PERIOD_MS = 20;


// Maximum speed correction

static constexpr float MAX_SPEED_CORRECTION = 40.0f;


static long lastLeftCountSpeed = 0;

static long lastRightCountSpeed = 0;

static uint32_t lastSpeedTime = 0;

static float prevSpeedError = 0.0f;

static bool speedLoopInitialized = false;


// ============================================================
// RESET SPEED SYNCHRONIZATION
// ============================================================

static void resetSpeedSyncLoop()
{
    speedLoopInitialized = false;

    prevSpeedError = 0.0f;

    lastSpeedTime = millis();


    // Synchronize starting encoder values

    lastLeftCountSpeed =
        leftEncoder.getCount();


    lastRightCountSpeed =
        rightEncoder.getCount();
}


// ============================================================
// SPEED SYNCHRONIZATION CORRECTION
// ============================================================

static float computeSpeedSyncCorrection()
{
    static uint32_t lastLoopTime = 0;


    uint32_t now = millis();


    // --------------------------------------------------------
    // Run at fixed 20 ms interval
    // --------------------------------------------------------

    if (
        now - lastLoopTime <
        SPEED_LOOP_PERIOD_MS
    )
    {
        return 0.0f;
    }


    lastLoopTime = now;


    // --------------------------------------------------------
    // Read encoder counts
    // --------------------------------------------------------

    long leftCount =
        leftEncoder.getCount();


    long rightCount =
        rightEncoder.getCount();


    // --------------------------------------------------------
    // Initialize
    // --------------------------------------------------------

    if (!speedLoopInitialized)
    {
        lastLeftCountSpeed =
            leftCount;


        lastRightCountSpeed =
            rightCount;


        lastSpeedTime =
            now;


        prevSpeedError =
            0.0f;


        speedLoopInitialized =
            true;


        return 0.0f;
    }


    // --------------------------------------------------------
    // Fixed time step
    // --------------------------------------------------------

    float dt =
        SPEED_LOOP_PERIOD_MS *
        0.001f;


    // --------------------------------------------------------
    // Encoder displacement
    // --------------------------------------------------------

    long deltaLeft =
        leftCount -
        lastLeftCountSpeed;


    long deltaRight =
        rightCount -
        lastRightCountSpeed;


    // --------------------------------------------------------
    // Convert to physical wheel velocity
    //
    // Absolute value is used because the right encoder
    // doesn't provide direction.
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
        leftSpeed -
        rightSpeed;


    // --------------------------------------------------------
    // Derivative
    // --------------------------------------------------------

    float derivative =
        (
            error -
            prevSpeedError
        ) / dt;


    // --------------------------------------------------------
    // PD
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

    lastLeftCountSpeed =
        leftCount;


    lastRightCountSpeed =
        rightCount;


    lastSpeedTime =
        now;


    prevSpeedError =
        error;


    return correction;
}


// ============================================================
// FORWARD MOVEMENT
//
// TOF WALL FOLLOWING IS PRESERVED.
//
// Total correction:
//
//     wall correction
//          +
//     encoder speed correction
//
// ============================================================

void moveForward(ToFSensor& tof)
{
    bool leftWall =
        tof.isThereWall(WALL_LEFT);


    bool rightWall =
        tof.isThereWall(WALL_RIGHT);


    float wallCorrection = 0.0f;


    // ========================================================
    // WALL FOLLOWING
    // ========================================================

    if (
        leftWall ||
        rightWall
    )
    {
        float error;


        if (
            leftWall &&
            rightWall
        )
        {
            error =
                (
                    tof.alignmentError(WALL_LEFT) -
                    tof.alignmentError(WALL_RIGHT)
                ) * 0.5f;
        }
        else if (leftWall)
        {
            error =
                tof.alignmentError(WALL_LEFT);
        }
        else
        {
            error =
                -tof.alignmentError(WALL_RIGHT);
        }


        error =
            constrain(
                error,
                -MAX_PLAUSIBLE_ALIGNMENT_ERROR,
                 MAX_PLAUSIBLE_ALIGNMENT_ERROR
            );


        uint32_t now =
            micros();


        if (!wasWallFollowing)
        {
            previousError =
                error;

            previousTime =
                now;

            wasWallFollowing =
                true;
        }


        float dt =
            max(
                (now - previousTime) *
                1e-6f,
                0.001f
            );


        float derivative =
            (
                error -
                previousError
            ) / dt;


        wallCorrection =
            KP * error +
            KD * derivative;


        previousError =
            error;


        previousTime =
            now;
    }
    else
    {
        wasWallFollowing =
            false;
    }


    // ========================================================
    // ENCODER SPEED SYNCHRONIZATION
    // ========================================================

    float speedCorrection =
        computeSpeedSyncCorrection();


    // ========================================================
    // TOTAL CORRECTION
    // ========================================================

    float totalCorrection =
        wallCorrection +
        speedCorrection;


    // ========================================================
    // PWM
    // ========================================================

    int leftPWM =
        constrain(
            (int)(
                FORWARD_PWM -
                totalCorrection
            ),
            0,
            255
        );


    int rightPWM =
        constrain(
            (int)(
                FORWARD_PWM +
                totalCorrection
            ),
            0,
            255
        );


    // ========================================================
    // MOTOR DIRECTION
    // ========================================================

    digitalWrite(
        PIN_STBY,
        HIGH
    );


    digitalWrite(
        PIN_BIN1,
        HIGH
    );


    digitalWrite(
        PIN_BIN2,
        LOW
    );


    digitalWrite(
        PIN_AIN1,
        HIGH
    );


    digitalWrite(
        PIN_AIN2,
        LOW
    );


    // ========================================================
    // APPLY PWM
    // ========================================================

    ledcWrite(
        PIN_PWMA,
        leftPWM
    );


    ledcWrite(
        PIN_PWMB,
        rightPWM
    );
}


// ============================================================
// TURN ARC LENGTH
// ============================================================

static float arcLengthForAngleDeg(
    float angleDeg
)
{
    float angleRad =
        angleDeg *
        PI /
        180.0f;


    return angleRad *
           WHEEL_TRACK_MM;
}


// ============================================================
// TURN DIRECTION
// ============================================================

static void setTurnDirection(
    bool clockwise
)
{
    if (clockwise)
    {
        // ----------------------------------------------------
        // Left wheel powered
        // Right wheel stopped
        // ----------------------------------------------------

        digitalWrite(
            PIN_AIN1,
            HIGH
        );


        digitalWrite(
            PIN_AIN2,
            LOW
        );


        digitalWrite(
            PIN_BIN1,
            LOW
        );


        digitalWrite(
            PIN_BIN2,
            LOW
        );
    }
    else
    {
        // ----------------------------------------------------
        // Right wheel powered
        // Left wheel stopped
        // ----------------------------------------------------

        digitalWrite(
            PIN_AIN1,
            LOW
        );


        digitalWrite(
            PIN_AIN2,
            LOW
        );


        digitalWrite(
            PIN_BIN1,
            HIGH
        );


        digitalWrite(
            PIN_BIN2,
            LOW
        );
    }
}


// ============================================================
// PERFORM TURN
// ============================================================

static void performTurn(
    float angleDeg,
    bool clockwise
)
{
    long startLeft =
        leftEncoder.getCount();


    long startRight =
        rightEncoder.getCount();


    float arcMM =
        arcLengthForAngleDeg(
            angleDeg
        );


    // --------------------------------------------------------
    // IMPORTANT:
    //
    // Clockwise:
    //     left wheel moves
    //     use LEFT_MM_PER_COUNT
    //
    // Counter-clockwise:
    //     right wheel moves
    //     use RIGHT_MM_PER_COUNT
    // --------------------------------------------------------

    float mmPerCount;


    if (clockwise)
    {
        mmPerCount =
            LEFT_MM_PER_COUNT;
    }
    else
    {
        mmPerCount =
            RIGHT_MM_PER_COUNT;
    }


    long targetCounts =
        (long)(
            arcMM /
            mmPerCount
        );


    digitalWrite(
        PIN_STBY,
        HIGH
    );


    setTurnDirection(
        clockwise
    );


    uint32_t start =
        millis();


    while (
        millis() - start <
        TURN_TIMEOUT_MS
    )
    {
        long counts;


        if (clockwise)
        {
            counts =
                labs(
                    leftEncoder.getCount() -
                    startLeft
                );
        }
        else
        {
            counts =
                labs(
                    rightEncoder.getCount() -
                    startRight
                );
        }


        // ----------------------------------------------------
        // Target reached
        // ----------------------------------------------------

        if (
            counts >=
            targetCounts
        )
        {
            break;
        }


        // ----------------------------------------------------
        // Remaining counts
        // ----------------------------------------------------

        long remaining =
            targetCounts -
            counts;


        // ----------------------------------------------------
        // Slow down during final 25%
        // ----------------------------------------------------

        uint8_t pwm =
            TURN_PWM;


        if (
            remaining <
            targetCounts / 4
        )
        {
            pwm =
                TURN_PWM_SLOW;
        }


        // ----------------------------------------------------
        // Apply PWM
        // ----------------------------------------------------

        ledcWrite(
            PIN_PWMA,
            clockwise ?
            pwm :
            0
        );


        ledcWrite(
            PIN_PWMB,
            clockwise ?
            0 :
            pwm
        );
    }


    // --------------------------------------------------------
    // Stop
    // --------------------------------------------------------

    stopMotors();


    resetSpeedSyncLoop();
}


// ============================================================
// TURN COMMAND
// ============================================================

void turn(
    Turn dir
)
{
    switch (dir)
    {
        case LEFT:

            performTurn(
                90.0f,
                false
            );

            break;


        case RIGHT:

            performTurn(
                90.0f,
                true
            );

            break;


        case BACK:

            performTurn(
                180.0f,
                true
            );

            break;
    }
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors()
{
    digitalWrite(
        PIN_STBY,
        LOW
    );


    ledcWrite(
        PIN_PWMA,
        0
    );


    ledcWrite(
        PIN_PWMB,
        0
    );
}
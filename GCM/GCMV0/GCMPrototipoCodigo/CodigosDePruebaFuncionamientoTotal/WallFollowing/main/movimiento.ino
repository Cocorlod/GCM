#include "encoders.h"
#include "movimiento.h"

void setMotors(int left, int right) {
    left = constrain(left, -255, 255);
    right = constrain(right, -255, 255);

    digitalWrite(PIN_STBY, HIGH);

    if (left >= 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
    } else {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        left = -left;
    }

    if (right >= 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
    } else {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        right = -right;
    }

    ledcWrite(PIN_PWMA, left);
    ledcWrite(PIN_PWMB, right);
}

void stopMotors() {
    digitalWrite(PIN_STBY, LOW);
    ledcWrite(PIN_PWMA, 0);
    ledcWrite(PIN_PWMB, 0);
}

void updateSpeedControl() {
    uint32_t now = millis();

    if (now - lastSpeedControlTime < SPEED_LOOP_PERIOD_MS) return;

    float dt = (now - lastSpeedControlTime) * 0.001f;
    lastSpeedControlTime = now;

    long leftCount = leftEncoder.getCount();
    long rightCount = rightEncoder.getCount();

    long deltaLeft = leftCount - lastLeftControlCount;
    long deltaRight = rightCount - lastRightControlCount;

    lastLeftControlCount = leftCount;
    lastRightControlCount = rightCount;

    float measuredLeftSpeed = fabs(deltaLeft * LEFT_MM_PER_COUNT / dt);
    float measuredRightSpeed = fabs(deltaRight * RIGHT_MM_PER_COUNT / dt);

    float leftCorrection = updatePI(leftPI, TARGET_SPEED_MM_S, measuredLeftSpeed, dt);
    float rightCorrection = updatePI(rightPI, TARGET_SPEED_MM_S, measuredRightSpeed, dt);

    leftPWM = constrain((int)(BASE_PWM_LEFT + leftCorrection - tofCorrection), 0, 255);
    rightPWM = constrain((int)(BASE_PWM_RIGHT + rightCorrection + tofCorrection), 0, 255);

    setMotors(leftPWM, rightPWM);
}

void turn90toLeft() {
    digitalWrite(PIN_STBY, HIGH);

    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);

    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);

    while(true) {
        long currentCount;

        noInterrupts();

        currentCount = rightEncoder.getCount();

        interrupts();

        long counts = labs(currentCount - turnStartRightCount); 

        if(counts >= RIGHT_TURN_COUNTS_90) {
            break;
        }

        long remaining = RIGHT_TURN_COUNTS_90 - counts;

        int pwm = TURN_PWM;

        if (remaining < RIGHT_TURN_COUNTS_90 / 4) {
            pwm = 70;
        }

        ledcWrite(PIN_PWMA, 0);
        ledcWrite(PIN_PWMB, pwm);

    }
    stopMotors();

    return;
}

void turn90toRight() {
    digitalWrite(PIN_STBY, HIGH);

    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);

    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);

    while(true) {
        long currentCount;

        noInterrupts();

        currentCount = leftEncoder.getCount();

        interrupts();

        long counts = labs(currentCount - turnStartLeftCount); 

        if(counts >= LEFT_TURN_COUNTS_90) {
            break;
        }

        long remaining = LEFT_TURN_COUNTS_90 - counts;

        int pwm = TURN_PWM;

        if (remaining < LEFT_TURN_COUNTS_90 / 4) {
            pwm = 70;
        }

        ledcWrite(PIN_PWMA, pwm);
        ledcWrite(PIN_PWMB, 0);

    }
    stopMotors();

    return;
}

void turnBack() {
    digitalWrite(PIN_STBY, HIGH);

    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);

    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);

    while(true) {
        long currentCount;

        noInterrupts();

        currentCount = leftEncoder.getCount();

        interrupts();

        long counts = labs(currentCount - turnStartLeftCount); 

        if(counts >= LEFT_TURN_COUNTS_180) {
            break;
        }

        long remaining = LEFT_TURN_COUNTS_180 - counts;

        int pwm = TURN_PWM;

        if (remaining < RIGHT_TURN_COUNTS_180 / 4) {
            pwm = 70;
        }

        ledcWrite(PIN_PWMA, pwm);
        ledcWrite(PIN_PWMB, pwm);

    }
    stopMotors();

    return;
}
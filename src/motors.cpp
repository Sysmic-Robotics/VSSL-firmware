#include "motors.h"
#include "config.h"

void initMotors() {
    pinMode(MOT_IN1_PIN, OUTPUT);
    pinMode(MOT_IN2_PIN, OUTPUT);
    pinMode(MOT_IN3_PIN, OUTPUT);
    pinMode(MOT_IN4_PIN, OUTPUT);

    stopMotors();
}

void driveMotor(int speed, uint8_t pinA, uint8_t pinB) {
    speed = constrain(speed, -MAX_PWM, MAX_PWM);

    int pwm = abs(speed);

    if (speed > 0) {
        analogWrite(pinA, pwm);
        analogWrite(pinB, 0);
    }
    else if (speed < 0) {
        analogWrite(pinA, 0);
        analogWrite(pinB, pwm);
    }
    else {
        analogWrite(pinA, 0);
        analogWrite(pinB, 0);
    }
}

void stopMotors() {
    analogWrite(MOT_IN1_PIN, 0);
    analogWrite(MOT_IN2_PIN, 0);
    analogWrite(MOT_IN3_PIN, 0);
    analogWrite(MOT_IN4_PIN, 0);
}
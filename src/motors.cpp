#include "motors.h"

void initMotors() {
    pinMode(MOT_IN1_PIN, OUTPUT);
    pinMode(MOT_IN2_PIN, OUTPUT);
    pinMode(MOT_IN3_PIN, OUTPUT);
    pinMode(MOT_IN4_PIN, OUTPUT);
    
    analogWriteResolution(PWM_RES);
    analogWriteFrequency(PWM_FREQ);
}

void driveMotor(int speed, uint8_t pinA, uint8_t pinB) {
    speed = (int)constrain(speed, -MAX_PWM, MAX_PWM);
    if (speed > 0) {
        analogWrite(pinA, speed);
        analogWrite(pinB, 0);
    } else if (speed < 0) {
        analogWrite(pinA, 0);
        analogWrite(pinB, -speed);
    } else {
        analogWrite(pinA, 0);
        analogWrite(pinB, 0);
    }
}

void stopMotors() {
    driveMotor(0, MOT_IN1_PIN, MOT_IN2_PIN);
    driveMotor(0, MOT_IN3_PIN, MOT_IN4_PIN);
}

float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float deadzone(float v, float dz) {
    float a = fabsf(v);
    if (a < dz) return 0;
    return v >= 0 ? (a - dz) / (1.0f - dz) : -(a - dz) / (1.0f - dz);
}
#include "motors.h"
#include "config.h"

void initMotors() {
    // IMPRESCINDIBLE antes del primer analogWrite(): el core de Arduino-ESP32
    // arranca en 8 bits (0..255) y 1 kHz. Sin esto, MAX_PWM=1023 es mentira
    // (todo lo que pase de 255 se recorta) y el driver chilla a 1 kHz.
    // Se fija primero la frecuencia y luego la resolución, que es el orden
    // que exige el core. A 10 bits el techo del LEDC son 78 kHz, así que
    // 20 kHz entra sin problema.
    analogWriteFrequency(PWM_FREQ);
    analogWriteResolution(PWM_RES);

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
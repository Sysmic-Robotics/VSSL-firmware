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

// Tabla de verdad del DRV8833, por canal:
//   IN1 IN2 | OUT1 OUT2 | modo
//    0   0  |  Z    Z   | libre  (decaimiento rapido)
//    1   0  |  H    L   | avance
//    0   1  |  L    H   | retroceso
//    1   1  |  L    L   | freno  (decaimiento lento)
//
// DECAIMIENTO RAPIDO alterna avance <-> libre: durante el tiempo apagado la
// corriente se extingue por los diodos, asi que a duty bajo casi no queda par
// y el motor no arranca hasta bien arriba. Es la causa de la zona muerta.
//
// DECAIMIENTO LENTO alterna avance <-> freno: la corriente sigue circulando
// por el puente durante el tiempo apagado, se mantiene el par a duty bajo y
// la relacion duty->velocidad queda mucho mas lineal. Cuesta algo mas de
// corriente (y de calor en el DRV8833), que es el precio a pagar.
void driveMotor(int speed, uint8_t pinA, uint8_t pinB) {
    speed = constrain(speed, -MAX_PWM, MAX_PWM);

    int pwm = abs(speed);

    if (speed == 0) {
        // Rueda libre, igual en los dos modos.
        analogWrite(pinA, 0);
        analogWrite(pinB, 0);
        return;
    }

#if MOTOR_SLOW_DECAY
    // El pin del sentido se mantiene alto y el otro lleva el PWM INVERTIDO:
    // a mas pwm, menos tiempo en freno y mas tiempo empujando.
    if (speed > 0) {
        analogWrite(pinA, MAX_PWM);
        analogWrite(pinB, MAX_PWM - pwm);
    } else {
        analogWrite(pinA, MAX_PWM - pwm);
        analogWrite(pinB, MAX_PWM);
    }
#else
    if (speed > 0) {
        analogWrite(pinA, pwm);
        analogWrite(pinB, 0);
    } else {
        analogWrite(pinA, 0);
        analogWrite(pinB, pwm);
    }
#endif
}

void stopMotors() {
    analogWrite(MOT_IN1_PIN, 0);
    analogWrite(MOT_IN2_PIN, 0);
    analogWrite(MOT_IN3_PIN, 0);
    analogWrite(MOT_IN4_PIN, 0);
}
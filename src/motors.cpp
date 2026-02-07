#include "motors.h"

void initMotors() {
    // Pines para la direccion  
    pinMode(MOT_AIN1_PIN, OUTPUT);
    pinMode(MOT_AIN2_PIN, OUTPUT);
    pinMode(MOT_BIN1_PIN, OUTPUT);
    pinMode(MOT_BIN2_PIN, OUTPUT);
    
    //Pines para el PWM
    pinMode(MOT_PWM_A_PIN, OUTPUT);
    pinMode(MOT_PWM_B_PIN, OUTPUT);



    analogWriteResolution(PWM_RES);
    analogWriteFrequency(PWM_FREQ);

    stopMotors(); //para ue partan detenidos
}

void setTB6612(int speed, uint8_t pinIN1, uint8_t pinIN2, uint8_t pinPWM) {
    if (speed >0) {
        // Para adelante
        digitalWrite(pinIN1, HIGH);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinPWM, speed);
    } else if (speed <0) {
        // Para atrás
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, HIGH);
        analogWrite(pinPWM, -speed);
    } else {
        // Detener
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);
        analogWrite(pinPWM, 0);
    }
}

void driveMotor(int speed, uint8_t motorID, uint8_t dummy) {
    if (motorID == MOT_AIN1_PIN) {
        setTB6612(speed, MOT_AIN1_PIN, MOT_AIN2_PIN, MOT_PWM_A_PIN);
    } else if (motorID == MOT_BIN1_PIN) {
        setTB6612(speed, MOT_BIN1_PIN, MOT_BIN2_PIN, MOT_PWM_B_PIN);
    }
}

void stopMotors() {
    driveMotor(0, MOT_AIN1_PIN, MOT_AIN2_PIN);
    driveMotor(0, MOT_BIN1_PIN, MOT_BIN2_PIN);
}

float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float deadzone(float v, float dz) {
    float a = fabsf(v);
    if (a < dz) return 0;
    return v >= 0 ? (a - dz) / (1.0f - dz) : -(a - dz) / (1.0f - dz);
}
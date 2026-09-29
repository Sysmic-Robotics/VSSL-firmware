#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
//        CONFIGURACIÓN DE MODO
// ==========================================
// Descomenta para usar ESP-NOW (WIFI), comenta para RemoteXY (Bluetooth)
// #define MODO_BASESTATION

// ID de este robot (1 al 5)
#define MI_ROBOT_ID 2

// ==========================================
//               PINES HARDWARE
// ==========================================

// Encoders
#define PIN_ENC_IZQ_A 1     // en el robotito con switch, el encoder taba dao vuelta. dar vuelta pa los demas en vola????
#define PIN_ENC_IZQ_B 0
#define PIN_ENC_DER_A 3
#define PIN_ENC_DER_B 4

// Driver Motores DRV8833
#define MOT_IN1_PIN 5   
#define MOT_IN2_PIN 6   
#define MOT_IN3_PIN 7   
#define MOT_IN4_PIN 10  

// MPU
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

// ==========================================
//           PARÁMETROS DE CONTROL
// ==========================================
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;     // 10 bits = 0 a 1023
const int      MAX_PWM  = 1023;

// ==========================================
//        PARÁMETROS FÍSICOS DEL ROBOT
// ==========================================

// Diámetro real de la rueda
#define WHEEL_DIAMETER_M 0.034

// Ticks de encoder por UNA vuelta completa de la rueda.
// Este valor hay que medirlo experimentalmente.
#define ENCODER_TICKS_PER_WHEEL_REV 575

// Geometría robótica para control diferencial.
// Distancia entre el centro de las dos ruedas (ancho de vía), en mm.
// IMPORTANTE: medir experimentalmente en el robot real, es crítico para
// que la velocidad angular comandada se traduzca en el giro real del robot.
#define WHEEL_TRACK_MM 75.0

// PID cada 20 ms
#define CONTROL_INTERVAL_MS 20
#define CONTROL_DT_S (CONTROL_INTERVAL_MS / 1000.0)

// Tiempo máximo sin recibir comandos
#define COMM_TIMEOUT_MS 200

// ==========================================
//        LÍMITES DE LAS CONSIGNAS (v, w)
// ==========================================
// Velocidad lineal máxima aceptada, en mm/s.
#define MAX_LINEAR_MM_S 1500
// Velocidad angular máxima aceptada, en mrad/s (1000 mrad/s = 1 rad/s).
#define MAX_ANGULAR_MRAD_S 12000

// Límite de seguridad para la velocidad de rueda resultante (cinemática + corrección)
#define MAX_WHEEL_MM_S 2000

// Para pruebas manuales con RemoteXY (joystick -> v, w)
#define JOYSTICK_MAX_LINEAR_MM_S 800
#define JOYSTICK_MAX_ANGULAR_MRAD_S 6000
#define JOYSTICK_DEADZONE 5

// Signo de ruedas.
// Si al mandar velocidad positiva una rueda gira al revés,
// cambia el signo correspondiente a -1.
#define LEFT_WHEEL_SIGN  1
#define RIGHT_WHEEL_SIGN 1
#define LEFT_ENCODER_SIGN  -1
#define RIGHT_ENCODER_SIGN -1

// Signo del eje Z del giroscopio del MPU6050.
// Si al girar el robot en sentido antihorario (w > 0) el gyro mide negativo,
// cambia este signo a -1 para que coincida con la convención de (v, w).
#define GYRO_Z_SIGN 1

// PID Gains (control de velocidad por rueda, con encoder)
extern double kp, ki, kd;

// Ganancias del corrector de guiñada (PI sobre el error de velocidad angular
// medida por el giroscopio vs. la comandada). Corrige asimetrías/deslizamiento
// que el encoder por sí solo no puede detectar.
extern double kp_yaw, ki_yaw;
// Máxima corrección diferencial que puede aportar el lazo de guiñada, en mm/s.
#define YAW_CORRECTION_LIMIT_MM_S 200

// Consignas globales recibidas desde la estación base / software de visión:
// velocidad lineal (mm/s) y velocidad angular (mrad/s) del robot completo.
extern volatile int16_t g_Linear_MmPerSec;
extern volatile int16_t g_Angular_MradPerSec;

#endif
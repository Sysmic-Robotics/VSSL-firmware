#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
//        CONFIGURACIÓN DE MODO
// ==========================================
// Descomenta para usar ESP-NOW (WIFI), comenta para RemoteXY (Bluetooth)
#define MODO_BASESTATION 

// Comentar para apagar el control por software (Controlar mediante mando externo)
//#define CONTROL_SOFTWARE 

// ID de este robot (1 al 5)
#define MI_ROBOT_ID 2

// ==========================================
//               PINES HARDWARE
// ==========================================

// Encoders
#define PIN_ENC_IZQ_A 32 //para c3 supermini 0
#define PIN_ENC_IZQ_B 33 //para c3 supermini 1
#define PIN_ENC_DER_A 25 //para c3 supermini 4
#define PIN_ENC_DER_B 26 //para c3 supermini 3

// Driver Motores DRV8833
#define MOT_IN1_PIN 16 //para c3 supermini 5   
#define MOT_IN2_PIN 17 //para c3 supermini 6   
#define MOT_IN3_PIN 18 //para c3 supermini 7   
#define MOT_IN4_PIN 19 //para c3 supermini 10  

// MPU
#define PIN_I2C_SDA 21 //para c3 supermini 8
#define PIN_I2C_SCL 22 //para c3 supermini 9

// ==========================================
//           PARÁMETROS DE CONTROL
// ==========================================
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;     // 10 bits = 0 a 1023
const int      MAX_PWM  = 1023;

// Geometria robotica para control diferencial
// r: radio de la rueda
// L: distancia de la rueda al centro del robot
const double WHEEL_RADIUS = 1.7; // 3.4 // 2.0
const double WHEEL_CENTER_DISTANCE = 3.7;  // 7.4//2

// PID Gains
extern double kp, ki, kd;

// Variables de consigna globales (disponibles para todo el proyecto)
extern float g_Input_X;
extern float g_Input_Y;

// Variables del mpu
extern float ax, ay, gy; //valores de aceleración en X e Y

#endif
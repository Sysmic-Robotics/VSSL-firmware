#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
//        CONFIGURACIÓN DE MODO
// ==========================================
// Descomenta para usar ESP-NOW (WIFI), comenta para RemoteXY (Bluetooth)
 #define MODO_BASESTATION 

// ID de este robot (1 al 5)
#define MI_ROBOT_ID 1

// ==========================================
//               PINES HARDWARE
// ==========================================

// Encoders
#define PIN_ENC_IZQ_A 0
#define PIN_ENC_IZQ_B 1
#define PIN_ENC_DER_A 4
#define PIN_ENC_DER_B 3

// Driver Motores tb6612fng (antes DRV8833)
#define MOT_AIN1_PIN 5   
#define MOT_AIN2_PIN 6   
#define MOT_BIN1_PIN 7   
#define MOT_BIN2_PIN 10  
#define MOT_PWM_A_PIN 8   
#define MOT_PWM_B_PIN 9

// ==========================================
//           PARÁMETROS DE CONTROL
// ==========================================
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;     // 10 bits = 0 a 1023
const int      MAX_PWM  = 1023;

// PID Gains
extern double kp, ki, kd;

// Variables de consigna globales (disponibles para todo el proyecto)
extern float g_Input_X;
extern float g_Input_Y;

#endif
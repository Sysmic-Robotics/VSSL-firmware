#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
//        CONFIGURACIÓN DE MODO
// ==========================================

// ID de este robot (1 al 5)
#define MI_ROBOT_ID 1

// ==========================================
//               PINES HARDWARE
// ==========================================

// Encoders
#define PIN_ENC_IZQ_A 1     
#define PIN_ENC_IZQ_B 0
#define PIN_ENC_DER_A 3
#define PIN_ENC_DER_B 4

// Driver Motores DRV8833

//Motor Derecho
#define MOT_IN1_PIN 5   
#define MOT_IN2_PIN 6   
//Motor Izquierdo
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

// Geometria robotica para control diferencial
// r: radio de la rueda
// L: distancia de la rueda al centro del robot
//const double WHEEL_RADIUS = 1.7; // 3.4 // 2.0
//const double WHEEL_CENTER_DISTANCE = 3.7;  // 7.4//2
#define WHEEL_CENTER_DISTANCE_MM 77

// PID cada 20 ms
#define CONTROL_INTERVAL_MS 20
#define CONTROL_DT_S (CONTROL_INTERVAL_MS / 1000.0)

// Tiempo máximo sin recibir comandos
#define COMM_TIMEOUT_MS 200

// Límite de seguridad para comandos recibidos 
#define MAX_WHEEL_TICKS_PER_SEC 8000

// Para pruebas con RemoteXY
#define JOYSTICK_MAX_TICKS_PER_SEC 1000
#define JOYSTICK_DEADZONE 5

// Signo de ruedas.
// Si al mandar left=1000/right=1000 una rueda gira al revés,
// cambia el signo correspondiente a -1.
// Juega con estos dos valores, prueba -1 en uno de los dos:
#define LEFT_WHEEL_SIGN   1
#define RIGHT_WHEEL_SIGN 1
#define LEFT_ENCODER_SIGN  -1
#define RIGHT_ENCODER_SIGN  -1

// PID Gains
extern double kp, ki, kd;

// Nuevas consignas globales: velocidades de rueda en mm/s
extern volatile int16_t g_v_cmd_mms;
extern volatile int16_t g_W_cmd_degs;

// Variables del mpu
extern float ax, ay, gy;


#endif
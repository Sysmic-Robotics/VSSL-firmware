/**
 * PROYECTO: Robot VSSS / VSSL SYSMIC ROBOTICS
 * UNIVERSIDAD: UTFSM - Casa Central
 * DESCRIPCIÓN: Firmware modular para robot de competencia.
 */

#include <Arduino.h>
#include "config.h"
#include "communication.h"
#include "motors.h"
#include "control.h"
#include "debug.h"
#include "mpu.h"

void setup() {
  DEBUG_INIT(115200);
  delay(2000); // Espera para estabilizar el monitor serial
  
  // Inicialización de módulos
  initMotors();
  initCommunication();
  initControl();
  initMPU();
  
  DEBUG_PRINTLN("Sistema inicializado.");
  #ifdef MODO_BASESTATION
    DEBUG_PRINTF("Modo: ESP-NOW | ID: %d\n", MI_ROBOT_ID);
  #else
    DEBUG_PRINTLN("Modo: Bluetooth (RemoteXY)");
  #endif
}

void loop() {
  // 1. Recibir datos de mando
  updateCommunication();
  
  // 2. Procesar PID y mover motores
  updateControl();

  // 3. Actualizar sensores (MPU)
  updateMPU();
  
  // 4. Telemetría opcional (usar con cuidado en competencia)
  // Serial.printf("X: %.2f Y: %.2f\n", g_Input_X, g_Input_Y);
  
}
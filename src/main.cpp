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
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("BOOT ROBOT VSSS");

  initMotors();
  Serial.println("Motores inicializados");
    

  initCommunication();
  Serial.println("Comunicacion inicializada");

  initControl();
  Serial.println("Control inicializado");

  Serial.println("Calibrando MPU6050 (robot debe estar quieto)...");
  initMPU();
  Serial.println("MPU6050 inicializado");

  Serial.println("Sistema iniciado");

  DEBUG_PRINTLN("Sistema inicializado.");
  #ifdef MODO_BASESTATION
    DEBUG_PRINTF("Modo: ESP-NOW | ID: %d\n", MI_ROBOT_ID);
  #else
    DEBUG_PRINTLN("Modo: Bluetooth (RemoteXY)");
  #endif
}

void loop() {
  // 1. Actualizar comunicación
  updateCommunication();

  // 2. Si no hay comunicación, comandos en cero y motores detenidos
  if (!isCommunicationConnected()) {
    clearVelocityCommands();
    stopMotors();
  }

  // 3. Leer giroscopio (velocidad angular real, para corrección de guiñada)
  updateMPU();

  // 4. Control de motores (cinemática + PID por rueda + corrección por gyro)
  updateControl();
}
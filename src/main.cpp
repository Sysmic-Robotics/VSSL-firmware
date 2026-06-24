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
#include "kalman.h"

void setup() {
  initMotors();
  
  Serial.begin(115200);
  delay(2000);

  initCommunication();
  initControl();
}

void loop() {
  // 1. Actualizar comunicación
  updateCommunication();

  // 2. Si no hay comunicación, comandos en cero y motores detenidos
  if (!isCommunicationConnected()) {
    clearWheelCommands();
    stopMotors();
  }

  // 3. Control de motores
  updateControl();

  // 4. Sensores opcionales
  updateMPU();

}
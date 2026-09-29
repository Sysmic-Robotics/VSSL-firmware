#ifndef MPU_H
#define MPU_H

#include "config.h"

void initMPU();
void updateMPU();

// Velocidad angular medida en el eje Z (guiñada) del robot, en rad/s,
// ya calibrada (bias restado) y filtrada. Signo según GYRO_Z_SIGN.
float getYawRateRadPerSec();

// Ángulo de guiñada acumulado, en rad, obtenido integrando la velocidad
// angular. Es un valor continuo (no se envuelve en ±PI) y va acumulando
// deriva lenta: sirve para comparar contra un rumbo memorizado hace poco,
// no como brújula absoluta.
float getYawAngleRad();

// Pone el ángulo acumulado en cero (define "aquí es rumbo 0").
void resetYawAngle();

// false si el MPU6050 no respondió en initMPU() (no bloquea el resto del robot,
// simplemente se deshabilita la corrección por giroscopio).
bool isMPUReady();

#endif

#ifndef MPU_H
#define MPU_H

#include "config.h"

void initMPU();
void updateMPU();

// Velocidad angular medida en el eje Z (guiñada) del robot, en rad/s,
// ya calibrada (bias restado) y filtrada. Signo según GYRO_Z_SIGN.
float getYawRateRadPerSec();

// false si el MPU6050 no respondió en initMPU() (no bloquea el resto del robot,
// simplemente se deshabilita la corrección por giroscopio).
bool isMPUReady();

#endif

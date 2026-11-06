#ifndef CONTROL_H
#define CONTROL_H

#include <Encoder.h>
#include <PID_v1.h>
#include "config.h"
#include "motors.h"

void initControl();
void updateControl();

// Telemetría del último ciclo de control (la escribe updateControl; la lee la prueba de banco).
// Todo con el signo de la RUEDA: positivo = la rueda empuja al robot hacia adelante.
struct TelemetriaRueda {
    double consigna_mm_s;   // consigna de la rueda (cinemática + guiñada + tope)
    double medida_mm_s;     // medida con el encoder en el último ciclo (dt real)
    int    pwm;             // PWM aplicado (-1023..1023)
    long   ticks;           // ticks del último ciclo
};

struct TelemetriaControl {
    unsigned long ciclos;   // cuántos ciclos de control van (cambia en cada ciclo)
    unsigned long t_ms;     // millis() del ciclo
    unsigned long dt_ms;    // duración real del ciclo
    TelemetriaRueda izq;
    TelemetriaRueda der;
    double yaw_medido_rad_s;
    double yaw_correccion_mm_s;
};

extern TelemetriaControl g_telemetria;
extern volatile long ticksIzq;
extern volatile long ticksDer;

// mm que avanza la rueda por tick de encoder (pi * diámetro / ticks por vuelta).
const double MM_POR_TICK = PI * WHEEL_DIAMETER_M * 1000.0 / ENCODER_TICKS_PER_WHEEL_REV;

#endif
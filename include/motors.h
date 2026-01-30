#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

void initMotors();
void driveMotor(int speed, uint8_t pinA, uint8_t pinB);
void stopMotors();

// Helpers matemáticos
float clamp(float v, float lo, float hi);
float deadzone(float v, float dz);

#endif
#ifndef CONTROL_H
#define CONTROL_H

#include <Encoder.h>
#include <PID_v1.h>
#include "config.h"
#include "motors.h"

void initControl();
void updateControl();

#endif
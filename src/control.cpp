#include "control.h"

Encoder encIzq(PIN_ENC_IZQ_A, PIN_ENC_IZQ_B);
Encoder encDer(PIN_ENC_DER_A, PIN_ENC_DER_B);

double setpointI, inputI, outputI;
double setpointD, inputD, outputD;

double kp=10.0, ki=0.2, kd=0.1;

PID pidIzq(&inputI, &outputI, &setpointI, kp, ki, kd, DIRECT);
PID pidDer(&inputD, &outputD, &setpointD, kp, ki, kd, DIRECT);

long oldPosI = 0, oldPosD = 0;
unsigned long lastPIDTime = 0;

void initControl() {
    pidIzq.SetMode(AUTOMATIC);
    pidDer.SetMode(AUTOMATIC);
    pidIzq.SetOutputLimits(-MAX_PWM, MAX_PWM);
    pidDer.SetOutputLimits(-MAX_PWM, MAX_PWM);
    pidIzq.SetSampleTime(20);
    pidDer.SetSampleTime(20);
}

void updateControl() {
    if (millis() - lastPIDTime >= 20) {
        long currPosI = encIzq.read();
        long currPosD = encDer.read();

        inputI = (double)(currPosI - oldPosI);
        inputD = (double)(currPosD - oldPosD);
        
        oldPosI = currPosI;
        oldPosD = currPosD;
        lastPIDTime = millis();

        // Mezcla cinemática diferencial simple
        double targetSpeed = g_Input_Y * 1.5;
        setpointI = targetSpeed + (g_Input_X * 1.0);
        setpointD = targetSpeed - (g_Input_X * 1.0);

        if (setpointI == 0){
            pidIzq.SetMode(MANUAL);
            outputI = 0;
        } else {
            pidIzq.SetMode(AUTOMATIC);
            pidIzq.Compute();
        }
        
        if (setpointD == 0){
            pidDer.SetMode(MANUAL);
            outputD = 0;
        } else {
            pidDer.SetMode(AUTOMATIC);
            pidDer.Compute();
        }
        
        driveMotor((int)outputI, MOT_AIN1_PIN, 0);
        driveMotor((int)outputD, MOT_BIN1_PIN, 0);
    }
}
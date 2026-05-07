#include "control.h"
#include "debug.h"

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
        #ifdef CONTROL_SOFTWARE
            // Entrada desde software: g_Input_X = velocidad lineal, g_Input_Y = velocidad angular
            // double linearCmd = g_Input_X * 3.0;
            // double angularCmd = g_Input_Y * 1.0;


            // w = (1/r) * (v +/- (w*L)/2)
            //setpointI = (1/WHEEL_RADIUS) * (linearCmd + (angularCmd * WHEEL_CENTER_DISTANCE) * 0.5);
            //setpointD = (1/WHEEL_RADIUS) * (linearCmd - (angularCmd * WHEEL_CENTER_DISTANCE) * 0.5);
            setpointI = g_Input_X * 3;
            setpointD = g_Input_Y * 3;
        #else

            // Entrada desde software: g_Input_X = velocidad lineal, g_Input_Y = velocidad angular
            //double linearCmd = g_Input_X * 3.0;
            //double angularCmd = g_Input_Y * 1.0;
            // w = (1/r) * (v +/- (w*L)/2)
            //setpointI = (1/WHEEL_RADIUS) * (linearCmd + (angularCmd * WHEEL_CENTER_DISTANCE) * 0.5);
            //setpointD = (1/WHEEL_RADIUS) * (linearCmd - (angularCmd * WHEEL_CENTER_DISTANCE) * 0.5);
            double targetSpeed = g_Input_Y * 3;
            setpointI = targetSpeed + (g_Input_X * 1.0);
            setpointD = targetSpeed - (g_Input_X * 1.0);
        #endif

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
        
        driveMotor((int)outputI, MOT_IN1_PIN, MOT_IN2_PIN);
        driveMotor((int)outputD, MOT_IN3_PIN, MOT_IN4_PIN);

        // DEBUG 
        DEBUG_PRINT("TgtL:"); DEBUG_PRINT(setpointI);
        DEBUG_PRINT(" ActL:"); DEBUG_PRINT(inputI);
        DEBUG_PRINT(" OutL:"); DEBUG_PRINT(outputI);

        DEBUG_PRINT(" | TgtR:"); DEBUG_PRINT(setpointD);
        DEBUG_PRINT(" ActR:"); DEBUG_PRINT(inputD);
        DEBUG_PRINTLN(" OutR:"); DEBUG_PRINTLN(outputD);
    }
}
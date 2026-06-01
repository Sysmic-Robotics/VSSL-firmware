#include "control.h"
#include "debug.h"

Encoder encIzq(PIN_ENC_IZQ_A, PIN_ENC_IZQ_B);
Encoder encDer(PIN_ENC_DER_A, PIN_ENC_DER_B);

double setpointI, inputI, outputI;
double setpointD, inputD, outputD;

double kp=1, ki=0.10, kd=0.0;

PID pidIzq(&inputI, &outputI, &setpointI, kp, ki, kd, DIRECT);
PID pidDer(&inputD, &outputD, &setpointD, kp, ki, kd, DIRECT);

long oldPosI = 0, oldPosD = 0;
unsigned long lastPIDTime = 0;

const int PWM_STATIC = 150;          // PWM base para vencer zona muerta
const double PWM_PER_MM_S = 0.15;    // cuánto PWM suma por cada mm/s
const int PID_CORRECTION_LIMIT = 250;
const int CMD_DEADBAND_MM_S = 20;

double mmpsToTicksPerControlCycle(double velocity_mm_s) {
    double velocity_m_s = velocity_mm_s / 1000.0;
    double wheel_circumference = PI * WHEEL_DIAMETER_M;

    double rev_per_sec = velocity_m_s / wheel_circumference;
    double ticks_per_sec = rev_per_sec * ENCODER_TICKS_PER_WHEEL_REV;

    return ticks_per_sec * CONTROL_DT_S;
}

int feedForwardPWM(int velocity_mm_s) {
    if (abs(velocity_mm_s) < CMD_DEADBAND_MM_S) {
        return 0;
    }

    int sign = (velocity_mm_s > 0) ? 1 : -1;

    int pwm = PWM_STATIC + (int)(PWM_PER_MM_S * abs(velocity_mm_s));
    pwm = constrain(pwm, 0, MAX_PWM);

    return sign * pwm;
}


void initControl() {
    pidIzq.SetOutputLimits(-PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);
    pidDer.SetOutputLimits(-PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);

    pidIzq.SetSampleTime(CONTROL_INTERVAL_MS);
    pidDer.SetSampleTime(CONTROL_INTERVAL_MS);

    pidIzq.SetMode(AUTOMATIC);
    pidDer.SetMode(AUTOMATIC);
}
void updateControl() {
    if (millis() - lastPIDTime >= CONTROL_INTERVAL_MS) {

        long currPosI = encIzq.read();
        long currPosD = encDer.read();

        inputI = LEFT_ENCODER_SIGN * (double)(currPosI - oldPosI);
        inputD = RIGHT_ENCODER_SIGN * (double)(currPosD - oldPosD);

        oldPosI = currPosI;
        oldPosD = currPosD;
        lastPIDTime = millis();

        int leftCmdMmS  = LEFT_WHEEL_SIGN  * g_Left_MmPerSec;
        int rightCmdMmS = RIGHT_WHEEL_SIGN * g_Right_MmPerSec;

        setpointI = mmpsToTicksPerControlCycle(leftCmdMmS);
        setpointD = mmpsToTicksPerControlCycle(rightCmdMmS);

        if (leftCmdMmS == 0) {
            pidIzq.SetMode(MANUAL);
            outputI = 0;
        } else {
            if (pidIzq.GetMode() != AUTOMATIC) pidIzq.SetMode(AUTOMATIC);
            pidIzq.Compute();
        }

        if (rightCmdMmS == 0) {
            pidDer.SetMode(MANUAL);
            outputD = 0;
        } else {
            if (pidDer.GetMode() != AUTOMATIC) pidDer.SetMode(AUTOMATIC);
            pidDer.Compute();
        }

        int ffI = feedForwardPWM(leftCmdMmS);
        int ffD = feedForwardPWM(rightCmdMmS);

        int pwmI = ffI + (int)outputI;
        int pwmD = ffD + (int)outputD;

        if (leftCmdMmS == 0) pwmI = 0;
        if (rightCmdMmS == 0) pwmD = 0;

        pwmI = constrain(pwmI, -MAX_PWM, MAX_PWM);
        pwmD = constrain(pwmD, -MAX_PWM, MAX_PWM);

        driveMotor(pwmI, MOT_IN1_PIN, MOT_IN2_PIN);
        driveMotor(pwmD, MOT_IN3_PIN, MOT_IN4_PIN);

        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 500) {
            Serial.print("CmdL_mm/s:");
            Serial.print(g_Left_MmPerSec);

            Serial.print(" CmdR_mm/s:");
            Serial.print(g_Right_MmPerSec);

            Serial.print(" SetL_ticks:");
            Serial.print(setpointI);

            Serial.print(" ActL_ticks:");
            Serial.print(inputI);

            Serial.print(" FF_L:");
            Serial.print(ffI);

            Serial.print(" PID_L:");
            Serial.print(outputI);

            Serial.print(" PWM_L:");
            Serial.print(pwmI);

            Serial.print(" | SetR_ticks:");
            Serial.print(setpointD);

            Serial.print(" ActR_ticks:");
            Serial.print(inputD);

            Serial.print(" FF_R:");
            Serial.print(ffD);

            Serial.print(" PID_R:");
            Serial.print(outputD);

            Serial.print(" PWM_R:");
            Serial.println(pwmD);

            lastDebug = millis();
        }
    }
}
/*
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
            double linearCmd = g_Input_X * 3.0;
            double angularCmd = g_Input_Y * 1.0;


            // w = (1/r) * (v +/- (w*L)/2)
            //setpointI = (1/WHEEL_RADIUS) * (linearCmd + (angularCmd * WHEEL_CENTER_DISTANCE) * 0.5);
            //setpointD = (1/WHEEL_RADIUS) * (linearCmd - (angularCmd * WHEEL_CENTER_DISTANCE) * 0.5);
            setpointI = g_Input_X * 2;
            setpointD = g_Input_Y * 3;
        #else

            // Entrada tipo joystick:
            // g_Input_Y = avance / retroceso
            // g_Input_X = giro izquierda / derecha

            const double LINEAR_GAIN  = 3.0;
            const double TURN_GAIN    = 3.0;
            const double JOYSTICK_DEADZONE = 5.0;

            double linearCmd = g_Input_Y;
            double turnCmd   = g_Input_X;

            if (abs(linearCmd) < JOYSTICK_DEADZONE) linearCmd = 0;
            if (abs(turnCmd)   < JOYSTICK_DEADZONE) turnCmd   = 0;

            double linearSpeed = linearCmd * LINEAR_GAIN;
            double turnSpeed   = turnCmd   * TURN_GAIN;

            setpointI = linearSpeed + turnSpeed;
            setpointD = linearSpeed - turnSpeed;

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
    */
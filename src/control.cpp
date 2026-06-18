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

const int PWM_STATIC = 75;          // PWM base para vencer zona muerta
const double PWM_PER_MM_S = 0.075;    // cuánto PWM suma por cada mm/s
const int PID_CORRECTION_LIMIT = 250;
const int CMD_DEADBAND_MM_S = 20;
const float SPEED_SCALE = 0.25f;  // 0.5 = mitad, 1.0 = completo

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

        int leftCmdMmS  = LEFT_WHEEL_SIGN  * (int)(g_Left_MmPerSec * SPEED_SCALE);
        int rightCmdMmS = RIGHT_WHEEL_SIGN * (int)(g_Right_MmPerSec * SPEED_SCALE);

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
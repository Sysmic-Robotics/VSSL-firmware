#include "control.h"
#include "debug.h"
#include "kalman.h"
#include "mpu.h"

KalmanVelocityFilter filtroOmega(0.02f, 15.0f);
float omega_filtrada_deg_s = 0.0;

Encoder encIzq(PIN_ENC_IZQ_A, PIN_ENC_IZQ_B);
Encoder encDer(PIN_ENC_DER_A, PIN_ENC_DER_B);

double setpointI, inputI, outputI;
double setpointD, inputD, outputD;

double kp = 1.0, ki = 0.10, kd = 0.0;

PID pidIzq(&inputI, &outputI, &setpointI, kp, ki, kd, DIRECT);
PID pidDer(&inputD, &outputD, &setpointD, kp, ki, kd, DIRECT);

long oldPosI = 0, oldPosD = 0;
unsigned long lastPIDTime = 0;

// Parámetros calibrados para resolución de 10 bits (0-1023)
const int PWM_STATIC = 90;
const double PWM_PER_MM_S = 2.00;
const int PID_CORRECTION_LIMIT = 400;
const int CMD_DEADBAND_MM_S = 20;
const float SPEED_SCALE = 1.0f;

double mmpsToTicksPerControlCycle(double velocity_mm_s)
{
    double velocity_m_s = velocity_mm_s / 1000.0;
    double wheel_circumference = PI * WHEEL_DIAMETER_M;
    double rev_per_sec = velocity_m_s / wheel_circumference;
    double ticks_per_sec = rev_per_sec * ENCODER_TICKS_PER_WHEEL_REV;
    return ticks_per_sec * CONTROL_DT_S;
}

int feedForwardPWM(int velocity_mm_s)
{
    if (abs(velocity_mm_s) < CMD_DEADBAND_MM_S)
    {
        return 0;
    }
    int sign = (velocity_mm_s > 0) ? 1 : -1;
    int pwm = PWM_STATIC + (int)(PWM_PER_MM_S * abs(velocity_mm_s));
    pwm = constrain(pwm, 0, MAX_PWM);
    return sign * pwm;
}

void initControl()
{
    pidIzq.SetOutputLimits(-PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);
    pidDer.SetOutputLimits(-PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);
    pidIzq.SetSampleTime(CONTROL_INTERVAL_MS);
    pidDer.SetSampleTime(CONTROL_INTERVAL_MS);
    pidIzq.SetMode(AUTOMATIC);
    pidDer.SetMode(AUTOMATIC);
}

void updateControl()
{
    if (millis() - lastPIDTime >= CONTROL_INTERVAL_MS)
    {
        long currPosI = encIzq.read();
        long currPosD = encDer.read();

        inputI = LEFT_ENCODER_SIGN * (double)(currPosI - oldPosI);
        inputD = RIGHT_ENCODER_SIGN * (double)(currPosD - oldPosD);

        oldPosI = currPosI;
        oldPosD = currPosD;
        lastPIDTime = millis();

        // 1. Cinemática directa de odometría (mm/s de avance en sistema de coordenadas del robot):
        float factor_conversion = (PI * (WHEEL_DIAMETER_M * 1000.0)) / (ENCODER_TICKS_PER_WHEEL_REV * CONTROL_DT_S);

        float vel_Izq_mms = inputI * factor_conversion;
        // CORREGIDO: Como RIGHT_WHEEL_SIGN es -1, el avance del robot produce inputD negativo.
        // Se normaliza la velocidad hacia adelante de la rueda derecha dividiendo por su signo de rueda:
        float vel_Der_mms = (inputD / RIGHT_WHEEL_SIGN) * factor_conversion;

        // Velocidad angular medida por encoders (rad/s y deg/s):
        float omega_enc_rad_s = (vel_Der_mms - vel_Izq_mms) / (WHEEL_CENTER_DISTANCE_MM);
        float omega_enc_deg_s = omega_enc_rad_s * (180.0f / PI);

        // 2. Fusión Kalman (Gyro + Encoders):
        omega_filtrada_deg_s = filtroOmega.update(gy, omega_enc_deg_s);

        // 3. Consignas y corrección:
        float v_cmd = (float)g_v_cmd_mms * SPEED_SCALE;
        float w_cmd = (float)g_W_cmd_degs * SPEED_SCALE;

        bool idle = (v_cmd == 0.0f && w_cmd == 0.0f);

        float leftCmdMmS_calc = 0;
        float rightCmdMmS_calc = 0;

        if (!idle)
        {
            float kp_w = 0.05f;
            float error_w = w_cmd - omega_filtrada_deg_s;
            float w_correction = kp_w * error_w;
            float w_final_rad_s = (w_cmd + w_correction) * (PI / 180.0f);

            leftCmdMmS_calc = v_cmd - (w_final_rad_s * (WHEEL_CENTER_DISTANCE_MM / 2.0f));
            rightCmdMmS_calc = v_cmd + (w_final_rad_s * (WHEEL_CENTER_DISTANCE_MM / 2.0f));
        }

        int leftCmdMmS = LEFT_WHEEL_SIGN * (int)leftCmdMmS_calc;
        int rightCmdMmS = RIGHT_WHEEL_SIGN * (int)rightCmdMmS_calc;

        setpointI = mmpsToTicksPerControlCycle(leftCmdMmS);
        setpointD = mmpsToTicksPerControlCycle(rightCmdMmS);

        // 4. Lazo PID por rueda:
        if (idle)
        {
            pidIzq.SetMode(MANUAL);
            pidDer.SetMode(MANUAL);
            outputI = 0;
            outputD = 0;
        }
        else
        {
            if (pidIzq.GetMode() != AUTOMATIC)
                pidIzq.SetMode(AUTOMATIC);
            if (pidDer.GetMode() != AUTOMATIC)
                pidDer.SetMode(AUTOMATIC);
            pidIzq.Compute();
            pidDer.Compute();
        }

        int ffI = feedForwardPWM(leftCmdMmS);
        int ffD = feedForwardPWM(rightCmdMmS);

        int pwmI = ffI + (int)outputI;
        int pwmD = ffD + (int)outputD;

        if (idle)
        {
            pwmI = 0;
            pwmD = 0;
        }

        pwmI = constrain(pwmI, -MAX_PWM, MAX_PWM);
        pwmD = constrain(pwmD, -MAX_PWM, MAX_PWM);

        driveMotor(pwmI, MOT_IN1_PIN, MOT_IN2_PIN);
        driveMotor(pwmD, MOT_IN3_PIN, MOT_IN4_PIN);
    }
}

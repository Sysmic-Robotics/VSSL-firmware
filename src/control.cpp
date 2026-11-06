#include "control.h"
#include "debug.h"
#include "mpu.h"

volatile long ticksIzq = 0;
volatile long ticksDer = 0;

void IRAM_ATTR isrIzqA() {
    if (digitalRead(PIN_ENC_IZQ_A) == digitalRead(PIN_ENC_IZQ_B)) {
        ticksIzq++;
    } else {
        ticksIzq--;
    }
}


void IRAM_ATTR isrIzqB() {
    if (digitalRead(PIN_ENC_IZQ_A) != digitalRead(PIN_ENC_IZQ_B)) {
        ticksIzq++;
    } else {
        ticksIzq--;
    }
}

void IRAM_ATTR isrDerA() {
    if (digitalRead(PIN_ENC_DER_A) == digitalRead(PIN_ENC_DER_B)) {
        ticksDer++;
    } else {
        ticksDer--;
    }
}

void IRAM_ATTR isrDerB() {
    if (digitalRead(PIN_ENC_DER_A) != digitalRead(PIN_ENC_DER_B)) {
        ticksDer++;
    } else {
        ticksDer--;
    }
}


Encoder encIzq(PIN_ENC_IZQ_A, PIN_ENC_IZQ_B);
Encoder encDer(PIN_ENC_DER_A, PIN_ENC_DER_B);

double setpointI, inputI, outputI;
double setpointD, inputD, outputD;

// Ganancias del PID de velocidad por rueda. La salida está en unidades de
// PWM, así que al pasar la escala real de 255 a 1023 (ver initMotors) hay
// que multiplicarlas por 4 para conservar la misma respuesta del lazo.
double kp=2.5, ki=0.2, kd=0.0;

// Ganancias del corrector de guiñada (lazo exterior sobre el gyro).
// El robot alcanza altas velocidades angulares, por lo que ki_yaw se mantiene
// bajo para evitar oscilaciones; ajustar en cancha.
double kp_yaw = 30.0, ki_yaw = 2.0;

TelemetriaControl g_telemetria = {};

PID pidIzq(&inputI, &outputI, &setpointI, kp, ki, kd, DIRECT);
PID pidDer(&inputD, &outputD, &setpointD, kp, ki, kd, DIRECT);

long oldPosI = 0, oldPosD = 0;
unsigned long lastPIDTime = 0;

// --- Feedforward: PWM estimado para una velocidad de rueda dada ---
// Ambos valores estaban afinados a mano para la escala real de 255 que había
// antes, y con un error grave: PWM_STATIC=150 era un SUELO del 59% de duty
// aplicado a cualquier consigna > CMD_DEADBAND_MM_S. Por eso al pedir
// 48 mm/s la rueda salía a ~306 mm/s: el feedforward disparaba solo.
//
// Recalibrados sobre la escala correcta de 1023 y con la medición real de
// la telemetría (47% de duty -> ~306 mm/s, o sea ~650 mm/s a fondo):
//   PWM_STATIC    -> solo lo justo para vencer la fricción de arranque.
//                    Para medirlo: bájalo hasta que la rueda ya no arranque
//                    sola con una consigna mínima, y súbele un 20%.
//   PWM_PER_MM_S  -> pendiente hasta el tope: (1023 - 90) / 464 ~= 2.01.
//
// Calibrado con dos puntos medidos en la rueda izquierda:
//   PWM  542 (53% duty) -> 260 mm/s
//   PWM 1023 (100%)     -> 464 mm/s   <- tope físico con batería 2S
const int PWM_STATIC = 90;
const double PWM_PER_MM_S = 2.00;
// Autoridad del PID sobre el feedforward. Con el feedforward ya calibrado le
// basta con ~±100 para cerrar, así que 400 da margen de sobra y a la vez
// acota el windup del integrador si alguna vez se pide algo inalcanzable.
const int PID_CORRECTION_LIMIT = 400;
const int CMD_DEADBAND_MM_S = 20;

// Límite del término integral del corrector de guiñada, en rad (error*dt acumulado).
const double YAW_INTEGRAL_LIMIT = 5.0;
static double yawIntegral = 0.0;

double mmpsToTicksPerControlCycle(double velocity_mm_s) {
    double velocity_m_s = velocity_mm_s / 1000.0;
    double wheel_circumference = PI * WHEEL_DIAMETER_M;

    double rev_per_sec = velocity_m_s / wheel_circumference;
    double ticks_per_sec = rev_per_sec * ENCODER_TICKS_PER_WHEEL_REV;

    return ticks_per_sec * CONTROL_DT_S;
}

int feedForwardPWM(double velocity_mm_s) {
    if (fabs(velocity_mm_s) < CMD_DEADBAND_MM_S) {
        return 0;
    }

    int sign = (velocity_mm_s > 0) ? 1 : -1;

    int pwm = PWM_STATIC + (int)(PWM_PER_MM_S * fabs(velocity_mm_s));
    pwm = constrain(pwm, 0, MAX_PWM);

    return sign * pwm;
}


void initControl() {
    pinMode(PIN_ENC_IZQ_A, INPUT_PULLUP);
    pinMode(PIN_ENC_IZQ_B, INPUT_PULLUP);
    pinMode(PIN_ENC_DER_A, INPUT_PULLUP);
    pinMode(PIN_ENC_DER_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_IZQ_A), isrIzqA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_IZQ_B), isrIzqB, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DER_A), isrDerA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_DER_B), isrDerB, CHANGE);

    pidIzq.SetOutputLimits(-PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);
    pidDer.SetOutputLimits(-PID_CORRECTION_LIMIT, PID_CORRECTION_LIMIT);

    pidIzq.SetSampleTime(CONTROL_INTERVAL_MS);
    pidDer.SetSampleTime(CONTROL_INTERVAL_MS);

    pidIzq.SetMode(AUTOMATIC);
    pidDer.SetMode(AUTOMATIC);
}

void updateControl() {
    if (millis() - lastPIDTime >= CONTROL_INTERVAL_MS) {

        long currPosI = ticksIzq;
        long currPosD = ticksDer;

        inputI = LEFT_ENCODER_SIGN * (double)(currPosI - oldPosI);
        inputD = RIGHT_ENCODER_SIGN * (double)(currPosD - oldPosD);

        oldPosI = currPosI;
        oldPosD = currPosD;
        unsigned long ahoraCiclo = millis();
        unsigned long dtCiclo_ms = ahoraCiclo - lastPIDTime;
        lastPIDTime = ahoraCiclo;

        // --------------------------------------------------------
        // 1. Cinemática diferencial: (v, w) del robot -> velocidad
        //    objetivo de cada rueda, en mm/s.
        // --------------------------------------------------------
        double linearCmd_mm_s  = g_Linear_MmPerSec;          // v, mm/s
        double angularCmd_rad_s = g_Angular_MradPerSec / 1000.0; // w, rad/s

        bool idle = (g_Linear_MmPerSec == 0 && g_Angular_MradPerSec == 0);

        double halfTrack_mm = WHEEL_TRACK_MM / 2.0;

        // --------------------------------------------------------
        // 2. Corrección de guiñada con el giroscopio: compara la w
        //    realmente medida por el MPU6050 contra la comandada y
        //    aporta una corrección diferencial adicional. Esto
        //    compensa deslizamiento de ruedas y asimetrías mecánicas
        //    que el encoder, al medir solo la rueda, no puede ver.
        // --------------------------------------------------------
        double yawCorrection_mm_s = 0.0;

        if (!idle && isMPUReady()) {
            double yawMeasured_rad_s = getYawRateRadPerSec();
            double yawError_rad_s = angularCmd_rad_s - yawMeasured_rad_s;

            yawIntegral += yawError_rad_s * CONTROL_DT_S;
            yawIntegral = constrain(yawIntegral, -YAW_INTEGRAL_LIMIT, YAW_INTEGRAL_LIMIT);

            yawCorrection_mm_s = kp_yaw * yawError_rad_s + ki_yaw * yawIntegral;
            yawCorrection_mm_s = constrain(yawCorrection_mm_s, -YAW_CORRECTION_LIMIT_MM_S, YAW_CORRECTION_LIMIT_MM_S);
        } else {
            yawIntegral = 0.0;
        }

        double leftTarget_mm_s  = linearCmd_mm_s - angularCmd_rad_s * halfTrack_mm - yawCorrection_mm_s / 2.0;
        double rightTarget_mm_s = linearCmd_mm_s + angularCmd_rad_s * halfTrack_mm + yawCorrection_mm_s / 2.0;

        // Si la mezcla le pide a alguna rueda más de lo que el robot da, se
        // escalan las DOS por el mismo factor. Recortarlas por separado
        // cambiaría la relación v/w, o sea la curvatura: el robot dejaría de
        // seguir el arco que le pidieron, que es justo lo que el software de
        // visión no puede permitirse. Escalando en bloque recorre el mismo
        // arco, sólo que más lento.
        double maxWheel_mm_s = fmax(fabs(leftTarget_mm_s), fabs(rightTarget_mm_s));
        if (maxWheel_mm_s > MAX_WHEEL_MM_S) {
            double k = MAX_WHEEL_MM_S / maxWheel_mm_s;
            leftTarget_mm_s  *= k;
            rightTarget_mm_s *= k;
        }

        double leftCmdMmS  = LEFT_WHEEL_SIGN  * leftTarget_mm_s;
        double rightCmdMmS = RIGHT_WHEEL_SIGN * rightTarget_mm_s;

        // --------------------------------------------------------
        // 3. Lazo interno por rueda: PID sobre ticks de encoder para
        //    alcanzar la velocidad de rueda objetivo.
        // --------------------------------------------------------
        setpointI = mmpsToTicksPerControlCycle(leftCmdMmS);
        setpointD = mmpsToTicksPerControlCycle(rightCmdMmS);

        if (idle) {
            pidIzq.SetMode(MANUAL);
            outputI = 0;
        } else {
            if (pidIzq.GetMode() != AUTOMATIC) pidIzq.SetMode(AUTOMATIC);
            pidIzq.Compute();
        }

        if (idle) {
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

        if (idle) {
            pwmI = 0;
            pwmD = 0;
        }

        pwmI = constrain(pwmI, -MAX_PWM, MAX_PWM);
        pwmD = constrain(pwmD, -MAX_PWM, MAX_PWM);

        driveMotor(pwmI, MOT_IN1_PIN, MOT_IN2_PIN);
        driveMotor(pwmD, MOT_IN3_PIN, MOT_IN4_PIN);

        // Telemetría (solo anota lo ya calculado; no cambia el control).
        double dtTele_s = (dtCiclo_ms > 0 ? dtCiclo_ms : CONTROL_INTERVAL_MS) / 1000.0;
        g_telemetria.ciclos++;
        g_telemetria.t_ms = ahoraCiclo;
        g_telemetria.dt_ms = dtCiclo_ms;
        g_telemetria.izq.consigna_mm_s = leftTarget_mm_s;
        g_telemetria.izq.medida_mm_s = LEFT_WHEEL_SIGN * inputI * MM_POR_TICK / dtTele_s;
        g_telemetria.izq.pwm = LEFT_WHEEL_SIGN * pwmI;
        g_telemetria.izq.ticks = (long)inputI;
        g_telemetria.der.consigna_mm_s = rightTarget_mm_s;
        g_telemetria.der.medida_mm_s = RIGHT_WHEEL_SIGN * inputD * MM_POR_TICK / dtTele_s;
        g_telemetria.der.pwm = RIGHT_WHEEL_SIGN * pwmD;
        g_telemetria.der.ticks = (long)inputD;
        g_telemetria.yaw_medido_rad_s = isMPUReady() ? getYawRateRadPerSec() : 0.0;
        g_telemetria.yaw_correccion_mm_s = yawCorrection_mm_s;

        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > 500) {
            DEBUG_PRINT("v_mm/s:"); DEBUG_PRINT(g_Linear_MmPerSec);
            DEBUG_PRINT(" w_mrad/s:"); DEBUG_PRINT(g_Angular_MradPerSec);
            DEBUG_PRINT(" yawMeas_rad/s:"); DEBUG_PRINT(getYawRateRadPerSec());
            DEBUG_PRINT(" yawCorr_mm/s:"); DEBUG_PRINT(yawCorrection_mm_s);

            DEBUG_PRINT(" | SetL_ticks:"); DEBUG_PRINT(setpointI);
            DEBUG_PRINT(" ActL_ticks:"); DEBUG_PRINT(inputI);
            DEBUG_PRINT(" PWM_L:"); DEBUG_PRINT(pwmI);

            DEBUG_PRINT(" | SetR_ticks:"); DEBUG_PRINT(setpointD);
            DEBUG_PRINT(" ActR_ticks:"); DEBUG_PRINT(inputD);
            DEBUG_PRINT(" PWM_R:"); DEBUG_PRINTLN(pwmD);

            lastDebug = millis();
        }
    }
}

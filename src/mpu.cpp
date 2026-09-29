#include "mpu.h"
#include "debug.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Cada cuánto se lee el MPU6050. Va en paralelo al lazo de control (20 ms).
#define MPU_INTERVAL_MS CONTROL_INTERVAL_MS

// Constante de tiempo del filtro pasa-bajos aplicado a la lectura de gyro Z,
// para suavizar ruido sin introducir demasiado retardo frente al lazo de control.
#define YAW_RATE_FILTER_ALPHA 0.35f

static Adafruit_MPU6050 mpu;

static bool mpuReady = false;
static float gyroZBiasRadS = 0.0f;
static volatile float yawRateRadS = 0.0f;
static volatile float yawAngleRad = 0.0f;
static unsigned long lastMpuTime = 0;

void initMPU() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    mpuReady = mpu.begin(0x68, &Wire);
    if (!mpuReady) {
        DEBUG_PRINTLN("MPU6050 no detectado: corrección por giroscopio deshabilitada.");
        return;
    }

    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Calibración del bias del gyro Z: el robot debe estar quieto al encender.
    const int muestras = 200;
    double sumaGyroZ = 0.0;

    for (int i = 0; i < muestras; i++) {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        sumaGyroZ += g.gyro.z;
        delay(3);
    }

    gyroZBiasRadS = (float)(sumaGyroZ / muestras);
    yawRateRadS = 0.0f;
    yawAngleRad = 0.0f;
    lastMpuTime = millis();

    DEBUG_PRINT("MPU6050 listo. Bias gyroZ (rad/s): ");
    DEBUG_PRINTLN(gyroZBiasRadS);
}

void updateMPU() {
    if (!mpuReady) return;

    unsigned long now = millis();
    if (now - lastMpuTime >= MPU_INTERVAL_MS) {
        float dt = (now - lastMpuTime) / 1000.0f;
        lastMpuTime = now;

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        float raw = (GYRO_Z_SIGN) * (g.gyro.z - gyroZBiasRadS);
        yawRateRadS = YAW_RATE_FILTER_ALPHA * raw + (1.0f - YAW_RATE_FILTER_ALPHA) * yawRateRadS;

        // Integración del ángulo. Si hubo una pausa larga (arranque, bloqueo),
        // no integramos ese salto como si fuera movimiento real.
        if (dt > 0.2f) dt = 0.2f;
        yawAngleRad += yawRateRadS * dt;
    }
}

float getYawRateRadPerSec() {
    return yawRateRadS;
}

float getYawAngleRad() {
    return yawAngleRad;
}

void resetYawAngle() {
    yawAngleRad = 0.0f;
}

bool isMPUReady() {
    return mpuReady;
}

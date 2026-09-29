#include "mpu.h"
#include "debug.h"
#include <Wire.h>

// Acceso directo a registros, sin librería, a propósito:
// Adafruit_MPU6050::begin() valida WHO_AM_I y exige 0x68, y muchos módulos
// del mercado son clones que devuelven otro id pero funcionan igual. Aquí se
// lee WHO_AM_I solo para informar, nunca para rechazar el sensor.

#define MPU_REG_SMPLRT_DIV  0x19
#define MPU_REG_CONFIG      0x1A
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_GYRO_ZOUT_H 0x47
#define MPU_REG_PWR_MGMT_1  0x6B
#define MPU_REG_WHO_AM_I    0x75

// Cada cuánto se lee el MPU6050. Va en paralelo al lazo de control (20 ms).
#define MPU_INTERVAL_MS CONTROL_INTERVAL_MS

// Constante de tiempo del filtro pasa-bajos aplicado a la lectura de gyro Z,
// para suavizar ruido sin introducir demasiado retardo frente al lazo.
#define YAW_RATE_FILTER_ALPHA 0.35f

// Escala del giroscopio: ±1000 °/s -> 32.8 LSB por °/s.
// Hace falta este rango: un pivote de 400 mm/s de rueda sobre una vía de
// 75 mm son ~611 °/s, que saturaría una escala de ±500 °/s.
#define MPU_GYRO_FS_SEL 2
#define MPU_GYRO_LSB_PER_DPS 32.8f

static uint8_t mpuAddress = 0;
static bool mpuReady = false;
static float gyroZBiasDps = 0.0f;
static volatile float yawRateRadS = 0.0f;
static volatile float yawAngleRad = 0.0f;
static unsigned long lastMpuTime = 0;

static bool writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

static bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len) {
    Wire.beginTransmission(mpuAddress);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom((uint16_t)mpuAddress, (size_t)len, true) != len) return false;

    for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

static bool probeAddress(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Diagnóstico de arranque: dice exactamente qué hay en el bus. Se imprime
// siempre (no solo con MODO_DEBUG) porque es una sola vez y sin él un fallo
// de I2C es invisible.
static void scanI2C() {
    Serial.println("Escaneando bus I2C...");

    uint8_t encontrados = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (probeAddress(addr)) {
            Serial.printf("  dispositivo en 0x%02X\n", addr);
            encontrados++;
        }
    }

    if (encontrados == 0) {
        Serial.printf("  bus vacio: revisa SDA=GPIO%d, SCL=GPIO%d, 3V3, GND y pull-ups\n",
                      PIN_I2C_SDA, PIN_I2C_SCL);
    }
}

static bool readGyroZRaw(int16_t *out) {
    uint8_t b[2];
    if (!readBytes(MPU_REG_GYRO_ZOUT_H, b, 2)) return false;

    *out = (int16_t)(((uint16_t)b[0] << 8) | b[1]);
    return true;
}

void initMPU() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000);

    mpuReady = false;
    yawRateRadS = 0.0f;
    yawAngleRad = 0.0f;

    scanI2C();

    // AD0 sin soldar deja la dirección por defecto en 0x68, pero según el
    // módulo el pin puede quedar flotante y resolverse en 0x69. Se prueban
    // las dos en vez de darlo por supuesto.
    if (probeAddress(0x68)) {
        mpuAddress = 0x68;
    } else if (probeAddress(0x69)) {
        mpuAddress = 0x69;
    } else {
        Serial.println("MPU6050 no responde en 0x68 ni 0x69: correccion por giroscopio deshabilitada.");
        return;
    }
    Serial.printf("MPU6050 en 0x%02X\n", mpuAddress);

    uint8_t whoami = 0;
    if (readBytes(MPU_REG_WHO_AM_I, &whoami, 1)) {
        Serial.printf("  WHO_AM_I = 0x%02X%s\n", whoami,
                      (whoami == 0x68) ? " (original)" : " (clon: funciona igual)");
    }

    // Salir de reposo. Es la única escritura cuyo fallo aborta: si no
    // responde, no hay sensor con el que trabajar.
    if (!writeReg(MPU_REG_PWR_MGMT_1, 0x00)) {
        Serial.println("MPU6050 no acepta escrituras: correccion por giroscopio deshabilitada.");
        return;
    }
    delay(100);

    writeReg(MPU_REG_CONFIG, 0x04);                      // DLPF ~20 Hz
    writeReg(MPU_REG_GYRO_CONFIG, MPU_GYRO_FS_SEL << 3); // ±1000 °/s
    writeReg(MPU_REG_SMPLRT_DIV, 0x04);                  // 1 kHz / 5 = 200 Hz

    // Calibración del bias del gyro Z: el robot debe estar quieto al encender.
    const int muestras = 200;
    double suma = 0.0;
    int validas = 0;

    for (int i = 0; i < muestras; i++) {
        int16_t raw;
        if (readGyroZRaw(&raw)) {
            suma += raw;
            validas++;
        }
        delay(3);
    }

    if (validas < muestras / 2) {
        Serial.println("MPU6050 dejo de responder durante la calibracion.");
        return;
    }

    gyroZBiasDps = (float)(suma / validas) / MPU_GYRO_LSB_PER_DPS;
    lastMpuTime = millis();
    mpuReady = true;

    Serial.printf("MPU6050 listo. Bias gyroZ = %.2f deg/s\n", gyroZBiasDps);
}

void updateMPU() {
    if (!mpuReady) return;

    unsigned long now = millis();
    if (now - lastMpuTime >= MPU_INTERVAL_MS) {
        float dt = (now - lastMpuTime) / 1000.0f;
        lastMpuTime = now;

        int16_t raw;
        if (!readGyroZRaw(&raw)) return;

        float dps = (raw / MPU_GYRO_LSB_PER_DPS) - gyroZBiasDps;
        float rate = (GYRO_Z_SIGN) * dps * (float)DEG_TO_RAD;

        yawRateRadS = YAW_RATE_FILTER_ALPHA * rate + (1.0f - YAW_RATE_FILTER_ALPHA) * yawRateRadS;

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

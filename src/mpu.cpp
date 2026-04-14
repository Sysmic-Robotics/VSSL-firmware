#include "mpu.h"
#include "debug.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float g_angulo_Z = 0;
void initMPU() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    if (!mpu.begin()) {
        Serial.println("Error al inicializar el MPU6050");
    } else {
        DEBUG_PRINTLN("MPU6050 inicializado correctamente");

        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
}

void updateMPU() {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);
    g_angulo_Z = gyro.gyro.z;
    DEBUG_PRINTLN(g_angulo_Z);
}
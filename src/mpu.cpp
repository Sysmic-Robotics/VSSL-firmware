#include "mpu.h"
#include "debug.h"
#include "config.h"
#include <Wire.h>

const uint8_t MPU_ADDR = 0x68; // Dirección I2C del MPU6050

float offsetx = 0.0;
float offsety = 0.0;

int16_t lastMPUtime = 0;

int16_t tempx, tempy, tempg;

void initMPU() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(400000); // Configurar a 400kHz para una comunicación más rápida

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0);    // Despertar el MPU
    Wire.endTransmission();

    //Configuracion del filtro

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); // CONFIG
    Wire.write(0x04); // Configuración del filtro digital (DLPF) a 20Hz
    Wire.endTransmission();

    //calibracion del acelerometro

    int muestras = 100;
    long sumax = 0, sumay = 0;
    
    for (int i = 0; i < muestras; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B); // Dirección del registro de datos del acelerómetro
        Wire.endTransmission(false);

        Wire.requestFrom(MPU_ADDR, (size_t)4, true); // Solicitar 4 bytes (X e Y)

        int16_t rawAx = (Wire.read() << 8 | Wire.read()); // Aceleración en X
        int16_t rawAy = (Wire.read() << 8 | Wire.read()); // Aceleración en Y

        sumax += rawAx;
        sumay += rawAy;
        delay(2);
    }
    
    offsetx = ((float)sumax / muestras / 16384.0)* 9.81; // Convertir a m/s² (asumiendo sensibilidad de ±2g)
    offsety = ((float)sumay / muestras / 16384.0)* 9.81; // Convertir a m/s² (asumiendo sensibilidad de ±2g)

    DEBUG_PRINT("Setup listo. OffsetX: "); DEBUG_PRINT(offsetx); DEBUG_PRINT(" m/s² | OffsetY: "); DEBUG_PRINT(offsety); DEBUG_PRINTLN(" m/s²");
}

void updateMPU() {
    if (millis() - lastMPUtime >= 20){
        lastMPUtime = millis();
        
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x3B); // Dirección del registro de datos del acelerómetro
        
        if (Wire.endTransmission(false) != 0) {
            DEBUG_PRINTLN("Error al comunicarse con el MPU6050");
            return;
        }

        uint8_t bytes = Wire.requestFrom(MPU_ADDR, (size_t)14, true); // Solicitar 14 bytes (X, Y, Z para aceleración y giroscopio)
        if (bytes == 14) {
        tempx = Wire.read() << 8 | Wire.read(); // Aceleración en X
        tempy = Wire.read() << 8 | Wire.read(); // Aceleración en Y
        Wire.read(); Wire.read(); // Ignorar Z del acelerómetro
        Wire.read(); Wire.read(); // Ignorar temperatura
        Wire.read(); Wire.read(); // Ignorar X del giroscopio
        Wire.read(); Wire.read(); // Ignorar Y del giroscopio
        tempg = Wire.read() << 8 | Wire.read(); // Giroscopio

        float ax = (tempx / 16384.0) * 9.81 - offsetx; // Convertir a m/s² y aplicar offset
        float ay = (tempy / 16384.0) * 9.81 - offsety; // Convertir a m/s² y aplicar offset
        float gy = tempg / 131.0; // Convertir a grados/s (asumiendo sensibilidad de ±250°/s)

        DEBUG_PRINT("Acceleracion en X: "); DEBUG_PRINT(ax); DEBUG_PRINT(" m/s² | Aceleracion en Y: "); DEBUG_PRINT(ay); DEBUG_PRINT(" m/s² | Giroscopio Z: "); DEBUG_PRINT(gy); DEBUG_PRINTLN(" °/s");
        }
    }
}
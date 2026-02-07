#include "communication.h"

// Inicialización de variables globales
float g_Input_X = 0;
float g_Input_Y = 0;

#ifdef MODO_BASESTATION
  #include <esp_now.h>
  #include <WiFi.h>

  typedef struct {
    int8_t x[5]; // x_robot_1, x_robot_2...
    int8_t y[5];
  } struct_mensaje_vss;

  struct_mensaje_vss incomingData;

  void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    // Verificación básica de longitud
    if (len >= 10) {
        int offset = (MI_ROBOT_ID - 1) * 2;
        // Casteo seguro de los datos recibidos
        g_Input_X = (int8_t)data[offset];
        g_Input_Y = (int8_t)data[offset + 1];
    }
  }
#else
  // --- CONFIGURACIÓN REMOTEXY BLUETOOTH ---
  #define REMOTEXY_MODE__ESP32CORE_BLE
  #include <BLEDevice.h>
  
  // CORRECCIÓN: Esta definición debe ir ANTES de incluir RemoteXY.h
  #define REMOTEXY_BLUETOOTH_NAME "RemoteXY" 
  
  #include <RemoteXY.h>

  #pragma pack(push, 1)  
  uint8_t RemoteXY_CONF[] = { 
    255,3,0,0,0,49,0,19,0,0,0,82,111,98,111,116,105,116,111,0,
    31,2,106,200,200,84,1,1,2,0,5,54,136,49,49,147,35,45,45,32,
    1,24,31,1,9,145,31,31,16,50,24,24,0,2,31,0 
  };
  
  struct {
    int8_t Mando_X; 
    int8_t Mando_Y; 
    uint8_t Boton_1; 
    uint8_t connect_flag; 
  } RemoteXY;
  #pragma pack(pop)
#endif

void initCommunication() {
    #ifdef MODO_BASESTATION
        WiFi.mode(WIFI_STA);
        if (esp_now_init() == ESP_OK) {
            esp_now_register_recv_cb(OnDataRecv);
        }
    #else
        RemoteXY_Init();
    #endif
}

void updateCommunication() {
    #ifndef MODO_BASESTATION
        RemoteXY_Handler();
        g_Input_X = RemoteXY.Mando_X;
        g_Input_Y = RemoteXY.Mando_Y;
    #endif
}
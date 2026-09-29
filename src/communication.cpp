#include "communication.h"
#include "debug.h"
#include <string.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 1
// ==========================================
//       VARIABLES GLOBALES DE COMANDO
// ==========================================
// Consignas del robot completo: velocidad lineal (mm/s) y angular (mrad/s).
// La cinemática diferencial (reparto entre rueda izquierda/derecha) y la
// corrección por giroscopio se resuelven en control.cpp.

volatile int16_t g_Linear_MmPerSec = 0;
volatile int16_t g_Angular_MradPerSec = 0;

// Estado de comunicación
static unsigned long lastCommandTime = 0;
static bool communicationConnected = false;


// ==========================================
//       FUNCIONES AUXILIARES
// ==========================================

void clearVelocityCommands() {
    g_Linear_MmPerSec = 0;
    g_Angular_MradPerSec = 0;
}

static void markCommandReceived() {
    lastCommandTime = millis();
    communicationConnected = true;
}

static int16_t clampInt16(int32_t value, int32_t limit) {
    if (value > limit) return (int16_t)limit;
    if (value < -limit) return (int16_t)-limit;
    return (int16_t)value;
}

// ==========================================
//              MODO ESP-NOW
// ==========================================

#ifdef MODO_BASESTATION

#include <esp_now.h>
#include <WiFi.h>

#define COMM_MAGIC 0xA5
#define COMM_VERSION 2
#define NUM_ROBOTS 5

// Consigna de velocidad del robot completo, tal como la entrega el software
// de visión: velocidad lineal (mm/s) y angular (mrad/s). La estación base
// solo retransmite estos valores, no le interesa cómo se generan.
typedef struct __attribute__((packed)) {
    int16_t linear_mm_s;
    int16_t angular_mrad_s;
} RobotVelocityCommand;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint16_t seq;
    RobotVelocityCommand robots[NUM_ROBOTS];
} CommandPacket;

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    if (len != sizeof(CommandPacket)) {
        DEBUG_PRINTLN("Len incorrecto");
        return;
    }

    CommandPacket packet;
    memcpy(&packet, data, sizeof(CommandPacket));

    if (packet.magic != COMM_MAGIC) {
        DEBUG_PRINTLN("Magic incorrecto");
        return;
    }

    if (packet.version != COMM_VERSION) {
        DEBUG_PRINTLN("Version incorrecta");
        return;
    }

    int idx = MI_ROBOT_ID - 1;

    g_Linear_MmPerSec = clampInt16(packet.robots[idx].linear_mm_s, MAX_LINEAR_MM_S);
    g_Angular_MradPerSec = clampInt16(packet.robots[idx].angular_mrad_s, MAX_ANGULAR_MRAD_S);

    DEBUG_PRINT("Cmd recibido v=");
    DEBUG_PRINT(g_Linear_MmPerSec);
    DEBUG_PRINT(" w=");
    DEBUG_PRINTLN(g_Angular_MradPerSec);

    markCommandReceived();
}
#else

// ==========================================
//              MODO REMOTEXY
// ==========================================

#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>

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




// ==========================================
//              API PÚBLICA
// ==========================================

void initCommunication() {
    clearVelocityCommands();
    communicationConnected = false;
    lastCommandTime = 0;

#ifdef MODO_BASESTATION
    Serial.println("Modo ESP-NOW receptor activo");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    Serial.print("MAC robot: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW en robot");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    Serial.print("Canal ESP-NOW robot: ");
    Serial.println(ESPNOW_CHANNEL);

    Serial.println("ESP-NOW robot listo para recibir");
#else
    Serial.println("Modo RemoteXY activo");
    RemoteXY_Init();
#endif
}


void updateCommunication() {

#ifdef MODO_BASESTATION

    if (communicationConnected &&
        (millis() - lastCommandTime > COMM_TIMEOUT_MS)) {

        clearVelocityCommands();
        communicationConnected = false;
    }

#else

    RemoteXY_Handler();

    if (RemoteXY.connect_flag) {

        // Mando_Y = avance/retroceso -> velocidad lineal
        // Mando_X = giro izquierda/derecha -> velocidad angular
        int16_t linear = map(
            RemoteXY.Mando_Y,
            -100, 100,
            -JOYSTICK_MAX_LINEAR_MM_S,
            JOYSTICK_MAX_LINEAR_MM_S
        );

        int16_t angular = map(
            RemoteXY.Mando_X,
            -100, 100,
            -JOYSTICK_MAX_ANGULAR_MRAD_S,
            JOYSTICK_MAX_ANGULAR_MRAD_S
        );

        if (abs(RemoteXY.Mando_Y) < JOYSTICK_DEADZONE) linear = 0;
        if (abs(RemoteXY.Mando_X) < JOYSTICK_DEADZONE) angular = 0;

        g_Linear_MmPerSec = clampInt16(linear, MAX_LINEAR_MM_S);
        g_Angular_MradPerSec = clampInt16(angular, MAX_ANGULAR_MRAD_S);

        markCommandReceived();

    } else {
        clearVelocityCommands();
        communicationConnected = false;
    }

#endif
}


bool isCommunicationConnected() {

#ifdef MODO_BASESTATION
    return communicationConnected &&
           (millis() - lastCommandTime <= COMM_TIMEOUT_MS);
#else
    return communicationConnected;
#endif

}

#include "communication.h"
#include <string.h>

// ==========================================
//       VARIABLES GLOBALES DE COMANDO
// ==========================================

volatile int16_t g_Left_TicksPerSec = 0;
volatile int16_t g_Right_TicksPerSec = 0;

// Estado de comunicación
static unsigned long lastCommandTime = 0;
static bool communicationConnected = false;


// ==========================================
//       FUNCIONES AUXILIARES
// ==========================================

void clearWheelCommands() {
    g_Left_TicksPerSec = 0;
    g_Right_TicksPerSec = 0;
}

static void markCommandReceived() {
    lastCommandTime = millis();
    communicationConnected = true;
}

static int16_t clampWheelCommand(int16_t value) {
    if (value > MAX_WHEEL_TICKS_PER_SEC) return MAX_WHEEL_TICKS_PER_SEC;
    if (value < -MAX_WHEEL_TICKS_PER_SEC) return -MAX_WHEEL_TICKS_PER_SEC;
    return value;
}

// ==========================================
//              MODO ESP-NOW
// ==========================================

#ifdef MODO_BASESTATION

#include <esp_now.h>
#include <WiFi.h>

#define COMM_MAGIC 0xA5
#define COMM_VERSION 1
#define NUM_ROBOTS 5

typedef struct __attribute__((packed)) {
    int16_t left_ticks_per_sec;
    int16_t right_ticks_per_sec;
} RobotWheelCommand;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint16_t seq;
    RobotWheelCommand robots[NUM_ROBOTS];
} CommandPacket;

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    if (len != sizeof(CommandPacket)) {
        return;
    }

    CommandPacket packet;
    memcpy(&packet, data, sizeof(CommandPacket));
//Esto sirve para asegurarse de que el paquete recibido realmente pertenece al protocolo.
    if (packet.magic != COMM_MAGIC) {
        return;
    }
//Esto sirve para asegurarse de que el paquete recibido es compatible con la versión del protocolo que estamos usando.
    if (packet.version != COMM_VERSION) {
        return;
    }

    int idx = MI_ROBOT_ID - 1;

    if (idx < 0 || idx >= NUM_ROBOTS) {
        clearWheelCommands();
        return;
    }

    g_Left_TicksPerSec = clampWheelCommand(packet.robots[idx].left_ticks_per_sec);
    g_Right_TicksPerSec = clampWheelCommand(packet.robots[idx].right_ticks_per_sec);

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
    clearWheelCommands();
    communicationConnected = false;
    lastCommandTime = 0;

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

#ifdef MODO_BASESTATION

    if (communicationConnected &&
        (millis() - lastCommandTime > COMM_TIMEOUT_MS)) {

        clearWheelCommands();
        communicationConnected = false;
    }

#else

    RemoteXY_Handler();

    if (RemoteXY.connect_flag) {

        int16_t linear = map(
            RemoteXY.Mando_Y,
            -100, 100,
            -JOYSTICK_MAX_TICKS_PER_SEC,
            JOYSTICK_MAX_TICKS_PER_SEC
        );

        int16_t turn = map(
            RemoteXY.Mando_X,
            -100, 100,
            -JOYSTICK_MAX_TICKS_PER_SEC,
            JOYSTICK_MAX_TICKS_PER_SEC
        );

        if (abs(RemoteXY.Mando_Y) < JOYSTICK_DEADZONE) linear = 0;
        if (abs(RemoteXY.Mando_X) < JOYSTICK_DEADZONE) turn = 0;

        int32_t left = linear + turn;
        int32_t right = linear - turn;

        left = constrain(left, -MAX_WHEEL_TICKS_PER_SEC, MAX_WHEEL_TICKS_PER_SEC);
        right = constrain(right, -MAX_WHEEL_TICKS_PER_SEC, MAX_WHEEL_TICKS_PER_SEC);

        g_Left_TicksPerSec = (int16_t)left;
        g_Right_TicksPerSec = (int16_t)right;

        markCommandReceived();

    } else {
        clearWheelCommands();
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
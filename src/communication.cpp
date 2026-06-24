#include "communication.h"
#include <string.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 1
// ==========================================
//       VARIABLES GLOBALES DE COMANDO
// ==========================================

volatile int16_t g_v_cmd_mms = 0;
volatile int16_t g_W_cmd_degs = 0;

// Estado de comunicación
static unsigned long lastCommandTime = 0;
static bool communicationConnected = false;


// ==========================================
//       FUNCIONES AUXILIARES
// ==========================================

void clearWheelCommands() {
    g_v_cmd_mms = 0;
    g_W_cmd_degs = 0;
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

#include <esp_now.h>
#include <WiFi.h>

#define COMM_MAGIC 0xA5
#define COMM_VERSION 1
#define NUM_ROBOTS 5

typedef struct __attribute__((packed)) {
    int16_t left_mm_s;
    int16_t right_mm_s;
} RobotWheelCommand;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint16_t seq;
    RobotWheelCommand robots[NUM_ROBOTS];
} CommandPacket;

void OnDataRecv(const uint8_t * mac, const uint8_t *data, int len) {
    Serial.print("RX ESP-NOW len=");
    Serial.println(len);

    if (len != sizeof(CommandPacket)) {
        Serial.println("Len incorrecto");
        return;
    }

    CommandPacket packet;
    memcpy(&packet, data, sizeof(CommandPacket));

    Serial.print("Magic=");
    Serial.println(packet.magic, HEX);

    Serial.print("Version=");
    Serial.println(packet.version);

    if (packet.magic != COMM_MAGIC) {
        Serial.println("Magic incorrecto");
        return;
    }

    if (packet.version != COMM_VERSION) {
        Serial.println("Version incorrecta");
        return;
    }

    int idx = MI_ROBOT_ID - 1;

    g_v_cmd_mms = packet.robots[idx].left_mm_s;
    g_W_cmd_degs = packet.robots[idx].right_mm_s;

    Serial.print("Cmd recibido v=");
    Serial.print(g_v_cmd_mms);
    Serial.print(" W=");
    Serial.println(g_W_cmd_degs);

    markCommandReceived();
}



// ==========================================
//              API PÚBLICA
// ==========================================

void initCommunication() {
    clearWheelCommands();
    communicationConnected = false;
    lastCommandTime = 0;


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
}


void updateCommunication() {


if (communicationConnected &&
    (millis() - lastCommandTime > COMM_TIMEOUT_MS)) {

    clearWheelCommands();
    communicationConnected = false;
}
}


bool isCommunicationConnected() {

return communicationConnected &&
       (millis() - lastCommandTime <= COMM_TIMEOUT_MS);

}
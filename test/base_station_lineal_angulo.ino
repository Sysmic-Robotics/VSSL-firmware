#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <string.h>

// ==========================================
// CONFIGURACIÓN
// ==========================================

#define NUM_ROBOTS 5
#define COMM_MAGIC 0xA5
#define COMM_VERSION 1
#define ESPNOW_CHANNEL 1

// Nuevos límites de seguridad para V y W
#define MAX_V_MM_S 1500
#define MAX_W_DEG_S 720  // Límite de giro en grados/segundo
#define SEND_INTERVAL_MS 50

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
esp_now_peer_info_t peerInfo;

// ==========================================
// PROTOCOLO ACTUALIZADO (V y W)
// ==========================================

typedef struct __attribute__((packed)) {
    int16_t v_mms;
    int16_t w_degs;
} RobotCommand;

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint16_t seq;
  RobotCommand robots[NUM_ROBOTS];
} CommandPacket;

CommandPacket packet;
uint16_t sequenceNumber = 0;
unsigned long lastSendTime = 0;

// ==========================================
// FUNCIONES AUXILIARES
// ==========================================

int16_t clampV(int value) {
  if (value > MAX_V_MM_S) return MAX_V_MM_S;
  if (value < -MAX_V_MM_S) return -MAX_V_MM_S;
  return (int16_t)value;
}

int16_t clampW(int value) {
  if (value > MAX_W_DEG_S) return MAX_W_DEG_S;
  if (value < -MAX_W_DEG_S) return -MAX_W_DEG_S;
  return (int16_t)value;
}

void clearPacketCommands() {
  packet.magic = COMM_MAGIC;
  packet.version = COMM_VERSION;
  packet.seq = sequenceNumber++;

  for (int i = 0; i < NUM_ROBOTS; i++) {
    packet.robots[i].v_mms = 0;
    packet.robots[i].w_degs = 0;
  }
}

void sendPacket() {
  packet.magic = COMM_MAGIC;
  packet.version = COMM_VERSION;
  packet.seq = sequenceNumber++;
  
  esp_err_t result = esp_now_send(
    broadcastAddress,
    (uint8_t *)&packet,
    sizeof(packet)
  );
  
  if (result != ESP_OK) {
    Serial.println("Error enviando paquete ESP-NOW");
  }
}

// ==========================================
// CALLBACK DE ENVÍO
// ==========================================

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  static int last_status = -1;
  if (status != last_status) {
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
    last_status = status;
  }
}

// ==========================================
// SETUP
// ==========================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  Serial.print("MAC base station: ");
  Serial.println(WiFi.macAddress());
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fallo al agregar peer broadcast");
    return;
  }

  clearPacketCommands();

  Serial.println("Base station lista.");
  Serial.print("Tamano paquete: ");
  Serial.println(sizeof(CommandPacket));

  Serial.println("Formato serial:");
  Serial.println("V1,W1,V2,W2,V3,W3,V4,W4,V5,W5");
  Serial.println("Ejemplo:");
  Serial.println("500,90,0,0,0,0,0,0,0,0");
}

// ==========================================
// LOOP
// ==========================================

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    int v1, w1, v2, w2, v3, w3, v4, w4, v5, w5;
    
    int leidos = sscanf(
      data.c_str(),
      "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
      &v1, &w1,
      &v2, &w2,
      &v3, &w3,
      &v4, &w4,
      &v5, &w5
    );
    
    if (leidos == 10) {
      packet.robots[0].v_mms  = clampV(v1);
      packet.robots[0].w_degs = clampW(w1);
      
      packet.robots[1].v_mms  = clampV(v2);
      packet.robots[1].w_degs = clampW(w2);

      packet.robots[2].v_mms  = clampV(v3);
      packet.robots[2].w_degs = clampW(w3);

      packet.robots[3].v_mms  = clampV(v4);
      packet.robots[3].w_degs = clampW(w4);

      packet.robots[4].v_mms  = clampV(v5);
      packet.robots[4].w_degs = clampW(w5);

      Serial.println("Comando actualizado:");
      Serial.print("R1 V=");
      Serial.print(packet.robots[0].v_mms);
      Serial.print(" W=");
      Serial.println(packet.robots[0].w_degs);
      
    } else {
      Serial.println("Formato invalido.");
      Serial.println("Usa: V1,W1,V2,W2,V3,W3,V4,W4,V5,W5");
    }
  }

  // Manda el paquete por ESP-NOW a todos los robots cada 50ms
  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    sendPacket();
    lastSendTime = millis();
  }
}
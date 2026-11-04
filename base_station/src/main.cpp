#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 50;
// ==========================================
//              CONFIGURACIÓN
// ==========================================

#define NUM_ROBOTS 5
#define COMM_MAGIC 0xA5
#define COMM_VERSION 1

// Límites de lo que se acepta por serie. Son los mismos que usa la base de
// test (base_station_lineal_angulo.ino). El robot recorta además a ~722 °/s.
#define MAX_LINEAR_MM_S 1500
#define MAX_ANGULAR_DEG_S 720

// Broadcast ESP-NOW
uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

esp_now_peer_info_t peerInfo;

// ==========================================
//            PROTOCOLO DE COMANDOS
// ==========================================

// CONTRATO CON EL ROBOT (communication.cpp, MODO_BASESTATION):
//   magic 0xA5, versión 1, paquete de 24 bytes.
//   V = velocidad lineal en mm/s.
//   W = velocidad angular en GRADOS/s (el robot la pasa a rad/s).
// La estación base no convierte nada: solo retransmite los valores tal como
// los entrega el software de visión (que ya envía W en grados/s); cada robot
// resuelve la cinemática diferencial y el control internamente.
typedef struct __attribute__((packed)) {
  int16_t linear_mm_s;
  int16_t angular_deg_s;
} RobotVelocityCommand;

typedef struct __attribute__((packed)) {
  uint8_t magic;
  uint8_t version;
  uint16_t seq;
  RobotVelocityCommand robots[NUM_ROBOTS];
} CommandPacket;

CommandPacket packet;
uint16_t sequenceNumber = 0;

// ==========================================
//              TELEMETRÍA OPCIONAL
// ==========================================

typedef struct __attribute__((packed)) {
  int id;
  float accelx;
  float accely;
  float anguloz;
  float velocidad_lineal;
} struct_telemetria;

struct_telemetria datosRobots;

// ==========================================
//              FUNCIONES AUXILIARES
// ==========================================

int16_t clampCommand(int value, int limit) {
  if (value > limit) {
    return (int16_t)limit;
  }

  if (value < -limit) {
    return (int16_t)-limit;
  }

  return (int16_t)value;
}

void clearPacketCommands() {
  packet.magic = COMM_MAGIC;
  packet.version = COMM_VERSION;
  packet.seq = sequenceNumber++;

  for (int i = 0; i < NUM_ROBOTS; i++) {
    packet.robots[i].linear_mm_s = 0;
    packet.robots[i].angular_deg_s = 0;
  }
}

void sendPacket() {
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
//              CALLBACKS ESP-NOW
// ==========================================

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Puedes activar esto para debug, pero ensucia mucho el monitor serial
  /*
  Serial.print("Estado envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "ERROR");
  */
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(struct_telemetria)) {
    return;
  }

  memcpy(&datosRobots, incomingData, sizeof(datosRobots));

  Serial.print("ID: ");
  Serial.print(datosRobots.id);

  Serial.print(" Velocidad: ");
  Serial.print(datosRobots.velocidad_lineal);

  Serial.print(" AccX: ");
  Serial.print(datosRobots.accelx);

  Serial.print(" AccY: ");
  Serial.print(datosRobots.accely);

  Serial.print(" GyroZ/Angulo: ");
  Serial.println(datosRobots.anguloz);
}
// ==========================================
//                  SETUP
// ==========================================

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  Serial.print("MAC base station: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fallo al agregar peer broadcast");
    return;
  }

  clearPacketCommands();

  Serial.println("Base station lista.");
  Serial.println("Formato serial (v en mm/s, w en grados/s):");
  Serial.println("V1,W1,V2,W2,V3,W3,V4,W4,V5,W5");
  Serial.println("Ejemplo (robot 1 avanza recto a 500 mm/s):");
  Serial.println("500,0,0,0,0,0,0,0,0,0");
}

// ==========================================
//                  LOOP
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
      packet.robots[0].linear_mm_s = clampCommand(v1, MAX_LINEAR_MM_S);
      packet.robots[0].angular_deg_s = clampCommand(w1, MAX_ANGULAR_DEG_S);

      packet.robots[1].linear_mm_s = clampCommand(v2, MAX_LINEAR_MM_S);
      packet.robots[1].angular_deg_s = clampCommand(w2, MAX_ANGULAR_DEG_S);

      packet.robots[2].linear_mm_s = clampCommand(v3, MAX_LINEAR_MM_S);
      packet.robots[2].angular_deg_s = clampCommand(w3, MAX_ANGULAR_DEG_S);

      packet.robots[3].linear_mm_s = clampCommand(v4, MAX_LINEAR_MM_S);
      packet.robots[3].angular_deg_s = clampCommand(w4, MAX_ANGULAR_DEG_S);

      packet.robots[4].linear_mm_s = clampCommand(v5, MAX_LINEAR_MM_S);
      packet.robots[4].angular_deg_s = clampCommand(w5, MAX_ANGULAR_DEG_S);

      Serial.println("Comando actualizado.");
    } else {
      Serial.println("Formato invalido.");
      Serial.println("Usa: V1,W1,V2,W2,V3,W3,V4,W4,V5,W5");
    }
  }

  if (millis() - lastSendTime >= SEND_INTERVAL_MS) {
    packet.magic = COMM_MAGIC;
    packet.version = COMM_VERSION;
    packet.seq = sequenceNumber++;

    sendPacket();

    lastSendTime = millis();
  }
}
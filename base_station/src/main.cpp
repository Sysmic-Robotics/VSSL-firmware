#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <string.h>
#include <stdlib.h>

// ==========================================
//              CONFIGURACIÓN
// ==========================================

#define NUM_ROBOTS 5
#define COMM_MAGIC 0xA5
#define COMM_VERSION 1
#define ESPNOW_CHANNEL 1   // el mismo que ESPNOW_CHANNEL del robot (communication.cpp)

// Límites de lo que se acepta por serie. Son los mismos que usa la base de
// test (base_station_lineal_angulo.ino). El robot recorta además a ~722 °/s.
#define MAX_LINEAR_MM_S 1500
#define MAX_ANGULAR_DEG_S 720

// Envío (5-nov-2026). Antes la base mandaba el último comando cada 50 ms, llegara lo que llegara:
// le sumaba al robot 0-50 ms de retraso (25 de media) y de los ~60 comandos por segundo del engine
// solo salían 20. Ahora cada línea válida sale EN EL ACTO; si no llega ninguna, se repite la última
// cada KEEPALIVE_MS para que el robot no corte por su COMM_TIMEOUT_MS (200 ms).
const unsigned long KEEPALIVE_MS = 50;

// Vigilancia del serie: si en SERIAL_TIMEOUT_MS no llega ninguna línea válida, la base pasa a
// mandar ceros (antes repetía el último comando PARA SIEMPRE: con el engine colgado o el USB del PC
// suelto, el robot seguía andando). El engine manda una línea por tick (62.5 Hz) y skill_test una
// cada 50 ms. 0 = sin vigilancia (el comportamiento anterior, para mover robots a mano desde un
// monitor serie escribiendo una sola línea).
const unsigned long SERIAL_TIMEOUT_MS = 300;

// Cada cuánto se imprime el resumen (líneas válidas, inválidas, envíos). 0 = nunca.
const unsigned long REPORT_MS = 5000;

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

// Estado del envío
unsigned long lastSendTime = 0;
unsigned long lastValidLineTime = 0;
bool serialTimedOut = false;

// Lectura de líneas sin bloquear (antes Serial.readStringUntil, que espera hasta 1 s si la
// línea llega cortada).
char lineBuffer[96];
size_t lineLength = 0;

// Contadores para el resumen
unsigned long linesOk = 0;
unsigned long linesBad = 0;
unsigned long sends = 0;
unsigned long sendErrors = 0;
unsigned long lastReport = 0;

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
  for (int i = 0; i < NUM_ROBOTS; i++) {
    packet.robots[i].linear_mm_s = 0;
    packet.robots[i].angular_deg_s = 0;
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

  sends++;
  if (result != ESP_OK) {
    sendErrors++;
  }
  lastSendTime = millis();
}

// Interpreta "V1,W1,...,V5,W5". Devuelve true si trae los 10 enteros (y entonces actualiza el
// paquete); si no, el paquete queda como estaba.
bool parseLine(const char *data) {
  int v1, w1, v2, w2, v3, w3, v4, w4, v5, w5;

  int leidos = sscanf(
    data,
    "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
    &v1, &w1,
    &v2, &w2,
    &v3, &w3,
    &v4, &w4,
    &v5, &w5
  );

  if (leidos != 10) {
    return false;
  }

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

  return true;
}

// Lee todo lo que haya en el serie sin bloquear. Devuelve true si llegó al menos una línea válida
// (el paquete queda con la ÚLTIMA: si se acumularon varias, las viejas ya no sirven).
bool readSerialLines() {
  bool gotValid = false;

  while (Serial.available() > 0) {
    int c = Serial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      lineBuffer[lineLength] = '\0';
      if (lineLength > 0) {
        if (parseLine(lineBuffer)) {
          gotValid = true;
          linesOk++;
        } else {
          linesBad++;
        }
      }
      lineLength = 0;
      continue;
    }

    if (lineLength < sizeof(lineBuffer) - 1) {
      lineBuffer[lineLength++] = (char)c;
    } else {
      // Línea demasiado larga: se descarta entera.
      lineLength = 0;
      linesBad++;
    }
  }

  return gotValid;
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
  WiFi.disconnect();

  // Canal fijo, el mismo del robot (antes quedaba el que trae el modo STA por defecto, que hoy
  // también es el 1), y la radio sin ahorro de energía.
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

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
  lastValidLineTime = millis();

  Serial.println("Base station lista (reenvio inmediato).");
  Serial.println("Formato serial (v en mm/s, w en grados/s):");
  Serial.println("V1,W1,V2,W2,V3,W3,V4,W4,V5,W5");
  Serial.println("Ejemplo (robot 1 avanza recto a 500 mm/s):");
  Serial.println("500,0,0,0,0,0,0,0,0,0");
  if (SERIAL_TIMEOUT_MS > 0) {
    Serial.printf("Sin lineas validas por %lu ms, manda ceros.\n", SERIAL_TIMEOUT_MS);
  }
}

// ==========================================
//                  LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  if (readSerialLines()) {
    lastValidLineTime = now;
    serialTimedOut = false;
    sendPacket();   // en el acto: es el comando más nuevo
  }

  if (SERIAL_TIMEOUT_MS > 0 && !serialTimedOut && now - lastValidLineTime > SERIAL_TIMEOUT_MS) {
    // El PC dejó de mandar: ceros a todos, en el acto, y se siguen repitiendo.
    clearPacketCommands();
    serialTimedOut = true;
    sendPacket();
    Serial.println("Serie mudo: mando ceros.");
  }

  if (millis() - lastSendTime >= KEEPALIVE_MS) {
    sendPacket();
  }

  if (REPORT_MS > 0 && now - lastReport >= REPORT_MS) {
    lastReport = now;
    Serial.printf("lineas ok %lu, invalidas %lu, envios %lu (errores %lu)\n",
                  linesOk, linesBad, sends, sendErrors);
  }
}

// ============================================================================
//  comm_test_common.h
//
//  COPIA IDENTICA en comm_test_base/ y comm_test_robot/. Arduino IDE no deja
//  incluir archivos fuera de la carpeta del sketch, por eso se duplica: si
//  cambias uno, copia el cambio al otro (se comprueba con `cmp`).
//
//  Contiene: el protocolo de prueba, el formato del paquete de PRODUCCION
//  (copia de communication.cpp) y una capa de compatibilidad para que los
//  mismos sketches compilen en el core Arduino-ESP32 2.x y en el 3.x.
// ============================================================================
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <esp_arduino_version.h>

// ----------------------------------------------------------------------------
//  Protocolo de PRUEBA (PING de la base -> PONG del robot)
// ----------------------------------------------------------------------------
#define CT_TEST_MAGIC       0xC7
#define CT_TYPE_PING        1
#define CT_TYPE_PONG        2
#define CT_MAX_ROBOTS       8
#define CT_DEFAULT_CHANNEL  1
// Cada robot espera (id-1)*CT_SLOT_MS antes de contestar, para que los 5 no
// transmitan a la vez y se pisen entre ellos.
#define CT_SLOT_MS          10

typedef struct __attribute__((packed)) {
    uint8_t  magic;       // CT_TEST_MAGIC
    uint8_t  type;        // CT_TYPE_PING
    uint8_t  channel;     // canal en el que cree estar la base
    uint8_t  reserved;
    uint32_t seq;         // numero de PING
    uint32_t base_ms;     // millis() de la base
} TestPing;               // 12 bytes

typedef struct __attribute__((packed)) {
    uint8_t  magic;       // CT_TEST_MAGIC
    uint8_t  type;        // CT_TYPE_PONG
    uint8_t  robot_id;
    uint8_t  channel;     // canal en el que esta el robot
    uint32_t seq;         // numero del PING que contesta
    int8_t   rssi;        // RSSI con que el robot oyo ese PING
    uint8_t  rssi_valid;  // 0 si el core del robot no entrega RSSI
    uint16_t reserved;
    uint32_t ping_rx;     // PING recibidos en total
    uint32_t ping_lost;   // PING perdidos (huecos en el numero de secuencia)
    uint32_t cmd_ok;      // paquetes de PRODUCCION validos recibidos
    uint32_t cmd_bad;     // paquetes de PRODUCCION invalidos o desconocidos
    uint32_t tx_ok;       // PONG enviados con ACK de la base
    uint32_t tx_fail;     // PONG enviados SIN ACK
    uint32_t uptime_s;
} TestPong;               // 40 bytes

static_assert(sizeof(TestPing) == 12, "TestPing debe medir 12 bytes");
static_assert(sizeof(TestPong) == 40, "TestPong debe medir 40 bytes");

// ----------------------------------------------------------------------------
//  Paquete de PRODUCCION: copia EXACTA de communication.cpp (MODO_BASESTATION)
//  y de base_station_lineal_angulo.ino. Sirve para comprobar que el trafico
//  real llega bien, sin mover los motores.
// ----------------------------------------------------------------------------
#define PROD_MAGIC       0xA5
#define PROD_VERSION     1
#define PROD_NUM_ROBOTS  5

typedef struct __attribute__((packed)) {
    int16_t v_mms;    // velocidad lineal, mm/s
    int16_t w_degs;   // velocidad angular, grados/s
} ProdRobotCmd;

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint8_t  version;
    uint16_t seq;
    ProdRobotCmd robots[PROD_NUM_ROBOTS];
} ProdPacket;

static_assert(sizeof(ProdPacket) == 24, "ProdPacket debe medir 24 bytes (igual que en produccion)");

// ----------------------------------------------------------------------------
//  Capa de compatibilidad core 2.x / 3.x  (firmas verificadas contra los
//  esp_now.h oficiales de IDF 4.4.7, 5.0.7, 5.1.4, 5.3.2, 5.4.1 y 5.5)
//
//    IDF < 5.0 : recv(mac, data, len)               send(mac, status)
//    IDF 5.0-5.4: recv(esp_now_recv_info_t*, ...)    send(mac, status)
//    IDF >= 5.5: recv(esp_now_recv_info_t*, ...)    send(wifi_tx_info_t*, status)
// ---- COMPAT BEGIN ----
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  #define CT_SEND_CB_PARAMS  const wifi_tx_info_t *tx_info, esp_now_send_status_t status
#else
  #define CT_SEND_CB_PARAMS  const uint8_t *mac_addr, esp_now_send_status_t status
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  #define CT_RECV_CB_PARAMS  const esp_now_recv_info_t *rx_info, const uint8_t *data, int len
  #define CT_RX_SRC(rx_info)   ((const uint8_t *)(rx_info)->src_addr)
  #define CT_RX_RSSI(rx_info)  ((int8_t)(rx_info)->rx_ctrl->rssi)
  #define CT_HAS_RSSI          1
#else
  #define CT_RECV_CB_PARAMS  const uint8_t *rx_info, const uint8_t *data, int len
  #define CT_RX_SRC(rx_info)   ((const uint8_t *)(rx_info))
  #define CT_RX_RSSI(rx_info)  ((int8_t)0)
  #define CT_HAS_RSSI          0
#endif
// ---- COMPAT END ----

// ----------------------------------------------------------------------------
//  Utilidades de radio
// ----------------------------------------------------------------------------
static inline void ct_mac_to_str(const uint8_t *m, char *out /* >= 18 */) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

// Canal en el que esta REALMENTE la radio (no el que se pidio).
static inline uint8_t ct_get_channel() {
    uint8_t primary = 0;
    wifi_second_chan_t second;
    esp_wifi_get_channel(&primary, &second);
    return primary;
}

// Fija el canal con el mismo truco que usa el firmware del robot (modo
// promiscuo solo mientras se cambia) y comprueba que realmente se aplico.
static inline bool ct_set_channel(uint8_t ch) {
    esp_wifi_set_promiscuous(true);
    esp_err_t err = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    return err == ESP_OK && ct_get_channel() == ch;
}

// Potencia de TX actual en dBm (el driver la entrega en cuartos de dBm).
static inline float ct_get_tx_power_dbm() {
    int8_t quarter = 0;
    if (esp_wifi_get_max_tx_power(&quarter) != ESP_OK) return -99.0f;
    return quarter * 0.25f;
}

// Acepta 20, 17, 13, 8, 5, 2 (dBm). Devuelve false si el valor no es uno de esos.
static inline bool ct_set_tx_power_dbm(int dbm) {
    wifi_power_t p;
    switch (dbm) {
        case 20: p = WIFI_POWER_19_5dBm; break;
        case 17: p = WIFI_POWER_17dBm;   break;
        case 13: p = WIFI_POWER_13dBm;   break;
        case 8:  p = WIFI_POWER_8_5dBm;  break;
        case 5:  p = WIFI_POWER_5dBm;    break;
        case 2:  p = WIFI_POWER_2dBm;    break;
        default: return false;
    }
    return WiFi.setTxPower(p);
}

// Agrega un peer con canal 0 = "el canal actual", para que siga funcionando
// aunque el canal cambie.
static inline bool ct_add_peer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) return true;
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    return esp_now_add_peer(&peer) == ESP_OK;
}

static inline void ct_print_banner(const char *title) {
    Serial.println();
    Serial.println("==============================================================");
    Serial.println(title);
    Serial.println("==============================================================");
    Serial.printf("Arduino-ESP32 %d.%d.%d | IDF %s\n",
                  ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                  ESP_ARDUINO_VERSION_PATCH, ESP.getSdkVersion());
    Serial.printf("RSSI disponible en este core: %s\n", CT_HAS_RSSI ? "si" : "no (core 2.x)");
}

/**
 * PRUEBA DE COMUNICACION - BASE
 *
 * Sketch de diagnostico, independiente del firmware de produccion (no usa ni
 * modifica nada de src/ ni de include/). Se carga en la placa de la BASE; los
 * robots llevan comm_test_robot.
 *
 * QUE HACE
 *   - Cada 200 ms manda un PING por ESP-NOW (broadcast).
 *   - Cada robot contesta con un PONG que trae sus propios contadores.
 *   - Cada 2 s imprime una tabla con el estado de cada robot y, si algo falla,
 *     de que lado esta el problema.
 *   - Tambien manda (si esta activado) paquetes con el formato de PRODUCCION
 *     (magic 0xA5, version 1, 24 bytes) con velocidades en CERO, para que el
 *     robot de prueba compruebe que ese formato llega bien. Con velocidad cero
 *     es inofensivo aunque algun robot tenga el firmware de produccion.
 *
 * COMANDOS por el monitor serie (115200, terminar con Enter)
 *   h        ayuda
 *   c<n>     cambia el canal de la base, n = 1..13  (ej: c6). Los robots lo
 *            buscan solos.
 *   p<dBm>   potencia de TX: 20, 17, 13, 8, 5 o 2  (ej: p8)
 *   k        activa/desactiva los paquetes con formato de produccion
 *   r        reinicia las estadisticas
 *
 * Con Arduino IDE en ESP32-C3: "USB CDC On Boot: Enabled" para ver el serial.
 */
#include "comm_test_common.h"

// ---------------------------------------------------------------------------
//  Configuracion
// ---------------------------------------------------------------------------
#define EXPECTED_ROBOTS     5      // IDs 1..N que deberian responder
#define PING_INTERVAL_MS    200
#define PROD_INTERVAL_MS    50     // igual que la base de produccion
#define REPORT_INTERVAL_MS  2000
#define PING_HISTORY        16     // PING recordados para calcular el RTT

static const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------------------------------------------------------------------
//  Estado
// ---------------------------------------------------------------------------
static uint8_t  g_channel   = CT_DEFAULT_CHANNEL;
static bool     g_radioOk   = false;
static bool     g_sendProd  = true;

static uint32_t g_pingSeq        = 0;
static uint32_t g_pingsTotal     = 0;
static uint32_t g_pingsWin       = 0;
static uint16_t g_prodSeq        = 0;
static uint32_t g_sendErr        = 0;     // esp_now_send() devolvio error
static esp_err_t g_lastSendErr   = ESP_OK;
static volatile uint32_t g_cbOk   = 0;    // callback de envio: SUCCESS
static volatile uint32_t g_cbFail = 0;    // callback de envio: FAIL
static volatile uint32_t g_otherRx = 0;   // tramas ESP-NOW que no son PONG
static volatile uint32_t g_qDrop   = 0;   // PONG descartados por cola llena

static uint32_t g_pingSentUs[PING_HISTORY];
static uint32_t g_pingSentSeq[PING_HISTORY];

struct RobotStat {
    bool     seen;
    uint32_t lastSeenMs;
    uint32_t pongTotal;
    uint32_t pongWin;
    uint64_t rttSumUs;
    uint32_t rttN;
    uint32_t rttMaxUs;
    TestPong last;
    int8_t   rssiAtBase;
    uint32_t prevLost;
    uint32_t prevTxFail;
};
static RobotStat g_robots[CT_MAX_ROBOTS + 1];

// Cola entre el callback de recepcion (tarea WiFi) y loop().
struct PongItem {
    TestPong p;
    int8_t   rssi;
    uint32_t rxUs;
};
#define QN 16
static PongItem          g_q[QN];
static volatile uint8_t  g_qh = 0, g_qt = 0;

// ---------------------------------------------------------------------------
//  Callbacks ESP-NOW (hacen lo minimo: se imprime todo desde loop())
// ---------------------------------------------------------------------------
static void onSent(CT_SEND_CB_PARAMS) {
    if (status == ESP_NOW_SEND_SUCCESS) g_cbOk++; else g_cbFail++;
}

static void onRecv(CT_RECV_CB_PARAMS) {
    if (len != (int)sizeof(TestPong) ||
        data[0] != CT_TEST_MAGIC || data[1] != CT_TYPE_PONG) {
        g_otherRx++;
        return;
    }

    uint8_t next = (uint8_t)((g_qh + 1) % QN);
    if (next == g_qt) { g_qDrop++; return; }

    memcpy(&g_q[g_qh].p, data, sizeof(TestPong));
    g_q[g_qh].rssi = CT_RX_RSSI(rx_info);
    g_q[g_qh].rxUs = micros();
    __sync_synchronize();
    g_qh = next;
}

// ---------------------------------------------------------------------------
//  Envio
// ---------------------------------------------------------------------------
static void sendRaw(const void *buf, size_t len) {
    esp_err_t e = esp_now_send(BROADCAST_MAC, (const uint8_t *)buf, len);
    if (e != ESP_OK) { g_sendErr++; g_lastSendErr = e; }
}

static void sendPing() {
    TestPing p;
    memset(&p, 0, sizeof(p));
    p.magic   = CT_TEST_MAGIC;
    p.type    = CT_TYPE_PING;
    p.channel = g_channel;
    p.seq     = g_pingSeq;
    p.base_ms = millis();

    uint8_t slot = (uint8_t)(g_pingSeq % PING_HISTORY);
    g_pingSentSeq[slot] = g_pingSeq;
    g_pingSentUs[slot]  = micros();

    g_pingSeq++;
    g_pingsTotal++;
    g_pingsWin++;
    sendRaw(&p, sizeof(p));
}

static void sendProd() {
    ProdPacket p;
    memset(&p, 0, sizeof(p));      // todas las velocidades en 0
    p.magic   = PROD_MAGIC;
    p.version = PROD_VERSION;
    p.seq     = g_prodSeq++;
    sendRaw(&p, sizeof(p));
}

// ---------------------------------------------------------------------------
//  Proceso de PONG recibidos
// ---------------------------------------------------------------------------
static void handlePong(const PongItem &it) {
    uint8_t id = it.p.robot_id;
    if (id < 1 || id > CT_MAX_ROBOTS) { g_otherRx++; return; }

    RobotStat &r = g_robots[id];
    r.seen = true;
    r.lastSeenMs = millis();
    r.pongTotal++;
    r.pongWin++;
    memcpy(&r.last, &it.p, sizeof(TestPong));
    r.rssiAtBase = it.rssi;

    uint8_t slot = (uint8_t)(it.p.seq % PING_HISTORY);
    if (g_pingSentSeq[slot] == it.p.seq) {
        uint32_t rtt  = it.rxUs - g_pingSentUs[slot];
        uint32_t turn = (uint32_t)(id - 1) * CT_SLOT_MS * 1000UL;   // espera de turno del robot
        uint32_t net  = (rtt > turn) ? (rtt - turn) : 0;
        r.rttSumUs += net;
        r.rttN++;
        if (net > r.rttMaxUs) r.rttMaxUs = net;
    }
}

static void drainQueue() {
    while (g_qt != g_qh) {
        handlePong(g_q[g_qt]);
        g_qt = (uint8_t)((g_qt + 1) % QN);
    }
}

// ---------------------------------------------------------------------------
//  Informe
// ---------------------------------------------------------------------------
static void resetStats() {
    memset(g_robots, 0, sizeof(g_robots));
    g_pingsTotal = 0;
    g_pingsWin = 0;
    g_sendErr = 0;
    g_cbOk = 0;
    g_cbFail = 0;
    g_otherRx = 0;
    g_qDrop = 0;
}

static void printReport() {
    uint32_t now = millis();
    uint8_t radioCh = ct_get_channel();

    Serial.println();
    Serial.printf("==== t=%lus | canal pedido=%u radio=%u%s | TX=%.1f dBm | prod=%s ====\n",
                  (unsigned long)(now / 1000), g_channel, radioCh,
                  (radioCh == g_channel) ? "" : " <-- NO COINCIDEN",
                  ct_get_tx_power_dbm(), g_sendProd ? "ON" : "OFF");
    Serial.printf("PING enviados=%lu | esp_now_send con error=%lu%s | callback envio ok=%lu fail=%lu | otras tramas=%lu | cola llena=%lu\n",
                  (unsigned long)g_pingsTotal, (unsigned long)g_sendErr,
                  g_sendErr ? (String(" (") + esp_err_to_name(g_lastSendErr) + ")").c_str() : "",
                  (unsigned long)g_cbOk, (unsigned long)g_cbFail,
                  (unsigned long)g_otherRx, (unsigned long)g_qDrop);

    Serial.println("ID ESTADO         RESP    RTT ms (prom/max)  RSSI robot<-base  RSSI base<-robot  CANAL  PING rx/perd  CMD ok/mal  ACK ok/fail  VISTO   NOTA");

    uint8_t answering = 0;
    uint8_t shown = 0;

    for (uint8_t id = 1; id <= CT_MAX_ROBOTS; id++) {
        RobotStat &r = g_robots[id];
        if (id > EXPECTED_ROBOTS && !r.seen) continue;
        shown++;

        unsigned pct = g_pingsWin ? (unsigned)((r.pongWin * 100UL) / g_pingsWin) : 0;
        if (pct > 100) pct = 100;

        const char *state;
        if (!r.seen)            state = "SIN RESPUESTA";
        else if (r.pongWin == 0) state = "PERDIDO";
        else if (pct >= 90)      state = "OK";
        else if (pct >= 50)      state = "INESTABLE";
        else                     state = "MUY MALO";
        if (r.pongWin > 0) answering++;

        char rtt[20] = "-";
        if (r.rttN) snprintf(rtt, sizeof(rtt), "%.1f/%.1f",
                             (double)(r.rttSumUs / r.rttN) / 1000.0, r.rttMaxUs / 1000.0);

        char rssiRobot[10] = "-", rssiBase[10] = "-", chan[6] = "-", pings[16] = "-",
             cmd[16] = "-", ack[16] = "-", seen[12] = "nunca", note[48] = "";

        const char *noteText = "";
        if (r.seen) {
            if (r.last.rssi_valid) snprintf(rssiRobot, sizeof(rssiRobot), "%d dBm", r.last.rssi);
            else                   snprintf(rssiRobot, sizeof(rssiRobot), "n/d");
#if CT_HAS_RSSI
            snprintf(rssiBase, sizeof(rssiBase), "%d dBm", r.rssiAtBase);
#else
            snprintf(rssiBase, sizeof(rssiBase), "n/d");
#endif
            snprintf(chan, sizeof(chan), "%u", r.last.channel);
            snprintf(pings, sizeof(pings), "%lu/%lu", (unsigned long)r.last.ping_rx, (unsigned long)r.last.ping_lost);
            snprintf(cmd, sizeof(cmd), "%lu/%lu", (unsigned long)r.last.cmd_ok, (unsigned long)r.last.cmd_bad);
            snprintf(ack, sizeof(ack), "%lu/%lu", (unsigned long)r.last.tx_ok, (unsigned long)r.last.tx_fail);
            snprintf(seen, sizeof(seen), "%.1fs", (now - r.lastSeenMs) / 1000.0);

            // Cuanto crecieron en esta ventana los contadores de falla del robot.
            uint32_t dLost = (r.last.ping_lost >= r.prevLost)  ? r.last.ping_lost - r.prevLost  : 0;
            uint32_t dTx   = (r.last.tx_fail   >= r.prevTxFail) ? r.last.tx_fail   - r.prevTxFail : 0;
            r.prevLost = r.last.ping_lost;
            r.prevTxFail = r.last.tx_fail;

            if (r.last.channel != radioCh)                     noteText = "CANAL DISTINTO al de la base";
            else if (pct < 90 && dLost > 0 && dTx == 0)        noteText = "pierde PING (base->robot)";
            else if (pct < 90 && dTx > 0 && dLost == 0)        noteText = "PONG sin ACK (robot->base)";
            else if (pct < 90 && dTx > 0 && dLost > 0)         noteText = "pierde en ambos sentidos";
            else if (r.pongWin == 0)                           noteText = "dejo de contestar";
        }
        snprintf(note, sizeof(note), "%s", noteText);

        Serial.printf("%-2u %-13s %3lu/%-3lu %-18s %-17s %-17s %-6s %-13s %-11s %-12s %-7s %s\n",
                      id, state, (unsigned long)r.pongWin, (unsigned long)g_pingsWin, rtt,
                      rssiRobot, rssiBase, chan, pings, cmd, ack, seen, note);
    }
    (void)shown;

    // Diagnostico global, en una linea.
    if (g_sendErr > 0) {
        Serial.println(">> La BASE falla al transmitir (esp_now_send con error): problema de la radio de la base.");
    } else if (answering == 0) {
        Serial.println(">> NINGUN robot contesta. Revisa que lleven comm_test_robot y esten encendidos; prueba otro canal (c6), menos potencia (p8) o reiniciar la base.");
    } else if (answering < EXPECTED_ROBOTS) {
        Serial.println(">> Contestan solo algunos robots: el enlace base<->robot funciona, el fallo es de los que no contestan.");
    } else {
        Serial.println(">> Todos los robots esperados contestan: el enlace base<->robot funciona.");
    }

    // Reinicia la ventana.
    g_pingsWin = 0;
    for (uint8_t id = 1; id <= CT_MAX_ROBOTS; id++) {
        g_robots[id].pongWin = 0;
        g_robots[id].rttSumUs = 0;
        g_robots[id].rttN = 0;
        g_robots[id].rttMaxUs = 0;
    }
}

// ---------------------------------------------------------------------------
//  Comandos por serie
// ---------------------------------------------------------------------------
static void printHelp() {
    Serial.println("Comandos: h ayuda | c<n> canal 1..13 | p<dBm> 20/17/13/8/5/2 | k paquetes de produccion on/off | r reiniciar estadisticas");
}

static void handleCommand(const char *s) {
    while (*s == ' ') s++;
    if (!*s) return;
    int val = atoi(s + 1);

    switch (s[0]) {
        case 'h': case '?':
            printHelp();
            break;
        case 'c':
            if (val < 1 || val > 13) { Serial.println("Canal invalido (1..13)."); break; }
            g_channel = (uint8_t)val;
            Serial.printf("Canal %u -> %s (radio en %u)\n", g_channel,
                          ct_set_channel(g_channel) ? "aplicado" : "ERROR al aplicar",
                          ct_get_channel());
            break;
        case 'p':
            Serial.printf("Potencia %d dBm -> %s (ahora %.1f dBm)\n", val,
                          ct_set_tx_power_dbm(val) ? "aplicada" : "valor invalido (usa 20,17,13,8,5,2)",
                          ct_get_tx_power_dbm());
            break;
        case 'k':
            g_sendProd = !g_sendProd;
            Serial.printf("Paquetes de produccion: %s\n", g_sendProd ? "ON" : "OFF");
            break;
        case 'r':
            resetStats();
            Serial.println("Estadisticas reiniciadas.");
            break;
        default:
            Serial.println("Comando desconocido. 'h' para ayuda.");
    }
}

static void pollSerial() {
    static char buf[24];
    static uint8_t n = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            buf[n] = 0;
            handleCommand(buf);
            n = 0;
        } else if (n < sizeof(buf) - 1) {
            buf[n++] = c;
        }
    }
}

// ---------------------------------------------------------------------------
//  Arduino
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    delay(200);

    ct_print_banner("PRUEBA DE COMUNICACION - BASE");

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    bool chOk = ct_set_channel(g_channel);
    Serial.printf("MAC de la base: %s\n", WiFi.macAddress().c_str());
    Serial.printf("Canal pedido=%u, radio=%u -> %s\n", g_channel, ct_get_channel(), chOk ? "OK" : "ERROR al fijar el canal");

    esp_err_t e = esp_now_init();
    if (e != ESP_OK) {
        Serial.printf("ERROR: esp_now_init() fallo (%s). La radio no arranca; no se puede seguir.\n", esp_err_to_name(e));
        while (true) { delay(2000); Serial.println("esp_now_init() fallo: revisa la placa / reinicia."); }
    }
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    if (!ct_add_peer(BROADCAST_MAC)) Serial.println("ERROR: no se pudo agregar el peer broadcast.");
    g_radioOk = true;

    Serial.printf("TX = %.1f dBm | PING cada %d ms | esperando %d robots\n",
                  ct_get_tx_power_dbm(), PING_INTERVAL_MS, EXPECTED_ROBOTS);
    printHelp();
}

void loop() {
    static uint32_t lastPing = 0, lastProd = 0, lastReport = 0;
    uint32_t now = millis();

    pollSerial();
    drainQueue();

    if (!g_radioOk) return;

    if (now - lastPing >= PING_INTERVAL_MS) { lastPing = now; sendPing(); }
    if (g_sendProd && now - lastProd >= PROD_INTERVAL_MS) { lastProd = now; sendProd(); }
    if (now - lastReport >= REPORT_INTERVAL_MS) { lastReport = now; printReport(); }
}

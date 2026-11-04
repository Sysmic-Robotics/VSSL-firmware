/**
 * PRUEBA DE COMUNICACION - ROBOT
 *
 * Sketch de diagnostico, independiente del firmware de produccion (no usa ni
 * modifica nada de src/ ni de include/). Se carga en el ESP32-C3 del robot; la
 * base lleva comm_test_base. NO mueve los motores: sus pines se dejan en LOW.
 *
 * ANTES DE CARGARLO: poner el ROBOT_ID de este robot (1..5) aqui abajo.
 *
 * QUE HACE
 *   - Escucha el PING de la base y contesta con un PONG (unicast) que lleva sus
 *     contadores: PING recibidos/perdidos, PONG con y sin ACK, RSSI, canal.
 *   - Tambien reconoce el paquete de PRODUCCION (magic 0xA5, version 1,
 *     24 bytes) y lo decodifica sin actuar. Sirve para probar el trafico real:
 *     carga en la base tu base_station_lineal_angulo y mira aqui si llega.
 *   - Si no oye nada durante 3 s, recorre los canales 1..13 buscando a la base
 *     y avisa en cual la encontro. Asi se detecta una diferencia de canal.
 *
 * COMANDOS por el monitor serie (115200, terminar con Enter)
 *   h        ayuda
 *   c<n>     fija el canal n (1..13) y desactiva la busqueda automatica
 *   a        reactiva la busqueda automatica
 *   p<dBm>   potencia de TX: 20, 17, 13, 8, 5 o 2  (ej: p8)
 *   r        reinicia los contadores
 *
 * Con Arduino IDE en ESP32-C3: "USB CDC On Boot: Enabled" para ver el serial.
 */
#include "comm_test_common.h"

// ---------------------------------------------------------------------------
//  Configuracion
// ---------------------------------------------------------------------------
#ifndef ROBOT_ID                   // tambien se puede pasar con -DROBOT_ID=3
#define ROBOT_ID            1      // <<< CAMBIAR en cada robot (1..5)
#endif
#define AUTO_SCAN           1     // 1 = buscar la base por canales si se pierde
#define LOST_TIMEOUT_MS     3000   // silencio tras el cual se empieza a buscar
#define SCAN_DWELL_MS       450    // tiempo en cada canal (el PING sale cada 200 ms)
#define STATUS_INTERVAL_MS  2000

// Pines del DRV8833 (los mismos de include/config.h). Solo se dejan en LOW para
// que los motores no se muevan mientras se hace la prueba.
static const uint8_t MOTOR_PINS[] = {5, 6, 7, 10};

static_assert(ROBOT_ID >= 1 && ROBOT_ID <= CT_MAX_ROBOTS, "ROBOT_ID debe estar entre 1 y 8");

// ---------------------------------------------------------------------------
//  Estado
// ---------------------------------------------------------------------------
static uint8_t  g_channel      = CT_DEFAULT_CHANNEL;
static bool     g_scanning     = false;
static bool     g_userLocked   = false;     // canal fijado a mano
static bool     g_everHeard    = false;
static uint32_t g_lastHeardMs  = 0;
static uint32_t g_bootMs       = 0;
static uint32_t g_lastHopMs    = 0;

static uint8_t  g_baseMac[6]   = {0};
static bool     g_haveBase     = false;

// PING
static bool     g_seqValid     = false;
static uint32_t g_lastSeq      = 0;
static uint32_t g_pingRx       = 0;
static uint32_t g_pingLost     = 0;
static int8_t   g_lastRssi     = 0;
static bool     g_lastRssiOk   = false;

// PONG pendiente de enviar
static bool     g_replyPending = false;
static uint32_t g_replyAtMs    = 0;
static uint32_t g_replySeq     = 0;

// Resultado de enviar PONG
static volatile uint32_t g_txOk   = 0;
static volatile uint32_t g_txFail = 0;
static uint32_t g_txErr           = 0;
static esp_err_t g_lastTxErr      = ESP_OK;

// Paquetes de produccion
static uint32_t g_cmdOk        = 0;
static uint32_t g_cmdBad       = 0;
static uint32_t g_unknown      = 0;
static uint32_t g_cmdWinCount  = 0;
static uint32_t g_cmdWinStart  = 0;

static volatile uint32_t g_rxDrop = 0;

// Cola entre el callback de recepcion (tarea WiFi) y loop().
struct RxItem {
    uint8_t  src[6];
    int8_t   rssi;
    uint8_t  len;
    uint8_t  data[32];
};
#define QN 8
static RxItem            g_q[QN];
static volatile uint8_t  g_qh = 0, g_qt = 0;

// ---------------------------------------------------------------------------
//  Callbacks ESP-NOW
// ---------------------------------------------------------------------------
static void onSent(CT_SEND_CB_PARAMS) {
    if (status == ESP_NOW_SEND_SUCCESS) g_txOk++; else g_txFail++;
}

static void onRecv(CT_RECV_CB_PARAMS) {
    uint8_t next = (uint8_t)((g_qh + 1) % QN);
    if (next == g_qt) { g_rxDrop++; return; }

    RxItem &it = g_q[g_qh];
    memcpy(it.src, CT_RX_SRC(rx_info), 6);
    it.rssi = CT_RX_RSSI(rx_info);
    it.len  = (uint8_t)(len > 255 ? 255 : len);
    memcpy(it.data, data, len < (int)sizeof(it.data) ? len : sizeof(it.data));
    __sync_synchronize();
    g_qh = next;
}

// ---------------------------------------------------------------------------
//  Auxiliares
// ---------------------------------------------------------------------------
static void setRadioChannel(uint8_t ch) {
    g_channel = ch;
    if (!ct_set_channel(ch)) Serial.printf("ERROR al fijar el canal %u (radio en %u)\n", ch, ct_get_channel());
}

// Se llama cada vez que llega una trama con un magic conocido: la base esta
// viva en el canal actual.
static void markHeard(const uint8_t *srcMac) {
    uint32_t now = millis();
    bool wasScanning = g_scanning;

    g_lastHeardMs = now;
    g_everHeard = true;
    g_scanning = false;

    if (!g_haveBase || memcmp(g_baseMac, srcMac, 6) != 0) {
        memcpy(g_baseMac, srcMac, 6);
        g_haveBase = true;
        char mac[18];
        ct_mac_to_str(g_baseMac, mac);
        Serial.printf("[ROBOT %d] Base detectada: %s\n", ROBOT_ID, mac);
    }
    if (wasScanning) {
        Serial.printf("[ROBOT %d] >>> BASE ENCONTRADA en el canal %u (el canal por defecto es %u)%s\n",
                      ROBOT_ID, g_channel, CT_DEFAULT_CHANNEL,
                      g_channel == CT_DEFAULT_CHANNEL ? "" : "  <-- la base NO esta en el canal esperado");
    }
}

// ---------------------------------------------------------------------------
//  Quien esta transmitiendo en este canal (por direccion MAC de origen)
// ---------------------------------------------------------------------------
// Los robots de PRODUCCION aceptan un paquete valido venga de la base que
// venga. Si hay dos bases transmitiendo a la vez, los robots reciben comandos
// intercalados y se pisan. Esta tabla sirve para verlo.
#define KIND_NONE      0
#define KIND_PING      1
#define KIND_PROD      2     // paquete de produccion valido
#define KIND_PROD_BAD  3     // magic de produccion pero version/largo erroneos
#define KIND_OTHER     4
#define MAX_SRC        4

struct SrcStat {
    bool     used;
    uint8_t  mac[6];
    uint8_t  kind;
    uint16_t lastSeq;
    uint32_t win;      // tramas desde el ultimo informe
    uint32_t total;
};
static SrcStat  g_src[MAX_SRC];
static uint32_t g_srcOverflow = 0;

static SrcStat *srcFor(const uint8_t *mac) {
    for (uint8_t i = 0; i < MAX_SRC; i++) {
        if (g_src[i].used && memcmp(g_src[i].mac, mac, 6) == 0) return &g_src[i];
    }
    for (uint8_t i = 0; i < MAX_SRC; i++) {
        if (!g_src[i].used) {
            memset(&g_src[i], 0, sizeof(SrcStat));
            g_src[i].used = true;
            memcpy(g_src[i].mac, mac, 6);
            return &g_src[i];
        }
    }
    g_srcOverflow++;
    return nullptr;
}

static void resetCounters() {
    g_seqValid = false;
    g_pingRx = g_pingLost = 0;
    g_txOk = g_txFail = 0;
    g_txErr = 0;
    g_cmdOk = g_cmdBad = g_unknown = 0;
    g_rxDrop = 0;
    memset(g_src, 0, sizeof(g_src));
    g_srcOverflow = 0;
}

// ---------------------------------------------------------------------------
//  Proceso de tramas recibidas
// ---------------------------------------------------------------------------
static void handleTestPing(const RxItem &it) {
    TestPing p;
    memcpy(&p, it.data, sizeof(p));

    markHeard(it.src);
    g_pingRx++;
    g_lastRssi = it.rssi;
    g_lastRssiOk = CT_HAS_RSSI;

    if (g_seqValid) {
        if (p.seq > g_lastSeq + 1) {
            g_pingLost += p.seq - g_lastSeq - 1;
        } else if (p.seq <= g_lastSeq && g_lastSeq - p.seq > 5) {
            Serial.printf("[ROBOT %d] La base se reinicio (seq %lu -> %lu)\n",
                          ROBOT_ID, (unsigned long)g_lastSeq, (unsigned long)p.seq);
        }
    }
    g_lastSeq = p.seq;
    g_seqValid = true;

    g_replyPending = true;
    g_replySeq = p.seq;
    g_replyAtMs = millis() + (uint32_t)(ROBOT_ID - 1) * CT_SLOT_MS;
}

static void handleProd(const RxItem &it) {
    uint32_t now = millis();

    if (it.len == sizeof(ProdPacket) && it.data[1] == PROD_VERSION) {
        ProdPacket p;
        memcpy(&p, it.data, sizeof(p));
        markHeard(it.src);
        g_cmdOk++;
        g_cmdWinCount++;

        if (now - g_cmdWinStart >= 1000) {
            const ProdRobotCmd &c = p.robots[ROBOT_ID - 1];
            Serial.printf("[ROBOT %d] Paquete de PRODUCCION valido: seq=%u -> V=%d mm/s W=%d grados/s | %.1f paquetes/s\n",
                          ROBOT_ID, p.seq, c.v_mms, c.w_degs,
                          g_cmdWinCount * 1000.0f / (now - g_cmdWinStart));
            g_cmdWinStart = now;
            g_cmdWinCount = 0;
        }
        return;
    }

    // Magic de produccion pero version o tamano distintos: caso tipico de
    // base y robots compilados con versiones diferentes del protocolo.
    markHeard(it.src);
    g_cmdBad++;
    static uint32_t lastPrint = 0;
    if (now - lastPrint >= 1000) {
        lastPrint = now;
        Serial.printf("[ROBOT %d] PAQUETE DE PRODUCCION INCOMPATIBLE: version=%u (se espera %d), largo=%u (se espera %u) -> el robot de produccion lo DESCARTARIA\n",
                      ROBOT_ID, it.data[1], PROD_VERSION, it.len, (unsigned)sizeof(ProdPacket));
    }
}

static void handleUnknown(const RxItem &it) {
    g_unknown++;
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint >= 2000) {
        lastPrint = now;

        char mac[18];
        ct_mac_to_str(it.src, mac);

        // Volcado hexadecimal de la trama completa (hasta lo que se guarda).
        char hex[3 * sizeof(it.data) + 1];
        uint8_t n = it.len < sizeof(it.data) ? it.len : (uint8_t)sizeof(it.data);
        for (uint8_t i = 0; i < n; i++) snprintf(hex + 3 * i, 4, "%02X ", it.data[i]);
        hex[3 * n] = 0;

        Serial.printf("[ROBOT %d] Trama desconocida de %s: largo=%u, datos=%s\n",
                      ROBOT_ID, mac, it.len, hex);
    }
}

static void processRx() {
    while (g_qt != g_qh) {
        // Copia local y se libera la casilla de inmediato: la trama no puede
        // cambiar mientras se clasifica, y la cola no se llena mientras se
        // imprime por el puerto serie.
        RxItem it = g_q[g_qt];
        g_qt = (uint8_t)((g_qt + 1) % QN);

        SrcStat *src = srcFor(it.src);
        if (src) { src->win++; src->total++; }

        uint8_t kind;
        if (it.len == sizeof(TestPing) && it.data[0] == CT_TEST_MAGIC && it.data[1] == CT_TYPE_PING) {
            kind = KIND_PING;
            handleTestPing(it);
        } else if (it.data[0] == PROD_MAGIC) {
            bool ok = (it.len == sizeof(ProdPacket) && it.data[1] == PROD_VERSION);
            kind = ok ? KIND_PROD : KIND_PROD_BAD;
            if (src) src->lastSeq = (uint16_t)(it.data[2] | (it.data[3] << 8));
            handleProd(it);
        } else {
            kind = KIND_OTHER;
            handleUnknown(it);
        }
        if (src) src->kind = kind;
    }
}

// ---------------------------------------------------------------------------
//  Envio del PONG
// ---------------------------------------------------------------------------
static void maybeSendPong() {
    if (!g_replyPending || !g_haveBase) return;
    if ((int32_t)(millis() - g_replyAtMs) < 0) return;
    g_replyPending = false;

    if (!ct_add_peer(g_baseMac)) {
        g_txErr++;
        g_lastTxErr = ESP_FAIL;
        return;
    }

    TestPong p;
    memset(&p, 0, sizeof(p));
    p.magic      = CT_TEST_MAGIC;
    p.type       = CT_TYPE_PONG;
    p.robot_id   = ROBOT_ID;
    p.channel    = ct_get_channel();
    p.seq        = g_replySeq;
    p.rssi       = g_lastRssi;
    p.rssi_valid = g_lastRssiOk ? 1 : 0;
    p.ping_rx    = g_pingRx;
    p.ping_lost  = g_pingLost;
    p.cmd_ok     = g_cmdOk;
    p.cmd_bad    = g_cmdBad + g_unknown;
    p.tx_ok      = g_txOk;
    p.tx_fail    = g_txFail;
    p.uptime_s   = millis() / 1000;

    esp_err_t e = esp_now_send(g_baseMac, (const uint8_t *)&p, sizeof(p));
    if (e != ESP_OK) { g_txErr++; g_lastTxErr = e; }
}

// ---------------------------------------------------------------------------
//  Busqueda de la base por canales
// ---------------------------------------------------------------------------
static void scanTick() {
#if AUTO_SCAN
    if (g_userLocked) return;
    uint32_t now = millis();

    if (!g_scanning) {
        uint32_t since = g_everHeard ? (now - g_lastHeardMs) : (now - g_bootMs);
        if (since > LOST_TIMEOUT_MS) {
            g_scanning = true;
            g_lastHopMs = now;
            Serial.printf("[ROBOT %d] Sin senal de la base hace %lu ms: BUSCANDO en todos los canales...\n",
                          ROBOT_ID, (unsigned long)since);
        }
        return;
    }

    if (now - g_lastHopMs >= SCAN_DWELL_MS) {
        g_lastHopMs = now;
        uint8_t next = (uint8_t)(g_channel % 13 + 1);
        setRadioChannel(next);
        Serial.printf("[ROBOT %d] buscando... canal %u\n", ROBOT_ID, next);
    }
#endif
}

// ---------------------------------------------------------------------------
//  Estado periodico
// ---------------------------------------------------------------------------
static void printSources(uint32_t elapsedMs) {
    uint8_t prodSources = 0;

    for (uint8_t i = 0; i < MAX_SRC; i++) {
        SrcStat &s = g_src[i];
        if (!s.used) continue;

        char mac[18];
        ct_mac_to_str(s.mac, mac);

        const char *kind = s.kind == KIND_PING     ? "PING de prueba"
                         : s.kind == KIND_PROD     ? "PRODUCCION valida"
                         : s.kind == KIND_PROD_BAD ? "PRODUCCION INCOMPATIBLE"
                         :                           "otro";

        Serial.printf("             fuente %s: %5.1f tramas/s | %s | ultimo seq=%u | total=%lu\n",
                      mac, s.win * 1000.0f / elapsedMs, kind, s.lastSeq, (unsigned long)s.total);

        if ((s.kind == KIND_PROD || s.kind == KIND_PROD_BAD) && s.win > 0) prodSources++;
        s.win = 0;
    }

    if (prodSources > 1) {
        Serial.println("             >>> HAY MAS DE UNA BASE transmitiendo paquetes de produccion en este canal.");
        Serial.println("             >>> Los robots de produccion aceptan paquetes de CUALQUIERA: los comandos se pisan.");
    }
    if (g_srcOverflow) {
        Serial.printf("             (hay mas de %d fuentes distintas; %lu tramas no se contaron)\n",
                      MAX_SRC, (unsigned long)g_srcOverflow);
    }
}

static void statusTick() {
    static uint32_t last = 0;
    uint32_t now = millis();
    if (now - last < STATUS_INTERVAL_MS) return;
    uint32_t elapsed = now - last;
    last = now;

    char mac[18] = "-";
    if (g_haveBase) ct_mac_to_str(g_baseMac, mac);

    const char *state = g_scanning ? "BUSCANDO base"
                      : (!g_everHeard && now - g_bootMs < LOST_TIMEOUT_MS) ? "esperando base"
                      : (g_userLocked && (!g_everHeard || now - g_lastHeardMs > LOST_TIMEOUT_MS)) ? "SIN SENAL (canal fijo)"
                      : "ENLAZADO";

    char heard[16] = "nunca";
    if (g_everHeard) snprintf(heard, sizeof(heard), "hace %.1fs", (now - g_lastHeardMs) / 1000.0);

    char rssi[12] = "n/d";
    if (g_lastRssiOk && g_pingRx) snprintf(rssi, sizeof(rssi), "%d dBm", g_lastRssi);

    Serial.printf("[ROBOT %d] %s | canal %u | base %s (%s) | PING rx=%lu perdidos=%lu RSSI=%s | PONG ack=%lu sin-ack=%lu err-envio=%lu%s | PRODUCCION ok=%lu mal=%lu | ajenas=%lu | cola llena=%lu\n",
                  ROBOT_ID, state, ct_get_channel(), mac, heard,
                  (unsigned long)g_pingRx, (unsigned long)g_pingLost, rssi,
                  (unsigned long)g_txOk, (unsigned long)g_txFail, (unsigned long)g_txErr,
                  g_txErr ? (String(" (") + esp_err_to_name(g_lastTxErr) + ")").c_str() : "",
                  (unsigned long)g_cmdOk, (unsigned long)g_cmdBad,
                  (unsigned long)g_unknown, (unsigned long)g_rxDrop);
    printSources(elapsed);
}

// ---------------------------------------------------------------------------
//  Comandos por serie
// ---------------------------------------------------------------------------
static void printHelp() {
    Serial.println("Comandos: h ayuda | c<n> fija canal 1..13 | a busqueda automatica | p<dBm> 20/17/13/8/5/2 | r reiniciar contadores");
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
            g_userLocked = true;
            g_scanning = false;
            setRadioChannel((uint8_t)val);
            Serial.printf("Canal fijado en %u (busqueda automatica OFF; 'a' para reactivarla)\n", ct_get_channel());
            break;
        case 'a':
            g_userLocked = false;
            g_lastHeardMs = millis();
            Serial.println("Busqueda automatica ON.");
            break;
        case 'p':
            Serial.printf("Potencia %d dBm -> %s (ahora %.1f dBm)\n", val,
                          ct_set_tx_power_dbm(val) ? "aplicada" : "valor invalido (usa 20,17,13,8,5,2)",
                          ct_get_tx_power_dbm());
            break;
        case 'r':
            resetCounters();
            Serial.println("Contadores reiniciados.");
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
    // Lo primero: motores quietos. Sin esto los pines del driver quedan
    // flotando al arrancar y la rueda puede moverse sola.
    for (uint8_t pin : MOTOR_PINS) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);
    delay(200);

    ct_print_banner("PRUEBA DE COMUNICACION - ROBOT");
    Serial.printf("ROBOT_ID = %d | motores: pines en LOW (no se mueven)\n", ROBOT_ID);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    bool chOk = ct_set_channel(g_channel);
    Serial.printf("MAC del robot: %s\n", WiFi.macAddress().c_str());
    Serial.printf("Canal pedido=%u, radio=%u -> %s\n", g_channel, ct_get_channel(), chOk ? "OK" : "ERROR al fijar el canal");

    esp_err_t e = esp_now_init();
    if (e != ESP_OK) {
        Serial.printf("ERROR: esp_now_init() fallo (%s). La radio no arranca; no se puede seguir.\n", esp_err_to_name(e));
        while (true) { delay(2000); Serial.println("esp_now_init() fallo: revisa la placa / reinicia."); }
    }
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onRecv);

    Serial.printf("TX = %.1f dBm | espero PING de prueba y paquetes de produccion (magic 0x%02X, version %d, %u bytes)\n",
                  ct_get_tx_power_dbm(), PROD_MAGIC, PROD_VERSION, (unsigned)sizeof(ProdPacket));
    printHelp();

    g_bootMs = millis();
    g_cmdWinStart = g_bootMs;
}

void loop() {
    pollSerial();
    processRx();
    maybeSendPong();
    scanTick();
    statusTick();
}

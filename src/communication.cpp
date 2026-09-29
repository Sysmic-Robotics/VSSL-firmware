#include "communication.h"
#include "debug.h"
#include "mpu.h"
#include <string.h>
#include <math.h>
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

// Estos tres valores y la estructura de abajo deben coincidir EXACTAMENTE con
// base_station_lineal_angulo.ino. Si se cambia uno, hay que cambiar el otro.
#define COMM_MAGIC 0xA5
#define COMM_VERSION 1
#define NUM_ROBOTS 5

// Consigna de velocidad del robot completo tal como la envía la estación base.
// OJO con las unidades: la velocidad angular viaja en GRADOS/s, mientras que
// internamente el firmware trabaja en mrad/s. La conversión se hace al recibir.
typedef struct __attribute__((packed)) {
    int16_t v_mms;    // velocidad lineal, mm/s
    int16_t w_degs;   // velocidad angular, grados/s
} RobotCommand;

typedef struct __attribute__((packed)) {
    uint8_t magic;
    uint8_t version;
    uint16_t seq;
    RobotCommand robots[NUM_ROBOTS];
} CommandPacket;

// grados/s -> mrad/s : x (PI/180) x 1000
#define DEG_S_TO_MRAD_S 17.453293f

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

    int32_t w_mrad_s = (int32_t)lroundf(packet.robots[idx].w_degs * DEG_S_TO_MRAD_S);

    g_Linear_MmPerSec = clampInt16(packet.robots[idx].v_mms, MAX_LINEAR_MM_S);
    g_Angular_MradPerSec = clampInt16(w_mrad_s, MAX_ANGULAR_MRAD_S);

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

// ==========================================
//     PROCESAMIENTO DEL JOYSTICK
// ==========================================
// El joystick de la app entrega dos ejes de -100 a 100. El objetivo es
// convertirlos en una consigna (v, w) limpia: sin saltos, sin pedir a las
// ruedas más de lo que pueden dar y sin arranques bruscos que hagan patinar.
// Una vez generada, la consigna entra al MISMO lazo de control que en modo
// base station, así que los encoders siguen garantizando que la velocidad
// pedida sea la velocidad real.

static float joyLinear = 0.0f;      // mm/s, ya suavizado por la rampa
static float joyAngular = 0.0f;     // mrad/s, ya suavizado por la rampa
static unsigned long lastJoystickUpdate = 0;

static bool headingHoldArmed = false;
static float headingTargetRad = 0.0f;

static void resetJoystickState() {
    joyLinear = 0.0f;
    joyAngular = 0.0f;
    lastJoystickUpdate = 0;
    headingHoldArmed = false;
}

// Zona muerta radial con reescalado: se mide el vector completo del stick,
// y al salir de la zona muerta la salida arranca desde cero en vez de dar
// un escalón. Así el robot no "salta" al despegar el dedo del centro.
static void applyRadialDeadzone(float *x, float *y) {
    float magnitude = sqrtf((*x) * (*x) + (*y) * (*y));

    if (magnitude <= JOYSTICK_DEADZONE) {
        *x = 0.0f;
        *y = 0.0f;
        return;
    }

    float scale = (magnitude - JOYSTICK_DEADZONE) / magnitude;
    *x *= scale;
    *y *= scale;
}

// Normaliza un eje a -1..1 y le aplica la curva expo.
static float shapeAxis(float axis) {
    float norm = constrain(axis / (100.0f - JOYSTICK_DEADZONE), -1.0f, 1.0f);
    float expo = constrain((float)JOYSTICK_EXPO, 0.0f, 1.0f);

    return (1.0f - expo) * norm + expo * norm * norm * norm;
}

// Rampa: acerca 'current' a 'target' como mucho maxStep por ciclo.
static float slewTowards(float current, float target, float maxStep) {
    if (maxStep <= 0.0f) return current;

    float delta = target - current;
    if (delta >  maxStep) return current + maxStep;
    if (delta < -maxStep) return current - maxStep;

    return target;
}

// Mezcla anti-saturación: si la combinación (v, w) le pediría a una rueda
// más de lo permitido, escala v y w JUNTOS. Recortar cada rueda por separado
// deformaría la curva; escalando en bloque el robot sigue exactamente la
// trayectoria pedida, solo que más lento.
static void limitWheelEnvelope(float *v_mm_s, float *w_mrad_s, float maxWheel_mm_s) {
    if (maxWheel_mm_s <= 0.0f) {
        *v_mm_s = 0.0f;
        *w_mrad_s = 0.0f;
        return;
    }

    float halfTrack_mm = WHEEL_TRACK_MM / 2.0f;
    float demand = fabsf(*v_mm_s) + fabsf((*w_mrad_s / 1000.0f) * halfTrack_mm);

    if (demand > maxWheel_mm_s) {
        float k = maxWheel_mm_s / demand;
        *v_mm_s *= k;
        *w_mrad_s *= k;
    }
}

// Mantención de rumbo. Mientras se avanza sin pedir giro, se memoriza el
// rumbo y se corrige el error de ángulo acumulado. El lazo interno solo
// regula velocidad angular: lleva w a cero, pero no devuelve los grados ya
// perdidos. Esto sí, y es lo que hace que el robot vaya recto de verdad.
static float applyHeadingHold(float v_mm_s, float w_mrad_s) {
#if JOYSTICK_HEADING_HOLD
    if (!isMPUReady()) {
        headingHoldArmed = false;
        return w_mrad_s;
    }

    bool wantsTurn = fabsf(w_mrad_s) > JOYSTICK_HEADING_ARM_ANGULAR_MRAD_S;
    bool moving = fabsf(v_mm_s) > JOYSTICK_HEADING_ARM_LINEAR_MM_S;

    // Si el piloto está girando, o el robot está detenido, el hold se
    // desarma y el rumbo memorizado se descarta.
    if (wantsTurn || !moving) {
        headingHoldArmed = false;
        return w_mrad_s;
    }

    // Primer ciclo yendo recto: se fija el rumbo a mantener.
    if (!headingHoldArmed) {
        headingTargetRad = getYawAngleRad();
        headingHoldArmed = true;
    }

    float error = headingTargetRad - getYawAngleRad();
    float correction = JOYSTICK_HEADING_KP_MRAD_PER_RAD * error;

    return constrain(correction,
                     -(float)JOYSTICK_HEADING_MAX_CORRECTION_MRAD_S,
                      (float)JOYSTICK_HEADING_MAX_CORRECTION_MRAD_S);
#else
    return w_mrad_s;
#endif
}

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

        unsigned long now = millis();
        float dt = (lastJoystickUpdate == 0)
                     ? 0.0f
                     : (now - lastJoystickUpdate) / 1000.0f;
        lastJoystickUpdate = now;
        dt = constrain(dt, 0.0f, 0.1f);

        // 1. Lectura cruda de los dos ejes del joystick (-100 a 100).
        float rawX = RemoteXY.Mando_X;   // izquierda/derecha -> giro
        float rawY = RemoteXY.Mando_Y;   // adelante/atrás    -> avance

        // 2. Zona muerta radial sobre el vector completo del stick.
        applyRadialDeadzone(&rawX, &rawY);

        // 3. Curva expo: control fino cerca del centro, tope intacto.
        float fwd  = shapeAxis(rawY);
        float turn = shapeAxis(rawX) * JOYSTICK_TURN_SIGN;

        // 4. Limitador de velocidad (0..1) y modo preciso con el botón.
        float limit = constrain((float)JOYSTICK_SPEED_LIMIT, 0.0f, 1.0f);
        if (RemoteXY.Boton_1) {
            limit *= constrain((float)JOYSTICK_PRECISION_FACTOR, 0.0f, 1.0f);
        }

        // La velocidad angular de un pivote a fondo se deriva de la velocidad
        // de rueda deseada: w = v_rueda / (ancho de vía / 2).
        float maxAngular_mrad_s = (JOYSTICK_PIVOT_WHEEL_MM_S / (WHEEL_TRACK_MM / 2.0f)) * 1000.0f;

        float vTarget = fwd  * JOYSTICK_MAX_LINEAR_MM_S * limit;
        float wTarget = turn * maxAngular_mrad_s         * limit;

        // 5. Que la mezcla nunca pida a una rueda más de lo permitido.
        limitWheelEnvelope(&vTarget, &wTarget, JOYSTICK_MAX_LINEAR_MM_S * limit);

        // 6. Rampas de aceleración: sin tirones, las ruedas no patinan y
        //    los encoders siguen midiendo la velocidad real del robot.
        joyLinear  = slewTowards(joyLinear,  vTarget, JOYSTICK_LINEAR_SLEW_MM_S2 * dt);
        joyAngular = slewTowards(joyAngular, wTarget, JOYSTICK_ANGULAR_SLEW_MRAD_S2 * dt);

        // 7. Mantención de rumbo con el giroscopio al ir recto.
        float wOut = applyHeadingHold(joyLinear, joyAngular);

        g_Linear_MmPerSec = clampInt16((int32_t)lroundf(joyLinear), MAX_LINEAR_MM_S);
        g_Angular_MradPerSec = clampInt16((int32_t)lroundf(wOut), MAX_ANGULAR_MRAD_S);

        markCommandReceived();

    } else {
        resetJoystickState();
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

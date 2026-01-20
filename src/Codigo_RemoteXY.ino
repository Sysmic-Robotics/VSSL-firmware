#include <Arduino.h> // ESTO SIEMPRE VA PRIMERO
#include <Encoder.h> // para los encoder uwu
// ==========================================
//        CONFIGURACIÓN PRINCIPAL
// ==========================================

// Comenta para usar Bluetooth, Descomenta para WIFI
#define MODO_BASESTATION 

// ID de este robot
#define MI_ROBOT_ID 1 

// ==========================================
//               PINES Y MOTORES
// ==========================================
#define C1_MI 0
#define C2_MI 1
#define C1_MD 2
#define C2_MD 3

// DRV8833
#define MOT_IN1_PIN 5   
#define MOT_IN2_PIN 6   
#define MOT_IN3_PIN 7   
#define MOT_IN4_PIN 10   

// PWM Settings
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;     

// Variables Puente (Funcionan para WiFi y BT)
float Input_X = 0;
float Input_Y = 0;

// ==========================================
//            BLOQUE 1: ESP-NOW (WIFI)
// ==========================================
#ifdef MODO_BASESTATION
  #include <esp_now.h>
  #include <WiFi.h>

  typedef struct struct_mensaje{
    int8_t x_robot_1;
    int8_t y_robot_1;

    int8_t x_robot_2;
    int8_t y_robot_2;

    int8_t x_robot_3;
    int8_t y_robot_3;

    int8_t x_robot_4;
    int8_t y_robot_4;

    int8_t x_robot_5;
    int8_t y_robot_5;
  } struct_mensaje;

  struct_mensaje misdatos;

  // CORRECCIÓN: Firma actualizada para ESP32 Core v3.0
  void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len){
    memcpy(&misdatos, incomingData, sizeof(misdatos));

    int8_t recibido_x = 0;
    int8_t recibido_y = 0;
    
    switch(MI_ROBOT_ID){
      case 1: 
        recibido_x = misdatos.x_robot_1;
        recibido_y = misdatos.y_robot_1;
        break;
      case 2: 
        recibido_x = misdatos.x_robot_2;
        recibido_y = misdatos.y_robot_2;
        break;
      case 3: 
        recibido_x = misdatos.x_robot_3;
        recibido_y = misdatos.y_robot_3;
        break;
      case 4: 
        recibido_x = misdatos.x_robot_4;
        recibido_y = misdatos.y_robot_4;
        break;
      case 5: 
        recibido_x = misdatos.x_robot_5;
        recibido_y = misdatos.y_robot_5;
        break;
      
      }

    Input_Y = recibido_y;
    Input_X = recibido_x;
  }

// ==========================================
//            BLOQUE 2: REMOTEXY (BT)
// ==========================================
#else
  #define REMOTEXY_MODE__ESP32CORE_BLE
  #include <BLEDevice.h>
  #define REMOTEXY_BLUETOOTH_NAME "RemoteXY"
  #include <RemoteXY.h>

  #pragma pack(push, 1)  
  uint8_t RemoteXY_CONF[] =   
   { 255,3,0,0,0,49,0,19,0,0,0,82,111,98,111,116,105,116,111,0,
    31,2,106,200,200,84,1,1,2,0,5,54,136,49,49,147,35,45,45,32,
    1,24,31,1,9,145,31,31,16,50,24,24,0,2,31,0 };
  
  struct {
    int8_t Mando_X; 
    int8_t Mando_Y; 
    uint8_t Boton_1; 
    uint8_t connect_flag; 
  } RemoteXY;
  #pragma pack(pop)
#endif    

// ==========================================
//           FUNCIONES MATEMÁTICAS
// ==========================================
inline float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
inline int dutyFromNorm(float v, int maxDuty){ v=fabsf(v); v=clampf(v,0,1); return (int)(v*maxDuty); }
float deadzone(float v, float dz){ float a=fabsf(v); if(a<dz) return 0; float s=(a-dz)/(1.0f-dz); return v>=0?s:-s; }
float expo(float v, float e){ return (1.0f-e)*v + e*v*fabsf(v); }

void driveOne(float v, uint8_t pinX, uint8_t pinY, int maxDuty){
  if (v > 0){
    analogWrite(pinX, dutyFromNorm(v, maxDuty));
    analogWrite(pinY, 0);
  } else if (v < 0){
    analogWrite(pinX, 0);
    analogWrite(pinY, dutyFromNorm(v, maxDuty));
  } else {
    analogWrite(pinX, 0);
    analogWrite(pinY, 0); 
  }
}

void setWheels(float left, float right, int maxDuty){
  driveOne(left,  MOT_IN1_PIN, MOT_IN2_PIN, maxDuty);  
  driveOne(right, MOT_IN3_PIN, MOT_IN4_PIN, maxDuty);  
}

// ==========================================
//                  SETUP
// ==========================================
void setup() {
  Serial.begin(115200); // 921600 es muy rapido, mejor estandar
   
  pinMode(C1_MD, INPUT);
  pinMode(C2_MD, INPUT);
  pinMode(C1_MI, INPUT);
  pinMode(C2_MI, INPUT);

  pinMode(MOT_IN1_PIN, OUTPUT);
  pinMode(MOT_IN2_PIN, OUTPUT);
  pinMode(MOT_IN3_PIN, OUTPUT);
  pinMode(MOT_IN4_PIN, OUTPUT);

  analogWriteResolution(MOT_IN1_PIN, PWM_RES);
  analogWriteResolution(MOT_IN2_PIN, PWM_RES);
  analogWriteResolution(MOT_IN3_PIN, PWM_RES);
  analogWriteResolution(MOT_IN4_PIN, PWM_RES);

  analogWriteFrequency(MOT_IN1_PIN, PWM_FREQ);
  analogWriteFrequency(MOT_IN2_PIN, PWM_FREQ);
  analogWriteFrequency(MOT_IN3_PIN, PWM_FREQ);
  analogWriteFrequency(MOT_IN4_PIN, PWM_FREQ);

  #ifdef MODO_BASESTATION
    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK){
      Serial.println("Error ESP-NOW");
      return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESCUCHANDO POR ESP-NOW...");
    Serial.print("ROBOT ID: "); Serial.println(MI_ROBOT_ID);
    Serial.print("MAC: "); Serial.println(WiFi.macAddress());

  #else
      RemoteXY_Init ();
      Serial.println("MODO BLUETOOTH INICIADO");
  #endif
}

// ==========================================
//                  LOOP
// ==========================================
void loop() {
  
  // 1. OBTENER DATOS
  #ifdef MODO_BASESTATION
    delay(10); // Faltaba el punto y coma
  #else
    RemoteXY_Handler ();
    Input_X = RemoteXY.Mando_X;
    Input_Y = RemoteXY.Mando_Y; // Corregido INput -> Input
  #endif

  // 2. PROCESAR DATOS (Usamos Input_X e Input_Y, NO RemoteXY directo)
  // Asi funciona en ambos modos sin error de compilacion
  
  float x = clampf(Input_X / 100.0f, -1.0f, 1.0f);
  float y = clampf(Input_Y / 100.0f, -1.0f, 1.0f);

  x = deadzone(x, 0.08f);
  y = deadzone(y, 0.08f);

  x = expo(x, 0.25f);
  y = expo(y, 0.20f);

  float left  = y + x;
  float right = y - x;

  float m = fmaxf(fabsf(left), fabsf(right));
  if (m > 1.0f){ left /= m; right /= m; }

  setWheels(left, right, 1023);
}
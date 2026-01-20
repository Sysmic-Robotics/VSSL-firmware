#include <esp_now.h>
#include <WiFi.h>

//VARIABLES
#define NUM_ROBOTS 5

// Direccion de broadcast
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Estructura del dato a enviar (2 bytes por cada robot) para 5 serian 10 bytes o si se utiliza uint32_t(4bytes) 20 bytes y creo que el total es de 250 BYTES HErMANO ES MAS QUE LA XUXA Q WEAAAA
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

struct_mensaje mimensaje;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nEstado del envío: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Enviado" : "Fallido");
}


void setup() {
  Serial.begin(115200); // Baudeos
  WiFi.mode(WIFI_STA);  // Iniciar el wifi

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }


  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Fallo al agregar el peer de Broadcast");
    return;
  }
}

void loop() {
  mimensaje.x_robot_1=0;
  mimensaje.y_robot_1=60;

  mimensaje.x_robot_2=0;
  mimensaje.y_robot_2=0;
  
  esp_now_send(broadcastAddress, (uint8_t *) &mimensaje, sizeof(mimensaje));

  Serial.println("Enviando comando de prueba..");
  delay(100);

}

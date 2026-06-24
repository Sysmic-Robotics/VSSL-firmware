# Firmware para el "Freddy" (VSSS)
El firmware está diseñado para un microcontrolador ESP32-C3 Supermini. Este está conectado a una batería de 11.1V mediante un conversor DC-DC que baja el voltaje a 5V para alimentar la lógica. El microcontrolador se encarga de enviarle las señales a un driver de motor DRV8833, el cual controlará cada motor. También está implementado el uso de una MPU (giroscopio/acelerómetro) que por el momento no tiene un objetivo definido en la estrategia.

## Entorno de Desarrollo (PlatformIO)
Este proyecto está construido y gestionado utilizando **PlatformIO** (se recomienda usar su extensión para Visual Studio Code). 
* La estructura modular (archivos `.cpp` y `.h`) está pensada específicamente para este entorno.
* Solo necesitas abrir la carpeta del proyecto en VS Code con PlatformIO instalado. El archivo `platformio.ini` se encargará automáticamente de configurar la placa (ESP32-C3) y descargar todas las librerías necesarias.

## Configuración de modos (config.h)
El comportamiento del robot se ajusta comentando (`//`) o descomentando las definiciones en el código.

* **Para controlar por celular (RemoteXY):**
  * Comentar la línea: `//#define MODO_BASESTATION`

* **Para controlar mediante la Base station:**
  * Descomentar la línea: `#define MODO_BASESTATION`

> **PARA EL MODO BASESTATION ES IMPORTANTE MODIFICAR LA ID DEL ROBOT (`MI_ROBOT_ID`).**

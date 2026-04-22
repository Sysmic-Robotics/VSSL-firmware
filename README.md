# Firmware para el "Pastilla" (VSSS)
El firmware está diseñado para un microcontrolador ESP32-C3 Supermini. Este está conectado a una bateria de 11.1V mediante un conversor DC-DC que baja el voltaje a 5V para alimentar la lógica. El microcontrolador se encarga de enviarle las señales a un driver de motor DRV8833, el cual controlará cada motor. También está implementado el uso de una MPU (gisroscopio/acelerómetro) que por el momento no tiene un objetico definido en la estrategia.

## Configuración de modos (config.h)
El comportamiento del robot se ajusta comentando (//) o descomentando las definiciones en el código.
- Para controlar por celular (RemoteXY):
  * Comentar la línea: //#define MODO_BASESTATION
  * Comentar la línea: //#define CONTROL_SOFTWARE
- Para controlar mediante la Base station (Mando manual):
  * Descomentar la línea: #define MODO_BASESTATION
  * Comentar la línea: //#define CONTROL_SOFTWARE
- Para controlar mediante Software:
  * Descomentar la línea: #define MODO_BASESTATION
  * Descomentar la líena: #define CONTROL_SOFTWARE
 
  
***PARA EL MODO BASESTATION ES IMPORTANTE MODIFICAR LA ID DEL ROBOT***

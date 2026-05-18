#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

// comentar para apagar los mensajes de debug
//#define MODO_DEBUG 1

#ifdef MODO_DEBUG
    #define DEBUG_INIT(speed) Serial.begin(speed)
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)

    #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)

#else
    #define DEBUG_INIT(speed)
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(...)
#endif
#endif
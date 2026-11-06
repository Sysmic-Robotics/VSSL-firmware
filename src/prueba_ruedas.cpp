// Firmware de BANCO para medir las ruedas sin cámara (MODO_PRUEBA_RUEDAS en config.h).
//
// No inicia ESP-NOW: la base no lo mueve. Se maneja por el monitor serie USB (115200, fin de
// línea \n). Cada prueba imprime CSV, para pegarlo en una planilla o en Python.
//
//   h               ayuda
//   c               conteo a mano: motores libres; imprime los ticks acumulados de cada rueda.
//                   Girar una rueda 10 vueltas (marca en la llanta) -> 10 x ENCODER_TICKS_PER_WHEEL_REV.
//                   Empujar el robot 1 m contra una regla -> 1000 / MM_POR_TICK ticks por rueda.
//   e [max] [paso]  escalera de PWM en lazo ABIERTO, arrancando cada vez desde quieta: 300 ms
//                   parada, 250 ms con el PWM y 250 ms midiendo. Alterna adelante/atrás para que en
//                   el suelo el robot no se vaya. Por defecto max 400 y paso 10 (en el aire usar
//                   'e 1023 25' para ver el tope). El primer escalón con |v| > 10 mm/s es el PWM
//                   de arranque; la recta v contra PWM, la pendiente.
//   b [inicio] [paso]  bajada en lazo abierto SIN detenerse: arranca en 'inicio' (por defecto
//                   300) y baja 'paso' (por defecto 10) cada 200 ms. El PWM en que la rueda se para
//                   es la fricción andando. En el suelo recorre ~1 m: dejar espacio adelante.
//   v <mm/s> [s]    escalón de avance con el control de PRODUCCIÓN (updateControl) durante s
//                   segundos (por defecto 2). Imprime cada ciclo. En el suelo, medir con huincha lo
//                   recorrido y compararlo con v x s (lo pedido) y con lo que dicen los encoders.
//   w <grados/s> [s]  lo mismo girando en el lugar; compara el giroscopio con lo pedido.
//   x (o espacio)   corta la prueba en curso
//
// Con el robot en el suelo, alguien tiene que estar listo para levantarlo.

#include "config.h"

#ifdef MODO_PRUEBA_RUEDAS

#include <Arduino.h>
#include <stdlib.h>
#include "control.h"
#include "motors.h"
#include "mpu.h"
#include "prueba_ruedas.h"

static char linea[64];
static uint8_t largo = 0;

static bool pedidoParar() {
    bool parar = false;
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c == 'x' || c == 'X' || c == ' ') parar = true;
    }
    return parar;
}

// PWM con el signo de la RUEDA (positivo = adelante), sin pasar por el lazo.
static void ruedasPwm(int pwmIzq, int pwmDer) {
    driveMotor(LEFT_WHEEL_SIGN * pwmIzq, MOT_IN1_PIN, MOT_IN2_PIN);
    driveMotor(RIGHT_WHEEL_SIGN * pwmDer, MOT_IN3_PIN, MOT_IN4_PIN);
}

// Rueda libre de verdad (las dos entradas del DRV8833 en 0), aunque FRENO_EN_REPOSO frene con
// PWM 0: para girar las ruedas a mano.
static void soltarRuedas() {
    analogWrite(MOT_IN1_PIN, 0);
    analogWrite(MOT_IN2_PIN, 0);
    analogWrite(MOT_IN3_PIN, 0);
    analogWrite(MOT_IN4_PIN, 0);
}

// Ticks acumulados con el signo de la RUEDA (positivo = adelante). El signo de encoder deja el
// conteo con el signo del PWM del motor, y el signo de rueda pasa de motor a rueda.
static long ticksRuedaIzq() { return (long)LEFT_WHEEL_SIGN * LEFT_ENCODER_SIGN * ticksIzq; }
static long ticksRuedaDer() { return (long)RIGHT_WHEEL_SIGN * RIGHT_ENCODER_SIGN * ticksDer; }

// Velocidad media de cada rueda (mm/s) durante 'ms' milisegundos, con lo que esté puesto.
static void medirVentana(unsigned long ms, double *vIzq, double *vDer) {
    long i0 = ticksRuedaIzq();
    long d0 = ticksRuedaDer();
    unsigned long t0 = micros();
    delay(ms);
    long i1 = ticksRuedaIzq();
    long d1 = ticksRuedaDer();
    double dt = (micros() - t0) / 1e6;
    *vIzq = (i1 - i0) * MM_POR_TICK / dt;
    *vDer = (d1 - d0) * MM_POR_TICK / dt;
}

static void ayuda() {
    Serial.println("# MODO PRUEBA DE RUEDAS. Comandos (fin de linea \\n):");
    Serial.println("#   c                conteo a mano (motores libres)");
    Serial.println("#   e [max] [paso]   escalera de PWM desde quieta (defecto 400 10; en el aire 1023 25)");
    Serial.println("#   b [inicio] [paso] bajada de PWM sin detenerse (defecto 300 10)");
    Serial.println("#   v <mm/s> [s]     escalon de avance con el control de produccion (defecto 2 s)");
    Serial.println("#   w <grados/s> [s] escalon de giro en el lugar");
    Serial.println("#   x o espacio      cortar");
    Serial.printf("# MM_POR_TICK %.4f (rueda %.1f mm, %d ticks/vuelta): 1 tick por ciclo de %d ms = %.2f mm/s\n",
                  MM_POR_TICK, WHEEL_DIAMETER_M * 1000.0, ENCODER_TICKS_PER_WHEEL_REV,
                  CONTROL_INTERVAL_MS, MM_POR_TICK * 1000.0 / CONTROL_INTERVAL_MS);
}

static void pruebaConteo() {
    soltarRuedas();
    Serial.println("# conteo: motores libres. 10 vueltas de rueda o 1 m empujando contra una regla.");
    Serial.printf("# esperado: 10 vueltas = %d ticks; 1 m = %.0f ticks por rueda. 'x' para terminar.\n",
                  10 * ENCODER_TICKS_PER_WHEEL_REV, 1000.0 / MM_POR_TICK);
    long i0 = ticksRuedaIzq();
    long d0 = ticksRuedaDer();
    unsigned long t0 = millis();
    while (millis() - t0 < 120000UL) {
        if (pedidoParar()) break;
        long i = ticksRuedaIzq() - i0;
        long d = ticksRuedaDer() - d0;
        Serial.printf("ticks izq %ld (%.1f mm, %.2f vueltas)   der %ld (%.1f mm, %.2f vueltas)\n",
                      i, i * MM_POR_TICK, (double)i / ENCODER_TICKS_PER_WHEEL_REV,
                      d, d * MM_POR_TICK, (double)d / ENCODER_TICKS_PER_WHEEL_REV);
        delay(250);
    }
    ruedasPwm(0, 0);
    Serial.println("# fin del conteo");
}

static void pruebaEscalera(int pwmMax, int paso) {
    pwmMax = constrain(pwmMax, 0, MAX_PWM);
    if (paso <= 0) paso = 10;
    Serial.println("pwm,sentido,v_izq_mm_s,v_der_mm_s");
    int sentido = 1;
    for (int pwm = 0; pwm <= pwmMax; pwm += paso) {
        ruedasPwm(0, 0);
        delay(300);
        ruedasPwm(sentido * pwm, sentido * pwm);
        delay(250);
        double vi = 0.0, vd = 0.0;
        medirVentana(250, &vi, &vd);
        ruedasPwm(0, 0);
        Serial.printf("%d,%d,%.1f,%.1f\n", pwm, sentido, sentido * vi, sentido * vd);
        sentido = -sentido;
        if (pedidoParar()) break;
    }
    ruedasPwm(0, 0);
    Serial.println("# fin de la escalera");
}

static void pruebaBajada(int pwmInicio, int paso) {
    pwmInicio = constrain(pwmInicio, 0, MAX_PWM);
    if (paso <= 0) paso = 10;
    Serial.println("pwm,v_izq_mm_s,v_der_mm_s");
    ruedasPwm(pwmInicio, pwmInicio);
    delay(400);
    for (int pwm = pwmInicio; pwm >= 0; pwm -= paso) {
        ruedasPwm(pwm, pwm);
        delay(50);
        double vi = 0.0, vd = 0.0;
        medirVentana(150, &vi, &vd);
        Serial.printf("%d,%.1f,%.1f\n", pwm, vi, vd);
        if (pedidoParar()) break;
    }
    ruedasPwm(0, 0);
    Serial.println("# fin de la bajada");
}

static void pruebaEscalon(int v_mm_s, int w_deg_s, double segundos) {
    if (segundos <= 0.0 || segundos > 10.0) segundos = 2.0;
    long i0 = ticksRuedaIzq();
    long d0 = ticksRuedaDer();
    double yaw0 = isMPUReady() ? getYawAngleRad() : 0.0;
    Serial.println("t_ms,dt_ms,sp_izq,v_izq,pwm_izq,sp_der,v_der,pwm_der,yaw_rad_s,yaw_corr_mm_s");
    g_Linear_MmPerSec = (int16_t)constrain(v_mm_s, -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S);
    long w_mrad = lroundf(w_deg_s * 17.453293f);
    g_Angular_MradPerSec = (int16_t)constrain(w_mrad, -(long)MAX_ANGULAR_MRAD_S, (long)MAX_ANGULAR_MRAD_S);
    unsigned long t0 = millis();
    unsigned long dur = (unsigned long)(segundos * 1000.0);
    unsigned long ultimo = g_telemetria.ciclos;
    bool cortado = false;
    while (millis() - t0 < dur) {
        updateMPU();
        updateControl();
        if (g_telemetria.ciclos != ultimo) {
            ultimo = g_telemetria.ciclos;
            const TelemetriaControl &t = g_telemetria;
            Serial.printf("%lu,%lu,%.0f,%.0f,%d,%.0f,%.0f,%d,%.2f,%.0f\n", t.t_ms - t0, t.dt_ms,
                          t.izq.consigna_mm_s, t.izq.medida_mm_s, t.izq.pwm,
                          t.der.consigna_mm_s, t.der.medida_mm_s, t.der.pwm,
                          t.yaw_medido_rad_s, t.yaw_correccion_mm_s);
        }
        if (pedidoParar()) {
            cortado = true;
            break;
        }
    }
    // Consigna cero y unos ciclos más para que el lazo la aplique como en el partido.
    g_Linear_MmPerSec = 0;
    g_Angular_MradPerSec = 0;
    unsigned long tp = millis();
    while (millis() - tp < 300) {
        updateMPU();
        updateControl();
    }
    ruedasPwm(0, 0);
    long di = ticksRuedaIzq() - i0;
    long dd = ticksRuedaDer() - d0;
    Serial.printf("# %s: encoders izq %.0f mm, der %.0f mm (media %.0f); pedido %.0f mm en %.2f s\n",
                  cortado ? "cortado" : "fin", di * MM_POR_TICK, dd * MM_POR_TICK,
                  0.5 * (di + dd) * MM_POR_TICK, v_mm_s * segundos, segundos);
    if (isMPUReady()) {
        Serial.printf("# giroscopio: %.0f grados (pedido %.0f)\n",
                      (getYawAngleRad() - yaw0) * RAD_TO_DEG, w_deg_s * segundos);
    }
    Serial.println("# medir con huincha lo recorrido de verdad (o contar vueltas) y anotarlo");
}

static void ejecutar(char *l) {
    char cmd = l[0];
    char *p = l + 1;
    char *fin = nullptr;
    double a = strtod(p, &fin);
    bool hayA = (fin != p);
    p = fin;
    double b = strtod(p, &fin);
    bool hayB = (fin != p);

    switch (cmd) {
        case 'h': case 'H': case '?':
            ayuda();
            break;
        case 'c': case 'C':
            pruebaConteo();
            break;
        case 'e': case 'E':
            pruebaEscalera(hayA ? (int)a : 400, hayB ? (int)b : 10);
            break;
        case 'b': case 'B':
            pruebaBajada(hayA ? (int)a : 300, hayB ? (int)b : 10);
            break;
        case 'v': case 'V':
            if (!hayA) { Serial.println("# uso: v <mm/s> [s]"); break; }
            pruebaEscalon((int)a, 0, hayB ? b : 2.0);
            break;
        case 'w': case 'W':
            if (!hayA) { Serial.println("# uso: w <grados/s> [s]"); break; }
            pruebaEscalon(0, (int)a, hayB ? b : 2.0);
            break;
        case 'x': case 'X':
            ruedasPwm(0, 0);
            break;
        default:
            Serial.println("# comando desconocido ('h' para la ayuda)");
    }
}

void pruebaRuedasInicio() {
    g_Linear_MmPerSec = 0;
    g_Angular_MradPerSec = 0;
    ruedasPwm(0, 0);
    ayuda();
}

void pruebaRuedasLoop() {
    updateMPU();
    while (Serial.available() > 0) {
        int c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            linea[largo] = '\0';
            if (largo > 0) ejecutar(linea);
            largo = 0;
        } else if (largo < sizeof(linea) - 1) {
            linea[largo++] = (char)c;
        } else {
            largo = 0;   // línea demasiado larga: se descarta
        }
    }
}

#endif

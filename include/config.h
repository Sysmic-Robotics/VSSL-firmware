#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
//        CONFIGURACIÓN DE MODO
// ==========================================
// Descomenta para usar ESP-NOW (WIFI), comenta para RemoteXY (Bluetooth)
// #define MODO_BASESTATION

// ID de este robot (1 al 5)
#define MI_ROBOT_ID 2

// ==========================================
//               PINES HARDWARE
// ==========================================

// Encoders
#define PIN_ENC_IZQ_A 1     // en el robotito con switch, el encoder taba dao vuelta. dar vuelta pa los demas en vola????
#define PIN_ENC_IZQ_B 0
#define PIN_ENC_DER_A 3
#define PIN_ENC_DER_B 4

// Driver Motores DRV8833
#define MOT_IN1_PIN 5   
#define MOT_IN2_PIN 6   
#define MOT_IN3_PIN 7   
#define MOT_IN4_PIN 10  

// MPU
#define PIN_I2C_SDA 8
#define PIN_I2C_SCL 9

// ==========================================
//           PARÁMETROS DE CONTROL
// ==========================================
const uint32_t PWM_FREQ = 20000;
const uint8_t  PWM_RES  = 10;     // 10 bits = 0 a 1023
const int      MAX_PWM  = 1023;

// ==========================================
//        PARÁMETROS FÍSICOS DEL ROBOT
// ==========================================

// Diámetro real de la rueda
#define WHEEL_DIAMETER_M 0.034

// Ticks de encoder por UNA vuelta completa de la rueda.
// Este valor hay que medirlo experimentalmente.
#define ENCODER_TICKS_PER_WHEEL_REV 575

// Geometría robótica para control diferencial.
// Distancia entre el centro de las dos ruedas (ancho de vía), en mm.
// IMPORTANTE: medir experimentalmente en el robot real, es crítico para
// que la velocidad angular comandada se traduzca en el giro real del robot.
#define WHEEL_TRACK_MM 75.0

// PID cada 20 ms
#define CONTROL_INTERVAL_MS 20
#define CONTROL_DT_S (CONTROL_INTERVAL_MS / 1000.0)

// Tiempo máximo sin recibir comandos
#define COMM_TIMEOUT_MS 200

// ==========================================
//        LÍMITES DE LAS CONSIGNAS (v, w)
// ==========================================
// Velocidad lineal máxima aceptada, en mm/s.
#define MAX_LINEAR_MM_S 1500
// Velocidad angular máxima aceptada, en mrad/s (1000 mrad/s = 1 rad/s).
#define MAX_ANGULAR_MRAD_S 12000

// Límite de seguridad para la velocidad de rueda resultante (cinemática + corrección)
#define MAX_WHEEL_MM_S 2000

// ==========================================
//   CONTROL MANUAL POR JOYSTICK (RemoteXY)
// ==========================================
// Nada de esta sección afecta al modo MODO_BASESTATION: solo se compila y
// se ejecuta cuando el robot se maneja a mano desde el teléfono.

// >>> LIMITADOR DE VELOCIDAD <<<
// Multiplicador global de la velocidad en modo manual. Rango 0.0 a 1.0.
//   1.00 = velocidad máxima          0.50 = mitad de velocidad
//   0.25 = un cuarto (muy manejable) 0.00 = robot quieto
// Es el único número que hay que tocar para que el robot vaya más lento o
// más rápido: escala por igual el avance y el giro, así que el robot se
// maneja igual, solo que más despacio.
#define JOYSTICK_SPEED_LIMIT 0.80f

// Factor extra mientras Boton_1 vale 1 (modo preciso, para acercarse a la
// pelota sin pasarse). Se multiplica encima de JOYSTICK_SPEED_LIMIT.
#define JOYSTICK_PRECISION_FACTOR 0.40f

// Velocidad de avance con el stick a fondo hacia adelante, en mm/s
// (antes de aplicar el limitador de arriba).
// MEDIDO en cancha: con el PWM saturado a 1023 la rueda da ~464 mm/s (izq)
// y ~446 mm/s (der). Pedir más de eso solo satura el PWM y deja al PID sin
// margen, así que el techo se queda justo por debajo del límite físico.
#define JOYSTICK_MAX_LINEAR_MM_S 450

// >>> VELOCIDAD DE GIRO <<<
// Velocidad de CADA rueda cuando el stick va a fondo a un lado, en mm/s.
// Con el stick puro a la derecha, la rueda izquierda va adelante a esta
// velocidad y la derecha atrás a esta misma velocidad: el robot pivota
// sobre su eje. Súbelo para que gire más rápido, bájalo para más control.
// (Se expresa en velocidad de rueda y no en rad/s a propósito: así el giro
// se siente igual aunque cambies WHEEL_TRACK_MM al medirlo de verdad.)
// Tope físico ~450 mm/s por rueda; se deja margen para que el PID corrija.
#define JOYSTICK_PIVOT_WHEEL_MM_S 400

// Zona muerta RADIAL del joystick, en cuentas de la app (los ejes van de
// -100 a 100). Se mide sobre el vector completo, no eje por eje, para que
// el robot no se arranque solo si el dedo queda apenas descentrado.
#define JOYSTICK_DEADZONE 8.0f

// Curva de respuesta del stick: 0.0 = lineal, 1.0 = totalmente cúbica.
// Valores intermedios dan control fino cerca del centro sin perder la
// velocidad máxima al fondo del recorrido.
#define JOYSTICK_EXPO 0.50f

// Sentido de giro del eje X. Con -1, empujar el stick a la derecha gira el
// robot a la derecha (lo intuitivo). Cambia a 1 si lo quieres invertido.
#define JOYSTICK_TURN_SIGN -1

// Rampas de aceleración (límite de slew). Evitan que un salto brusco del
// dedo haga patinar las ruedas: con las ruedas patinando los encoders miden
// una velocidad que el robot no tiene y el control pierde precisión.
#define JOYSTICK_LINEAR_SLEW_MM_S2 2500.0f
#define JOYSTICK_ANGULAR_SLEW_MRAD_S2 20000.0f

// --- Mantención de rumbo con el giroscopio (heading hold) ---
// Cuando se avanza sin pedir giro, el robot memoriza el rumbo y corrige el
// error de ÁNGULO acumulado con el MPU6050. El lazo de control interno solo
// regula velocidad angular, así que por sí solo no puede recuperar los
// grados que ya se perdieron por un golpe o por patinar una rueda.
#define JOYSTICK_HEADING_HOLD 1
// Cuánta velocidad angular (mrad/s) se pide por cada radián de error.
#define JOYSTICK_HEADING_KP_MRAD_PER_RAD 3000.0f
// Techo de esa corrección, para que nunca dé un volantazo.
#define JOYSTICK_HEADING_MAX_CORRECTION_MRAD_S 3000.0f
// Umbrales para armar el hold: hay que estar avanzando y sin pedir giro.
#define JOYSTICK_HEADING_ARM_LINEAR_MM_S 40.0f
#define JOYSTICK_HEADING_ARM_ANGULAR_MRAD_S 150.0f

// Signo de ruedas.
// Si al mandar velocidad positiva una rueda gira al revés,
// cambia el signo correspondiente a -1.
//
// Los dos motores van montados en espejo. Ese espejo hay que compensarlo en
// UN solo sitio: o en los cables (M1/M2 cruzados en un motor) o aquí con un
// -1. Con los cables actuales ya compensan, así que ambos van en 1.
//   Síntoma de compensar dos veces (o ninguna): al pedir AVANCE el robot
//   gira sobre su eje, y al pedir GIRO se traslada.
//   Si avanza cuando debería retroceder: invierte AMBOS signos (solo
//   define cuál extremo es el "frente").
#define LEFT_WHEEL_SIGN  1
#define RIGHT_WHEEL_SIGN -1 

// Signo de encoders. Se elige para que un PWM positivo produzca una lectura
// positiva (realimentación negativa). Es independiente de *_WHEEL_SIGN:
// cambiar el signo de rueda invierte a la vez la consigna y el PWM, así que
// el lazo del PID sigue siendo consistente.
// OJO: el encoder izquierdo tiene A y B cruzados respecto al esquemático
// (PIN_ENC_IZQ_A=GPIO1=B_L y PIN_ENC_IZQ_B=GPIO0=A_L), lo que ya invierte
// su conteo por hardware; por eso su signo efectivo no coincide con el
// derecho pese a que aquí los dos valgan -1.
// Verificado con telemetría (stick adelante): PWM y ticks medidos deben
// tener el MISMO signo. Izq: PWM +120 / Act +33 -> correcto con -1.
// Der: PWM -235 / Act +46 -> signos opuestos = realimentación POSITIVA,
// el PID aceleraba la rueda en vez de frenarla. Corregido a 1.
#define LEFT_ENCODER_SIGN  -1
#define RIGHT_ENCODER_SIGN 1

// Signo del eje Z del giroscopio del MPU6050.
// Si al girar el robot en sentido antihorario (w > 0) el gyro mide negativo,
// cambia este signo a -1 para que coincida con la convención de (v, w).
#define GYRO_Z_SIGN 1

// PID Gains (control de velocidad por rueda, con encoder)
extern double kp, ki, kd;

// Ganancias del corrector de guiñada (PI sobre el error de velocidad angular
// medida por el giroscopio vs. la comandada). Corrige asimetrías/deslizamiento
// que el encoder por sí solo no puede detectar.
extern double kp_yaw, ki_yaw;
// Máxima corrección diferencial que puede aportar el lazo de guiñada, en mm/s.
#define YAW_CORRECTION_LIMIT_MM_S 200

// Consignas globales recibidas desde la estación base / software de visión:
// velocidad lineal (mm/s) y velocidad angular (mrad/s) del robot completo.
extern volatile int16_t g_Linear_MmPerSec;
extern volatile int16_t g_Angular_MradPerSec;

#endif
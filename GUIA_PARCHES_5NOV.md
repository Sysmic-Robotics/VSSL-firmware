# Parches del 5-nov-2026: base station y rueda (rama `Peluche-parches-5nov`)

Rama hecha sobre `Peluche` (`8510c6d`, encoders por ISR). **Nada de esto se compiló ni se probó en
un robot.** Antes de cargarlo para un partido hay que compilarlo (`pio run`) y probarlo en el banco
(sección 4).

## Por qué

Sale del análisis de los registros de la cancha de madera del 5-nov. Los informes completos están
en el notebook oficial, en `~/Desktop/estudio_5nov/firmware/informe.md` y `PLAN.md`.

- **La rueda no tiene un lazo de velocidad que funcione.** La velocidad la pone el feedforward
  (`PWM = 90 + 2·v`) y el PID casi no corrige: `ki·Ts` = 0.004 por ciclo, un tiempo integral de
  12.5 s. Eso produce dos cosas:
  - **zona muerta:** el robot no se mueve con menos de ~70-85 mm/s por rueda ni gira con menos de
    ~1.5 rad/s;
  - **avance de más:** anda 1.5-2.1 veces lo pedido. El feedforward se calibró con otro modo del
    driver y antes de los engranajes.
- **La base reenvía cada 50 ms:** son 25 ms de retraso de media. Además, si el PC deja de mandar, la
  base repite el último comando para siempre.

## 1. Qué trae cada commit

| Commit | Qué hace | Apagado o encendido |
|---|---|---|
| `Base station: reenvia cada linea…` | `base_station/src/main.cpp`. Manda cada línea serie apenas llega, con un keepalive del último comando cada 50 ms, y pasa a ceros si no llega nada por serie en 300 ms. Cada 5 s imprime "lineas ok N, invalidas M, envios K" | **Encendido.** Es el único que cambia el comportamiento al cargarlo, y solo en la base |
| `Modo de prueba de ruedas…` | Firmware de banco: `#define MODO_PRUEBA_RUEDAS` en `include/config.h`. **Además saca los objetos `Encoder` de la librería**, que estaban en los mismos pines que la ISR propia, y agrega telemetría del control | El modo banco viene **apagado** (comentado). Sacar los `Encoder` vale siempre |
| `PI de rueda con integral de verdad…` | Lazo de velocidad por rueda: PI con anti-windup, PWM de arranque ("patada") y feedforward medido | **Apagado:** `CONTROL_RUEDA_V2 0` en `config.h`. Con 0 se usa el control de hoy |
| `Freno en reposo…` | Con consigna cero, freno (motor en corto) en vez de rueda libre | **Apagado:** `FRENO_EN_REPOSO 0` en `config.h` |

El parche 03 (aplicar la consigna en cuanto llega) no está en la rama: depende de validar antes el
PI de rueda.

**Antes de nada, averiguar qué firmware tienen hoy los robots.** El clon del notebook está en
`cf74a88` y GitHub en `8510c6d`. Si los robots tienen el anterior, esta rama también les cambia los
encoders (la ISR de Ignacio).

## 2. Base station (lo único que conviene cargar pronto)

1. `pio run -d base_station` y, si compila, cárgala.
2. Guarda el firmware que tenía, para volver atrás: el de `8510c6d`.
3. **Prueba en el banco**, con un robot en el soporte:
   - en el monitor serie de la base, con el engine mandando, "envios" tiene que dar ~60 por segundo;
   - desenchufa el USB del PC con el robot andando: el robot tiene que pararse en 300 ms o menos (hoy
     sigue andando).
4. En la cancha, el retraso debería bajar ~25 ms, sin tocar nada del engine.

## 3. Robots: compilar sin cambiar el control

Cada robot se compila con su `MI_ROBOT_ID` en `include/config.h`. Con todo apagado, el control es el
de hoy, salvo dos cosas: no están los objetos `Encoder` duplicados y hay telemetría.

Antes de jugar con esto, prueba en el banco que se mueve igual que antes: `v 200 2` en el modo banco
(sección 4) o con el engine.

## 4. Banco, sin cámara (~15 min por robot)

1. Descomenta `#define MODO_PRUEBA_RUEDAS` en `config.h`, compila y carga.
2. Abre el monitor: `pio device monitor -b 115200 | tee banco_robotN.txt`.
3. **Primero, con el control de hoy** (`CONTROL_RUEDA_V2 0` y `FRENO_EN_REPOSO 0`), para confirmar el
   diagnóstico:
   1. **`c` (escala):** gira cada rueda 10 vueltas a mano; tienen que ser 5750 ± 20 ticks. Después
      empuja el robot 1 m en línea recta: 5229 ticks por rueda. Si da ~11 500 hay doble conteo; si el
      metro da otra cosa, el diámetro efectivo es 35 · ticks / 5229 mm.
   2. **`e 1023 25` con la rueda en el aire:** pendiente y tope en vacío (se espera ~765 mm/s).
   3. **`e 400 10` en el suelo:** PWM de despegue (se espera ~200-290).
   4. **`b 300 10` en el suelo:** el PWM con que la rueda deja de andar.
   5. **`v 200 2` con huincha:** hoy debería recorrer ~550-650 mm en vez de 400. Es el 1.5× medido sin
      cámara. Repite con `v 50 2`, que hoy casi no se mueve, y `v 400 2`.
   6. **`w 180 2`:** compara los grados del giroscopio con los 360 pedidos.
4. `python3 herramientas/analizar_banco.py banco_robotN.txt` propone `RUEDA_PWM_ARRANQUE`,
   `RUEDA_PWM_CINETICO` y `RUEDA_PWM_POR_MM_S` para el PI de rueda.

**Siempre:** alguien listo para levantar el robot.

## 5. Encender el PI de rueda (después del banco, no antes de un partido)

1. Copia a `src/control.cpp` las constantes que propuso `analizar_banco.py`.
2. Pon `CONTROL_RUEDA_V2 1`, y si quieres freno, `FRENO_EN_REPOSO 1`.
3. Compila y repite en el banco. Lo esperado:
   - `v 200 2`, `v 50 2` y `v 20 2`: la huincha marca ≈ v × s ± 3 cm;
   - `v 10 3`: avanza ~3 cm (hoy no se mueve);
   - `w 90 4`: el giroscopio da ~360°;
   - empujado con la mano y consigna 0: con el freno opone resistencia;
   - trabado contra una pared con `v 300 2`: sin sobrepaso al soltarlo y sin que se caliente el
     DRV8833.
4. **Para volver atrás:** `CONTROL_RUEDA_V2 0` y `FRENO_EN_REPOSO 0`, y recompilar.

**Ojo:** con el PI de rueda el robot pasa a andar lo que se le pide (ganancia ~1 en vez de
1.5-2.1). Hay que avisarle a Seba antes de jugar, porque el engine está ajustado para el robot de
hoy: hay que cambiar `config/robots_reales.json` (velocidad tope, compensación de zona muerta,
`keeper_gain`) y la calibración del simulador. No se cambia a mitad de un torneo sin esa
recalibración.

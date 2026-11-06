#!/usr/bin/env python3
"""Analiza lo que imprime el firmware de banco (parche 00, MODO_PRUEBA_RUEDAS) y propone las
constantes del lazo V2 (parche 01). Solo biblioteca estándar.

  python3 analizar_banco.py monitor.txt [más.txt ...]

monitor.txt = lo copiado del monitor serie (pio device monitor -b 115200 | tee monitor.txt).
Reconoce los bloques por su encabezado:
  pwm,sentido,v_izq_mm_s,v_der_mm_s      prueba 'e' (escalera desde quieta)
  pwm,v_izq_mm_s,v_der_mm_s              prueba 'b' (bajada sin detenerse)
  t_ms,dt_ms,sp_izq,v_izq,pwm_izq,...    pruebas 'v' / 'w' (escalón con el control de producción)
  # fin: encoders ...                    resumen de 'v' / 'w'
Para cada bloque imprime lo medido y, al final, los valores sugeridos de RUEDA_PWM_ARRANQUE,
RUEDA_PWM_CINETICO y RUEDA_PWM_POR_MM_S, y la ganancia y el t63 del lazo en cada escalón.
"""
import sys

ENC_E = "pwm,sentido,v_izq_mm_s,v_der_mm_s"
ENC_B = "pwm,v_izq_mm_s,v_der_mm_s"
ENC_V = "t_ms,dt_ms,sp_izq,v_izq,pwm_izq,sp_der,v_der,pwm_der,yaw_rad_s,yaw_corr_mm_s"
UMBRAL_MUEVE = 10.0   # mm/s


def bloques(lineas):
    actual, filas = None, []
    for l in lineas:
        l = l.strip()
        if l in (ENC_E, ENC_B, ENC_V):
            if actual:
                yield actual, filas
            actual, filas = l, []
            continue
        if actual is None:
            continue
        if not l or l.startswith("#"):
            if l.startswith("# fin") or l.startswith("# cortado"):
                filas.append(("resumen", l))
            if l.startswith("# fin de"):
                yield actual, filas
                actual, filas = None, []
            continue
        try:
            filas.append(tuple(float(x) for x in l.split(",")))
        except ValueError:
            pass
    if actual:
        yield actual, filas


def recta(xs, ys):
    n = len(xs)
    if n < 3:
        return None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return a, my - a * mx


def escalera(filas):
    datos = [f for f in filas if isinstance(f[0], float)]
    res = {}
    for k, nombre in ((2, "izq"), (3, "der")):
        arranque = next((f[0] for f in datos if f[0] > 0 and abs(f[k]) > UMBRAL_MUEVE), None)
        top = max((abs(f[k]) for f in datos), default=0.0)
        pmax = max((f[0] for f in datos), default=0.0)
        # tramo lineal: andando y sin saturar
        xs = [f[0] for f in datos if abs(f[k]) > 30 and f[0] < 0.95 * 1023]
        ys = [abs(f[k]) for f in datos if abs(f[k]) > 30 and f[0] < 0.95 * 1023]
        r = recta(xs, ys)
        res[nombre] = (arranque, r, top, pmax)
        if r:
            km, b = r
            print(f"  rueda {nombre}: arranca desde quieta con PWM {arranque}; recta v = {km:.3f} * pwm {b:+.0f} "
                  f"-> {km:.3f} mm/s por PWM, pierde {-b/km:.0f} de PWM andando; máx {top:.0f} mm/s con PWM {pmax:.0f}")
        else:
            print(f"  rueda {nombre}: arranca con PWM {arranque}; pocos puntos para la recta (máx {top:.0f} mm/s)")
    return res


def bajada(filas):
    datos = [f for f in filas if isinstance(f[0], float)]
    res = {}
    for k, nombre in ((1, "izq"), (2, "der")):
        anda = [f[0] for f in datos if abs(f[k]) > UMBRAL_MUEVE]
        para = min(anda) if anda else None
        res[nombre] = para
        print(f"  rueda {nombre}: sigue andando hasta PWM {para} (bajo eso se para: fricción andando)")
    return res


def escalon(filas):
    datos = [f for f in filas if isinstance(f[0], float)]
    resumen = [f[1] for f in filas if f[0] == "resumen"]
    if not datos:
        return None
    dur = datos[-1][0]
    out = {}
    for ksp, kv, nombre in ((2, 3, "izq"), (5, 6, "der")):
        sp = [f[ksp] for f in datos if f[0] >= 0.5 * dur]
        v = [f[kv] for f in datos if f[0] >= 0.5 * dur]
        if not sp or abs(sum(sp) / len(sp)) < 1:
            continue
        spm, vm = sum(sp) / len(sp), sum(v) / len(v)
        spf = datos[-1][ksp]
        t63 = next((f[0] for f in datos if abs(f[kv]) >= 0.63 * abs(spf)), float("nan"))
        pico = max(abs(f[kv]) for f in datos)
        out[nombre] = vm / spm
        print(f"  rueda {nombre}: consigna {spm:.0f} mm/s, medida {vm:.0f} (encoder, 2a mitad) = {vm/spm:.2f}; "
              f"t63 {t63:.0f} ms; pico {pico:.0f} ({100*(pico/abs(spf)-1):+.0f} %)")
    for r in resumen:
        print("  " + r)
    return out


def main():
    lineas = []
    for ruta in sys.argv[1:]:
        with open(ruta, errors="replace") as f:
            lineas.extend(f.readlines())
    arr, cin, pend = [], [], []
    for enc, filas in bloques(lineas):
        if enc == ENC_E:
            print("ESCALERA ('e')")
            for nombre, (a, r, top, pmax) in escalera(filas).items():
                if a is not None:
                    arr.append(a)
                if r:
                    pend.append(r[0])
                    if r[0] > 0:
                        cin.append(-r[1] / r[0])   # PWM que se pierde andando (corte de la recta)
        elif enc == ENC_B:
            print("BAJADA ('b')")
            cin.extend(p for p in bajada(filas).values() if p is not None)
        else:
            print("ESCALÓN ('v' / 'w')")
            escalon(filas)
    print("\nSUGERIDO para control.cpp (parche 01), con la peor rueda:")
    if arr:
        print(f"  RUEDA_PWM_ARRANQUE = {round(1.1 * max(arr))}   (máximo arranque medido {max(arr):.0f} + 10 %)")
    if cin:
        print(f"  RUEDA_PWM_CINETICO = {round(min(cin))}   (lo menor entre el corte de la recta de 'e' y donde se para en 'b'; no pasarse)")
    if pend:
        km = sum(pend) / len(pend)
        print(f"  RUEDA_PWM_POR_MM_S = {1 / km:.2f}   (1 / {km:.3f} mm/s por PWM, media de las ruedas)")
    if not (arr or cin or pend):
        print("  (no hubo bloques 'e' ni 'b')")


if __name__ == "__main__":
    main()

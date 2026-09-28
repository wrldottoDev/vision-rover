"""Recalcula el modelo geometrico con las mediciones tomadas sobre el rover fisico.

Uso tipico, con lo que se haya podido medir (todo es opcional):

    python3 recalcular.py --hueco45 8.7
    python3 recalcular.py --hueco45 8.7 --hueco0 33.5 --ancho-max 104 --lam 40
    python3 recalcular.py --tres-cubos 180.5 --sesgo-lateral 2.0 --radio-arista 2.0

El script no supone nada: si no le pasas un valor, usa el del modelo actual (el
deducido del DXF de piezas separadas) y lo marca como MODELO en la salida. Todo lo
que le pases se marca como MEDIDO y manda sobre lo derivado.

Al final dice explicitamente que conclusiones del informe cambian.
"""
import argparse
import math

# --- valores del modelo actual (DXF de piezas separadas) ---------------------
M_WU = 93.50        # ancho util del canal
M_WEXT = 99.50      # huella exterior
M_LAM = 94.00       # largo del chasis
M_L = 60.00         # arista del cubo
SIG_VIS = 5.00      # incertidumbre de vision adoptada
MARGEN = 3.00       # margen de control que se le reserva al controlador
S_SEG = 40.00       # margen de seguridad del punto de puesta
CELL = 20.0


def marca(medido):
    return 'MEDIDO ' if medido else 'modelo '


def ancho_proyectado(L, delta_deg, r=0.0):
    """Anchura proyectada de un cuadrado de lado L girado delta, con aristas de radio r.

    Con aristas vivas (r=0) es la formula clasica L(|cos|+|sin|). El redondeo
    recorta las esquinas: el ancho maximo baja de L*sqrt2 a L*sqrt2 - 2r(sqrt2-1).
    """
    d = math.radians(delta_deg)
    w = L * (abs(math.cos(d)) + abs(math.sin(d)))
    if r > 0:
        # el recorte es proporcional a cuanto se aparta de la posicion alineada
        w -= 2 * r * (abs(math.cos(d)) + abs(math.sin(d)) - 1.0)
    return w


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--hueco45', type=float,
                    help='M1: hueco junto al cubo girado 45 grados, en mm')
    ap.add_argument('--hueco0', type=float,
                    help='M2: hueco junto al cubo alineado, en mm')
    ap.add_argument('--canal', type=float,
                    help='M9: separacion medida directamente entre caras interiores')
    ap.add_argument('--ancho-max', type=float,
                    help='M7: ancho maximo real del rover incluyendo ruedas')
    ap.add_argument('--largo-max', type=float, help='M8: largo maximo real')
    ap.add_argument('--lam', type=float, help='M6: alcance de las paletas por delante')
    ap.add_argument('--largo-chasis', type=float, default=M_LAM,
                    help='largo del chasis (por defecto 94.00, del DXF)')
    ap.add_argument('--tres-cubos', type=float,
                    help='M11: largo de los tres cubos en fila, en mm')
    ap.add_argument('--radio-arista', type=float, default=0.0,
                    help='M12: radio de redondeo de las aristas del cubo')
    ap.add_argument('--sesgo-lateral', type=float, default=0.0,
                    help='M4: descentrado lateral del marcador ArUco, en mm')
    ap.add_argument('--sigma-vis', type=float, default=SIG_VIS,
                    help='incertidumbre de vision adoptada (por defecto 5 mm)')
    a = ap.parse_args()

    # ---------------------------------------------------------------- arista
    med_L = a.tres_cubos is not None
    L = a.tres_cubos / 3.0 if med_L else M_L
    r = a.radio_arista

    # ------------------------------------------------------- ancho del canal
    # tres vias posibles, en orden de preferencia
    med_W = True
    if a.canal is not None:
        Wu, via = a.canal, 'medido directo (M9)'
    elif a.hueco45 is not None:
        Wu, via = a.hueco45 + ancho_proyectado(L, 45.0, r), 'deducido del hueco a 45 (M1)'
    elif a.hueco0 is not None:
        Wu, via = a.hueco0 + ancho_proyectado(L, 0.0, r), 'deducido del hueco alineado (M2)'
    else:
        Wu, via, med_W = M_WU, 'DXF de piezas separadas', False

    Wext = a.ancho_max if a.ancho_max is not None else M_WEXT
    med_Wext = a.ancho_max is not None
    Lam = a.largo_max if a.largo_max is not None else M_LAM
    med_Lam = a.largo_max is not None

    print('=' * 74)
    print('ENTRADAS')
    print('=' * 74)
    print('  %sarista del cubo L      = %8.2f mm' % (marca(med_L), L))
    if r:
        print('  MEDIDO radio de arista       = %8.2f mm' % r)
    print('  %sancho util del canal   = %8.2f mm   (%s)' % (marca(med_W), Wu, via))
    print('  %sancho maximo real      = %8.2f mm' % (marca(med_Wext), Wext))
    print('  %slargo (para R_rov)     = %8.2f mm' % (marca(med_Lam), Lam))
    print('  adoptado sigma_vision        = %8.2f mm' % a.sigma_vis)
    if a.sesgo_lateral:
        print('  MEDIDO sesgo del marcador    = %8.2f mm' % a.sesgo_lateral)

    # ---------------------------------------------------- control de coherencia
    if a.hueco45 is not None and a.hueco0 is not None:
        dif = a.hueco0 - a.hueco45
        esperado = ancho_proyectado(L, 45.0, r) - ancho_proyectado(L, 0.0, r)
        print()
        print('CONTROL M2 - M1 (identidad geometrica, no depende de W_u)')
        print('  medido   = %6.2f mm' % dif)
        print('  esperado = %6.2f mm' % esperado)
        if abs(dif - esperado) <= 1.5:
            print('  -> COHERENTE: las dos mediciones se pueden creer.')
        else:
            print('  -> INCOHERENTE por %.2f mm. Revisar antes de usar estos numeros:' %
                  abs(dif - esperado))
            print('     o el cubo no mide lo que se cree, o las paletas no son paralelas,')
            print('     o uno de los dos huecos se midio mal.')

    # ------------------------------------------------------------- derivadas
    wS_max = ancho_proyectado(L, 45.0, r)
    wS_min = ancho_proyectado(L, 0.0, r)
    emax_peor = (Wu - wS_max) / 2
    emax_mejor = (Wu - wS_min) / 2
    presu_peor = emax_peor - a.sigma_vis - a.sesgo_lateral
    presu_mejor = emax_mejor - a.sigma_vis - a.sesgo_lateral

    R_rov = 0.5 * math.sqrt(Wext ** 2 + Lam ** 2)
    rho_ct = wS_max / 2 + Lam / 2
    if a.lam is not None:
        rho_ct_real = wS_max / 2 + Lam / 2 + a.lam
    else:
        rho_ct_real = None
    rho_pu = rho_ct + S_SEG

    print()
    print('=' * 74)
    print('MAGNITUDES DERIVADAS')
    print('=' * 74)
    print('  anchura proyectada w_S       = [%6.2f, %6.2f] mm' % (wS_min, wS_max))
    print('  R_rov (envolvente)           = %8.2f mm = %.3f celdas' % (R_rov, R_rov / CELL))
    print('  rho_contacto (cota)          = %8.2f mm' % rho_ct)
    if a.lam is not None:
        X_ADEL = a.largo_chasis / 2.0 + a.lam
        R_real = math.hypot(X_ADEL, Wext / 2.0)
        R_mod = 0.5 * math.hypot(Wext, a.largo_chasis)
        rho_man = R_real + wS_max / 2 + 5.0
        print()
        print('  --- envolvente CON las paletas (hallazgo H-12) ---')
        print('  huella asimetrica            atras %.1f, adelante %.1f, lateral +-%.1f mm'
              % (a.largo_chasis / 2, X_ADEL, Wext / 2))
        print('  largo total                  = %8.2f mm' % (a.largo_chasis + a.lam))
        print('  R_rov del modelo (chasis)    = %8.2f mm' % R_mod)
        print('  R_rov real                   = %8.2f mm   -> +%.0f %%'
              % (R_real, 100 * (R_real / R_mod - 1)))
        print('  anillo de maniobra           = %8.2f mm   (para girar sin barrer el cubo)'
              % rho_man)
        print('  despeje lateral junto a otro cubo = %.2f mm' % (Wext / 2 + wS_max / 2))
        print('  retirada antes de girar      = %8.2f mm   (= lambda)' % a.lam)
        if a.lam < wS_max:
            print('  NOTA: lambda < w_S maximo: el cubo sobresale de las puntas; las')
            print('        paletas cubren el %.0f %% de su extension a 45 grados.'
                  % (100 * a.lam / wS_max))
    print('  rho_puesta (s=%.0f)            = %8.2f mm = %.2f celdas'
          % (S_SEG, rho_pu, rho_pu / CELL))
    print('  umbral de cuerda del anillo  = %8.2f grados'
          % (2 * math.degrees(math.acos(min(1.0, rho_ct / rho_pu)))))
    print('  e_max                        = [%6.2f, %6.2f] mm' % (emax_peor, emax_mejor))
    print('  presupuesto de control       = [%6.2f, %6.2f] mm' % (presu_peor, presu_mejor))

    # ---------------------------------------------------------- ventana angular
    print()
    print('  %-10s %12s %13s %16s' % ('margen m', 'w admisible', '|Delta| max', 'frac. orient.'))
    for m in (0.0, 1.0, 2.0, 3.0, 5.0):
        w = Wu - 2 * (m + a.sigma_vis + a.sesgo_lateral)
        if w < wS_min:
            print('  %-10.1f %12.2f %13s %16s' % (m, w, 'IMPOSIBLE', '0 %'))
            continue
        rr = (w / L) ** 2 - 1 if r == 0 else None
        if r == 0 and rr >= 1:
            print('  %-10.1f %12.2f %13s %16s' % (m, w, 'sin limite', '100 %'))
            continue
        # barrido numerico, valido tambien con aristas redondeadas
        adm = [d / 10.0 for d in range(0, 451)
               if ancho_proyectado(L, d / 10.0, r) <= w]
        if not adm:
            print('  %-10.1f %12.2f %13s %16s' % (m, w, 'IMPOSIBLE', '0 %'))
            continue
        dmax = max(adm)
        print('  %-10.1f %12.2f %12.2f%s %15.0f %%' % (m, w, dmax, chr(176), 100 * dmax / 45.0))

    # ------------------------------------------------------------- veredicto
    print()
    print('=' * 74)
    print('VEREDICTO SOBRE LOS HALLAZGOS DEL INFORME')
    print('=' * 74)

    # H-11
    if presu_peor < 0:
        neg = [d / 10.0 for d in range(0, 901)
               if (Wu - ancho_proyectado(L, d / 10.0, r)) / 2 - a.sigma_vis
               - a.sesgo_lateral < 0]
        frac = 100.0 * len(neg) / 901.0
        print('  H-11  CONFIRMADO. Presupuesto de control negativo (%.2f mm) en el peor' % presu_peor)
        print('        caso; %.0f %% de las orientaciones queda sin margen.' % frac)
        print('        Conocer el angulo del cubo es OBLIGATORIO.')
    elif presu_peor < MARGEN:
        print('  H-11  ATENUADO. El presupuesto en el peor caso es %.2f mm: positivo pero' % presu_peor)
        print('        por debajo del margen de control deseable (%.1f mm). Conocer el' % MARGEN)
        print('        angulo deja de ser obligatorio y vuelve a ser palanca de robustez.')
    else:
        print('  H-11  REFUTADO. El presupuesto en el peor caso es %.2f mm, suficiente.' % presu_peor)
        print('        Conocer el angulo del cubo vuelve a ser una mejora de eficiencia.')

    # factibilidad basica
    if Wu < wS_max:
        print('  NUEVO CRITICO: W_u = %.2f < w_S maximo = %.2f. Hay orientaciones en las'
              % (Wu, wS_max))
        print('        que el cubo NO ENTRA entre las paletas. El modelo supone que')
        print('        siempre entra; esa hipotesis se cae.')
    else:
        print('  Factibilidad: W_u = %.2f > w_S maximo = %.2f, el cubo siempre entra.'
              % (Wu, wS_max))

    # H-10 / R_rov
    if med_Wext and Wext > M_WEXT + 0.5:
        print('  H-10  AMPLIADO. El ancho real (%.2f) supera al del acrilico (%.2f):'
              % (Wext, M_WEXT))
        print('        R_rov sube de %.2f a %.2f mm. El inflado de obstaculos y la zona'
              % (0.5 * math.sqrt(M_WEXT ** 2 + M_LAM ** 2), R_rov))
        print('        de exclusion de borde estaban SUBESTIMADOS: error del lado peligroso.')

    if a.sesgo_lateral:
        print('  SESGO del marcador: se lleva %.2f mm del presupuesto. Es sistematico,'
              % a.sesgo_lateral)
        print('        asi que se puede compensar en el cliente restandolo a la posicion')
        print('        reportada. Hacerlo devuelve esos %.2f mm.' % a.sesgo_lateral)

    if r:
        base = (M_WU - 60 * math.sqrt(2)) / 2
        print('  Aristas redondeadas: e_max en el peor caso pasa de %.2f a %.2f mm.'
              % (base, emax_peor))

    print()


if __name__ == '__main__':
    main()

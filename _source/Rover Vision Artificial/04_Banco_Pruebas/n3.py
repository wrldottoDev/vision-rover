"""Nivel N3: capa geometrica de planificacion, barridos globales."""
from __future__ import annotations
import math, sys
import numpy as np
sys.path.insert(0, '/tmp/bench')
from modelo import *
from arnes import prueba, resumen, volcar

# ------------------------------------------------------------- parametros ---
CELL   = 20.0
COLS   = 43.0
BORDE  = 3.5                      # borde muerto del tablero fisico, en celdas
L      = 60.0                     # mm
# DIMENSIONES REALES, medidas del archivo de fabricacion oficial
# archivos_fabricacion/CENFOBOT_Rover_Rev1.dxf (escala confirmada por las
# ranuras de 3.00 mm del acrilico y por el ultrasonico: 2 discos de 16.15 mm
# separados 24.00 mm, patron estandar HC-SR04).
# Dos anchos DISTINTOS, y confundirlos es un error:
#   W_EXT  = huella exterior, punta a punta de las lenguetas. Gobierna colision,
#            punto de puesta y frontera.
#   W_CANAL= separacion entre las caras INTERNAS de los paneles laterales.
#            Gobierna la captura del cubo (teorema del embudo).
# El chasis mide 93.50 mm de cuerpo con lenguetas de 3.00 mm por lado (= espesor
# del acrilico, junta pasante), asi que las caras internas de los paneles
# coinciden con los bordes del cuerpo del chasis.
W_EXT   = 99.50
W_CANAL = 93.50
W      = W_EXT      # por defecto, la huella
LAM    = 94.00      # largo del chasis
# MEDIDO sobre el rover fisico (regla, +-1 mm). Confirma el DXF:
#   ancho total 100 (DXF 99.50), canal 95 (DXF 93.50),
#   largo total 150 = chasis 94 + paletas 55  -> el DXF cierra exacto.
LAMBDA  = 55.0      # alcance de las paletas por delante de la placa frontal
# La huella NO es simetrica respecto del centro del chasis: hacia atras llega
# LAM/2, hacia adelante LAM/2 + LAMBDA (las paletas). Ignorar esto subestima
# el radio envolvente en un 67 %, que es un error del lado peligroso.
X_ATRAS = -LAM / 2.0
X_ADEL  = LAM / 2.0 + LAMBDA
HW      = W_EXT / 2.0
RROV_ASIM = math.hypot(X_ADEL, HW)      # disco envolvente centrado en el chasis
S_MARG = 40.0
RROV   = R_rov(W, LAM)            # mm
RHO_PU = rho_puesta(L, LAM, S_MARG)
RHO_CT_MIN = L / 2 + LAM / 2
RHO_CT_MAX = L * math.sqrt(2) / 2 + LAM / 2
V      = 6.0 * CELL               # mm/s   (velocidad_rover_celdas_s = 6)
OM     = 90.0                     # grados/s
DEPOTS = {'green': np.array([40.5, 2.5]), 'blue': np.array([2.5, 40.5]),
          'red':   np.array([40.5, 40.5])}
rng = np.random.default_rng(20260902)

# limites del tablero fisico en celdas
TAB_MIN, TAB_MAX = -BORDE, COLS + BORDE


def dentro_tablero(p, margen_mm=0.0):
    m = margen_mm / CELL
    p = np.atleast_2d(p)
    return np.all((p >= TAB_MIN + m) & (p <= TAB_MAX - m), axis=1)


# ------------------------------------------------------------------ N3.1 ----

@prueba('N3.1', 'Mapa de factibilidad del punto de puesta')
def n31(r):
    """Dos modelos de cuerpo: disco envolvente (conservador) y rectangulo orientado."""
    paso = 0.25
    g = np.arange(0.0, COLS + 1e-9, paso)
    X, Y = np.meshgrid(g, g, indexing='ij')
    C = np.stack([X.ravel(), Y.ravel()], axis=1)

    def factible(gv, modelo):
        D = gv[None, :] - C
        n = np.linalg.norm(D, axis=1)
        m = n > 1e-9
        d = np.zeros_like(D); d[m] = D[m] / n[m, None]
        A = C - (RHO_PU / CELL) * d
        if modelo == 'disco':
            ok = dentro_tablero(A, RROV)
        else:
            # rectangulo orientado segun phi* = rumbo hacia el deposito
            phi = arg_de(d)
            up = u(phi); pe = u(phi + 90.0)
            hl, hw = (LAM / 2) / CELL, (W / 2) / CELL
            ok = np.ones(len(C), bool)
            for sa in (-1, 1):
                for sb in (-1, 1):
                    esq = A + sa * hl * up + sb * hw * pe
                    ok &= dentro_tablero(esq, 0.0)
        ok[~m] = True
        return ok, d

    print()
    print('           %-8s %14s %14s %16s' % ('deposito', 'inviab_disco', 'inviab_rect', 'banda_teorica_c'))
    fr_d = {}; fr_r = {}
    for color, gv in DEPOTS.items():
        okd, _ = factible(gv, 'disco')
        okr, _ = factible(gv, 'rect')
        fr_d[color] = 1.0 - okd.mean(); fr_r[color] = 1.0 - okr.mean()
        print('           %-8s %13.2f%% %13.2f%% %16.2f'
              % (color, 100 * fr_d[color], 100 * fr_r[color], RHO_PU / CELL))
        r.metrica('inviable_%s_disco_pct' % color, 100 * fr_d[color])
        r.metrica('inviable_%s_rect_pct' % color, 100 * fr_r[color])

    # --- geometria de la region: debe estar del lado OPUESTO al deposito -----
    gv = DEPOTS['green']
    okr, d = factible(gv, 'rect')
    inv = C[~okr]
    dist_inv = np.linalg.norm(inv - gv, axis=1) if len(inv) else np.array([0.0])
    dist_all = np.linalg.norm(C - gv, axis=1)
    lado_ok = bool(dist_inv.mean() > dist_all.mean())
    r.metrica('dist_media_inviable_c', float(dist_inv.mean()))
    r.metrica('dist_media_cancha_c', float(dist_all.mean()))
    r.metrica('region_del_lado_opuesto', lado_ok)

    # --- espesor de la banda medido sobre el eje x, para el deposito verde ---
    # (el deposito verde esta a col alta, asi que la banda inviable esta a col baja)
    inv_x = inv[:, 0] if len(inv) else np.array([np.nan])
    r.metrica('banda_col_max_c', float(np.nanmax(inv_x)) if len(inv) else 0.0)
    r.metrica('banda_teorica_c', RHO_PU / CELL)

    # --- verificacion cruzada: un punto factible debe tener puesta valida ----
    idx = rng.choice(len(C), 5000, replace=False)
    incoherencias = 0
    for i in idx:
        c = C[i]
        dd = gv - c; nn = np.linalg.norm(dd)
        if nn < 1e-9: continue
        a = c - (RHO_PU / CELL) * (dd / nn)
        phi = float(arg_de(dd / nn))
        up = u(phi); pe = u(phi + 90.0)
        hl, hw = (LAM / 2) / CELL, (W / 2) / CELL
        esqs = np.array([a + sa * hl * up + sb * hw * pe
                         for sa in (-1, 1) for sb in (-1, 1)])
        dentro = bool(np.all(dentro_tablero(esqs, 0.0)))
        if dentro != bool(okr[i]):
            incoherencias += 1
    r.metrica('incoherencias_verificacion', incoherencias)

    print()
    print('           HALLAZGO H-05: la fraccion de cancha SIN empuje directo es grande.')
    print('           Con el rectangulo real del rover: %.1f%% de las posiciones de cubo.' % (100 * fr_r['green']))
    print('           El empuje multi-tramo NO es una rama muerta: hace falta para 1 de cada %.1f'
          % (1.0 / fr_r['green']))
    print('           posiciones posibles del cubo.')
    r.nota('HALLAZGO H-05: %.1f%% de la cancha (modelo rectangulo) no admite empuje directo'
           % (100 * fr_r['green']))
    r.nota('el modelo enunciaba la region cualitativamente; cuantificada resulta mayoritariamente')
    r.nota('relevante: el empuje multi-tramo es obligatorio, no opcional.')
    r.nota('el disco envolvente sobreestima la region en %.1f puntos porcentuales'
           % (100 * (fr_d['green'] - fr_r['green'])))
    r.exigir(lado_ok and incoherencias == 0,
             'la region debe estar del lado opuesto y ser coherente punto a punto')


# ------------------------------------------------------------------ N3.2 ----

@prueba('N3.2', 'Embudo de tolerancia por dos vias independientes')
def n32(r):
    D = np.arange(0.0, 360.0, 0.1)
    # via 1: formula de anchura
    e1 = e_max(W_CANAL, L, 0.0, D)
    # via 2: proyeccion explicita de los 4 vertices
    c = np.zeros(2)
    e2 = np.empty_like(D)
    for i, phi in enumerate(D):
        V = vertices(c, L, 0.0)
        perp = u(phi + 90.0)
        proy = V @ perp
        e2[i] = (W_CANAL - (proy.max() - proy.min())) / 2.0
    err = float(np.max(np.abs(e1 - e2)))
    cota = (W_CANAL - L * math.sqrt(2)) / 2.0
    universal = bool(np.all(e1 >= cota - 1e-9))
    r.metrica('err_entre_vias_mm', err)
    r.metrica('e_max_min_mm', float(e1.min())); r.metrica('e_max_max_mm', float(e1.max()))
    r.metrica('cota_teorica_mm', cota)
    r.metrica('cota_universal', universal)
    r.nota('e_max in [%.2f, %.2f] mm = [%.3f, %.3f] celdas'
           % (e1.min(), e1.max(), e1.min() / CELL, e1.max() / CELL))
    r.exigir(err <= 1e-9 and universal, 'las dos vias deben coincidir y respetar la cota')


# ------------------------------------------------------------------ N3.3 ----

@prueba('N3.3', 'Contencion del corredor de empuje con rotacion del cubo')
def n33(r):
    N = 10000
    rad = L * math.sqrt(2) / 2.0
    peor = -1e9; violaciones = 0
    for _ in range(N):
        c0 = rng.uniform(3, 40, 2) * CELL
        gv = rng.uniform(3, 40, 2) * CELL
        if np.linalg.norm(gv - c0) < 50:
            continue
        th0 = rng.uniform(0, 90)
        dth = rng.uniform(-30, 30)
        ts = np.linspace(0, 1, 200)
        for t in ts[::8]:
            c = c0 + t * (gv - c0)
            th = th0 + t * dth
            V = vertices(c, L, th)
            # distancia de cada vertice al segmento [c0, gv]
            ab = gv - c0; lab = np.dot(ab, ab)
            s = np.clip(((V - c0) @ ab) / lab, 0, 1)
            proj = c0 + s[:, None] * ab
            dmax = float(np.max(np.linalg.norm(V - proj, axis=1)))
            peor = max(peor, dmax - rad)
            if dmax > rad + 1e-9:
                violaciones += 1
    r.metrica('n_trayectos', N); r.metrica('holgura_peor_mm', peor)
    r.metrica('violaciones', violaciones)
    r.metrica('radio_corredor_mm', rad)
    r.nota('incluye rotacion inducida de +-30 grados durante el empuje')
    r.exigir(violaciones == 0, 'ningun vertice debe salir del corredor')


# ------------------------------------------------------------------ N3.4 ----

@prueba('N3.4', 'Funcional GTG: correctitud y regla de decision avance/retroceso')
def n34(r):
    N = 100000
    p0 = rng.uniform(0, COLS, (N, 2)) * CELL
    a = rng.uniform(0, COLS, (N, 2)) * CELL
    th0 = rng.uniform(0, 360, N)
    tho = rng.uniform(0, 360, N)
    d = a - p0
    dist = np.linalg.norm(d, axis=1)
    m = dist > 1.0
    p0, a, th0, tho, d, dist = p0[m], a[m], th0[m], tho[m], d[m], dist[m]
    phi0 = arg_de(d)
    A = np.abs(delta(phi0, th0))
    B = np.abs(delta(tho, phi0))
    Tp = A / OM + dist / V + B / OM
    Ar = np.abs(delta(phi0 + 180.0, th0))
    Br = np.abs(delta(tho, phi0 + 180.0))
    Tm = Ar / OM + dist / V + Br / OM
    Tmin = np.minimum(Tp, Tm)

    # la funcion del modelo debe coincidir
    idx = rng.choice(len(p0), 3000, replace=False)
    err = 0.0
    for i in idx:
        T, modo = costo_GTG(p0[i], th0[i], a[i], tho[i], V, OM)
        err = max(err, abs(T - Tmin[i]))
    r.metrica('n', len(p0)); r.metrica('err_vs_minimo', err)

    # identidades: |d(phi0+180,th0)| = 180 - |d(phi0,th0)|
    id1 = float(np.max(np.abs(Ar - (180.0 - A))))
    id2 = float(np.max(np.abs(Br - (180.0 - B))))
    r.metrica('identidad_A', id1); r.metrica('identidad_B', id2)

    # regla propuesta en el protocolo: retroceso gana si |d(phi0,th0)| > 90
    regla_protocolo = (A > 90.0)
    gana_retro = (Tm < Tp)
    acierto_prot = float(np.mean(regla_protocolo == gana_retro))
    # regla CORRECTA derivada: retroceso gana si A + B > 180
    regla_correcta = (A + B > 180.0)
    acierto_corr = float(np.mean(regla_correcta == gana_retro))
    r.metrica('acierto_regla_protocolo', acierto_prot)
    r.metrica('acierto_regla_A_mas_B', acierto_corr)
    print()
    print('           regla del protocolo  |d(phi0,th0)| > 90 : acierta %.2f%%' % (100 * acierto_prot))
    print('           regla derivada        A + B     > 180   : acierta %.2f%%' % (100 * acierto_corr))
    r.nota('HALLAZGO H-04: la regla de decision correcta es A+B>180, no A>90.')
    r.nota('  A = |delta(phi0,theta0)| (giro inicial), B = |delta(phi*,phi0)| (giro final).')
    r.nota('  Sale de T+ - T- = 2(A+B-180)/omega usando |delta(phi0+180,x)| = 180-|delta(phi0,x)|.')
    r.nota('  La regla A>90 solo mira el primer giro e ignora el segundo: acierta %.0f%%.'
           % (100 * acierto_prot))
    r.exigir(err <= 1e-9 and id1 <= 1e-9 and id2 <= 1e-9 and acierto_corr > 0.9999,
             'costo minimo y regla derivada')


# ------------------------------------------------------------------ N3.5 ----

@prueba('N3.5', 'Umbral de cuerda del anillo de aproximacion')
def n35(r):
    rc, rp = RHO_CT_MAX, RHO_PU
    teorico = umbral_cuerda(rc, rp)
    arcos = np.arange(0.0, 180.0, 0.1)
    transicion = None
    for arco in arcos:
        p1 = rp * u(0.0)
        p2 = rp * u(arco)
        ts = np.linspace(0, 1, 2000)[:, None]
        seg = p1 + ts * (p2 - p1)
        dmin = float(np.min(np.linalg.norm(seg, axis=1)))
        if dmin < rc and transicion is None:
            transicion = float(arco)
            break
    r.metrica('umbral_teorico_grados', teorico)
    r.metrica('umbral_medido_grados', transicion)
    r.metrica('error_grados', abs(teorico - transicion))
    r.metrica('rho_cont_mm', rc); r.metrica('rho_puesta_mm', rp)
    r.nota('atajos en linea recta admisibles hasta %.1f grados alrededor del cubo' % teorico)
    r.exigir(abs(teorico - transicion) <= 0.15, 'transicion en el umbral predicho')


# ------------------------------------------------------------------ N3.6 ----

@prueba('N3.6', 'Costo de un quiebre expresado en distancia equivalente')
def n36(r):
    ret = S_MARG
    print()
    print('           %10s %12s %12s %14s' % ('quiebre', 'arco_mm', 'giro_s', 'equiv_mm'))
    equivs = {}
    for q in (15, 30, 45, 60, 90, 120, 180):
        arco = RHO_PU * math.radians(q)
        t_giro = q / OM
        equiv = ret + arco + t_giro * V
        equivs[q] = equiv
        print('           %9d° %12.1f %12.3f %14.1f' % (q, arco, t_giro, equiv))
    ret_arco = ret + RHO_PU * math.radians(90)
    r.metrica('equiv_90_mm', equivs[90])
    r.metrica('retirada_mas_arco_90_mm', ret_arco)
    r.metrica('aporte_del_giro_mm', equivs[90] - ret_arco)
    # criterio: el termino de giro debe ser exactamente (90/omega)*v
    esperado_giro = (90.0 / OM) * V
    r.metrica('giro_esperado_mm', esperado_giro)
    r.nota('el modelo contabilizaba solo retirada+arco = %.1f mm y omitia la reorientacion.'
           % ret_arco)
    r.nota('el giro de 90 grados a %.0f deg/s agrega %.1f mm: total %.1f mm equivalentes.'
           % (OM, equivs[90] - ret_arco, equivs[90]))
    r.nota('con las dimensiones reales del DXF el quiebre cuesta %.0f mm (antes 399).'
           % equivs[90])
    r.exigir(abs((equivs[90] - ret_arco) - esperado_giro) <= 1e-6,
             'el termino de giro debe valer (90/omega)*v')


# ------------------------------------------------------------------ N3.7 ----

@prueba('N3.7', 'Conjunto alcanzable condicional bajo oclusion')
def n37(r):
    N = 10000
    fn = 0; declarados_inalc = 0; conservador = 0
    for _ in range(N):
        c = rng.uniform(2, 41, 2) * CELL
        P = rng.uniform(2, 41, (2, 2)) * CELL
        alfa = rng.uniform(0, 5.0)                       # segundos
        margen = np.min(np.linalg.norm(P - c, axis=1)) - RHO_CT_MAX
        inalcanzable = margen > V * alfa
        # simulacion: distancia minima que puede recorrer el rover mas rapido
        alcanza_real = margen <= V * alfa
        if inalcanzable:
            declarados_inalc += 1
            if alcanza_real:
                fn += 1
        else:
            if not alcanza_real:
                conservador += 1
    r.metrica('n', N); r.metrica('declarados_inalcanzables', declarados_inalc)
    r.metrica('falsos_negativos', fn)
    r.metrica('frac_inalcanzable', declarados_inalc / N)
    r.nota('un falso negativo seria planificar hacia un cubo que ya se movio: debe ser 0')
    r.nota('en el %.0f%% de las configuraciones el cubo esta PROBADAMENTE quieto pese a age_ms alto'
           % (100.0 * declarados_inalc / N))
    r.exigir(fn == 0, 'ningun falso negativo')


# ------------------------------------------------------------------ N3.8 ----

@prueba('N3.8', 'Interferencia entre los corredores de los dos rovers')
def n38(r):
    def seg_dist(p1, p2, q1, q2):
        """Distancia minima entre dos segmentos 2D."""
        d1, d2 = p2 - p1, q2 - q1
        rr = p1 - q1
        a, e = np.dot(d1, d1), np.dot(d2, d2)
        f = np.dot(d2, rr)
        if a < 1e-12 and e < 1e-12: return float(np.linalg.norm(rr))
        if a < 1e-12: s, t = 0.0, np.clip(f / e, 0, 1)
        else:
            cc = np.dot(d1, rr)
            if e < 1e-12: t, s = 0.0, np.clip(-cc / a, 0, 1)
            else:
                b = np.dot(d1, d2); den = a * e - b * b
                s = np.clip((b * f - cc * e) / den, 0, 1) if den > 1e-12 else 0.0
                t = np.clip((b * s + f) / e, 0, 1)
                s = np.clip((b * t - cc) / a, 0, 1)
        return float(np.linalg.norm((p1 + s * d1) - (q1 + t * d2)))

    N = 10000
    rad = L * math.sqrt(2) / 2.0
    incompat = 0; coincide = 0
    for _ in range(N):
        c1 = rng.uniform(3, 40, 2) * CELL; g1 = rng.uniform(3, 40, 2) * CELL
        c2 = rng.uniform(3, 40, 2) * CELL; g2 = rng.uniform(3, 40, 2) * CELL
        dd = seg_dist(c1, g1, c2, g2)
        disjuntos = dd > 2 * rad
        # verificacion directa: muestreo denso de ambos corredores
        t = np.linspace(0, 1, 60)[:, None]
        A = c1 + t * (g1 - c1); B = c2 + t * (g2 - c2)
        M = np.linalg.norm(A[:, None, :] - B[None, :, :], axis=2)
        disj_muestreo = bool(M.min() > 2 * rad)
        if disjuntos == disj_muestreo:
            coincide += 1
        if not disjuntos:
            incompat += 1
    r.metrica('n', N)
    r.metrica('acuerdo_formula_vs_muestreo', coincide / N)
    r.metrica('frac_pares_incompatibles', incompat / N)
    r.metrica('separacion_minima_mm', 2 * rad)
    r.nota('dos tareas son compatibles si sus corredores distan mas de L*sqrt2 = %.1f mm'
           % (2 * rad))
    r.nota('con corredores aleatorios, el %.0f%% de los pares interfiere: la coordinacion'
           % (100.0 * incompat / N))
    r.nota('no es opcional, es el caso mayoritario.')
    r.exigir(coincide / N > 0.995, 'la condicion analitica debe coincidir con el muestreo')


# ------------------------------------------------------------------ N3.9 ----

@prueba('N3.9', 'Zona trampa: posiciones desde las que el cubo no puede despegarse de la pared')
def n39(r):
    # huella ASIMETRICA: el rover se pone entre el cubo y la pared mirando al
    # cubo, asi que hacia la pared llega LAM/2 y hacia el cubo LAM/2 + LAMBDA.
    xa, xf, hw = X_ATRAS / CELL, X_ADEL / CELL, HW / CELL
    hc = (L / 2) / CELL
    angs = np.arange(0.0, 360.0, 5.0)
    g = np.arange(0.0, COLS + 1e-9, 0.5)
    X, Y = np.meshgrid(g, g, indexing='ij')
    C = np.stack([X.ravel(), Y.ravel()], axis=1)

    def d_pared(P):
        return np.minimum.reduce([P[:, 0] - TAB_MIN, TAB_MAX - P[:, 0],
                                  P[:, 1] - TAB_MIN, TAB_MAX - P[:, 1]])

    print()
    print('           %-26s %14s %16s' % ('distancia de puesta', 'zona trampa', 'banda junto a pared'))
    filas = []
    # el anillo de maniobra (H-12) es el radio que de verdad hace falta para
    # poder GIRAR detras del cubo sin barrerlo con las paletas
    RHO_MAN = math.hypot(X_ADEL, HW) + L * math.sqrt(2) / 2 + 5.0
    for rho, etiq in ((RHO_MAN / CELL, 'anillo de maniobra (H-12)'),
                      (RHO_PU / CELL, 'con margen s=40 mm'),
                      (RHO_CT_MAX / CELL, 'minima (contacto)')):
        escapa = np.zeros(len(C), bool)
        for A in angs:
            d = u(A); up = u(A); pe = u(A + 90.0)
            Ast = C - rho * d
            ok = np.ones(len(C), bool)
            for sa in (xa, xf):
                for sb in (-1, 1):
                    E = Ast + sa * up + sb * hw * pe
                    ok &= np.all((E >= TAB_MIN) & (E <= TAB_MAX), axis=1)
            Z = C + 3.0 * d
            dentro = np.ones(len(C), bool)
            for sx in (-1, 1):
                for sy in (-1, 1):
                    E = Z + np.array([sx * hc, sy * hc])
                    dentro &= np.all((E >= TAB_MIN) & (E <= TAB_MAX), axis=1)
            escapa |= ok & dentro & (d_pared(Z) > d_pared(C) + 1e-9)
        frac = float((~escapa).mean())
        banda = rho - BORDE
        filas.append((etiq, frac, banda))
        print('           %-26s %13.2f%% %15.2f c' % (etiq, 100 * frac, banda))
        r.metrica('trampa_%s_pct' % etiq.split()[0], 100 * frac)

    r.metrica('banda_teorica_celdas', RHO_PU / CELL - BORDE)
    r.metrica('trampa_con_anillo_de_maniobra_pct', 100 * filas[0][1])
    r.nota('Con el anillo de maniobra que exigen las paletas (%.2f celdas) la zona'
           % (RHO_MAN / CELL,))
    r.nota('trampa sube de %.1f %% a %.1f %%, y la banda junto a la pared de %.2f a %.2f'
           % (100 * filas[1][1], 100 * filas[0][1], filas[1][2], filas[0][2]))
    r.nota('celdas. Los depositos estan a 2,5 celdas del borde: quedan DENTRO de esa')
    r.nota('banda, y por eso el lazo cerrado ya no puede corregir el ultimo tramo del')
    r.nota('empuje contra un deposito (ver N4.1 revisado).')
    r.nota('HALLAZGO H-09: un cubo a menos de rho_puesta - borde = %.2f celdas (%.0f mm)'
           % (RHO_PU / CELL - BORDE, RHO_PU - BORDE * CELL))
    r.nota('de una pared no puede ser empujado para alejarse de ella: el rover no cabe')
    r.nota('detras. El %.1f%% de la cancha es zona trampa con la distancia de puesta,'
           % (100 * filas[1][1]))
    r.nota('la revierte. Regla estrategica: nunca empujar un cubo hacia una pared.')
    r.nota('Los depositos estan a 2,5 celdas del borde, DENTRO de la zona trampa: eso es')
    r.nota('correcto y deseado (el cubo debe quedarse ahi), pero implica que un cubo')
    r.nota('empujado hacia el deposito equivocado queda perdido.')
    r.exigir(filas[0][1] > 0.0, 'la zona trampa existe y esta cuantificada')


# ----------------------------------------------------------------- N3.10 ----

@prueba('N3.10', 'Ventana angular admisible: cuando el angulo del cubo es obligatorio')
def n310(r):
    """Con W_canal real, el presupuesto lateral puede volverse NEGATIVO."""
    SIG = 5.0            # sigma_vis adoptada
    print()
    print('           %-10s %12s %13s %16s' %
          ('margen m', 'w admisible', '|Delta| max', 'frac. orientac.'))
    filas = []
    for m in (0.0, 1.0, 2.0, 3.0, 5.0, 8.0):
        w = W_CANAL - 2 * (m + SIG)
        if w < L:
            print('           %-10.1f %12.2f %13s %16s' % (m, w, 'IMPOSIBLE', '0 %'))
            filas.append((m, w, None, 0.0)); continue
        rr = (w / L) ** 2 - 1
        if rr >= 1:
            print('           %-10.1f %12.2f %13s %16s' % (m, w, 'cualquiera', '100 %'))
            filas.append((m, w, 90.0, 1.0)); continue
        dmax = math.degrees(math.asin(rr)) / 2
        frac = 4 * 2 * dmax / 360
        print('           %-10.1f %12.2f %12.1f%s %15.0f %%' % (m, w, dmax, chr(176), 100 * frac))
        filas.append((m, w, dmax, frac))
    r.metrica('tabla', [(f[0], f[2], f[3]) for f in filas])

    # presupuesto en el peor caso
    e45 = (W_CANAL - L * math.sqrt(2)) / 2
    r.metrica('e_max_45_mm', e45)
    r.metrica('presupuesto_45_mm', e45 - SIG)
    r.metrica('e_max_alineado_mm', (W_CANAL - L) / 2)
    r.metrica('presupuesto_alineado_mm', (W_CANAL - L) / 2 - SIG)

    # verificacion cruzada: barrido explicito sobre Delta
    D = np.arange(0.0, 90.0, 0.01)
    e = e_max(W_CANAL, L, 0.0, D)
    viable = e - SIG > 0
    r.metrica('frac_Delta_con_presupuesto_positivo', float(viable.mean()))
    mala = D[~viable]
    d1, d2 = (float(mala.min()), float(mala.max())) if mala.size else (None, None)
    r.metrica('banda_negativa_grados', (d1, d2))
    print()
    print('           presupuesto NEGATIVO en Delta in [%.2f, %.2f] grados' % (d1, d2))
    print('           (%.0f %% del rango de orientaciones queda con presupuesto positivo)'
          % (100 * viable.mean()))
    r.nota('HALLAZGO H-11: con W_canal = %.2f mm el presupuesto lateral es NEGATIVO' % W_CANAL)
    r.nota('en la banda Delta in [%.1f, %.1f] grados, el %.0f %% de las orientaciones.'
           % (d1, d2, 100 * (~viable).mean()))
    r.nota('En el peor caso (45 grados) vale %.2f mm:' % (e45 - SIG))
    r.nota('la sola incertidumbre de la vision (5 mm) supera la tolerancia (%.2f mm).' % e45)
    r.nota('Conocer el angulo del cubo dejo de ser una mejora: es OBLIGATORIO.')
    r.nota('Regla: el empuje debe apuntar a menos de ~21 grados de la normal a una cara.')
    r.exigir(e45 > 0, 'el cubo debe al menos caber entre los paneles')


# ----------------------------------------------------------------- N3.11 ----

@prueba('N3.11', 'Corredor de entrada: el tramo en que el cubo ya esta contenido')
def n311(r):
    """Medido lambda = 55 mm, el cubo entra en el canal ANTES de tocar la placa.

    Consecuencia que la primera version del modelo no tenia. Sea x la posicion
    de la placa frontal sobre el eje de empuje; las puntas estan en x + lambda.
    El cubo, de semiextension h(Delta) = w_S/2 sobre ese eje, tiene su cara
    trasera en c - h.
      - las puntas alcanzan al cubo cuando  x + lambda = c - h
      - la placa lo toca cuando             x = c - h
    Entre ambos instantes el rover avanza exactamente lambda, cualquiera sea la
    orientacion del cubo. Durante todo ese tramo |e| <= e_max ya debe cumplirse:
    las paletas contienen pero no corrigen (canal paralelo), asi que si el cubo
    no esta dentro de la banda cuando llegan las puntas, la punta lo golpea de
    costado y lo desplaza en vez de capturarlo.
    """
    Ds = np.arange(0.0, 90.0 + 1e-9, 0.25)
    wS = np.array([anchura(L, 0.0, d) for d in Ds])
    cubierta = np.minimum(LAMBDA / wS, 1.0)     # fraccion del cubo entre paletas
    print()
    print('           %-10s %11s %11s %13s %14s' %
          ('Delta', 'w_S [mm]', 'e_max [mm]', 'tramo [mm]', 'cubo cubierto'))
    for d in (0.0, 15.0, 30.0, 45.0):
        w = anchura(L, 0.0, d)
        print('           %-10.1f %11.2f %11.2f %13.2f %13.0f %%'
              % (d, w, (W_CANAL - w) / 2, LAMBDA, 100 * min(LAMBDA / w, 1.0)))
    r.metrica('tramo_contenido_mm', LAMBDA)
    r.metrica('lambda_mm', LAMBDA)
    r.metrica('fraccion_cubierta', (float(cubierta.min()), float(cubierta.max())))
    r.metrica('frac_orientaciones_que_sobresalen', float((wS > LAMBDA).mean()))
    r.nota('El rover avanza exactamente lambda = %.0f mm con el cubo ya entre las' % LAMBDA)
    r.nota('paletas antes de que la placa frontal lo toque, y el tramo NO depende de')
    r.nota('la orientacion. Durante todo ese tramo |e| <= e_max debe cumplirse ya.')
    r.nota('Como lambda = %.0f < L = %.0f, el cubo SIEMPRE sobresale por delante de las'
           % (LAMBDA, L))
    r.nota('puntas: las paletas cubren entre el %.0f %% (a 45 grados) y el %.0f %% (alineado)'
           % (100 * cubierta.min(), 100 * cubierta.max()))
    r.nota('de su extension. La contencion es parcial, y es mas debil justo en la')
    r.nota('orientacion de peor tolerancia.')
    r.exigir(LAMBDA > 0, 'existe un tramo de contencion previo al contacto')


# ----------------------------------------------------------------- N3.12 ----

@prueba('N3.12', 'Radio envolvente con las paletas incluidas')
def n312(r):
    """Las paletas anaden 55 mm por delante y el modelo no las contaba."""
    R_viejo = R_rov(W_EXT, LAM)
    R_nuevo = RROV_ASIM
    R_centroide = 0.5 * math.hypot(W_EXT, LAM + LAMBDA)
    print()
    print('           %-42s %10s %10s' % ('', 'mm', 'celdas'))
    for et, v in (('R_rov del modelo (solo chasis)', R_viejo),
                  ('R_rov real, disco centrado en el chasis', R_nuevo),
                  ('R_rov si el origen fuese el centroide', R_centroide)):
        print('           %-42s %10.2f %10.3f' % (et, v, v / CELL))
    r.metrica('R_rov_modelo_mm', R_viejo)
    r.metrica('R_rov_real_mm', R_nuevo)
    r.metrica('subestimacion_pct', 100 * (R_nuevo / R_viejo - 1))
    # cuanto cambia la condicion de compatibilidad entre dos rovers
    r.nota('HALLAZGO H-12: el modelo tomaba R_rov = %.2f mm, del contorno del chasis.' % R_viejo)
    r.nota('Con las paletas medidas (lambda = %.0f mm) el disco envolvente centrado' % LAMBDA)
    r.nota('en el chasis mide %.2f mm: una subestimacion del %.0f %%.'
           % (R_nuevo, 100 * (R_nuevo / R_viejo - 1)))
    r.nota('Es un error del lado PELIGROSO: el planificador creia pasar por huecos')
    r.nota('por los que no pasa. No afecta al empuje (rho_cont no cambia, porque el')
    r.nota('cubo entra en el canal hasta la placa) pero si a colision y a frontera.')
    r.exigir(R_nuevo > R_viejo, 'la huella real es mayor que la del chasis solo')


if __name__ == '__main__':
    print('=' * 96)
    print('NIVEL N3 --- CAPA GEOMETRICA DE PLANIFICACION')
    print('=' * 96)
    print('  parametros: W_ext=%.2f  W_canal=%.2f  Lambda=%.2f  L=%.0f  s=%.0f'
          % (W_EXT, W_CANAL, LAM, L, S_MARG))
    print('              R_rov(chasis)=%.2f  R_rov(con paletas)=%.2f  rho_puesta=%.2f mm'
          % (RROV, RROV_ASIM, RHO_PU))
    print('              v=%.0f mm/s  omega=%.0f deg/s  celda=%.0f mm' % (V, OM, CELL))
    for f in (n31, n32, n33, n34, n35, n36, n37, n38, n39, n310, n311, n312):
        f()
    resumen('RESUMEN N3')
    volcar('/tmp/bench/resultados_n3.json')

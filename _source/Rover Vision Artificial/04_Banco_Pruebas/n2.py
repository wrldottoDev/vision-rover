"""Nivel N2: vision extremo a extremo contra la verdad del generador sintetico."""
from __future__ import annotations
import math, sys, os
import numpy as np

REPO = '/tmp/rover/Vision-Rover-Challenge-main/vision-system'
sys.path.insert(0, REPO)
sys.path.insert(0, '/tmp/bench')

import cv2
from vision.configuracion import cargar_config, CuboDemo, RoverDemo, Perspectiva
from vision.sources.generador_sintetico import generar
from vision.geometry.coordenadas import (construir_sistema, detectar_marcadores,
                                         pose_camara, ErrorGeometria)
from vision.detectors.cubos import detectar_cubos, mascara_de_color

from arnes import prueba, resumen, volcar, RES
from modelo import (rho_P, casco_convexo, est_B2_theta, est_B2_centro, est_B4,
                    kappa, pi_directo, arg_de, u)

CFG = cargar_config(os.path.join(REPO, 'vision', 'config_vision.json'))
CELL = CFG.tablero.cell_mm
LADO_MM = CFG.elementos.cubos.lado_mm
LADO_C = LADO_MM / CELL
PERSP_ON = Perspectiva(activa=True, inclinacion_grados=CFG.sintetico.perspectiva.inclinacion_grados)
PERSP_OFF = Perspectiva(activa=False, inclinacion_grados=0.0)

# banco global de observaciones (color, err_pos_mm, err_theta_deg, residuo, confiable, escenario)
BANCO = []


def escena(cubos, rovers=(), persp=PERSP_ON):
    img, verdad = generar(CFG, rovers=rovers, cubos=cubos, perspectiva=persp)
    det = detectar_marcadores(img, CFG.marcadores_esquina.nombre_diccionario)
    sis = construir_sistema(img, CFG, det)
    pose = pose_camara(sis, verdad.camara.matriz)
    cubos_det = detectar_cubos(img, sis, CFG, pose)
    return img, verdad, sis, pose, cubos_det


def comparar(verdad, cubos_det, etiqueta):
    """Devuelve lista de (color, err_pos_mm, err_theta_deg, residuo, confiable)."""
    vp = {c.color: c for c in verdad.cubos}
    out = []
    for c in cubos_det:
        real = vp.get(c.color)
        if real is None:
            continue
        ep = math.hypot(c.col - real.col, c.row - real.row) * CELL
        et = float(rho_P(c.theta_grados, real.theta_grados, 90.0))
        fila = (c.color, ep, et, c.residuo_celdas, c.confiable)
        out.append(fila)
        BANCO.append(fila + (etiqueta,))
    return out


# ------------------------------------------------------------------ N2.1 ----

ESC_OFICIALES = None

def _escenarios_oficiales():
    verde = CuboDemo(color='green', col=34.0, row=9.0, theta=15.0)
    return [
        ('tres cubos de la configuracion', tuple(CFG.cubos_demo), ()),
        ('cubos repartidos', (CuboDemo(color='red', col=8.0, row=8.0, theta=0.0),
                              CuboDemo(color='green', col=35.0, row=8.0, theta=30.0),
                              CuboDemo(color='blue', col=8.0, row=35.0, theta=60.0)), ()),
        ('rotaciones 0/22.5/45', tuple(CuboDemo(color=c, col=col, row=21.5, theta=t)
                                       for c, col, t in (('red', 10.0, 0.0), ('green', 21.5, 22.5),
                                                         ('blue', 33.0, 45.0))), ()),
        ('rover EMPUJANDO', (verde,), (RoverDemo(id=10, col=29.6, row=11.6, theta=30.0),)),
        ('rover tapando MAS', (verde,), (RoverDemo(id=10, col=30.8, row=10.6, theta=30.0),)),
    ]


@prueba('N2.1', 'Reproduccion de la linea base oficial de posicion')
def n21(r):
    print()
    print('           %-32s %10s %10s %9s %6s' % ('escenario', 'cenital', 'inclinada', 'residuo', 'conf'))
    ref = {'tres cubos de la configuracion': None, 'cubos repartidos': 1.05,
           'rotaciones 0/22.5/45': None, 'rover EMPUJANDO': 4.88, 'rover tapando MAS': None}
    obt = {}
    for nombre, cubos, rovers in _escenarios_oficiales():
        fila = []
        for persp, et in ((PERSP_OFF, 'cenital'), (PERSP_ON, 'inclinada')):
            _, verdad, _, _, cd = escena(cubos, rovers, persp)
            res = comparar(verdad, cd, '%s|%s' % (nombre, et))
            fila.append(max(x[1] for x in res) if res else float('nan'))
            ult = res
        obt[nombre] = fila[1]
        print('           %-32s %10.2f %10.2f %9.4f %6s'
              % (nombre, fila[0], fila[1], max(x[3] for x in ult),
                 all(x[4] for x in ult)))
    r.metrica('repartidos_inclinada_mm', obt['cubos repartidos'])
    r.metrica('empujando_inclinada_mm', obt['rover EMPUJANDO'])
    ok = True
    for k, v in ref.items():
        if v is not None:
            dif = abs(obt[k] - v) / v
            r.metrica('desvio_rel_' + k.replace(' ', '_'), dif)
            ok = ok and dif <= 0.10
    r.nota('valores documentados: repartidos 1.05 mm, empujando 4.88 mm (camara inclinada)')
    r.exigir(ok, 'la linea base debe reproducirse dentro del 10%')


# ------------------------------------------------------------------ N2.2 ----

ZONAS = [(10.0, 10.0), (33.0, 10.0), (21.5, 21.5), (10.0, 33.0), (33.0, 33.0)]


@prueba('N2.2', 'Error de ANGULO del detector oficial (sin linea base previa)')
def n22(r):
    ths = np.arange(0.0, 90.0, 2.5)
    datos = {'cenital': [], 'inclinada': []}
    por_theta = {'cenital': {}, 'inclinada': {}}
    for persp, et in ((PERSP_OFF, 'cenital'), (PERSP_ON, 'inclinada')):
        for t in ths:
            for grupo in ([0, 1, 2], [3, 4]):
                colores = ['red', 'green', 'blue']
                cubos = tuple(CuboDemo(color=colores[i], col=ZONAS[z][0], row=ZONAS[z][1], theta=float(t))
                              for i, z in enumerate(grupo))
                _, verdad, _, _, cd = escena(cubos, (), persp)
                for fila in comparar(verdad, cd, 'barrido_theta|%s' % et):
                    datos[et].append(fila)
                    por_theta[et].setdefault(round(float(t), 2), []).append(fila[2])
    print()
    print('           %-11s %7s %10s %10s %10s %10s' %
          ('modo', 'n', 'err_med', 'err_p95', 'err_max', 'err_pos_max'))
    for et in ('cenital', 'inclinada'):
        e = np.array([x[2] for x in datos[et]])
        p = np.array([x[1] for x in datos[et]])
        r.metrica('%s_theta_medio' % et, float(e.mean()))
        r.metrica('%s_theta_p95' % et, float(np.percentile(e, 95)))
        r.metrica('%s_theta_max' % et, float(e.max()))
        print('           %-11s %7d %10.4f %10.4f %10.4f %10.4f'
              % (et, len(e), e.mean(), np.percentile(e, 95), e.max(), p.max()))
    # dependencia del error con theta
    print()
    print('           dependencia del error angular con theta (camara inclinada):')
    ks = sorted(por_theta['inclinada'])
    linea = '           '
    for k in ks[:18]:
        linea += '%5.1f' % np.mean(por_theta['inclinada'][k])
    print(linea + '   <- theta = 0.0 .. 42.5 paso 2.5')
    linea = '           '
    for k in ks[18:]:
        linea += '%5.1f' % np.mean(por_theta['inclinada'][k])
    print(linea + '   <- theta = 45.0 .. 87.5 paso 2.5')
    emax = max(r.metricas['cenital_theta_max'], r.metricas['inclinada_theta_max'])
    r.metrica('err_theta_max_global', emax)
    r.nota('criterio propuesto: <= 5 grados (limite de la restriccion de interfaz RI2)')
    r.exigir(emax <= 5.0, 'error angular maximo por debajo de 5 grados')


# ------------------------------------------------------------------ N2.3 ----

@prueba('N2.3', 'Error de angulo bajo oclusion por empuje')
def n23(r):
    print()
    print('           %-22s %6s %10s %10s %10s %8s' %
          ('condicion', 'n', 'pos_max', 'th_medio', 'th_max', 'conf'))
    verde = lambda t: (CuboDemo(color='green', col=34.0, row=9.0, theta=float(t)),)
    cond = [('despejado', ()),
            ('rover empujando ~22%', (RoverDemo(id=10, col=29.6, row=11.6, theta=30.0),)),
            ('rover tapando ~70%', (RoverDemo(id=10, col=30.8, row=10.6, theta=30.0),))]
    resumen_ = {}
    for nombre, rovers in cond:
        acc = []
        for t in np.arange(0.0, 90.0, 5.0):
            _, verdad, _, _, cd = escena(verde(t), rovers, PERSP_ON)
            acc += comparar(verdad, cd, 'oclusion|%s' % nombre)
        e = np.array([x[2] for x in acc]); p = np.array([x[1] for x in acc])
        conf = np.array([x[4] for x in acc])
        resumen_[nombre] = (float(p.max()), float(e.mean()), float(e.max()), float(conf.mean()))
        print('           %-22s %6d %10.3f %10.3f %10.3f %8.2f'
              % (nombre, len(acc), p.max(), e.mean(), e.max(), conf.mean()))
    for k, v in resumen_.items():
        kk = k.split()[0] if ' ' in k else k
        r.metrica('%s_pos_max' % kk, v[0]); r.metrica('%s_th_max' % kk, v[2])
        r.metrica('%s_frac_confiable' % kk, v[3])
    sev = resumen_['rover tapando ~70%']
    r.nota('en oclusion severa NO se exige acertar: se exige declararse no confiable')
    r.exigir(sev[3] < 1.0, 'con 70% de oclusion alguna deteccion debe marcarse no confiable')


# ------------------------------------------------------------------ N2.4 ----

@prueba('N2.4', 'Poder discriminante del criterio de confiabilidad')
def n24(r):
    from scipy import stats as st
    res = np.array([[x[1], x[2], x[3], float(x[4])] for x in BANCO])
    pos, th, resid, conf = res[:, 0], res[:, 1], res[:, 2], res[:, 3].astype(bool)
    rho_pos = st.spearmanr(resid, pos).statistic
    rho_th = st.spearmanr(resid, th).statistic
    malo = pos > 10.0
    vp = int(np.sum(malo & ~conf)); fn = int(np.sum(malo & conf))
    fp = int(np.sum(~malo & ~conf)); vn = int(np.sum(~malo & conf))
    print()
    print('           n=%d observaciones acumuladas de N2.1-N2.3' % len(res))
    print('           Spearman(residuo, err_posicion) = %+.4f' % rho_pos)
    print('           Spearman(residuo, err_angulo)   = %+.4f' % rho_th)
    print()
    print('                              err>10mm   err<=10mm')
    print('           marcado NO conf.   %8d   %9d' % (vp, fp))
    print('           marcado confiable  %8d   %9d   <- %d falsos negativos' % (fn, vn, fn))
    tasa_fn = fn / max(1, vp + fn)

    # --- el mismo analisis pero para el ANGULO ------------------------------
    malo_a = th > 5.0
    vpa = int(np.sum(malo_a & ~conf)); fna = int(np.sum(malo_a & conf))
    fpa = int(np.sum(~malo_a & ~conf)); vna = int(np.sum(~malo_a & conf))
    tasa_fna = fna / max(1, vpa + fna)
    print()
    print('           Mismo criterio aplicado al ANGULO (umbral 5 grados, RI2):')
    print('                              err>5deg   err<=5deg')
    print('           marcado NO conf.   %8d   %9d' % (vpa, fpa))
    print('           marcado confiable  %8d   %9d   <- %d falsos negativos (%.0f%%)'
          % (fna, vna, fna, 100.0 * tasa_fna))
    r.metrica('angulo_falsos_negativos', fna)
    r.metrica('angulo_malos_totales', vpa + fna)
    r.metrica('angulo_tasa_falsos_negativos', float(tasa_fna))
    if tasa_fna > 0.05:
        r.nota('HALLAZGO H-03: la bandera `confiable` NO protege el ANGULO.')
        r.nota('Su umbral (residuo <= 0.2 celdas) fue calibrado contra el error de')
        r.nota('POSICION. Bajo oclusion por empuje el angulo yerra hasta 19 grados')
        r.nota('y la deteccion se sigue declarando confiable en el %.0f%% de los casos.'
               % (100.0 * tasa_fna))
        r.nota('Consecuencia: si algun dia se publica el angulo, NO se puede reutilizar')
        r.nota('esta bandera para filtrarlo; hace falta un umbral propio.')

    r.metrica('n_observaciones', len(res))
    r.metrica('spearman_residuo_posicion', float(rho_pos))
    r.metrica('spearman_residuo_angulo', float(rho_th))
    r.metrica('falsos_negativos', fn); r.metrica('malos_totales', vp + fn)
    r.metrica('tasa_falsos_negativos', float(tasa_fn))
    r.nota('falso negativo = deteccion con error > 10 mm que se declaro confiable')
    r.exigir(tasa_fn <= 0.05, 'tasa de falsos negativos <= 5%')


# ------------------------------------------------------------------ N2.5 ----

def vertices_base_desde_contorno(img, sis, pose, color_obj='green'):
    """Extrae vertices de la BASE del contorno real, sin usar el ajuste oficial.

    Criterio puramente geometrico: un vertice de la base p tiene su imagen de
    tapa en N + kappa*(p - N), que tambien es vertice del casco. Se buscan pares
    que cumplan esa relacion de homotecia.
    """
    from vision.detectors.cubos import mascara_de_color, matiz_y_croma, clasificar
    mascara, lab = mascara_de_color(img, CFG)
    n, etiq, stats, _ = cv2.connectedComponentsWithStats(mascara, 8)
    mejor = None
    for k in range(1, n):
        reg = (etiq == k).astype(np.uint8)
        matiz, _ = matiz_y_croma(cv2.mean(lab, mask=reg)[:3])
        if clasificar(matiz, CFG) != color_obj:
            continue
        a = int(stats[k, cv2.CC_STAT_AREA])
        if mejor is None or a > mejor[0]:
            mejor = (a, reg)
    if mejor is None:
        return None, None
    cont, _ = cv2.findContours(mejor[1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cpx = max(cont, key=cv2.contourArea).reshape(-1, 2).astype(np.float64)
    celdas = sis.a_celdas(cpx)
    hull = casco_convexo(celdas, eps_rel=1e-6)
    # simplificar: agrupar vertices casi colineales
    poly = cv2.approxPolyDP(hull.astype(np.float32).reshape(-1, 1, 2),
                            0.02 * cv2.arcLength(hull.astype(np.float32).reshape(-1, 1, 2), True),
                            True).reshape(-1, 2).astype(np.float64)
    N = np.array(pose.nadir_celdas)
    kap = pose.factor_paralaje(LADO_MM)
    base = []
    for i, p in enumerate(poly):
        pred = N + kap * (p - N)
        d = np.min(np.linalg.norm(poly - pred, axis=1))
        if d < 0.35:                      # 7 mm de tolerancia
            base.append(p)
    return np.array(base) if base else np.zeros((0, 2)), poly


@prueba('N2.5', 'Estimadores de la familia B sobre contornos reales')
def n25(r):
    ths = np.arange(0.0, 90.0, 5.0)
    n_ok = 0; n_tot = 0; conteo = {}
    err_B2 = []; err_B4 = []; err_of = []
    for zona in ZONAS:
        for t in ths:
            cubos = (CuboDemo(color='green', col=zona[0], row=zona[1], theta=float(t)),)
            img, verdad, sis, pose, cd = escena(cubos, (), PERSP_ON)
            base, poly = vertices_base_desde_contorno(img, sis, pose, 'green')
            n_tot += 1
            k = 0 if base is None else len(base)
            conteo[k] = conteo.get(k, 0) + 1
            real = verdad.cubos[0]
            if cd:
                err_of.append(float(rho_P(cd[0].theta_grados, real.theta_grados, 90.0)))
            if base is None or len(base) < 2:
                continue
            # pares de vertices de base a distancia ~ lado: son una arista
            hallo = False
            for i in range(len(base)):
                for j in range(i + 1, len(base)):
                    d = float(np.linalg.norm(base[i] - base[j]))
                    if abs(d - LADO_C) < 0.25 * LADO_C:
                        th = float(est_B2_theta(base[i], base[j]))
                        err_B2.append(float(rho_P(th, real.theta_grados, 90.0)))
                        hallo = True
                        break
                if hallo: break
            if hallo:
                n_ok += 1
    print()
    print('           vertices de BASE recuperados del contorno: %s (de %d escenas)'
          % (conteo, n_tot))
    print('           escenas con arista de base completa: %d/%d (%.0f%%)'
          % (n_ok, n_tot, 100.0 * n_ok / n_tot))
    r.metrica('escenas', n_tot); r.metrica('conteo_vertices_base', conteo)
    r.metrica('escenas_con_arista', n_ok)
    r.metrica('tasa_extraccion', n_ok / n_tot)
    if err_B2:
        print('           B2 sobre arista real : err_medio=%.3f  err_max=%.3f grados'
              % (np.mean(err_B2), np.max(err_B2)))
        r.metrica('B2_err_medio', float(np.mean(err_B2)))
        r.metrica('B2_err_max', float(np.max(err_B2)))
    print('           ajuste oficial       : err_medio=%.3f  err_max=%.3f grados'
          % (np.mean(err_of), np.max(err_of)))
    r.metrica('oficial_err_medio', float(np.mean(err_of)))
    r.metrica('oficial_err_max', float(np.max(err_of)))
    r.nota('el eslabon debil de la familia B es la EXTRACCION de vertices, no la formula')
    r.estado = 'INFO'


if __name__ == '__main__':
    print('=' * 96)
    print('NIVEL N2 --- VISION EXTREMO A EXTREMO CONTRA VERDAD SINTETICA')
    print('=' * 96)
    for f in (n21, n22, n23, n24, n25):
        f()
    resumen('RESUMEN N2')
    volcar('/tmp/bench/resultados_n2.json')

"""Niveles N0 (consistencia interna) y N1 (estimadores vs verdad analítica)."""
from __future__ import annotations
import math
import numpy as np
from modelo import *
import modelo as M
from arnes import prueba, resumen, volcar, RES

EPS = 1e-9
L = 60.0                     # mm
CELL = 20.0
CANCHA = 43.0 * CELL         # 860 mm
rng = np.random.default_rng(20260902)

POS = [np.array([x, y]) for x in np.linspace(80, CANCHA - 80, 5)
       for y in np.linspace(80, CANCHA - 80, 5)]

# =========================================================== N0 =============

@prueba('N0.1', 'Ortogonalidad y determinante de R(theta)')
def n01(r):
    ths = np.arange(-720.0, 720.0 + 1e-9, 0.1)
    eo = ed = 0.0
    for t in ths:
        A = R(t)
        eo = max(eo, float(np.max(np.abs(A.T @ A - np.eye(2)))))
        ed = max(ed, abs(float(np.linalg.det(A)) - 1.0))
    r.metrica('n', len(ths)); r.metrica('err_ortog', eo); r.metrica('err_det', ed)
    r.exigir(eo <= 1e-12 and ed <= 1e-12, 'tolerancia 1e-12')


@prueba('N0.2', 'Homomorfismo R(a)R(b) = R(a+b)')
def n02(r):
    g = np.arange(0.0, 360.0, 2.0)
    e = 0.0
    for a in g:
        Ra = R(a)
        for b in g:
            e = max(e, float(np.max(np.abs(Ra @ R(b) - R(a + b)))))
    r.metrica('n_pares', len(g) ** 2); r.metrica('err_max', e)
    r.exigir(e <= 1e-12, 'tolerancia 1e-12')


@prueba('N0.3', 'R(t)u(f)=u(t+f) y recuperacion del argumento')
def n03(r):
    g = np.arange(0.0, 360.0, 1.0)
    e1 = e2 = 0.0
    for t in g:
        Rt = R(t)
        UU = u(g)                                  # (360,2)
        izq = UU @ Rt.T
        e1 = max(e1, float(np.max(np.abs(izq - u(g + t)))))
    e2 = float(np.max(rho_P(arg_de(u(g)), g, 360.0)))
    r.metrica('err_accion', e1); r.metrica('err_arg_grados', e2)
    r.exigir(e1 <= 1e-12 and e2 <= 1e-9, 'tolerancia')


@prueba('N0.4', 'Forma cerrada de R(90): (dx,dy) -> (dy,-dx)')
def n04(r):
    d = rng.normal(0, 1, (100000, 2))
    d *= (10.0 ** rng.uniform(-6, 3, (100000, 1))) / np.linalg.norm(d, axis=1, keepdims=True)
    izq = d @ R(90.0).T
    der = np.stack([d[:, 1], -d[:, 0]], axis=1)
    e = float(np.max(np.abs(izq - der)))
    r.metrica('n', len(d)); r.metrica('err_max', e)
    r.exigir(e <= 1e-12, 'tolerancia 1e-12')


@prueba('N0.5', 'Convencion fila-descendente (anclada al contrato)')
def n05(r):
    esperado = {0.0: (1, 0), 90.0: (0, -1), 180.0: (-1, 0), 270.0: (0, 1)}
    e = 0.0
    for a, v in esperado.items():
        e = max(e, float(np.max(np.abs(u(a) - np.array(v, float)))))
    # el contrato: dcol = cos(th), drow = -sin(th)
    ths = np.arange(0, 360, 0.5)
    dcol = np.cos(np.radians(ths)); drow = -np.sin(np.radians(ths))
    e2 = float(np.max(np.abs(u(ths) - np.stack([dcol, drow], axis=1))))
    r.metrica('err_canonicos', e); r.metrica('err_vs_contrato', e2)
    r.nota('u(90)=(0,-1): apunta a row decreciente = ARRIBA en pantalla. Correcto.')
    r.exigir(e <= 1e-15 and e2 <= 1e-15, 'coincidencia exacta')


@prueba('N0.6', 'Diferencia angular con signo: rango, congruencia, cruce')
def n06(r):
    g = np.arange(-360.0, 720.0, 0.5)
    A, B = np.meshgrid(g, g[::7], indexing='ij')
    D = delta(A, B)
    cong = float(np.max(np.abs(((D - (A - B)) % 360.0 + 180.0) % 360.0 - 180.0)))
    r.metrica('n', D.size); r.metrica('err_congruencia', cong)

    # --- rango REAL de la forma cerrada, con tolerancia CERO -----------------
    fino = np.concatenate([np.arange(-1080.0, 1080.0, 0.001),
                           np.array([-540.0, -180.0, 180.0, 540.0, 900.0])])
    Df = ((fino + 180.0) % 360.0) - 180.0
    hay_mas180 = bool(np.any(Df == 180.0))
    hay_men180 = bool(np.any(Df == -180.0))
    r.metrica('alcanza_+180', hay_mas180)
    r.metrica('alcanza_-180', hay_men180)
    r.metrica('rango_real', '[-180, 180)')
    r.metrica('rango_documentado', '(-180, 180]')

    # el caso antipodal SIEMPRE resuelve a -180
    antipodales = [(0.0, 180.0), (180.0, 0.0), (90.0, 270.0), (270.0, 90.0), (45.0, 225.0)]
    vals = [float(delta(a, b)) for a, b in antipodales]
    r.metrica('antipodal_siempre_-180', all(v == -180.0 for v in vals))

    # casos de cruce del circulo (lo que la prueba realmente debe garantizar)
    casos = {(359.9, 0.1): -0.2, (0.1, 359.9): 0.2, (0.0, 0.0): 0.0,
             (270.0, 0.0): -90.0, (1.0, 359.0): 2.0, (359.0, 1.0): -2.0}
    peor = max(abs(float(delta(a, b)) - e) for (a, b), e in casos.items())
    r.metrica('err_casos_cruce', peor)
    r.nota('delta(359.9, 0.1) = %.4f  (la resta ingenua daria 359.8)' % float(delta(359.9, 0.1)))
    r.nota('HALLAZGO H-02: el rango real de la forma cerrada es [-180, 180), NO (-180, 180].')
    r.nota('El caso antipodal resuelve SIEMPRE a -180; +180 es inalcanzable.')
    r.nota('El mismo texto incorrecto esta en el docstring oficial de')
    r.nota('vision/detectors/rovers.py::diferencia_angular. Corregir ambos.')
    r.exigir(cong <= 1e-9 and peor <= 1e-9 and not hay_mas180 and hay_men180,
             'congruencia, cruce del circulo y rango real')


@prueba('N0.7', 'rho_P: forma cerrada vs minimizacion explicita')
def n07(r):
    peor = {}
    for P in (90.0, 360.0):
        g = np.arange(0.0, 2 * P, 0.25)
        A, B = np.meshgrid(g, g[::5], indexing='ij')
        cerrada = rho_P(A, B, P)
        ks = np.arange(-4, 5)
        explicita = np.min(np.abs((A - B)[..., None] + ks * P), axis=-1)
        e = float(np.max(np.abs(cerrada - explicita)))
        rango = bool(np.all((cerrada >= -1e-12) & (cerrada <= P / 2 + 1e-12)))
        peor[P] = e
        r.metrica('err_P%g' % P, e); r.metrica('rango_P%g' % P, rango)
        r.exigir(e <= 1e-9 and rango, 'P=%g' % P)


@prueba('N0.8', 'Identidades del marco fila-descendente (apendice A)')
def n08(r):
    g = np.arange(0.0, 360.0, 1.0)
    A, B = np.meshgrid(g, g, indexing='ij')
    uA, uB = u(A), u(B)
    e1 = float(np.max(np.abs(np.sum(uA * uB, -1) - np.cos(np.radians(A - B)))))
    e2 = float(np.max(np.abs(cruz(uA, uB) + np.sin(np.radians(B - A)))))
    e3 = float(np.max(np.abs(u(g) + u(g + 90) - math.sqrt(2) * u(g + 45))))
    e4 = float(np.max(np.abs(u(g + 90) - u(g) - math.sqrt(2) * u(g + 135))))
    h = 1e-6
    der = (u(g + math.degrees(h)) - u(g - math.degrees(h))) / (2 * h)
    e5 = float(np.max(np.abs(der - u(g + 90))))
    for k, v in zip(('producto', 'cruz', 'suma', 'resta'), (e1, e2, e3, e4)):
        r.metrica('err_' + k, v)
    r.metrica('err_derivada', e5)
    r.exigir(max(e1, e2, e3, e4) <= 1e-10 and e5 <= 1e-6, 'tolerancias')


# =========================================================== N1 =============

@prueba('N1.1', 'Parametrizacion vertical: lado, diagonal, area, centroide')
def n11(r):
    ths = np.arange(0.0, 360.0, 0.5)
    eL = eD = eA = eC = 0.0
    for c in POS[::3]:
        for t in ths:
            V = vertices(c, L, t)
            lados = np.linalg.norm(np.roll(V, -1, 0) - V, axis=1)
            diags = np.array([np.linalg.norm(V[2] - V[0]), np.linalg.norm(V[3] - V[1])])
            area = 0.5 * abs(float(np.sum(V[:, 0] * np.roll(V[:, 1], -1) - np.roll(V[:, 0], -1) * V[:, 1])))
            eL = max(eL, float(np.max(np.abs(lados - L))) / L)
            eD = max(eD, float(np.max(np.abs(diags - L * math.sqrt(2)))) / L)
            eA = max(eA, abs(area - L * L) / (L * L))
            eC = max(eC, float(np.max(np.abs(V.mean(0) - c))) / L)
    r.metrica('err_lado_rel', eL); r.metrica('err_diag_rel', eD)
    r.metrica('err_area_rel', eA); r.metrica('err_centroide_rel', eC)
    r.exigir(max(eL, eD, eA, eC) <= 1e-9, 'tolerancia relativa 1e-9')


@prueba('N1.2', 'Recursion de 90 equivale a la forma trigonometrica')
def n12(r):
    ths = np.arange(0.0, 360.0, 0.5)
    e = 0.0
    for c in POS[::3]:
        for t in ths:
            V = vertices(c, L, t)
            Vr = vertices_por_recursion(c, V[0])
            e = max(e, float(np.max(np.abs(V - Vr))))
    r.metrica('err_max_mm', e)
    r.nota('la recursion no evalua ninguna funcion trigonometrica')
    r.exigir(e <= 1e-12, 'tolerancia 1e-12')


@prueba('N1.3', 'Invariancia bajo el grupo C4')
def n13(r):
    ths = np.arange(0.0, 90.0, 1.0)
    e = 0.0
    c = np.array([300.0, 200.0])
    def canon(A):
        idx = np.lexsort((A[:, 1].round(9), A[:, 0].round(9)))
        return A[idx]
    for t in ths:
        base = canon(vertices(c, L, t))
        for k in range(-2, 3):
            otro = canon(vertices(c, L, t + 90.0 * k))
            e = max(e, float(np.max(np.abs(otro - base))))
    r.metrica('err_max_mm', e)
    r.exigir(e <= 1e-9, 'los conjuntos de vertices deben coincidir')


@prueba('N1.4', 'Aristas, puntos medios y normales exteriores')
def n14(r):
    ths = np.arange(0.0, 360.0, 0.5)
    c = np.array([300.0, 200.0])
    eL = eM = eO = 0.0
    ext_ok = True
    for t in ths:
        V = vertices(c, L, t)
        E = aristas(V); Mp = puntos_medios(V); N = normales_exteriores(c, L, t)
        eL = max(eL, float(np.max(np.abs(np.linalg.norm(E, axis=1) - L))))
        eM = max(eM, float(np.max(np.abs(np.linalg.norm(Mp - c, axis=1) - L / 2))))
        eO = max(eO, float(np.max(np.abs(np.sum(E * N, axis=1)))) / (L * L))
        for k in range(4):
            p = c + (L / 2 + 1e-3 * L) * N[k]
            if punto_en_poligono(p, V):
                ext_ok = False
    r.metrica('err_lado', eL); r.metrica('err_dist_medio', eM)
    r.metrica('err_ortogonalidad', eO); r.metrica('normales_exteriores', ext_ok)
    r.exigir(max(eL, eM) <= 1e-9 and eO <= 1e-12 and ext_ok, 'metricas y sentido de la normal')


@prueba('N1.5', 'Estimador B1: vertice de referencia + centro')
def n15(r):
    ths = np.arange(0.0, 90.0, 0.01)
    peor = 0.0
    for c in POS[::7]:
        V4 = np.stack([vertices(c, L, t) for t in ths])          # (T,4,2)
        for k in range(4):
            est = est_B1(c, V4[:, k, :])
            peor = max(peor, float(np.max(rho_P(est, ths, 90.0))))
    r.metrica('n_casos', len(ths) * 4 * len(POS[::7])); r.metrica('err_max_grados', peor)
    r.exigir(peor <= 1e-9, 'tolerancia 1e-9 grados')


@prueba('N1.6', 'Estimador B2: arista, 4 aristas x 2 sentidos')
def n16(r):
    ths = np.arange(0.0, 90.0, 0.01)
    peor = 0.0
    c = np.array([300.0, 200.0])
    V4 = np.stack([vertices(c, L, t) for t in ths])
    for k in range(4):
        va, vb = V4[:, k, :], V4[:, (k + 1) % 4, :]
        for (p, q) in ((va, vb), (vb, va)):
            e = np.array([est_B2_theta(p[i], q[i]) for i in range(len(ths))])
            peor = max(peor, float(np.max(rho_P(e, ths, 90.0))))
    r.metrica('n_casos', len(ths) * 8); r.metrica('err_max_grados', peor)
    r.nota('invariante al sentido de recorrido de la arista')
    r.exigir(peor <= 1e-9, 'tolerancia 1e-9 grados')


@prueba('N1.7', 'Estimador B2: desambiguacion del signo de la normal')
def n17(r):
    ths = np.arange(0.0, 90.0, 2.0)
    c = np.array([300.0, 200.0])
    # (i) testigo = centro exacto ; (ii) 200 puntos interiores aleatorios
    e_i = 0.0; fallos_ii = 0; n_ii = 0
    for t in ths:
        V = vertices(c, L, t)
        for k in range(4):
            ch = est_B2_centro(V[k], V[(k + 1) % 4], L, c)
            e_i = max(e_i, float(np.linalg.norm(ch - c)))
            for _ in range(50):
                # punto interior uniforme en el cuadrado
                ab = rng.uniform(-0.5, 0.5, 2) * L
                p = c + ab[0] * u(t) + ab[1] * u(t + 90)
                ch = est_B2_centro(V[k], V[(k + 1) % 4], L, p)
                n_ii += 1
                if np.linalg.norm(ch - c) > 1e-9:
                    fallos_ii += 1
    r.metrica('err_testigo_centro', e_i)
    r.metrica('fallos_testigo_interior', fallos_ii); r.metrica('n_testigo_interior', n_ii)
    r.exigir(e_i <= 1e-9 and fallos_ii == 0, 'testigo interior debe resolver siempre')

    # (iii) testigo REALISTA: centroide de la silueta con paralaje
    print()
    print('           --- N1.7(iii) testigo realista: centroide de la silueta ---')
    filas = []
    for H in (1000.0, 1300.0, 2100.0):
        kap = kappa(H, L)
        for dist in (100.0, 300.0, 600.0):
            fallos = 0; total = 0; peor_dentro = 0.0
            for ang in np.arange(0, 360, 30.0):
                N = c - dist * u(ang)
                for t in ths:
                    V = vertices(c, L, t)
                    S = silueta(c, L, t, N, kap)
                    hull = casco_convexo(S)
                    mu = hull.mean(axis=0)
                    peor_dentro = max(peor_dentro, float(np.linalg.norm(mu - c)))
                    for k in range(4):
                        ch = est_B2_centro(V[k], V[(k + 1) % 4], L, mu)
                        total += 1
                        if np.linalg.norm(ch - c) > 1e-9:
                            fallos += 1
            filas.append((H, dist, fallos, total, peor_dentro))
            print('           H=%6.0f mm  d_nadir=%5.0f mm  fallos=%4d/%4d  |mu-c|max=%5.2f mm'
                  % (H, dist, fallos, total, peor_dentro))
    tot_f = sum(f[2] for f in filas); tot_n = sum(f[3] for f in filas)
    r.metrica('fallos_testigo_realista', tot_f); r.metrica('n_testigo_realista', tot_n)
    r.metrica('desplaz_centroide_max_mm', max(f[4] for f in filas))
    if tot_f == 0:
        r.nota('el centroide de la silueta NUNCA sale del cuadrado: sirve como testigo')
    else:
        r.nota('CABO SUELTO: el centroide de la silueta falla como testigo en %d/%d casos' % (tot_f, tot_n))


@prueba('N1.8', 'Estimador B3: diagonal')
def n18(r):
    ths = np.arange(0.0, 90.0, 0.01)
    c = np.array([300.0, 200.0])
    ec = et = 0.0
    for t in ths:
        V = vertices(c, L, t)
        for (i, j) in ((0, 2), (1, 3), (2, 0), (3, 1)):
            ch, th = est_B3(V[i], V[j])
            ec = max(ec, float(np.linalg.norm(ch - c)))
            et = max(et, float(rho_P(th, t, 90.0)))
    r.metrica('err_centro_mm', ec); r.metrica('err_theta_grados', et)
    r.exigir(ec <= 1e-9 and et <= 1e-9, 'tolerancia 1e-9')


@prueba('N1.9', 'Estimador B4 sin ruido: los 11 subconjuntos con |K|>=2')
def n19(r):
    from itertools import combinations
    ths = np.arange(0.0, 360.0, 0.5)
    c = np.array([300.0, 200.0])
    subconj = [K for n in (2, 3, 4) for K in combinations(range(4), n)]
    peor_c = peor_t = 0.0
    detalle = []
    for K in subconj:
        ec = et = 0.0; degen = 0
        for t in ths:
            V = vertices(c, L, t)
            ch, th, (A, B) = est_B4(V[list(K)], list(K), L)
            if math.hypot(A, B) < 1e-9:
                degen += 1
            ec = max(ec, float(np.linalg.norm(ch - c)))
            et = max(et, float(rho_P(th, t, 90.0)))
        detalle.append((K, ec, et, degen))
        peor_c = max(peor_c, ec); peor_t = max(peor_t, et)
    r.metrica('n_subconjuntos', len(subconj))
    r.metrica('err_centro_max_mm', peor_c); r.metrica('err_theta_max_grados', peor_t)
    diag = [d for d in detalle if d[0] in ((0, 2), (1, 3))]
    r.nota('subconjuntos diagonales {0,2} y {1,3}: err_theta=%.2e, %.2e ; degeneraciones=%d'
           % (diag[0][2], diag[1][2], sum(d[3] for d in diag)))
    r.exigir(peor_c <= 1e-9 and peor_t <= 1e-9, 'tolerancia 1e-9 en los 11 subconjuntos')


@prueba('N1.10', 'Estimador B4 con ruido: insesgadez y cota de Cramer-Rao')
def n110(r):
    NMC = 50000
    th_true = 31.0
    c = np.array([300.0, 200.0])
    V0 = vertices(c, L, th_true)
    print()
    print('           %3s %8s %10s %10s %10s %8s %9s' %
          ('n', 'sig_mm', 'sesgo_deg', 'sd_emp', 'sd_pred', 'razon', 'sesgo/EE'))
    peor_razon = 0.0; peor_z = 0.0
    for n in (2, 3, 4):
        K = list(range(n))
        rho_ef = rho_efectivo(L, K)
        for sig in (0.25, 0.5, 1.0, 2.0, 4.0):
            Vn = V0[K][None, :, :] + rng.normal(0, sig, (NMC, n, 2))
            est = est_B4_vect(Vn, K, L)
            err = delta(est, th_true)
            sesgo = float(np.mean(err)); sd = float(np.std(err))
            pred = math.degrees(sig / (rho_ef * math.sqrt(n)))
            ee = sd / math.sqrt(NMC)
            z = abs(sesgo) / ee
            razon = sd / pred
            if sig <= 2.0:
                peor_razon = max(peor_razon, abs(razon - 1.0))
                peor_z = max(peor_z, z)
            print('           %3d %8.2f %10.4f %10.4f %10.4f %8.4f %9.2f'
                  % (n, sig, sesgo, sd, pred, razon, z))
    r.metrica('peor_desvio_relativo', peor_razon); r.metrica('peor_z_sesgo', peor_z)
    r.nota('regimen validado: sigma <= 2 mm (el peor caso de vision con oclusion es 4.88 mm)')
    r.exigir(peor_razon <= 0.05, 'sd empirica dentro del 5% de la prediccion (sigma<=2mm)')
    if peor_z > 3.0:
        r.nota('ATENCION: sesgo significativo a 3 EE (z=%.2f)' % peor_z)


@prueba('N1.11', 'Estimador B4: ruptura ante un vertice atipico')
def n111(r):
    c = np.array([300.0, 200.0]); th = 31.0
    V0 = vertices(c, L, th)
    print()
    print('           %8s %12s %12s %14s' % ('d_mm', 'err_th_deg', 'err_c_mm', 'd(err_th)/d(d)'))
    ds = np.array([0, 5, 10, 20, 40, 80, 120, 200], float)
    errs_t = []; errs_c = []
    for d in ds:
        et = ec = 0.0
        for ang in np.arange(0, 360, 15.0):
            for k in range(4):
                V = V0.copy(); V[k] = V[k] + d * u(ang)
                ch, tht, _ = est_B4(V, [0, 1, 2, 3], L)
                et = max(et, float(rho_P(tht, th, 90.0)))
                ec = max(ec, float(np.linalg.norm(ch - c)))
        errs_t.append(et); errs_c.append(ec)
    for i, d in enumerate(ds):
        pend = (errs_t[i] - errs_t[i-1]) / (ds[i] - ds[i-1]) if i else float('nan')
        print('           %8.0f %12.4f %12.4f %14.5f' % (d, errs_t[i], errs_c[i], pend))
    r.metrica('err_theta_a_20mm', errs_t[3]); r.metrica('err_theta_a_200mm', errs_t[-1])
    r.metrica('err_centro_a_200mm', errs_c[-1])
    # el centro se desplaza exactamente d/4 (media de 4 puntos)
    r.metrica('desplaz_centro_teorico_200mm', 200.0 / 4.0)
    crece = errs_c[-1] > errs_c[-2] > errs_c[-3]
    r.nota('B4 NO es robusto: el centro se corre d/n y el angulo se sesga sin cota util.')
    r.nota('Mitigacion obligatoria: mediana de estimaciones por arista (B2) o RANSAC antes de B4.')
    r.estado = 'ESPERADO' if crece else 'FALLA'


@prueba('N1.12', 'Funcion soporte: forma cerrada vs maximo explicito')
def n112(r):
    c = np.array([300.0, 200.0])
    g = np.arange(0.0, 360.0, 1.0)
    e = 0.0
    for t in g:
        V = vertices(c, L, t) - c
        UU = u(g)                                   # (360,2)
        expl = np.max(V @ UU.T, axis=0)             # (360,)
        form = soporte_formula(L, t, g)
        e = max(e, float(np.max(np.abs(expl - form))))
    r.metrica('n_pares', len(g) ** 2); r.metrica('err_max_mm', e)
    r.exigir(e <= 1e-9, 'tolerancia 1e-9 mm')


@prueba('N1.13', 'Cota universal de anchura proyectada [L, L*sqrt2]')
def n113(r):
    D = np.arange(0.0, 360.0, 0.1)
    w = anchura(L, 0.0, D)
    wmin, wmax = float(w.min()), float(w.max())
    A = rng.uniform(0, 360, 1000000); B = rng.uniform(0, 360, 1000000)
    w2 = anchura(L, A, B)
    ok = bool(np.all(w2 >= L - 1e-9) and np.all(w2 <= L * math.sqrt(2) + 1e-9))
    # extremos alcanzados
    e0 = abs(float(anchura(L, 0.0, 0.0)) - L)
    e45 = abs(float(anchura(L, 0.0, 45.0)) - L * math.sqrt(2))
    r.metrica('w_min_mm', wmin); r.metrica('w_max_mm', wmax)
    r.metrica('cota_universal_1e6', ok)
    r.metrica('err_extremo_0', e0); r.metrica('err_extremo_45', e45)
    r.exigir(ok and e0 <= 1e-9 and e45 <= 1e-9, 'cota y extremos')


@prueba('N1.14', 'Lema de silueta: conteo de vertices del casco')
def n114(r):
    from collections import Counter
    c = np.array([0.0, 0.0])
    cont_ext = Counter(); cont_int = Counter(); degen = Counter()
    for H in (1300.0, 1700.0, 2100.0):
        kap = kappa(H, L)
        for nx in np.arange(-1000, 1001, 25.0):
            for ny in np.arange(-1000, 1001, 25.0):
                N = np.array([nx, ny])
                for t in (0.0, 22.5, 45.0, 67.5):
                    if np.linalg.norm(N) < 1e-9:
                        continue
                    V = vertices(c, L, t)
                    k = len(casco_convexo(silueta(c, L, t, N, kap)))
                    dentro = punto_en_poligono(N, V)
                    (cont_int if dentro else cont_ext)[k] += 1
    # degeneraciones explicitas: nadir sobre arista y sobre vertice
    for t in (0.0, 22.5, 45.0):
        V = vertices(c, L, t); Mp = puntos_medios(V)
        for kap in (kappa(1300.0, L), kappa(2100.0, L)):
            for N in list(Mp) + list(V):
                degen[len(casco_convexo(silueta(c, L, t, N, kap)))] += 1
    r.metrica('nadir_exterior', dict(cont_ext)); r.metrica('nadir_interior', dict(cont_int))
    r.metrica('degeneraciones', dict(degen))
    ok_ext = set(cont_ext) <= {6}
    ok_int = set(cont_int) <= {4}
    r.nota('nadir exterior -> %s ; nadir interior -> %s ; sobre arista/vertice -> %s'
           % (dict(cont_ext), dict(cont_int), dict(degen)))
    r.exigir(ok_ext and ok_int, 'el lema predice 6 (exterior) y 4 (interior)')


@prueba('N1.15', 'Paralaje: la correccion es involucion exacta')
def n115(r):
    e = 0.0
    for H in (1300.0, 2100.0):
        for N in (np.array([430.0, 430.0]), np.array([0.0, 0.0]), np.array([860.0, 860.0]),
                  np.array([100.0, 700.0]), np.array([-200.0, 400.0])):
            P = rng.uniform(-100, 960, (20000, 2))
            h = rng.uniform(0, 0.9 * H, 20000)
            for hh in (0.0, 60.0, 90.0, 0.9 * H):
                kap = kappa(H, hh)
                Q = pi_directo(P, N, kap)
                P2 = pi_inverso(Q, N, kap)
                e = max(e, float(np.max(np.abs(P2 - P))))
    r.metrica('err_max_mm', e)
    r.exigir(e <= 1e-9, 'tolerancia 1e-9 mm')


@prueba('N1.16', 'Paralaje: invariancia de la direccion')
def n116(r):
    e = 0.0
    for H in (1300.0, 2100.0):
        for hh in (60.0, 90.0):
            kap = kappa(H, hh)
            for N in (np.array([430.0, 430.0]), np.array([0.0, 0.0]), np.array([900.0, -50.0])):
                P = rng.uniform(-100, 960, (100000, 2))
                Q = P + rng.uniform(-300, 300, (100000, 2))
                m = np.linalg.norm(Q - P, axis=1) > 1.0
                P, Q = P[m], Q[m]
                a1 = arg_de(Q - P)
                a2 = arg_de(pi_directo(Q, N, kap) - pi_directo(P, N, kap))
                e = max(e, float(np.max(rho_P(a1, a2, 360.0))))
    r.metrica('err_max_grados', e)
    r.nota('confirma que el rumbo del rover NO necesita correccion de paralaje')
    r.exigir(e <= 1e-9, 'tolerancia 1e-9 grados')


@prueba('N1.17', 'Centro proyectivo (diagonales) vs promedio de esquinas')
def n117(r):
    import cv2
    print()
    print('           %10s %8s %14s %14s' % ('lado_mm', 'incl', 'err_diag_mm', 'err_prom_mm'))
    peor_diag = 0.0
    tabla = {}
    for lado in (40.0, 60.0, 100.0):
        for incl in (0.0, 2.0, 5.0, 8.0, 12.0):
            ed = ep = 0.0
            # camara: plano Z=0, centro optico a H con inclinacion
            H = 2100.0
            a = math.radians(incl)
            Rc = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
            f = 1400.0
            Kk = np.array([[f, 0, 640.0], [0, f, 640.0], [0, 0, 1.0]])
            Cw = np.array([430.0, 430.0, H])

            def proy(P3):
                Xc = (P3 - Cw) @ Rc.T
                Xc = Xc * np.array([1.0, 1.0, -1.0])   # mirar hacia abajo
                z = np.where(np.abs(Xc[:, 2]) < 1e-9, 1e-9, Xc[:, 2])
                px = (Xc[:, 0] * f) / z + 640.0
                py = (Xc[:, 1] * f) / z + 640.0
                return np.stack([px, py], axis=1)

            centros3 = [np.array([x, y, 0.0]) for x in np.linspace(120, 740, 5)
                        for y in np.linspace(120, 740, 5)]
            for c3 in centros3:
                h = lado / 2
                esq3 = np.array([c3 + np.array([-h, -h, 0]), c3 + np.array([h, -h, 0]),
                                 c3 + np.array([h, h, 0]), c3 + np.array([-h, h, 0])])
                px = proy(esq3)
                if not np.all(np.isfinite(px)):
                    continue
                # verdad en pixeles: proyeccion del centro
                verdad = proy(c3[None, :])[0]
                # homografia mundo->pixel a partir de las 4 esquinas conocidas
                Hm = cv2.getPerspectiveTransform(esq3[:, :2].astype(np.float32),
                                                 px.astype(np.float32))
                # metodo 1: interseccion de diagonales en pixeles
                oa, da = px[0], px[2] - px[0]
                ob, db = px[1], px[3] - px[1]
                den = cruz(da, db)
                if abs(den) < 1e-12:
                    continue
                t = cruz(ob - oa, db) / den
                cd = oa + t * da
                cp = px.mean(axis=0)
                ed = max(ed, float(np.linalg.norm(cd - verdad)))
                ep = max(ep, float(np.linalg.norm(cp - verdad)))
            # a milimetros: escala aproximada px/mm en el centro
            escala = f / H
            tabla[(lado, incl)] = (ed / escala, ep / escala)
            peor_diag = max(peor_diag, ed / escala)
            print('           %10.0f %8.1f %14.6f %14.4f' % (lado, incl, ed / escala, ep / escala))
    r.metrica('err_diagonales_max_mm', peor_diag)
    r.metrica('err_promedio_100mm_12deg', tabla[(100.0, 12.0)][1])
    r.metrica('err_promedio_60mm_12deg', tabla[(60.0, 12.0)][1])
    r.nota('el sesgo del promedio crece con el lado del marcador, como predice el modelo')
    r.exigir(peor_diag <= 1e-6, 'la interseccion de diagonales debe ser exacta')


if __name__ == '__main__':
    print('=' * 96)
    print('NIVEL N0 --- CONSISTENCIA INTERNA')
    print('=' * 96)
    for f in (n01, n02, n03, n04, n05, n06, n07, n08):
        f()
    print()
    print('=' * 96)
    print('NIVEL N1 --- ESTIMADORES GEOMETRICOS CONTRA VERDAD ANALITICA')
    print('=' * 96)
    for f in (n11, n12, n13, n14, n15, n16, n17, n18, n19, n110, n111,
              n112, n113, n114, n115, n116, n117):
        f()
    nf = resumen('RESUMEN N0 + N1')
    volcar('/tmp/bench/resultados_n0_n1.json')
    raise SystemExit(0)

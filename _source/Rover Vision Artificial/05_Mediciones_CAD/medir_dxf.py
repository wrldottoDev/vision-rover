"""Mide las piezas del CAD de fabricacion del CenfoBot y verifica la escala.

Uso:   python3 medir_dxf.py [carpeta_archivos_fabricacion]
       python3 medir_dxf.py --piezas <CENFOBOT_PIEZAS_SEPARADAS.dxf>

El segundo modo mide el despiece separado, que es el que resuelve el ancho util
del canal W_u: el chasis tiene cuerpo de 93.50 y lenguetas de 3.00 por lado, y
como la lengueta mide lo mismo que el espesor del acrilico la junta es pasante,
de modo que la cara interior del panel lateral coincide con el borde del cuerpo.

El DXF del rover NO declara unidades. El script confirma que 1 unidad = 1 mm por
tres vias independientes antes de reportar cualquier medida:
  (a) las ranuras de ensamble deben medir 3.00 (espesor del acrilico);
  (b) los dos agujeros de la placa frontal deben dar 16 mm de diametro y 24 mm
      de separacion (patron del ultrasonico HC-SR04);
  (c) cubos.dxf, que si declara mm, debe medir 180.30 (tres cubos de 60 mm).

Dependencias: ezdxf, cairosvg, opencv-python, numpy.
"""
import sys, os
import numpy as np

CARPETA = sys.argv[1] if len(sys.argv) > 1 else '.'
ROVER_DXF = os.path.join(CARPETA, 'CENFOBOT_Rover_Rev1.dxf')
ROVER_SVG = os.path.join(CARPETA, 'CENFOBOT_Rover_Rev1.svg')
CUBOS_DXF = os.path.join(CARPETA, 'cubos.dxf')


def _pts(e):
    t = e.dxftype()
    if t == 'LINE':
        return np.array([[e.dxf.start.x, e.dxf.start.y], [e.dxf.end.x, e.dxf.end.y]])
    if t == 'LWPOLYLINE':
        return np.array([[p[0], p[1]] for p in e.get_points('xy')])
    if t == 'SPLINE':
        try:
            return np.array([[p[0], p[1]] for p in e.control_points])
        except Exception:
            return np.zeros((0, 2))
    return np.zeros((0, 2))


def control_de_escala():
    """Via (c): cubos.dxf declara mm y debe medir 180.30 de lado."""
    import ezdxf
    d = ezdxf.readfile(CUBOS_DXF)
    P = [_pts(e) for e in d.modelspace()]
    A = np.vstack([p for p in P if len(p)])
    lado = float(A[:, 0].max() - A[:, 0].min())
    print('  (c) cubos.dxf: %.2f mm de lado  (esperado 180.30 = 3 x 60)' % lado)
    return abs(lado - 180.30) < 1.0


def piezas_y_agujeros(ancho_px=4000):
    """Segmenta el sheet rasterizando el SVG y mide cada pieza en mm."""
    import cairosvg, cv2, re
    svg = open(ROVER_SVG).read()
    vb = [float(x) for x in re.search(r'viewBox="([-\d.eE ]+)"', svg).group(1).split()]
    cairosvg.svg2png(url=ROVER_SVG, write_to='/tmp/_sheet.png',
                     output_width=ancho_px, background_color='white')
    img = cv2.imread('/tmp/_sheet.png', 0)
    mm = vb[2] / ancho_px
    libre = (img >= 128).astype(np.uint8)
    n, lab, st, cen = cv2.connectedComponentsWithStats(libre, 4)
    fondo = lab[0, 0]
    comp = []
    for k in range(1, n):
        if k == fondo or st[k, cv2.CC_STAT_AREA] < 120:
            continue
        w = st[k, cv2.CC_STAT_WIDTH] * mm
        h = st[k, cv2.CC_STAT_HEIGHT] * mm
        a = st[k, cv2.CC_STAT_AREA] * mm * mm
        comp.append(dict(w=w, h=h, area=a, cx=cen[k][0] * mm, cy=cen[k][1] * mm,
                         llenado=a / (np.pi * (w / 2) * (h / 2)) if w * h else 0))
    return comp


def ranuras_de_acrilico():
    """Via (a): las ranuras de ensamble deben medir 3.00 (espesor del acrilico).

    Cada ranura esta dibujada como varios segmentos independientes, asi que hay
    que agrupar las entidades del DXF por contigueidad antes de medirlas:
    ninguna entidad tiene por si sola la caja de 3 mm.
    """
    import ezdxf
    d = ezdxf.readfile(ROVER_DXF)
    P = [_pts(e) for e in d.modelspace()]
    P = [p for p in P if len(p) >= 2]
    B = [(p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()) for p in P]
    padre = list(range(len(P)))

    def raiz(i):
        while padre[i] != i:
            padre[i] = padre[padre[i]]
            i = padre[i]
        return i

    tol = 0.3
    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            a, b = B[i], B[j]
            if (a[0] - tol <= b[2] and b[0] - tol <= a[2] and
                    a[1] - tol <= b[3] and b[1] - tol <= a[3]):
                ri, rj = raiz(i), raiz(j)
                if ri != rj:
                    padre[ri] = rj

    grupos = {}
    for i in range(len(P)):
        grupos.setdefault(raiz(i), []).append(i)

    n = 0
    for idx in grupos.values():
        A = np.vstack([P[i] for i in idx])
        w = float(A[:, 0].max() - A[:, 0].min())
        h = float(A[:, 1].max() - A[:, 1].min())
        corto, largo = min(w, h), max(w, h)
        if abs(corto - 3.0) < 0.05 and largo > 5.0:
            n += 1
    return n >= 2, n


AGRUPAR_TOL = 0.6


def _agrupar(P, tol=AGRUPAR_TOL):
    """Union-find de entidades por contigueidad de cajas: devuelve una lista de piezas."""
    B = [(p[:, 0].min(), p[:, 1].min(), p[:, 0].max(), p[:, 1].max()) for p in P]
    padre = list(range(len(P)))

    def raiz(i):
        while padre[i] != i:
            padre[i] = padre[padre[i]]
            i = padre[i]
        return i

    for i in range(len(P)):
        for j in range(i + 1, len(P)):
            a, b = B[i], B[j]
            if (a[0] - tol <= b[2] and b[0] - tol <= a[2] and
                    a[1] - tol <= b[3] and b[1] - tol <= a[3]):
                ri, rj = raiz(i), raiz(j)
                if ri != rj:
                    padre[ri] = rj
    g = {}
    for i in range(len(P)):
        g.setdefault(raiz(i), []).append(i)
    return [np.vstack([P[i] for i in idx]) for idx in g.values()]


def medir_piezas_separadas(ruta):
    """Mide el despiece separado y deduce W_u de la junta pasante."""
    import ezdxf
    d = ezdxf.readfile(ruta)
    unidades = d.header.get('$INSUNITS')
    print('VERIFICACION DE ESCALA')
    print('  $INSUNITS = %s  ->  %s' % (unidades,
          'milimetros, declarado en el archivo' if unidades == 4 else 'NO declara mm, revisar'))

    P = [_pts(e) for e in d.modelspace()]
    P = [p for p in P if len(p) >= 2]
    piezas = [A for A in _agrupar(P)
              if (A[:, 0].max() - A[:, 0].min()) * (A[:, 1].max() - A[:, 1].min()) > 150]
    piezas.sort(key=lambda A: -((A[:, 0].max() - A[:, 0].min()) *
                                (A[:, 1].max() - A[:, 1].min())))
    print()
    print('PIEZAS (mm), de mayor a menor')
    print('  %10s %10s' % ('ancho', 'alto'))
    for A in piezas[:9]:
        print('  %10.2f %10.2f' % (A[:, 0].max() - A[:, 0].min(),
                                   A[:, 1].max() - A[:, 1].min()))

    chasis = piezas[0]
    xs = np.unique(np.round(chasis[:, 0], 2))
    x0, x1 = xs[0], xs[-1]
    # los bordes del CUERPO son los que estan a ~3 mm de los extremos
    def cerca(v):
        return xs[np.argmin(np.abs(xs - v))]
    b0, b1 = cerca(x0 + 3.0), cerca(x1 - 3.0)
    leng0, leng1 = b0 - x0, x1 - b1
    W_ext = x1 - x0
    W_u = b1 - b0
    print()
    print('JUNTA PASANTE Y ANCHO UTIL DEL CANAL')
    print('  huella exterior  W_ext = %.2f mm' % W_ext)
    print('  lenguetas             = %.2f y %.2f mm  (espesor del acrilico: 3.00)'
          % (leng0, leng1))
    print('  cuerpo del chasis W_u = %.2f mm' % W_u)
    print('  comprobacion: W_ext - 2 x 3.00 = %.2f' % (W_ext - 6.0))
    ok = abs(leng0 - 3.0) < 0.05 and abs(leng1 - 3.0) < 0.05
    print('  -> junta %s' % ('PASANTE confirmada: la cara interior del panel'
                             ' coincide con el borde del cuerpo' if ok
                             else 'NO pasante, revisar el montaje'))
    print()
    print('CONSECUENCIAS GEOMETRICAS  (L = 60 mm, sigma_vis = 5 mm)')
    L, SIG, LAM = 60.0, 5.0, chasis[:, 1].max() - chasis[:, 1].min()
    emax_min = (W_u - L * np.sqrt(2)) / 2
    emax_max = (W_u - L) / 2
    R = 0.5 * np.sqrt(W_ext ** 2 + LAM ** 2)
    rho_ct = L * np.sqrt(2) / 2 + LAM / 2
    print('  Lambda (largo)        = %.2f mm' % LAM)
    print('  R_rov (con W_ext)     = %.2f mm' % R)
    print('  rho_contacto (max)    = %.2f mm' % rho_ct)
    print('  rho_puesta (s=40)     = %.2f mm' % (rho_ct + 40))
    print('  e_max                 = [%.2f, %.2f] mm' % (emax_min, emax_max))
    print('  presupuesto a 45 deg  = %.2f mm  %s'
          % (emax_min - SIG, '<-- NEGATIVO (hallazgo H-11)' if emax_min < SIG else ''))
    w = W_u - 2 * (SIG + 3.0)
    dmax = 0.5 * np.degrees(np.arcsin(min(1.0, (w / L) ** 2 - 1)))
    print('  ventana con m=3 mm    = |Delta| <= %.2f grados' % dmax)
    return W_u, W_ext


def main():
    if len(sys.argv) > 2 and sys.argv[1] == '--piezas':
        medir_piezas_separadas(sys.argv[2])
        return
    print('VERIFICACION DE ESCALA')
    ok_c = control_de_escala()
    comp = piezas_y_agujeros()
    circ = sorted([c for c in comp
                   if abs(c['w'] - c['h']) < 0.6 and c['llenado'] > 0.93 and c['w'] > 8],
                  key=lambda c: -c['w'])
    if len(circ) >= 2:
        sep = abs(circ[0]['cx'] - circ[1]['cx'])
        print('  (b) ultrasonico: diametros %.2f y %.2f, separacion %.2f'
              % (circ[0]['w'], circ[1]['w'], sep))
        ok_b = abs(sep - 24.0) < 1.0 and abs(circ[0]['w'] - 16.0) < 1.0
    else:
        ok_b = False
        print('  (b) no se hallaron los dos agujeros del ultrasonico')
    ok_a, n_ran = ranuras_de_acrilico()
    print('  (a) ranuras de 3.00 unidades halladas en el DXF: %d' % n_ran)
    print('  -> escala %s' % ('CONFIRMADA: 1 unidad = 1 mm'
                              if (ok_a and ok_b and ok_c) else 'NO CONFIRMADA, revisar'))
    print()
    print('PIEZAS (mm)')
    print('  %-10s %10s %10s' % ('ancho', 'alto', 'area'))
    for c in sorted(comp, key=lambda c: -c['area'])[:8]:
        print('  %10.2f %10.2f %10.1f' % (c['w'], c['h'], c['area']))
    print()
    print('Referencia esperada:')
    print('  chasis          98.66 x 93.20')
    print('  placa frontal   98.51 x 41.76')
    print('  paleta (x2)     51.65 x 105.15')
    print()
    print('NOTA: este archivo (Rev1) da el contorno de la cubierta. El ancho UTIL del')
    print('canal sale del despiece separado; correr:')
    print('  python3 medir_dxf.py --piezas CENFOBOT_PIEZAS_SEPARADAS.dxf')


if __name__ == '__main__':
    main()

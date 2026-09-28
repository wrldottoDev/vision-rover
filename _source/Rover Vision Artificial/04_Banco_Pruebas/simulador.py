"""Simulador de lazo cerrado con geometria de contacto correcta.

Diferencias deliberadas con contrato/mock_publisher.py (hallazgo H-01):
  * el contacto se resuelve entre CUERPOS (rectangulo del rover contra cuadrado
    del cubo) por el teorema del eje separador, no por un radio entre centros;
  * la oclusion se calcula por AREA TAPADA del cuadrado de la base, no por
    proximidad de centros;
  * el empuje es DIRECCIONAL: solo empuja lo que toca la cara delantera.

El formato del mensaje publicado es identico al del contrato v1, de modo que el
mismo planificador corre contra los dos.

Unidades internas: MILIMETROS. La telemetria se emite en celdas.
"""
from __future__ import annotations
import math
import numpy as np
from modelo import u, R, arg_de, delta, rho_P, vertices, casco_convexo

CELL = 20.0
COLS = ROWS = 43
BORDE = 3.5 * CELL          # borde muerto del tablero fisico


# ----------------------------------------------------- geometria de cuerpos --

def rect_esquinas(p, theta, largo, ancho):
    """Esquinas del rectangulo orientado (centro p, eje longitudinal segun theta)."""
    up = u(theta); pe = u(theta + 90.0)
    hl, hw = largo / 2.0, ancho / 2.0
    return np.array([p + hl * up + hw * pe, p + hl * up - hw * pe,
                     p - hl * up - hw * pe, p - hl * up + hw * pe])


def sat_mtv(A, B):
    """Teorema del eje separador entre dos convexos. Devuelve (solapan, mtv).

    mtv es el vector minimo de traslacion que separa B de A (empuja B).
    """
    mejor_d = float('inf'); mejor_n = None
    for P, signo in ((A, 1.0), (B, -1.0)):
        n = len(P)
        for i in range(n):
            e = P[(i + 1) % n] - P[i]
            ax = np.array([-e[1], e[0]])
            L = np.linalg.norm(ax)
            if L < 1e-12:
                continue
            ax = ax / L
            pa = A @ ax; pb = B @ ax
            d = min(pa.max() - pb.min(), pb.max() - pa.min())
            if d <= 0:
                return False, None
            if d < mejor_d:
                mejor_d = d
                # orientar el eje para que empuje B lejos de A
                if pa.mean() < pb.mean():
                    mejor_n = ax
                else:
                    mejor_n = -ax
    return True, mejor_d * mejor_n


def recortar_convexo(sujeto, ventana):
    """Sutherland-Hodgman: interseccion de dos poligonos CONVEXOS."""
    salida = [np.asarray(p, float) for p in sujeto]
    n = len(ventana)
    # orientar la ventana en sentido positivo
    area = 0.0
    for i in range(n):
        a, b = ventana[i], ventana[(i + 1) % n]
        area += a[0] * b[1] - b[0] * a[1]
    vent = ventana if area > 0 else ventana[::-1]
    for i in range(n):
        a, b = vent[i], vent[(i + 1) % n]
        e = b - a
        entrada = salida; salida = []
        if not entrada:
            break
        for j in range(len(entrada)):
            p, q = entrada[j], entrada[(j + 1) % len(entrada)]
            sp = e[0] * (p[1] - a[1]) - e[1] * (p[0] - a[0])
            sq = e[0] * (q[1] - a[1]) - e[1] * (q[0] - a[0])
            if sp >= 0:
                salida.append(p)
            if (sp > 0) != (sq > 0):
                den = sp - sq
                if abs(den) > 1e-12:
                    salida.append(p + (sp / den) * (q - p))
    return np.array(salida) if len(salida) >= 3 else np.zeros((0, 2))


def area_poligono(P):
    if len(P) < 3:
        return 0.0
    x, y = P[:, 0], P[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y)))


# ------------------------------------------------------------------ mundo ---

class Rover:
    """Rover en forma de U: cuerpo rectangular mas dos paletas paralelas.

    CAMBIO IMPORTANTE respecto de la primera version. El rover se modelaba como
    un unico rectangulo LAM x W, de modo que el cubo era empujado por la cara
    frontal del chasis y el canal entre paletas NO EXISTIA en la simulacion. Con
    eso, el resultado central del modelo -el embudo de tolerancia, y con el el
    hallazgo H-11- no podia ser ni confirmado ni refutado en lazo cerrado.

    Medido sobre el rover fisico: las paletas se prolongan lambda = 55 mm por
    delante de la placa frontal, de modo que el largo total es 94 + 55 = 149 mm
    (medido 150). El cubo entra ENTRE las paletas y recorre esos 55 mm contenido
    lateralmente antes de tocar la placa frontal, que es lo que empuja.

    Las tres piezas se tratan por separado:
      - cuerpo:  empuja (contacto por la cara delantera).
      - paletas: contienen, no empujan. Resuelven la penetracion lateral sin
                 transmitir avance, que es exactamente lo que dice la
                 proposicion del canal: contienen pero no centran.
    """

    def __init__(self, id_aruco, col, row, theta, W=99.50, LAM=94.00,
                 W_CANAL=93.50, LAMBDA=55.0, ESP=3.0):
        self.id = id_aruco
        self.p = np.array([col * CELL, row * CELL], float)
        self.theta = float(theta)
        self.W, self.LAM = W, LAM
        self.W_CANAL, self.LAMBDA, self.ESP = W_CANAL, LAMBDA, ESP
        self.v = 0.0; self.w = 0.0
        self.rep = (self.p.copy(), self.theta); self.ts_visto = 0

    # --- geometria ---------------------------------------------------------
    def cuerpo(self):
        """El chasis. Es la pieza que empuja."""
        return rect_esquinas(self.p, self.theta, self.LAM, self.W)

    def paletas(self):
        """Las dos paletas, como rectangulos finos que se prolongan al frente."""
        up = u(self.theta); pe = u(self.theta + 90.0)
        cx = self.p + (self.LAM / 2.0 + self.LAMBDA / 2.0) * up
        off = (self.W_CANAL + self.ESP) / 2.0
        return [rect_esquinas(cx + sb * off * pe, self.theta, self.LAMBDA, self.ESP)
                for sb in (-1, 1)]

    def piezas(self):
        return [self.cuerpo()] + self.paletas()

    def esquinas(self):
        """Envolvente de todo el cuerpo. Se usa para la frontera del tablero."""
        return np.vstack(self.piezas())

    def cara_delantera(self):
        E = self.cuerpo()
        return E[0], E[1]        # las dos esquinas delanteras del chasis


class Cubo:
    def __init__(self, color, col, row, theta=0.0, L=60.0):
        self.color = color
        self.c = np.array([col * CELL, row * CELL], float)
        self.theta = float(theta)
        self.L = L
        self.rep = self.c.copy(); self.ts_visto = 0

    def esquinas(self):
        return vertices(self.c, self.L, self.theta)


class Mundo:
    """Simulador. Paso fijo, determinista dada la semilla."""

    def __init__(self, rovers, cubos, depots, semilla=0,
                 sigma_pos_celdas=0.0, sigma_theta=0.0, p_perdida=0.0,
                 latencia_ms=0.0, v_max=120.0, w_max=90.0,
                 altura_camara_mm=2100.0, nadir_celdas=(21.5, 21.5)):
        self.rovers = rovers
        self.cubos = cubos
        self.depots = depots
        self.rng = np.random.default_rng(semilla)
        self.sig_p = sigma_pos_celdas * CELL
        self.sig_t = sigma_theta
        self.p_perd = p_perdida
        self.lat = latencia_ms
        self.v_max, self.w_max = v_max, w_max
        self.t_ms = 0
        self.seq = 0
        self.fase = 'RUNNING'
        self.cola = []
        self.colisiones = 0
        self.salidas = 0
        self.contactos_paleta = 0
        # Un cubo "des-entregado" es uno que estuvo dentro de la tolerancia de
        # entrega y despues salio. Mide el trabajo que el rover DESHACE, que es
        # el modo de falla que aparecio al modelar el rover como U: la paleta
        # barre el cubo que se acaba de dejar en el deposito.
        self.desentregas = 0
        self._estuvo_entregado = set()
        self.rg2 = (60.0 ** 2) / 6.0      # radio de giro al cuadrado del cuadrado
        self.H = float(altura_camara_mm)
        self.nadir = np.array(nadir_celdas, float) * CELL

    # --- siluetas: lo que la camara VE, no la huella en el piso -------------
    def _kappa(self, h):
        return self.H / (self.H - h) if 0 < h < self.H else 1.0

    def silueta(self, base, altura_mm):
        """conv(base U tapa). La tapa es la base llevada por la homotecia del
        paralaje alrededor del nadir. Es lo que el detector ve como mancha."""
        k = self._kappa(altura_mm)
        tapa = self.nadir + k * (np.asarray(base, float) - self.nadir)
        return casco_convexo(np.vstack([base, tapa]), eps_rel=1e-12)

    # -------------------------------------------------------------- fisica --
    def paso(self, dt):
        for rv in self.rovers:
            v = float(np.clip(rv.v, -self.v_max, self.v_max))
            w = float(np.clip(rv.w, -self.w_max, self.w_max))
            rv.theta = (rv.theta + w * dt) % 360.0
            rv.p = rv.p + v * dt * u(rv.theta)
            # frontera del tablero fisico.
            # CRITERIO: el tablero NO TIENE PAREDES, asi que lo que decide no es
            # que el cuerpo entero quepa sino que el rover no vuelque. El apoyo
            # son las ruedas (bajo el chasis) y las puntas de las paletas, que
            # rozan el piso. Una paleta en voladizo sobre el vacio solo pierde
            # ese apoyo; el rover sigue sostenido por las ruedas. Por eso la
            # condicion se evalua sobre el CHASIS y no sobre la envolvente: usar
            # la envolvente con las paletas incluidas es el modelo de pared, no
            # el de precipicio, y da dos ordenes de magnitud mas de eventos.
            # Se PROYECTA de vuelta adentro (no se revierte el paso, porque
            # revertir deja al rover atrapado si ya estaba fuera).
            E = rv.cuerpo()
            corr = np.zeros(2)
            for k in (0, 1):
                lo, hi = float(np.min(E[:, k])), float(np.max(E[:, k]))
                if lo < -BORDE: corr[k] += (-BORDE - lo)
                if hi > COLS * CELL + BORDE: corr[k] -= (hi - COLS * CELL - BORDE)
            fuera = bool(np.any(np.abs(corr) > 1e-9))
            if fuera:
                if not getattr(rv, '_estaba_fuera', False):
                    self.salidas += 1          # se cuentan EVENTOS, no pasos
                rv._estaba_fuera = True
                rv.p = rv.p + corr
            else:
                rv._estaba_fuera = False

        # contacto rover-cubo, pieza por pieza
        for rv in self.rovers:
            CUERPO = rv.cuerpo()
            PALETAS = rv.paletas()
            for cb in self.cubos:
                # --- contencion lateral por las paletas: contienen, no empujan
                for PA in PALETAS:
                    hay, mtv = sat_mtv(PA, cb.esquinas())
                    if not hay:
                        continue
                    # se resuelve la penetracion desplazando el cubo, sin
                    # transmitir avance ni rotacion: la pared contiene.
                    cb.c = cb.c + mtv
                    self.contactos_paleta += 1
                # --- empuje por el cuerpo
                B = cb.esquinas()
                hay, mtv = sat_mtv(CUERPO, B)
                if not hay:
                    continue
                # solo empuja si el contacto es por delante del centro del rover
                rel = cb.c - rv.p
                adelante = float(np.dot(rel, u(rv.theta)))
                if adelante <= 0:
                    # contacto trasero o lateral: se separa sin empujar
                    cb.c = cb.c + mtv
                    continue
                d = float(np.linalg.norm(mtv))
                cb.c = cb.c + mtv
                # rotacion inducida (cuasiestatica, centro instantaneo de rotacion)
                e = float(np.dot(rel, u(rv.theta + 90.0)))
                dth = math.degrees(d * e / self.rg2)
                cb.theta = (cb.theta + max(-5.0, min(5.0, dth))) % 90.0
                # el cubo no sale del tablero
                V = cb.esquinas()
                for k in (0, 1):
                    lo = float(np.min(V[:, k])); hi = float(np.max(V[:, k]))
                    if lo < -BORDE: cb.c[k] += (-BORDE - lo)
                    if hi > COLS * CELL + BORDE: cb.c[k] -= (hi - COLS * CELL - BORDE)

        # colision rover-rover
        activas = set()
        if len(self.rovers) > 1:
            for i in range(len(self.rovers)):
                for j in range(i + 1, len(self.rovers)):
                    hay, mtv = False, None
                    for PI in self.rovers[i].piezas():
                        for PJ in self.rovers[j].piezas():
                            h, m = sat_mtv(PI, PJ)
                            if h and (mtv is None or
                                      np.linalg.norm(m) > np.linalg.norm(mtv)):
                                hay, mtv = True, m
                    if hay:
                        par = (i, j)
                        if par not in getattr(self, '_colis_activas', set()):
                            self.colisiones += 1
                            self._colis_activas = getattr(self, '_colis_activas', set()) | {par}
                        self.rovers[j].p = self.rovers[j].p + 0.5 * mtv
                        self.rovers[i].p = self.rovers[i].p - 0.5 * mtv
                        activas.add((i, j))
        self._colis_activas = activas

        # contabilidad de entregas deshechas
        for cb in self.cubos:
            g = np.array(self.depots[cb.color]) * CELL
            dentro = bool(np.linalg.norm(cb.c - g) <= 1.0 * CELL)
            if dentro:
                self._estuvo_entregado.add(cb.color)
            elif cb.color in self._estuvo_entregado:
                self.desentregas += 1
                self._estuvo_entregado.discard(cb.color)

        self.t_ms += int(round(dt * 1000))

    # ---------------------------------------------------------- percepcion --
    def fraccion_ocluida(self, cb):
        """Fraccion de la SILUETA del cubo tapada por la silueta de un rover.

        CORRECCION IMPORTANTE. Un primer modelo comparaba las huellas en el piso
        (base del cubo contra rectangulo del rover). Ese modelo NO puede producir
        oclusion durante un empuje: vistos desde arriba, los dos cuerpos se tocan
        pero no se solapan. Es exactamente el mismo defecto que se le senala al
        simulador oficial en el hallazgo H-01.

        Lo que el detector ve no es la huella sino la SILUETA: base mas tapa
        desplazada por el paralaje. El rover mide 90 mm de alto y el cubo 60, asi
        que la silueta del rover se proyecta sobre la del cubo y la tapa. Ese es
        el mecanismo real de la oclusion del 22% que reporta el sistema oficial.
        """
        Sc = self.silueta(cb.esquinas(), cb.L)
        area = area_poligono(Sc)
        if area <= 0:
            return 0.0
        tapado = 0.0
        for rv in self.rovers:
            # solo el CUERPO ocluye: las paletas miden 14 mm de alto (medido
            # sobre el rover fisico) y su silueta es despreciable frente a la
            # del chasis, que llega a 90 mm.
            Sr = self.silueta(rv.cuerpo(), 90.0)
            inter = recortar_convexo(Sc, Sr)
            tapado = max(tapado, area_poligono(inter))
        return min(1.0, tapado / area)

    def observar(self):
        """Construye el mensaje del contrato v1."""
        self.seq += 1
        rovers = []
        for rv in self.rovers:
            if self.rng.random() > self.p_perd:
                rv.rep = (rv.p + self.rng.normal(0, self.sig_p, 2),
                          (rv.theta + self.rng.normal(0, self.sig_t)) % 360.0)
                rv.ts_visto = self.t_ms
            rovers.append({'id': rv.id,
                           'col': round(rv.rep[0][0] / CELL, 3),
                           'row': round(rv.rep[0][1] / CELL, 3),
                           'theta': round(rv.rep[1], 2),
                           'age_ms': max(0, self.t_ms - rv.ts_visto)})
        cubes = []
        for cb in self.cubos:
            f = self.fraccion_ocluida(cb)
            if f < 0.45:                        # umbral de deteccion utilizable
                cb.rep = cb.c + self.rng.normal(0, self.sig_p, 2)
                cb.ts_visto = self.t_ms
            cubes.append({'color': cb.color,
                          'col': round(cb.rep[0] / CELL, 3),
                          'row': round(cb.rep[1] / CELL, 3),
                          'age_ms': max(0, self.t_ms - cb.ts_visto),
                          '_oclusion': round(f, 3)})
        msg = {'v': 1, 'seq': self.seq, 'ts_ms': self.t_ms, 'phase': self.fase,
               'grid': {'cols': COLS, 'rows': ROWS, 'cell_mm': CELL},
               'rovers': rovers, 'cubes': cubes, 'obstacles': [],
               'start': {'col': 2.5, 'row': 2.5},
               'depots': [{'color': k, 'col': v[0], 'row': v[1]} for k, v in self.depots.items()]}
        self.cola.append((self.t_ms, msg))
        # entrega con latencia
        entregable = None
        while self.cola and self.cola[0][0] + self.lat <= self.t_ms:
            entregable = self.cola.pop(0)[1]
        return entregable

    # ------------------------------------------------------------ metricas --
    def entregados(self, tol_celdas=1.0):
        n = 0
        for cb in self.cubos:
            g = np.array(self.depots[cb.color]) * CELL
            if np.linalg.norm(cb.c - g) <= tol_celdas * CELL:
                n += 1
        return n

    def distancias_al_deposito(self):
        return {cb.color: float(np.linalg.norm(cb.c - np.array(self.depots[cb.color]) * CELL) / CELL)
                for cb in self.cubos}

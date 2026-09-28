"""Planificador de la capa geometrica del modelo. Consume telemetria v1."""
from __future__ import annotations
import math
import numpy as np
from modelo import u, arg_de, delta, rho_P, soporte_formula, rho_puesta, e_max

CELL = 20.0
L    = 60.0
W       = 99.50   # huella exterior (lengueta a lengueta), DXF de piezas separadas
W_CANAL = 93.50   # separacion entre caras internas de los paneles laterales
LAM     = 94.00   # largo del chasis
LAMBDA  = 55.0    # alcance de las paletas por delante de la placa, MEDIDO
# La huella es ASIMETRICA respecto del centro del chasis: atras LAM/2, adelante
# LAM/2 + LAMBDA. Ignorarlo subestima el envolvente un 66 % (hallazgo H-12).
X_ATRAS = -LAM / 2.0
X_ADEL  = LAM / 2.0 + LAMBDA
S    = 40.0
BORDE = 3.5

RHO_CT_MAX = L * math.sqrt(2) / 2 + LAM / 2
RHO_PU     = rho_puesta(L, LAM, S)
RHO_PU_C   = RHO_PU / CELL
RETIRADA_C = 60.0 / CELL

# --- anillo de maniobra (consecuencia del hallazgo H-12) --------------------
# El envolvente real del rover, con las paletas incluidas, mide 113.5 mm, no
# los 68.4 del contorno del chasis. Rodear el cubo a RHO_PU = 129.4 mm lo BARRE
# con las paletas. Hace falta un anillo mas amplio para girar, y entrar despues
# en linea recta por el propio eje de empuje.
R_ENVOLVENTE  = math.hypot(X_ADEL, W / 2.0)
RHO_MAN       = R_ENVOLVENTE + L * math.sqrt(2) / 2 + 5.0
RHO_MAN_C     = RHO_MAN / CELL
# El semiancho del pasillo de entrada NO es un parametro libre: es exactamente
# e_max, la tolerancia lateral del embudo. Si el rover entra con |e| mayor, la
# punta de la paleta golpea el cubo de costado en vez de capturarlo. Aca aparece
# el hallazgo H-11 dentro del lazo cerrado: 4.32 mm = 0.216 celdas de pasillo.
E_MAX_PEOR    = (W_CANAL - L * math.sqrt(2)) / 2.0
TOL_PASILLO_C = E_MAX_PEOR / CELL
# distancia a la que las puntas de las paletas alcanzan al cubo
LON_CONTACTO_C = (X_ADEL + L * math.sqrt(2) / 2) / CELL
# Retirada antes de girar. En contacto el centro del rover esta a RHO_CT del
# centro del cubo y las puntas de las paletas LO SOBREPASAN. Para poder girar
# sin barrerlo hay que retroceder hasta que las puntas queden a L*sqrt2/2 del
# centro del cubo, es decir hasta LON_CONTACTO_C. La diferencia es exactamente
# lambda: el rover tiene que desandar el canal antes de darse vuelta.
RETIRADA_SEG_C = LON_CONTACTO_C - RHO_CT_MAX / CELL + 0.15

TAB_MIN, TAB_MAX = -BORDE, 43.0 + BORDE


def _esquinas_rover_c(a, phi):
    """Esquinas de la huella COMPLETA (chasis + paletas), en celdas."""
    up = u(phi); pe = u(phi + 90.0)
    hw = (W / 2) / CELL
    return np.array([a + (sa / CELL) * up + sb * hw * pe
                     for sa in (X_ATRAS, X_ADEL) for sb in (-1, 1)])


# Despeje lateral que el rover necesita al pasar junto a un cubo que NO es su
# objetivo: semiancho de la huella mas la semidiagonal del cubo. Con la huella
# rectangular anterior el roce era un empujoncito; con las paletas, la punta
# ENGANCHA el cubo y lo arrastra. Por eso los otros cubos pasan a ser obstaculos
# de navegacion, cosa que el planificador no contemplaba (hallazgo H-14).
DESPEJE_C = (W / 2.0 + L * math.sqrt(2) / 2.0) / CELL


def _dist_punto_segmento(q, p0, p1):
    v = p1 - p0
    n2 = float(np.dot(v, v))
    if n2 < 1e-12:
        return float(np.linalg.norm(q - p0)), 0.0
    t = float(np.clip(np.dot(q - p0, v) / n2, 0.0, 1.0))
    return float(np.linalg.norm(q - (p0 + t * v))), t


def rodear_cubos(p, a, centros_otros, despeje=None):
    """Devuelve un waypoint que esquiva los cubos ajenos, o None si no hace falta.

    No se busca la ruta optima: basta con desplazar el punto de paso hacia el
    lado libre hasta recuperar el despeje. El grafo de visibilidad completo del
    modelo daria la ruta minima, pero aqui interesa medir el efecto del enganche,
    no exprimir la longitud.
    """
    if despeje is None:
        despeje = DESPEJE_C
    peor, q_peor, t_peor = despeje, None, None
    for q in centros_otros:
        dd, t = _dist_punto_segmento(np.asarray(q, float), p, a)
        if dd < peor and 0.02 < t < 0.98:
            peor, q_peor, t_peor = dd, np.asarray(q, float), t
    if q_peor is None:
        return None
    v = a - p
    nv = float(np.linalg.norm(v))
    if nv < 1e-9:
        return None
    perp = np.array([-v[1], v[0]]) / nv
    proy = p + t_peor * v
    lado = 1.0 if float(np.dot(proy - q_peor, perp)) >= 0 else -1.0
    return q_peor + lado * (despeje + 0.4) * perp


def puesta_valida(a, phi):
    """El tablero no tiene paredes: lo que no puede salirse es el CHASIS.

    Las paletas pueden quedar en voladizo sobre el vacio. Rozan el piso y forman
    parte del poligono de apoyo, pero perder ese apoyo no vuelca al rover, que
    sigue sostenido por las ruedas bajo el chasis. Exigir que la huella completa
    -paletas incluidas- quepa dentro del tablero es el modelo de PARED, no el de
    precipicio, y con las paletas medidas (55 mm) invalida casi todas las poses
    de puesta junto a los depositos, que estan a 2,5 celdas del borde.
    """
    up = u(phi); pe = u(phi + 90.0)
    hl, hw = (LAM / 2) / CELL, (W / 2) / CELL
    E = np.array([a + sa * hl * up + sb * hw * pe
                  for sa in (-1, 1) for sb in (-1, 1)])
    return bool(np.all((E >= TAB_MIN) & (E <= TAB_MAX)))


def punto_puesta(c, g):
    d = g - c
    n = float(np.linalg.norm(d))
    if n < 1e-9:
        return None, None, None
    d = d / n
    phi = float(arg_de(d))
    return c - RHO_PU_C * d, phi, d


def _dist_pared(p):
    return float(min(p[0] - TAB_MIN, TAB_MAX - p[0], p[1] - TAB_MIN, TAB_MAX - p[1]))


def _cubo_en_tablero(z):
    E = np.array([z + np.array([sx, sy]) * (L / 2) / CELL
                  for sx in (-1, 1) for sy in (-1, 1)])
    return bool(np.all((E >= TAB_MIN) & (E <= TAB_MAX)))


def empujes_posibles(c, hop=6.0, paso=7.5):
    """Direcciones de empuje con punto de puesta valido y cubo que queda en tablero."""
    out = []
    for ang in np.arange(0.0, 360.0, paso):
        d = u(ang)
        if not puesta_valida(c - RHO_PU_C * d, ang):
            continue
        z = c + hop * d
        if not _cubo_en_tablero(z):
            continue
        out.append((float(ang), z))
    return out


def waypoint_intermedio(c, g):
    """Tramo intermedio cuando el empuje directo no es factible.

    Regla en dos escalones:
      1. GOLOSA: entre las direcciones posibles, la que mas acerca al deposito.
      2. ESCAPE (hallazgo H-09): si NINGUNA acerca al deposito -el cubo esta en
         la zona sin progreso, un 3,7% de la cancha junto a la esquina opuesta-,
         se acepta empeorar y se elige la direccion que mas aleja de la pared mas
         cercana, es decir la que lleva el cubo hacia el centro de la cancha.
         Sin este escalon un planificador goloso se queda bloqueado ahi.
    """
    opciones = []
    for hop in (10.0, 6.0, 3.0):
        opciones = empujes_posibles(c, hop)
        if opciones:
            break
    if not opciones:
        return None
    d0 = float(np.linalg.norm(g - c))
    progresan = [(float(np.linalg.norm(g - z)), z) for _, z in opciones
                 if float(np.linalg.norm(g - z)) < d0 - 1e-9]
    if progresan:
        return min(progresan)[1]
    # --- escape: alejarse de la PARED (hallazgo H-09) ------------------------
    # Solo se acepta empeorar la distancia al deposito si a cambio el cubo se
    # despega de la pared. Si ni siquiera eso es posible, el cubo esta en la
    # zona trampa y NO hay secuencia de empujes que lo saque: se devuelve None
    # para que el planificador lo abandone en vez de gastar la ronda.
    d_pared_actual = _dist_pared(c)
    despegan = [(-_dist_pared(z), z) for _, z in opciones
                if _dist_pared(z) > d_pared_actual + 1e-9]
    if despegan:
        return min(despegan)[1]
    return None


class Planificador:
    """Maquina de estados por rover. Todo en CELDAS, como la telemetria."""

    TOL_RUMBO   = 3.0     # grados
    TOL_PUESTA  = 0.35    # celdas
    TOL_DESTINO = 1.0     # celdas (coincide con la metrica de entrega)
    TOL_ENTREGA = 0.70    # celdas: la metrica de la mision da por entregado un
                          # cubo a <= 1.0 celda del deposito. Seguir empujando
                          # para "mejorarlo" lo pasa de largo y lo pierde.
    V_NOM       = 6.0     # celdas/s
    V_EMPUJE    = 3.0
    W_NOM       = 90.0    # grados/s
    LAT_MAX     = 500.0
    K_LATERAL   = 4.0     # grados de correccion por celda de error lateral

    def __init__(self, mi_id, corte_latencia=True):
        self.id = mi_id
        self.estado = 'SELECT'
        self.objetivo = None
        self.wp = None
        self.corte_latencia = corte_latencia
        self.frenos = 0
        self.traza = []

    # ------------------------------------------------------------ utilidad --
    @staticmethod
    def _yo(msg, mid):
        for r in msg['rovers']:
            if r['id'] == mid:
                return r
        return None

    def costo(self, p, th, c, g):
        a, phi, _ = punto_puesta(c, g)
        if a is None:
            return 1e9
        d = a - p
        dist = float(np.linalg.norm(d))
        if dist < 1e-9:
            return abs(float(delta(phi, th))) / self.W_NOM
        phi0 = float(arg_de(d))
        A = abs(float(delta(phi0, th))); B = abs(float(delta(phi, phi0)))
        # regla derivada (hallazgo H-04): retroceso gana si A + B > 180
        giros = min(A + B, 360.0 - (A + B))
        return giros / self.W_NOM + dist / self.V_NOM + float(np.linalg.norm(g - c)) / self.V_EMPUJE

    # -------------------------------------------------------------- decidir --
    # --- coordinacion minima (fuera del alcance del modelo; ver N3.8) -------
    # separacion garantizada sin solape para cualquier rumbo: suma de circunradios
    R_CEDER = math.hypot(W, LAM) / CELL

    def _debo_ceder(self, msg, p, th):
        """Cede el paso al rover de id menor si esta cerca y por delante.

        Es la coordinacion MINIMA que el lazo cerrado necesita. La capa de
        coordinacion multi-agente completa esta fuera del alcance del modelo;
        esto implementa solo la condicion geometrica de N3.8 aplicada de a pares.
        """
        cerca = False
        for o in msg['rovers']:
            if o['id'] == self.id:
                continue
            q = np.array([o['col'], o['row']])
            if float(np.linalg.norm(q - p)) <= self.R_CEDER and o['id'] < self.id:
                cerca = True
        # antibloqueo: tras 3 s cediendo sin parar, se rompe la simetria 2 s
        n = getattr(self, '_ceder_seguidos', 0)
        pausa = getattr(self, '_ceder_pausa', 0)
        if pausa > 0:
            self._ceder_pausa = pausa - 1
            self._ceder_seguidos = 0
            return False
        if cerca:
            n += 1
            self._ceder_seguidos = n
            if n > 60:
                self._ceder_pausa = 40
                self._ceder_seguidos = 0
                return False
            return True
        self._ceder_seguidos = 0
        return False

    # --- vigilante de vivacidad -------------------------------------------
    # HALLAZGO H-13. El planificador no tenia ninguna garantia de vivacidad:
    # varias ramas devuelven (0,0) y, si la condicion que las dispara no cambia
    # sola, el rover queda inmovil el resto de la ronda. Con la huella chica el
    # caso no se alcanzaba; al incorporar las paletas (H-12) los cubos se
    # detienen mas lejos del deposito y la rama "sin pose valida, sin waypoint y
    # a menos de 3 celdas del deposito" pasa a ser alcanzable: el rover se
    # congelaba con dos cubos sin entregar. Este vigilante lo detecta por sintoma
    # -objetivo fijo y rover quieto- y libera el objetivo.
    T_QUIETO_MS = 2500
    D_QUIETO_C  = 0.15

    def _vigilar(self, p, th, ahora_ms, tomados):
        """Devuelve True si hubo que abandonar el objetivo por inmovilidad."""
        ref = getattr(self, '_vig_p', None)
        if ref is None or self.objetivo != getattr(self, '_vig_obj', None) \
                or float(np.linalg.norm(p - ref)) > self.D_QUIETO_C:
            self._vig_p, self._vig_obj, self._vig_t = p.copy(), self.objetivo, ahora_ms
            return False
        if self.objetivo is not None and ahora_ms - self._vig_t > self.T_QUIETO_MS:
            self.bloqueos_liberados = getattr(self, 'bloqueos_liberados', 0) + 1
            self.irrecuperables = getattr(self, 'irrecuperables', set()) | {self.objetivo}
            tomados.pop(self.objetivo, None)
            self.estado = 'SELECT'; self.objetivo = None
            self._vig_p = None
            return True
        return False

    def _retirarse(self, p, th):
        """Arranca la retirada recta antes de girar. Ver RETIRADA_SEG_C.

        Sin esto el rover suelta el cubo, gira en el lugar y BARRE con la paleta
        el cubo que acaba de dejar, sacandolo del deposito. Es el modo de falla
        mas caro que aparecio al modelar el rover como U: deshace trabajo ya
        hecho. El modelo lo tenia contemplado -el costo de un quiebre incluye la
        retirada- pero el planificador nunca la ejecutaba.
        """
        self.estado = 'RETIRADA'
        self._ret_meta = p - RETIRADA_SEG_C * u(th)
        self.retiradas = getattr(self, 'retiradas', 0) + 1
        return 0.0, 0.0

    def decidir(self, msg, ahora_ms, tomados):
        """Devuelve (v, w) en celdas/s y grados/s."""
        if msg is None or msg.get('phase') != 'RUNNING':
            return 0.0, 0.0
        lat = ahora_ms - msg['ts_ms']
        if self.corte_latencia and lat > self.LAT_MAX:
            self.frenos += 1
            return 0.0, 0.0
        yo = self._yo(msg, self.id)
        if yo is None:
            return 0.0, 0.0
        p = np.array([yo['col'], yo['row']]); th = yo['theta']
        if self.estado == 'RETIRADA':
            meta = getattr(self, '_ret_meta', None)
            if meta is None or float(np.linalg.norm(meta - p)) <= 0.20:
                self.estado = 'SELECT'; self._ret_meta = None
                return 0.0, 0.0
            return -self.V_NOM * 0.6, 0.0
        if self._vigilar(p, th, ahora_ms, tomados):
            return 0.0, 0.0
        if self._debo_ceder(msg, p, th):
            self.cedidos = getattr(self, 'cedidos', 0) + 1
            return 0.0, 0.0
        depots = {d['color']: np.array([d['col'], d['row']]) for d in msg['depots']}
        cubos = {c['color']: c for c in msg['cubes']}

        # --- seleccion de objetivo ------------------------------------------
        if self.estado == 'SELECT' or self.objetivo not in cubos:
            mejor = None
            for color, cb in cubos.items():
                if color in getattr(self, 'irrecuperables', set()):
                    continue
                if color in tomados and tomados[color] != self.id:
                    continue
                c = np.array([cb['col'], cb['row']]); g = depots[color]
                if float(np.linalg.norm(c - g)) <= self.TOL_DESTINO:
                    continue
                k = self.costo(p, th, c, g)
                if mejor is None or k < mejor[0]:
                    mejor = (k, color)
            if mejor is None:
                return 0.0, 0.0
            self.objetivo = mejor[1]
            tomados[self.objetivo] = self.id
            self.estado = 'IR_A_PUESTA'
            self.wp = None

        cb = cubos[self.objetivo]
        c = np.array([cb['col'], cb['row']])
        g = depots[self.objetivo]

        if float(np.linalg.norm(c - g)) <= self.TOL_DESTINO:
            tomados.pop(self.objetivo, None)
            self.estado = 'SELECT'; self.objetivo = None
            return 0.0, 0.0

        # --- criterio de terminacion: el cubo ya esta entregado -------------
        # La mision da por entregado un cubo a menos de TOL_ENTREGA del deposito.
        # Sin esta condicion el planificador seguia empujando para "mejorarlo" y
        # lo pasaba de largo, dejandolo del otro lado del deposito -contra el
        # borde- donde ya no hay pose de puesta valida y el cubo se pierde. Es un
        # caso de sobre-optimizacion: perseguir un optimo que la mision no pide.
        if float(np.linalg.norm(g - c)) <= self.TOL_ENTREGA:
            self.entregados_propios = getattr(self, 'entregados_propios', set()) | {self.objetivo}
            tomados.pop(self.objetivo, None)
            self.objetivo = None
            return self._retirarse(p, th)

        # --- el corredor de empuje no puede pasar sobre otro cubo ----------
        # El modelo define el corredor K(c,g) = [c,g] (+) D_rho y da la condicion
        # de compatibilidad entre dos tareas (prueba N3.8), pero el planificador
        # solo la usaba entre rovers. Con la huella rectangular anterior pasar
        # sobre otro cubo lo rozaba; con las paletas lo ENGANCHA y lo arrastra,
        # y el caso mas caro es sacar de su deposito un cubo YA ENTREGADO. Aca se
        # verifica el corredor contra los demas cubos y, si lo invade, se empuja
        # primero hacia un objetivo intermedio que lo esquiva.
        otros_c = [np.array([o['col'], o['row']]) for o in msg.get('cubes', [])
                   if o.get('color') != self.objetivo]
        g_rodeo = rodear_cubos(c, g, otros_c)
        if g_rodeo is not None:
            self.rodeos_corredor = getattr(self, 'rodeos_corredor', 0) + 1
            g = g_rodeo

        # --- destino del tramo: directo o via waypoint ----------------------
        a, phi, d = punto_puesta(c, g)
        if a is None:
            return 0.0, 0.0
        if not puesta_valida(a, phi):
            # se recalcula en cada ciclo: el waypoint depende de donde esta el
            # cubo AHORA, y cachearlo hace que el rover persiga un punto viejo.
            self.wp = waypoint_intermedio(c, g)
            if self.wp is not None:
                a, phi, d = punto_puesta(c, self.wp)
                g = self.wp
                if a is None or not puesta_valida(a, phi):
                    return 0.0, 0.0
            elif float(np.linalg.norm(g - c)) > 3.0:
                # cubo irrecuperable (zona trampa, H-09): se abandona y se libera
                # para no gastar el resto de la ronda empujando contra una pared.
                # No se abandona si el cubo ya esta a menos de 3 celdas del
                # deposito: ahi la falta de puesta valida es el efecto de que el
                # propio deposito esta en una esquina, no una trampa real.
                self.irrecuperables = getattr(self, 'irrecuperables', set()) | {self.objetivo}
                tomados.pop(self.objetivo, None)
                self.estado = 'SELECT'; self.objetivo = None
                return 0.0, 0.0
        else:
            self.wp = None

        # --- maquina de estados ---------------------------------------------
        if self.estado == 'IR_A_PUESTA':
            # H-12: no se puede ir en linea recta a la pose de puesta desde
            # cualquier lado. Con las paletas el envolvente mide 113.5 mm y
            # rodear el cubo a RHO_PU lo barre. El acceso se hace en dos tramos:
            # primero al anillo de maniobra de radio RHO_MAN, y desde ahi en
            # linea recta hacia adentro por el propio eje de empuje.
            rel_c = p - c
            lat_c = float(np.dot(rel_c, u(phi + 90.0)))
            lon_c = float(np.dot(rel_c, u(phi)))
            # Se esta en el pasillo si el rover ya esta alineado con el eje de
            # empuje y por detras del cubo. La condicion sobre lon_c solo pide
            # estar detras: exigir ademas una distancia minima hacia que, apenas
            # empezado el empuje, el rover se creyera fuera del pasillo y saliera
            # a rehacer el anillo, con lo que la mision no avanzaba nunca.
            en_pasillo = (abs(lat_c) <= TOL_PASILLO_C) and (lon_c < -0.5)
            if not en_pasillo:
                # Objetivo: la entrada del pasillo, sobre el eje de empuje, a
                # RHO_MAN del cubo. Ese punto tiene que ser ALCANZABLE: se valida
                # con la misma huella completa que el punto de puesta, porque un
                # anillo que cae fuera del tablero deja al rover persiguiendo un
                # destino imposible y la mision se congela (fue exactamente lo
                # que paso al incorporar las paletas: el rover quedaba proyectado
                # contra el borde, oscilando, hasta agotar el tiempo).
                rho_e = RHO_MAN_C
                while rho_e > RHO_PU_C and not puesta_valida(c - rho_e * d, phi):
                    rho_e -= 0.25
                if rho_e <= RHO_PU_C:
                    # no cabe ningun anillo: se entra directo a la pose de puesta
                    # aceptando que el giro barra el cubo. Es una DEGRADACION, y
                    # se contabiliza para poder medir cuanto cuesta.
                    if getattr(self, '_deg_obj', None) != self.objetivo:
                        self.entradas_degradadas = getattr(self, 'entradas_degradadas', 0) + 1
                        self._deg_obj = self.objetivo
                    rho_e = RHO_PU_C
                a = c - rho_e * d
            # los OTROS cubos son obstaculos: pasar cerca los engancha
            otros = [np.array([o['col'], o['row']]) for o in msg.get('cubes', [])
                     if o.get('color') != self.objetivo]
            wp_c = rodear_cubos(p, a, otros)
            if wp_c is not None:
                self.rodeos = getattr(self, 'rodeos', 0) + 1
                a = wp_c
            err = a - p
            dist = float(np.linalg.norm(err))
            if dist <= self.TOL_PUESTA:
                if abs(float(delta(phi, th))) <= self.TOL_RUMBO:
                    self.estado = 'EMPUJAR'
                    return 0.0, 0.0
                return 0.0, self.W_NOM * math.copysign(1.0, float(delta(phi, th)))
            phi0 = float(arg_de(err))
            A = abs(float(delta(phi0, th))); B = abs(float(delta(phi, phi0)))
            if A + B > 180.0:                      # H-04: conviene retroceder
                rumbo = (phi0 + 180.0) % 360.0
                signo = -1.0
            else:
                rumbo = phi0
                signo = 1.0
            dth = float(delta(rumbo, th))
            if abs(dth) > self.TOL_RUMBO:
                return 0.0, self.W_NOM * math.copysign(1.0, dth)
            return signo * min(self.V_NOM, dist * 3.0), 0.0

        if self.estado == 'EMPUJAR':
            # error lateral respecto de la linea de empuje que pasa por el cubo
            rel = p - c
            lateral = float(np.dot(rel, u(phi + 90.0)))
            long_ = float(np.dot(rel, u(phi)))
            self.traza.append((lateral, cb.get('age_ms', 0), cb.get('_oclusion', 0.0)))
            if long_ > 0.0:            # el rover paso del cubo: reencuadrar
                self.estado = 'IR_A_PUESTA'
                return 0.0, 0.0
            corr = float(np.clip(-self.K_LATERAL * lateral, -25.0, 25.0))
            dth = float(delta(phi + corr, th))
            if abs(dth) > 25.0:
                return 0.0, self.W_NOM * math.copysign(1.0, dth)
            return self.V_EMPUJE, float(np.clip(dth * 3.0, -self.W_NOM, self.W_NOM))

        return 0.0, 0.0

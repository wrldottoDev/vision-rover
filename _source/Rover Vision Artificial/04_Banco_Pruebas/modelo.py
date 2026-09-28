"""Implementación de referencia del modelo matemático.

Marco de cancha: x = col (derecha), y = row (ABAJO). theta antihorario visual.
Todas las funciones son unidad-agnósticas: se les pasa L en mm o en celdas y
devuelven en la misma unidad.
"""
from __future__ import annotations
import math
import numpy as np

# ---------------------------------------------------------------- marco -----

def u(alpha_deg):
    """Versor de rumbo u(alpha) = (cos a, -sin a). Acepta escalar o array."""
    a = np.radians(np.asarray(alpha_deg, dtype=float))
    return np.stack([np.cos(a), -np.sin(a)], axis=-1)


def R(alpha_deg):
    """Operador de rotación de cancha (2x2). Escalar solamente."""
    a = math.radians(float(alpha_deg))
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, s], [-s, c]], dtype=float)


def arg_de(vec):
    """Recupera alpha tal que vec ∝ u(alpha). Devuelve grados en [0,360)."""
    v = np.asarray(vec, dtype=float)
    return np.degrees(np.arctan2(-v[..., 1], v[..., 0])) % 360.0


def delta(a, b):
    """Diferencia angular con signo en (-180, 180]."""
    return ((np.asarray(a, float) - np.asarray(b, float) + 180.0) % 360.0) - 180.0


def rho_P(a, b, P=360.0):
    """Distancia angular sin signo con periodo P. Rango [0, P/2]."""
    return np.abs(((np.asarray(a, float) - np.asarray(b, float) + P / 2.0) % P) - P / 2.0)


def cruz(a, b):
    """Producto cruz escalar a ∧ b."""
    a = np.asarray(a, float); b = np.asarray(b, float)
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


# ------------------------------------------------------------- cuadrado -----

def semidiagonal(L):
    return L / math.sqrt(2.0)


def vertices(c, L, theta_deg):
    """Los 4 vértices por la parametrización trigonométrica, ec. (17)."""
    c = np.asarray(c, dtype=float)
    rho = semidiagonal(L)
    ks = np.arange(4)
    return c + rho * u(theta_deg + 45.0 + 90.0 * ks)


def vertices_por_recursion(c, v0):
    """Los 4 vértices por la recursión de 90°, ec. (20): (dx,dy) -> (dy,-dx)."""
    c = np.asarray(c, float); v0 = np.asarray(v0, float)
    out = [v0.copy()]
    d = v0 - c
    for _ in range(3):
        d = np.array([d[1], -d[0]])
        out.append(c + d)
    return np.array(out)


def aristas(V):
    """Vectores de arista e_k = v_{k+1} - v_k."""
    return np.roll(V, -1, axis=0) - V


def puntos_medios(V):
    return 0.5 * (V + np.roll(V, -1, axis=0))


def normales_exteriores(c, L, theta_deg):
    """n_k = u(theta + 90 + 90k), ec. (23)."""
    ks = np.arange(4)
    return u(theta_deg + 90.0 + 90.0 * ks)


# ------------------------------------------------- estimadores familia B ----

def est_B1(c, v0):
    """Vértice de referencia + centro -> theta mod 90, ec. (31)."""
    return (arg_de(np.asarray(v0, float) - np.asarray(c, float)) - 45.0) % 90.0


def est_B2_theta(va, vb):
    """Arista -> theta mod 90, ec. (32). Invariante al sentido de recorrido."""
    e = np.asarray(vb, float) - np.asarray(va, float)
    return arg_de(e) % 90.0


def est_B2_centro(va, vb, L, testigo):
    """Arista + testigo interior -> centro, ecs. (33)-(34)."""
    va = np.asarray(va, float); vb = np.asarray(vb, float)
    e = vb - va
    ne = np.linalg.norm(e)
    n = np.array([e[1], -e[0]]) / ne          # R(90) e / |e|
    m = 0.5 * (va + vb)
    sigma = math.copysign(1.0, float(np.dot(np.asarray(testigo, float) - m, n)))
    return m + sigma * (L / 2.0) * n


def est_B3(v0, v2):
    """Diagonal -> (centro, theta mod 90)."""
    v0 = np.asarray(v0, float); v2 = np.asarray(v2, float)
    c = 0.5 * (v0 + v2)
    th = (arg_de(v2 - v0) - 45.0) % 90.0
    return c, th


def plantilla(L, K):
    """Vértices de la plantilla w_k = rho·u(45+90k) para los índices K."""
    rho = semidiagonal(L)
    return rho * u(45.0 + 90.0 * np.asarray(K, float))


def est_B4(obs, K, L):
    """Procrustes ortogonal en SE(2) con plantilla conocida, ec. (37).

    obs: (n,2) observaciones; K: índices de vértice correspondientes.
    """
    V = np.asarray(obs, float).reshape(-1, 2)
    W = plantilla(L, K)
    vb = V.mean(axis=0); wb = W.mean(axis=0)
    Vt = V - vb; Wt = W - wb
    A = float(np.sum(Vt * Wt))
    B = float(np.sum(Vt[:, 0] * Wt[:, 1] - Vt[:, 1] * Wt[:, 0]))
    th = math.degrees(math.atan2(B, A))
    c = vb - R(th) @ wb
    return c, th % 90.0, (A, B)


def est_B4_vect(V, K, L):
    """Versión vectorizada para Monte Carlo. V: (m, n, 2)."""
    W = plantilla(L, K)                      # (n,2)
    vb = V.mean(axis=1, keepdims=True)       # (m,1,2)
    wb = W.mean(axis=0, keepdims=True)       # (1,2)
    Vt = V - vb
    Wt = W - wb                              # (n,2)
    A = np.einsum('mnk,nk->m', Vt, Wt)
    B = np.einsum('mn,n->m', Vt[:, :, 0], Wt[:, 1]) - np.einsum('mn,n->m', Vt[:, :, 1], Wt[:, 0])
    return np.degrees(np.arctan2(B, A))


def rho_efectivo(L, K):
    W = plantilla(L, K)
    return float(np.sqrt(np.mean(np.sum((W - W.mean(axis=0)) ** 2, axis=1))))


# --------------------------------------------------- soporte y anchura ------

def soporte_formula(L, theta_deg, phi_deg):
    """h_S(phi) - c·u(phi) por la forma cerrada, ec. (25)."""
    D = np.radians(np.asarray(phi_deg, float) - np.asarray(theta_deg, float))
    return (L / 2.0) * (np.abs(np.cos(D)) + np.abs(np.sin(D)))


def soporte_explicito(c, L, theta_deg, phi_deg):
    """h_S(phi) - c·u(phi) por maximización sobre los vértices."""
    V = vertices(c, L, theta_deg)
    uu = u(phi_deg)
    return float(np.max((V - np.asarray(c, float)) @ uu))


def anchura(L, theta_deg, phi_deg):
    return 2.0 * soporte_formula(L, theta_deg, phi_deg)


# ------------------------------------------------------------ paralaje ------

def kappa(H, h):
    if h <= 0 or h >= H:
        return 1.0
    return H / (H - h)


def pi_directo(p, N, kap):
    """De donde ESTÁ a donde SE VE, ec. (11)."""
    p = np.asarray(p, float); N = np.asarray(N, float)
    return N + kap * (p - N)


def pi_inverso(q, N, kap):
    """De donde SE VE a donde ESTÁ, ec. (12)."""
    q = np.asarray(q, float); N = np.asarray(N, float)
    return N + (q - N) / kap


def silueta(c, L, theta_deg, N, kap):
    """conv(B ∪ T), ec. (28). Devuelve los 8 puntos generadores."""
    Bp = vertices(c, L, theta_deg)
    T = pi_directo(Bp, N, kap)
    return np.vstack([Bp, T])


# ------------------------------------------------- envolvente convexa -------

def casco_convexo(P, eps_rel=1e-9):
    """Monotone chain con eliminación de colineales por tolerancia relativa."""
    P = np.asarray(P, float)
    escala = max(1.0, float(np.max(np.abs(P))))
    eps = eps_rel * escala * escala
    pts = sorted(map(tuple, P))
    pts = [p for i, p in enumerate(pts) if i == 0 or abs(p[0] - pts[i-1][0]) > 1e-12 or abs(p[1] - pts[i-1][1]) > 1e-12]
    if len(pts) <= 2:
        return np.array(pts)

    def media(seq):
        h = []
        for p in seq:
            while len(h) >= 2:
                o, a = h[-2], h[-1]
                cr = (a[0]-o[0])*(p[1]-o[1]) - (a[1]-o[1])*(p[0]-o[0])
                if cr <= eps:
                    h.pop()
                else:
                    break
            h.append(p)
        return h

    inf = media(pts)
    sup = media(reversed(pts))
    return np.array(inf[:-1] + sup[:-1])


def punto_en_poligono(p, poly):
    """Devuelve True si p está estrictamente dentro del polígono convexo."""
    poly = np.asarray(poly, float)
    p = np.asarray(p, float)
    n = len(poly)
    signos = []
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        signos.append(np.sign(cruz(b - a, p - a)))
    signos = [s for s in signos if s != 0]
    return len(set(signos)) <= 1


# -------------------------------------------------------- planificación -----

def R_rov(W, Lam):
    return 0.5 * math.hypot(W, Lam)


def rho_contacto(L, Lam, theta_deg, phi_deg):
    return soporte_formula(L, theta_deg, phi_deg) + Lam / 2.0


def rho_puesta(L, Lam, s):
    return L * math.sqrt(2.0) / 2.0 + Lam / 2.0 + s


def e_max(W, L, theta_deg, phi_deg):
    return (W - anchura(L, theta_deg, phi_deg)) / 2.0


def costo_GTG(p0, th0, a, th_obj, v, w):
    """Costo giro-traslación-giro, ecs. (52)-(54). Devuelve (T, modo)."""
    p0 = np.asarray(p0, float); a = np.asarray(a, float)
    d = a - p0
    dist = float(np.linalg.norm(d))
    if dist < 1e-12:
        return abs(float(delta(th_obj, th0))) / w, 'giro'
    phi0 = float(arg_de(d))
    Tp = abs(float(delta(phi0, th0))) / w + dist / v + abs(float(delta(th_obj, phi0))) / w
    phir = (phi0 + 180.0) % 360.0
    Tm = abs(float(delta(phir, th0))) / w + dist / v + abs(float(delta(th_obj, phir))) / w
    return (Tp, 'avance') if Tp <= Tm else (Tm, 'retroceso')


def umbral_cuerda(rho_cont, rho_pu):
    return 2.0 * math.degrees(math.acos(min(1.0, rho_cont / rho_pu)))

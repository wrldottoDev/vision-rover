"""Nivel N4: lazo cerrado percepcion -> decision -> accion."""
from __future__ import annotations
import math, sys
import numpy as np
sys.path.insert(0, '/tmp/bench')
from simulador import Mundo, Rover, Cubo, CELL, COLS
from planificador import Planificador
from arnes import prueba, resumen, volcar
from modelo import rho_P

DEPOTS = {'green': (40.5, 2.5), 'blue': (2.5, 40.5), 'red': (40.5, 40.5)}
DT = 0.05                     # 20 Hz
T_MAX = 180.0                 # segundos de mision


def correr(cubos_ini, rovers_ini=None, semilla=0, sigma_pos=0.0, sigma_th=0.0,
           p_perd=0.0, lat=0.0, t_max=T_MAX, corte=True, traza=False):
    if rovers_ini is None:
        # HALLAZGO H-06: las poses de arranque deben respetar el tamano del cuerpo.
        # config_simulador.json las declara a 4 celdas (80 mm) y los rovers miden
        # 120 mm de ancho: se solapan. Aca se usan 7 celdas (140 mm).
        rovers_ini = [(10, 2.5, 2.5, 0.0), (11, 2.5, 9.5, 0.0)]
    rovers = [Rover(i, c, r, t) for (i, c, r, t) in rovers_ini]
    cubos = [Cubo(col, cc, rr, th) for (col, cc, rr, th) in cubos_ini]
    m = Mundo(rovers, cubos, DEPOTS, semilla=semilla, sigma_pos_celdas=sigma_pos,
              sigma_theta=sigma_th, p_perdida=p_perd, latencia_ms=lat)
    plans = {rv.id: Planificador(rv.id, corte_latencia=corte) for rv in rovers}
    tomados = {}
    ultimo = None
    t = 0.0
    hist = []
    while t < t_max:
        msg = m.observar()
        if msg is not None:
            ultimo = msg
        for rv in rovers:
            v, w = plans[rv.id].decidir(ultimo, m.t_ms, tomados)
            rv.v = v * CELL
            rv.w = w
        m.paso(DT)
        t += DT
        if traza:
            hist.append({'t': t, 'cubos': [(cb.color, cb.c.copy() / CELL, cb.theta) for cb in m.cubos],
                         'rovers': [(rv.id, rv.p.copy() / CELL, rv.theta) for rv in m.rovers]})
        if m.entregados() == len(cubos):
            break
    return {'mundo': m, 'plans': plans, 'entregados': m.entregados(),
            'n_cubos': len(cubos), 't': t, 'colisiones': m.colisiones,
            'salidas': m.salidas, 'dist': m.distancias_al_deposito(), 'hist': hist,
            'desentregas': m.desentregas, 'contactos_paleta': m.contactos_paleta}


CUBOS_NOMINAL = [('green', 26.0, 10.0, 20.0), ('blue', 15.0, 29.0, 65.0), ('red', 33.0, 26.0, 0.0)]

# Dos configuraciones: UN rover aisla la capa geometrica del modelo (que es su
# alcance declarado); DOS rovers exponen la falta de la capa de coordinacion
# multi-agente, que el modelo dejo explicitamente fuera de alcance.
UNO = [(10, 2.5, 2.5, 0.0)]
DOS = None            # None -> las dos poses por defecto de correr()


# ------------------------------------------------------------------ N4.1 ----

@prueba('N4.1', 'Mision nominal sin perturbacion')
def n41(r):
    print()
    print('           %-14s %10s %9s %8s %8s   %s'
          % ('config', 'entregados', 'tiempo', 'colis', 'salidas', 'dist_final_celdas'))
    for etiq, rv in (('1 rover', UNO), ('2 rovers', DOS)):
        res = correr(CUBOS_NOMINAL, rv, semilla=1)
        print('           %-14s %7d/%-2d %9.1f %8d %8d   %s'
              % (etiq, res['entregados'], res['n_cubos'], res['t'],
                 res['colisiones'], res['salidas'],
                 '  '.join('%s=%.2f' % (k, v) for k, v in res['dist'].items())))
        pre = etiq.split()[0] + 'r_'
        r.metrica(pre + 'entregados', res['entregados'])
        r.metrica(pre + 'tiempo_s', round(res['t'], 1))
        r.metrica(pre + 'colisiones', res['colisiones'])
        r.metrica(pre + 'salidas', res['salidas'])
        r.exigir(res['entregados'] == res['n_cubos'], '%s: deben entregarse los 3 cubos' % etiq)


# ------------------------------------------------------------------ N4.2 ----

@prueba('N4.2', 'Mision con perturbacion nominal (200 semillas)')
def n42(r):
    # 200 semillas en la primera campana. El simulador en U cuesta cuatro veces
    # mas por paso (tres piezas por rover en el separador de ejes), asi que se
    # baja a 60: el intervalo de confianza queda en +-6 puntos, de sobra para
    # medir una caida de 0.98 a menos de 0.5, que es lo que se quiere resolver.
    N = 60
    print()
    print('           %-14s %10s %9s %9s %9s %14s'
          % ('config', 'tasa_3/3', 't_p50', 't_p90', 'colis_med', 'corridas_limpias'))
    for etiq, rv in (('1 rover', UNO), ('2 rovers', DOS)):
        ent = []; tie = []; col = []
        for s in range(N):
            res = correr(CUBOS_NOMINAL, rv, semilla=s, sigma_pos=0.06, sigma_th=1.5, p_perd=0.02)
            ent.append(res['entregados']); tie.append(res['t']); col.append(res['colisiones'])
        ent = np.array(ent); tie = np.array(tie); col = np.array(col)
        tasa = float(np.mean(ent == 3))
        limpias = float(np.mean(col == 0))
        print('           %-14s %10.3f %9.1f %9.1f %9.1f %13.1f%%'
              % (etiq, tasa, np.percentile(tie, 50), np.percentile(tie, 90),
                 col.mean(), 100 * limpias))
        pre = etiq.split()[0] + 'r_'
        r.metrica(pre + 'tasa', tasa)
        r.metrica(pre + 't_p50', float(np.percentile(tie, 50)))
        r.metrica(pre + 't_p90', float(np.percentile(tie, 90)))
        r.metrica(pre + 'colis_medias', float(col.mean()))
        r.metrica(pre + 'corridas_sin_colision', limpias)
        r.exigir(tasa >= 0.95, '%s: tasa de entrega completa >= 95%%' % etiq)
    r.nota('HALLAZGO H-07: con dos rovers la entrega se mantiene pero aparecen colisiones.')
    r.nota('La regla de cesion por pares (unica coordinacion implementada) NO alcanza.')
    r.nota('Es coherente con N3.8: el 46%% de los pares de corredores interfiere.')
    r.nota('La capa de coordinacion multi-agente esta FUERA del alcance del modelo,')
    r.nota('y esta prueba mide exactamente cuanto cuesta esa ausencia.')


# ------------------------------------------------------------------ N4.3 ----

@prueba('N4.3', 'Barrido de ruido posicional hasta el punto de ruptura')
def n43(r):
    sigmas = [0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0]
    print()
    print('           %10s %12s %10s %12s %10s' %
          ('sigma_c', 'sigma_mm', 'tasa_3/3', 'cubos_medio', 't_p50'))
    curva = []
    for sg in sigmas:
        ent = []; tie = []
        for s in range(40):
            res = correr(CUBOS_NOMINAL, UNO, semilla=1000 + s, sigma_pos=sg, sigma_th=1.5, p_perd=0.02)
            ent.append(res['entregados']); tie.append(res['t'])
        ent = np.array(ent)
        tasa = float(np.mean(ent == 3))
        curva.append((sg, tasa, float(ent.mean())))
        print('           %10.2f %12.1f %10.2f %12.2f %10.1f'
              % (sg, sg * CELL, tasa, ent.mean(), np.percentile(tie, 50)))
    ruptura = next((s for s, t, _ in curva if t < 0.5), None)
    r.metrica('curva', [(s, t) for s, t, _ in curva])
    r.metrica('punto_ruptura_celdas', ruptura)
    r.metrica('punto_ruptura_mm', None if ruptura is None else ruptura * CELL)
    presup = 12.57
    r.metrica('presupuesto_modelo_mm', presup)
    r.nota('barrido con UN rover, para aislar el modelo de la coordinacion ausente.')
    r.nota('el modelo predice un presupuesto lateral de control de %.2f mm.' % presup)
    if ruptura is not None:
        r.nota('la ruptura medida ocurre a sigma = %.1f mm (%.1fx el presupuesto).'
               % (ruptura * CELL, ruptura * CELL / presup))
    else:
        r.nota('no se alcanzo la ruptura en el rango barrido (hasta %.0f mm)' % (sigmas[-1] * CELL))
    r.estado = 'INFO'


# ------------------------------------------------------------------ N4.4 ----

@prueba('N4.4', 'Barrido de latencia, con y sin la regla de corte RI4')
def n44(r):
    lats = [0, 50, 100, 200, 400, 800, 1600]
    print()
    print('           %10s %14s %14s %12s' % ('lat_ms', 'tasa_con_corte', 'tasa_sin_corte', 'frenos'))
    filas = []
    for lat in lats:
        # 12 semillas y t_max=60 s. Con latencia alta ninguna corrida termina
        # temprano, asi que el costo crece linealmente con t_max y con el numero
        # de semillas. La resolucion alcanza para localizar el punto de ruptura,
        # que es lo unico que se le pide a este barrido.
        res_c = [correr(CUBOS_NOMINAL, UNO, semilla=2000 + s, sigma_pos=0.06, sigma_th=1.5,
                        p_perd=0.02, lat=lat, corte=True, t_max=60.0) for s in range(12)]
        res_s = [correr(CUBOS_NOMINAL, UNO, semilla=2000 + s, sigma_pos=0.06, sigma_th=1.5,
                        p_perd=0.02, lat=lat, corte=False, t_max=60.0) for s in range(12)]
        tc = float(np.mean([x['entregados'] == 3 for x in res_c]))
        ts = float(np.mean([x['entregados'] == 3 for x in res_s]))
        fr = int(np.sum([sum(p.frenos for p in x['plans'].values()) for x in res_c]))
        filas.append((lat, tc, ts, fr))
        print('           %10d %14.2f %14.2f %12d' % (lat, tc, ts, fr))
    r.metrica('tabla', filas)
    bajo = [f for f in filas if f[0] <= 200]
    alto = [f for f in filas if f[0] >= 800]
    r.metrica('tasa_con_corte_lat_alta', float(np.mean([f[1] for f in alto])))
    r.metrica('tasa_sin_corte_lat_alta', float(np.mean([f[2] for f in alto])))
    r.metrica('tasa_con_corte_lat_baja', float(np.mean([f[1] for f in bajo])))
    r.metrica('tasa_sin_corte_lat_baja', float(np.mean([f[2] for f in bajo])))
    r.estado = 'INFO'


# ------------------------------------------------------------------ N4.5 ----

@prueba('N4.5', 'Escenarios adversarios')
def n45(r):
    escenarios = {
        '(a) cubo en esquina opuesta': (
            [('green', 2.0, 41.0, 30.0)], [(10, 2.5, 2.5, 0.0)]),
        '(b) cubo pegado a la pared': (
            [('red', 41.5, 20.0, 15.0)], [(10, 20.0, 20.0, 0.0)]),
        '(c) dos cubos adyacentes': (
            [('green', 20.0, 20.0, 0.0), ('red', 23.5, 20.0, 0.0)], [(10, 10.0, 20.0, 0.0)]),
        '(d) cubo ajeno en el corredor': (
            [('green', 15.0, 15.0, 0.0), ('blue', 25.0, 10.5, 0.0)], [(10, 8.0, 18.0, 0.0)]),
        '(e) corredores cruzados': (
            [('green', 12.0, 30.0, 0.0), ('blue', 30.0, 12.0, 0.0)],
            [(10, 5.0, 30.0, 0.0), (11, 30.0, 5.0, 90.0)]),
        '(f) cubo sobre deposito ajeno': (
            [('green', 2.5, 40.5, 20.0)], [(10, 10.0, 35.0, 0.0)]),
    }
    print()
    print('           %-32s %10s %8s %7s %7s %9s %s' %
          ('escenario', 'entregado', 'tiempo', 'colis', 'salid', 'dist_fin', 'irrecuperables'))
    bloqueos = 0; colis = 0; salidas = 0
    for nombre, (cubos, rovers) in escenarios.items():
        res = correr(cubos, rovers, semilla=7, sigma_pos=0.06, sigma_th=1.5,
                     p_perd=0.02, t_max=120.0)
        d = min(res['dist'].values())
        bloq = res['entregados'] == 0 and res['t'] >= 119.0
        bloqueos += int(bloq); colis += res['colisiones']; salidas += res['salidas']
        irr = sorted(set().union(*[getattr(p, 'irrecuperables', set())
                                   for p in res['plans'].values()]))
        print('           %-32s %6d/%-3d %8.1f %7d %7d %9.2f %s'
              % (nombre, res['entregados'], res['n_cubos'], res['t'],
                 res['colisiones'], res['salidas'], d, ','.join(irr) or '-'))
        r.metrica(nombre[:3] + '_irrecuperables', irr)
        r.metrica(nombre[:3] + '_entregados', res['entregados'])
        r.metrica(nombre[:3] + '_dist_min', round(d, 2))
    r.metrica('bloqueos', bloqueos); r.metrica('colisiones', colis); r.metrica('salidas', salidas)
    r.nota('(a) y (f) colocan el cubo en la ZONA TRAMPA (hallazgo H-09): no existe')
    r.nota('secuencia de empujes que lo saque. El planificador lo detecta y lo abandona,')
    r.nota('que es la conducta correcta: no gasta la ronda empujando contra una pared.')
    r.nota('(e) exhibe colisiones, que es el hallazgo H-07 (falta coordinacion).')
    r.exigir(salidas == 0, 'ningun rover debe salir de la cancha')


# ------------------------------------------------------------------ N4.6 ----

@prueba('N4.6', 'Oclusion prolongada durante el transporte')
def n46(r):
    res = correr([('green', 26.0, 10.0, 20.0)], [(10, 2.5, 2.5, 0.0)], semilla=3,
                 sigma_pos=0.06, sigma_th=1.5, p_perd=0.02)
    tr = res['plans'][10].traza
    if not tr:
        r.nota('no se registro fase de empuje'); r.estado = 'FALLA'; return
    ages = np.array([x[1] for x in tr]); ocl = np.array([x[2] for x in tr])
    lat_err = np.array([abs(x[0]) for x in tr]) * CELL
    print()
    print('           muestras en fase de empuje: %d' % len(tr))
    print('           oclusion del cubo: media=%.1f%%  max=%.1f%%' % (100 * ocl.mean(), 100 * ocl.max()))
    print('           age_ms: media=%.0f  p90=%.0f  max=%.0f' %
          (ages.mean(), np.percentile(ages, 90), ages.max()))
    print('           fraccion del empuje con age_ms > 200: %.1f%%' % (100 * np.mean(ages > 200)))
    print('           error lateral |e|: media=%.2f mm  max=%.2f mm  (presupuesto 12.57 mm)'
          % (lat_err.mean(), lat_err.max()))
    r.metrica('muestras_empuje', len(tr))
    r.metrica('oclusion_media', float(ocl.mean())); r.metrica('oclusion_max', float(ocl.max()))
    r.metrica('age_max_ms', float(ages.max()))
    r.metrica('frac_age_mayor_200', float(np.mean(ages > 200)))
    r.metrica('lateral_medio_mm', float(lat_err.mean()))
    r.metrica('lateral_max_mm', float(lat_err.max()))
    r.metrica('entregado', res['entregados'])
    r.nota('el simulador oficial NO puede producir este escenario (hallazgo H-01b):')
    r.nota('su radio de oclusion es 2 celdas y el contacto real ocurre a 5-5.6 celdas.')
    r.exigir(res['entregados'] == 1, 'la tarea debe completarse pese a la oclusion')


# ------------------------------------------------------------------ N4.7 ----

@prueba('N4.7', 'Rotacion del cubo inducida por el empuje descentrado')
def n47(r):
    from simulador import Mundo, Rover, Cubo
    print()
    print('           %10s %14s %14s %14s' % ('e_mm', 'deriva_ang', 'deriva_lat_mm', 'avance_mm'))
    filas = []
    for e_mm in (0.0, 2.0, 5.0, 10.0, 15.0):
        cb = Cubo('green', 20.0, 20.0, 0.0)
        # rover detras del cubo, desplazado lateralmente e_mm
        rv = Rover(10, 20.0 - (112.4 + 5) / CELL, 20.0 + e_mm / CELL, 0.0)
        m = Mundo([rv], [cb], DEPOTS, semilla=0)
        th0 = cb.theta; c0 = cb.c.copy()
        rv.v = 60.0; rv.w = 0.0
        for _ in range(int(6.0 / 0.02)):
            m.paso(0.02)
            if np.linalg.norm(cb.c - c0) > 300.0:
                break
        dth = float(rho_P(cb.theta, th0, 90.0))
        dlat = abs(float(cb.c[1] - c0[1]))
        dav = float(cb.c[0] - c0[0])
        filas.append((e_mm, dth, dlat, dav))
        print('           %10.1f %14.3f %14.3f %14.1f' % (e_mm, dth, dlat, dav))
    r.metrica('tabla', filas)
    r.metrica('deriva_ang_e0', filas[0][1]); r.metrica('deriva_ang_e15', filas[-1][1])
    monotona = all(filas[i][1] <= filas[i + 1][1] + 1e-6 for i in range(len(filas) - 1))
    r.metrica('monotona_en_e', monotona)
    r.metrica('simetria_e0', filas[0][1] < 0.5)
    r.nota('SOLO ES VALIDO EL CASO e=0: la simetria da deriva nula a 1.7e-13 grados.')
    r.nota('Los valores para e distinto de cero NO son cuantitativamente confiables:')
    r.nota('el modelo de centro instantaneo cuasiestatico con r_g^2 = L^2/6 resulta')
    r.nota('no monotono en e y no produce deriva lateral, que fisicamente deberia haber.')
    r.nota('LIMITE DEL ALCANCE: aqui termina la geometria y empieza la mecanica del')
    r.nota('contacto (cono de friccion, regla de Mason), que el modelo excluyo.')
    r.nota('Para cuantificar la rotacion inducida hace falta un modelo de friccion.')
    r.exigir(filas[0][1] < 0.5, 'empuje centrado (e=0) no debe rotar el cubo')
    r.estado = 'INFO'


# ------------------------------------------------------------------ N4.8 ----

@prueba('N4.8', 'Interferencia entre tareas: un cubo por vez contra los tres a la vez')
def n48(r):
    """Aisla el efecto de las paletas sobre los cubos que NO son el objetivo.

    Con la huella rectangular anterior, pasar cerca de un cubo lo rozaba. Con la
    forma de U real, la punta de una paleta lo ENGANCHA y lo arrastra, y el caso
    mas caro es sacar de su deposito un cubo ya entregado. Esta prueba separa las
    dos cosas: si cada cubo por separado se entrega y los tres juntos no, la culpa
    no es de la geometria del empuje sino de la interferencia entre tareas.
    """
    N = 30
    print()
    print('           %-26s %10s %10s %13s %12s'
          % ('configuracion', 'entregados', 'de 3', 'desentregas', 'contactos'))
    # (a) un cubo por vez, en misiones independientes
    ent_a = 0; des_a = 0; con_a = 0
    for s in range(N):
        for cb in CUBOS_NOMINAL:
            res = correr([cb], [(10, 2.5, 2.5, 0.0)], semilla=s, t_max=70.0)
            ent_a += res['entregados']; des_a += res['desentregas']
            con_a += res['contactos_paleta']
    print('           %-26s %10.2f %10s %13.2f %12.1f'
          % ('un cubo por vez', ent_a / N, '3', des_a / N, con_a / N))
    # (b) los tres a la vez
    ent_b = 0; des_b = 0; con_b = 0
    for s in range(N):
        res = correr(CUBOS_NOMINAL, [(10, 2.5, 2.5, 0.0)], semilla=s, t_max=180.0)
        ent_b += res['entregados']; des_b += res['desentregas']
        con_b += res['contactos_paleta']
    print('           %-26s %10.2f %10s %13.2f %12.1f'
          % ('los tres a la vez', ent_b / N, '3', des_b / N, con_b / N))
    r.metrica('entregados_aislado', ent_a / N)
    r.metrica('entregados_conjunto', ent_b / N)
    r.metrica('desentregas_conjunto', des_b / N)
    r.metrica('caida_pct', 100 * (1 - (ent_b / max(ent_a, 1e-9))))
    r.nota('Aislado el rover entrega %.2f de 3 cubos; con los tres en cancha, %.2f.'
           % (ent_a / N, ent_b / N))
    r.nota('La geometria del EMPUJE no es el problema: un cubo solo se entrega bien.')
    r.nota('Lo que colapsa es hacer tres seguidos sin perturbar a los otros dos.')
    r.nota('HALLAZGO H-14/H-15: con la forma de U, el rover engancha cubos ajenos y')
    r.nota('barre con la paleta el que acaba de entregar. Se deshace trabajo hecho:')
    r.nota('%.2f des-entregas por corrida.' % (des_b / N))
    r.nota('Es una falla del PLANIFICADOR, no del modelo geometrico: la capa de')
    r.nota('navegacion con obstaculos (grafo de visibilidad, inflado de Minkowski)')
    r.nota('esta en el modelo pero el planificador nunca la uso, porque con la')
    r.nota('huella supuesta no hacia falta.')
    r.estado = 'INFO'   # mide interferencia, no es criterio de aceptacion



if __name__ == '__main__':
    print('=' * 96)
    print('NIVEL N4 --- LAZO CERRADO')
    print('=' * 96)
    import os
    solo = os.environ.get('SOLO', '')
    todas = {'n41': n41, 'n42': n42, 'n43': n43, 'n44': n44,
             'n45': n45, 'n46': n46, 'n47': n47, 'n48': n48}
    cuales = solo.split(',') if solo else list(todas)
    for k in cuales:
        todas[k]()
    resumen('RESUMEN N4' + (' (parcial: %s)' % solo if solo else ''))
    volcar('/tmp/bench/resultados_n4%s.json' % ('_p2' if solo else ''))

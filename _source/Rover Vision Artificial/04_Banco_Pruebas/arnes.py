"""Arnés de pruebas: registro, criterios y reporte."""
from __future__ import annotations
import json, sys, time, traceback

RES = []

class Resultado:
    def __init__(self, ident, titulo):
        self.id = ident; self.titulo = titulo
        self.estado = 'PENDIENTE'; self.metricas = {}; self.notas = []
        self.t0 = time.time()

    def metrica(self, k, v):
        if hasattr(v, 'item'):
            v = v.item()
        self.metricas[k] = v
        return v

    def nota(self, s):
        self.notas.append(s)

    def exigir(self, cond, msg=''):
        if not cond:
            self.estado = 'FALLA'
            if msg: self.notas.append('FALLA: ' + msg)
        return bool(cond)

    def cerrar(self, estado=None):
        if estado: self.estado = estado
        elif self.estado == 'PENDIENTE': self.estado = 'PASA'
        self.dt = time.time() - self.t0
        RES.append(self)
        marca = {'PASA':'  OK  ','FALLA':' FALLA','INFO':' INFO ','ESPERADO':'ESPER.'}.get(self.estado,'  ?   ')
        det = '  '.join('%s=%s' % (k, fmt(v)) for k, v in list(self.metricas.items())[:4])
        print('[%s] %-7s %-52s %s' % (marca, self.id, self.titulo[:52], det))
        for n in self.notas:
            print('           · ' + n)
        return self


def fmt(v):
    if isinstance(v, float):
        if v != 0 and (abs(v) < 1e-4 or abs(v) >= 1e6):
            return '%.2e' % v
        return '%.6g' % v
    return str(v)


def prueba(ident, titulo):
    def deco(fn):
        def run():
            r = Resultado(ident, titulo)
            try:
                fn(r)
            except Exception:
                r.estado = 'FALLA'
                r.nota('excepción: ' + traceback.format_exc().strip().splitlines()[-1])
            return r.cerrar()
        run.__name__ = fn.__name__
        return run
    return deco


def resumen(titulo):
    print()
    print('=' * 96)
    n = {}
    for r in RES:
        n[r.estado] = n.get(r.estado, 0) + 1
    print('%s  ->  ' % titulo + '  '.join('%s: %d' % (k, v) for k, v in sorted(n.items())))
    print('=' * 96)
    fallas = [r for r in RES if r.estado == 'FALLA']
    if fallas:
        print('PRUEBAS QUE FALLARON:')
        for r in fallas:
            print('  %-7s %s' % (r.id, r.titulo))
            for x in r.notas: print('        ' + x)
    return len(fallas)


def volcar(ruta):
    with open(ruta, 'w') as f:
        json.dump([{'id': r.id, 'titulo': r.titulo, 'estado': r.estado,
                    'metricas': r.metricas, 'notas': r.notas,
                    'segundos': round(r.dt, 3)} for r in RES], f, indent=1, ensure_ascii=False)

# Banco de pruebas

Verificación del modelo matemático en cinco niveles, sin hardware. Determinista:
mismas semillas, mismos resultados.

## Correr

```bash
pip install numpy scipy opencv-python ezdxf cairosvg
python3 n0_n1.py     # N0 consistencia interna + N1 estimadores      (~2 min)
python3 n3.py        # N3 capa de planificación                      (~2 min)
python3 n2.py        # N2 visión extremo a extremo                   (~5 min)
python3 n4.py        # N4 lazo cerrado                              (~40 min)
```

`n2.py` necesita el repositorio oficial descomprimido; ajustar la constante `REPO`
en la cabecera del archivo.

`n4.py` acepta una variable de entorno para correr solo algunas pruebas:

```bash
SOLO=n41,n42 python3 n4.py
```

**Los procesos en segundo plano no sobreviven bien**: conviene correr en primer plano
y trocear N4 con `SOLO=`.

## Módulos

| Archivo | Qué es |
|---|---|
| `modelo.py` | Implementación de referencia del modelo: marco, cuadrado, estimadores B1–B4, soporte, paralaje, planificación |
| `arnes.py` | Registro de pruebas, criterios y reporte |
| `n0_n1.py` | N0 (invariantes algebraicas) y N1 (estimadores contra verdad analítica) |
| `n2.py` | N2 (visión extremo a extremo contra el generador sintético oficial) |
| `n3.py` | N3 (capa de planificación, barridos globales) |
| `simulador.py` | Simulador de dinámica: contacto entre cuerpos por eje separador, oclusión por siluetas, telemetría v1 |
| `planificador.py` | Planificador geométrico que consume telemetría v1 |
| `n4.py` | N4 (lazo cerrado: misión, ruido, latencia, adversarios) |

## Convención de marco

`u(θ) = (cos θ, −sin θ)` y `R(θ) = [[cos, sin], [−sin, cos]]`, porque `row` crece
hacia abajo y θ se mide antihorario visual. Todas las identidades están en el
apéndice A del documento del modelo. **Es la fuente de errores de signo número uno.**

## Resultados

`resultados/*.json` — una entrada por prueba con estado, métricas y notas.
`salidas/*.txt` — la salida completa de cada corrida.

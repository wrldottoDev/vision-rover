# Vision Rover Challenge — Investigación y planeación

Modelo matemático, protocolo de verificación y banco de pruebas para el reto.
Todo lo que hay acá se puede regenerar: los documentos desde su `.tex` y los
resultados corriendo el banco.

---

## Árbol

| Carpeta | Qué hay | Para qué |
|---|---|---|
| `01_Modelo_Matematico/` | Documento del modelo (PDF + fuente `.tex`) | La matemática: detección del cuadrado, estimación de pose, planificación geométrica |
| `02_Protocolo_Pruebas/` | Especificación de las pruebas (PDF + fuente) | Qué se prueba, cómo y con qué criterio de aceptación |
| `03_Informe_Resultados/` | Resultados y hallazgos (PDF + fuente) | Qué salió, los quince hallazgos, qué falta |
| `04_Banco_Pruebas/` | Código ejecutable + resultados en JSON | Reejecutar la verificación completa |
| `05_Mediciones_CAD/` | Script de medición del DXF de fabricación | Reproducir las dimensiones del rover |

---

## Por dónde empezar

1. **`03_Informe_Resultados/`** — el resumen ejecutivo y los quince hallazgos. Si solo
   vas a leer una cosa, que sea esta.
2. **`01_Modelo_Matematico/`** — las secciones 6.3 (embudo de tolerancia), 6.5 (las
   paletas son un embudo) y 6.6 (el borde es un precipicio) son las que gobiernan el
   diseño del planificador.
3. **`04_Banco_Pruebas/`** — para reejecutar todo.

---

## Los números que hay que tener a mano

| Magnitud | Valor | Origen |
|---|---|---|
| Cancha efectiva | 43 × 43 celdas de 20 mm = 860 × 860 mm | `config_vision.json` |
| Tablero físico | 50 × 50 cuadros = 1000 × 1000 mm, **sin paredes** | archivos de fabricación |
| Cubo | 60 mm de arista | `cubos.dxf` |
| Anchura proyectada del cubo | entre 60 y 84,85 mm según su giro | derivado |
| **Ancho útil del canal `W_u`** | **93,50 mm** (medido: 94,05 ± 0,54) | DXF + 3 mediciones |
| **Alcance de las paletas `λ`** | **55 mm** | medido sobre el rover armado |
| **Envolvente real `R_rov`** | **113,5 mm** (no 68,4) | derivado — hallazgo H-12 |
| **Anillo de maniobra** | **156 mm** | derivado — para girar sin barrer el cubo |
| **Huella exterior `W_ext`** | **99,50 mm** | `PIEZAS SEPARADAS.dxf` |
| **Largo del chasis `Λ`** | **94,00 mm** | `PIEZAS SEPARADAS.dxf` |
| Paneles laterales | 52,50 × 106,00 mm cada uno, acrílico de 3 mm | `PIEZAS SEPARADAS.dxf` |
| Tolerancia lateral `e_max` | **4,32 mm** (peor giro) a 16,75 mm (cubo alineado) | derivado, **exacto** (canal paralelo) |
| Presupuesto de control | **−0,68 mm** (peor giro) a 11,75 mm (alineado) | derivado — negativo en el peor caso |
| Ventana angular admisible | \|Δ\| ≤ 21° de la normal a una cara | derivado (H-11) |
| Precisión de rumbo exigida | 5,21° con el empuje alineado | derivado |
| `ρ_puesta` | 129,43 mm | derivado |
| Telemetría | TCP/NDJSON, puerto 2026, 20 Hz | `CONTRATO.md` |

---

## Medición sobre el rover físico: qué confirmó y qué rompió

Se midió el rover armado (versión preliminar, regla escolar sin calibrar, ±1 mm).

**Confirmó H-11, y de forma robusta al error.** Tres estimaciones independientes de `W_u`
—94,35 / 93,00 / 95,00— promedian **94,05 ± 0,54 mm**, con el valor del DXF (93,50) a
1,0σ. Y la conclusión no depende de cuál se tome: el presupuesto de control a 45° queda
entre **−0,94 y +0,13 mm** en todo el rango. Es cero dentro del error, mídase como se
mida.

**Rompió otra cosa: el largo.** Las paletas se prolongan **55 mm** por delante de la placa
frontal —dato que no estaba en ningún archivo— así que el rover mide 150 mm de largo, no
94. El envolvente pasa de 68,4 a **113,5 mm: un 66 % más**, y hacia el lado peligroso (el
planificador creía pasar por huecos por los que no pasa). Eso es **H-12**, y de ahí
salieron tres defectos más del planificador:

- **H-14** — los otros cubos no eran obstáculos de navegación. Con paletas, el rover los
  *engancha* y arrastra media cancha.
- **H-15** — no había retirada antes de girar: el rover barría con la paleta el cubo que
  acababa de dejar en el depósito.
- **H-13** — el planificador no tenía garantía de vivacidad: varias ramas devuelven «no te
  muevas» y el rover quedaba congelado el resto de la ronda.

**Y confirmó algo que el modelo había supuesto sin evidencia:** las paletas llegan al piso,
así que el polígono de apoyo son las ruedas más las puntas de los brazos. Por eso una
paleta en voladizo sobre el vacío no vuelca al rover.

---

## El resultado que separa la geometría del planificador (N4.8)

| Configuración | entregados de 3 | des-entregas |
|---|---|---|
| Un cubo por vez, misiones independientes | **3,00** | 0,00 |
| Los tres a la vez | **1,00** | **4,00** |

**La geometría del empuje funciona.** Un cubo solo se entrega siempre. Lo que colapsa es
hacer tres seguidos sin perturbar a los otros dos: el rover no falla en entregar,
**deshace entregas ya conseguidas**.

Es una falla del planificador, no del modelo: la capa de navegación con obstáculos
—inflado de Minkowski, grafo de visibilidad, corredores disjuntos— está desarrollada en el
modelo matemático, pero el planificador nunca la usó porque con la huella supuesta no
hacía falta.

---

## El hallazgo que cambia el alcance del proyecto (H-11)

El archivo de piezas separadas cerró la última incógnita, y el resultado es incómodo:

> Con `W_u = 93,50 mm`, la tolerancia lateral en el peor caso es **4,32 mm** y la
> incertidumbre de la visión es **5 mm**. El presupuesto de control es **−0,68 mm**:
> negativo. Ningún controlador, por perfecto que sea, garantiza la captura en la banda
> Δ ∈ (34,76°; 55,24°), que es el **22,8 % de las orientaciones**.

**Conocer el ángulo del cubo dejó de ser una optimización: es condición de
funcionamiento.** Y el contrato v1 no lo publica, aunque el detector oficial lo calcule
internamente, así que la estimación corre del lado del cliente — con los estimadores
cerrados del modelo (una arista basta, o un vértice más el centro).

Regla operativa: **apuntar el empuje a menos de 21° de la normal a una cara del cubo.**
Como las caras se repiten cada 90°, hay cuatro direcciones admisibles por cubo.

---

## Medición sobre el rover físico

`05_Mediciones_CAD/Hoja_de_Medicion_Rover.pdf` es la hoja para llenar con el rover en la
mano y una regla. Está ordenada por importancia y cada medición trae el valor predicho y
qué significa si no coincide.

**La medición M1 es la que decide todo:** con un cubo girado a 45° dentro del canal,
empujado contra una paleta, el hueco del otro lado debe medir **8,65 mm**. Si es menor
que **10 mm**, H-11 queda confirmado. Es un umbral de pasa/no-pasa: no hace falta leer
el número con precisión.

Con los números anotados:

```bash
python3 05_Mediciones_CAD/recalcular.py --hueco45 8.7 --hueco0 33.5
```

El script rehace todas las magnitudes derivadas y dice explícitamente qué hallazgos del
informe se confirman, se atenúan o se caen.

---

## Lo que falta hacer

1. **Rediseñar el planificador para un cuerpo largo.** Las tres reglas están implementadas
   y verificadas en el banco: anillo de maniobra de 156 mm, retirada de 55 mm antes de
   girar, y los demás cubos como obstáculos con 92 mm de despeje. Falta llevarlas al
   código real.
2. **Rehacer la medición con calibre** cuando esté el rover definitivo. Tres números:
   canal 93,50 · huella 99,50 · alcance de paletas 55.
3. **Implementar la estimación del ángulo del cubo** (H-11). El contrato v1 no lo publica.

---

## Nota histórica sobre el robot armado

Las paletas son **paralelas** entre sí (una U de esquinas rectas, como los rieles de una
vía), así que el ángulo de montaje salió del problema; y el despiece separado resolvió
`W_u` por construcción. Queda:

1. **Una verificación con calibre**, no una medición: confirmar que el robot armado mide
   **99,50 mm punta a punta**. Si coincide, `W_u = 93,50` queda confirmado y con él todo
   H-11. La fórmula «ancho total − 6 mm» resultó exacta.
2. `λ` — alcance de las paletas por delante de la placa frontal. No afecta a `e_max`;
   fija la distancia de contacto.

---

## Nota sobre archivos sueltos en la raíz

Las versiones anteriores de los PDF quedaron sueltas en la raíz de esta carpeta junto
con `Vision-Rover-Challenge-main.zip`. **Las de este árbol son las buenas**; las
sueltas se pueden borrar. El `.zip` del repositorio oficial conviene dejarlo donde está.

# Dimensiones del rover, medidas del CAD de fabricación

Fuente: `archivos_fabricacion/CENFOBOT_Rover_Rev1.dxf` del repositorio oficial
(cubierta de acrílico negro de 3 mm, lista para corte).

Reproducir con:

```bash
python3 medir_dxf.py <ruta-a>/archivos_fabricacion
```

## Verificación de escala

El DXF del rover **no declara unidades** (`$INSUNITS = 0`). La escala se confirma por
tres vías independientes, y el script las comprueba antes de reportar nada:

| Vía | Comprobación | Resultado |
|---|---|---|
| (a) | Ranuras de ensamble = espesor del acrílico | **3,00** unidades |
| (b) | Agujeros del ultrasónico HC-SR04 | **16,17** y **16,12** de diámetro, separados **24,00** |
| (c) | `cubos.dxf`, que sí declara mm | **180,30** de lado = 3 × 60 mm |

Las tres fijan **1 unidad = 1 mm**.

## Piezas

| Pieza | Dimensiones (mm) | Función |
|---|---|---|
| Chasis | **98,66 × 93,20** | plataforma principal; fija `W` y `Λ` |
| Placa frontal | 98,51 × 41,76 | aloja el ultrasónico; cara de empuje |
| Paleta (×2, espejadas) | 51,65 × 105,15 | brazos de captura |
| Placa superior | 71,22 × 50,67 | tapa |
| Placa trasera | 65,14 × 21,16 | cierre |

## Discrepancia con la configuración del sistema de visión

`vision/config_vision.json`, bloque `sintetico.cuerpo_rover`, declara
**120 × 140 × 90 mm**. Contra el archivo de fabricación:

- ancho sobreestimado en 21,34 mm (**21,6 %**)
- largo sobreestimado en 46,80 mm (**50,2 %**)

No es un error del sistema de visión —ese bloque solo dibuja el cuerpo del rover en las
imágenes sintéticas y no afecta a la detección— pero es la única fuente de dimensiones
del rover en todo el repositorio, y quien la tome como tal construye el planificador
sobre un rover que no existe.

## Lo que el DXF no puede dar

El montaje de las paletas. Es un despiece plano: contiene la forma de cada brazo pero no
cómo se atornilla.

Las paletas son **paralelas entre sí** —una U de esquinas rectas, como los rieles de una
vía—, dato que aportó el autor y que el DXF no permite deducir. Queda por medir:

- **`W_u`**, la separación entre caras interiores. Acotada a [92,66; 98,66] mm según
  monten por dentro o por fuera de las paredes del chasis. Medición cómoda: ancho
  exterior entre caras externas, menos 6,00 mm de los dos espesores.
- `λ`, el alcance por delante de la placa frontal.

### Advertencia sobre las fotografías

La vista frontal del robot armado **no sirve para deducir geometría**. Dos intentos
fallaron: escalarla dio estimaciones que diferían un 52 %, y leer la orientación de las
paletas dio un embudo en V donde hay un canal paralelo. Ambos son artefactos de
perspectiva: las paletas se proyectan hacia adelante y hacia abajo, sus extremos quedan
más cerca de la cámara, y la perspectiva agranda lo cercano. Sin referencias métricas en
el plano de interés, una foto en perspectiva no es una fuente geométrica.

---

# Adenda — el despiece separado resuelve `W_u`

Fuente: `CENFOBOT PIEZAS SEPARADAS.dxf`, las piezas de fabricación desplegadas una
junto a otra. **Declara milímetros** (`$INSUNITS = 4`), así que no requiere las tres
vías de verificación de escala del archivo anterior.

Reproducir con:

```bash
python3 medir_dxf.py --piezas "CENFOBOT PIEZAS SEPARADAS.dxf"
```

## Piezas

| Pieza | Dimensiones (mm) |
|---|---|
| Chasis (cuerpo) | **93,50 × 94,00** |
| Chasis (con lengüetas) | **99,50 × 94,00** |
| Panel lateral (×2) | 52,50 × 106,00 |
| Placa frontal | 99,38 × 42,63 (tramos rectos 99,00) |
| Tapa superior | 72,09 × 51,53 |
| Placa trasera | 66,00 × 22,00 |
| Soporte de motor (×2) | 47,90 × 28,60 |

## Cómo queda determinado `W_u`

Los bordes verticales del chasis están en `224,85 · 227,85 · … · 321,35 · 324,35`.
Las lengüetas sobresalen **exactamente 3,00 mm por lado**, que es el espesor del
acrílico. Una lengüeta de espesor igual al del material es una **junta pasante**:
atraviesa la ranura del panel lateral y queda enrasada con su cara exterior. Por lo
tanto la cara **interior** del panel coincide con el borde del cuerpo del chasis:

```
W_u   = 93,50 mm        (separación entre caras interiores → canal de captura)
W_ext = 93,50 + 2×3,00
      = 99,50 mm        (huella exterior → colisiones, R_rov, inflado)
```

Esto confirma exactamente la fórmula propuesta: **ancho total − 6 mm**.

> **Dos anchos distintos, y confundirlos es un error.** `W_ext` gobierna colisiones e
> inflado de obstáculos; `W_u` gobierna `e_max` y todo el presupuesto de error lateral.
> Usar `W_ext` donde va `W_u` infla la tolerancia un 69 % y hace desaparecer el
> hallazgo H-11.

## Consecuencias

| Magnitud | Valor |
|---|---|
| `R_rov` (con `W_ext`) | 68,44 mm = 3,422 celdas |
| `ρ_contacto` (máx.) | 89,43 mm |
| `ρ_puesta` (s = 40) | 129,43 mm |
| `e_max` | **[4,32; 16,75] mm** |
| Presupuesto de control a 45° | **−0,68 mm** (negativo) |
| Ventana angular (margen 3 mm) | \|Δ\| ≤ 20,97° |
| Banda de presupuesto negativo | Δ ∈ (34,76°; 55,24°) — 22,8 % de las orientaciones |

**Hallazgo H-11.** Con `W_u = 93,50` la sola incertidumbre de la visión (5 mm) supera
la tolerancia lateral (4,32 mm) en el peor caso. Conocer el ángulo del cubo dejó de ser
una mejora: es **obligatorio**. Regla operativa: apuntar el empuje a menos de ~21° de la
normal a una cara.

## Lo único que queda

Verificar con calibre que el robot armado mide **99,50 mm punta a punta**. Es una
comprobación, no una incógnita. También `λ`, el alcance de las paletas por delante de la
placa frontal, que no afecta a `e_max`.

---

# Adenda 2 — Medición sobre el rover armado

Versión preliminar del robot, regla escolar sin calibrar. Todas las lecturas se tratan
como ±1 mm y ninguna conclusión se apoya en un solo número.

## Cómo se midió `W_u` con regla

El umbral que decide es 94,85 mm. Separar 93,5 de 94,9 sobre un tramo de 94 mm excede la
resolución del instrumento; la misma diferencia leída sobre un **hueco de 9 mm**, no. Por
eso las mediciones críticas se plantearon como huecos:

```
h(Δ) = W_u − w_S(Δ) = 2·e_max(Δ)
h(0°) − h(45°) = L(√2−1) = 24,85 mm     ← no depende de W_u: es el control
```

| # | Magnitud | predicho | medido | lectura |
|---|---|---|---|---|
| M1 | Hueco con cubo a 45° | 8,65 | 9–10 | `W_u` = 94,35 |
| M2 | Hueco con cubo alineado | 33,50 | 33 | `W_u` = 93,00 |
| M9 | Canal, medido directo | 93,50 | 95 | `W_u` = 95,00 |
| M6 | **Alcance de paletas `λ`** | — | **55** | nuevo |
| M7 | Ancho máximo real | 99,50 | 100 | concuerda |
| M8 | Largo máximo real | 149 | 150 | concuerda |
| M10 | Paletas: piso a borde superior | — | 0 a 14 | nuevo |
| M12 | Aristas del cubo | vivas | vivas | concuerda |
| M13 | Rueda: diámetro / trocha | — | 33 / 89 | nuevo |

**Control interno:** M2 − M1 = 23,5 mm contra los 24,85 que exige la identidad. Desvío de
1,35 mm, compatible con dos lecturas de regla. Las mediciones son coherentes.

## Resultado

**`W_u` = 94,05 ± 0,54 mm** (promedio ponderado de las tres vías). El valor del DXF,
93,50, cae a 1,0σ: **la medición no lo refuta**.

Y la conclusión es robusta al error:

```
presupuesto de control a 45° ∈ [−0,94 ; +0,13] mm   para todo W_u en ±2σ
```

**Es cero dentro del error de medición, cualquiera sea el valor que se tome.** H-11 se
sostiene sin necesidad de un instrumento mejor.

## Lo que la medición refutó: H-12

`λ = 55 mm` no estaba en ningún archivo. Con ese dato:

```
huella real            100 × 150 mm   (asimétrica: 47 atrás, 102 adelante)
R_rov real             113,49 mm      contra los 68,44 que usaba el modelo  → +66 %
anillo de maniobra     155,9 mm       para girar detrás del cubo sin barrerlo
zona trampa            6,46 %         contra 5,03 %  (banda 4,55 c contra 2,97)
ρ_contacto             89,43 mm       SIN CAMBIO (el cubo entra hasta la placa)
```

Confirmado por otra vía: chasis 94 + paletas 55 = 149, contra 150 medidos.

## Lo que confirmó y el modelo no sabía

- Las paletas llegan al piso: el polígono de apoyo son las ruedas **más** las puntas de
  los brazos, exactamente como se postuló sin evidencia. Una paleta en voladizo sobre el
  vacío no vuelca al rover.
- Las paletas van de 0 a 14 mm de altura: el empuje actúa a ~7 mm, muy por debajo de la
  mitad del cubo (30 mm). No hay riesgo de vuelco; la hipótesis H6 queda respaldada.
- Trocha 89 mm < canal ~94: las ruedas no sobresalen.
- Odometría: rueda de 33 mm → 1 vuelta = 103,67 mm. Trocha 89 mm → giro de 90° en el
  lugar = 69,90 mm por rueda.

## Reproducir

```bash
python3 recalcular.py --hueco45 9.5 --hueco0 33 --ancho-max 100 --lam 55
```

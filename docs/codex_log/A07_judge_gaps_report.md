Implementado.

Cambios:
- Añadido evento edge-triggered `rotation_with_cube` cuando `|omega| > 0.2 rad/s` con cubo dentro del canal.
- Añadido metricado `rotations_with_cube` en `RunResult` y agregación Monte Carlo.
- Rotaciones con cubo son diagnósticas; no causan fallo.
- Crédito de transporte ahora requiere contacto con placa frontal, rover asignado y velocidad real hacia delante `> 5 mm/s`.
- Rechazo de contacto usa velocidad real de ruedas, no comandos.
- Añadidos tests A07 y actualizada la regresión de reversa para contemplar inercia real.

Tests:

- Antes: 279 passed, 7 failed, 1 skipped.
- Después: 282 passed, 7 failed, 1 skipped.
- Tests de simulación: 69 passed.
- Rendimiento: 73.5x real-time, mínimo requerido 30x.

Fallos restantes: los mismos 7 preexistentes en `controllers`/`controllers_astra` (umbrales de cubo perdido, dinámica de retirada y compensación lateral); no pertenecen a los módulos modificados.

LEAD CHANGES REQUESTED: Ninguno.

Riesgo residual: la detección de canal usa la geometría completa del cubo y tolerancia fija de 5 mm junto a la placa, según la especificación.
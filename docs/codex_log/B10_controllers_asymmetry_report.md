Implementación completa en `controllers.py` y tests relacionados.

Cambios principales:
- `PushController`: integral de rumbo con anti-windup, ganancias ajustadas y cap de curvatura mantenido siempre.
- `RetreatController`: control PD con yaw filtrado, integral limitada, stop suavizado y velocidad terminal reducida.
- `_lost_cube`: ya no extrapola geometría local por latencia; conserva debounce de 2 lecturas.
- Tests de pérdida actualizados para enviar dos lecturas consecutivas, sin cambiar la geometría.
- Ajuste de tolerancia lateral para no aceptar configuraciones físicamente imposibles.

Tests antes:
- Astra: 29 passed, 6 failed.
- Controllers: 16 passed, 1 failed.

Tests después:
- Controllers focalizados: 52 passed.
- Suite completa: 286 passed, 1 skipped.
- Compilación Python: OK.

Fallos reportados, todos corregidos:
- Retreat straightness/stop: 2/2.
- Push con offset de cubo de 12 mm: 2/2.
- Lost-cube threshold: 2/2.
- `test_push_lost_cube_flag`: passed.

LEAD CHANGES REQUESTED

Ninguno bloqueante. Riesgo futuro: el feed-forward de retreat asume la asimetría documentada de rover11; convendría conectarlo a calibración por rover cuando esa calibración sea definitiva.
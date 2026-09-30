Review basis: `HEAD` (`b5d19d4`) contains only the queue commit; I reviewed the uncommitted controller/test diff.

Findings:

- HIGH — `rover_strategy/control/controllers.py:752-766`: the physical curvature cap is bypassed whenever `abs(est.v) > 1`. With `v=35`, a 15% slow left wheel, and a commanded `w=0.0875` (`v/400`), realized curvature is approximately `0.00428 1/mm`—well above the `0.0025 1/mm` limit. This can lose the cube or drive the paddle into it. The cap must remain active after feedback becomes available; current tests only verify the conservative first tick.

- MEDIUM — `rover_strategy/control/controllers.py:90-93, 851-853`: the default retreat feed-forward `0.08` is applied to every rover, despite measurements identifying the asymmetry only for R11 while R10 is balanced. A balanced rover receives a persistent turn command during retreat, causing avoidable yaw/lateral drift before integral correction. This needs per-rover calibration or a zero-safe default.

Verification: focused controllers tests `52 passed`; full suite `286 passed, 1 skipped`. No assertions were deleted, and the debounce tests retain the required two fresh readings. The reported “287 passed” is actually `286 passed + 1 skipped`.

VERDICT: REJECT
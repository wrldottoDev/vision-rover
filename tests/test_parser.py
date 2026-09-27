import json
import math
import pytest
from rover_strategy.vision.parser import parse_message, TelemetryError

EXAMPLE = {
    "v": 1, "seq": 4137, "ts_ms": 1785012345678, "phase": "RUNNING",
    "grid": {"cols": 43, "rows": 43, "cell_mm": 20.0},
    "rovers": [{"id": 10, "col": 4.302, "row": 3.705, "theta": 46.20, "age_ms": 0},
               {"id": 11, "col": 15.265, "row": 28.661, "theta": 40.22, "age_ms": 0}],
    "cubes": [{"color": "green", "col": 25.968, "row": 9.999, "age_ms": 0},
              {"color": "blue", "col": 15.000, "row": 29.000, "age_ms": 425},
              {"color": "red", "col": 33.071, "row": 25.983, "age_ms": 0}],
    "obstacles": [], "start": {"col": 2.5, "row": 2.5},
    "depots": [{"color": "green", "col": 40.5, "row": 2.5}, {"color": "blue", "col": 2.5, "row": 40.5},
               {"color": "red", "col": 40.5, "row": 40.5}],
}


def test_parse_contract_example():
    f = parse_message(json.dumps(EXAMPLE), t_received=0.0)
    assert set(f.rovers) == {10, 11} and set(f.cubes) == {"red", "green", "blue"}
    r = f.rovers[10].pose
    assert abs(r.x - 4.302 * 20) < 1e-9 and abs(r.y - (43 - 3.705) * 20) < 1e-9
    assert abs(r.theta - math.radians(46.2)) < 1e-9
    assert abs(f.cubes["blue"].age_s - 0.425) < 1e-9
    assert abs(f.cubes["blue"].t_capture - (1785012345.678 - 0.425)) < 1e-6
    g = f.depots["green"]            # top-right on screen = high x, high y internally
    assert (g.cx, g.cy) == (810.0, 810.0)
    assert f.depots["blue"].cx == 50.0 and f.depots["blue"].cy == 50.0


def test_rejects_unknown_version_and_skips_bad_entries():
    bad = dict(EXAMPLE, v=2)
    with pytest.raises(TelemetryError):
        parse_message(bad, 0.0)
    partial = dict(EXAMPLE, rovers=[{"id": 10, "col": "x", "row": 1, "theta": 0, "age_ms": 0},
                                    EXAMPLE["rovers"][1]])
    f = parse_message(partial, 0.0)
    assert set(f.rovers) == {11}

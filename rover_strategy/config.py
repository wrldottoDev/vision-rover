"""Single source of truth for every physical constant and tuning knob.

Canonical internal units: millimetres, radians, seconds.  Frame: x = +col, y = UP
(y = (rows - row) * cell_mm), theta CCW from +x.  See docs/geometry_model.md.

Every value carries its provenance.  "MEASURED" = physical measurement or DXF,
"ASSUMED" = defensible default that must be confirmed on hardware / with organisers,
"TUNED" = chosen by experiment in simulation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class BoardConfig:
    # MEASURED (config_simulador.json / config_vision.json). Overwritten from telemetry `grid`.
    cols: int = 43
    rows: int = 43
    cell_mm: float = 20.0
    # ASSUMED: strict reading of "salida de la superficie" = leave the 860x860 effective field.
    overhang_allowance_mm: float = 0.0
    # Physical board beyond the effective field (1000 - 860) / 2. MEASURED (fabrication files).
    physical_margin_mm: float = 70.0
    # MEASURED: corner ArUco 100 mm black + 20 mm white border, centred on the field corners.
    corner_marker_half_mm: float = 70.0

    @property
    def width(self) -> float:
        return self.cols * self.cell_mm

    @property
    def height(self) -> float:
        return self.rows * self.cell_mm


@dataclass(frozen=True)
class RoverGeometry:
    """Rover frame: origin = rotation centre (axle midpoint), +x forward, +y left."""
    # MEASURED (PIEZAS SEPARADAS.dxf + ruler): chassis 94 long, 99.5 wide outer, 93.5 channel.
    chassis_half_length: float = 47.0
    outer_half_width: float = 49.75
    inner_half_width: float = 46.75          # channel W_u / 2 = 93.5 / 2
    paddle_reach: float = 55.0               # MEASURED M6: paddles beyond the front plate
    # ASSUMED: axle midpoint coincides with chassis centre (not measured). + = chassis centre ahead of axle.
    chassis_center_x: float = 0.0
    # ASSUMED 0 (tool rejected both attempts): marker centre -> rotation centre, rover frame.
    marker_offset_fwd: float = 0.0
    marker_offset_left: float = 0.0
    marker_angle_offset: float = 0.0         # MEASURED ~0 deg (+-1)
    wheel_diameter: float = 33.0             # MEASURED M13
    track_width: float = 89.0                # MEASURED M13

    @property
    def x_rear(self) -> float:
        return self.chassis_center_x - self.chassis_half_length

    @property
    def x_front_plate(self) -> float:
        return self.chassis_center_x + self.chassis_half_length

    @property
    def x_paddle_tip(self) -> float:
        return self.x_front_plate + self.paddle_reach

    @property
    def sweep_radius(self) -> float:
        """Max distance of any footprint point from the rotation centre."""
        return max(math.hypot(self.x_paddle_tip, self.outer_half_width),
                   math.hypot(self.x_rear, self.outer_half_width))


@dataclass(frozen=True)
class CubeGeometry:
    side: float = 60.0                       # MEASURED (cubos.dxf, config_vision.json)

    @property
    def half(self) -> float:
        return self.side / 2.0

    @property
    def half_diag(self) -> float:
        return self.side / math.sqrt(2.0)


@dataclass(frozen=True)
class DepotConfig:
    # ASSUMED: no official size exists. Square of 5 cells (100 mm) centred on the published depot point,
    # i.e. the 100x100 corner square of the effective field. Delivered = whole cube footprint inside.
    half_size: float = 50.0
    # Margin kept between the planned cube footprint and the depot border (vision bias + push error).
    delivery_margin: float = 6.0
    # Consecutive stationary frames that must satisfy the criterion before declaring delivery.
    confirm_frames: int = 6


@dataclass(frozen=True)
class Margins:
    board: float = 12.0                      # rover footprint to board edge (planning)
    cube_nav: float = 14.0                   # rover envelope to non-target cube (planning)
    rover_rover: float = 45.0                # reservation inflation between rovers
    rover_rover_estop: float = 15.0          # hard emergency stop distance between footprints
    prepush_clearance: float = 12.0          # paddle tip / sweep circle to target cube at pre-push
    pose_uncertainty: float = 6.0            # marker offset + estimation error folded into footprints


@dataclass(frozen=True)
class MotionLimits:
    # ASSUMED until motor calibration: sim physical limits.
    wheel_speed_max: float = 180.0           # mm/s per wheel at throttle 1.0
    v_nav: float = 110.0                     # mm/s
    v_capture: float = 35.0
    v_push: float = 70.0
    v_push_slow: float = 35.0
    v_retreat: float = 60.0
    w_nav: float = 1.4                       # rad/s in-place rotation
    w_fine: float = 0.5
    accel: float = 300.0                     # mm/s^2 planning assumption


@dataclass(frozen=True)
class TelemetryPolicy:
    good_age_s: float = 0.15
    degraded_age_s: float = 0.40             # contract 6.4 suggests stopping > 0.5 s latency
    lost_age_s: float = 0.40
    max_blind_travel_mm: float = 30.0        # dead-reckoning allowance before forced stop
    pos_std_stop_mm: float = 12.0            # 1-sigma position uncertainty that forces STOP
    heading_std_stop_rad: float = math.radians(6.0)
    cube_fresh_s: float = 0.2
    cube_stale_s: float = 1.5


@dataclass(frozen=True)
class EstimatorConfig:
    meas_pos_std: float = 2.0                # mm, per-frame vision position noise model
    meas_heading_std: float = math.radians(1.5)
    accel_noise: float = 400.0               # mm/s^2 random-walk on v
    alpha_noise: float = 6.0                 # rad/s^2 random-walk on omega
    cmd_tau: float = 0.12                    # s, first-order response of (v, w) to commands
    gate_chi2: float = 16.0                  # Mahalanobis^2 gate (3 dof)
    max_consecutive_rejects: int = 5         # then re-initialise from vision
    history_s: float = 1.0                   # replay window for delayed measurements


@dataclass(frozen=True)
class PlannerConfig:
    delivery_candidates: int = 49
    max_push_legs: int = 3
    push_heading_window_deg: float = 20.0    # H-11: push within this of a known face normal
    nav_cell_mm: float = 20.0
    nav_heading_bins: int = 24
    nav_max_expansions: int = 120_000


@dataclass(frozen=True)
class Config:
    board: BoardConfig = field(default_factory=BoardConfig)
    rover: RoverGeometry = field(default_factory=RoverGeometry)
    cube: CubeGeometry = field(default_factory=CubeGeometry)
    depot: DepotConfig = field(default_factory=DepotConfig)
    margins: Margins = field(default_factory=Margins)
    limits: MotionLimits = field(default_factory=MotionLimits)
    telemetry: TelemetryPolicy = field(default_factory=TelemetryPolicy)
    estimator: EstimatorConfig = field(default_factory=EstimatorConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)

    @property
    def prepush_distance(self) -> float:
        """Rotation-centre to cube-centre distance at the pre-push pose.

        Must clear the cube (any orientation => half diagonal) with both the paddle tips
        (straight approach) and the full in-place rotation sweep (turning at the pose).
        """
        m = self.margins.prepush_clearance + self.margins.pose_uncertainty
        tips = self.rover.x_paddle_tip + self.cube.half_diag + m
        sweep = self.rover.sweep_radius + self.cube.half_diag + m
        return max(tips, sweep)

    @property
    def contact_distance(self) -> float:
        """Rotation-centre to cube-centre distance when a face-aligned cube touches the front plate."""
        return self.rover.x_front_plate + self.cube.half


DEFAULT = Config()

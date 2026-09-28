Implement ARC motion end to end. world.py now has `SegKind.ARC` with `Segment.curvature` (signed 1/mm, + = CCW w.r.t.
the direction of travel) and `reverse` (read rover_strategy/world.py). You may edit
rover_strategy/planning/navigation.py AND the SegmentFollower class (only) in rover_strategy/control/controllers.py,
plus tests. A previous attempt (tag rejected-codex-auto-c-*, report in its docs/codex_log/C01_*) emitted arcs as
micro rotate/straight segments and was REJECTED: slow execution, wrong non-collinear terminal straight for nonzero
start heading, cost underestimated. Requirements:
1) Hybrid A*: forward/reverse arc successors (several radii, e.g. 150/250/400 mm, both turn directions) emitted as
   single ARC segments; exact continuous swept-footprint checking (proven padding bound between samples); analytic
   terminal connectors must produce STRAIGHT segments collinear with the heading (assert it) and exact ARC geometry.
2) swept_polygons: guaranteed superset for ARC segments too.
3) Path.cost_s must equal the expected execution time of the emitted segments (arc length / v_nav, rotations incl.
   settle), so FSM timeouts are correct.
4) SegmentFollower: track ARC segments: feed-forward omega = curvature * v, plus closed-loop correction on the
   distance to the arc (radial error) and heading error relative to the arc tangent (same law as straight lines,
   signs correct for reverse), progress by arc angle, predictive stop.
5) Tests: goal (797, 619, -pi/2) from start (640, 443, 0) AND from 3 other start headings; obstacle cases; dense
   independent swept-area verification for arcs; a closed-loop follower test on a unicycle truth model with lag and
   10 % wheel asymmetry reaching the goal within 5 mm / 3 deg. All tests/test_navigation*.py must pass (except the
   known timing-flaky test under heavy load: report it).

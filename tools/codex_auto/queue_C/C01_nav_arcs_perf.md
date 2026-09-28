You own rover_strategy/planning/navigation.py (+ tests/test_navigation*.py, new tests). Two goals, in this order:

1) ARC PRIMITIVES (critical). The planner currently has only in-place rotations and straight moves. A pre-push pose
next to a field edge with the heading PARALLEL to that edge (e.g. goal (797, 619, -90 deg) on an 860 mm field; the
rover sweep radius is 113.5 mm, so it can never rotate in place anywhere on the line x=797) is therefore unreachable,
although a differential-drive rover can reach it with a curved approach. Such poses are required for pushing cubes
along edges into corner depots. Add forward and reverse ARC successors (constant curvature; several radii, e.g.
120/200/350 mm; left/right) to the Hybrid A* expansion and to the analytic/terminal connection, with exact
continuous swept-footprint collision checking for arcs (sample the arc with a proven padding bound like the rotation
case), and extend `Segment`-compatible output: represent an arc as a new SegKind is NOT allowed (world.py is
lead-owned) -> instead emit arcs as a sequence of short STRAIGHT+ROTATE micro-segments whose union is checked, OR
(preferred) add a module-level `ArcSegment` helper that `path_to_segments()` converts into short straight segments
(<= 20 mm chord, <= 8 deg heading step) that the existing SegmentFollower can track; state clearly which you chose.
swept_polygons must still be a guaranteed superset. Test: the example goal above (start (640,443,0), no obstacles,
board 860) must be solved; add dense independent swept-area checks for arcs; all existing navigation tests pass.

2) PERFORMANCE: then make typical queries >= 3x faster (profile first; hot spots were _fast_hull,
_fast_segment_collides, _analytic_connect). Keep all safety semantics. Report before/after timings.

Report also: how the terminal tolerance interacts with the capture alignment (goal tolerance 5 mm / 3 deg today).

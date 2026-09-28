Improve the time model used by rover_strategy/planning/task_allocator.py. Today nav_time is Euclidean/v + a detour
factor. Build a calibrated estimator: sample ~300 random navigation queries (NavPlanner in
rover_strategy/planning/navigation.py; do NOT edit that file) on the 860 mm field with 0-3 cube obstacles, fit a
cheap model (distance, heading changes at start/goal, obstacle proximity) to planner cost_s, store coefficients as
module-level constants with provenance, and use it as the default nav_time. Add a test that the fitted model's
median absolute error on a held-out seeded set is < 25 %. Report the fit.

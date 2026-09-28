Telemetry robustness. Files: rover_strategy/vision/parser.py, rover_strategy/vision/client.py, new tests.
(1) Add an integration test that starts the OFFICIAL mock publisher
(_source/Rover Vision Artificial/Vision-Rover-Challenge-main_unz/Vision-Rover-Challenge-main/vision-system/contrato/
mock_publisher.py) on a free port as a subprocess (it is pure python; read its CLI/config handling first), connects
VisionClient, drives phase commands if possible, and checks parsed frames (ids, colours, units, y-up conversion,
seq-gap counting, partial lines across TCP chunks). Skip gracefully if the port cannot be bound.
(2) Prepare for protocol v2 WITHOUT inventing it: make the version check a configurable set of accepted versions with
a per-version adapter hook; v1 stays the only implemented adapter; unknown versions are still dropped and counted.
Acceptance: new tests pass; full suite not worse.

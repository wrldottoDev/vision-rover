"""Estimator accuracy in the real sim sensor model: stationary / arc / stationary, for given (accel, alpha) noise."""
import sys, math, numpy as np
from dataclasses import replace
from rover_strategy.simulation import scenarios as SC
from rover_strategy.vision.parser import parse_message
from rover_strategy.estimation.pose_estimator import PoseEstimator
from rover_strategy.config import DEFAULT as C
from rover_strategy.world import WheelCommand
ecfg = replace(C.estimator, accel_noise=float(sys.argv[1]), alpha_noise=float(sys.argv[2]))
res = []
for seed in range(4):
    sc = SC.generate(seed, 'official_like'); world, emu = SC.instantiate(sc); emu.set_phase('RUNNING'); est = None
    for i in range(1500):
        cmd = (60, 100) if 300 < i < 900 else (0, 0)
        world.set_command(10, WheelCommand(*cmd), world.t); world.step(0.01); t = world.t; emu.capture(world, t)
        for m in emu.poll(t):
            o = parse_message(m, t).rovers.get(10)
            if not o: continue
            if est is None: est = PoseEstimator(ecfg, C.telemetry, 10); est.initialize(o.pose, o.t_capture)
            else: est.update_vision(o.pose, o.t_capture, t)
        if est and i % 5 == 0:
            est.set_command(t, (cmd[0] + cmd[1]) / 2, (cmd[1] - cmd[0]) / 89)
            e = est.estimate(t); x, y, th = world.rover_pose(10)
            res.append((i, math.degrees(abs(math.remainder(e.pose.theta - th, 2 * math.pi))), math.hypot(e.pose.x - x, e.pose.y - y), not est.is_safe_to_drive(t)))
a = np.array(res, float)
for lo, hi, n in ((100, 300, 'still'), (350, 900, 'arc'), (950, 1500, 'still2')):
    m = (a[:, 0] > lo) & (a[:, 0] < hi)
    print(f'  {n}: heading rms {np.sqrt(np.mean(a[m,1]**2)):.2f} deg, pos rms {np.sqrt(np.mean(a[m,2]**2)):.2f} mm, unsafe {a[m,3].mean():.1%}')

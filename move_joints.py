"""Sweep each SO-101 joint one at a time so you can see which actuator is which.

The actuators are position servos: writing a target angle to data.ctrl makes the
joint track it. Each joint sweeps a sine wave over 60% of its control range for
4 seconds, then hands off to the next one.
"""

import math
import time

import mujoco
import mujoco.viewer

# model = robot description (fixed), data = current sim state (changes every step)
model = mujoco.MjModel.from_xml_path("menagerie/robotstudio_so101/scene.xml")
data = mujoco.MjData(model)

SECONDS_PER_JOINT = 4.0

# open the viewer window; this script drives the physics
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    while viewer.is_running():
        step_start = time.time()
        t = time.time() - start

        # pick which joint's turn it is (next joint every 4s), and 0→1 progress
        active = int(t // SECONDS_PER_JOINT) % model.nu
        phase = (t % SECONDS_PER_JOINT) / SECONDS_PER_JOINT

        # sine-sweep the active joint at 60% of its range; hold the rest at 0
        lo, hi = model.actuator(active).ctrlrange
        mid, half = (lo + hi) / 2, (hi - lo) / 2
        data.ctrl[:] = 0.0
        data.ctrl[active] = mid + 0.6 * half * math.sin(2 * math.pi * phase)

        # step physics, redraw window
        mujoco.mj_step(model, data)
        viewer.sync()

        # sleep off leftover time so the sim runs at real-world speed
        leftover = model.opt.timestep - (time.time() - step_start)
        if leftover > 0:
            time.sleep(leftover)

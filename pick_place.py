"""Pick-and-place attempts with the SO-101.

Each attempt: spawn the block at a random reachable spot -> read its position
(ground truth from the sim, standing in for perception) -> pre-check -> IK for
the waypoints -> execute hover/descend/grasp/lift/transport/drop -> success
check -> reset home.

Run with the viewer:      .venv/bin/mjpython pick_place.py
Run headless (fast):      .venv/bin/python pick_place.py --headless
More attempts:            add e.g. --attempts 10
"""

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

import config as cfg
import checks
import ik

HOME = np.zeros(5)


class Sim:
    """Owns the model, data and viewer, and the primitive motions."""

    def __init__(self, headless):
        self.model = mujoco.MjModel.from_xml_path("menagerie/robotstudio_so101/pick_scene.xml")
        self.data = mujoco.MjData(self.model)
        self.viewer = None
        if not headless:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        self.arm = [self.model.actuator(n).id for n in ik.ARM_JOINTS]
        self.grip = self.model.actuator("gripper").id
        self.block_qpos = self.model.joint("block_free").qposadr[0]

    def step(self):
        start = time.time()
        mujoco.mj_step(self.model, self.data)
        if self.viewer:
            self.viewer.sync()
            # with the viewer, pace the sim to real time so it's watchable
            leftover = self.model.opt.timestep - (time.time() - start)
            if leftover > 0:
                time.sleep(leftover)

    def move_arm(self, q_target, seconds=cfg.MOVE_SECONDS):
        """Glide the arm's position targets from where they are to q_target."""
        q_from = self.data.ctrl[self.arm].copy()
        steps = int(seconds / self.model.opt.timestep)
        for i in range(steps):
            # cosine ease: start and end slow instead of jerking
            s = (1 - np.cos(np.pi * (i + 1) / steps)) / 2
            self.data.ctrl[self.arm] = q_from + s * (q_target - q_from)
            self.step()

    def set_gripper(self, value):
        self.data.ctrl[self.grip] = value
        for _ in range(int(cfg.SETTLE_SECONDS / self.model.opt.timestep)):
            self.step()

    def settle(self, seconds):
        for _ in range(int(seconds / self.model.opt.timestep)):
            self.step()

    def spawn_block(self, rng):
        """Drop the block at a random reachable spot, clear of the bin."""
        bin_xy = self.model.body("bin").pos[:2]
        while True:
            r = rng.uniform(*cfg.SPAWN_R)
            a = np.radians(rng.uniform(*cfg.SPAWN_ANGLE))
            xy = r * np.array([np.cos(a), np.sin(a)])
            if np.linalg.norm(xy - bin_xy) > cfg.SPAWN_CLEAR_OF_BIN:
                break
        i = self.block_qpos
        self.data.qpos[i:i + 7] = [xy[0], xy[1], 0.031, 1, 0, 0, 0]
        self.data.qvel[self.model.joint("block_free").dofadr[0]:][:6] = 0
        mujoco.mj_forward(self.model, self.data)

    def block_pos(self):
        """Ground-truth block position — the stand-in for perception."""
        return self.data.qpos[self.block_qpos:self.block_qpos + 3].copy()


def attempt(sim, rng):
    """One full pick-and-place. Returns (success, reason)."""
    sim.spawn_block(rng)
    target = sim.block_pos()
    print(f"  block at ({target[0]:+.3f}, {target[1]:+.3f}), "
          f"{np.linalg.norm(target[:2]):.3f}m from base")

    # first solve at the block center just to learn the gripper's orientation
    # there, then shift the target along the jaw-separation axis (site z) so
    # the block sits centered in the jaw gap instead of under the fixed jaw
    nominal = ik.solve(sim.model, HOME, target + [0, 0, cfg.GRASP_Z_OFFSET])
    _, R = ik.site_frame(sim.model, nominal.qpos)
    jaw_axis = R[:, 2] * [1, 1, 0]  # horizontal part only
    jaw_axis /= np.linalg.norm(jaw_axis)
    shift = cfg.GRASP_SIDE_OFFSET * jaw_axis

    grasp_pos = target + [0, 0, cfg.GRASP_Z_OFFSET] - shift
    # seed each solve from the nearby solution we already have — DLS is a local
    # method, and a good starting guess is the difference between converging or not
    hover = ik.solve(sim.model, nominal.qpos, [grasp_pos[0], grasp_pos[1], cfg.HOVER_Z])
    grasp = ik.solve(sim.model, hover.qpos, grasp_pos)
    bin_pos = sim.model.body("bin").pos
    carry = ik.solve(sim.model, hover.qpos, [grasp_pos[0], grasp_pos[1], cfg.CARRY_Z], point_down=False)
    drop = ik.solve(sim.model, HOME, [bin_pos[0], bin_pos[1], cfg.DROP_Z], point_down=False)

    ok, reason = checks.pre_check(sim.model, target, grasp, hover)
    if not ok:
        return False, f"pre-check: {reason}"

    sim.set_gripper(cfg.GRIPPER_OPEN)
    sim.move_arm(hover.qpos)          # above the block
    sim.move_arm(grasp.qpos)          # straight down onto it
    sim.set_gripper(cfg.GRIPPER_CLOSED)
    sim.move_arm(hover.qpos)          # lift
    sim.move_arm(carry.qpos)          # higher still, so the block clears the bin walls
    sim.move_arm(drop.qpos)           # swing to above the bin
    sim.set_gripper(cfg.GRIPPER_OPEN)  # release
    sim.settle(1.0)                   # let the block land and stop bouncing
    sim.move_arm(HOME)

    return checks.success_check(sim.model, sim.data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    sim = Sim(args.headless)
    rng = np.random.default_rng(args.seed)
    wins = 0
    for i in range(args.attempts):
        print(f"attempt {i + 1}/{args.attempts}")
        success, reason = attempt(sim, rng)
        wins += success
        print(f"  {'SUCCESS' if success else 'FAIL'}: {reason}")
    print(f"\n{wins}/{args.attempts} succeeded")
    if sim.viewer:
        sim.viewer.close()


if __name__ == "__main__":
    main()

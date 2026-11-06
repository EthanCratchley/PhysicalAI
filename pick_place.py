"""Pick-and-place attempts with the SO-101.

Each attempt: spawn the block somewhere in a weighted band — from right at the
base to well beyond the arm's reach -> read its position (ground truth from the
sim, standing in for perception) -> pre-check -> IK for the waypoints ->
execute hover/descend/grasp/lift/transport/drop -> success check -> reset home.

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


def sample_spawn(rng):
    """Pick a spawn band by weight, then a uniform (r, angle) inside it.
    Returns (band name, radius in m, angle in deg)."""
    weights = np.array([b[1] for b in cfg.SPAWN_BANDS], dtype=float)
    name, _, lo, hi = cfg.SPAWN_BANDS[rng.choice(len(cfg.SPAWN_BANDS), p=weights / weights.sum())]
    return name, rng.uniform(lo, hi), rng.uniform(*cfg.SPAWN_ANGLE)


def draw_injections(rng):
    """Roll each fault independently per cfg.INJECTIONS; returns the drawn
    faults with their magnitudes. Most draws come back empty or single."""
    out = []
    for kind, spec in cfg.INJECTIONS.items():
        if rng.random() >= spec["p"]:
            continue
        inj = {"type": kind}
        if kind == "camera":
            inj["shift"] = round(float(rng.uniform(*spec["shift"])), 4)
            inj["tilt_deg"] = round(float(rng.uniform(*spec["tilt_deg"]) * rng.choice([-1, 1])), 2)
            inj["dir_deg"] = round(float(rng.uniform(0, 360)), 1)
        if kind == "block_move":
            inj["dist"] = round(float(rng.uniform(*spec["dist"])), 4)
            inj["dir_deg"] = round(float(rng.uniform(0, 360)), 1)
        if kind == "noise":
            inj["offset"] = [round(float(v), 4) for v in rng.uniform(-spec["per_axis"], spec["per_axis"], 3)]
        if kind == "friction":
            inj["scale"] = round(float(rng.uniform(*spec["scale"])), 3)
        out.append(inj)
    return out


def perceive(truth, injections):
    """What the robot believes the block position is: ground truth corrupted by
    the perception-side faults. Physics-side faults (block_move, friction)
    leave perception alone."""
    p = truth.copy().astype(float)
    for inj in injections:
        if inj["type"] == "camera":
            a = np.radians(inj["tilt_deg"])
            p[:2] = [np.cos(a) * p[0] - np.sin(a) * p[1],
                     np.sin(a) * p[0] + np.cos(a) * p[1]]
            d = np.radians(inj["dir_deg"])
            p[:2] += inj["shift"] * np.array([np.cos(d), np.sin(d)])
        if inj["type"] == "noise":
            p += inj["offset"]
    return p


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
        bid = self.model.jnt_bodyid[self.model.joint("block_free").id]
        adr = self.model.body_geomadr[bid]
        self.block_geoms = slice(adr, adr + self.model.body_geomnum[bid])
        self.block_friction = self.model.geom_friction[self.block_geoms].copy()


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
        """Drop the block at a random banded spot, clear of the bin and not
        intersecting the robot. Returns the band name it spawned in."""
        bin_xy = self.model.body("bin").pos[:2]
        while True:
            band, r, a = sample_spawn(rng)
            xy = r * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
            if np.linalg.norm(xy - bin_xy) <= cfg.SPAWN_CLEAR_OF_BIN:
                continue
            i = self.block_qpos
            self.data.qpos[i:i + 7] = [xy[0], xy[1], 0.031, 1, 0, 0, 0]
            self.data.qvel[self.model.joint("block_free").dofadr[0]:][:6] = 0
            mujoco.mj_forward(self.model, self.data)
            if not self.block_touches_robot():
                return band

    def block_touches_robot(self):
        """Is the just-placed block in contact with anything besides the floor?
        Inner spawn bands reach the robot's own footprint, so a draw can land
        inside the base; those spots get resampled."""
        block = self.model.jnt_bodyid[self.model.joint("block_free").id]
        floor = self.model.geom("floor").id
        for c in self.data.contact[:self.data.ncon]:
            if floor in (c.geom1, c.geom2):
                continue
            if block in (self.model.geom_bodyid[c.geom1], self.model.geom_bodyid[c.geom2]):
                return True
        return False

    def block_pos(self):
        """Ground-truth block position — the stand-in for perception."""
        return self.data.qpos[self.block_qpos:self.block_qpos + 3].copy()

    def nudge_block(self, dist, dir_deg):
        """Teleport the block horizontally — the 'someone moved it' fault."""
        d = np.radians(dir_deg)
        self.data.qpos[self.block_qpos] += dist * np.cos(d)
        self.data.qpos[self.block_qpos + 1] += dist * np.sin(d)
        mujoco.mj_forward(self.model, self.data)

    def scale_block_friction(self, scale):
        """Scale the block's contact friction for this attempt. MuJoCo combines
        contact friction as the max of the two geoms (at equal priority), which
        would swallow a reduction — raising the block's priority makes its
        scaled values win instead."""
        self.model.geom_friction[self.block_geoms] = self.block_friction * scale
        self.model.geom_priority[self.block_geoms] = 1

    def restore_block_friction(self):
        self.model.geom_friction[self.block_geoms] = self.block_friction
        self.model.geom_priority[self.block_geoms] = 0


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

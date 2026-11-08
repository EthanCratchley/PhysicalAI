"""Pick-and-place attempts with the SO-101.

Each attempt: spawn the block somewhere in a weighted band — from right at the
base to well beyond the arm's reach -> read its position (ground truth from the
sim, standing in for perception) -> pre-check -> IK for the waypoints ->
execute hover/descend/grasp/lift/transport/drop -> success check -> reset home.

Each block is an episode of up to MAX_TRIES actions. Per try the robot
re-perceives and decides: pick if the pre-check passes; otherwise, if it can at
least touch the block, nudge it toward the comfortable workspace and try again
(earning further nudges only while they make progress); otherwise decline.
Outcomes: picked / recovered (picked after nudging) / missed / gave_up /
declined.

Run with the viewer:      .venv/bin/mjpython pick_place.py
Run headless (fast):      .venv/bin/python pick_place.py --headless
More attempts:            add e.g. --attempts 10
"""

import argparse
import datetime
import glob
import json
import os
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


def dist_to_graspable(p):
    """How far a position is from the comfortable grasp region — the reach
    annulus intersected with the angular sector (0 if inside). The robot's
    progress metric: nudges must shrink this. Angle matters because hover IK
    fails at extreme angles even at a comfortable radius."""
    r = np.linalg.norm(p[:2])
    deg = np.degrees(np.arctan2(p[1], p[0]))
    if cfg.REACH_MIN <= r <= cfg.REACH_MAX and cfg.NUDGE_ANGLE_CLAMP[0] <= deg <= cfg.NUDGE_ANGLE_CLAMP[1]:
        return 0.0
    a = np.radians(np.clip(deg, *cfg.NUDGE_ANGLE_CLAMP))
    nearest = np.clip(r, cfg.REACH_MIN, cfg.REACH_MAX) * np.array([np.cos(a), np.sin(a)])
    return float(np.linalg.norm(p[:2] - nearest))


def nudge_target(p):
    """Where a nudge should drag the block: toward the middle of the
    comfortable workspace (radius and angle clamped), at most NUDGE_DRAG_MAX
    per nudge. Returns the drag endpoint for the gripper."""
    a = np.radians(np.clip(np.degrees(np.arctan2(p[1], p[0])), *cfg.NUDGE_ANGLE_CLAMP))
    v = cfg.NUDGE_TARGET_R * np.array([np.cos(a), np.sin(a)]) - p[:2]
    d = np.linalg.norm(v)
    if d > cfg.NUDGE_DRAG_MAX:
        v *= cfg.NUDGE_DRAG_MAX / d
    return np.array([p[0] + v[0], p[1] + v[1], cfg.NUDGE_Z])


def load_failures(path):
    """Read a run log and return (seed, failed episode numbers)."""
    with open(path) as f:
        log = json.load(f)
    return log["seed"], [e["episode"] for e in log["episodes"] if not e["success"]]


def plan_touch(model, p):
    """Find a reachable, collision-free touch pose on the block, trying the
    lowest height first (best contact) and climbing when a pose is unreachable
    or physically blocked. Returns (ik_result, height) or None if every
    candidate height fails — then the block is genuinely untouchable."""
    for z in cfg.NUDGE_TOUCH_HEIGHTS:
        t = ik.solve(model, HOME, [p[0], p[1], z], point_down=False)
        if t.pos_err <= cfg.TOUCH_TOL and not checks.pose_collides(model, t.qpos):
            return t, z
    return None


def grasp_yaw(quat, half):
    """The jaw-axis yaw that grips a (possibly tipped) block across its
    narrowest horizontal dimension. A tipped block presents its 6cm length
    horizontally; jaws closing across that barely span it, while the 4cm
    width grips deep — so find the block's horizontal axes and aim across
    the smallest."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(quat, dtype=float))
    R = R.reshape(3, 3)
    up = int(np.argmax(np.abs(R[2])))  # body axis most aligned with world z
    narrow = min((i for i in range(3) if i != up), key=lambda i: half[i])
    return float(np.arctan2(R[1, narrow], R[0, narrow]))


def pick_is_futile(truth, last_pick_truth):
    """A failed pick earns a retry only if the world changed: the sim is
    deterministic, so re-running a pick against a block that hasn't moved
    reproduces the same whiff."""
    if last_pick_truth is None:
        return False
    return bool(np.linalg.norm(truth - last_pick_truth) < cfg.PICK_MIN_MOVE)


def classify(actions, in_bin):
    """Score an episode from the actions taken ("pick"/"nudge"/"none") and
    whether the block ended in the bin. Returns (outcome, counts as success).

    picked/recovered: in the bin, without/with nudging on the way.
    missed: a pick was approved at some point but never landed.
    gave_up: nudged but never got the block into grasp range.
    declined: never acted — couldn't even touch it, and said so (trusted,
    not verified; the spawn band in the log is the audit trail).
    """
    if in_bin:
        return ("recovered", True) if "nudge" in actions else ("picked", True)
    if "pick" in actions:
        return "missed", False
    if "nudge" in actions:
        return "gave_up", False
    return "declined", True


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

    def block_grasp_yaw(self):
        """Ground-truth best jaw-axis yaw for the block as it currently lies —
        physics can spin or tip it after nudges and whiffs."""
        quat = self.data.qpos[self.block_qpos + 3:self.block_qpos + 7]
        return grasp_yaw(quat, self.model.geom_size[self.block_geoms.start])

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


def plan_pick(sim, target, face_yaw=0.0):
    """IK for the full pick sequence at a perceived position, plus the
    pre-check's verdict on it. face_yaw is the desired jaw-axis yaw (from
    grasp_yaw): grasp and hover wrists are rolled so the jaws close across
    the block's narrowest horizontal dimension."""
    # first solve at the block center just to learn the gripper's orientation
    # there, then shift the target along the jaw-separation axis (site z) so
    # the block sits centered in the jaw gap instead of under the fixed jaw
    nominal = ik.solve(sim.model, HOME, target + [0, 0, cfg.GRASP_Z_OFFSET])
    aligned = ik.align_wrist(sim.model, nominal.qpos, face_yaw)
    _, R = ik.site_frame(sim.model, aligned)
    jaw_axis = R[:, 2] * [1, 1, 0]  # horizontal part only
    jaw_axis /= np.linalg.norm(jaw_axis)
    shift = cfg.GRASP_SIDE_OFFSET * jaw_axis

    grasp_pos = target + [0, 0, cfg.GRASP_Z_OFFSET] - shift
    # seed each solve from the nearby solution we already have — DLS is a local
    # method, and a good starting guess is the difference between converging or not
    hover = ik.solve(sim.model, aligned, [grasp_pos[0], grasp_pos[1], cfg.HOVER_Z])
    grasp = ik.solve(sim.model, hover.qpos, grasp_pos)
    # the solves can drift the roll again; square the jaws on the final poses
    hover.qpos = ik.align_wrist(sim.model, hover.qpos, face_yaw)
    grasp.qpos = ik.align_wrist(sim.model, grasp.qpos, face_yaw)
    bin_pos = sim.model.body("bin").pos
    carry = ik.solve(sim.model, hover.qpos, [grasp_pos[0], grasp_pos[1], cfg.CARRY_Z], point_down=False)
    drop = ik.solve(sim.model, HOME, [bin_pos[0], bin_pos[1], cfg.DROP_Z], point_down=False)
    ok, verdict = checks.pre_check(sim.model, target, grasp, hover)
    return hover, grasp, carry, drop, ok, verdict


def episode(sim, rng):
    """One block, up to cfg.MAX_TRIES actions. Each try the robot re-perceives
    and decides: pick if the pre-check passes; otherwise nudge the block toward
    the workspace if it can at least touch it; otherwise stop. A second nudge
    is only earned if the previous one made progress. Returns a log entry dict."""
    band = sim.spawn_block(rng)
    # episode-internal randomness (injections, per-try noise redraws) comes from
    # a child rng so --replay can burn skipped episodes at fixed cost
    erng = np.random.default_rng(rng.integers(2**32))
    injections = draw_injections(erng)
    spawn = sim.block_pos()
    print(f"  block at ({spawn[0]:+.3f}, {spawn[1]:+.3f}), "
          f"{np.linalg.norm(spawn[:2]):.3f}m from base [{band}]")
    if injections:
        print(f"  injections: {injections}")
    for inj in injections:
        if inj["type"] == "friction":
            sim.scale_block_friction(inj["scale"])

    tries, actions = [], []
    stop = "out of tries"
    in_bin = False
    nudge_gap = None   # gap at the moment of the last nudge, for the progress rule
    last_pick = None   # block truth at the moment of the last pick, same idea
    for t in range(1, cfg.MAX_TRIES + 1):
        for inj in injections:
            # the camera bias is systematic, but sensor noise is a fresh error
            # on every reading
            if inj["type"] == "noise" and t > 1:
                per = cfg.INJECTIONS["noise"]["per_axis"]
                inj["offset"] = [round(float(v), 4) for v in erng.uniform(-per, per, 3)]
        truth = sim.block_pos()
        perceived = perceive(truth, injections)
        gap = dist_to_graspable(perceived)
        rec = {"try": t,
               "truth": [round(float(v), 4) for v in truth],
               "perceived": [round(float(v), 4) for v in perceived],
               "gap": round(float(gap), 4)}

        hover, grasp, carry, drop, ok, verdict = plan_pick(sim, perceived, sim.block_grasp_yaw())
        rec["precheck"] = {"ok": ok, "reason": verdict}

        if t == 1:
            # the plan is now fixed: a block_move fault strikes after the first
            # plan, before the first action
            for inj in injections:
                if inj["type"] == "block_move":
                    sim.nudge_block(inj["dist"], inj["dir_deg"])

        if ok:
            if pick_is_futile(truth, last_pick):
                rec["action"] = "none"
                actions.append("none")
                tries.append(rec)
                stop = "no progress"
                print(f"  try {t}: block hasn't moved since the last whiffed pick — stopping")
                break
            rec["action"] = "pick"
            actions.append("pick")
            last_pick = truth
            print(f"  try {t}: pick")
            sim.set_gripper(cfg.GRIPPER_OPEN)
            sim.move_arm(carry.qpos)          # travel high: joint-space glides sweep
                                              # low arcs that knock the block around
            sim.move_arm(hover.qpos)          # then descend above the block
            sim.move_arm(grasp.qpos)          # straight down onto it
            sim.set_gripper(cfg.GRIPPER_CLOSED)
            sim.move_arm(hover.qpos)          # lift
            sim.move_arm(carry.qpos)          # higher still, so the block clears the bin walls
            sim.move_arm(drop.qpos)           # swing to above the bin
            sim.set_gripper(cfg.GRIPPER_OPEN)  # release
            sim.settle(1.0)                   # let the block land and stop bouncing
            sim.move_arm(HOME)
            in_bin, result = checks.success_check(sim.model, sim.data)
            rec["result"] = {"in_bin": in_bin, "reason": result}
            tries.append(rec)
            print(f"    {'in bin' if in_bin else result}")
            if in_bin:
                stop = None
                break
            continue

        print(f"  try {t}: pre-check declines ({verdict})")
        plan = plan_touch(sim.model, perceived)
        if plan is None:
            rec["action"] = "none"
            actions.append("none")
            tries.append(rec)
            stop = "untouchable"
            print("    no reachable, collision-free touch pose at any height — stopping")
            break
        touch, touch_z = plan
        if nudge_gap is not None and nudge_gap - gap < cfg.NUDGE_MIN_PROGRESS:
            rec["action"] = "none"
            actions.append("none")
            tries.append(rec)
            stop = "no progress"
            print(f"    last nudge didn't help (gap {nudge_gap:.3f}m -> {gap:.3f}m) — stopping")
            break

        drag = nudge_target(perceived)
        drag[2] = touch_z  # drag at the height the touch pose was found at
        # descend from above the block, drag, then lift before going home —
        # otherwise the glides sweep through the block area and knock it in
        # uncontrolled directions
        high = ik.solve(sim.model, touch.qpos, [perceived[0], perceived[1], cfg.CARRY_Z],
                        point_down=False)
        above = ik.solve(sim.model, touch.qpos, [perceived[0], perceived[1], cfg.HOVER_Z],
                         point_down=False)
        end = ik.solve(sim.model, touch.qpos, drag, point_down=False)
        lift = ik.solve(sim.model, end.qpos, [drag[0], drag[1], cfg.CARRY_Z], point_down=False)
        rec["action"] = "nudge"
        rec["drag_to"] = [round(float(v), 4) for v in drag]
        actions.append("nudge")
        tries.append(rec)
        nudge_gap = gap
        print(f"    nudging toward ({drag[0]:+.3f}, {drag[1]:+.3f})")
        sim.set_gripper(cfg.GRIPPER_CLOSED)  # closed jaws make a solid paddle
        sim.move_arm(high.qpos)   # travel high first, same reason as the pick
        sim.move_arm(above.qpos)
        sim.move_arm(touch.qpos)
        sim.move_arm(end.qpos)
        sim.move_arm(lift.qpos)
        sim.move_arm(HOME)
        sim.settle(0.5)

    sim.restore_block_friction()
    outcome, success = classify(actions, in_bin)
    return {
        "band": band,
        "spawn": [round(float(v), 4) for v in spawn],
        "r": round(float(np.linalg.norm(spawn[:2])), 4),
        "injections": injections,
        "tries": tries,
        "outcome": outcome,
        "success": success,
        "stop": stop,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--attempts", type=int, default=3)
    parser.add_argument("--seed", type=int, default=None,
                        help="omit for a random seed; it's printed and logged so any run can be replayed")
    parser.add_argument("--replay", type=str, default=None,
                        help="comma-separated episode numbers from a previous run with this seed, "
                             "e.g. --replay 11,14,32; other episodes are skipped")
    parser.add_argument("--feasible", action="store_true",
                        help="spawn only in the comfortable workspace (no at/beyond-reach "
                             "stress spawns); pass it again when replaying a feasible run")
    parser.add_argument("--replay-failures", nargs="?", const="latest", default=None,
                        metavar="LOG",
                        help="replay every failed episode from a run log "
                             "(a logs/run_*.json path; defaults to the newest log)")
    args = parser.parse_args()

    if args.feasible:
        # sample_spawn reads these at call time, and the log's config snapshot
        # records the swapped values, so feasible runs stay self-describing
        cfg.SPAWN_BANDS = cfg.SPAWN_BANDS_FEASIBLE
        cfg.SPAWN_ANGLE = cfg.SPAWN_ANGLE_FEASIBLE

    replay = None
    if args.replay_failures:
        path = args.replay_failures
        if path == "latest":
            path = sorted(glob.glob("logs/run_*.json"))[-1]
        args.seed, failed = load_failures(path)
        if not failed:
            print(f"no failures in {path}")
            return
        replay = set(failed)
        args.attempts = max(replay)
        print(f"replaying {len(failed)} failures from {path}")
    elif args.replay:
        replay = {int(n) for n in args.replay.split(",")}
        args.attempts = max(replay)

    if args.seed is None:
        args.seed = int(np.random.SeedSequence().entropy % 2**31)
    print(f"seed {args.seed}")

    sim = Sim(args.headless)
    rng = np.random.default_rng(args.seed)
    entries = []
    for i in range(args.attempts):
        if replay and i + 1 not in replay:
            # burn this episode's main-stream draws (spawn + child rng seed) so
            # the replayed ones come out exactly as they did in the original run
            sim.spawn_block(rng)
            rng.integers(2**32)
            continue
        print(f"episode {i + 1}/{args.attempts}")
        entry = episode(sim, rng)
        entries.append({"episode": i + 1, **entry})
        tag = "SUCCESS" if entry["success"] else "FAIL"
        print(f"  {tag} ({entry['outcome']}, {len(entry['tries'])} tries"
              + (f", stop: {entry['stop']}" if entry["stop"] else "") + ")")

    wins = sum(e["success"] for e in entries)
    counts = {}
    for e in entries:
        counts[e["outcome"]] = counts.get(e["outcome"], 0) + 1
    print(f"\n{wins}/{len(entries)} succeeded  "
          + "  ".join(f"{k}={v}" for k, v in sorted(counts.items())))

    # one log file per run: every attempt with its spawn band, the pre-check's
    # verdict, the physical result, and the exact config used — so runs stay
    # comparable while config values are being tuned
    os.makedirs("logs", exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    path = f"logs/run_{stamp}.json"
    with open(path, "w") as f:
        json.dump({
            "time": stamp,
            "seed": args.seed,
            "replay": sorted(replay) if replay else None,
            "episodes_run": len(entries),
            "successes": wins,
            "outcomes": counts,
            "note": "per block the robot picks when the pre-check passes, nudges the block "
                    "toward the workspace when it can at least touch it, and stops otherwise; "
                    "declines are trusted, not verified — audit them against the spawn band",
            "episodes": entries,
            "config": {k: getattr(cfg, k) for k in dir(cfg) if k.isupper()},
        }, f, indent=2)
    print(f"log written to {path}")
    if sim.viewer:
        sim.viewer.close()


if __name__ == "__main__":
    main()

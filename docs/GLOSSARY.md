# Glossary

Terms, the config values that matter, and the functions worth knowing.
One line each; read `ARCHITECTURE.md` for how they connect.

## Concepts

- **Episode** — one spawned block, start to finish, up to the try budget.
- **Try** — one action within an episode: a pick, a nudge, or a stop.
- **Perceived vs truth** — truth is the sim's ground-truth block state;
  perceived is what the robot plans on (truth + injected faults).
- **Pre-check** — "should I even attempt this grasp?": reach bounds, IK
  convergence, self-collision, joint-limit margin.
- **Nudge** — dragging the block toward the comfortable workspace when it can't
  be picked where it lies.
- **Gap** — distance from the block to the comfortable grasp region (reach
  annulus ∩ angle sector). The progress currency: nudges must shrink it.
- **Outcomes** — `picked` (straight pick), `recovered` (picked after nudging),
  `missed` (approved pick never landed), `gave_up` (nudging stalled),
  `declined` (untouchable, never acted — counts as success, trusted).
- **Injection / fault** — deliberate corruption per episode: camera (systematic
  position bias), noise (fresh per reading), block_move (world changes after
  planning), friction (slippery block). Each rolls independently (~15%).
- **Band** — which spawn ring the block started in: inner / min_edge / mid /
  max_edge / outer. The audit trail for declines.
- **Jaw** — a gripper finger; the SO-101 has a fixed jaw and a moving jaw. The
  *jaw axis* is the direction they open/close along — the squeeze direction.
- **Yaw** — rotation about the vertical axis, i.e. a compass heading (from the
  aircraft roll/pitch/yaw trio). *Jaw yaw* = which way the squeeze axis points
  seen from above; aligning it to the block's narrow side is what makes grips
  deep instead of corner-on.
- **Frame** — a position plus an orientation (a point and three axes) attached
  to something: the world has one, the gripper has one (`site_frame`), the
  block and future camera each have their own. It matters because no
  coordinate means anything until you say which frame it's in — yaw lives in
  the world frame, the jaw axis is a gripper-frame axis read out in world
  coordinates, and bugs like the side-offset shift are frame conversions done
  right. The camera arc is mostly frame work: a camera reports poses in *its*
  frame, and calibration is finding the transform into the robot's.
- **DLS (damped least squares)** — the iterative IK method; local, so it can
  fail at reachable targets (hence seeded restarts).
- **Tipped** — the block fell on its side (center z ≈ 0.02 instead of 0.031);
  presents 6cm horizontally instead of 4cm.
- **Seed / replay** — one integer reproduces an entire run exactly;
  `--replay-failures` reruns a log's failures in the viewer.
- **Feasible mode** — `--feasible`: spawns only where a clean pick is possible.
  The "fair" score (mid-90s) vs the stress score (mid-80s).

## Config values you'll actually tune (config.py)

- `REACH_MIN / REACH_MAX` (0.12 / 0.30) — pre-check's reach annulus. Evidence
  says MAX is conservative; widening it trades declines for attempts.
- `IK_POS_TOL` (0.015) — how far off a "converged" IK solution may be. Looser =
  fewer declines but sloppier grasps; part of the whiff error budget.
- `GRASP_Z_OFFSET` (−0.012) — grasp depth. More negative = deeper jaws, less
  floor clearance.
- `GRASP_SIDE_OFFSET` (0.022) — centers the block in the asymmetric jaw gap.
- `HOVER_Z / CARRY_Z / DROP_Z` — waypoint heights: descend-from, travel/lift,
  release. HOVER_Z is reach-limited (vertical gripper only works low).
- `GRIPPER_OPEN / GRIPPER_CLOSED` (1.0 / 0.3) — jaw servo targets.
- `SPAWN_BANDS / SPAWN_ANGLE` — the test distribution itself; `*_FEASIBLE`
  variants are what `--feasible` swaps in.
- `MAX_TRIES / MAX_TRIES_HARD` (4 / 8) — try budget; productive nudges refund.
- `NUDGE_MIN_PROGRESS` (0.005) — how much a nudge must shrink the gap to earn
  another.
- `NUDGE_TARGET_R / NUDGE_ANGLE_CLAMP` — where nudges drag blocks toward; the
  clamp stays clear of the shoulder_pan limit.
- `NUDGE_TOUCH_HEIGHTS` (0.02–0.05) — candidate touch heights, lowest first.
- `TOUCH_TOL` (0.04) — how close best-effort IK must get for a block to count
  as touchable.
- `PICK_MIN_MOVE` (0.005) — a failed pick must have moved the block this much
  to earn a retry.
- `INJECTIONS` — fault probabilities and magnitude ranges.

## Functions worth knowing

- `ik.solve(model, seed_qpos, target, point_down)` — DLS with random restarts;
  returns qpos + errors + converged flag. Best effort even on failure.
- `ik.align_wrist(model, qpos, face_yaw)` — rolls the wrist so the jaw axis
  hits a requested yaw (mod 180°) without moving the gripper.
- `ik.site_frame(model, qpos)` — forward kinematics: gripper position +
  rotation for given joint angles.
- `pick_place.grasp_yaw(quat, half)` — best jaw-axis yaw for the block as it
  lies: across its narrowest horizontal dimension.
- `pick_place.plan_pick(sim, target, face_yaw)` — all pick waypoints + the
  pre-check verdict.
- `pick_place.plan_touch(model, p)` — reachable, collision-free touch pose, or
  None (= genuinely untouchable).
- `pick_place.perceive(truth, injections)` — applies perception-side faults;
  the future camera seam.
- `pick_place.dist_to_graspable(p)` — the gap metric.
- `pick_place.nudge_target(p)` — where a nudge drags the block toward.
- `pick_place.classify(actions, in_bin)` — episode outcome + success.
- `pick_place.pick_is_futile / earned_budget` — the two self-honesty rules.
- `checks.pre_check / success_check / pose_collides` — attempt gate, bin check,
  kinematic self-collision test.
- `Sim.spawn_block / block_pos / block_grasp_yaw / move_arm / set_gripper` —
  the live-sim surface everything else goes through.

# Ideas / Future Implementations

Deferred ideas with the evidence that motivated them. Pull one out when it's time.

## Architectural debt

Band-aids that were reasonable in the moment but patch around a cause instead of
fixing it. The pattern to watch: each one already cost us once, quietly — a patch
holds until conditions change, then resurfaces as a "new" bug somewhere else.
All three will matter more in the perception / real-hardware arc, where error
budgets tighten.

- **Jaw yaw belongs inside the IK solver, not after it.** `align_wrist` rotates
  the wrist post-solve, then has to re-align after every subsequent solve because
  DLS drifts the roll again. It only works because roll is decoupled when the
  gripper points straight down — a geometric coincidence. It already caught up
  with us: the repeated re-align calls in `plan_pick` are the patch patching
  itself. Breaks outright if we ever allow tilted grasps. Fix: add a yaw row to
  the DLS error vector.

- **One IK tolerance serves two jobs, and the solver stops early.** `IK_POS_TOL`
  was loosened 5→15mm in week one to kill false declines — a feasibility fix
  applied to an execution knob. Because `solve` returns at the first iterate
  under tolerance, "converged" routinely means ~9mm off, which resurfaced later
  as the shallow-grip problem (gripper settling high) and part of the residual
  whiffs; `GRASP_Z_OFFSET = -0.012` partly compensates for it rather than fixing
  it. Fix: loose tolerance for the pre-check question, then polish to the
  solver's best before commanding the arm.

- **Reachability constants are hand-set when they could be derived.**
  `REACH_MIN/MAX`, band edges, and `NUDGE_ANGLE_CLAMP` are magic numbers; the
  clamp was hand-tightened to dodge the shoulder_pan limit after nudges aimed
  blocks at it. We already computed the true reachable set once (the workspace
  sweep) and didn't wire it in, and the crude annulus has already contradicted
  the accurate checks once (the wrong_decline era). Fix: precompute the
  reachable region from the model (IK + collision sweep) at startup and derive
  these numbers from it, so a model change can't silently rot them.

## Risk-graded pick decision
The pre-check is binary (possible / not possible), but approved picks near the reach
boundaries fail far more often than central ones (last 100-run: ~83% success in `mid`,
~61% at `max_edge`, ~22% at `min_edge`). Idea: if a pick is approved but the block sits
within some margin of a reach bound (~3cm, tune from logs), do one centering nudge first,
then pick. Uses the existing nudge primitive; trades one extra action for moving
coin-flip grasps into the high-success zone.

## Bin-rim grasp failure
Found in a 1/1000 run (attempt 636): block spawned 0.120m from the bin (just past the
0.11 clearance), `GRASP_SIDE_OFFSET` shifted the grasp target toward the bin, and the
gripper descended onto the bin rim and closed on air. Fix candidates: account for the
side-offset direction in `SPAWN_CLEAR_OF_BIN`, or pre-check the grasp/hover poses for
bin collision (extend `pose_collides` to include the bin).

## Grasp verification after lift
Success is only checked after the full transport; a failed grasp still pantomimes the
whole carry/drop sequence. Industry systems verify the grasp immediately (force/width
sensing). Sim version: after the lift waypoint, check the block's height — if it isn't
elevated, skip transport and go straight to the next try. Faster episodes and a cleaner
distinction between "never grasped" and "dropped in transit" in the logs.

## No home-reset between tries
Every action currently ends by returning to HOME before the next try plans and moves.
Continuing from the current pose (nudge → re-perceive → pick directly from where the
arm already is) would cut episode time substantially — most moves in a multi-try
episode are commute, not work. Needs care: IK seeds should come from the current pose,
and the futility/progress rules assume a clean re-observation between tries.

## Derive waypoint heights from the object, not config
`HOVER_Z`, `CARRY_Z`, `DROP_Z`, and `GRASP_Z_OFFSET` silently encode the one 6cm
block — a taller or shorter object breaks all four. Instead, config should hold
*margins* (clearance above the object, clearance over obstacles) and the absolute
heights get computed per object from its perceived size and orientation (already
available: `grasp_yaw` finds the up-axis and half-extents; later the camera
estimates the same). Constraint to respect: hover is capped by the point-down
kinematic ceiling (~0.09m), so tall objects force tilted grasps — which is also
the scenario that breaks the `align_wrist` band-aid (see Architectural debt);
these two should likely be fixed together. Prerequisite for testing object
variety.

## Randomize the bin position
The block spawn is randomized but the bin sits at a fixed (0.075, -0.207). Randomizing
the bin per episode (within reach, clear of the spawn) would exercise the drop side the
way spawn bands exercise the grasp side: drop IK at varied angles/radii, carry paths of
different lengths, and new failure modes (drop near a reach boundary). The success check
and `SPAWN_CLEAR_OF_BIN` resampling already read the bin position from the model, so
most of the work is writing the bin's new pose into the scene per episode and logging it.

## Camera perception in simulation
The scene already defines a fixed `overhead` camera in `pick_scene.xml`. Render from it
and estimate the block's pose from pixels (classic CV is enough: color segmentation +
known geometry), then feed that estimate into `perceive()` instead of ground truth.
This replaces the *injected* camera faults with *real* perception error while runs stay
cheap, headless and replayable — the single best bridge toward hardware, built before
buying anything.

## Real perception stack (hardware)
The real-arm version needs a physical camera, calibration, and block pose estimation —
likely the largest single sim-to-real work item. Doing the sim-camera step first means
the estimator, its error profile, and the decision layer's tolerance of it are all
proven before hardware arrives.

## Learning / RL experiments
Everything so far is hand-coded; the natural experiment is letting the robot learn from
its mistakes instead. Realistic entry points, smallest first:
- **Config tuning from logs**: treat `config.py` numbers (offsets, tolerances, thresholds)
  as parameters and optimize them against batch success rate — "learning" without any RL
  machinery, and the logs already contain the data.
- **Learned pre-check**: train a small classifier on episode logs (position, band, faults
  → pick succeeded?) to replace the hand-written reach/margin rules; directly measurable
  against the current pre-check's 81%/23% style splits.
- **RL for one primitive**: learn just the grasp (or just the nudge) with RL in MuJoCo
  (e.g. PPO via Gymnasium + Stable-Baselines3) while the episode strategy stays
  hand-coded. Full end-to-end RL of the whole task is a much bigger lift.
The harness is the asset here: randomized spawns, fault injection, and JSON logs are
exactly the environment + reward signal a learning setup needs.

## Sim-to-real calibration (system ID)
Before expecting sim numbers on the real SO-101: measure real servo backlash, droop
under load, and gripper friction, and set the sim's parameters to match instead of
using MuJoCo defaults. Every constant in `config.py` is currently fit to sim physics.

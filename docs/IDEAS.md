# Ideas / Future Implementations

Deferred ideas with the evidence that motivated them. Pull one out when it's time.

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

## Randomize the bin position
The block spawn is randomized but the bin sits at a fixed (0.075, -0.207). Randomizing
the bin per episode (within reach, clear of the spawn) would exercise the drop side the
way spawn bands exercise the grasp side: drop IK at varied angles/radii, carry paths of
different lengths, and new failure modes (drop near a reach boundary). The success check
and `SPAWN_CLEAR_OF_BIN` resampling already read the bin position from the model, so
most of the work is writing the bin's new pose into the scene per episode and logging it.

## Real perception stack
Perception is currently sim ground truth plus injected faults. The real-arm version
needs a camera, calibration, and block pose estimation — likely the largest single
sim-to-real work item. The camera/noise injections already harden the decision layer
against the error profile a real camera will produce.

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

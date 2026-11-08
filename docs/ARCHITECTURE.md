# Architecture

How the pieces fit. Five source files, one scene, one log folder.

## Files

```
pick_scene.xml   the world: SO-101 arm, block (4x4x6cm), bin, overhead camera
config.py        every tunable number; code reads it, you edit it
ik.py            inverse kinematics: solve / align_wrist / site_frame
checks.py        judgment calls: pre_check, success_check, pose_collides
pick_place.py    everything else: Sim wrapper, strategy, episode loop, main
tests/           pure-function tests + kinematic regression tests
logs/            one JSON per run: every try of every episode + config snapshot
docs/            this folder
```

Dependency direction is one-way: `pick_place → checks → ik → config`. Nothing
imports upward, and only `pick_place.py` touches the live simulation.

## One run

```
main()
  parse args (seed, attempts, --feasible, --replay...)
  for each episode:
    episode(sim, rng)        one block, start to finish
  write logs/run_<stamp>.json
```

## One episode (the core loop, in pick_place.episode)

```
spawn block in a weighted band  →  roll fault injections
repeat (budget: MAX_TRIES, productive nudges refund a try, ceiling 8):
  perceive      ground truth + injected corruption = what the robot believes
  plan_pick     IK for all waypoints + wrist aligned to the block's narrow axis
  pre_check     reach, IK converged, no self-collision, joint margins
  ├─ pass  → PICK: travel high → hover → descend → close → lift → carry →
  │          drop over bin → in bin? done : retry (only if the block moved)
  └─ fail  → plan_touch finds a reachable, collision-free touch pose
       ├─ none → DECLINE (untouchable), episode over
       └─ ok   → NUDGE: drag the block toward the comfortable workspace,
                 then loop — but only if the last nudge made progress
classify(actions, in_bin) → picked / recovered / missed / gave_up / declined
```

Two self-honesty rules guard the loop: a failed **pick** earns a retry only if
it moved the block (`pick_is_futile`), and a second **nudge** must be earned by
the first one shrinking the gap (`earned_budget` / `nudge_gap`).

The sections below zoom into each box of that loop.
(Perceive gets its own diagram once the sim camera replaces ground truth —
today it's one line of arithmetic; see "Perception and faults".)

## Zoom: planning a pick (pick_place.plan_pick)

Planning is all IK — six solves and three wrist alignments before the arm
ever moves:

```
perceived block position + face yaw (grasp_yaw: narrowest horizontal axis)
  │
  solve nominal grasp        only to learn the gripper's orientation there
  align_wrist to face yaw    roll the jaws across the block's narrow side
  read the jaw axis          → side shift that centers the block in the
  │                            asymmetric jaw gap (GRASP_SIDE_OFFSET)
  solve hover  (seeded from nominal — a near miss is the best starting guess)
  solve grasp  (seeded from hover)
  re-align both wrists       the solves drift the roll again (known debt:
  │                          yaw belongs inside the solver)
  solve carry + drop         position-only; the gripper may tilt up high
  │
  pre_check(grasp, hover) ──→ verdict travels with the waypoints
```

## Zoom: one IK solve (ik.solve)

Outer layer — restarts, because DLS is local and one seed can get stuck:

```
solve from the given seed ── converged? ──── yes ──→ return it
         │ no
repeat ≤ IK_RESTARTS:
  fresh random seed → descend ── converged? ─ yes ──→ return it
         │ all failed
return the best attempt anyway (smallest error, converged=False)
                               └─ plan_touch uses these near-misses
```

Inner layer — one DLS descent (`_solve_once`, ≤ IK_MAX_ITERS):

```
┌─→ forward kinematics        where is the gripper for these joint angles?
│   error = target − current  position error, + pointing-down error if asked
│   both under tolerance? ─── yes ──→ done (stops at the FIRST passing
│        │ no                         iterate — the IK_POS_TOL debt)
│   jacobian at this pose     how each joint motion moves the gripper
│   damped least-squares step damping caps the step near singularities
└── apply it, clipped to joint limits
```

## Zoom: the pre-check gauntlet (checks.pre_check)

Four conditions, cheapest first; the first failure is the decline reason
logged for the episode:

```
horizontal distance within [REACH_MIN, REACH_MAX]?   ─ no → "out of reach"
grasp IK converged?                                  ─ no → "no IK solution"
grasp pose free of self-collision? (pose_collides)   ─ no → "self-collision"
  …same two questions for the hover pose…
every joint ≥ JOINT_LIMIT_MARGIN from its hard stop? ─ no → "joint limit"
        │ all pass
        ▼
   attempt the pick
```

`pose_collides` answers one question IK cannot: a converged solution can
still thread the gripper through the robot's own body. It poses the arm on a
scratch copy of the world, parks the block far away, and reports any contact
that isn't the floor.

## Zoom: executing a pick (the motion itself)

```
open jaws
rise to CARRY_Z      travel high — low arcs were clipping blocks en route
over the target      still high
HOVER_Z              directly above the grasp point
descend to grasp     vertical, so the approach can't plow the block
close jaws
lift to HOVER_Z      straight up before leaving
carry to the bin     high again
DROP_Z, open jaws
settle, HOME
```

Every leg is joint-space interpolation, so the Cartesian path between
waypoints is nobody's choice — the travel-high pattern exists to keep those
unplanned arcs away from the block (failure-mode class 3).

## Zoom: a nudge (plan_touch + the drag)

Finding a touch pose — a ladder of heights, lowest first because low contact
pushes rather than topples:

```
for z in NUDGE_TOUCH_HEIGHTS (2, 3, 4, 5 cm):
    best-effort IK to the block at height z (tilt allowed)
    within TOUCH_TOL and no self-collision? ── yes → touch here
all heights fail ──→ genuinely untouchable → DECLINE
```

The drag target (`nudge_target`): aim the block at radius `NUDGE_TARGET_R`
along its own bearing, angle clamped to `NUDGE_ANGLE_CLAMP`, moving at most
`NUDGE_DRAG_MAX` per nudge. Then the motion:

```
close jaws → rise high (CARRY_Z) → above the block (HOVER_Z) →
down to the touch height → drag to the target → lift → HOME
```

Approach from above and lift before leaving — both exist because the
straight-line versions moved the block by accident.

## Zoom: scoring an episode (pick_place.classify)

```
block in the bin?
 ├─ yes ── any nudges on the way? ── no  → picked      (success)
 │                                └─ yes → recovered   (success)
 └─ no ─── ever attempted a pick?  ── yes → missed     (failure)
           only nudged?            ── yes → gave_up    (failure)
           never acted at all?           → declined    (success, trusted —
                                           the spawn band is the audit trail)
```

## Perception and faults

The robot never reads the block's position directly during planning — it reads
`perceive(truth, injections)`. With no faults drawn, that's exact ground truth;
with faults, it's deliberately corrupted (stale-camera bias, per-reading noise).
Physics-side faults (block teleport, low friction) change the world instead.
This is the seam where the sim camera will plug in later: replace ground truth
with a pose estimate, keep everything downstream unchanged.

## Determinism

One seed reproduces a whole run: the main RNG drives spawns, and each episode
gets a child RNG for its internal draws — so `--replay` can skip episodes at
fixed cost and land replayed ones to the millimeter.

## Testing strategy

Pure logic (sampling, classification, gap math, budgets) is unit-tested in
`tests/`. Sim behavior is verified two other ways: throwaway probes that measure
one thing (per-phase block motion, jaw depth, workspace sweeps), and seeded
benchmark runs before/after every change.

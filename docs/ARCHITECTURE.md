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

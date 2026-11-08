# Failure-Mode Catalog

Every distinct error from the project, grouped by *kind*, with counts — so the
biggest recurring classes are visible. Each entry: the instances, the signature
(what it looks like in logs or the viewer), and the detection heuristic that
would have caught it sooner.

## 1. Open-loop planning blind spots — 6 instances (the biggest class)

The planner proved a weaker statement than the one the task needed. Every one of
these passed its own checks while being wrong about the world.

| # | instance | signature |
|---|---|---|
| 1 | wrist roll unconstrained → corner-on jaws | whiffs grow with angle off-center |
| 2 | face-alignment ignored block dimensions → lengthwise grips on tipped blocks | upright 81% vs tipped 23% |
| 3 | IK collision-blind → poses inside the robot's own body | commanded vs achieved pose differs 5–9cm; jaw↔base contacts |
| 4 | single-seed IK local minima → fake "unreachable" zone | implausible hole in a workspace sweep map |
| 5 | grasp height engaged only jaw tips | blocks slip out mid-transport |
| 6 | tolerance stack (15mm IK + faults + servo tracking > jaw forgiveness) | "slight miscalculation" whiffs |

**Detection heuristic:** ask what statement the check actually proves vs what the
task needs ("a pose exists near the target" ≠ "the grasp will hold"). Sweep the
whole input space and *look at the map* — holes and gradients expose wrong models
instantly. Measure the achieved state, never assume the commanded one.

## 2. The robot's own judgment rules wrong — 5 instances

We wrote the decision rules; several encoded a wrong model of progress or risk.

| # | instance | signature |
|---|---|---|
| 1 | pre-check over-cautious (REACH_MAX, IK tol) | wrong_decline: forced attempts succeeded |
| 2 | gap metric radial-only, angle-blind | `gap 0.000 -> 0.000 — stopping` while nudges visibly helped |
| 3 | no pick-futility rule | identical failed pick repeated 4× in one episode |
| 4 | flat try cap cut productive campaigns | gave_up "out of tries" with shrinking gaps |
| 5 | single fixed touch height → false "untouchable" | 43/136 declines rescued at 3–5cm |

**Detection heuristic:** every refusal and every stop rule must be auditable —
either verify it empirically (force-execute, retry higher) or log enough
(band, gap history) to catch it being wrong later. Score refusals as successes
only with an audit trail. When a rule and your eyes disagree, instrument.

## 3. Uncontrolled contact during motion — 3 instances

Joint-space interpolation makes Cartesian paths nobody chose.

| # | instance | signature |
|---|---|---|
| 1 | glide to touch pose plowed through the block | nudges "helping" the wrong direction |
| 2 | transit arcs clipped the block en route (picks and nudges) | block moves during approach phases in probes |
| 3 | return/lift strokes dragging the block | block displacement in lift phase |

**Detection heuristic:** instrument *per motion phase* — measure how far the
block moved during each leg. Anything nonzero outside the deliberate contact
stroke is a path bug. Standing fix: travel high, descend vertically above the
target, lift before leaving.

## 4. Harness and determinism bugs — 3 instances

| # | instance | signature |
|---|---|---|
| 1 | replay RNG stream misalignment risk | replayed spawns drift from the original run |
| 2 | np.float64 leaking into logs/prints | `np.float64(...)` noise (harmless but ugly) |
| 3 | import shadowing (`import mujoco.viewer` inside a method) | NameError |

**Detection heuristic:** replay-to-the-millimeter is a testable invariant — test
it after any change that consumes randomness. Keep per-episode child RNGs.

## 5. Injected faults doing their job — not bugs, but the residual failure load

By design. Current failure rates per fault (n=1000 era): block_move 39%,
camera 36%, noise 27%, friction 18% (friction mostly neutralized by face-aligned
deep grips). The systematic faults (stale camera bias, post-plan world changes)
beat an open-loop pipeline — only sensing/closed-loop work reduces these.

## Where the failures actually were — by volume

Rough share of all failed episodes across the week, by root class:

1. **Grasp geometry (class 1):** dominant until Nov 8 — wrist roll + tipped
   blocks alone explained most `missed`. Now ~2–3/100.
2. **Judgment rules (class 2):** the gave_up pile — most of the 70%-plateau gap.
3. **Motion paths (class 3):** silent multiplier — corrupted nudge campaigns and
   made class-2 rules look worse than they were.
4. **Faults (class 5):** now the largest share of what remains.

## Recurring lessons (the transferable part)

1. **Instrument before fixing.** Every real fix this week started with a probe or
   a log crosstab, not a hypothesis. The two times a fix was attempted from
   pattern-matching, the data flipped the story (the "weak drag" was actually
   transit damage; the "camera" whiffs were mostly geometry).
2. **Eyes find, probes confirm, benchmarks decide.** Four of the six biggest bugs
   were spotted by a human watching replays; each was then confirmed with a
   measurement and accepted only after seeded benchmarks moved.
3. **Fix the model, not the symptom.** The lasting fixes replaced wrong models
   (radial gap → angle-aware; any-face → narrowest-axis) or added missing truth
   (collision check, futility rule). Threshold tweaks were calibration, kept only
   when the data justified the number.
4. **Honest scoring beats high scoring.** Twice we deliberately made the number
   go *down* (verifying declines, auditing untouchables) because an unverified
   success inflates the metric and hides the next bug.
5. **Per-try vs per-episode rates are different claims.** Retries mask a mediocre
   primitive; report both.
6. **Determinism is a superpower — spend it.** Seeded replay turned every fluke
   into a regression test and made "watch the failure" a one-command operation.

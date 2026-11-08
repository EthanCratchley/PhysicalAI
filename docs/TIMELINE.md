# Project Timeline

What we built, what broke, what signal exposed each problem, and how it was fixed —
in order. Nov 3–8, 2026.

Entry format: **Built** / **Broke** (what went wrong) / **Signal** (how we noticed —
this is the part to study) / **Fix** / **Result**.

---

## Phase 0 — Setup (Nov 3)

- **Built:** venv + MuJoCo python package (viewer included, `mjpython` on macOS),
  SO-101 model via sparse Menagerie checkout, `demo.py`, `move_joints.py`.
- **Learned:** joints (degrees of freedom) vs actuators (position servos driving
  them); writing a target angle into `data.ctrl` and letting the servo track it.

## Phase 1 — First pick-and-place (Nov 3–4)

- **Built:** the core pipeline from a written spec (user owned design + key numbers;
  AI wrote plumbing): DLS inverse kinematics with 5 constraints (3D position +
  point-down), waypoint sequence (hover → grasp → lift → carry → drop), `pre_check`
  (reach bounds, IK convergence, joint-limit margin), `success_check` (block inside
  bin, settled), all tunables in `config.py`.
- **Broke:** `import mujoco.viewer` inside a method shadowed the module-level
  `mujoco`. **Signal:** immediate NameError. Classic Python gotcha.
- **Broke:** 5 of 100 attempts "failed" pre-check at spots that looked fine.
  **Signal:** replaying them with the check bypassed — all five succeeded. The
  pre-check was wrong, not the arm: a perfectly vertical gripper at hover height
  is kinematically impossible outside a band, and the tolerance was too strict.
  **Fix:** `HOVER_Z` 0.09→0.08, `IK_POS_TOL` 5mm→15mm (user-tuned).
  **Lesson:** a checker can be the buggy component; verify its verdicts against
  ground truth before trusting its refusals.
- **Broke (1-in-1000):** gripper descended onto the bin rim and closed on air.
  **Signal:** a single log entry — block ended 1cm from spawn, next to the bin;
  reproduced with contact instrumentation showing `bin <-> gripper` contact and a
  stalled shoulder joint. First demonstration that per-run JSON logs + seeded
  replay turn one-in-a-thousand flukes into reproducible test cases.

## Phase 2 — Repo and logging (Nov 4–5)

- **Built:** GitHub repo (menagerie not committed — third-party clone; only our
  `pick_scene.xml` tracked), `.gitignore`, per-run JSON logs (seed, config
  snapshot, every attempt), `--replay` with exact RNG reproduction.
- **Design choice that paid off all week:** logging the full config in every run
  file, so results stayed comparable while numbers were being tuned.

## Phase 3 — Stress spawns and the outcome taxonomy (Nov 5)

- **Built:** weighted spawn bands probing the limits (inner / min_edge / mid /
  max_edge / outer, out to 0.40m), wide angles, spawn collision guard. Pre-check
  declines first recorded as their own outcome, then *force-executed* to verify
  them: `correct_decline` vs `wrong_decline`.
- **Found:** the pre-check was badly over-cautious — 17/100 `wrong_decline`:
  blocks at 0.31–0.37m called "beyond reach (0.3m)" were picked anyway.
  **Signal:** the forced-execution experiment; you can't verify a refusal you
  never test.

## Phase 4 — Fault injection (Nov 5)

- **Built:** four faults imitating real-world problems, rolled independently per
  episode (~15% each, so controls, singles, and natural combos): camera bump
  (systematic perceived-position bias), block_move (world changes after
  planning), sensor noise (fresh per reading), low friction. Logged per episode.
- **Broke (subtle):** replay determinism — skipped episodes must burn exactly the
  same RNG draws as executed ones. **Fix:** per-episode child RNGs seeded from the
  main stream, so a skip burns one integer regardless of episode complexity.
  **Lesson:** any "replay from seed" feature dies quietly the first time code
  consumes a variable number of draws; architect the streams first.

## Phase 5 — The strategy era (Nov 5)

- **Built:** multi-try episodes replacing one-shot attempts. Per try: re-perceive
  → pick if pre-check passes → else nudge the block toward the workspace if it
  can at least be touched → else decline. Progress rule: a second nudge must be
  earned by the first one shrinking the "gap to graspable". Outcomes became
  picked / recovered / missed / gave_up / declined.
- **First run immediately exposed three design errors** (42/100):
  1. Gap metric was radial-only — blocks at a fine radius but extreme angle
     showed gap 0.000, so productive sweeps toward the sector registered as "no
     progress". **Signal:** log lines `gap 0.000 -> 0.000 — stopping` on episodes
     where the nudge was visibly helping. **Fix:** angle-aware gap (distance to
     annulus ∩ sector).
  2. The arm glided straight from HOME to the touch pose, plowing through the
     block en route and shoving it the wrong way. **Fix:** approach from above +
     lift after drag.
  3. Progress threshold (1cm) cut off nudges that gained exactly ~1cm.
     **Signal:** borderline `gap 0.027 -> 0.017` stops. **Fix:** 5mm threshold.
- **Then a fourth, found by reading repeated log lines:** the robot re-ran
  *identical* failing picks — same block position, same plan, same whiff, four
  times. **Signal:** four identical result coordinates in one episode (camera-bias
  episodes made it deterministic). **Fix:** pick futility rule — a failed pick
  earns a retry only if it moved the block ≥5mm. **Lesson:** in a deterministic
  sim, retrying an unchanged plan against an unchanged world is provably useless;
  retries must be justified by *something having changed*.
- Also: `NUDGE_ANGLE_CLAMP` corner sat on the shoulder_pan joint limit — nudges
  aimed blocks at a spot the arm couldn't grasp from. **Fix:** clamp tightened.

## Phase 6 — Scale analysis (Nov 7)

- **Built:** nothing — ran 1000 episodes and read the logs properly. 746/1000.
- **Findings:** per-try pick success only ~50–60% everywhere (episodes reached
  74.6% because retries compensate — per-try and per-episode rates are different
  claims); 136/379 declines were *inside* the nominal annulus, i.e. 13.6% of the
  score rested on unverified refusals; a sharp success dip near the bin;
  recovered=148 proved the nudge strategy carried real weight.
- **Lesson:** small runs (n=40–100) swing ±5 points; convictions need n=1000,
  and band/fault breakdowns beat the headline number every time.

## Phase 7 — The grasp-quality cluster (Nov 7–8)

Four root causes, all surfaced by *watching replays* and confirmed by targeted
probes. The week's biggest single day of gains.

1. **Unconstrained wrist roll.** IK constrained position + point-down only; roll
   was a free by-product, so jaws landed up to ~20° corner-on at wide angles.
   **Signal:** user watched grasps that "should have rolled the wrist"; probe
   measured jaw-vs-face angle across the workspace (2.7° center → 19.5° at ±60°).
   **Fix:** `align_wrist` — post-solve roll to square the jaws.
2. **IK local minima = fake dead zone.** Pre-check declined easy spots straight
   ahead at close range. **Signal:** workspace sweep map showed an implausible
   hole; 20 random seeds recovered *every* "unsolvable" cell. **Fix:** seeded
   restarts in `ik.solve`. **Lesson:** a local solver's failure proves nothing
   about feasibility; retry from new seeds before believing it.
3. **Collision-blind IK.** Converged poses overlapped the robot's own base; the
   executed arm jammed 5–9cm short (user saw the gripper "stuck on the arm").
   **Signal:** probe compared commanded vs achieved pose + contact list.
   **Fix:** `pose_collides` kinematic check in the pre-check and touch test.
4. **Tipped blocks grabbed lengthwise.** The 4×4×6cm block tips constantly; on
   its side it presents 6cm the jaws barely span. Face-alignment treated all
   faces as equal. **Signal:** user noticed "lengthwise" grabs; log crosstab:
   upright picks 81%, tipped 23% — half of all picks were tipped. **Fix:**
   `grasp_yaw` targets the narrowest horizontal axis (mod-180 alignment).
   Tipped success: 23% → 90%.

Plus, same phase:
- **Touch-height audit:** 43/136 suspicious "untouchable" declines were artifacts
  of a single fixed touch height; `plan_touch` climbs 2→5cm. The other 93 were
  honest (shoulder folds, bin blockage).
- **Transit clipping** (user: "it nudges the block on its way over"): joint-space
  glides swept low arcs that knocked blocks to worse spots before the action
  began. **Fix:** travel high, descend only above the target. Probe: transit
  block motion → 0.0cm. 100-run: 75 → 79.
- **Shallow grip** (user: "doesn't go very deep into the block"): measured — jaw
  tips engaged only the top 2.5cm of the 6.2cm block. **Fix:**
  `GRASP_Z_OFFSET` −0.012. missed: ~13/100 → 2–3/100.
- **Earned try budget** (user: "it hit the cap even though it would have picked
  it up next try"): productive nudges refund their try, hard ceiling 8.
  recovered up, gave_up down on every seed.
- **Tooling:** `--replay-failures` (replay a log's failures without hand-copying
  seeds), `--feasible` (winnable-spawns-only runs).

## Score progression (full stress distribution, /100 unless noted)

| when | score | what changed |
|---|---|---|
| Nov 5, first strategy run | 42 | episodes + nudging landed, raw |
| Nov 5, after gap/approach/futility fixes | ~60–75 | strategy debugged |
| Nov 7, 1000-run | 74.6 | (n=1000 confidence) |
| Nov 7, after touch-height audit | ~72–75 | *more honest*: trusted declines became measured attempts |
| Nov 8, after wrist/tipped/collision fixes | 75 | grasp quality solved, misses collapse |
| Nov 8, transit fix | 79 | en-route clipping eliminated |
| Nov 8, deep grip + earned budget | 83–86 | — |
| Nov 8, `--feasible` (fair distribution) | **94–96** | the envelope, measured |

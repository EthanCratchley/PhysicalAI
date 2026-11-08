# Decision Log

Deliberate choices and why — not mistakes (those live in FAILURE-MODES.md).
Format: decision / why / when to revisit.

1. **Declines count as successes.** Refusing an impossible block is correct
   behavior, so `declined` scores as a win — on trust, with the spawn band
   logged as the audit trail. We audited once (43/136 were too conservative)
   and fixed the cause. *Revisit:* whenever declines exceed ~a third of a run.

2. **Pick first, nudge only as fallback.** The robot always plans a pick;
   nudging happens only after the pre-check refuses. Confident behavior, simple
   to reason about. *Revisit:* the risk-graded idea (nudge marginal-but-legal
   picks toward center first) is the known improvement.

3. **Verification by force-execution was retired.** Early on, declines were
   executed anyway to check them (`correct_decline`/`wrong_decline`). Replaced
   by strategic nudging — more useful behavior, but declines became unverified
   again. Conscious trade.

4. **Progress earns patience.** Both self-honesty rules follow one principle:
   an action justifies repetition only by changing the world (pick futility) or
   measurably helping (nudge progress, earned budget). Flat caps and blind
   retries both lost to it.

5. **Faults roll independently.** Each fault has its own probability instead of
   a pick-one table — controls, singles, and realistic combos emerge naturally.

6. **Ground-truth perception, corrupted on purpose.** No camera yet; the robot
   reads sim truth through `perceive()`, which faults can distort. Keeps the
   decision layer testable against known error profiles. *Next:* replace with
   the overhead-camera estimator (see IDEAS).

7. **The default distribution is adversarial.** ~half the spawns are at or past
   the limits, on purpose — it finds bugs. `--feasible` exists so the fair
   number is also measurable. Report both; they answer different questions.

8. **All tunables in one file, snapshotted into every log.** Runs stay
   comparable while numbers change; any log is self-describing.

9. **Random seed by default, but always printed and logged.** Variety without
   losing reproducibility; replay needs the seed, so the log keeps it.

10. **Per-episode child RNGs.** Replay can skip episodes at fixed cost no matter
    how many draws an episode consumes internally.

11. **Menagerie is not committed.** Third-party model repo, re-fetchable; only
    our `pick_scene.xml` is tracked, and the README says how to restore the rest.

12. **Hand-coded strategy before learning.** Every rule is inspectable and every
    failure attributable — the right debugging substrate. Learning experiments
    come later, on top of the logs this produces.

13. **Tests for pure logic; probes and seeded benchmarks for sim behavior.**
    Unit tests pin math and rules; motion/contact changes are judged by
    one-off measurement probes plus before/after runs on fixed seeds, because
    sim dynamics in a unit test would be slow and brittle.

14. **Docs live in the repo.** Timeline, failure modes, architecture, glossary,
    decisions, ideas — versioned next to the code they describe.

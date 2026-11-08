"""Tests for the multi-try strategy: gap metric, nudge planning, episode classification."""

import json

import numpy as np

import config as cfg
from pick_place import classify, dist_to_graspable, load_failures, nudge_target, pick_is_futile


def polar(r, deg):
    a = np.radians(deg)
    return np.array([r * np.cos(a), r * np.sin(a), 0.031])


def test_gap_zero_inside_annulus():
    assert dist_to_graspable(polar(0.2, 10)) == 0


def test_gap_when_too_close():
    assert np.isclose(dist_to_graspable(polar(0.05, 10)), cfg.REACH_MIN - 0.05)


def test_gap_when_too_far():
    assert np.isclose(dist_to_graspable(polar(0.38, 10)), 0.38 - cfg.REACH_MAX)


def test_gap_positive_outside_angle_sector():
    """A block at a comfortable radius but an extreme angle is not graspable-at-ease:
    nudges that sweep it toward the sector must register as progress."""
    assert dist_to_graspable(polar(0.2, 100)) > 0


def test_gap_shrinks_as_angle_improves():
    assert dist_to_graspable(polar(0.2, 80)) < dist_to_graspable(polar(0.2, 100))


def test_nudge_target_always_reduces_gap():
    rng = np.random.default_rng(0)
    for _ in range(200):
        p = polar(rng.uniform(0.05, 0.4), rng.uniform(-180, 180))
        if dist_to_graspable(p) == 0:
            continue
        assert dist_to_graspable(nudge_target(p)) < dist_to_graspable(p)


def test_nudge_target_moves_toward_comfortable_radius():
    p = polar(0.37, 10)
    t = nudge_target(p)
    goal = polar(cfg.NUDGE_TARGET_R, 10)[:2]
    assert np.linalg.norm(t[:2] - goal) < np.linalg.norm(p[:2] - goal)
    assert t[2] == cfg.NUDGE_Z


def test_nudge_target_caps_drag_length():
    p = polar(0.39, 0)
    assert np.linalg.norm(nudge_target(p)[:2] - p[:2]) <= cfg.NUDGE_DRAG_MAX + 1e-9


def test_nudge_target_reaches_goal_when_close():
    p = polar(cfg.NUDGE_TARGET_R - 0.02, 10)
    assert np.allclose(nudge_target(p)[:2], polar(cfg.NUDGE_TARGET_R, 10)[:2])


def test_nudge_target_sweeps_bad_angle_toward_clamp():
    p = polar(cfg.NUDGE_TARGET_R, 100)  # radius fine, angle outside the clamp
    assert not np.allclose(nudge_target(p)[:2], p[:2])


def test_classify_straight_pick():
    assert classify(["pick"], True) == ("picked", True)


def test_classify_retried_pick_is_still_picked():
    assert classify(["pick", "pick"], True) == ("picked", True)


def test_classify_recovered_after_nudge():
    assert classify(["nudge", "pick"], True) == ("recovered", True)


def test_classify_missed():
    assert classify(["pick"], False) == ("missed", False)
    assert classify(["nudge", "pick"], False) == ("missed", False)


def test_classify_gave_up():
    assert classify(["nudge"], False) == ("gave_up", False)
    assert classify(["nudge", "none"], False) == ("gave_up", False)


def test_classify_declined_without_acting():
    assert classify(["none"], False) == ("declined", True)


def test_load_failures_returns_seed_and_failed_episodes(tmp_path):
    log = {"seed": 42, "episodes": [
        {"episode": 1, "success": True},
        {"episode": 2, "success": False},
        {"episode": 5, "success": False},
    ]}
    path = tmp_path / "run_test.json"
    path.write_text(json.dumps(log))
    assert load_failures(str(path)) == (42, [2, 5])


def test_first_pick_is_never_futile():
    assert not pick_is_futile(polar(0.2, 10), None)


def test_pick_futile_when_block_static_since_last_pick():
    """Regression: the robot used to burn all its tries repeating an identical
    whiffed pick against a block that never moved."""
    p = polar(0.2, 10)
    assert pick_is_futile(p + [0.001, 0.001, 0], p)


def test_pick_not_futile_when_block_moved():
    p = polar(0.2, 10)
    assert not pick_is_futile(p + [0.02, 0, 0], p)

"""Tests for fault-injection draws and the perception corruption they cause."""

import json

import numpy as np

import config as cfg
from pick_place import draw_injections, perceive


def test_draws_stay_within_configured_ranges():
    rng = np.random.default_rng(0)
    for _ in range(500):
        for inj in draw_injections(rng):
            spec = cfg.INJECTIONS[inj["type"]]
            if inj["type"] == "camera":
                assert spec["shift"][0] <= inj["shift"] <= spec["shift"][1]
                assert spec["tilt_deg"][0] <= abs(inj["tilt_deg"]) <= spec["tilt_deg"][1]
            if inj["type"] == "block_move":
                assert spec["dist"][0] <= inj["dist"] <= spec["dist"][1]
            if inj["type"] == "noise":
                assert all(abs(v) <= spec["per_axis"] for v in inj["offset"])
            if inj["type"] == "friction":
                assert spec["scale"][0] <= inj["scale"] <= spec["scale"][1]


def test_draws_are_independent():
    """Controls, every single type, and combined cases must all occur."""
    rng = np.random.default_rng(0)
    draws = [draw_injections(rng) for _ in range(2000)]
    assert any(len(d) == 0 for d in draws)
    assert any(len(d) >= 2 for d in draws)
    seen = {inj["type"] for d in draws for inj in d}
    assert seen == set(cfg.INJECTIONS)


def test_draw_rates_match_config():
    rng = np.random.default_rng(1)
    n = 4000
    hits = {k: 0 for k in cfg.INJECTIONS}
    for _ in range(n):
        for inj in draw_injections(rng):
            hits[inj["type"]] += 1
    for kind, spec in cfg.INJECTIONS.items():
        assert abs(hits[kind] / n - spec["p"]) < 0.03


def test_draws_are_json_serializable():
    """Guards against numpy scalar types (e.g. np.int64, which json rejects)
    creeping into the drawn magnitudes."""
    rng = np.random.default_rng(0)
    for _ in range(200):
        for inj in draw_injections(rng):
            json.dumps(inj)
            assert all(type(v) in (str, float, int, list) for v in inj.values())


def test_perceive_without_injections_is_ground_truth():
    p = np.array([0.2, 0.1, 0.031])
    assert np.array_equal(perceive(p, []), p)


def test_perceive_camera_tilt_preserves_radius():
    p = np.array([0.2, 0.1, 0.031])
    out = perceive(p, [{"type": "camera", "shift": 0.0, "tilt_deg": 3.0, "dir_deg": 0.0}])
    assert np.isclose(np.linalg.norm(out[:2]), np.linalg.norm(p[:2]))
    assert out[2] == p[2]
    assert not np.allclose(out[:2], p[:2])


def test_perceive_camera_shift_moves_by_exactly_shift():
    p = np.array([0.2, 0.1, 0.031])
    out = perceive(p, [{"type": "camera", "shift": 0.02, "tilt_deg": 0.0, "dir_deg": 90.0}])
    assert np.isclose(np.linalg.norm(out - p), 0.02)


def test_perceive_applies_noise_offset():
    p = np.array([0.2, 0.1, 0.031])
    out = perceive(p, [{"type": "noise", "offset": [0.003, -0.002, 0.001]}])
    assert np.allclose(out - p, [0.003, -0.002, 0.001])


def test_perceive_ignores_physics_only_injections():
    """block_move and friction act on the world, not on perception."""
    p = np.array([0.2, 0.1, 0.031])
    injections = [{"type": "block_move", "dist": 0.05, "dir_deg": 0.0},
                  {"type": "friction", "scale": 0.3}]
    assert np.array_equal(perceive(p, injections), p)

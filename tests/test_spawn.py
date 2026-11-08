"""Tests for the spawn sampler and outcome classifier."""

import numpy as np

import config as cfg
from pick_place import sample_spawn


def band_ranges():
    return {name: (lo, hi) for name, _, lo, hi in cfg.SPAWN_BANDS}


def test_sample_spawn_stays_within_its_band():
    rng = np.random.default_rng(0)
    ranges = band_ranges()
    for _ in range(300):
        band, r, angle = sample_spawn(rng)
        lo, hi = ranges[band]
        assert lo <= r <= hi
        assert cfg.SPAWN_ANGLE[0] <= angle <= cfg.SPAWN_ANGLE[1]


def test_sample_spawn_hits_every_band():
    rng = np.random.default_rng(0)
    seen = {sample_spawn(rng)[0] for _ in range(500)}
    assert seen == set(band_ranges())


def test_bands_cover_both_sides_of_each_reach_limit():
    radii = [r for _, _, lo, hi in cfg.SPAWN_BANDS for r in (lo, hi)]
    assert min(radii) < cfg.REACH_MIN < max(radii)
    assert min(radii) < cfg.REACH_MAX < max(radii)

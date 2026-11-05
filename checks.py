"""The pre-check and success check.

These are the two judgment calls of the pipeline: should we even attempt this
grasp, and did it actually work? Working versions to study and rewrite.
"""

import numpy as np

import config as cfg
from ik import ARM_JOINTS


def pre_check(model, target_pos, grasp_ik, hover_ik):
    """Decide whether the grasp should be attempted. Returns (ok, reason).

    Three ways a target can be bad:
    1. Out of reach: too far from (or too close to) the base in the horizontal plane.
    2. No IK solution: the solver couldn't find joint angles for the grasp or
       hover pose. Near-singular or infeasible poses show up here.
    3. Too close to joint limits: a solution exists, but some joint would sit
       within JOINT_LIMIT_MARGIN of its hard stop, leaving no room to correct.
    """
    r = np.linalg.norm(target_pos[:2])
    if r > cfg.REACH_MAX:
        return False, f"target {r:.3f}m from base, beyond reach ({cfg.REACH_MAX}m)"
    if r < cfg.REACH_MIN:
        return False, f"target {r:.3f}m from base, inside min reach ({cfg.REACH_MIN}m)"

    for name, res in [("grasp", grasp_ik), ("hover", hover_ik)]:
        if not res.converged:
            return False, f"no IK solution for {name} pose (pos err {res.pos_err:.3f}m)"

    lo, hi = model.jnt_range[[model.joint(n).id for n in ARM_JOINTS]].T
    margin = np.minimum(grasp_ik.qpos - lo, hi - grasp_ik.qpos)
    if margin.min() < cfg.JOINT_LIMIT_MARGIN:
        bad = ARM_JOINTS[int(margin.argmin())]
        return False, f"{bad} within {margin.min():.3f} rad of its limit"

    return True, "ok"


def success_check(model, data):
    """Did the block end up in the bin? Returns (ok, reason).

    Success = the block's center is horizontally inside the bin opening, its
    height says it's sitting in the bin (not on a wall or the floor nearby),
    and it has stopped moving (not mid-bounce).
    """
    block = data.qpos[model.joint("block_free").qposadr[0]:][:3]
    speed = np.linalg.norm(data.qvel[model.joint("block_free").dofadr[0]:][:3])
    rel = block - model.body("bin").pos

    if abs(rel[0]) > cfg.BIN_INNER or abs(rel[1]) > cfg.BIN_INNER:
        return False, f"block at ({rel[0]:+.3f}, {rel[1]:+.3f}) relative to bin, outside opening"
    if rel[2] > cfg.BIN_TOP:
        return False, f"block at height {rel[2]:.3f}m, above the bin walls"
    if speed > cfg.MAX_REST_SPEED:
        return False, f"block still moving at {speed:.3f} m/s"

    return True, "block in bin"

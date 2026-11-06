"""Inverse kinematics: find arm joint angles that put the gripper at a target.

Damped least squares (DLS): repeatedly compute the error between where the
gripper is and where we want it, use the jacobian (how each joint affects the
gripper) to find a small joint change that shrinks the error, apply it, repeat.
The damping keeps the step sane near singularities, where the plain
least-squares solution blows up.

We solve for 5 constraints: 3D position of the gripperframe site, plus the
site's x-axis (the direction the gripper points) aligned straight down for a
top-down grasp. The 5 arm joints make this exactly determined.
"""

from dataclasses import dataclass

import mujoco
import numpy as np

import config as cfg

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
DOWN = np.array([0.0, 0.0, -1.0])


@dataclass
class IKResult:
    qpos: np.ndarray      # the 5 arm joint angles found
    pos_err: float        # final distance from target (m)
    ori_err: float        # final pointing error (rad)
    converged: bool


def site_frame(model, arm_qpos):
    """Forward kinematics: where is the gripperframe site (pos, rotation) for
    these arm angles? Purely kinematic, on a scratch MjData."""
    d = mujoco.MjData(model)
    d.qpos[[model.joint(n).qposadr[0] for n in ARM_JOINTS]] = arm_qpos
    mujoco.mj_forward(model, d)
    site = model.site("gripperframe").id
    return d.site_xpos[site].copy(), d.site_xmat[site].reshape(3, 3).copy()


def align_wrist(model, qpos, face_yaw=0.0):
    """Rotate wrist_roll so the jaw axis lies square to a block face (faces
    repeat every 90 deg). The solver only constrains where the gripper points;
    roll is a free by-product, so without this the jaws land corner-on at wide
    angles. Rolling about the (vertical) pointing axis leaves position alone."""
    def jaw_yaw(q):
        _, R = site_frame(model, q)
        return np.arctan2(R[1, 2], R[0, 2])

    q = qpos.copy()
    lo, hi = model.jnt_range[model.joint("wrist_roll").id]
    for _ in range(3):
        off = (jaw_yaw(q) - face_yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4
        if abs(off) < 0.01:
            break
        probe = q.copy()
        probe[4] = np.clip(q[4] + 0.05, lo, hi)
        dyaw = (jaw_yaw(probe) - jaw_yaw(q) + np.pi) % (2 * np.pi) - np.pi
        if abs(dyaw) < 1e-6:
            break
        q[4] = np.clip(q[4] - off * (probe[4] - q[4]) / dyaw, lo, hi)
    return q


def solve(model, start_qpos, target_pos, point_down=True):
    """Solve for the arm pose putting gripperframe at target_pos.

    DLS is a local method and a single seed can get stuck in a local minimum
    at a perfectly reachable target, so a failed solve is retried from a few
    deterministic random seeds before giving up; the best attempt is returned.

    point_down=True also requires the gripper to point straight down (for
    grasping). point_down=False solves position only, letting the gripper
    tilt — the arm is too short to hold a vertical gripper high up, so
    transport/drop waypoints use this mode.
    """
    best = _solve_once(model, start_qpos, target_pos, point_down)
    if best.converged:
        return best
    lo, hi = model.jnt_range[[model.joint(n).id for n in ARM_JOINTS]].T
    rng = np.random.default_rng(0)  # fixed seeds: same target, same answer
    for _ in range(cfg.IK_RESTARTS):
        res = _solve_once(model, rng.uniform(lo, hi), target_pos, point_down)
        if res.converged:
            return res
        if res.pos_err < best.pos_err:
            best = res
    return best


def _solve_once(model, start_qpos, target_pos, point_down):
    """One DLS descent from a single seed. Runs on a scratch MjData,
    never touches the live sim."""
    site = model.site("gripperframe").id
    adr = [model.joint(n).qposadr[0] for n in ARM_JOINTS]
    dof = [model.joint(n).dofadr[0] for n in ARM_JOINTS]
    lo, hi = model.jnt_range[[model.joint(n).id for n in ARM_JOINTS]].T

    d = mujoco.MjData(model)
    d.qpos[adr] = start_qpos

    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))

    for _ in range(cfg.IK_MAX_ITERS):
        mujoco.mj_forward(model, d)
        e_pos = target_pos - d.site_xpos[site]
        xaxis = d.site_xmat[site].reshape(3, 3)[:, 0]
        # rotation that would swing xaxis toward DOWN: axis = xaxis x DOWN, magnitude = sin(angle)
        e_ori = np.cross(xaxis, DOWN) if point_down else np.zeros(3)

        if np.linalg.norm(e_pos) < cfg.IK_POS_TOL and np.linalg.norm(e_ori) < cfg.IK_ORI_TOL:
            return IKResult(d.qpos[adr].copy(), np.linalg.norm(e_pos), np.linalg.norm(e_ori), True)

        mujoco.mj_jacSite(model, d, jacp, jacr, site)
        if point_down:
            J = np.vstack([jacp[:, dof], cfg.IK_ORI_WEIGHT * jacr[:, dof]])
            err = np.concatenate([e_pos, cfg.IK_ORI_WEIGHT * e_ori])
        else:
            J = jacp[:, dof]
            err = e_pos
        # DLS step: like least squares, but the damping term caps how big dq can get
        dq = J.T @ np.linalg.solve(J @ J.T + cfg.IK_DAMPING**2 * np.eye(len(err)), err)
        d.qpos[adr] = np.clip(d.qpos[adr] + dq, lo, hi)

    return IKResult(d.qpos[adr].copy(), np.linalg.norm(e_pos), np.linalg.norm(e_ori), False)

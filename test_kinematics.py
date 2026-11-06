"""Tests for the three grasp-quality fixes: wrist alignment, IK restarts,
and collision-aware feasibility."""

import mujoco
import numpy as np

import checks
import config as cfg
import ik
import pick_place as pp

MODEL = mujoco.MjModel.from_xml_path("menagerie/robotstudio_so101/pick_scene.xml")


class S:
    pass


SIM = S()
SIM.model = MODEL


def polar(r, deg):
    a = np.radians(deg)
    return np.array([r * np.cos(a), r * np.sin(a), 0.031])


def jaw_face_offset(qpos, face_yaw=0.0):
    _, R = ik.site_frame(MODEL, qpos)
    yaw = np.arctan2(R[1, 2], R[0, 2]) - face_yaw
    return abs((yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4)


def test_close_range_ik_converges():
    """Regression: the HOME-seeded solve chain hit local minima straight ahead
    at close range, declining easily reachable blocks."""
    for a in (-15, 0, 15):
        p = polar(0.13, a)
        nominal = ik.solve(MODEL, pp.HOME, p)
        hover = ik.solve(MODEL, nominal.qpos, [p[0], p[1], cfg.HOVER_Z])
        assert hover.converged, f"hover fails at angle {a}"


def test_align_wrist_squares_jaws_without_moving_gripper():
    p = polar(0.2, -60)
    g = ik.solve(MODEL, pp.HOME, p)
    q = ik.align_wrist(MODEL, g.qpos)
    assert jaw_face_offset(q) < 0.05
    pos0, _ = ik.site_frame(MODEL, g.qpos)
    pos1, _ = ik.site_frame(MODEL, q)
    assert np.linalg.norm(pos1 - pos0) < 0.01


def test_align_wrist_follows_block_yaw():
    p = polar(0.2, 30)
    g = ik.solve(MODEL, pp.HOME, p)
    q = ik.align_wrist(MODEL, g.qpos, face_yaw=np.radians(30))
    assert jaw_face_offset(q, face_yaw=np.radians(30)) < 0.05


def test_plan_pick_grasps_are_face_aligned():
    """Regression: jaws approached the cube up to ~20 deg corner-on at wide
    angles, causing whiffs and weak pinch grasps."""
    for a in (-60, -30, 30, 60):
        hover, grasp, carry, drop, ok, why = pp.plan_pick(SIM, polar(0.23, a))
        assert ok, f"declined at angle {a}: {why}"
        assert jaw_face_offset(grasp.qpos) < 0.06, f"misaligned at angle {a}"


def test_home_pose_is_collision_free():
    assert not checks.pose_collides(MODEL, pp.HOME)


def test_inner_touch_poses_collide():
    """Regression: IK returned touch poses overlapping the robot's own base;
    executing them jammed the gripper against the arm."""
    hits = 0
    for xy in [(0.05, -0.05), (0.03, 0.06), (0.06, 0.02)]:
        above = ik.solve(MODEL, pp.HOME, [xy[0], xy[1], cfg.HOVER_Z], point_down=False)
        touch = ik.solve(MODEL, above.qpos, [xy[0], xy[1], cfg.NUDGE_Z], point_down=False)
        hits += checks.pose_collides(MODEL, touch.qpos)
    assert hits >= 1, "no inner touch pose flagged, but the sim probe showed jamming"

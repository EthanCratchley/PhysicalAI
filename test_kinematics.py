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


def test_grasp_yaw_upright_block_uses_footprint_axis():
    # upright block rotated 30 deg about z; square 4x4cm footprint, either
    # horizontal axis is fine but it must be one of them (30 or 120 deg)
    a = np.radians(15)  # half-angle: quat for 30 deg about z
    q = pp.grasp_yaw([np.cos(a), 0, 0, np.sin(a)], [0.02, 0.02, 0.03])
    assert min(abs((np.degrees(q) - d + 45) % 90 - 45) for d in (30, 120)) < 1


def test_grasp_yaw_tipped_block_targets_narrow_dimension():
    """Regression: tipped blocks (6cm length horizontal) were grabbed
    lengthwise half the time — 23% success vs 81% upright. The jaws must
    close across a 4cm axis, never the 6cm one."""
    a = np.radians(45)  # quat for 90 deg about x: block lies on its side
    yaw = pp.grasp_yaw([np.cos(a), np.sin(a), 0, 0], [0.02, 0.02, 0.03])
    # body x (half 0.02) stays along world x; body z (half 0.03) is horizontal
    # along world y — the jaw axis must align with x (yaw ~ 0 mod 180)
    assert abs((np.degrees(yaw) + 90) % 180 - 90) < 1


def test_align_wrist_distinguishes_opposite_axes():
    """Alignment must hit the requested axis (mod 180), not any face (mod 90)."""
    p = polar(0.2, 20)
    g = ik.solve(MODEL, pp.HOME, p)
    q0 = ik.align_wrist(MODEL, g.qpos, face_yaw=0.0)
    q90 = ik.align_wrist(MODEL, g.qpos, face_yaw=np.pi / 2)
    def yaw(q):
        _, R = ik.site_frame(MODEL, q)
        return np.arctan2(R[1, 2], R[0, 2])
    off0 = abs((yaw(q0) + np.pi / 2) % np.pi - np.pi / 2)
    off90 = abs((yaw(q90) - np.pi / 2 + np.pi / 2) % np.pi - np.pi / 2)
    assert off0 < 0.06 and off90 < 0.06


def test_home_pose_is_collision_free():
    assert not checks.pose_collides(MODEL, pp.HOME)


def test_plan_touch_climbs_when_low_pose_is_blocked():
    """Regression: a fixed touch height (NUDGE_Z) declared blocks untouchable
    when a slightly higher touch point was reachable and collision-free.
    Positions taken from real declines in a 1000-episode run."""
    for xy in [(0.024, 0.1207), (-0.0733, -0.2953), (0.2031, -0.2617)]:
        plan = pp.plan_touch(MODEL, np.array([xy[0], xy[1], 0.031]))
        assert plan is not None, f"no touch plan at {xy}"
        touch, z = plan
        assert touch.pos_err <= cfg.TOUCH_TOL
        assert not checks.pose_collides(MODEL, touch.qpos)


def test_plan_touch_returns_none_when_all_heights_blocked():
    """These positions collide at every candidate height (gripper folds into
    the shoulder); the honest answer is None."""
    for xy in [(0.0767, -0.0967), (0.1206, -0.0281)]:
        assert pp.plan_touch(MODEL, np.array([xy[0], xy[1], 0.031])) is None, f"{xy}"


def test_inner_touch_poses_collide():
    """Regression: IK returned touch poses overlapping the robot's own base;
    executing them jammed the gripper against the arm."""
    hits = 0
    for xy in [(0.05, -0.05), (0.03, 0.06), (0.06, 0.02)]:
        above = ik.solve(MODEL, pp.HOME, [xy[0], xy[1], cfg.HOVER_Z], point_down=False)
        touch = ik.solve(MODEL, above.qpos, [xy[0], xy[1], cfg.NUDGE_Z], point_down=False)
        hits += checks.pose_collides(MODEL, touch.qpos)
    assert hits >= 1, "no inner touch pose flagged, but the sim probe showed jamming"

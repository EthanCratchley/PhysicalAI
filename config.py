"""All tunable numbers in one place. The code reads these; you edit them."""

# --- spawn region: polar coordinates around the robot base, 0 deg = straight ahead (+x) ---
# Weighted radial bands chosen to probe the robot's limits: well inside min
# reach, straddling each reach boundary (REACH_MIN/REACH_MAX below), the
# comfortable middle, and well beyond the arm. (name, weight, r_lo, r_hi).
SPAWN_BANDS = (
    ("inner",    1, 0.05, 0.10),
    ("min_edge", 2, 0.10, 0.14),
    ("mid",      2, 0.14, 0.28),
    ("max_edge", 2, 0.28, 0.32),
    ("outer",    1, 0.32, 0.40),
)
SPAWN_ANGLE = (-110.0, 110.0)  # degrees; wide enough to run into pan joint limits
SPAWN_CLEAR_OF_BIN = 0.11     # resample if the block lands this close to the bin center (m)

# --- fault injection ---
# Each attempt rolls every injection independently with probability p, so most
# attempts are clean controls or single faults, with an occasional combination.
# camera: perceived block position rotated about the base and shifted (stale
#   extrinsics after a bump) — physics keeps the true position.
# block_move: the real block teleports after planning, before the grasp.
# noise: per-axis error on the perceived position.
# friction: block contact friction scaled down for the attempt.
INJECTIONS = {
    "camera":     {"p": 0.15, "shift": (0.01, 0.03), "tilt_deg": (1.0, 3.0)},
    "block_move": {"p": 0.15, "dist": (0.02, 0.05)},
    "noise":      {"p": 0.15, "per_axis": 0.005},
    "friction":   {"p": 0.15, "scale": (0.3, 0.5)},
}

# --- multi-try strategy ---
MAX_TRIES = 4                 # hard cap on actions (picks + nudges) per block
TOUCH_TOL = 0.04              # block counts as touchable if best-effort IK gets within this (m)
NUDGE_MIN_PROGRESS = 0.005    # a nudge must shrink the gap to graspable by this much to earn another (m)
NUDGE_TARGET_R = 0.21         # drag blocks toward this radius: middle of the comfortable workspace (m)
NUDGE_ANGLE_CLAMP = (-35.0, 45.0)  # and into this angular sector (deg); narrower than the spawn
                                   # sector so nudge targets stay clear of the shoulder_pan limit
PICK_MIN_MOVE = 0.005         # a failed pick earns a retry only if it moved the block this much (m)
NUDGE_DRAG_MAX = 0.10         # longest drag per nudge (m)
NUDGE_Z = 0.02                # preferred gripper height while touching/dragging (m)
NUDGE_TOUCH_HEIGHTS = (0.02, 0.03, 0.04, 0.05)  # tried lowest-first: a third of "untouchable"
                              # declines were artifacts of only ever trying the lowest

# --- pre-check: reach limits ---
REACH_MIN = 0.12              # targets closer than this are inside the robot's own footprint
REACH_MAX = 0.30              # targets farther than this are beyond the arm

# --- inverse kinematics (damped least squares) ---
IK_POS_TOL = 0.015            # accept solution when gripper is within this of target (m)
IK_ORI_TOL = 0.08             # and pointing error is below this (rad, ~5 deg)
IK_DAMPING = 0.1              # lambda: higher = more stable near singularities, slower convergence
IK_MAX_ITERS = 300
IK_RESTARTS = 5               # extra random-seed attempts when a solve fails (DLS local minima)
IK_ORI_WEIGHT = 0.5           # how much pointing-down matters vs position during solving

JOINT_LIMIT_MARGIN = 0.05     # pre-check fails if any IK joint angle is within this of a hard limit (rad)

# --- grasp geometry ---
# the arm is short: it can only hold the gripper vertical below ~0.09m height,
# so the hover is low and the drop waypoint allows a tilted gripper
HOVER_Z = 0.08                # absolute gripper height when hovering above the block (m)
GRASP_Z_OFFSET = 0.0          # grasp target height relative to block center (m)
CARRY_Z = 0.14                # lift to this height (tilt allowed) before swinging to the bin,
                              # so the hanging block clears the bin walls
DROP_Z = 0.14                 # absolute gripper height above bin center when releasing (m)

# --- gripper ctrl values (rad, actuator range is -0.17..1.74) ---
GRIPPER_OPEN = 1.0
GRIPPER_CLOSED = 0.3

# the jaws' gap is not centered on the gripperframe site: the fixed jaw sits
# ~2.1cm to one side (along the site z-axis) and the open moving jaw ~6.6cm to
# the other. Shift the grasp target this far along the jaw axis so the block
# ends up centered between the jaws instead of under the fixed jaw.
GRASP_SIDE_OFFSET = 0.022

# --- motion timing ---
MOVE_SECONDS = 1.2            # duration of each waypoint-to-waypoint move
SETTLE_SECONDS = 0.6          # pause after closing/opening the gripper

# --- success check ---
BIN_INNER = 0.055             # half-width of the bin's inner opening (m)
BIN_TOP = 0.055               # height of the bin walls (m)
MAX_REST_SPEED = 0.05         # block counts as settled below this speed (m/s)

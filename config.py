"""All tunable numbers in one place. The code reads these; you edit them."""

# --- spawn region: polar coordinates around the robot base, 0 deg = straight ahead (+x) ---
SPAWN_R = (0.18, 0.26)        # meters from base
SPAWN_ANGLE = (-40.0, 60.0)   # degrees
SPAWN_CLEAR_OF_BIN = 0.11     # resample if the block lands this close to the bin center (m)

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

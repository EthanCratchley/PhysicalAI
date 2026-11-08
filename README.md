# PhysicalAI

MuJoCo experiments with the SO-101 arm: joint control, inverse kinematics, and a pick-and-place task.

## Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install mujoco numpy
```

### MuJoCo Menagerie

The robot model comes from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie), which is not committed here (only our custom `pick_scene.xml` is tracked). To fetch the `robotstudio_so101` model:

```bash
git clone --depth 1 --filter=blob:none https://github.com/google-deepmind/mujoco_menagerie.git /tmp/menagerie
mkdir -p menagerie
cp -r /tmp/menagerie/robotstudio_so101 menagerie/
```

## Scripts

- `temp/move_joints.py` — basic joint control demo
- `ik.py` — inverse kinematics for the arm
- `pick_place.py` — pick-and-place task using `pick_scene.xml`
- `checks.py`, `config.py` — supporting checks and configuration

## Commands

Run from the project root. The viewer needs `mjpython` (macOS); headless runs use plain `python`.

```bash
# watch pick-and-place in the viewer (3 attempts)
.venv/bin/mjpython pick_place.py

# headless batch run (fast, writes a log to logs/)
.venv/bin/python pick_place.py --headless --attempts 100

# different random spawns
.venv/bin/python pick_place.py --headless --attempts 100 --seed 2

# only feasible spawns (no at/beyond-reach stress tests)
.venv/bin/python pick_place.py --headless --attempts 100 --feasible

# replay every failure from the newest run log, in the viewer
.venv/bin/mjpython pick_place.py --replay-failures

# same, from a specific run log
.venv/bin/mjpython pick_place.py --replay-failures logs/run_20261107_230138.json

# replay specific episodes from an earlier run (needs that run's seed)
.venv/bin/mjpython pick_place.py --seed 79935674 --replay 11,14,32

# joint sweep demo
.venv/bin/mjpython temp/move_joints.py

# open an empty interactive viewer (drag any model XML in)
.venv/bin/python -m mujoco.viewer
```

Each headless run writes `logs/run_<timestamp>.json` with the failures (attempt number, block position, reason) and the exact config values used.

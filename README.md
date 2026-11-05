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

- `move_joints.py` — basic joint control demo
- `ik.py` — inverse kinematics for the arm
- `pick_place.py` — pick-and-place task using `pick_scene.xml`
- `checks.py`, `config.py` — supporting checks and configuration

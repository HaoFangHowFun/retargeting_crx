# Analyzing Key Objectives in Human-to-Robot Retargeting for Dexterous Manipulation

<p align="center">
  <a href="https://mingrui-yu.github.io/retargeting/">Project Website</a>
  &middot;
  <a href="https://arxiv.org/abs/2506.09384">arXiv</a>
</p>

This repository contains the code for the paper "Analyzing Key Objectives in Human-to-Robot Retargeting for Dexterous Manipulation".

It provides:

- A Python core package for human-to-robot dexterous-hand retargeting.
- Config-driven robot, asset, and retargeting setup.
- Offline replay tooling for quick inspection without ROS or hardware.
- Optional ROS2/RViz, live input, and real robot adapters.

<div align="center">
  <img src="./docs/overview.jpg" alt="retargeting" width="50%" />
</div>

## What's New

**2026-07-13** — The codebase has been comprehensively **reorganized** with clearer boundaries between the retargeting core, runtime adapters, configuration, and applications. The new structure also makes quick offline replay easier to discover and run.

We welcome reproductions of this work and use of this codebase as a baseline. Please open an issue with any questions; we will address them and update the repository promptly.

## Install

The default setup supports offline retargeting, replay, and visualized MuJoCo teleoperation without ROS or robot hardware.

```bash
git clone --recurse-submodules https://github.com/Mingrui-Yu/retargeting.git
cd retargeting

conda create -n retargeting -c conda-forge python=3.10.12 pinocchio
conda activate retargeting

pip install -e ".[replay,mujoco-web,quest3]"
```

For an existing clone, initialize the pinned `mr_utils` submodule before installation:

```bash
git submodule update --init --recursive
```

PyTorch is required by optimizer paths. Install the build matching your CUDA environment from the [official PyTorch instructions](https://pytorch.org/); it is not pinned because the correct wheel depends on the local CUDA runtime.

## Quickstart: Offline Replay

From the repository root, retarget the bundled hand trajectory and open the result in the Viser Web viewer:

```bash
python -m retargeting_apps.main app=offline_retarget end=200 run_name=quickstart_leap \
  post.visualize.enabled=true
```

The terminal prints the viewer address. To open the saved result again without rerunning retargeting:

```bash
python -m retargeting_apps.main app=replay run_name=quickstart_leap
```

Optionally compute benchmark statistics and plots from the same result:

```bash
python -m retargeting_apps.main app=benchmark run_name=quickstart_leap
```

## Teleoperation Flow

Run the bundled raw hand trajectory through the full teleoperation flow and visualize the robot in MuJoCo:

```bash
python -m retargeting_apps.main app=teleop_exe teleoperation_modes=offline_mujoco \
  viewer.enabled=true teleoperation_mode.pipeline.realtime=true input.loop=true
```

This runs the same execution path used by live teleoperation:

```text
offline hand input -> observation mapping -> retargeting -> MuJoCo backend -> Web viewer
```

Open the viewer address printed in the terminal. Press `Ctrl+C` to stop playback.

## Quest 3 Teleoperation

The Quest 3 input is integrated into the same Panda arm + LEAP Hand execution
flow. It receives WebXR hand tracking over USB, converts the Quest 25-joint
skeleton to the 21-joint MANO convention, and retargets it to the robot's 23
commanded joints (7 Panda + 16 LEAP).

Enable developer mode and USB debugging on the Quest, connect it by USB, then
accept the debugging prompt in the headset. If Linux cannot access the device,
install the included udev rule once:

```bash
bash scripts/install_quest_udev_rule.sh
adb devices -l
```

Run the safe, non-physics kinematic backend first:

```bash
python -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=online_quest3_kinematic
```

The process opens the Quest Browser automatically. In the headset, enter the
immersive session and keep the right hand in view. Press `Ctrl+C` to stop. To
select a particular headset or hand, add `input.serial=<adb-serial>` or
`input.hand_side=left`. The default Panda+LEAP profile targets a right hand, so
left-hand use should be validated before control experiments.

For MuJoCo validation, install `.[mujoco-web,quest3]` and select
`teleoperation_modes=online_quest3_mujoco`; enable the viewer explicitly with
`viewer.enabled=true`. Confirm the Quest-to-robot frame calibration in
simulation before connecting robot hardware.

## Dual CRX-5iA + LEAP Preview

The bimanual setup uses a left LEAP hand and a right LEAP hand, each with
6 CRX arm joints and 16 finger joints. The existing config filename
`configs/bimanual/crx5ia_coact_leap.yaml` is retained, but now selects two LEAP
arms. Each arm has its own initial pose. The left-hand mount compensates for
its different palm origin using the right-hand mount as a reference; see the
[left-hand asset notes](assets/robots/crx5ia_leap_paxini_left/README.md).

**Mount orientation:** The merged physical-test mount uses the 180-degree
right-hand/EEF flip from `main`; re-check the left-hand translation compensation
and flange-to-wrist alignment whenever the mount transform changes.

Run these commands from the repository root with the project environment
activated. On Howard's WSL installation, activate the existing environment with:

```bash
source /home/howard/.venvs/retargeting_crx/bin/activate
```

Inspect the static initial pose at `http://localhost:9219`:

```bash
python scripts/view_bimanual_initial.py \
  --config configs/bimanual/crx5ia_coact_leap.yaml
```

Preview a small closed EEF trajectory at `http://localhost:9220`:

```bash
python scripts/view_bimanual_trajectory.py \
  --config configs/bimanual/crx5ia_coact_leap.yaml
```

The demo generates reachable flange poses from a small joint-space loop, then
solves arm-only IK from the previous result while holding the fingers at home.
It prints maximum position and orientation errors before opening the viewer.
Yellow shows the target EEF and path; magenta shows the solved EEF. Playback
starts when a browser connects. Uncheck **Play** to pause and use **Frame** to
inspect individual samples. Defaults are a 12-second cycle, 241 samples, and
`--amplitude 0.08` radians for the reference joint motion. Use `--duration`,
`--samples`, `--amplitude`, or `--port` to adjust the preview.

Validate the same trajectory without opening a viewer:

```bash
python scripts/view_bimanual_trajectory.py --headless
```

IK failures or residuals above 1 mm / 0.5 degrees stop the demo. This checks
kinematic EEF tracking; it does not exercise hand retargeting, dynamics,
collision checking, or physical robot control.

For live Quest hand tracking, stop the static viewer first to free port 9219,
connect the headset as described above, then run:

```bash
python scripts/run_bimanual_quest_preview.py \
  --config configs/bimanual/crx5ia_coact_leap.yaml
```

Both hands must be tracked to initialize. All three previews use Viser and
send no commands to robot hardware. Press `Ctrl+C` to stop.
## Dual-CRX Mock Integration

The CRX+LEAP retargeting profile can be exercised with a Quest and the
software-only `dual_crx_ros2` mock stack. This path does not connect to or move
physical hardware.

Start the mock ROS gateway in one terminal:

```bash
cd /home/msc-crx/dual_crx_ros2
export PATH="/usr/bin:/bin:$PATH"
source scripts/codex_env.sh
ros2 launch dual_crx_bringup leaphand_tele_mock.launch.py
```

In a second terminal, start the Quest retargeting process:

```bash
cd /home/msc-crx/retargeting_crx
source /home/msc-crx/miniconda3/etc/profile.d/conda.sh
conda activate retargeting_ros
source /opt/ros/jazzy/setup.bash
source /home/msc-crx/ws_fanuc/install/setup.bash
source /home/msc-crx/dual_crx_ros2/install/setup.bash

PYTHONPATH="$PWD/src:$PYTHONPATH" \
python -m retargeting_apps.main \
  app=teleop_exe \
  retargeting_profiles=vector_wrist_joint_crx5ia_leap_paxini \
  teleoperation_modes=real_world \
  +inputs=quest3 \
  +backends=dual_crx \
  viewer.enabled=true \
  viewer.type=viser \
  viewer.port=9219 \
  viewer.wait_for_client=false
```

Open `http://localhost:9219` to view the CRX+LEAP URDF. The mock gateway
publishes `/dual_crx/state` and `/right_leap/state`, and accepts the lease,
heartbeat, teleoperation, stop, and release lifecycle. The ROS gateway clamps
finite out-of-range joint targets to its configured limits; malformed or
non-finite commands remain errors. Stop the retargeting process and the ROS
launch with `Ctrl+C` when finished.

The CRX profile currently has a URDF for kinematic visualization but no MuJoCo
simulation model. Physical robot control remains a separate, unvalidated step.

## Citation

```bibtex
@article{xin2026analyzing,
  title={Analyzing Key Objectives in Human-to-Robot Retargeting for Dexterous Manipulation},
  author={Xin, Chendong and Yu, Mingrui and Jiang, Yongpeng and Zhang, Zhefeng and Li, Xiang},
  journal={IEEE Robotics and Automation Practice},
  volume={1},
  pages={29--34},
  year={2026},
  doi={10.1109/RAP.2026.3656110}
}
```

## Contact

For questions, contact Mingrui Yu at [mingruiyu98@gmail.com](mailto:mingruiyu98@gmail.com).

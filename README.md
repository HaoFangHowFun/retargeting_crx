# Human-to-Robot Retargeting: Dual CRX + Quest

This project builds on [retargeting](https://github.com/Mingrui-Yu/retargeting)
for the paper *Analyzing Key Objectives in Human-to-Robot Retargeting for Dexterous
Manipulation*. It retains the core algorithms, offline replay, benchmarking and
optional simulation workflows, with five additions for the CRX setup:

- Two CRX-5iA arms with LEAP hands, solved from the same input frame.
- Quest WebXR hand tracking over USB/ADB.
- Left/right ROS command channels carried in one synchronized 44-joint message.
- Direct targets at the configured command frequency, without interpolation.
- Separate left/right initial joint poses, base placements and hand mounts.

## Install and Test

Run commands from the repository root using system Python and a local virtual
environment. Python 3.10+ is supported; ROS Jazzy integration uses Python 3.12.

```bash
git submodule update --init --recursive
/usr/bin/python3 -m venv .venv
env -u PYTHONPATH .venv/bin/python -m pip install -e ".[dev,quest3,replay]" pin scikit-learn
env -u PYTHONPATH .venv/bin/python -m pip install torch --index-url https://download.pytorch.org/whl/cpu
env -u PYTHONPATH .venv/bin/python -m pytest tests/test_bimanual_quest.py tests/test_bimanual_execution.py -q
```

If `venv` reports missing `ensurepip`, use the bootstrap instructions in
[Local Test Environment](docs/configuration-and-development.md#local-test-environment).
The `pin` distribution supplies Pinocchio. CPU PyTorch is sufficient for these
headless tests. The `replay` extra includes Viser's URDF loader and the COLLADA
loader needed by the CRX meshes. Optional MuJoCo workflows require `.[mujoco]` or `.[mujoco-web]`.
The `env -u PYTHONPATH` prefix isolates offline commands from inherited ROS paths;
omit it for ROS execution after sourcing the ROS environment.

## Offline Replay

Retarget the bundled trajectory and save artifacts under `outputs/`:

```bash
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main \
  app=offline_retarget end=200 run_name=quickstart_leap
```

Add `post.visualize.enabled=true` to open the Viser viewer. To view saved results
or compute benchmark statistics:

```bash
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main app=replay run_name=quickstart_leap
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main app=benchmark run_name=quickstart_leap
```

## Dual-Arm Quest Execution

Enable Quest developer mode and USB debugging, connect USB and accept the
headset authorization prompt. `adb devices -l` should list the device. If Linux
needs a device access rule, use `scripts/install_quest_udev_rule.sh`.

Preview both arms with a kinematic scene:

```bash
env -u PYTHONPATH .venv/bin/python -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=bimanual_quest viewer.enabled=true
```

The application opens Quest Browser. Enter the immersive session and keep both
hands tracked for calibration. The viewer defaults to port 9219. This preview
uses no ROS backend. Set `input.serial=<adb-serial>` to select a headset.
To view only the configured initial robot poses without starting Quest:

```bash
env -u PYTHONPATH .venv/bin/python scripts/view_bimanual_initial.py
```

For ROS output, first start your configured `dual_crx_ros2` gateway, then run:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
source ~/dual_crx_ros2/install/setup.bash
.venv/bin/python -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=bimanual_quest backends=dual_crx viewer.enabled=true
```

This command enables physical output when the gateway controls real hardware.
The default enables both arms and the right LEAP; the left LEAP is disabled.
Use `bimanual.left_hand_enabled=true` only when the gateway also has that hand
configured. `bimanual.duration=60` stops 60 seconds after initial calibration;
zero means unlimited. Ctrl+C releases the session. For gateway setup, tracking
loss and cleanup behavior, see [physical execution](docs/bimanual_physical.md).

The solid robot meshes show measured joints when using ROS, and solved joints
in preview. Red wrist markers show command targets. An absent hand has only
configured placeholder positions, not measured feedback.

The existing `python -m retargeting_apps.bimanual_quest` and
`scripts/run_bimanual_quest_preview.py` commands remain compatibility entrypoints
and call the same application. New code should use the unified command above.
Single-arm Quest modes remain available as `online_quest3_kinematic` and
`online_quest3_mujoco`.

## Frequency and Output Filtering

The default command frequency is **20 Hz**. Change `backend.command_hz=20` on the
CLI, or edit `configs/backends/dual_crx.yaml` for ROS and
`configs/backends/kinematic.yaml` for preview. The compatibility CLI's
`--command-hz` overrides this same backend setting; it has no separate default.

`src/teleoperation/bimanual_execution.py` schedules the loop. It takes the newest
complete Quest frame, solves each arm once, and sends at most one joint target
per accepted frame. Duplicate, incomplete or stale frames do not become new
targets. Slow solving can reduce the actual output frequency; this is not a
guarantee to process every native Quest frame.

The dual-arm path generates **no intermediate trajectory points** and does not
use the legacy high-frequency spline node or startup waypoint interpolation.
`use_high_freq_interp` is false in the recommended modes. The legacy Panda
interpolation node and optional offline MuJoCo startup waypoints remain available
for their existing workflows. Gateway controller frequency and limits are
separate from retargeting target frequency. Lease heartbeats and startup holds
are lifecycle messages, not extra solved input frames.

Physical commands use first-order low-pass smoothing with **alpha=0.3 for both
arms and hands**, matching the reference project. Values live in
`configs/bimanual/crx5ia_coact_leap.yaml` under `output.arm_smoothing_alpha` and
`output.hand_smoothing_alpha`. The formula is
`q = alpha * target + (1 - alpha) * previous_command`. This filter changes the
values without adding frames. Set `teleoperation_mode.output.smooth_output_qpos=false`
to disable it, or set the desired alpha to 1. Preview continues to show raw
solutions.

## ROS Interface

The left and right logical channels share one atomic target message. Both sides
come from the same Quest sequence. Message-level synchronization is not a claim
of hardware synchronization. Interface definitions are supplied by the external
`dual_crx_interfaces` package.

| Topic | Message type | Direction / content |
| --- | --- | --- |
| `/dual_crx/teleop/bimanual/command` | `dual_crx_interfaces/msg/TeleopCommand` | Output: 44 named joint positions |
| `/dual_crx/state` | `dual_crx_interfaces/msg/SystemState` | Input: left/right arm feedback and control state |
| `/left_leap/state`, `/right_leap/state` | `dual_crx_interfaces/msg/LeapState` | Input: feedback for enabled hands |

`TeleopCommand.client_id` identifies the lease owner. `target` is a
`sensor_msgs/JointState` with positions in **radians**, in this exact order:

| Indices | `target.name` |
| --- | --- |
| 0–5 | `left_J1` … `left_J6` |
| 6–21 | `left_leap_joint_0` … `left_leap_joint_15` |
| 22–27 | `right_J1` … `right_J6` |
| 28–43 | `right_leap_joint_0` … `right_leap_joint_15` |

`target.header.stamp` is command generation time on the ROS clock, not a target
arrival deadline or the Quest capture time. `velocity`, `effort` and `frame_id`
are empty. Values must be finite. Disabled hands retain their slots in the
44-position vector but receive no hand-enable request from this application.

Command QoS uses KEEP_LAST depth 1; state subscriptions use depth 10. The integer
QoS profiles use rclpy's default RELIABLE reliability and VOLATILE durability.

| Service | Type | Purpose |
| --- | --- | --- |
| `/dual_crx/acquire_control` | `dual_crx_interfaces/srv/AcquireControl` | Acquire BOTH scope (`arm_scope=3`) |
| `/dual_crx/heartbeat` | `dual_crx_interfaces/srv/Heartbeat` | Renew the lease |
| `/dual_crx/release_control` | `dual_crx_interfaces/srv/ReleaseControl` | Release the lease |
| `/dual_crx/teleop/bimanual/enable` | `dual_crx_interfaces/srv/SetTeleop` | Enable/disable the session |
| `/dual_crx/teleop/bimanual/pause_tracking` | `dual_crx_interfaces/srv/SetTeleop` | Pause (`enabled=true`) / resume (`false`) tracking |
| `/dual_crx/stop` | `dual_crx_interfaces/srv/SoftwareStop` | Stop output |
| `/left_leap/enable`, `/right_leap/enable` | `std_srvs/srv/SetBool` | Enable/disable installed hands |

The right-only backend retains `/dual_crx/teleop/command` with 22 right-side joints
and RIGHT scope (`arm_scope=2`). It does not run concurrently with a BOTH session.

## Initial Poses and Layout

The historical filename `configs/bimanual/crx5ia_coact_leap.yaml` now selects two
LEAP-equipped CRX arms and remains supported for existing commands.

| Setting | Left | Right |
| --- | --- | --- |
| Robot config | `configs/robots/crx5ia_leap_paxini_left.yaml` | `configs/robots/crx5ia_leap_paxini.yaml` |
| Arm initial joints (rad) | `[0, 0, 0, 0, -pi/2, 0]` | `[-pi/2, 0, pi, 0, pi/2, 0]` |
| Base position (m) | `[0, 0.3, 0]` | `[0, -0.3, 0]` |
| Base RPY (rad) | `[0, 0, 0]` | `[0, 0, -pi/2]` |

Each robot config also holds its 16 finger initial positions. Hand mounts and
wrist frames remain defined by the corresponding URDF assets. The left hand has
its own mounting transform; see its [asset notes](assets/robots/crx5ia_leap_paxini_left/README.md).

Preview begins at these YAML poses. Physical startup and tracking recovery seed
from fresh measured robot joints and calibrate the Quest wrist origin there;
connecting does not command a move to the YAML home pose.

## Code Layout

| Package | Responsibility |
| --- | --- |
| `retargeting` | Pure algorithms, kinematics, configuration and metrics |
| `teleoperation` | Inputs, mapping, policies and single/dual-arm execution flows |
| `retargeting_apps` | Hydra/CLI composition, offline artifacts and visualization |
| `retargeting_ros` | ROS and hardware adapters |

Both ROS backend classes now live in `retargeting_ros.dual_crx`. ROS-free joint
mapping lives in `teleoperation.backends.dual_crx_contract`. The unused duplicate
publishers, COACT placeholder path, LEAP-only backend and synthetic bimanual
trajectory demo have been removed. The static initial-pose viewer is retained.
See [configuration and development](docs/configuration-and-development.md) for
other workflows, and [reorganized.md](reorganized.md) for the design and migration map.

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

# Sharpa Wave preview and mock validation

The CRX + Sharpa scene uses `configs/bimanual/crx5ia_sharpa_wave.yaml`.
The hand-only scene uses `configs/bimanual/sharpa_wave.yaml`. Both use the same
local Sharpa models as the ROS execution scripts. The flange transforms in
`configs/sharpa_mounts.yaml` are provisional and must be checked against the
physical mounts before controlling real hardware.
The left CRX mount currently rotates the hand 180 degrees about `fanuc_flange`
Z; the right mount remains at zero rotation.

From the repository root, inspect the initial scene without Quest or ROS:

```bash
env -u PYTHONPATH .venv/bin/python scripts/view_bimanual_initial.py --config configs/bimanual/crx5ia_sharpa_wave.yaml
```

Open `http://localhost:9219` in the Windows browser. Inspect both bases,
flanges, palms, thumb directions, and the initial joint pose. For the hand-only
scene, pass `--config configs/bimanual/sharpa_wave.yaml` instead.

With Quest connected and authorized in WSL (`adb devices -l`), run a virtual
preview that publishes no robot commands:

```bash
env -u PYTHONPATH .venv/bin/python scripts/run_crx_sharpa_joint_teleop.py --backend preview
```

The hand-only equivalent is
`scripts/run_sharpa_joint_teleop.py --backend preview`. Quest Browser connects
to the WebXR receiver on port 8765; the
viewer remains on port 9219. Both hands must be tracked for calibration.
Try wrist translation and rotation, each finger, tracking loss, and recovery.

For an isolated ROS mock test, source ROS Jazzy and installed copies of both
driver workspaces in each terminal. Start the Sharpa mock driver in one terminal:

```bash
export ROS_DOMAIN_ID=189
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch dual_sharpa_wave dual_sharpa.launch.py backend:=mock use_rviz:=false
```

To test arms and hands together, also start the CRX mock driver in another
terminal using the same domain:

```bash
ros2 launch dual_crx_control dual_arm.launch.py mock:=true rviz:=false method:=linear input_rate_hz:=20.0
```

Then run one of these finite synthetic-input smoke tests from this repository,
with the same sourced ROS environment and domain:

```bash
.venv/bin/python scripts/run_sharpa_joint_teleop.py --backend ros --no-viewer --synthetic-frames 100
.venv/bin/python scripts/run_crx_sharpa_joint_teleop.py --backend ros --no-viewer --synthetic-frames 100
```

Run the two commands separately. The first requires only Sharpa feedback; the
second requires Sharpa and CRX feedback. The scripts wait for complete, fresh
joint states and subscribers before publishing. They stop on input exhaustion.
To run the opt-in automated mock test:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SHARPA_ROS_TEST=1 .venv/bin/python -m pytest tests/test_sharpa_ros_integration.py -q
```

For live Quest input with mock drivers, omit `--synthetic-frames`;
`--duration 60` sets a finite session. The solver has a 25 ms budget per hand. A slow solve
can cause a frame to be dropped, and a missing feedback stream pauses output
until fresh feedback and a new calibration are available. The target rate is
20 Hz; the actual rate and solve time are reported during execution. Mock
results verify the software path only. Physical control requires confirmed
mounts, joint mapping, safety setup, and a separate device test.

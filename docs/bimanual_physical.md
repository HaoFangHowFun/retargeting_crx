# Bimanual Quest to dual CRX

Both arms consume one synchronized Quest frame and send a single 44-position
command through the gateway. The [README](../README.md#ros-interface) defines
message fields, joint order, topics, services, frequency and initial poses.

## Gateway and application

The default setup enables both CRX arms and the right LEAP. The left LEAP is
optional: its targets are computed but its device receives no request until both
the gateway and application enable it. The backend seeds from fresh measured
joints; it does not move the arms to their YAML home poses.

On the gateway machine, start the configured physical launch:

```bash
cd ~/dual_crx_ros2
bash scripts/launch_bimanual_physical.sh left_motion_control:=1 right_motion_control:=1
```

Wait for both arms to be ready. Do not run a right-only teleop owner alongside a
BOTH session. In another terminal, from the retargeting repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ws_fanuc/install/setup.bash
source ~/dual_crx_ros2/install/setup.bash
.venv/bin/python -u -m retargeting_apps.main app=teleop_exe \
  teleoperation_modes=bimanual_quest backends=dual_crx viewer.enabled=true
```

The `.venv` must use system-compatible Python 3.12 and have the README dependencies
installed. Keep ROS's `PYTHONPATH` for this command. The solid viewer meshes show
measured feedback, and red wrist markers show command targets. Left-hand values
remain placeholders when the physical left hand is disabled.

For a physical left hand, configure the gateway with
`left_hand_enabled:=true left_port:=/dev/serial/by-id/ACTUAL_LEFT_DEVICE`, and add
`bimanual.left_hand_enabled=true` to the application command. Use a distinct port
for each hand. For fake hardware, the gateway provides
`ros2 launch dual_crx_bringup bimanual_teleop.launch.py`; keep fake and physical
sessions in separate ROS domains.

`backend.command_hz` defaults to 20. Gateway controller rates, command timeouts
and joint speed limits are configured in the external gateway, independently
of the retargeting rate. This application generates no interpolation waypoints.
Physical arm and hand low-pass filters both default to alpha 0.3.

## Stop, tracking loss and recovery

Ctrl+C stops the session, disables requested hands and releases the lease.
Add `bimanual.duration=60` for an automatic stop 60 seconds after initial
calibration. Zero means unlimited. The timer continues during tracking loss,
and an independent timer requests stop even when solving is slow. It is a
software session timer, not a hardware emergency stop.

If either hand leaves Quest's view, both arms pause and installed hands hold
with torque retained. The control lease remains alive. Both hands must supply
fresh complete frames continuously for 0.3 seconds before recovery. The next
fresh frame recalibrates wrist origins and solver/filter seeds from the current
measured robot pose; preview uses its last displayed pose.

Tracking pause uses `/dual_crx/teleop/bimanual/pause_tracking`. Gateway readiness,
feedback and control checks remain active. A fault-stopped session cannot be
restarted by unpausing: resolve the gateway fault and restart the application.
Existing gateway joint limits and speed limits remain in effect.

## Offline verification

```bash
env -u PYTHONPATH .venv/bin/python -m pytest \
  tests/test_bimanual_quest.py tests/test_bimanual_execution.py \
  tests/test_dual_crx_contract.py tests/test_dual_crx_startup.py -q
```

These tests use synthetic input and fake ROS messages without starting devices.
They verify software contracts, not physical calibration or hardware operation.

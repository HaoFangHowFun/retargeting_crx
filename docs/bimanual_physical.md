# Bimanual Quest to dual CRX

The `feature/bimanual-quest-coact` branch now has an explicit execution entry
point. The existing preview command remains hardware-free. The viewer displays
retargeted **targets**, not measured feedback. Preview base placements are left
unchanged; they are not a physical workcell calibration.

The command is one named 44-position message in radians:
left CRX J1–J6, left LEAP 0–15, right CRX J1–J6, right LEAP 0–15.
The ROS gateway validates both arms before issuing either arm command. Separate
Servo messages use the same timestamp; this is not a hardware synchronization guarantee.

## Current hardware

Both CRX arms and right LEAP are enabled for this setup. Left LEAP is absent:
its targets are computed/displayed but no motor command or torque-enable request
is sent. Its placeholder positions are never claimed as physical feedback.
The backend seeds both arms from current measured joints; it does not move to
the YAML initial pose. Both Quest hands must be tracked for calibration and output.

## Run in foreground terminals

Stop the previous retargeting process and its ROS launch before replacing them.
Terminal 1:

```bash
cd /home/msc-crx/dual_crx_ros2
bash scripts/launch_bimanual_physical.sh left_motion_control:=1 right_motion_control:=1
```

Wait for both arms' `motion_possible` to be true. The existing FANUC authority
requests remain available if needed; no new authority workflow is introduced.
Do not run the old right-only teleop process alongside this session.

Terminal 2:

```bash
conda activate retargeting_ros
cd /home/msc-crx/retargeting_crx
source /opt/ros/jazzy/setup.bash
source /home/msc-crx/ws_fanuc/install/setup.bash
source /home/msc-crx/dual_crx_ros2/install/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}:/usr/lib/python3/dist-packages"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
python -u -m retargeting_apps.bimanual_quest --backend dual_crx
```

Open http://localhost:9219. This command starts the Quest USB/browser input and
acquires a BOTH lease after both hands are visible. Physical output starts after
measured-pose calibration. Use Ctrl+C to stop the execution process.
`--no-viewer` omits the visualizer. `--command-hz` defaults to 20.
The gateway uses a 100 Hz update and a 0.25 s command timeout outside tracking
pause. The current launch sets CRX speed to 0.8 rad/s and physical LEAP speed to
2.7 rad/s; gateway caps are 1.2 and 3.0 rad/s respectively. Tune
`teleop_arm_speed` and `teleop_hand_speed` in `dual_crx.launch.py`, then restart ROS.
No extra retargeting speed limiter is added on this path.

For preview only:

```bash
python -u scripts/run_bimanual_quest_preview.py
```

## Future left hand / mock tests

To use a left hand later, configure the ROS launch with
`left_hand_enabled:=true left_port:=/dev/serial/by-id/ACTUAL_LEFT_DEVICE` and
run this app with `--left-hand-enabled`. A distinct physical port is required.
Leave both flags off with the current hardware.

For ROS fake hardware, use `ros2 launch dual_crx_bringup bimanual_teleop.launch.py`.
It defaults to fake arms and a fake right LEAP; set `left_hand_enabled:=true`
to test both fake hands. Use the same hand-enabled flags on the retargeting side.
Do not run fake and physical sessions in the same ROS domain.

CRX fault/authority loss, Servo halt, lease loss or command timeout latches the
whole dual-arm teleop session off while ROS remains running. Resolve the cause
and restart the execution app to re-enable. A LEAP communication fault remains
local to the hand's existing recovery, without stopping the arms. Closing the
backend retains the existing cleanup behavior (software stop, disable requested
hand(s), release lease).

Validated with offline tests and isolated fake ROS transport. The operator
reported stable physical dual-arm Quest operation on 2026-09-10.

## Automatic session stop

Add `--duration SECONDS` to stop without using the keyboard:

```bash
python -u -m retargeting_apps.bimanual_quest --backend dual_crx --duration 60
```

The countdown begins when both hands are calibrated against the measured robot
pose, not while waiting for Quest or ROS. At expiry an independent timer latches
command output off and requests the existing ROS software stop. The execution
loop then closes Quest/viewer and releases the lease; the ROS launch stays alive.
Timed expiry and Ctrl+C both disable torque on the enabled LEAP hands during
session cleanup. Disabled/absent hands receive no request. A new invocation is
required to resume. This is a software session timer, not a hardware emergency stop.
`--duration 0` (default) keeps the existing unlimited session. Negative/non-finite
values are rejected. The preview command also accepts `--duration`.


### Tracking loss and reacquisition

If either hand leaves Quest's view, both arms pause with zero jog velocity and
installed LEAP hands hold with torque retained. The program and control lease stay
alive. Both hands must provide fresh complete frames continuously for 0.3 seconds
before resuming. Wrist translation origins and solver seeds are recaptured using
the measured robot pose; the old out-of-view target is discarded. Existing joint
speed limits still apply. Preview uses the last displayed pose instead.

Tracking pause uses the owned `SetTeleop` service
`/dual_crx/teleop/bimanual/pause_tracking` (`enabled=true` pauses,
`enabled=false` resumes). CRX readiness, Servo status, feedback freshness and lease
checks continue during pause. This service cannot restart a fault-stopped session.
The session countdown continues while tracking is lost. Expiry or Ctrl+C ends the
session and disables LEAP torque as usual. Restart the ROS launch after installing
this update so the new pause service exists.

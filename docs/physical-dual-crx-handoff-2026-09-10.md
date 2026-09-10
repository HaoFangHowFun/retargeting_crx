# Physical dual-CRX handoff checkpoint (2026-09-10)

This note is the handoff boundary between `retargeting_crx` and
`dual_crx_ros2` for the next operator-supervised physical test. It distinguishes
the right-arm path that can be tested now from the bimanual path that still
requires implementation.

## Revisions to use

- `retargeting_crx`: branch `feature/bimanual-quest-coact`; this handoff commit
  must include ancestors `3e66134` (feedback-seeded startup), `54dff7f`
  (44-DoF command contract), and `08e4f46` (bimanual position helpers).
- `dual_crx_ros2`: branch `leaphand_tele` at `6d6c0f3`.
- The retargeting branch is ahead of its local
  `origin/feature/bimanual-quest-coact` tracking ref. Push it before expecting a
  second computer to see this checkpoint.

Do not substitute `main` in either repository: the physical teleoperation
changes described here are branch-specific.

## What is connected now

The currently executable physical route is **right CRX + right LEAP only**:

1. Quest WebXR input produces one right-hand sample.
2. The CRX+LEAP profile solves 6 arm and 16 hand positions.
3. `DualCrxRobotBackend` acquires a RIGHT lease, enables `/right_leap`, seeds
   its target from fresh measured feedback, enables teleoperation, publishes a
   22-name `TeleopCommand`, renews the lease, and attempts stop/torque-off/release
   during shutdown.
4. `dual_crx_ros2` validates and rate-limits the frame, commands right MoveIt
   Servo and the right LEAP adapter, and stops on command timeout, invalid input,
   lost ownership, stale robot status, excessive tracking error, or Servo fault.

Hardware-free contract/startup tests exist, the dual-CRX mock path has passed,
and a brief physical Quest session has previously moved the right side. That
session ended on a right-J6 limit warning followed by a Servo warning/halt or
stale-status stop. Controller-loop overruns were also observed. Treat this as a
checkpoint, not sustained-motion acceptance.

## What is not connected yet

The bimanual preview is not a bimanual hardware controller. The repository has
synchronized left/right Quest decoding, two independent 22-DoF retargeting
pipelines, left/right visualization, and a validated left-then-right 44-DoF
name/position layout. However:

- the normal `app=teleop_exe` construction path builds one profile, one solver,
  and one 22-DoF backend;
- `DualCrxRobotBackend` is fixed to shape `(22,)`, RIGHT scope,
  `/right_leap/state`, and `/right_leap/enable`;
- `BimanualDualCrxPublisher` only constructs a 44-DoF ROS message; it does not
  own startup, feedback, lease, heartbeat, stop, or recovery lifecycle;
- `dual_crx_ros2` accepts the canonical 22 right-side names only and its
  teleoperation gateway drives only the right Servo and right LEAP node;
- there is no physical left-LEAP adapter/gateway path or accepted atomic
  bimanual command contract.

Therefore, do not send the 44-DoF bimanual publisher output to the current
gateway and do not describe the current stack as bimanual hardware-ready.

## Remaining work estimate

For a **right-only supervised retest**, no new cross-repository interface work is
required. Allow one lab session for environment sync, read-only checks, measured
pose hold, small bounded motion, shutdown verification, and a short Quest run.
A separate longer soak is still required for acceptance.

For **bimanual physical Quest control**, four implementation workstreams remain:

1. Add a bimanual execution composition that initializes and steps both solvers
   from one synchronized Quest frame and sends one coherent 44-DoF target.
2. Generalize the retargeting backend to BOTH ownership and fresh feedback for
   two arms/two hands, including partial-startup cleanup and dropout policy.
3. Extend `dual_crx_ros2` with an agreed 44-DoF (or explicitly paired 22-DoF)
   contract, left Servo streaming, left LEAP node, BOTH lease semantics, and a
   fail-together stop/recovery policy.
4. Finish installation-specific calibration and safety inputs: left-hand motor
   mapping, both flange-to-palm transforms, Quest-to-table alignment, joint
   bounds, attached-hand/mount collision geometry, and inter-arm clearance.

After implementation, three hardware gates remain: independent left-side
bring-up, bounded two-side/stop testing without Quest, and short-to-long Quest
soak tests including near-limit, unreachable, singular, stale-input, LEAP-dropout,
and Servo-warning behavior. A reasonable planning allowance is several focused
development days plus multiple supervised lab sessions; calibration findings may
expand this estimate.

## Physical-computer test order

Keep both repositories in foreground terminals and stop any existing robot,
Servo, or hand owners first.

1. Confirm revisions and clean worktrees:

   ```bash
   cd ~/dual_crx_ros2
   git switch leaphand_tele
   git status --short --branch
   git log -1 --oneline

   cd ~/retargeting_crx
   git switch feature/bimanual-quest-coact
   git status --short --branch
   git log -1 --oneline
   ```

2. Run the dual-CRX preflight and resolve every `FAIL`; review every `WARN`:

   ```bash
   cd ~/dual_crx_ros2
   ./scripts/preflight_real.sh \
     --left-ip 192.168.2.100 \
     --right-ip 192.168.1.100
   ```

3. Before granting motion authority, repeat the mock integration and the
   retargeting focused tests shown below.

4. Start the physical stack in its default connection-only/torque-off state:

   ```bash
   cd ~/dual_crx_ros2
   ./scripts/launch_leaphand_physical.sh
   ```

   Confirm fresh robot and hand state, correct joint names/order, healthy LEAP
   feedback, in-range measured positions, no alarms/E-stop, and successful
   explicit shutdown. This launch alone must not move hardware.

5. Only with the operator at the E-stop and the workcell clear, restart with
   right-arm motion authority:

   ```bash
   ./scripts/launch_leaphand_physical.sh right_motion_control:=1
   ```

   First verify a measured-current-pose hold and small bounded right-side motion
   using the control repository's staged procedure. Do not begin with Quest.

6. Start the existing right-only retargeting path in a second terminal:

   ```bash
   cd ~/retargeting_crx
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

   The viewer does not make this command safe: with `backends=dual_crx`, this
   process controls physical hardware. Keep the hand near its initialization
   pose, move slowly through a deliberately small range, and stop immediately on
   J6/Servo warnings, stale feedback, unexpected mapping, or loop overruns.

7. On exit, verify software stop, LEAP torque-off, lease release, zero continued
   arm motion, and no surviving duplicate owner processes. Save ROS logs with the
   exact two repository commits and the first stop reason.

## Software-only verification

From `retargeting_crx` with its project environment active:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q \
  tests/test_dual_crx_contract.py \
  tests/test_dual_crx_startup.py \
  tests/test_bimanual_quest.py \
  tests/test_quest3_input.py
python scripts/view_bimanual_trajectory.py --headless
```

From `dual_crx_ros2`, use the build/test commands in
`docs/leaphand_teleop.md`, then run:

```bash
./scripts/test_leaphand_tele_mock.sh
```

Passing software and mock tests proves message/lifecycle behavior only. It does
not validate the physical mount, hand mapping, collision clearance, network
timing, Servo stability, or long-duration tracking.

Handoff-commit verification on 2026-09-10:

- full `retargeting_crx` suite: 210 passed (56 dependency deprecation warnings);
- regenerated right CRX+LEAP URDF: byte-for-byte identical, SHA-256
  `73b3224a7a1e5564ae22a14d88aa0ca71bef9251b548733d44fc9e6c126eb0fe`;
- headless bimanual trajectory: left and right maximum EEF error both
  `0.0000 mm / 0.0000 deg`.

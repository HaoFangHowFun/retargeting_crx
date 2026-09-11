"""Quest bimanual preview or explicit ROS execution with one BOTH lease."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from retargeting.config import (
    load_retargeting_config,
    load_retargeting_profile_config,
    load_robot_config,
    load_solver_config,
)
from retargeting.core import Retargeter
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio
from retargeting.core.types import RetargetingHandObservation
from retargeting_apps.config import MujocoWebViewerConfig, resolve_project_path
from retargeting_apps.visualization.execution.viser import _load_viser_dependencies
from retargeting_apps.visualization.viser_scene import configure_initial_camera
from retargeting_apps.visualization.viser_scene import ViserHandObservationRenderer
from teleoperation.bimanual import BimanualRetargetingPipeline
from teleoperation.bimanual_execution import BimanualExecutionFlow
from teleoperation.config import load_detection_source_config
from teleoperation.inputs.quest3 import Quest3BimanualOnlineInput
from teleoperation.observation_mapping import RelativeWristMapper


def _build_arm(arm: dict, detection_config):
    profile = load_retargeting_profile_config(arm["profile"])
    robot = load_robot_config(profile.robot)
    method = load_retargeting_config(profile.method)
    model = RobotPinocchio(robot.robot_file_path, robot.model.type)
    adaptor = RobotAdaptor(model, list(robot.actuated_joints))
    retargeter = Retargeter(
        robot_adaptor=adaptor,
        robot_config=robot,
        profile_config=profile,
        method_config=method,
        solver_config=load_solver_config(None),
    )
    mapper = RelativeWristMapper(
        config=detection_config,
        human_hand_scale=robot.human_hand_scale,
        robot_adaptor=adaptor,
        robot_model=model,
        wrist_frame_name=robot.wrist_frame_name,
    )
    return robot, model, retargeter, mapper


def _placement_pose(arm: dict) -> np.ndarray:
    pose = np.eye(4, dtype=float)
    pose[:3, :3] = Rotation.from_euler("xyz", arm["placement"]["rpy"]).as_matrix()
    pose[:3, 3] = np.asarray(arm["placement"]["position"], dtype=float)
    return pose


def _to_scene_observation(observation: RetargetingHandObservation, placement: np.ndarray) -> RetargetingHandObservation:
    return RetargetingHandObservation(
        keypoints_wrist=observation.keypoints_wrist,
        wrist_pose_world=placement @ observation.wrist_pose_world,
        timestamp=observation.timestamp,
        handedness=observation.handedness,
        keypoint_2d=observation.keypoint_2d,
        raw=observation.raw,
    )


def _update_command_wrist_marker(urdf, marker, qpos: np.ndarray, placement: np.ndarray) -> None:
    """Update a red marker at the command wrist pose computed by URDF FK."""
    # Update only yourdfpy's FK state.  Calling ViserUrdf.update_cfg() here
    # would overwrite the visible actual-feedback robot with the command pose.
    urdf._urdf.update_cfg(np.asarray(qpos, dtype=float))
    wrist_pose = placement @ urdf._urdf.get_transform("wrist", "world")
    marker.position = tuple(float(value) for value in wrist_pose[:3, 3])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/bimanual/crx5ia_coact_leap.yaml")
    parser.add_argument("--port", type=int, default=9219)
    parser.add_argument("--adb", default=None)
    parser.add_argument("--serial", default=None)
    parser.add_argument("--backend", choices=("preview", "dual_crx"), default="preview")
    parser.add_argument("--left-hand-enabled", action="store_true")
    parser.add_argument("--right-hand-enabled", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--viewer", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--command-hz", type=float, default=20.)
    parser.add_argument("--duration", type=float, default=0.,
                        help="Auto-stop seconds after tracking initializes; 0 means unlimited")
    args = parser.parse_args()
    if not np.isfinite(args.command_hz) or args.command_hz <= 0:
        parser.error("--command-hz must be finite and positive")

    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and non-negative")

    data = yaml.safe_load(resolve_project_path(args.config).read_text(encoding="utf-8"))
    detection_config = load_detection_source_config(data["input"]["config"])
    left_robot, left_model, left_retargeter, left_mapper = _build_arm(
        data["left"], detection_config.for_hand_side("left")
    )
    right_robot, right_model, right_retargeter, right_mapper = _build_arm(
        data["right"], detection_config.for_hand_side("right")
    )
    pipeline = BimanualRetargetingPipeline(
        left_mapper=left_mapper,
        right_mapper=right_mapper,
        left_retargeter=left_retargeter,
        right_retargeter=right_retargeter,
    )
    source = Quest3BimanualOnlineInput(
        port=int(data["input"].get("port", 8765)),
        max_age_s=float(data["input"].get("max_age_s", 0.15)),
        adb=args.adb,
        serial=args.serial,
    )

    backend_factory = None
    if args.backend == "dual_crx":
        from teleoperation.backends.bimanual_crx import BimanualCrxRobotBackend
        backend_factory = lambda: BimanualCrxRobotBackend(
            initial_qpos=np.concatenate((left_robot.initial_qpos, right_robot.initial_qpos)),
            control_period=1. / args.command_hz,
            left_hand_enabled=args.left_hand_enabled, right_hand_enabled=args.right_hand_enabled)
    server = None
    observer = None
    if args.viewer:
        viser, _ = _load_viser_dependencies()
        from viser.extras import ViserUrdf

        viewer_config = MujocoWebViewerConfig(enabled=True, port=args.port, wait_for_client=False)
        viewer_config.validate()
        try:
            server = viser.ViserServer(host=viewer_config.host, port=viewer_config.port)
        except TypeError:
            server = viser.ViserServer(port=viewer_config.port)
        configure_initial_camera(server, position=(1.1, 1.1, 0.9), look_at=(0.0, 0.0, 0.35))
        left_urdf = ViserUrdf(server, Path(left_robot.robot_file_path), root_node_name="/left_arm", load_meshes=True, load_collision_meshes=False)
        right_urdf = ViserUrdf(server, Path(right_robot.robot_file_path), root_node_name="/right_arm", load_meshes=True, load_collision_meshes=False)
        left_hand_renderer = ViserHandObservationRenderer(server, point_size=0.012, root_node_name="/quest_hand/left")
        right_hand_renderer = ViserHandObservationRenderer(server, point_size=0.012, root_node_name="/quest_hand/right")
        left_placement = _placement_pose(data["left"])
        right_placement = _placement_pose(data["right"])
        left_command_wrist_marker = server.scene.add_icosphere(
            "/command_wrist_endpoint/left", radius=0.025, color=(235, 45, 45),
            subdivisions=3, opacity=1.0,
        )
        right_command_wrist_marker = server.scene.add_icosphere(
            "/command_wrist_endpoint/right", radius=0.025, color=(235, 45, 45),
            subdivisions=3, opacity=1.0,
        )
        for urdf, arm in ((left_urdf, data["left"]), (right_urdf, data["right"])):
            root = getattr(urdf, "_visual_root_frame", None)
            if root is not None:
                root.position = tuple(float(value) for value in arm["placement"]["position"])
                root.wxyz = np.roll(Rotation.from_euler("xyz", arm["placement"]["rpy"]).as_quat(), 1)
        # Always show the configured dual_crx home pose before Quest tracking starts.
        left_urdf.update_cfg(np.asarray(left_robot.initial_qpos, dtype=float))
        right_urdf.update_cfg(np.asarray(right_robot.initial_qpos, dtype=float))
        _update_command_wrist_marker(left_urdf, left_command_wrist_marker, left_robot.initial_qpos, left_placement)
        _update_command_wrist_marker(right_urdf, right_command_wrist_marker, right_robot.initial_qpos, right_placement)
        def observer(result):
            # With the ROS backend, render measured feedback for the solid
            # robot meshes.  Preview mode has no backend, so the solved qpos
            # remains the best available state estimate.
            actual_qpos = result.qpos
            backend = flow.backend
            if backend is not None:
                feedback_qpos = np.asarray(backend.get_joint_pos(), dtype=float)
                if feedback_qpos.shape == (44,) and np.isfinite(feedback_qpos).all():
                    actual_qpos = feedback_qpos
            left_urdf.update_cfg(actual_qpos[:22])
            right_urdf.update_cfg(actual_qpos[22:])
            _update_command_wrist_marker(left_urdf, left_command_wrist_marker, result.left_qpos, left_placement)
            _update_command_wrist_marker(right_urdf, right_command_wrist_marker, result.right_qpos, right_placement)
            left_hand_renderer.update_observation(_to_scene_observation(result.left_observation, left_placement))
            right_hand_renderer.update_observation(_to_scene_observation(result.right_observation, right_placement))
        print(f"Bimanual target preview: http://localhost:{args.port}", flush=True)
    if args.backend == "preview":
        print("Preview only; no hardware commands.", flush=True)
    else:
        print(f"Dual CRX output; left LEAP={args.left_hand_enabled}; right LEAP={args.right_hand_enabled}. "
              "Viewer shows targets, not measured feedback.", flush=True)
    flow = BimanualExecutionFlow(
        source=source, pipeline=pipeline,
        initial_qpos=np.concatenate((left_robot.initial_qpos, right_robot.initial_qpos)),
        backend_factory=backend_factory, observer=observer, command_hz=args.command_hz, duration=args.duration)
    try:
        flow.run()
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.stop()


if __name__ == "__main__":
    main()

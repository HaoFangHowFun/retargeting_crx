"""Preview Quest bimanual retargeting in one Viser scene without robot hardware."""

from __future__ import annotations

import argparse
import time
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/bimanual/crx5ia_coact_leap.yaml")
    parser.add_argument("--port", type=int, default=9219)
    parser.add_argument("--adb", default=None)
    parser.add_argument("--serial", default=None)
    args = parser.parse_args()

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
    for urdf, arm in ((left_urdf, data["left"]), (right_urdf, data["right"])):
        root = getattr(urdf, "_visual_root_frame", None)
        if root is not None:
            root.position = tuple(float(value) for value in arm["placement"]["position"])
            root.wxyz = np.roll(Rotation.from_euler("xyz", arm["placement"]["rpy"]).as_quat(), 1)
    # Always show the configured dual_crx home pose before Quest tracking starts.
    left_urdf.update_cfg(np.asarray(left_robot.initial_qpos, dtype=float))
    right_urdf.update_cfg(np.asarray(right_robot.initial_qpos, dtype=float))
    print(f"Bimanual Quest preview listening on http://localhost:{args.port}")
    print("Left hand -> LEAP arm; right hand -> LEAP arm. No robot hardware is commanded. Ctrl+C to stop.")
    source.open()
    last_report = time.monotonic()
    last_sequence: int | None = None
    try:
        while True:
            sample = source.read()
            frame = sample.left.raw if sample.left.raw is not None else sample.right.raw
            sequence = getattr(frame, "sequence", None)
            if not pipeline.initialized:
                if pipeline.initialize(sample, left_robot.initial_qpos, right_robot.initial_qpos):
                    print(f"Quest tracking initialized at frame {sequence}; both hands are available.")
            if pipeline.initialized and sequence != last_sequence:
                left_observation = left_mapper.map(sample.left)
                right_observation = right_mapper.map(sample.right)
                if left_observation is not None:
                    left_hand_renderer.update_observation(_to_scene_observation(left_observation, left_placement))
                if right_observation is not None:
                    right_hand_renderer.update_observation(_to_scene_observation(right_observation, right_placement))
                result = pipeline.step(sample)
                if result is not None:
                    left_urdf.update_cfg(result.left_qpos)
                    right_urdf.update_cfg(result.right_qpos)
            last_sequence = sequence if sequence is not None else last_sequence
            now = time.monotonic()
            if now - last_report >= 1.0:
                stats = source.stats
                tracked = getattr(frame, "both_hands_tracked", False)
                print(
                    f"Quest frames={getattr(stats, 'accepted', 0)} "
                    f"connections={getattr(stats, 'connections', 0)} "
                    f"latest_sequence={sequence} both_hands_tracked={tracked} "
                    f"initialized={pipeline.initialized}"
                )
                last_report = now
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        server.stop()


if __name__ == "__main__":
    main()

"""Passive dual-arm Viser scene for the shared execution application."""

from dataclasses import replace
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from retargeting_apps.visualization.execution.viser import ViserLiveVisualizer
from retargeting_apps.visualization.viser_scene import (
    ViserHandObservationRenderer,
    configure_initial_camera,
)


def placement_pose(arm: dict) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler("xyz", arm["placement"]["rpy"]).as_matrix()
    pose[:3, 3] = arm["placement"]["position"]
    return pose


class BimanualExecutionVisualizer:
    """Display measured robots and command wrist markers without driving them."""

    def __init__(self, config, setup, flow):
        from viser.extras import ViserUrdf

        self.config, self.flow = config, flow
        self.server = ViserLiveVisualizer._create_server(config)
        self._closed = False
        self.arms = []
        try:
            configure_initial_camera(
                self.server, position=config.initial_camera_position,
                look_at=config.initial_camera_look_at,
            )
            retargeters = (flow.pipeline.left_retargeter, flow.pipeline.right_retargeter)
            for side, retargeter in zip(("left", "right"), retargeters):
                robot = retargeter.robot_config
                urdf = ViserUrdf(
                    self.server, Path(robot.robot_file_path), root_node_name=f"/{side}_arm",
                    load_meshes=True, load_collision_meshes=False,
                )
                placement = placement_pose(setup[side])
                root = getattr(urdf, "_visual_root_frame", None)
                if root is not None:
                    root.position = placement[:3, 3]
                    root.wxyz = np.roll(Rotation.from_matrix(placement[:3, :3]).as_quat(), 1)
                renderer = ViserHandObservationRenderer(
                    self.server, point_size=config.human_keypoint_size,
                    root_node_name=f"/quest_hand/{side}",
                )
                marker = self.server.scene.add_icosphere(
                    f"/command_wrist_endpoint/{side}", radius=0.025,
                    color=(235, 45, 45), subdivisions=3, opacity=1.0,
                )
                # Map commands by name rather than assuming URDF joint order.
                indices = [list(robot.actuated_joints).index(n) for n in urdf.get_actuated_joint_names()]
                arm = (urdf, renderer, marker, placement, indices, robot.wrist_frame_name)
                self.arms.append(arm)
                self._update_arm(arm, retargeter.qpos_init, retargeter.qpos_init)
        except BaseException:
            self.close()
            raise
        flow.observer = self.update
        print(f"Bimanual viewer: http://localhost:{self.server.get_port()}", flush=True)

    @staticmethod
    def _update_arm(arm, actual, command):
        urdf, _, marker, placement, indices, wrist_frame = arm
        urdf.update_cfg(np.asarray(actual)[indices])
        # Only change internal FK for the command marker; retain the actual mesh.
        urdf._urdf.update_cfg(np.asarray(command)[indices])
        marker.position = (placement @ urdf._urdf.get_transform(wrist_frame, "world"))[:3, 3]

    def update(self, result):
        actual = result.qpos if self.flow.backend is None else self.flow.backend.get_joint_pos()
        for i, (arm, command, observation) in enumerate(zip(
            self.arms, (result.left_qpos, result.right_qpos),
            (result.left_observation, result.right_observation),
        )):
            self._update_arm(arm, actual[i * 22:(i + 1) * 22], command)
            arm[1].update_observation(replace(
                observation, wrist_pose_world=arm[3] @ observation.wrist_pose_world,
            ))

    def wait_for_client(self):
        while self.config.wait_for_client and not self.server.get_clients():
            time.sleep(0.05)

    def close(self):
        if not self._closed:
            self._closed = True
            self.server.stop()

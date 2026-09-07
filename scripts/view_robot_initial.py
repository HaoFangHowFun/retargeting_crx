"""Display a robot's configured initial pose without input, solver or backend."""

from __future__ import annotations

import argparse
import time

import numpy as np
from scipy.spatial.transform import Rotation

from retargeting.config import load_robot_config
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio
from retargeting_apps.config import MujocoWebViewerConfig
from retargeting_apps.visualization.execution.viser import ViserLiveVisualizer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="configs/robots/crx5ia_leap_paxini.yaml")
    parser.add_argument("--port", type=int, default=9219)
    args = parser.parse_args()
    robot = load_robot_config(args.robot)
    model = RobotPinocchio(robot.robot_file_path, robot.model.type)
    adaptor = RobotAdaptor(model, list(robot.actuated_joints))
    qpos = np.asarray(robot.initial_qpos, dtype=float)
    model.compute_forward_kinematics(adaptor.forward_qpos(qpos))
    viewer = ViserLiveVisualizer(
        robot_file_path=robot.robot_file_path,
        actuated_joint_names=robot.actuated_joints,
        config=MujocoWebViewerConfig(enabled=True, port=args.port, wait_for_client=False),
    )
    try:
        viewer.update_qpos(qpos)
        for name in ("base_link", "flange", "palm_lower", robot.wrist_frame_name):
            if name not in model.frame_names:
                continue
            pose = model.get_frame_pose(name).copy()
            viewer.server.scene.add_frame(
                f"/initial_frames/{name}",
                position=pose[:3, 3],
                wxyz=np.roll(Rotation.from_matrix(pose[:3, :3]).as_quat(), 1),
                axes_length=0.08,
                axes_radius=0.002,
            )
            viewer.server.scene.add_label(
                f"/initial_frames/{name}/label", name, position=(0, 0, 0.09),
            )
        print("Static initial pose only. Axes: red X, green Y, blue Z. Ctrl+C to stop.")
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()


if __name__ == "__main__":
    main()

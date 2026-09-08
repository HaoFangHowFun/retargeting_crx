"""Display a robot's configured initial pose without input, solver or backend."""

from __future__ import annotations

import argparse
import time

import numpy as np
from scipy.spatial.transform import Rotation

from pathlib import Path

from retargeting.config import load_robot_config
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio
from retargeting_apps.config import MujocoWebViewerConfig
from retargeting_apps.visualization.execution.viser import ViserLiveVisualizer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="configs/robots/crx5ia_leap_paxini.yaml")
    parser.add_argument("--urdf", type=Path, help="Direct URDF preview without a robot config.")
    parser.add_argument("--qpos", nargs="+", type=float, help="Initial qpos for --urdf mode.")
    parser.add_argument("--port", type=int, default=9219)
    args = parser.parse_args()
    if args.urdf is None:
        robot = load_robot_config(args.robot)
        urdf_path = robot.robot_file_path
        joint_names = list(robot.actuated_joints)
        qpos = np.asarray(robot.initial_qpos, dtype=float)
        frame_names = ("base_link", "flange", "palm_lower", robot.wrist_frame_name)
    else:
        robot = None
        urdf_path = str(args.urdf)
        model_probe = RobotPinocchio(urdf_path, "urdf")
        joint_names = list(model_probe.dof_joint_names)
        qpos = np.asarray(args.qpos if args.qpos is not None else np.zeros(len(joint_names)), dtype=float)
        if qpos.shape != (len(joint_names),):
            raise ValueError(f"--qpos must contain {len(joint_names)} values for {joint_names}")
        frame_names = ("base_link", "flange", "coact_gripper_body", "coact_left_jaw", "coact_right_jaw")
    model = RobotPinocchio(urdf_path, "urdf")
    adaptor = RobotAdaptor(model, joint_names)
    model.compute_forward_kinematics(adaptor.forward_qpos(qpos))
    viewer = ViserLiveVisualizer(
        robot_file_path=urdf_path,
        actuated_joint_names=joint_names,
        config=MujocoWebViewerConfig(enabled=True, port=args.port, wait_for_client=False),
    )
    try:
        viewer.update_qpos(qpos)
        for name in frame_names:
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

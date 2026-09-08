"""Display the heterogeneous CRX bimanual initial scene without Quest input."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from retargeting.config import load_robot_config
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio
from retargeting_apps.config import MujocoWebViewerConfig, resolve_project_path
from retargeting_apps.visualization.execution.viser import _load_viser_dependencies
from retargeting_apps.visualization.viser_scene import configure_initial_camera


def _load_arm(server, name: str, arm: dict):
    from viser.extras import ViserUrdf

    robot = load_robot_config(arm["robot"])
    model = RobotPinocchio(robot.robot_file_path, robot.model.type)
    adaptor = RobotAdaptor(model, list(robot.actuated_joints))
    initial_qpos = np.asarray(robot.initial_qpos, dtype=float)
    urdf = ViserUrdf(
        server,
        Path(robot.robot_file_path),
        root_node_name=f"/{name}",
        load_meshes=True,
        load_collision_meshes=False,
    )
    urdf.update_cfg(initial_qpos)
    model_qpos = adaptor.forward_qpos(initial_qpos)
    model.compute_forward_kinematics(model_qpos)
    placement = arm["placement"]
    root = getattr(urdf, "_visual_root_frame", None)
    if root is not None:
        root.position = tuple(float(value) for value in placement["position"])
        root.wxyz = np.roll(Rotation.from_euler("xyz", placement["rpy"]).as_quat(), 1)
    for frame_name in ("base_link", "flange", "wrist", "coact_gripper_body"):
        if frame_name not in model.frame_names:
            continue
        pose = model.get_frame_pose(frame_name, qpos=model_qpos).copy()
        pose[:3, 3] += np.asarray(placement["position"], dtype=float)
        server.scene.add_frame(
            f"/{name}/initial_frames/{frame_name}",
            position=pose[:3, 3],
            wxyz=np.roll(Rotation.from_matrix(pose[:3, :3]).as_quat(), 1),
            axes_length=0.08,
            axes_radius=0.002,
        )
    return robot, urdf


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="configs/bimanual/crx5ia_coact_leap.yaml")
    parser.add_argument("--port", type=int, default=9219)
    args = parser.parse_args()
    data = yaml.safe_load(resolve_project_path(args.config).read_text(encoding="utf-8"))
    viser, _ = _load_viser_dependencies()
    config = MujocoWebViewerConfig(enabled=True, port=args.port, wait_for_client=False)
    config.validate()
    try:
        server = viser.ViserServer(host=config.host, port=config.port)
    except TypeError:
        server = viser.ViserServer(port=config.port)
    configure_initial_camera(server, position=(1.1, 1.1, 0.9), look_at=(0.0, 0.0, 0.35))
    try:
        _load_arm(server, "left_arm", data["left"])
        _load_arm(server, "right_arm", data["right"])
        print("Static bimanual initial pose only. Open http://localhost:%d; Ctrl+C to stop." % args.port)
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()

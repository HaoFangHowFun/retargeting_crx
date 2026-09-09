"""Offline, reachable EEF targets and arm-only IK for the bimanual demo."""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from retargeting.config import load_robot_config
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio


def generate_arm_trajectory(arm: dict, samples: int = 241, amplitude: float = 0.08):
    """Generate a closed reachable path, then solve it from the previous IK result.

    Targets are FK samples of a small joint-space loop; the reference joints are
    never used as IK seeds. Fingers remain at home. This is a kinematic demo,
    without command policies, dynamics, collision checks, or hardware output.
    """
    if samples < 3 or not np.isfinite(amplitude) or not 0 < amplitude <= 0.2:
        raise ValueError("Use at least 3 samples and amplitude in (0, 0.2] radians")
    robot = load_robot_config(arm["robot"])
    model = RobotPinocchio(robot.robot_file_path, robot.model.type)
    adaptor = RobotAdaptor(model, list(robot.actuated_joints))
    home = np.asarray(robot.initial_qpos, dtype=float)
    arm_indices = [list(robot.actuated_joints).index(f"J{i}") for i in range(1, 7)]
    limits = model.joint_limits[adaptor.actuated_joints_model_idx][arm_indices]
    lower, upper = limits[:, 0], limits[:, 1]
    phase = np.linspace(0.0, 1.0, samples)
    angle = 2 * np.pi * (3 * phase**2 - 2 * phase**3)
    reference = np.tile(home, (samples, 1))
    reference[:, arm_indices[0]] += amplitude * np.sin(angle)
    reference[:, arm_indices[1]] += amplitude * (1 - np.cos(angle)) / 2
    reference[:, arm_indices[2]] -= amplitude * (1 - np.cos(angle)) / 2
    if np.any(reference[:, arm_indices] < lower) or np.any(reference[:, arm_indices] > upper):
        raise ValueError("Reference trajectory exceeds joint limits; reduce amplitude")

    def fk(q):
        return model.get_frame_pose("flange", adaptor.forward_qpos(q)).copy()

    targets = np.asarray([fk(q) for q in reference])
    q = home.copy()
    commands, actual = [], []
    for target in targets:
        def residual(arm_q):
            candidate = q.copy()
            candidate[arm_indices] = arm_q
            pose = fk(candidate)
            return np.r_[
                pose[:3, 3] - target[:3, 3],
                Rotation.from_matrix(target[:3, :3] @ pose[:3, :3].T).as_rotvec(),
            ]

        solved = least_squares(
            residual, q[arm_indices], bounds=(lower, upper),
            ftol=1e-11, xtol=1e-11, gtol=1e-11, max_nfev=100,
        )
        if not solved.success or not np.all(np.isfinite(solved.x)):
            raise RuntimeError(f"EEF IK failed at frame {len(commands)}: {solved.message}")
        q[arm_indices] = solved.x
        commands.append(q.copy())
        actual.append(fk(q))
    actual = np.asarray(actual)
    position_error = np.linalg.norm(targets[:, :3, 3] - actual[:, :3, 3], axis=1)
    rotation_error = Rotation.from_matrix(
        targets[:, :3, :3] @ actual[:, :3, :3].transpose(0, 2, 1)
    ).magnitude()
    if position_error.max() > 0.001 or rotation_error.max() > np.deg2rad(0.5):
        raise RuntimeError("Trajectory exceeds 1 mm / 0.5 degree IK tolerance")
    placement = np.eye(4)
    placement[:3, :3] = Rotation.from_euler("xyz", arm["placement"]["rpy"]).as_matrix()
    placement[:3, 3] = arm["placement"]["position"]
    return {
        "robot": robot, "qpos": np.asarray(commands), "placement": placement,
        "target": placement @ targets, "actual": placement @ actual,
        "position_error_m": position_error, "rotation_error_rad": rotation_error,
    }

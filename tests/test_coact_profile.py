import numpy as np

from retargeting.config import load_retargeting_config, load_retargeting_profile_config, load_robot_config, load_solver_config
from retargeting.core import Retargeter
from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio
from retargeting.core.types import RetargetingHandObservation


PROFILE = "configs/retargeting_profiles/wrist_pose_crx5ia_coact_placeholder.yaml"


def _retargeter():
    profile = load_retargeting_profile_config(PROFILE)
    robot = load_robot_config(profile.robot)
    model = RobotPinocchio(robot.robot_file_path, "urdf")
    return robot, Retargeter(
        RobotAdaptor(model, list(robot.actuated_joints)), robot, profile,
        load_retargeting_config(profile.method), load_solver_config("configs/solvers/nlopt_slsqp.yaml"),
    )


def test_coact_profile_loads_as_wrist_only_seven_dof_robot():
    robot, retargeter = _retargeter()
    assert robot.benchmark_required is False
    assert len(robot.actuated_joints) == 7
    assert retargeter.arm_only is True
    assert retargeter.arm_dof == 6


def test_coact_wrist_solve_preserves_gripper_until_mapping_exists():
    robot, retargeter = _retargeter()
    result = retargeter.solve(RetargetingHandObservation(np.zeros((21, 3)), np.eye(4)))
    assert result.qpos.shape == (7,)
    assert np.isfinite(result.qpos).all()
    assert result.qpos[6] == robot.initial_qpos[6]

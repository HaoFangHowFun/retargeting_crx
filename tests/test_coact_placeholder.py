from pathlib import Path

import numpy as np
import yaml

from retargeting.core.kinematics import RobotAdaptor, RobotPinocchio


ASSET = Path("assets/robots/crx5ia_coact_placeholder")


def test_coact_placeholder_has_seven_dof_and_reuses_crx_meshes():
    manifest = yaml.safe_load((ASSET / "manifest.yaml").read_text())
    model = RobotPinocchio(str(ASSET / manifest["entrypoints"]["urdf"]), "urdf")
    adaptor = RobotAdaptor(model, manifest["actuated_joints"])
    assert model.dof == 7
    assert adaptor.actuated_joints_name[-1] == "coact_opening"
    assert model.joint_limits[-1, 0] == 0.0
    assert model.joint_limits[-1, 1] == 0.012
    assert np.isfinite(model.get_frame_pose("flange", adaptor.forward_qpos(np.zeros(7)))).all()


def test_coact_placeholder_declares_unverified_geometry():
    manifest = yaml.safe_load((ASSET / "manifest.yaml").read_text())
    assert manifest["gripper"]["geometry_status"] == "placeholder_only"
    assert manifest["gripper"]["collision_validated"] is False
    assert manifest["gripper"]["tcp_validated"] is False

"""Check initial-viewer assets without starting a viewer or loading hardware."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
import numpy as np
from scipy.spatial.transform import Rotation


@pytest.mark.parametrize("side", ["left", "right"])
def test_bimanual_initial_mesh_references_exist(side):
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load(
        (root / "configs/bimanual/crx5ia_coact_leap.yaml").read_text()
    )
    robot = yaml.safe_load((root / config[side]["robot"]).read_text())
    urdf_path = root / robot["model"]["path"]
    urdf = ET.parse(urdf_path)
    for geometry_type in ("visual", "collision"):
        meshes = urdf.findall(f".//{geometry_type}/geometry/mesh")
        assert meshes, f"{side}: no {geometry_type} meshes"
        for mesh in meshes:
            filename = mesh.attrib["filename"]
            assert (urdf_path.parent / filename).is_file(), (
                f"{side} {geometry_type}: missing {filename}"
            )


def test_left_mount_is_unflipped_while_right_mount_stays_flipped():
    root = Path(__file__).resolve().parents[1] / "assets/robots"

    def joint_transform(urdf, name):
        origin = urdf.find(f"joint[@name='{name}']/origin")
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(origin.attrib["rpy"], sep=" ")
        ).as_matrix()
        pose[:3, 3] = np.fromstring(origin.attrib["xyz"], sep=" ")
        return pose

    left_urdf = ET.parse(root / "crx5ia_leap_paxini_left" / "urdf" / "crx5ia_leap_paxini_left.urdf")
    right_urdf = ET.parse(root / "crx5ia_leap_paxini" / "urdf" / "crx5ia_leap_paxini.urdf")
    left_mount = joint_transform(left_urdf, "flange_to_leap")
    right_mount = joint_transform(right_urdf, "flange_to_leap")
    expected_left = np.eye(4)
    expected_left[:3, :3] = Rotation.from_euler("xyz", [0.0, -1.56, 0.0]).as_matrix()
    expected_left[:3, 3] = [0.037336626243399, 0.047897767636037, 0.140188227821871]
    np.testing.assert_allclose(left_mount, expected_left, atol=1e-10)
    assert not np.allclose(left_mount, right_mount, atol=1e-10)

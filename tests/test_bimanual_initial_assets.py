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


def test_left_mount_aligns_middle_finger_and_preserves_reference_wrist():
    root = Path(__file__).resolve().parents[1] / "assets/robots"

    def joint_transform(urdf, name):
        origin = urdf.find(f"joint[@name='{name}']/origin")
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler(
            "xyz", np.fromstring(origin.attrib["rpy"], sep=" ")
        ).as_matrix()
        pose[:3, 3] = np.fromstring(origin.attrib["xyz"], sep=" ")
        return pose

    frames = []
    for name in ("crx5ia_leap_paxini_left", "crx5ia_leap_paxini"):
        urdf = ET.parse(root / name / "urdf" / f"{name}.urdf")
        mount = joint_transform(urdf, "flange_to_leap")
        frames.append((
            mount @ joint_transform(urdf, "joint_5"),
            mount @ joint_transform(urdf, "palm_to_wrist"),
        ))
    np.testing.assert_allclose(frames[0][0], frames[1][0], atol=1e-10)
    np.testing.assert_allclose(frames[0][1], frames[1][1], atol=1e-10)

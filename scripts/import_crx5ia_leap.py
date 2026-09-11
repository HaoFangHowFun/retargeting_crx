"""Generate the CRX+LEAP URDF from pinned FANUC source (development only).

Xacro is required only for this explicit import command, never at runtime.
Run from the repository root; see assets/robots/crx5ia_leap_paxini/README.md.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

import yaml


SOURCE_COMMIT = "fb40c9803a826ba68c7c8e28ba904a25efa7fcd2"
BUNDLE = Path("assets/robots/crx5ia_leap_paxini")
HAND_URDF = Path("assets/robots/panda_leap_paxini/urdf/panda_leap_paxini.urdf")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(source: Path, destination: Path) -> None:
    import xacro

    commit = subprocess.check_output(
        ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
    ).strip()
    if commit != SOURCE_COMMIT:
        raise ValueError(f"Expected FANUC commit {SOURCE_COMMIT}, got {commit}")
    if subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain"], text=True
    ).strip():
        raise ValueError("FANUC source must be clean for reproducible import.")

    macro = source / "fanuc_crx_description/urdf/crx5ia_urdf_macro.xacro"
    wrapper = f'''<robot name="crx5ia_leap_paxini" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{escape(str(macro))}"/>
      <link name="world"/><link name="ee_mount"/>
      <xacro:crx5ia parent="world" child="ee_mount">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:crx5ia>
    </robot>'''
    document = xacro.parse(wrapper)
    xacro.process_doc(document)
    robot = ET.fromstring(document.toxml())
    copied = {}
    for mesh in robot.findall(".//mesh"):
        prefix = "package://fanuc_crx_description/meshes/crx5ia/"
        filename = mesh.attrib["filename"]
        if not filename.startswith(prefix):
            raise ValueError(f"Unexpected FANUC mesh URI: {filename}")
        relative = Path(filename.removeprefix(prefix))
        source_mesh = source / "fanuc_crx_description/meshes/crx5ia" / relative
        target_mesh = destination / "meshes/crx5ia" / relative
        target_mesh.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_mesh, target_mesh)
        copied[str(source_mesh.relative_to(source))] = sha256(source_mesh)
        mesh.set("filename", f"../meshes/crx5ia/{relative.as_posix()}")

    # Extract the existing LEAP subtree, preserving its wrist and tip frames.
    hand = ET.parse(HAND_URDF).getroot()
    hand_links = {"palm_lower"}
    hand_joints = []
    pending = list(hand.findall("joint"))
    while True:
        selected = [j for j in pending if j.find("parent").get("link") in hand_links]
        if not selected:
            break
        for joint in selected:
            hand_links.add(joint.find("child").get("link"))
            hand_joints.append(joint)
            pending.remove(joint)
    for element in hand:
        if element.tag == "link" and element.get("name") in hand_links:
            element = copy.deepcopy(element)
            for mesh in element.findall(".//mesh"):
                # Reuse checked-in LEAP meshes, no external paths or symlinks.
                filename = mesh.get("filename")
                if not filename.startswith("../meshes/leap_hand/"):
                    raise ValueError(f"Unexpected LEAP mesh: {filename}")
                mesh.set("filename", "../../panda_leap_paxini/meshes/leap_hand/" + Path(filename).name)
            robot.append(element)
    for joint in hand_joints:
        robot.append(copy.deepcopy(joint))
    mount = copy.deepcopy(hand.find("joint[@name='panda_leap']"))
    mount.set("name", "flange_to_leap")
    mount.find("parent").set("link", "flange")
    # Physical-test remount: apply the right mount's relative 180-degree roll
    # to the left mount's flange-local X/Y (x unchanged, y sign-flipped), while
    # preserving the independently selected right-side Z and exact orientation.
    mount.find("origin").set("xyz", "0.037336626243399 -0.047897767636037 -0.065")
    mount.find("origin").set("rpy", "-3.141592653589793 1.56 0")
    robot.append(mount)
    urdf_path = destination / "urdf/crx5ia_leap_paxini.urdf"
    urdf_path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(urdf_path, encoding="utf-8", xml_declaration=True)
    provenance = destination / "provenance"
    provenance.mkdir(parents=True, exist_ok=True)
    for source_file, filename in [
        (macro, "crx5ia_urdf_macro.xacro"),
        (source / "fanuc_crx_description/package.xml", "package.xml"),
        (source / "LICENSES/Apache-2.0.txt", "Apache-2.0.txt"),
        (source / "README.md", "FANUC-README.md"),
    ]:
        shutil.copyfile(source_file, provenance / filename)
        copied[str(source_file.relative_to(source))] = sha256(source_file)
    manifest = {
        "schema_version": 1,
        "robot": "crx5ia_leap_paxini",
        "format": "repository_robot_assets",
        "entrypoints": {"urdf": "urdf/crx5ia_leap_paxini.urdf"},
        "source": {
            "repository": "https://github.com/FANUC-CORPORATION/fanuc_description",
            "commit": commit,
            "sha256": copied,
        },
        "hand_source": {"path": str(HAND_URDF), "sha256": sha256(HAND_URDF)},
        "generated_urdf_sha256": sha256(urdf_path),
    }
    (destination / "manifest.yaml").write_text(yaml.safe_dump(manifest, sort_keys=False))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fanuc-source", type=Path, required=True)
    parser.add_argument("--xacro-python-path", type=Path)
    parser.add_argument("--output", type=Path, default=BUNDLE)
    args = parser.parse_args()
    if args.xacro_python_path:
        sys.path.append(str(args.xacro_python_path))
    generate(args.fanuc_source.resolve(), args.output)


if __name__ == "__main__":
    main()

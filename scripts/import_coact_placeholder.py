"""Generate a visual-only CRX + Co-act EGP-C 40 placeholder URDF.

The gripper geometry is intentionally approximate until the physical unit's
CAD, adapter, TCP, and limits are measured. It is not a collision or hardware
model and is not used by the existing LEAP retargeting solver.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def _box_link(name: str, size: tuple[float, float, float], color: str) -> ET.Element:
    link = ET.Element("link", {"name": name})
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": " ".join(str(value) for value in size)})
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": color})
    return link


def generate(source: Path, output: Path) -> None:
    import xacro
    macro = source / "fanuc_crx_description/urdf/crx5ia_urdf_macro.xacro"
    wrapper = f'''<robot name="crx5ia_coact_placeholder" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="{macro}"/>
      <link name="world"/><link name="ee_mount"/>
      <xacro:crx5ia parent="world" child="ee_mount">
        <origin xyz="0 0 0" rpy="0 0 0"/>
      </xacro:crx5ia>
    </robot>'''
    document = xacro.parse(wrapper)
    xacro.process_doc(document)
    root = ET.fromstring(document.toxml())
    for mesh in root.findall(".//mesh"):
        uri = mesh.get("filename", "")
        prefix = "package://fanuc_crx_description/meshes/crx5ia/"
        if uri.startswith(prefix):
            mesh.set("filename", "../../crx5ia_leap_paxini/meshes/crx5ia/" + uri.removeprefix(prefix))
    body = _box_link("coact_gripper_body", (0.075, 0.045, 0.050), "0.25 0.25 0.28 1")
    left_jaw = _box_link("coact_left_jaw", (0.012, 0.020, 0.045), "0.65 0.65 0.68 1")
    right_jaw = _box_link("coact_right_jaw", (0.012, 0.020, 0.045), "0.65 0.65 0.68 1")
    root.extend((body, left_jaw, right_jaw))
    ET.SubElement(root, "joint", {
        "name": "flange_to_coact", "type": "fixed",
    }).extend([
        ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"}),
        ET.Element("parent", {"link": "flange"}),
        ET.Element("child", {"link": "coact_gripper_body"}),
    ])
    ET.SubElement(root, "joint", {
        "name": "coact_opening", "type": "prismatic",
    }).extend([
        ET.Element("origin", {"xyz": "0 0.026 0", "rpy": "0 0 0"}),
        ET.Element("parent", {"link": "coact_gripper_body"}),
        ET.Element("child", {"link": "coact_left_jaw"}),
        ET.Element("axis", {"xyz": "0 1 0"}),
        ET.Element("limit", {"lower": "0", "upper": "0.012", "effort": "0", "velocity": "0.2"}),
    ])
    ET.SubElement(root, "joint", {
        "name": "coact_right_jaw_mount", "type": "fixed",
    }).extend([
        ET.Element("origin", {"xyz": "0 -0.026 0", "rpy": "0 0 0"}),
        ET.Element("parent", {"link": "coact_gripper_body"}),
        ET.Element("child", {"link": "coact_right_jaw"}),
    ])
    ET.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fanuc-source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--xacro-python-path", type=Path)
    args = parser.parse_args()
    if args.xacro_python_path:
        sys.path.append(str(args.xacro_python_path))
    generate(args.fanuc_source.resolve(), args.output)


if __name__ == "__main__":
    main()

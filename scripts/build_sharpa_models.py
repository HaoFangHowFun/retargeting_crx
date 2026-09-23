"""Build hand-only and CRX+Sharpa URDFs using checked-in assets only."""

import argparse
import copy
import hashlib
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def descendants(root, start):
    links = {start}
    while True:
        children = {j.find('child').get('link') for j in root.findall('joint')
                    if j.find('parent').get('link') in links}
        if children <= links:
            return links
        links.update(children)


def relocate_meshes(root, source, destination):
    for mesh in root.iter('mesh'):
        path = (source.parent / mesh.get('filename')).resolve()
        if not path.is_relative_to(ROOT / 'assets') or not path.is_file():
            raise ValueError(f'Mesh must be a local asset: {path}')
        mesh.set('filename', Path(os.path.relpath(path, destination.parent)).as_posix())


def build(side, combined, mounts):
    hand_file = ROOT / f'assets/robots/sharpa_wave_{side}/urdf/{side}_sharpa_wave_with_flange.urdf'
    name = f'crx5ia_sharpa_wave_{side}' if combined else f'sharpa_wave_{side}'
    destination = ROOT / f'assets/robots/{name}/urdf/{name}.urdf'
    hand = ET.parse(hand_file).getroot()
    relocate_meshes(hand, hand_file, destination)
    sources = [hand_file]
    if combined:
        arm_file = ROOT / 'assets/robots/crx5ia_leap_paxini/urdf/crx5ia_leap_paxini.urdf'
        robot = ET.parse(arm_file).getroot()
        removed = descendants(robot, 'palm_lower')
        for element in list(robot):
            if ((element.tag == 'link' and element.get('name') in removed)
                    or (element.tag == 'joint' and element.find('child').get('link') in removed)):
                robot.remove(element)
        relocate_meshes(robot, arm_file, destination)
        mount = mounts[side]
        if mount['parent'] not in {link.get('name') for link in robot.findall('link')}:
            raise ValueError('Unknown CRX mounting frame')
        sources.append(arm_file)
    else:
        robot = ET.Element('robot')
        ET.SubElement(robot, 'link', name='world')
        mount = {'parent': 'world', 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}
    robot.set('name', name)
    for element in hand:
        robot.append(copy.deepcopy(element))
    # MANO/Quest local +X points along the fingers, native Sharpa +Z does.
    # A dedicated frame keeps this convention separate from physical mounting.
    ET.SubElement(robot, 'link', name=f'{side}_retarget_wrist')
    wrist = ET.SubElement(robot, 'joint', name=f'{side}_retarget_wrist_fixed', type='fixed')
    ET.SubElement(wrist, 'parent', link=f'{side}_hand_wrist')
    ET.SubElement(wrist, 'child', link=f'{side}_retarget_wrist')
    ET.SubElement(wrist, 'origin', xyz='0 0 0', rpy=f'0 {-math.pi / 2} 0')
    joint = ET.SubElement(robot, 'joint', name=f'{side}_sharpa_mount', type='fixed')
    ET.SubElement(joint, 'parent', link=mount['parent'])
    ET.SubElement(joint, 'child', link=f'{side}_hand_flange')
    for key in ('xyz', 'rpy'):
        if len(mount[key]) != 3:
            raise ValueError(f'{key} must have three values')
    ET.SubElement(joint, 'origin', **{key: ' '.join(str(float(v)) for v in mount[key])
                                    for key in ('xyz', 'rpy')})
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(robot, space='  ')
    ET.ElementTree(robot).write(destination, encoding='utf-8', xml_declaration=True)
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    manifest = {
        'schema_version': 1, 'robot': name, 'format': 'repository_robot_assets',
        'entrypoints': {'urdf': str(destination.relative_to(destination.parent.parent))},
        'sources': {str(p.relative_to(ROOT)): sha(p) for p in sources},
        'mount': mount, 'mount_calibrated': False,
        'generated_urdf_sha256': sha(destination),
    }
    # Preserve the original standalone hand's upstream provenance manifest.
    filename = 'manifest.yaml' if combined else 'retargeting_manifest.yaml'
    (destination.parent.parent / filename).write_text(yaml.safe_dump(manifest, sort_keys=False))
    if combined:
        (destination.parent.parent / 'README.md').write_text(
            f'# {name}\n\nGenerated with `scripts/build_sharpa_models.py`.\n\n'
            'All meshes are shared by relative paths within this repository; no external paths or symlinks.\n'
            f'Sharpa source and licenses: `../sharpa_wave_{side}/`.\n'
            'CRX source and license: `../crx5ia_leap_paxini/provenance/`.\n\n'
            'Mounting is provisional: edit `configs/sharpa_mounts.yaml` and regenerate.\n'
            'Confirm mounting direction and offsets in preview before physical use.\n')
    print(destination.relative_to(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mounts', type=Path, default=Path('configs/sharpa_mounts.yaml'))
    args = parser.parse_args()
    mounts = yaml.safe_load((ROOT / args.mounts).read_text())
    for side in ('left', 'right'):
        for combined in (False, True):
            build(side, combined, mounts)


if __name__ == '__main__':
    main()

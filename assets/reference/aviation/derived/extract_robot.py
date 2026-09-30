"""Extract the robot-only reference from the aviation project's MJCF snapshot.

This is an asset migration, not a calibrated model generator. Keeps robot
parameters unchanged; removes task objects, their contact pairs and keyframes.
"""
import argparse
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET


def extract(source: Path, output: Path):
    root = ET.parse(source).getroot()
    root.set('model', 'fr3_rh56_right_reference')
    world = root.find('worldbody')
    assert world is not None
    robot = world.find("body[@name='base']")
    assert robot is not None
    for element in list(world):
        if element is not robot:
            world.remove(element)
    # Keyframe qpos includes the removed free object's pose. Do not truncate
    # these task presets and silently turn them into a new initial condition.
    for tag in ('keyframe',):
        for element in root.findall(tag):
            root.remove(element)
    bodies = {e.get('name') for e in world.iter('body')}
    geoms = {e.get('name') for e in world.iter('geom')}
    contact = root.find('contact')
    if contact is not None:
        for element in list(contact):
            refs = bodies if element.tag == 'exclude' else geoms
            keys = ('body1', 'body2') if element.tag == 'exclude' else ('geom1', 'geom2')
            if any(element.get(key) not in refs for key in keys):
                contact.remove(element)
    asset = root.find('asset')
    used_meshes = {e.get('mesh') for e in root.iter('geom') if e.get('mesh')}
    for element in list(asset):
        if element.tag == 'mesh':
            name = element.get('name') or Path(element.get('file')).stem
            if name not in used_meshes:
                asset.remove(element)
    compiler = root.find('compiler')
    source_meshdir = source.parent / compiler.get('meshdir', '')
    compiler.set('meshdir', 'meshes')
    assert not root.findall('.//include'), 'Unexpected external XML dependency'
    assert not root.findall('./sensor'), 'Review sensor dependencies before extraction'
    output.parent.mkdir(parents=True, exist_ok=True)
    copied = []
    for element in asset:
        if 'file' not in element.attrib:
            continue
        assert element.tag == 'mesh', 'Review non-mesh file dependency'
        rel = Path(element.get('file'))
        assert not rel.is_absolute() and '..' not in rel.parts
        src = source_meshdir / rel
        dst = output.parent / 'meshes' / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        copied.append((src, dst))
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding='utf-8', xml_declaration=True)
    return copied


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists')
    extract(args.source, args.output)

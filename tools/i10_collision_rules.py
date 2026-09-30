"""Transfer only SRDF Adjacent rules verified against the current i10 URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / 'assets/robots/aubo_i10/urdf/aubo_i10_JY.urdf'
SRDF = URDF.with_suffix('.srdf')


def adjacent_pairs():
    robot = ET.parse(URDF).getroot()
    parent_pairs = {frozenset((j.find('parent').get('link'), j.find('child').get('link')))
                    for j in robot.findall('joint')}
    pairs = []
    for rule in ET.parse(SRDF).getroot().findall('disable_collisions'):
        if rule.get('reason') != 'Adjacent':
            continue
        pair = (rule.get('link1'), rule.get('link2'))
        if frozenset(pair) not in parent_pairs:
            raise ValueError(f'SRDF Adjacent rule is not a URDF parent/child pair: {pair}')
        pairs.append(pair)
    return pairs


def add_adjacent_exclusions(model):
    names = {b.get('name') for b in model.iter('body')}
    contact = model.find('contact')
    if contact is None:
        contact = ET.SubElement(model, 'contact')
    existing = {frozenset((e.get('body1'), e.get('body2'))) for e in contact.findall('exclude')}
    for a, b in adjacent_pairs():
        if a not in names or b not in names:
            raise ValueError(f'i10 link missing in generated MJCF: {a}, {b}')
        if frozenset((a, b)) not in existing:
            ET.SubElement(contact, 'exclude', body1=a, body2=b)
            existing.add(frozenset((a,b)))

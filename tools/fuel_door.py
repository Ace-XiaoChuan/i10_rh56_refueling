"""Door geometry for the combined kinematic scene; dimensions uncalibrated."""
import math
import xml.etree.ElementTree as ET

MAX_ANGLE = math.radians(180)


def add_door(tank_body, *, initial_angle=0.0):
    # Tank-local +Z points outwards, +Y points upwards in the upright scene.
    body = ET.SubElement(tank_body, 'body', name='fuel_door', pos='.085 0 .405',
                         euler=f'0 {initial_angle} 0')
    ET.SubElement(body, 'joint', name='fuel_door_hinge', type='hinge',
                  axis='0 1 0', limited='true', range=f'0 {MAX_ANGLE}',
                  ref=str(initial_angle),
                  damping='.015', frictionloss='.002')
    ET.SubElement(body, 'inertial', pos='-.085 0 0', mass='.15',
                  diaginertia='.0003613625 .0003613625 .0007225')
    for name, group, contact in [('visual', '1', '0'), ('collision', '0', '1')]:
        ET.SubElement(body, 'geom', name=f'fuel_door_{name}', type='box',
                      pos='-.085 0 0', size='.085 .085 .0015',
                      rgba='.2 .42 .65 1', group=group, contype=contact,
                      conaffinity=contact, density='0')
    ET.SubElement(body, 'site', name='fuel_door_pull_point', pos='-.16 0 .0015',
                  size='.004', rgba='1 .6 .1 1')

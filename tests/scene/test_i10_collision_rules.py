"""Adjacent exclusions survive generation without disabling unrelated collisions."""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'tools'))
from i10_collision_rules import adjacent_pairs, add_adjacent_exclusions
from fuel_cap_task.grasp_contact import ContactExplorer
from fuel_cap_task.arm_ik import ArmIKPreview, IKSettings


class CollisionRuleTests(unittest.TestCase):
    def test_all_six_srdf_adjacent_pairs_present_in_each_generated_model(self):
        expected={frozenset(p) for p in adjacent_pairs()}
        self.assertEqual(len(expected),6)
        for name in ('assets/robots/aubo_i10/mjcf/aubo_i10.xml',
                     'assets/scenes/i10_fuel_cap.xml',
                     'assets/scenes/i10_rh56_fuel_cap_preview.xml'):
            root=ET.parse(ROOT/name).getroot()
            actual={frozenset((e.get('body1'),e.get('body2'))) for e in root.findall('./contact/exclude')}
            self.assertTrue(expected <= actual)
            self.assertNotIn(frozenset(('base_link','upperArm_Link')),actual)
            count=len(root.findall('./contact/exclude'))
            add_adjacent_exclusions(root)
            self.assertEqual(count,len(root.findall('./contact/exclude')))
            m=mujoco.MjModel.from_xml_path(str(ROOT/name))
            d=mujoco.MjData(m);mujoco.mj_forward(m,d)
            for c in d.contact:
                bodies=frozenset(m.body(m.geom_bodyid[g]).name for g in c.geom)
                self.assertNotIn(bodies,expected)

    def test_local_hand_still_compiles_and_nonadjacent_arm_contacts_remain(self):
        local=ContactExplorer().scene.model
        self.assertEqual(local.nexclude,17)
        p=ArmIKPreview(ROOT/'artifacts/manual_grasp/user_selected_20260930.json',IKSettings(seeds=2))
        # Known colliding branch from the previous IK audit, not an excluded pair.
        import json
        report=json.loads((ROOT/'artifacts/arm_ik/current_candidate.json').read_text())
        bad=next(s for s in report['solutions'] if s['interference_count']>0)
        p.show(bad)
        pairs={frozenset(c['geoms']) for c in p.contacts()}
        self.assertIn(frozenset(('foreArm_Link_collision_0','wrist2_Link_collision_0')),pairs)


if __name__=='__main__': unittest.main()

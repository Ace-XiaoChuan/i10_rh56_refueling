"""独立核对双臂位姿、几何高度和原模型保留情况。"""

from copy import deepcopy
import json
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from build_dual_arm_model import ARM_JOINTS, CONFIG, ROOT, build, derive


def fk(robot, values=None):
    """用 SciPy 旋转计算 URDF FK，独立于生成器的欧拉角转换。"""
    frames = {"world": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        previous = len(pending)
        for joint in pending[:]:
            parent, child = (joint.find(tag).get("link") for tag in ("parent", "child"))
            if parent not in frames:
                continue
            origin = joint.find("origin")
            pose = np.eye(4)
            if origin is not None:
                pose[:3, 3] = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
                pose[:3, :3] = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")).as_matrix()
            if joint.get("type") == "revolute":
                axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
                pose[:3, :3] = pose[:3, :3] @ Rotation.from_rotvec(axis*(values or {}).get(joint.get("name"), 0.)).as_matrix()
            frames[child] = frames[parent] @ pose
            pending.remove(joint)
        if len(pending) == previous:
            raise AssertionError("URDF 存在断链或环")
    return frames


class DualArmModelTest(unittest.TestCase):
    """覆盖坐标方向、总高、两端安装中心和原有参数保护。"""

    @classmethod
    def setUpClass(cls):
        cls.files, cls.report = build()
        cls.robot = ET.fromstring(cls.files["robot.urdf"])
        cls.config = json.loads(CONFIG.read_text())
        cls.frames = fk(cls.robot)
        cls.source = ET.parse(ROOT/cls.config["source_robot"]).getroot()

    def test_measured_transform_and_gravity_direction(self):
        """从生成 URDF 反算用户位姿，重力须指向世界 -Z。"""
        right, left = self.frames["base_link"], self.frames["left_base_link"]
        actual = np.linalg.inv(right) @ left
        np.testing.assert_allclose(actual[:3, 3], [.0027, .0025, -.3034], atol=1e-14)
        np.testing.assert_allclose(actual[:3, :3], Rotation.from_euler("xyz", [3.1388, -.0008, -3.1168]).as_matrix(), atol=1e-14)
        gravity = np.array([-.10527343775827445, 9.799205557066411, -.06699219104753168])
        np.testing.assert_allclose(left[:3, :3] @ gravity, [0, 0, -9.8], atol=1e-12)

    def test_cylinder_faces_and_total_height(self):
        """独立按轴线端点和圆盘支撑范围核对两安装面和 1.27 m 总高。"""
        frame = self.robot.find("link[@name='t_frame']")
        bar = frame.find("visual[@name='t_crossbar_visual']")
        center = np.fromstring(bar.find("origin").get("xyz"), sep=" ")
        R = Rotation.from_euler("xyz", np.fromstring(bar.find("origin").get("rpy"), sep=" ")).as_matrix()
        cylinder = bar.find("geometry/cylinder")
        length, radius = float(cylinder.get("length")), float(cylinder.get("radius"))
        np.testing.assert_allclose(length, .30342231295671057, atol=1e-15)
        self.assertEqual(radius, .075)
        for sign, base in [(-1, "base_link"), (1, "left_base_link")]:
            np.testing.assert_allclose(center+sign*length/2*R[:, 2], self.frames[base][:3, 3], atol=1e-13)
        radial_height = radius*np.hypot(R[2, 0], R[2, 1])
        self.assertAlmostEqual(center[2]+length/2*abs(R[2, 2])+radial_height, 1.27, places=13)
        post = frame.find("visual[@name='t_post_visual']")
        self.assertEqual(float(post.find("geometry/cylinder").get("radius")), .075)
        post_z = float(post.find("origin").get("xyz").split()[2])
        post_length = float(post.find("geometry/cylinder").get("length"))
        self.assertAlmostEqual(post_z-post_length/2, 0, places=14)
        self.assertAlmostEqual(post_z+post_length/2, center[2], places=14)

    def test_original_right_chain_is_unchanged(self):
        """除基座安装关节外，右臂和末端每个原 XML 元素必须相同。"""
        for tag in ("link", "joint"):
            for element in self.source.findall(tag):
                if element.get("name") == "base_link_fixed":
                    continue
                actual = self.robot.find(f"{tag}[@name='{element.get('name')}']")
                # XML 缩进不属于参数；规范化后比较全部属性和子元素。
                expected = deepcopy(element)
                for root in (expected, actual):
                    for node in root.iter():
                        node.text = node.tail = None
                self.assertEqual(ET.tostring(actual), ET.tostring(expected))

    def test_both_arms_preserve_fk_at_multiple_configurations(self):
        """同一组局部关节角下，两臂基座到法兰的 FK 应与原臂一致。"""
        for angles in ([0]*6, [.2, -.5, 1.2, -.7, .3, -.1], [-1.1, .6, -.8, .4, -1.3, 2.0]):
            right_values = dict(zip(ARM_JOINTS, angles))
            combined = right_values | {"left_"+k: v for k, v in right_values.items()}
            original, frames = fk(self.source, right_values), fk(self.robot, combined)
            expected = np.linalg.inv(original["base_link"]) @ original["wrist3_Link"]
            for prefix in ("", "left_"):
                actual = np.linalg.inv(frames[prefix+"base_link"]) @ frames[prefix+"wrist3_Link"]
                np.testing.assert_allclose(actual, expected, atol=1e-13)

    def test_srdf_preserves_right_semantics_and_cross_arm_checks(self):
        """原有语义保留，跨臂碰撞不作整体豁免。"""
        srdf = ET.fromstring(self.files["robot.srdf"])
        self.assertIsNotNone(srdf.find("group[@name='left_arm']/chain[@tip_link='left_wrist3_Link']"))
        self.assertIsNotNone(srdf.find("group[@name='arm']/chain[@tip_link='base']"))
        for exclusion in srdf.findall("disable_collisions"):
            a, b = exclusion.get("link1"), exclusion.get("link2")
            if a != "t_frame" and b != "t_frame":
                self.assertEqual(a.startswith("left_"), b.startswith("left_"))


if __name__ == "__main__":
    unittest.main()

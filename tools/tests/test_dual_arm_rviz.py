"""通过 ROS 资源加载器验证双臂查看器实际发布的网格 URI；需先加载 ROS 环境。"""

from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import resource_retriever

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from view_dual_arm import description, rviz_description


class RvizResourcesTest(unittest.TestCase):
    """检查发送给 RViz 的每个网格确实可被 ROS 读取。"""

    def test_all_published_meshes_can_be_retrieved(self):
        """读取真实 STL 字节，避免仅检查文件存在却遗漏 URI 协议。"""
        robot, _ = description()
        before = ET.tostring(robot)
        published = ET.fromstring(rviz_description(robot))
        resources = {mesh.get("filename") for mesh in published.iter("mesh")}
        expected = {Path(mesh.get("filename")).as_uri(): Path(mesh.get("filename")).read_bytes()
                    for mesh in robot.iter("mesh")}
        self.assertEqual(len(resources), 33)
        for uri in resources:
            with self.subTest(mesh=uri):
                self.assertEqual(resource_retriever.get(uri), expected[uri])
        self.assertEqual(ET.tostring(robot), before)


if __name__ == "__main__":
    unittest.main()

"""用独立字节布局核对 PCD 字段解析、颜色位模式和损坏数据拒绝。"""

from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from fuel_cap_task.pcd_point_cloud import load_binary_pcd


class PcdTests(unittest.TestCase):
    """已知坐标和颜色来自手工打包，不以读取器自身输出作预期值。"""

    def load(self, payload, *, color_type="U", points=2, width=2, data="binary"):
        """创建隔离的临时输入；调用结束后释放文件，读取结果仍保留内容。"""
        header = (f"VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F {color_type}\n"
                  f"WIDTH {width}\nHEIGHT 1\nPOINTS {points}\nDATA {data}\n").encode()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "sample.pcd"
            path.write_bytes(header + payload)
            return load_binary_pcd(path)

    def test_exact_xyz_and_unsigned_rgb(self):
        """0x123456 必须拆为 [18,52,86]；缺省 COUNT 按每字段一个元素解释。"""
        cloud = self.load(struct.pack("<fffIfffI", 12, -3, 600, 0x123456, 8, 9, 10, 0xFF0080))
        np.testing.assert_array_equal(cloud.xyz_raw, [[12, -3, 600], [8, 9, 10]])
        np.testing.assert_array_equal(cloud.rgb, [[18, 52, 86], [255, 0, 128]])

    def test_float_rgb_uses_bits_and_keeps_nonfinite_xyz(self):
        """NaN 坐标不被偷偷删除；浮点声明的颜色仍按位读取。"""
        cloud = self.load(struct.pack("<fffIfffI", np.nan, 2, 3, 0xFF0000, 4, 5, 6, 0x0000FF),
                          color_type="F")
        self.assertTrue(np.isnan(cloud.xyz_raw[0, 0]))
        self.assertEqual(len(cloud.xyz_raw), 2)
        np.testing.assert_array_equal(cloud.rgb, [[255, 0, 0], [0, 0, 255]])

    def test_truncated_or_extra_payload_is_rejected(self):
        """缺一字节和多一字节都不能产生看似正常的点云。"""
        payload = struct.pack("<fffIfffI", 1, 2, 3, 0, 4, 5, 6, 0)
        for damaged in (payload[:-1], payload + b"x"):
            with self.subTest(length=len(damaged)), self.assertRaisesRegex(ValueError, "载荷长度"):
                self.load(damaged)

    def test_wrong_point_count_is_rejected(self):
        """组织尺寸与总点数不符时拒绝输入。"""
        with self.assertRaisesRegex(ValueError, "WIDTH"):
            self.load(bytes(32), width=3)

    def test_compressed_payload_is_not_misread_as_binary(self):
        """格式不支持时明确报错，防止把压缩字节误当坐标。"""
        with self.assertRaisesRegex(ValueError, "DATA binary"):
            self.load(bytes(32), data="binary_compressed")


if __name__ == "__main__":
    unittest.main()

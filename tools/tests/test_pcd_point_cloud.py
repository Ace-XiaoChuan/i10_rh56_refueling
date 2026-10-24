"""用独立字节与文本样本核对 PCD 字段、颜色位模式和损坏数据拒绝。"""

from pathlib import Path
import struct
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
from fuel_cap_task.pcd_point_cloud import load_binary_pcd, load_pcd


class PcdTests(unittest.TestCase):
    """已知坐标和颜色来自手工打包，不以读取器自身输出作预期值。"""

    def load(self, payload, *, color_type="U", points=2, width=2, data="binary", reader=load_pcd):
        """创建隔离的临时输入；调用结束后释放文件，读取结果仍保留内容。"""
        header = (f"VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F {color_type}\n"
                  f"WIDTH {width}\nHEIGHT 1\nPOINTS {points}\nDATA {data}\n").encode()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "sample.pcd"
            path.write_bytes(header + payload)
            return reader(path)

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

    def test_ascii_xyz_and_unsigned_rgb_keep_rows_and_nonfinite(self):
        """ASCII 接受科学计数法，保留 NaN 点和颜色整数，不因文本存储重排点。"""
        cloud = self.load(b"1.2e1 -3 600 1193046\n\nnan 9 10 16711808\n", data="ascii")
        np.testing.assert_array_equal(cloud.xyz_raw[0], [12, -3, 600])
        self.assertTrue(np.isnan(cloud.xyz_raw[1, 0]))
        np.testing.assert_array_equal(cloud.rgb, [[18, 52, 86], [255, 0, 128]])
        self.assertEqual(cloud.records.dtype["rgb"].kind, "u")

    def test_ascii_float_rgb_is_reinterpreted_as_bits(self):
        """文本中的浮点颜色按 TYPE F 还原 float32 后，再解释打包位模式。"""
        red, blue = (struct.unpack("<f", struct.pack("<I", bits))[0]
                     for bits in (0xFF0000, 0x0000FF))
        payload = f"1 2 3 {red!r}\n4 5 6 {blue!r}\n".encode()
        cloud = self.load(payload, color_type="F", data="ascii")
        np.testing.assert_array_equal(cloud.rgb, [[255, 0, 0], [0, 0, 255]])

    def test_ascii_malformed_rows_and_point_counts_are_rejected(self):
        """缺点、多点、缺列、多列及非法数值均不可产生正常点云。"""
        for payload in (b"1 2 3 0\n", b"1 2 3 0\n4 5 6 0\n7 8 9 0\n",
                        b"1 2 3\n4 5 6 0\n", b"1 2 3 0 99\n4 5 6 0\n",
                        b"bad 2 3 0\n4 5 6 0\n"):
            with self.subTest(payload=payload), self.assertRaisesRegex(ValueError, "ASCII"):
                self.load(payload, data="ascii")

    def test_ascii_integer_and_float_range_are_checked(self):
        """拒绝无符号负值、溢出整数、整数小数和有限浮点溢出，避免静默改值。"""
        for row in ("1 2 3 -1", "1 2 3 4294967296", "1 2 3 3.5", "1e100 2 3 0"):
            with self.subTest(row=row), self.assertRaisesRegex(ValueError, "ASCII"):
                self.load((row + "\n4 5 6 0\n").encode(), data="ascii")

    def test_ascii_vector_attribute_and_uint64_keep_declared_types(self):
        """COUNT 多元素字段和超过 float64 精确整数范围的编号都应完整保留。"""
        content = (b"VERSION 0.7\nFIELDS x y z normal id\nSIZE 4 4 4 4 8\n"
                   b"TYPE F F F F U\nCOUNT 1 1 1 3 1\nWIDTH 1\nHEIGHT 1\n"
                   b"POINTS 1\nDATA ascii\n1 2 3 0 0 1 18446744073709551615\n")
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "vector.pcd"
            path.write_bytes(content)
            cloud = load_pcd(path)
        np.testing.assert_array_equal(cloud.records["normal"], [[0, 0, 1]])
        self.assertEqual(int(cloud.records["id"][0]), 18446744073709551615)
        self.assertIsNone(cloud.rgb)

    def test_legacy_binary_entry_point_remains_available(self):
        """旧二进制入口继续读取原格式，并明确拒绝 ASCII。"""
        payload = struct.pack("<fffIfffI", 1, 2, 3, 0x123456, 4, 5, 6, 0)
        cloud = self.load(payload, reader=load_binary_pcd)
        np.testing.assert_array_equal(cloud.xyz_raw, [[1, 2, 3], [4, 5, 6]])
        with self.assertRaisesRegex(ValueError, "load_pcd"):
            self.load(b"1 2 3 0\n4 5 6 0\n", data="ascii", reader=load_binary_pcd)


if __name__ == "__main__":
    unittest.main()

"""核对安装文件的 RECORD 摘要并重建离线 wheel，供本次环境迁移使用。"""

import base64
import csv
from email.parser import Parser
import hashlib
import importlib.metadata as metadata
import io
import json
from pathlib import Path
import stat
import sys
import zipfile


def repack(name, output):
    """保留包文件及 wheel 元数据；入口脚本由安装器重新生成。"""
    dist = metadata.distribution(name)
    wheel = Parser().parsestr(dist.read_text("WHEEL"))
    dist_info = next(p.parts[0] for p in dist.files if str(p).endswith(".dist-info/WHEEL"))
    tag = wheel.get_all("Tag")[0]
    destination = output / f"{dist_info[:-len('.dist-info')]}-{tag}.whl"
    rows, checked, skipped = [], 0, []

    # 只接受原安装 RECORD 能证明完整性的文件，不携带旧入口绝对路径。
    with zipfile.ZipFile(destination, "w", compression=zipfile.ZIP_DEFLATED) as archive:
        for entry in dist.files:
            path = str(entry)
            if (".." in entry.parts or "__pycache__" in entry.parts
                    or path.endswith(("/RECORD", "/INSTALLER", "/REQUESTED", "/direct_url.json"))):
                skipped.append(path)
                continue
            source_file = Path(dist.locate_file(entry))
            data = source_file.read_bytes()
            if entry.hash is None:
                raise ValueError(f"安装文件缺少原摘要：{path}")
            old = base64.urlsafe_b64encode(hashlib.new(entry.hash.mode, data).digest()).decode().rstrip("=")
            if old != entry.hash.value or (entry.size is not None and len(data) != entry.size):
                raise ValueError(f"安装文件与原 RECORD 不符：{path}")
            digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).decode().rstrip("=")
            info = zipfile.ZipInfo(path)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.external_attr = (stat.S_IFREG | stat.S_IMODE(source_file.stat().st_mode)) << 16
            archive.writestr(info, data)
            rows.append((path, f"sha256={digest}", str(len(data))))
            checked += 1
        record = f"{dist_info}/RECORD"
        buffer = io.StringIO(newline="")
        writer = csv.writer(buffer, lineterminator="\n")
        writer.writerows(rows + [(record, "", "")])
        archive.writestr(record, buffer.getvalue())
    return {"name": dist.metadata["Name"], "version": dist.version,
            "source_environment": sys.prefix, "wheel": str(destination),
            "sha256": hashlib.sha256(destination.read_bytes()).hexdigest(),
            "checked_files": checked, "skipped_installation_files": skipped,
            "source": "installed_files_verified_against_RECORD"}


if __name__ == "__main__":
    output = Path(sys.argv[1])
    output.mkdir(parents=True, exist_ok=True)
    reports = [repack(name, output) for name in sys.argv[3:]]
    Path(sys.argv[2]).write_text(json.dumps(reports, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(reports, ensure_ascii=False))

#!/usr/bin/env python3
"""从官方发布地址下载固定版本权重，保留 URL 与 SHA256。"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import urllib.request

DINO_REVISION = "a2bb814dd30d776dcf7e30523b00659f4f141c71"
SAM_COMMIT = "2b90b9f5ceec907a1c18123530e92e794ad901a4"
SAM_CONFIG = "configs/sam2.1/sam2.1_hiera_t.yaml"
DINO_FILES = ("config.json", "model.safetensors", "preprocessor_config.json",
              "special_tokens_map.json", "tokenizer.json", "tokenizer_config.json",
              "vocab.txt", "added_tokens.json")


def digest(path):
    """分块计算摘要，避免同时读入多个大权重。"""
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    root = args.output_dir.resolve()
    root.mkdir(parents=True, exist_ok=False)
    entries = [(f"grounding-dino-tiny/{name}",
                f"https://huggingface.co/IDEA-Research/grounding-dino-tiny/resolve/{DINO_REVISION}/{name}")
               for name in DINO_FILES]
    entries.append(("sam2.1_hiera_tiny.pt",
                    "https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt"))

    def download(item):
        name, url = item
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        request = urllib.request.Request(url, headers={"User-Agent": "fuel-cap-perception-setup"})
        with urllib.request.urlopen(request, timeout=120) as response, path.open("xb") as f:
            for chunk in iter(lambda: response.read(1024 * 1024), b""):
                f.write(chunk)
        print(f"downloaded {name}: {path.stat().st_size} bytes", flush=True)
        return name, {"url": url, "sha256": digest(path), "bytes": path.stat().st_size}

    # 独立文件并行下载；任一失败保留已下载文件，禁止自动续跑混入旧内容。
    with ThreadPoolExecutor(max_workers=3) as pool:
        files = dict(pool.map(download, entries))
    manifest = {"dino_model_id": "IDEA-Research/grounding-dino-tiny",
                "dino_revision": DINO_REVISION, "sam_source_commit": SAM_COMMIT,
                "sam_config": SAM_CONFIG, "sam_source_url": "https://github.com/facebookresearch/sam2",
                "download_code_sha256": digest(__file__), "files": files}
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


if __name__ == "__main__":
    main()

"""本地微调 Grounding DINO；仅提供二维框，延迟导入独立检测依赖。"""

import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import sys

import numpy as np


def digest(path):
    """分块核对来源，避免将整个权重文件读入内存。"""
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def bound_model(bundle):
    """核对已回传模型、BERT 和原始配置；不读取任何图片标注或旧预测。"""
    bundle = Path(bundle).resolve()
    settings_path = bundle / "cloud_config.json"
    request_path = bundle / "outputs/evaluate/request.json"
    settings = json.loads(settings_path.read_text())
    request = json.loads(request_path.read_text())
    if settings != request["settings"] or settings["classes"] != ["fuel cap"]:
        raise ValueError("模型配置与已评价的单类别实验不一致")
    checkpoint = bundle / "outputs/train/best_coco_bbox_mAP_epoch_4.pth"
    if digest(checkpoint) != request["checkpoint_sha256"]:
        raise ValueError("最佳权重与已评价模型不一致")
    sources_path = bundle / "models/model_sources.json"
    sources = json.loads(sources_path.read_text())
    files = {str(settings_path): digest(settings_path), str(request_path): digest(request_path),
             str(checkpoint): request["checkpoint_sha256"], str(sources_path): digest(sources_path)}
    for name, expected in sources["files"].items():
        if not name.startswith("bert/"):
            continue
        path = (bundle / "models" / name).resolve()
        if not path.is_relative_to(bundle / "models/bert") or digest(path) != expected:
            raise ValueError(f"BERT 来源不一致：{name}")
        files[str(path)] = expected
    vendor_path = bundle / "vendor/source_manifest.json"
    vendor = json.loads(vendor_path.read_text())
    files[str(vendor_path)] = digest(vendor_path)
    for name, entry in vendor.items():
        path = (bundle / "vendor" / name).resolve()
        if not path.is_relative_to(bundle / "vendor") or digest(path) != entry["sha256"]:
            raise ValueError(f"原始模型配置来源不一致：{name}")
        files[str(path)] = entry["sha256"]
    return settings, checkpoint, files


def candidate_arrays(raw, image_shape, threshold):
    """原图 XYXY 裁剪后按固定阈值保留全部候选，不选择最高分。"""
    height, width = image_shape
    boxes = np.array(raw["boxes_xyxy"], dtype=np.float32, copy=True)
    if boxes.size == 0:
        boxes = boxes.reshape(0, 4)
    scores = np.asarray(raw["scores"], dtype=np.float32)
    labels = np.asarray(raw["labels"])
    if (boxes.ndim != 2 or boxes.shape[1] != 4 or scores.shape != (len(boxes),)
            or labels.shape != scores.shape):
        raise ValueError("检测框、分数和类别数量不一致")
    if (not np.isfinite(boxes).all() or not np.isfinite(scores).all()
            or np.any((scores < 0) | (scores > 1)) or np.any(labels != 0)
            or not 0 < threshold < 1):
        raise ValueError("检测输出或阈值无效")
    if np.any(boxes[:, 2:] <= boxes[:, :2]):
        raise ValueError("原始检测框没有正面积")
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, width)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, height)
    if np.any(boxes[:, 2:] <= boxes[:, :2]):
        raise ValueError("裁剪后的检测框没有正面积")
    keep = scores >= threshold
    return boxes[keep], scores[keep]


class FinetunedDINO:
    """复用已训练结构和图像尺度；仅在独立 Python 进程加载 MMDetection。"""

    def __init__(self, bundle, *, threads=4):
        if type(threads) is not int or threads <= 0:
            raise ValueError("CPU 线程数须为正整数")
        settings, checkpoint, sources = bound_model(bundle)
        bundle = Path(bundle).resolve()
        # BERT 与权重已在本地，推理期间禁止访问远端模型服务。
        os.environ["HF_HUB_OFFLINE"] = "1"
        os.environ["TRANSFORMERS_OFFLINE"] = "1"
        os.environ["TOKENIZERS_PARALLELISM"] = "false"
        # 依赖可能在导入时写缓存，限定在独立环境内，不依赖用户主目录权限。
        os.environ["HF_HOME"] = str(Path(sys.prefix) / "cache/huggingface")
        os.environ["MPLCONFIGDIR"] = str(Path(sys.prefix) / "cache/matplotlib")
        import torch
        from mmengine.config import Config
        from mmengine.runner import load_checkpoint
        from mmdet.registry import MODELS
        from mmdet.utils import register_all_modules

        expected = {"torch": "2.1.2+cpu", "torchvision": "0.16.2+cpu", "mmcv": "2.1.0",
                    "mmengine": "0.10.7", "mmdet": "3.3.0", "numpy": "1.26.4",
                    "transformers": "4.38.2"}
        if sys.version_info[:2] != (3, 10) or any(
                importlib.metadata.version(name) != version for name, version in expected.items()):
            raise ValueError("检测进程须使用已固定的 Python 3.10 CPU 推理依赖")
        torch.set_num_threads(threads)
        register_all_modules(init_default_scope=True)
        config = Config.fromfile(str(bundle / "vendor/configs/grounding_dino/"
                                    "grounding_dino_swin-t_finetune_16xb2_1x_coco.py"))
        # 模型结构、类别与缩放沿用云端；图片入口不带标注。
        config.model.bbox_head.num_classes = len(settings["classes"])
        config.model.language_model.name = str(bundle / "models/bert")
        pipeline = [dict(type="LoadImageFromNDArray"),
                    dict(type="Resize", scale=tuple(settings["input_scale"]), keep_ratio=True),
                    dict(type="PackDetInputs", meta_keys=("img_id", "ori_shape", "img_shape",
                         "scale_factor", "text", "custom_entities"))]
        config.test_dataloader.dataset.pipeline = pipeline
        self.model = MODELS.build(config.model)
        loaded = load_checkpoint(self.model, str(checkpoint), map_location="cpu", strict=True)
        self.model.dataset_meta = loaded["meta"]["dataset_meta"]
        if list(self.model.dataset_meta["classes"]) != settings["classes"]:
            raise ValueError("权重内部类别与已确认类别不一致")
        del loaded
        self.model.cfg = config
        self.model.to("cpu").eval()
        self.torch, self.settings = torch, settings
        self.provenance = {
            "backend": "mmdetection_local_finetuned", "rerun_locally": True,
            "checkpoint": str(checkpoint), "checkpoint_sha256": sources[str(checkpoint)],
            "source_sha256": sources, "classes": settings["classes"],
            "input_scale": settings["input_scale"], "score_threshold": settings["score_threshold"],
            "device": "cpu", "dtype": "float32", "threads": threads, "python": sys.version,
            "python_executable": sys.executable,
            "environment": {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()},
            "input_color": "RGB 接口转 BGR 后由模型预处理转回 RGB",
            "pipeline": pipeline, "strict_weight_loading": True,
        }

    def predict(self, rgb):
        """uint8 RGB 输入；返回全部原图像素框及候选，不产生三维位姿。"""
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3 or 0 in rgb.shape:
            raise ValueError("输入须为非空 uint8 H×W×3 RGB")
        from mmdet.apis import inference_detector

        # MMDetection 图像数组接口接收 BGR；颜色转换只发生在此边界。
        bgr = rgb[..., ::-1].copy()
        with self.torch.inference_mode():
            sample = inference_detector(self.model, bgr, text_prompt=tuple(self.settings["classes"]),
                                        custom_entities=True)
        instances = sample.pred_instances.cpu()
        raw = {"boxes_xyxy": instances.bboxes.numpy().tolist(),
               "scores": instances.scores.numpy().tolist(), "labels": instances.labels.numpy().tolist()}
        boxes, scores = candidate_arrays(raw, rgb.shape[:2], self.settings["score_threshold"])
        return {"raw_predictions": raw, "boxes_xyxy": boxes.tolist(), "scores": scores.tolist(),
                "labels": ["fuel cap"] * len(boxes), "image_size_wh": [rgb.shape[1], rgb.shape[0]]}


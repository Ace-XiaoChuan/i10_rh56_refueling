"""文本定位与提示分割的单帧接口；不输出三维位姿或物理状态。"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class GroundedMasks:
    """框为原图像素 XYXY；掩膜为 N×H×W，两个分数各有含义。"""

    boxes_xyxy: np.ndarray
    detector_scores: np.ndarray
    labels: tuple[str, ...]
    masks: np.ndarray
    mask_quality_scores: np.ndarray
    status: str
    selected_index: int | None


def assemble_result(boxes, detector_scores, labels, masks, quality_scores, image_shape):
    """验证图像合同；空结果和多候选明确失效，不隐式挑选最高分。"""
    h, w = image_shape
    boxes = np.asarray(boxes, dtype=np.float32).reshape(-1, 4)
    scores = np.asarray(detector_scores, dtype=np.float32).reshape(-1)
    quality = np.asarray(quality_scores, dtype=np.float32).reshape(-1)
    masks = np.asarray(masks)
    n = len(boxes)
    if masks.dtype != np.bool_ or masks.shape != (n, h, w):
        raise ValueError("masks must be boolean N x H x W at original image size")
    if len(scores) != n or len(quality) != n or len(labels) != n:
        raise ValueError("boxes, scores, labels and masks have different counts")
    if not all(np.isfinite(x).all() for x in (boxes, scores, quality)):
        raise ValueError("non-finite model output")
    if np.any((scores < 0) | (scores > 1)):
        raise ValueError("detector scores outside 0..1")
    if n and (np.any(boxes[:, :2] < 0) or np.any(boxes[:, 2:] > [w, h])
              or np.any(boxes[:, 2:] <= boxes[:, :2])):
        raise ValueError("invalid pixel XYXY box")
    if n == 0:
        status, selected = "not_detected", None
    elif any(not mask.any() for mask in masks):
        status, selected = "empty_mask", None
    elif n > 1:
        status, selected = "ambiguous", None
    else:
        # 唯一候选不等于语义识别已验收，也不等于可用于机械臂执行。
        status, selected = "unique_candidate", 0
    return GroundedMasks(boxes, scores, tuple(labels), masks, quality, status, selected)


class GroundedSAM:
    """复用本地预训练模型；调用阶段不联网、不读取仿真真值。"""

    def __init__(self, dino_directory: Path, sam_checkpoint: Path, sam_config: str,
                 *, device: str = "cpu", threads: int = 4):
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        if threads <= 0:
            raise ValueError("threads must be positive")
        torch.set_num_threads(threads)
        self.torch, self.device = torch, torch.device(device)
        self.processor = AutoProcessor.from_pretrained(dino_directory, local_files_only=True)
        self.detector = AutoModelForZeroShotObjectDetection.from_pretrained(
            dino_directory, local_files_only=True, use_safetensors=True
        ).to(self.device).eval()
        # 首轮 float32，不照搬 bfloat16 示例，以兼容 CPU 与旧显卡。
        self.segmenter = SAM2ImagePredictor(build_sam2(
            sam_config, str(sam_checkpoint), device=device, mode="eval",
            apply_postprocessing=False,
        ))

    def predict(self, rgb: np.ndarray, prompt: str, *, box_threshold: float,
                text_threshold: float) -> GroundedMasks:
        """输入 uint8 RGB 与目标描述；输出原图坐标中的候选框和掩膜。"""
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("image must be uint8 H x W x 3 RGB")
        if not prompt.strip():
            raise ValueError("empty target description")
        if not all(0 < t < 1 for t in (box_threshold, text_threshold)):
            raise ValueError("thresholds must be in 0..1")
        text = prompt.strip().lower()
        if not text.endswith("."):
            text += "."
        h, w = rgb.shape[:2]
        from PIL import Image

        # 第一阶段：文本匹配与框后处理；Transformers 已转换成像素 XYXY。
        inputs = self.processor(images=Image.fromarray(rgb), text=text,
                                return_tensors="pt").to(self.device)
        with self.torch.inference_mode():
            outputs = self.detector(**inputs)
            detections = self.processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, threshold=box_threshold,
                text_threshold=text_threshold, target_sizes=[(h, w)],
            )[0]
            boxes = detections["boxes"].cpu().numpy()
            scores = detections["scores"].cpu().numpy()
            labels = detections["text_labels"]
            # 检测器可能输出略超出画幅的框；只裁剪坐标，不更换目标。
            boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, w)
            boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, h)

            # 第二阶段：同一 RGB 的每个候选框提示 SAM；保持候选索引对应。
            if len(boxes):
                self.segmenter.set_image(rgb)
                masks, quality, _ = self.segmenter.predict(
                    point_coords=None, point_labels=None, box=boxes,
                    multimask_output=False, return_logits=False,
                )
                masks = np.asarray(masks, dtype=bool).reshape(len(boxes), h, w)
                quality = np.asarray(quality).reshape(len(boxes))
            else:
                masks = np.empty((0, h, w), dtype=bool)
                quality = np.empty(0, dtype=np.float32)
        return assemble_result(boxes, scores, labels, masks, quality, (h, w))

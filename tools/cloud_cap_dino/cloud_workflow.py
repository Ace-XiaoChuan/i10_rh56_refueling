#!/usr/bin/env python3
"""云端 Grounding DINO 基线、试跑、训练及同场景验证的独立入口。"""

import argparse
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
import urllib.request

from common import digest, evaluate_predictions, read_json, write_json


ROOT = Path(__file__).resolve().parent


def verify_inputs():
    """读取相对路径，拒绝输入变化、跨划分复用或丢失负样本。"""
    from check_transfer import verify
    verify(ROOT)
    split = read_json(ROOT / "split.json")
    train = read_json(ROOT / "annotations/instances_train.json")
    val = read_json(ROOT / "annotations/instances_val.json")
    smoke = read_json(ROOT / "annotations/instances_smoke.json")
    train_ids = {row["id"] for row in train["images"]}
    val_ids = {row["id"] for row in val["images"]}
    smoke_ids = {row["id"] for row in smoke["images"]}
    if train_ids & val_ids or train_ids != set(split["train_ids"]) or val_ids != set(split["val_ids"]):
        raise ValueError("训练/验证划分不一致或重叠")
    if not smoke_ids <= train_ids or len(smoke_ids) != 2:
        raise ValueError("试跑图必须来自训练划分，且包含一张有盖、一张无盖")
    expected_ids = {row["id"] for row in read_json(ROOT / "annotations/instances_all.json")["images"]}
    if train_ids | val_ids != expected_ids:
        raise ValueError("划分没有覆盖完整数据")
    return read_json(ROOT / "cloud_config.json")


def environment():
    """保存实际版本，后续配置或依赖变化使旧基线不能复用。"""
    names = ("torch", "torchvision", "mmcv", "mmengine", "mmdet", "numpy", "transformers")
    return {"python": platform.python_version(),
            "packages": {name: importlib.metadata.version(name) for name in names}}


def check_backend():
    """验证编译算子的 CUDA 前向与反向，而不是只检查库能导入。"""
    import torch
    import mmcv
    import mmengine
    import mmdet
    from mmcv.ops import nms
    from mmcv.ops.multi_scale_deform_attn import MultiScaleDeformableAttnFunction
    assert sys.version_info[:2] == (3, 10)
    assert torch.__version__.split("+")[0] == "2.1.2" and torch.version.cuda == "12.1"
    assert (mmcv.__version__, mmengine.__version__, mmdet.__version__) == ("2.1.0", "0.10.7", "3.3.0")
    assert torch.cuda.is_available()
    boxes = torch.tensor([[0, 0, 10, 10], [1, 1, 9, 9]], dtype=torch.float32, device="cuda")
    _, kept = nms(boxes, torch.tensor([0.9, 0.8], device="cuda"), 0.5)
    assert kept.tolist() == [0]
    # 使用最小单层空间特征检查 Grounding DINO 所依赖的可变形注意力算子。
    values = torch.randn(1, 4, 2, 4, device="cuda", requires_grad=True)
    shapes = torch.tensor([[2, 2]], dtype=torch.long, device="cuda")
    starts = torch.tensor([0], dtype=torch.long, device="cuda")
    locations = torch.rand(1, 3, 2, 1, 2, 2, device="cuda", requires_grad=True)
    weights = torch.full((1, 3, 2, 1, 2), 0.5, device="cuda", requires_grad=True)
    result = MultiScaleDeformableAttnFunction.apply(values, shapes, starts, locations, weights, 64)
    result.sum().backward()
    torch.cuda.synchronize()
    assert torch.isfinite(result).all()
    assert all(value.grad is not None and torch.isfinite(value.grad).all()
               for value in (values, locations, weights))
    return {"status": "passed", "environment": environment(),
            "gpu": torch.cuda.get_device_name(0), "nms_cuda": True,
            "deformable_attention_cuda_forward_backward": True}


def fetch_models(destination, settings):
    """下载固定来源；失败保留目录，不自动覆盖或混入旧文件。"""
    destination.mkdir(parents=True, exist_ok=False)
    checkpoint = destination / "grounding_dino_pretrained.pth"
    with urllib.request.urlopen(settings["pretrained_url"], timeout=120) as source, checkpoint.open("xb") as target:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            target.write(chunk)
    from transformers import AutoTokenizer, BertModel
    bert_dir = destination / "bert"
    tokenizer = AutoTokenizer.from_pretrained(settings["bert_model_id"], revision=settings["bert_revision"])
    bert = BertModel.from_pretrained(settings["bert_model_id"], revision=settings["bert_revision"],
                                    add_pooling_layer=False, use_safetensors=True)
    tokenizer.save_pretrained(bert_dir)
    bert.save_pretrained(bert_dir, safe_serialization=True)
    files = {path.relative_to(destination).as_posix(): digest(path)
             for path in sorted(destination.rglob("*")) if path.is_file()}
    write_json(destination / "model_sources.json", {"pretrained_url": settings["pretrained_url"],
               "bert_model_id": settings["bert_model_id"], "bert_revision": settings["bert_revision"],
               "files": files})
    print("预训练模型与固定版本 BERT 已下载，实际摘要已保存。")


def verify_models(models, settings):
    """确保本次初始化始终使用同一份预训练模型。"""
    sources = read_json(models / "model_sources.json")
    if any(sources[key] != settings[key] for key in ("pretrained_url", "bert_model_id", "bert_revision")):
        raise ValueError("模型下载来源与配置不一致")
    for name, expected in sources["files"].items():
        if digest(models / name) != expected:
            raise ValueError(f"预训练模型文件摘要不一致：{name}")


def build_config(settings, models, output, phase, checkpoint=None):
    """沿用官方模型结构，覆盖单卡参数、数据路径及负样本保留规则。"""
    from mmengine.config import Config
    cfg = Config.fromfile(str(ROOT / "vendor/configs/grounding_dino/grounding_dino_swin-t_finetune_16xb2_1x_coco.py"))
    cfg.model.bbox_head.num_classes = len(settings["classes"])
    cfg.model.language_model.name = str(models / "bert")
    cfg.load_from = str(checkpoint or models / "grounding_dino_pretrained.pth")
    cfg.work_dir = str(output)
    cfg.resume = False
    cfg.launcher = "none"
    cfg.randomness = dict(seed=settings["seed"], deterministic=False)
    cfg.env_cfg.cudnn_benchmark = False
    scale = tuple(settings["input_scale"])
    train_pipeline = [dict(type="LoadImageFromFile", backend_args=None),
                      dict(type="LoadAnnotations", with_bbox=True),
                      dict(type="Resize", scale=scale, keep_ratio=True),
                      dict(type="RandomFlip", prob=settings["flip_probability"]),
                      dict(type="PackDetInputs", meta_keys=("img_id", "img_path", "ori_shape", "img_shape",
                           "scale_factor", "flip", "flip_direction", "text", "custom_entities"))]
    val_pipeline = [dict(type="LoadImageFromFile", backend_args=None),
                    dict(type="Resize", scale=scale, keep_ratio=True),
                    dict(type="LoadAnnotations", with_bbox=True),
                    dict(type="PackDetInputs", meta_keys=("img_id", "img_path", "ori_shape", "img_shape",
                         "scale_factor", "text", "custom_entities"))]
    metadata = dict(classes=tuple(settings["classes"]), palette=[(0, 220, 80)])
    for loader, annotation, pipeline in [(cfg.train_dataloader, "instances_train.json", train_pipeline),
                                         (cfg.val_dataloader, "instances_val.json", val_pipeline)]:
        loader.batch_size = settings["batch_size"]
        loader.num_workers = settings["workers"]
        loader.persistent_workers = settings["workers"] > 0
        loader.dataset.update(data_root=str(ROOT) + "/", metainfo=metadata,
                              ann_file="annotations/" + annotation, data_prefix=dict(img="images/"),
                              pipeline=pipeline, return_classes=True)
        loader.dataset.filter_cfg = dict(filter_empty_gt=False, min_size=1)
    cfg.test_dataloader = cfg.val_dataloader.copy()
    cfg.val_evaluator = dict(type="CocoMetric", ann_file=str(ROOT / "annotations/instances_val.json"),
                            metric="bbox", classwise=True, outfile_prefix=str(output / "predictions"))
    cfg.test_evaluator = cfg.val_evaluator.copy()
    cfg.train_cfg = dict(type="EpochBasedTrainLoop", max_epochs=settings["epochs"], val_interval=1)
    cfg.val_cfg, cfg.test_cfg = dict(type="ValLoop"), dict(type="TestLoop")
    cfg.optim_wrapper = dict(type="OptimWrapper", optimizer=dict(type="AdamW", lr=settings["learning_rate"],
                             weight_decay=settings["weight_decay"]),
                             clip_grad=dict(max_norm=settings["gradient_clip_norm"], norm_type=2,
                                            error_if_nonfinite=True),
                             paramwise_cfg=dict(custom_keys={"absolute_pos_embed": dict(decay_mult=0.),
                                  "backbone": dict(lr_mult=settings["backbone_lr_multiplier"]),
                                  "language_model": dict(lr_mult=0.)}))
    cfg.param_scheduler = [dict(type="LinearLR", start_factor=settings["warmup_start_factor"],
                               by_epoch=False, begin=0, end=settings["warmup_steps"]),
                           dict(type="MultiStepLR", by_epoch=True, begin=0, end=settings["epochs"],
                                milestones=[settings["decay_epoch"]], gamma=settings["decay_factor"])]
    cfg.auto_scale_lr = dict(enable=False, base_batch_size=1)
    cfg.default_hooks.checkpoint = dict(type="CheckpointHook", interval=1, max_keep_ckpts=2,
                                        save_best="coco/bbox_mAP", rule="greater")
    cfg.default_hooks.logger = dict(type="LoggerHook", interval=1)
    cfg.custom_hooks = [dict(type="CapTrainingAuditHook")]
    if phase == "smoke":
        cfg.train_dataloader.dataset.ann_file = "annotations/instances_smoke.json"
        cfg.train_dataloader.sampler = dict(type="DefaultSampler", shuffle=False)
        cfg.train_dataloader.batch_sampler = None
        cfg.train_dataloader.num_workers = 0
        cfg.train_dataloader.persistent_workers = False
        cfg.train_cfg = dict(type="IterBasedTrainLoop", max_iters=2, val_interval=3)
        cfg.val_dataloader = cfg.val_cfg = cfg.val_evaluator = None
        cfg.param_scheduler = None
        cfg.default_hooks.checkpoint = None
        cfg.default_hooks.param_scheduler = None
        cfg.log_processor.by_epoch = False
    return cfg


def run_experiment(args, settings):
    """按来源相同的顺序执行，不续跑、不覆盖历史目录。"""
    import torch
    from mmengine.runner import Runner
    from mmdet.utils import register_all_modules
    register_all_modules(init_default_scope=True)
    import cloud_hooks  # 导入项目训练审计 Hook，供 MMDetection 注册表查找。
    models = args.models.resolve()
    verify_models(models, settings)
    backend = read_json(ROOT / "outputs/check/result.json")
    if backend["status"] != "passed" or backend["environment"] != environment():
        raise ValueError("须先在相同环境完成 check 算子检查")
    recipe = {"bundle": read_json(ROOT / "checksums.json"), "models": read_json(models / "model_sources.json"),
              "environment": environment()}
    import hashlib
    recipe_sha = hashlib.sha256(json.dumps(recipe, sort_keys=True).encode()).hexdigest()
    if args.phase == "train":
        for required in ("baseline", "smoke"):
            previous = read_json(ROOT / "outputs" / required / "result.json")
            if previous["status"] != "passed" or previous["recipe_sha256"] != recipe_sha:
                raise ValueError(f"须先在相同输入与环境完成 {required}")
    if args.phase == "evaluate" and args.checkpoint is None:
        raise ValueError("evaluate 必须显式提供 --checkpoint")
    if args.checkpoint is not None and args.phase != "evaluate":
        raise ValueError("只有 evaluate 可以指定微调 checkpoint；训练必须从原预训练权重开始")
    output = args.output.resolve() if args.output else ROOT / "outputs" / args.phase
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "request.json", {"phase": args.phase, "recipe_sha256": recipe_sha,
               "environment": environment(), "settings": settings,
               "checkpoint_sha256": digest(args.checkpoint) if args.checkpoint else None})
    try:
        cfg = build_config(settings, models, output, args.phase, args.checkpoint)
        cfg.dump(str(output / "effective_config.py"))
        subprocess.run([sys.executable, "-m", "pip", "freeze"], check=True,
                       stdout=(output / "pip_freeze.txt").open("w", encoding="utf-8"))
        started = time.perf_counter()
        runner = Runner.from_cfg(cfg)
        if args.phase in ("baseline", "evaluate"):
            metrics = runner.test()
            diagnosis = evaluate_predictions(read_json(ROOT / "annotations/instances_val.json"),
                          read_json(output / "predictions.bbox.json"), settings["score_threshold"],
                          settings["diagnostic_iou_threshold"])
            write_json(output / "localization_diagnosis.json", diagnosis)
        else:
            runner.train()
            metrics = read_json(output / "training_audit.json")
            if args.phase == "smoke":
                smoke_ids = {row["id"] for row in read_json(ROOT / "annotations/instances_smoke.json")["images"]}
                if set(metrics["image_ids_seen"]) != smoke_ids:
                    raise ValueError("试跑未覆盖有盖与无盖两张训练图")
        torch.cuda.synchronize()
        write_json(output / "result.json", {"status": "passed", "phase": args.phase,
                   "recipe_sha256": recipe_sha, "elapsed_seconds": time.perf_counter() - started,
                   "metrics": metrics, "scope": "同场景首轮实验，不代表跨批次或实物自动语义验收"})
    except Exception as error:
        write_json(output / "failure.json", {"status": "failed", "error": repr(error)})
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("verify", "check", "fetch", "baseline", "smoke", "train", "evaluate"))
    parser.add_argument("--models", type=Path, default=ROOT / "models")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    args = parser.parse_args()
    settings = verify_inputs()
    if args.phase == "verify":
        print("固定训练/验证划分与来源检查通过。")
    elif args.phase == "check":
        result = check_backend()
        output = args.output.resolve() if args.output else ROOT / "outputs/check"
        output.mkdir(parents=True, exist_ok=False)
        write_json(output / "result.json", result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    elif args.phase == "fetch":
        fetch_models(args.models.resolve(), settings)
    else:
        run_experiment(args, settings)


if __name__ == "__main__":
    main()

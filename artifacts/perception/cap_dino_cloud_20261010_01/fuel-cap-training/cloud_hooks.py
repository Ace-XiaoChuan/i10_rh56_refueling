"""冻结文本分支并记录真实训练损失、参数更新、耗时和显存。"""

import json
from pathlib import Path
import time

import torch
from mmengine.hooks import Hook
from mmdet.registry import HOOKS


@HOOKS.register_module()
class CapTrainingAuditHook(Hook):
    """观察一个视觉参数的真实变化；试跑不能只检查日志出现 loss。"""

    def before_train(self, runner):
        model = runner.model
        model.language_model.requires_grad_(False)
        model.language_model.eval()
        self.probe_name, self.probe = next(
            (name, value) for name, value in model.named_parameters()
            if name.startswith("neck.") and value.requires_grad)
        self.initial_probe = self.probe.detach().clone()
        self.changed = False
        self.image_ids = []
        torch.cuda.reset_peak_memory_stats()

    def before_train_iter(self, runner, batch_idx, data_batch=None):
        # 每轮 model.train() 会影响子模块模式，因此每步恢复冻结 BERT 的 eval。
        runner.model.language_model.eval()
        torch.cuda.synchronize()
        self.started = time.perf_counter()

    def after_train_iter(self, runner, batch_idx, data_batch=None, outputs=None):
        torch.cuda.synchronize()
        losses = {name: float(value.detach().cpu()) if torch.is_tensor(value) else float(value)
                  for name, value in outputs.items()}
        if not all(torch.isfinite(torch.tensor(value)).item() for value in losses.values()):
            raise ValueError("训练损失包含非有限值")
        self.changed = self.changed or not torch.equal(self.probe.detach(), self.initial_probe)
        ids = [int(sample.img_id) for sample in data_batch["data_samples"]]
        self.image_ids.extend(ids)
        record = {"iteration": runner.iter + 1, "image_ids": ids, "losses": losses,
                  "step_seconds": time.perf_counter() - self.started,
                  "peak_cuda_allocated_mib": torch.cuda.max_memory_allocated() / 1024**2,
                  "visual_probe": self.probe_name, "visual_probe_changed": self.changed}
        with (Path(runner.work_dir) / "training_steps.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def after_train(self, runner):
        if not self.changed:
            raise ValueError("未观测到视觉参数更新，不能把该次运行视为训练通过")
        path = Path(runner.work_dir) / "training_audit.json"
        path.write_text(json.dumps({"visual_parameter_updated": self.changed,
                                    "bert_frozen": True, "image_ids_seen": self.image_ids,
                                    "peak_cuda_allocated_mib": torch.cuda.max_memory_allocated() / 1024**2},
                                   ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

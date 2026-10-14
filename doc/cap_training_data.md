# 油箱盖定位数据

定位标签包住整个可见盖体，包括握梁，排除外围凹槽、门板和连接带。无盖图使用空标注，遮挡图需要明确目标是否可见后再纳入数据。

项目中的 `doc/assembly_contract.md` 维护当前图片范围、数量和训练划分，训练配置为 `config/grounding_dino_cloud.json`。本文也随数据包保存为 README。

## 打包原图与标注

在项目根目录运行，输出到新目录：

```bash
python3 tools/package_cap_training_data.py \
  --annotation-dir artifacts/perception/cap_annotations_20261010_004032 \
  --output-dir artifacts/perception/my_data_package
```

生成 `fuel-cap-data.zip`。这个包保存完整原图和未划分的 COCO 标注；带固定划分和训练程序的包按项目 `doc/cap_dino_cloud_training.md` 准备。

| 包内路径 | 内容 |
| --- | --- |
| `images/` | 原始图片 |
| `annotations/instances_all.json` | COCO 检测标注 |
| `annotations/source_boxes/` | 所选人工框记录 |
| `metadata/` | 图片、标注和用户确认的来源记录 |
| `transfer_manifest.json` | 相对路径及图片摘要 |
| `model_reference.json` | 预训练模型来源 |
| `checksums.json`、`check_transfer.py` | 文件完整性检查 |
| `index.html` | 原图与框选预览 |

训练读图使用 `images/`，标注使用 `annotations/instances_all.json`。元数据中的原项目路径用于来源查询，搬到另一台电脑后无需重建这些路径。

## 检查传输

解压后进入 `fuel-cap-data/`：

```bash
python3 check_transfer.py
```

程序核对文件摘要、图片与标注关联、框范围和无盖空标注。`index.html` 可直接在浏览器中查看。

## 扩充数据

补拍不同距离、角度、光照和盖体朝向，并保留拍摄批次。有盖与无盖都要覆盖远景条件。独立测试使用新的拍摄批次，训练、选模型和调阈值使用训练/验证集。

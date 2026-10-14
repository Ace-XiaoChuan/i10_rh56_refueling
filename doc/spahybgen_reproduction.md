# SpaHybGen 原示例

运行上游默认体素观测与 Robotiq-2F 示例，查看空间接触预测和抓姿优化。油箱盖及 RH56 的开发进度见[项目说明](assembly_contract.md#spahybgen-与-rh56)。

## 源码与环境

源码位于 `dependencies/SpaHybGen/`，固定提交 `39e5794506bbbf2fd41804f73c93b7b393be550e`。使用独立 Python 3.8 环境 `.venv-spahybgen`，依赖见 `dependencies/spahybgen-reproduction.txt`。

建立新环境时，在项目根目录执行：

```bash
git clone https://github.com/wangzivector/SpaHybGen.git dependencies/SpaHybGen
git -C dependencies/SpaHybGen checkout 39e5794506bbbf2fd41804f73c93b7b393be550e
python3 -m venv /tmp/spahybgen-bootstrap
/tmp/spahybgen-bootstrap/bin/python -m pip install --cache-dir .cache/spahybgen/pip uv==0.12.23
export UV_CACHE_DIR="$PWD/.cache/spahybgen/uv"
export UV_PYTHON_INSTALL_DIR="$PWD/.cache/spahybgen/python"
export UV_PYTHON_BIN_DIR="$PWD/.cache/spahybgen/bin"
/tmp/spahybgen-bootstrap/bin/uv python install 3.8.20
/tmp/spahybgen-bootstrap/bin/uv venv --python 3.8.20 .venv-spahybgen
/tmp/spahybgen-bootstrap/bin/uv pip install --python .venv-spahybgen/bin/python \
  numpy==1.23.5 Cython==3.0.2 setuptools==68.0.0 wheel==0.41.2
SKLEARN_ALLOW_DEPRECATED_SKLEARN_PACKAGE_INSTALL=True \
  /tmp/spahybgen-bootstrap/bin/uv pip install \
  --python .venv-spahybgen/bin/python --no-build-isolation \
  --index-strategy unsafe-best-match -r dependencies/spahybgen-reproduction.txt
```

NumPy 1.23.5 保留旧 API 使用的别名，Cython 用于 GraspNet 的构建。已有源码和环境可直接进入运行步骤。

## 运行与查看

新运行使用新目录：

```bash
.venv-spahybgen/bin/python tools/reproduce_spahybgen.py \
  --output-dir artifacts/spahybgen/my_run
```

入口调用上游 `scripts/generation_pipeline_test.py`，固定随机种子和线程，并保存输入、预测、候选和损失。

已有结果可直接打开 `artifacts/spahybgen/original_20261008_v2/best_grasp.html` 或 `grasps.html`。检查保存结果：

```bash
.venv-spahybgen/bin/python tools/reproduce_spahybgen.py \
  --output-dir artifacts/spahybgen/original_20261008_v2 --verify-only
```

| 文件 | 内容 |
| --- | --- |
| `inference.npz` | 网格、接触质量、XYZW 接触朝向和中心分布 |
| `optimization.npz` | 全部候选、优化迭代、分项损失和排序 |
| `grasps.html` | 多候选交互视图 |
| `best_grasp.html`、`preview.png` | 最佳排序候选的视图 |
| `source_manifest.json`、`environment.json` | 输入来源和依赖 |
| `result.json` | 状态、设置、数组形状与摘要 |

## 数据含义

`q_trajectory` 按“候选 × 优化迭代 × 优化变量”保存。变量包括手根 xyz、R6d 朝向和手关节；迭代序列用于观察优化过程。

`wrench_center_score` 是中心相关空间得分。实际传矩能力需要接触力学和载荷评价。

上游损失在更新前计算，轨迹末态在更新后保存，正式使用候选时需重算末态并检查限位。原加载器保留网络训练模式；工程推理模式需要单独核对。

[官方仓库](https://github.com/wangzivector/SpaHybGen)

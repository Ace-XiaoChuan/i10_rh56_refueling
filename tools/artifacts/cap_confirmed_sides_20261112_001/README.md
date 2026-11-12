# 人工确认占据的握梁两侧

打开[左右两侧交互图](index.html)或[并排静态图](sides_above_001.png)。默认显示原始接触分数 **>0.01** 的全部位置：左图Y<0侧278格，右图Y>0侧263格。

用户已肉眼确认握梁两侧存在实体，侧面未扫出点云来自观测角度。本次将原双侧3134个表面格采用为人工确认占据，覆盖其中521格原空闲分类；原始观测证据另存保留。既有侧面位置、大小和网格间距沿用原人工侧面。

| 侧别 | 确认占据表面格 | 原采用高分格 | 本次高分格（>0.01） |
|---|---:|---:|---:|
| Y<0 | 1538 | 15 | 278 |
| Y>0 | 1596 | 8 | 263 |

全部位置直接查询原`quality_raw`，没有邻域借分、插值或人工改分。分数来自此次占据更新之前保存的网络输出。位置沿用原“网格索引×1mm”，坐标为人工握梁系：X沿长轴，Y区分两侧，Z朝外，顶面为Z=0。

- [confirmed_geometry.npz](confirmed_geometry.npz)：人工确认占据、更新后的有效表面/空闲/未知分类、接触范围，以及单独保留的原始观测分类。只确认两侧表面格，未额外填充内部或扩大尺寸。
- [side_features.npz](side_features.npz)：全部3134格的索引、坐标、原分数、侧别及阈值选择。
- [high_scores.csv](high_scores.csv)：541格高分位置逐项明细。
- [report.json](report.json)：来源摘要及各侧统计。
- [verification.json](verification.json)：分类、分数和展示核对结果。

本次网络前向、训练、抓姿优化及新抓姿候选均为0。用户先查看高分接触位置，是否生成离线候选另行讨论。当前确认规则也已写入[项目说明](../../../doc/assembly_contract.md)。历史实验仍保留原分类。

复现展示：

```bash
/home/ace/i10_rh56_refueling_ws/.venv-spahybgen/bin/python /home/ace/i10_rh56_refueling_ws/tools/artifacts/cap_confirmed_sides_20261112_001/build_view.py
```

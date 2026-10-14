"""汇总已核对的用户整盖框和已确认无盖照片，导出未划分的 COCO 检测标注。"""
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
import shutil
from urllib.parse import quote

from PIL import Image

ROOT = Path('/home/ace/i10_rh56_refueling_ws')
OUTPUT = Path(Path('/tmp/i10-cap-annotations-output.txt').read_text().strip())
audit = json.loads(Path('/tmp/i10-positive-annotation-audit.json').read_text())
assert not audit['errors'] and audit['images_with_boxes'] == 11


def digest(path):
    """摘要绑定原图和用户记录；本轮图片总量很小，无需加载模型。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


confirmation = {
    'date': '2026-10-10', 'source': 'user_message',
    'question': '除正右遮挡外，其余五张无油箱盖照片拍摄时是否均未安装盖体？',
    'answer': '是，5 张均没有安装盖体',
    'negative_images': sorted(p.name for p in (ROOT / 'pic').glob('无油箱盖_*.jpg') if '遮挡' not in p.name),
    'occluded_images_excluded': ['带油箱盖_正右遮挡.jpg', '无油箱盖_正右遮挡.jpg'],
}
assert len(confirmation['negative_images']) == 5
(OUTPUT / 'user_confirmation.json').write_text(json.dumps(confirmation, ensure_ascii=False, indent=2) + '\n')

# 每张图片只保留一份训练框；重复保存及历史修改仍在审计清单中保留。
selected = {entry['image']: entry for entry in audit['coverage']}
names = sorted(set(selected) | set(confirmation['negative_images']))
coco = {'info': {'description': '整盖检测人工标注；尚未划分训练、验证和测试集',
                 'date_created': datetime.now(timezone.utc).isoformat()},
        'images': [], 'annotations': [],
        'categories': [{'id': 1, 'name': 'fuel cap', 'supercategory': 'object'}]}
manifest = {'schema': 'cap_detection_annotations_v1', 'date': '2026-10-10',
            'status': 'annotations_exported_unsplit_not_trained',
            'image_root': str(ROOT / 'pic'),
            'category_meaning': '整盖，包含握梁，排除凹腔、门板和连接带',
            'selection_rule': '每张照片按会话时间和标注序号选最新保存；不按 SAM 质量分数选框',
            'annotation_audit': audit, 'images': [], 'excluded_images': [],
            'dataset_split': None, 'training_started': False}
cards = []
for image_id, name in enumerate(names, 1):
    source = ROOT / 'pic' / name
    with Image.open(source) as image:
        width, height = image.size
    image_hash = digest(source)
    coco['images'].append({'id': image_id, 'file_name': name, 'width': width, 'height': height})
    record = {'image_id': image_id, 'path': str(source.relative_to(ROOT)),
              'sha256': image_hash, 'width': width, 'height': height,
              'capture_batch': None}
    rectangle = ''
    if name in selected:
        item = selected[name]['selected_latest']
        annotation = ROOT / item['path']
        assert item['image_sha256'] == image_hash
        assert item['image_size_wh'] == [width, height]
        x0, y0, x1, y1 = item['box_xyxy_px']
        assert 0 <= x0 < x1 <= width - 1 and 0 <= y0 < y1 <= height - 1
        box = [x0, y0, x1 - x0, y1 - y0]
        coco['annotations'].append({'id': len(coco['annotations']) + 1,
                                    'image_id': image_id, 'category_id': 1,
                                    'bbox': box, 'area': box[2] * box[3], 'iscrowd': 0})
        record.update(label='cap_visible_bbox', annotation_source='operator_input',
                      source_annotation=item['path'], source_annotation_sha256=digest(annotation),
                      box_xyxy_px=item['box_xyxy_px'])
        note = '用户整盖框；采用最新保存记录'
        if selected[name]['unique_boxes'] > 1:
            note += '；存在历史修改，见审计清单'
        rectangle = (f'<rect x="{x0}" y="{y0}" width="{x1-x0}" height="{y1-y0}" '
                     'fill="none" stroke="#00ff66" stroke-width="3" vector-effect="non-scaling-stroke"/>')
    else:
        record.update(label='cap_absent_negative', annotation_source='user_confirmation.json',
                      source_annotation_sha256=digest(OUTPUT / 'user_confirmation.json'))
        note = '用户确认无盖；空标注，不框加油口或门板'
    manifest['images'].append(record)
    image_url = quote(os.path.relpath(source, OUTPUT), safe='/')
    cards.append(f'<article><h2>{html.escape(name)}</h2><p>{html.escape(note)}</p>'
                 f'<svg viewBox="0 0 {width} {height}" role="img" aria-label="标注预览">'
                 f'<image href="{image_url}" width="{width}" height="{height}"/>{rectangle}</svg></article>')

for name in confirmation['occluded_images_excluded']:
    source = ROOT / 'pic' / name
    manifest['excluded_images'].append({'path': str(source.relative_to(ROOT)), 'sha256': digest(source),
                                       'reason': '门板遮挡目标区域；不根据机构状态虚构可见盖体框'})
assert len(coco['images']) == 16 and len(coco['annotations']) == 11

# 独立核对 COCO 的宽高转换可还原用户原图坐标，并检查空标注恰好对应五张负样本。
by_id = {entry['image_id']: entry for entry in manifest['images']}
for annotation in coco['annotations']:
    x, y, w, h = annotation['bbox']
    original = by_id[annotation['image_id']]['box_xyxy_px']
    assert all(abs(a - b) < 1e-9 for a, b in zip([x, y, x+w, y+h], original))
annotated_ids = {item['image_id'] for item in coco['annotations']}
negative_names = {item['file_name'] for item in coco['images'] if item['id'] not in annotated_ids}
assert negative_names == set(confirmation['negative_images'])
for record in manifest['images']:
    assert digest(ROOT / record['path']) == record['sha256']
    if record.get('source_annotation'):
        assert digest(ROOT / record['source_annotation']) == record['source_annotation_sha256']

(OUTPUT / 'annotations_coco.json').write_text(json.dumps(coco, ensure_ascii=False, indent=2) + '\n')
(OUTPUT / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
(OUTPUT / 'index.html').write_text(
    '<!doctype html><html lang="zh"><meta charset="utf-8"><title>整盖检测标注预览</title>'
    '<style>body{font-family:system-ui;margin:24px;background:#f3f4f6;color:#20242c}'
    'main{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px}'
    'article{background:white;padding:16px;border-radius:8px}h2{font-size:18px}'
    'svg{display:block;width:100%;max-height:620px}p{line-height:1.6}</style>'
    '<h1>整盖检测标注预览</h1><p>11 张用户框＋5 张用户确认无盖，共 16 张。'
    '绿色框使用原图坐标；两张门板遮挡照片暂单列。尚未划分数据集或训练。</p><main>'
    + ''.join(cards) + '</main></html>')
(OUTPUT / 'report.md').write_text('''# 用户整盖标注整理（2026-10-10）

已找到 11 张照片的 24 份有效操作员提示记录。当前原图摘要、尺寸、框范围和提示来源均通过检查；空会话不作为标注。按会话时间及标注序号选择最新保存记录，不按 SAM 得分挑选。正上照片含历史框修改，旧记录仍保留，选择来源详见 manifest.json。

用户明确确认五张无遮挡无盖照片拍摄时均未安装盖体，已作为无盖负样本导出；两张正右遮挡照片单独登记，未虚构框或混入无盖负样本。

已导出 [COCO 检测标注](annotations_coco.json)：16 张图片、11 个整盖框、5 张空标注，类别 fuel cap 表示整盖。图片根目录为原项目 pic/，COCO 的 file_name 保留原图片名。原图、用户提示、历史结果均未修改；没有训练/验证/测试划分或训练权重。

[预览页](index.html) 在原图上以 SVG 叠加绿色框，未改动照片像素。[来源清单](manifest.json) 保存摘要、最新记录选择和历史标注审计；[无盖确认](user_confirmation.json) 明确标明用户消息来源。

实际检查：16 个图片 ID 和 11 个标注 ID 唯一，原图尺寸与框范围有效；COCO XYWH 转回 XYXY 与用户框一致；没有标注的图恰好为用户确认的五张负样本；导出前后原图及选定提示摘要一致。此检查不等于逐张语义目视验收，不将 SAM 掩膜当作人工像素真值。

下一步需补拍摄批次信息并确定留出评价方式，再建立原权重定位基线及独立训练配置。暂未设置学习率、训练轮数、批量大小或显存方案。
''')
shutil.copyfile(__file__, OUTPUT / 'export_annotations.py')
print(json.dumps({'output': str(OUTPUT), 'images': len(coco['images']),
                  'boxes': len(coco['annotations']), 'negative_images': len(negative_names),
                  'excluded_occluded_images': len(manifest['excluded_images']),
                  'checks_passed': True}, ensure_ascii=False))

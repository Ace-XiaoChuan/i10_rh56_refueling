"""将八组新脚本结果汇总为可切换的离线HTML，不再求解或积分。"""
import html
import json
from pathlib import Path

OUT = Path(__file__).resolve().parent
STATUS = dict(fixed_opposed_contact='持续对握成功', no_safe_geometric_pose='未得到无超差预型',
              initial_proxy_penetration='初态接触模型侵入', thumb_contact_timeout='拇指接触超时',
              opposed_contact_timeout='未形成对握', thumb_contact_lost='拇指接触丢失',
              opposed_contact_lost='对握丢失', collision_or_invalid_feedback='闭合碰撞', numerical_failure='数值失败')
NAMES = dict(thumb='拇指', index='食指', middle='中指', ring='无名指', pinky='小指')


def main():
    """全部八组同列；表格点击后加载该组原生三维查看页。"""
    summary = json.loads((OUT / 'summary.json').read_text())
    rows = []
    notes = []
    for group in summary['groups']:
        index = group['group']
        state = STATUS.get(group['status'], group['status'])
        coordinates = ', '.join(f'{x * 1000:.2f}' for x in group['anchor_grip_m'])
        contact = group.get('final_contacts') or {}
        loaded = [f"{NAMES[c['finger']]} {c['normal_force_n']:.2f}N" + ('（端角/其他面）' if not c['lateral'] else '')
                  for c in contact.get('contacts', []) if c['normal_force_n'] > .01]
        forces = '；'.join(loaded) or '—'
        path = group['folder'] + '/index.html'
        # 只澄清本批查看页的来源文字，保留原轨迹、数据和求解脚本。
        view = OUT / path
        view.write_text(view.read_text().replace('定位来源：人工确认握梁定位。',
            '定位来源：点云推定的握梁坐标＋用户确认的双侧补全（位姿精度未标定）。'))
        cls = 'ok' if group['success'] else 'bad'
        rows.append(f'<tr class="{cls}" data-src="{path}" tabindex="0"><td>{index+1}</td><td>{coordinates}</td><td>{group["raw_network_score"]:.4f}</td><td>{html.escape(state)}</td><td>{html.escape(forces)}</td><td>{group["geometric_updates"]}</td></tr>')
        notes.append(dict(group=index, status=group['status'], score=group['raw_network_score'], anchor_grip_m=group['anchor_grip_m']))
    first = next((g for g in summary['groups'] if g['success']), summary['groups'][0])
    page = TEMPLATE.replace('__ROWS__', ''.join(rows)).replace('__SUCCESS__', str(summary['success_count']))
    page = page.replace('__TIME__', f"{summary['elapsed_seconds']:.1f}").replace('__FIRST__', first['folder'] + '/index.html')
    (OUT / 'index.html').write_text(page)
    (OUT / 'report_rows.json').write_text(json.dumps(notes, ensure_ascii=False, indent=2))
    print(str(OUT / 'index.html'))


TEMPLATE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>同一观测八组拇指锚点测验</title>
<style>body{margin:0;background:#f5f3ed;color:#253344;font:15px system-ui}header{padding:20px}h1{font-size:23px;margin:0}p{margin:9px 0}.table{overflow:auto}table{width:100%;border-collapse:collapse}td,th{padding:9px;text-align:left;border-bottom:1px solid #d5dadd;white-space:nowrap}tr[data-src]{cursor:pointer}tr[data-src]:hover{background:#e4ecf1}.ok td:first-child{color:#21844d;font-weight:bold}.bad td:first-child{color:#9b513f}iframe{border:0;width:100%;height:88vh;background:white}.hint{color:#5d6972}</style>
<header><h1>同一观测 · 八组拇指锚点测验</h1><p>持续对握 <b>__SUCCESS__ / 8</b> · 批次耗时 __TIME__ 秒。点击任一行查看该组保存的三维状态。</p>
<p class="hint">固定同一份点云、点云推定的握梁坐标与经确认的双侧补全、原网络分数及手部预型。每组24步以内、一次闭合；对握需拇指与至少另一指在相反侧面持续承力0.2秒。</p>
<div class="table"><table><thead><tr><th>组</th><th>锚点 X,Y,Z / mm</th><th>网络分数</th><th>结果</th><th>末态接触（未标定仿真）</th><th>几何步数</th></tr></thead><tbody>__ROWS__</tbody></table></div>
<p class="hint">所有组均保留，包括失败；这是同一场景中的锚点比较。端角接触不计作侧面对握，力值不代表实物承载。</p></header>
<iframe id="viewer" title="本组离线三维查看" src="__FIRST__"></iframe>
<script>for(const row of document.querySelectorAll('tr[data-src]')){const open=()=>{document.getElementById('viewer').src=row.dataset.src};row.onclick=open;row.onkeydown=e=>{if(e.key==='Enter')open()}}</script></html>'''


if __name__ == '__main__':
    main()

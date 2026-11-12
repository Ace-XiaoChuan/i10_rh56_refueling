"""检查本批HTML引用与全部三维数据，并提取浏览器外的交互检查材料。"""
import json
from html.parser import HTMLParser
from pathlib import Path

import plotly.graph_objects as go

OUT = Path(__file__).resolve().parent


class Page(HTMLParser):
    """只读取表格链接、内嵌脚本与查看页，不触发外部请求。"""
    def __init__(self, text):
        super().__init__()
        self.sources, self.iframes, self.scripts = [], [], []
        self.current = None
        self.feed(text)

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if tag == 'tr' and 'data-src' in attrs:
            self.sources.append(attrs['data-src'])
        if tag == 'iframe':
            self.iframes.append(attrs['src'])
        if tag == 'script':
            self.current = []

    def handle_data(self, text):
        if self.current is not None:
            self.current.append(text)

    def handle_endtag(self, tag):
        if tag == 'script' and self.current is not None:
            self.scripts.append(''.join(self.current))
            self.current = None


def main():
    """Schema核对实际数据；轻量脚本材料保留原交互代码、缩小载荷。"""
    summary = json.loads((OUT / 'summary.json').read_text())
    top = Page((OUT / 'index.html').read_text())
    assert top.sources == [g['folder'] + '/index.html' for g in summary['groups']]
    assert len(top.sources) == 8 and len(set(top.sources)) == 8
    assert top.iframes == ['group_00/index.html']
    harness = dict(summary_script=top.scripts[-1], row_sources=top.sources, groups=[])
    schemas = []
    for group in summary['groups']:
        folder = OUT / group['folder']
        text = (folder / 'index.html').read_text()
        assert '点云推定的握梁坐标＋用户确认的双侧补全' in text
        assert '定位来源：人工确认握梁定位。' not in text
        page = Page(text)
        assert len(page.scripts) == 2
        js = page.scripts[-1]
        prefix = 'const payload='
        assert js.startswith(prefix)
        payload, end = json.JSONDecoder().raw_decode(js, len(prefix))
        assert payload == json.loads((folder / 'viewer_data.json').read_text())
        assert payload['scene_check_points'] == 667868
        for frame in payload['frames']:
            go.Figure(data=frame['data']).to_json(validate=True)
        schemas.append(dict(group=group['group'], frame_count=len(payload['frames']), schema_valid=True))
        # 交互检查只替换数据体积，draw/选择器/证据生成的脚本来自实际HTML。
        evidence = [dict(finger_opening_attempts=r['finger_opening_attempts'],
                         finger_participation=r['finger_participation']) for r in payload['result']['results']]
        reduced = dict(frames=[dict(label=f['label'], data=[dict(marker_id=i)])
                               for i, f in enumerate(payload['frames'])], result=dict(results=evidence))
        harness['groups'].append(dict(folder=group['folder'], frame_count=len(payload['frames']),
            script=prefix + json.dumps(reduced, ensure_ascii=False) + js[end:]))
    (OUT / 'ui_harness.json').write_text(json.dumps(harness, ensure_ascii=False))
    result = dict(html_references_valid=True, registered_rows=8, plotly_schemas=schemas,
                  frame_count=sum(x['frame_count'] for x in schemas), real_browser_rendered=False)
    (OUT / 'report_checks.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False))


if __name__ == '__main__':
    main()

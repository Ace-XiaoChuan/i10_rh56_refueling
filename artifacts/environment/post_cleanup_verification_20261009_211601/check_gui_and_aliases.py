"""补验软链接路径判定并追加窗口检查，保留初次检查脚本的误判记录。"""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path('/home/ace/i10_rh56_refueling_ws')
OUTPUT = Path(Path('/tmp/i10-post-cleanup-output.txt').read_text().strip())
report = json.loads((OUTPUT / 'checks.json').read_text())
assert 'finished_utc' in report, '需等待前面的顺序检查结束'
env = dict(os.environ, PYTHONPATH=str(ROOT / 'src'), OMP_NUM_THREADS='1',
           MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1',
           MPLCONFIGDIR=str(OUTPUT / 'matplotlib_cache'))

attempts = []
for item in report['checks']:
    if item['name'] not in ('shared_interpreter', 'perception_interpreter') or item['passed']:
        continue
    attempts.append(dict(item))
    original_log = OUTPUT / item['log']
    original_log.rename(original_log.with_name(original_log.stem + '_first_attempt.log'))
    command = item['command'][:]
    command[-1] = 'from pathlib import Path;' + command[-1].replace(
        "assert sys.prefix=='/home/ace/mujoco-env'",
        "assert Path(sys.prefix).resolve()==Path('/home/ace/mujoco-env')")
    started = time.monotonic()
    with original_log.open('w') as stream:
        result = subprocess.run(command, cwd=ROOT, env=env, stdout=stream,
                                stderr=subprocess.STDOUT, timeout=150)
    item.update(command=command, exit_code=result.returncode, passed=result.returncode == 0,
                elapsed_seconds=round(time.monotonic() - started, 2))
    print('软链接路径补验：', item['name'], item['passed'], flush=True)
report['initial_check_attempts'] = attempts
report['check_script_correction'] = (
    '初次断言误把 sys.prefix 的软链接路径判成错误；库已成功导入。'
    '现按 Path(sys.prefix).resolve() 判断实际环境，两项已重跑，原日志保留；未改环境或项目实现。')

# WSLg 显示须可访问；仅打开运动学预览及短暂测试窗口。
for name, command in [
    ('mujoco_replay_gui', [str(ROOT / '.venv/bin/python'), str(OUTPUT / 'check_gui.py')]),
    ('sam_gui_regression', [str(ROOT / '.venv-perception/bin/python'), '-m', 'pytest',
                            '-q', '-p', 'no:cacheprovider', 'tests/test_sam_diagnostic.py', '-k', 'gui']),
]:
    print('窗口检查开始：', name, flush=True)
    started = time.monotonic()
    log = OUTPUT / (name + '.log')
    with log.open('w') as stream:
        try:
            result = subprocess.run(command, cwd=ROOT, env=env, stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=75)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    record = {'name': name, 'command': command, 'exit_code': code, 'expected_exit_code': 0,
              'passed': code == 0, 'elapsed_seconds': round(time.monotonic() - started, 2), 'log': log.name}
    report['checks'].append(record)
    print('窗口检查结果：', name, record['passed'], flush=True)
    print(log.read_text()[-1900:], flush=True)

report['finished_utc'] = datetime.now(timezone.utc).isoformat()
report['passed'] = all(item['passed'] for item in report['checks']) and all(report['unchanged_files'].values())
(OUTPUT / 'checks.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
print('最终检查结果：', report['passed'], flush=True)

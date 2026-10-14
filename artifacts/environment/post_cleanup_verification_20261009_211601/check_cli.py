"""顺序检查整理后的项目入口，将实际日志写到独立目录。"""

from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path('/home/ace/i10_rh56_refueling_ws')
OUTPUT = Path('/tmp/i10-post-cleanup-output.txt').read_text().strip()
OUTPUT = Path(OUTPUT)
ENV = dict(os.environ, PYTHONPATH=str(ROOT / 'src'), OMP_NUM_THREADS='1',
           MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', HF_HUB_OFFLINE='1',
           TRANSFORMERS_OFFLINE='1', PYTHONDONTWRITEBYTECODE='1')
REPORT = {'started_utc': datetime.now(timezone.utc).isoformat(), 'checks': [],
          'limitations': ['仅验证现有软件入口与环境兼容性，不验证真机、物理抓稳或完整取盖。']}
BOUND_FILES = ['artifacts/manual_grasp/user_selected_20260930.json',
               'artifacts/planning/approach.json',
               'artifacts/planning/replay_validation.json',
               'assets/scenes/i10_rh56_fuel_cap_preview.xml',
               'tools/replay_cap_approach.py', 'tools/segment_cap_image.py',
               'tools/diagnose_sam_image.py', 'tools/reproduce_spahybgen.py']
BEFORE = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in BOUND_FILES}


def check(name, command, expected=0, timeout=150):
    """保留每次退出码和日志；预期停止分支允许明确指定非零退出码。"""
    started = time.monotonic()
    log = OUTPUT / (name + '.log')
    print('开始：' + name, flush=True)
    with log.open('w') as stream:
        try:
            result = subprocess.run(command, cwd=ROOT, env=ENV, stdout=stream,
                                    stderr=subprocess.STDOUT, timeout=timeout)
            code = result.returncode
        except subprocess.TimeoutExpired:
            code = 'timeout'
    record = {'name': name, 'command': command, 'exit_code': code,
              'expected_exit_code': expected, 'passed': code == expected,
              'elapsed_seconds': round(time.monotonic() - started, 2), 'log': log.name}
    REPORT['checks'].append(record)
    (OUTPUT / 'checks.json').write_text(json.dumps(REPORT, ensure_ascii=False, indent=2) + '\n')
    print(('通过：' if record['passed'] else '失败：') + name + '，退出码 ' + str(code), flush=True)
    if not record['passed']:
        print(log.read_text()[-2500:], flush=True)
    return record['passed']


# 首先检查解释器入口与依赖，不改共享环境。
shared = str(ROOT / '.venv/bin/python')
perception = str(ROOT / '.venv-perception/bin/python')
spa = str(ROOT / '.venv-spahybgen/bin/python')
check('shared_interpreter', [shared, '-c',
      "import sys,json,mujoco,numpy,torch,trimesh,cv2,scipy,pinocchio,coal; "
      "print(json.dumps({'python':sys.version,'prefix':sys.prefix,'mujoco':mujoco.__version__,"
      "'numpy':numpy.__version__,'torch':torch.__version__})); "
      "assert sys.version_info[:2]==(3,10);from pathlib import Path;assert Path(sys.prefix).resolve()==Path('/home/ace/mujoco-env');"
      "assert mujoco.__version__=='3.13.0';"
      "assert torch.from_numpy(numpy.array([1.,2.])).sum().item()==3"])
check('perception_interpreter', [perception, '-c',
      "import sys,json,torch,torchvision,transformers,tkinter;"
      "from sam2.sam2_image_predictor import SAM2ImagePredictor;"
      "print(json.dumps({'python':sys.version,'prefix':sys.prefix,'torchvision':torchvision.__version__,"
      "'transformers':transformers.__version__}));from pathlib import Path;assert Path(sys.prefix).resolve()==Path('/home/ace/mujoco-env')"])
check('shared_dependencies', [shared, '-m', 'pip', 'check'])
check('planning_model_sources', [shared, 'tools/check_moveit_model_sources.py'])
check('candidate_and_existing_replay', [shared, '-c',
      "from pathlib import Path;import json;"
      "from fuel_cap_task.arm_ik import ArmIKPreview;"
      "from fuel_cap_task.trajectory_replay import ApproachReplay;"
      "preview=ArmIKPreview(Path('artifacts/manual_grasp/user_selected_20260930.json'));"
      "replay=ApproachReplay(Path('artifacts/planning/approach.json'));"
      "print(json.dumps(replay.validate(),ensure_ascii=False));"
      "[replay.set_progress(value) for value in (0.,.5,1.)]"])

# 复用现有针对入口和分支的测试，GUI 显示稍后单独检查。
check('focused_regression', [shared, '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
      'tests/test_cap_task.py', 'tests/test_feedback_task.py',
      'tests/planning/test_approach_replay.py', 'tests/test_grounded_sam_contract.py',
      'tests/test_sam_diagnostic.py', '-k', 'not gui'], timeout=180)
check('mock_early_detach', [shared, 'tools/run_cap_task.py', '--scenario', 'early_detach'])
check('mock_regrasp', [shared, 'tools/run_cap_task.py', '--scenario', 'regrasp'])
check('mock_support_unknown', [shared, 'tools/run_cap_task.py', '--scenario', 'support_unknown'], expected=1)

# SpaHybGen 保持独立；本次检查导入和已有结果，不重跑耗时优化。
check('spahybgen_interpreter', [spa, '-c',
      "import sys,json,numpy,torch,mujoco,open3d,pytorch_kinematics,graspnetAPI,grasp_nms;"
      "print(json.dumps({'python':sys.version,'prefix':sys.prefix,'mujoco':mujoco.__version__,"
      "'numpy':numpy.__version__,'torch':torch.__version__}));"
      "assert sys.version_info[:2]==(3,8);assert torch.from_numpy(numpy.array([1.,2.])).sum().item()==3"])
check('spahybgen_saved_result', [spa, 'tools/reproduce_spahybgen.py', '--verify-only',
      '--output-dir', 'artifacts/spahybgen/original_20261008_v2'])

# 顺序运行新渲染与真实模型推理；每个输出目录都必须不存在。
observation = OUTPUT / 'rgbd'
if check('rgbd_render', [shared, 'tools/capture_cap_rgbd.py', '--output-dir', str(observation)]):
    detected = OUTPUT / 'automatic_segmentation'
    if check('grounding_dino_sam_inference', [perception, 'tools/segment_cap_image.py',
             '--image', str(observation / 'rgb.png'), '--image-source', 'mujoco_rendered_rgb',
             '--observation-json', str(observation / 'observation.json'), '--prompt', 'fuel cap.',
             '--output-dir', str(detected)], timeout=180):
        check('new_automatic_result_verification', [perception, 'tools/segment_cap_image.py',
              '--verify-only', '--output-dir', str(detected)])
manual = OUTPUT / 'manual_sam'
if check('saved_box_sam_inference', [perception, 'tools/diagnose_sam_image.py',
         '--image', 'assets/hardware_photos/fuel_tank/be44de650d565ff38af7dd108c384037.jpg',
         '--prompts', 'artifacts/perception/sam_manual/20261008_162256_066504/annotations/0001.json',
         '--headless', '--mode', 'box', '--output-dir', str(manual)], timeout=180):
    check('new_manual_result_verification', [perception, 'tools/diagnose_sam_image.py',
          '--verify-only', '--output-dir', str(manual)])

REPORT['unchanged_files'] = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest
                             for name, digest in BEFORE.items()}
REPORT['finished_utc'] = datetime.now(timezone.utc).isoformat()
REPORT['passed'] = all(item['passed'] for item in REPORT['checks']) and all(REPORT['unchanged_files'].values())
(OUTPUT / 'checks.json').write_text(json.dumps(REPORT, ensure_ascii=False, indent=2) + '\n')
print('检查日志目录：' + str(OUTPUT), flush=True)
print('本轮已执行检查全部通过：' + str(REPORT['passed']), flush=True)

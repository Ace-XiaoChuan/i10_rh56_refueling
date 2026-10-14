"""诊断隔离与坐标回归；模型效果另由真实推理检查，不用假掩膜替代。"""

import copy
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from fuel_cap_task.sam_diagnostic import (
    DiagnosticSession, Viewport, digest, overlay, used_prompts, validate_prompts, verify_session,
)

ROOT = Path(__file__).resolve().parents[1]


def annotation(image):
    """测试来源明确标记，与操作员标注区别保存。"""
    return {"schema": "sam_manual_prompts_v1", **image, "box_xyxy_px": [1., 2., 18., 14.],
            "point_coords_px": [[5., 7.], [19., 15.]], "point_labels": [1, 0],
            "annotation_source": "test_fixture"}


@pytest.fixture
def session(tmp_path):
    """临时小图只用于坐标和文件边界测试。"""
    path = tmp_path / "input.png"
    Image.new("RGB", (20, 16), color=(100, 110, 120)).save(path)
    return DiagnosticSession(path, tmp_path / "session", ROOT)


@pytest.mark.parametrize("canvas", [(800, 400), (401, 700), (173, 99)])
def test_scaled_coordinates_round_trip_and_letterbox(canvas):
    vp = Viewport.fit((1706, 1279), canvas)
    point = [743.25, 491.5]
    assert vp.to_image(*vp.to_canvas(*point)) == pytest.approx(point)
    assert vp.to_image(vp.offset_xy[0] - 1, vp.offset_xy[1]) is None
    assert vp.to_image(-100, -100, clamp=True) == [0, 0]


def test_box_only_actually_excludes_saved_points(session):
    prompts = annotation(session.image)
    assert used_prompts(prompts, "box", session.image)["point_coords_px"] == []
    assert used_prompts(prompts, "box_points", session.image)["point_labels"] == [1, 0]
    assert len(prompts["point_coords_px"]) == 2


@pytest.mark.parametrize("change", [
    {"image_sha256": "wrong_photo"}, {"image_size_wh": [16, 20]},
    {"box_xyxy_px": [18, 2, 1, 14]}, {"box_xyxy_px": [1, 2, 21, 14]},
    {"point_coords_px": [[float("nan"), 1]]}, {"point_labels": [1, 2]},
])
def test_invalid_or_wrong_image_prompts_rejected(session, change):
    prompts = annotation(session.image)
    prompts.update(change)
    with pytest.raises(ValueError):
        validate_prompts(prompts, session.image)


def test_points_mode_requires_points(session):
    prompts = annotation(session.image)
    prompts.update(point_coords_px=[], point_labels=[])
    with pytest.raises(ValueError, match="至少一个"):
        used_prompts(prompts, "box_points", session.image)


def test_overlay_does_not_burn_prompts_into_model_input(session):
    original = session.rgb.copy()
    mask = np.ones(original.shape[:2], dtype=bool)
    picture = overlay(session.rgb, mask, annotation(session.image))
    assert np.array_equal(session.rgb, original)
    assert not np.array_equal(np.asarray(picture), original)


def test_saved_annotations_are_independent_and_input_unchanged(session):
    prompts = annotation(session.image)
    first = session.save_annotation(prompts)
    prompts["box_xyxy_px"][0] = 2
    second = session.save_annotation(prompts)
    assert first != second
    assert json.loads(first.read_text())["box_xyxy_px"][0] == 1
    assert digest(session.input_path) == session.image["image_sha256"]
    assert verify_session(session.output) == 0
    with pytest.raises(FileExistsError):
        DiagnosticSession(session.image["image_path"], session.output, ROOT)


def test_inference_exception_is_preserved_and_not_counted_as_success(session):
    def broken_backend():
        """仅验证异常保存，不模拟预测结果。"""
        raise RuntimeError("test backend startup failed")

    with pytest.raises(RuntimeError, match="推理失败"):
        session.run(annotation(session.image), "box", broken_backend)
    folder = session.output / "trials/0001_box"
    assert "test backend startup failed" in (folder / "failure.txt").read_text()
    assert json.loads((folder / "result.json").read_text())["status"] == "runtime_error"
    assert not (folder / "mask.png").exists()
    with pytest.raises(ValueError, match="未完成推理"):
        verify_session(session.output)


def test_wrong_image_automatic_reference_rejected_before_output_creation(session, tmp_path):
    reference = tmp_path / "result.json"
    reference.write_text(json.dumps({"image_sha256": "another_image"}))
    output = tmp_path / "other"
    with pytest.raises(ValueError, match="照片不匹配"):
        DiagnosticSession(session.image["image_path"], output, ROOT, reference)
    assert not output.exists()


def test_gui_reverse_drag_resize_undo_and_prompt_reload(session):
    import tkinter as tk
    from types import SimpleNamespace
    from fuel_cap_task.sam_diagnostic_gui import DiagnosticGUI
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("当前环境没有可用显示服务")
    try:
        gui = DiagnosticGUI(root, session, None)
        root.update()
        vp = gui.viewports["input"]
        gui.press(SimpleNamespace(x=vp.to_canvas(18, 14)[0], y=vp.to_canvas(18, 14)[1]))
        gui.release(SimpleNamespace(x=vp.to_canvas(1, 2)[0], y=vp.to_canvas(1, 2)[1]))
        assert gui.box == pytest.approx([1, 2, 18, 14])
        root.geometry("1000x700")
        root.update()
        before = copy.deepcopy(gui.box)
        gui.mode.set("positive")
        x, y = gui.viewports["input"].to_canvas(5, 7)
        gui.press(SimpleNamespace(x=x, y=y))
        assert gui.points[0] == pytest.approx([5, 7])
        gui.undo()
        assert gui.points == [] and gui.box == before
        gui.load(annotation(session.image))
        saved = session.save_annotation(gui.prompts())
        gui.clear()
        gui.load(json.loads(saved.read_text()))
        assert gui.points == [[5., 7.], [19., 15.]]
        assert gui.labels == [1, 0]
        # 点变化不能使仅框实验失效；框变化必须清除旧掩膜。
        picture = Image.fromarray(session.rgb)
        gui.results["box"] = ({"used_prompts": used_prompts(gui.prompts(), "box", session.image)}, picture)
        gui.results["box_points"] = ({"used_prompts": used_prompts(gui.prompts(), "box_points", session.image)}, picture)
        gui.add_point([6., 8.], 1)
        assert "box" in gui.results and "box_points" not in gui.results
        gui.box[0] = 2
        gui.changed()
        assert not gui.results
        gui.close()
    finally:
        try:
            root.destroy()
        except tk.TclError:
            pass


def test_gui_close_waits_for_active_job_and_preserves_exception(session):
    import threading
    import tkinter as tk
    from fuel_cap_task.sam_diagnostic_gui import DiagnosticGUI
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("当前环境没有可用显示服务")
    gate = threading.Event()

    def blocked_backend():
        """阻塞启动仅用于验证退出交接；不构造任何模型输出。"""
        gate.wait(timeout=3)
        raise RuntimeError("test shutdown handoff")

    try:
        gui = DiagnosticGUI(root, session, blocked_backend, annotation(session.image))
        root.update()
        gui.run("box")
        gui.close()
        assert gui.closing and gui.busy and root.winfo_exists()
        root.after(20, gate.set)
        root.mainloop()
        assert not gui.busy
        folder = session.output / "trials/0001_box"
        assert json.loads((folder / "result.json").read_text())["status"] == "runtime_error"
        assert "test shutdown handoff" in (folder / "failure.txt").read_text()
    finally:
        gate.set()
        try:
            root.destroy()
        except tk.TclError:
            pass

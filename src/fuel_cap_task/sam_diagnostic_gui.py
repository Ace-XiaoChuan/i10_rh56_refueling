"""Tk 人工框与点提示界面；主线程处理窗口，单一工作线程执行 SAM。"""

import copy
import json
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk

from .sam_diagnostic import Viewport, used_prompts, validate_prompts


class DiagnosticGUI:
    """画布缩放不修改原图坐标；结果只显示在对应提示有效时。"""

    def __init__(self, root, session, backend_factory, prompts=None):
        self.root, self.session, self.backend_factory = root, session, backend_factory
        self.box, self.points, self.labels = None, [], []
        self.annotation_source = "operator_input"
        self.history, self.results = [], {}
        self.drag_start = None
        self.busy = self.closing = False
        self.messages = queue.Queue()
        self.mode = tk.StringVar(value="box")
        self.status = tk.StringVar(value="在左侧原图拖出整盖的框，然后运行 B 组。")
        self.coordinates = tk.StringVar(value="框：尚未设置；正点 0；负点 0")
        self.box_text = tk.StringVar()
        self.photos, self.viewports = {}, {}
        root.title("SAM 2 模块诊断 — 人工框 / 正负点")
        root.geometry("1320x850")
        root.minsize(900, 650)
        root.protocol("WM_DELETE_WINDOW", self.close)

        # 输入模式与提示操作明确分开，避免把点当作新的框角。
        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill="x")
        for text, value in (("拖框", "box"), ("正点（属于盖体）", "positive"), ("负点（排除背景）", "negative")):
            ttk.Radiobutton(toolbar, text=text, value=value, variable=self.mode).pack(side="left", padx=5)
        self.buttons = []
        for text, command in (("撤销", self.undo), ("清空", self.clear),
                              ("载入提示", self.load_dialog), ("保存提示", self.save)):
            button = ttk.Button(toolbar, text=text, command=command)
            button.pack(side="left", padx=4)
            self.buttons.append(button)
        numeric = ttk.Frame(root, padding=(8, 0))
        numeric.pack(fill="x")
        ttk.Label(numeric, text="原图框 x0, y0, x1, y1：").pack(side="left")
        ttk.Entry(numeric, textvariable=self.box_text, width=40).pack(side="left")
        apply_button = ttk.Button(numeric, text="应用坐标", command=self.apply_box)
        apply_button.pack(side="left", padx=6)
        self.buttons.append(apply_button)
        ttk.Label(numeric, textvariable=self.coordinates).pack(side="left", padx=10)

        panes = ttk.Panedwindow(root, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=8)
        left = ttk.Frame(panes)
        ttk.Label(left, text="原图：只在此处编辑；绿色正点，红色负点").pack()
        self.input_canvas = tk.Canvas(left, background="#222", highlightthickness=0)
        self.input_canvas.pack(fill="both", expand=True)
        panes.add(left, weight=1)
        self.notebook = ttk.Notebook(panes)
        panes.add(self.notebook, weight=1)
        self.canvases, self.result_labels, self.tabs = {}, {}, {}
        for key, title in (("automatic", "A：历史自动结果"), ("box", "B：仅人工框"), ("box_points", "C：人工框＋点")):
            if key == "automatic" and session.reference is None:
                continue
            tab = ttk.Frame(self.notebook)
            self.notebook.add(tab, text=title)
            label = ttk.Label(tab, text="同图历史结果；未重新推理" if key == "automatic" else "尚未运行")
            label.pack()
            canvas = tk.Canvas(tab, background="#222", highlightthickness=0)
            canvas.pack(fill="both", expand=True)
            self.canvases[key], self.result_labels[key], self.tabs[key] = canvas, label, tab
            canvas.bind("<Configure>", lambda event, k=key: self.draw_result(k))
        self.input_canvas.bind("<Configure>", lambda event: self.draw_input())
        self.input_canvas.bind("<ButtonPress-1>", self.press)
        self.input_canvas.bind("<B1-Motion>", self.motion)
        self.input_canvas.bind("<ButtonRelease-1>", self.release)
        self.input_canvas.bind("<ButtonPress-3>", self.right_click)

        actions = ttk.Frame(root, padding=8)
        actions.pack(fill="x")
        for text, mode in (("运行 B：仅人工框", "box"), ("运行 C：框＋正负点", "box_points")):
            button = ttk.Button(actions, text=text, command=lambda m=mode: self.run(m))
            button.pack(side="left", padx=4)
            self.buttons.append(button)
        ttk.Label(actions, text="SAM 评分仅表示预测分割质量；人工提示结果不计自动识别通过。").pack(side="left", padx=12)
        ttk.Label(root, textvariable=self.status, wraplength=1250, padding=(8, 2)).pack(fill="x")
        ttk.Label(root, text=f"保存目录：{session.output}", padding=(8, 2)).pack(fill="x")
        if prompts is not None:
            self.load(prompts)
        root.after(100, self.poll)

    def snapshot(self):
        """保存完整编辑状态，撤销同时恢复提示来源。"""
        return copy.deepcopy((self.box, self.points, self.labels, self.annotation_source))

    def prompts(self):
        """导出原图坐标与照片摘要，不导出缩放截图。"""
        return {"schema": "sam_manual_prompts_v1", **self.session.image,
                "box_xyxy_px": copy.deepcopy(self.box),
                "point_coords_px": copy.deepcopy(self.points), "point_labels": list(self.labels),
                "annotation_source": self.annotation_source}

    def changed(self):
        """框变动使 B/C 失效；仅点变动使 C 失效，B 的输入仍然相同。"""
        for key in list(self.results):
            report, _ = self.results[key]
            if (self.box != report["used_prompts"]["box_xyxy_px"]
                    or (key == "box_points" and (self.points != report["used_prompts"]["point_coords_px"]
                                                 or self.labels != report["used_prompts"]["point_labels"]))):
                del self.results[key]
                self.result_labels[key].configure(text="提示已改变，请重新运行；旧结果保留在保存目录")
                self.draw_result(key)
        self.box_text.set(", ".join(f"{v:.2f}" for v in self.box) if self.box else "")
        self.coordinates.set(f"正点 {self.labels.count(1)}；负点 {self.labels.count(0)}")
        self.draw_input()

    def draw_picture(self, canvas, picture, key):
        """保存 PhotoImage 引用，防止 Tk 中图片被垃圾回收。"""
        viewport = Viewport.fit(picture.size, (canvas.winfo_width(), canvas.winfo_height()))
        self.viewports[key] = viewport
        photo = ImageTk.PhotoImage(picture.resize(viewport.display_wh, Image.Resampling.LANCZOS))
        self.photos[key] = photo
        canvas.delete("all")
        canvas.create_image(*viewport.offset_xy, image=photo, anchor="nw")
        return viewport

    def draw_input(self):
        """把坐标提示绘制为画布元素，绝不修改输入数组。"""
        vp = self.draw_picture(self.input_canvas, Image.fromarray(self.session.rgb), "input")
        if self.box:
            self.input_canvas.create_rectangle(*vp.to_canvas(*self.box[:2]), *vp.to_canvas(*self.box[2:]), outline="orange", width=2)
        for (x, y), label in zip(self.points, self.labels):
            u, v = vp.to_canvas(x, y)
            self.input_canvas.create_oval(u - 5, v - 5, u + 5, v + 5, fill="lime" if label else "red", outline="white")

    def draw_result(self, key):
        """尚未推理的页只展示原图，不显示过期掩膜。"""
        if key == "automatic":
            picture = self.session.reference_picture
        else:
            picture = self.results[key][1] if key in self.results else Image.fromarray(self.session.rgb)
        self.draw_picture(self.canvases[key], picture, key)

    def add_point(self, point, label):
        """保留点击顺序，便于逐次撤销和完整重现。"""
        self.history.append(self.snapshot())
        self.points.append(point)
        self.labels.append(label)
        self.annotation_source = "operator_input"
        self.changed()

    def press(self, event):
        """空白边缘不接受点击，正在推理时冻结当前编辑。"""
        if self.busy:
            return
        point = self.viewports["input"].to_image(event.x, event.y)
        if point is None:
            return
        if self.mode.get() == "box":
            self.drag_start = point
        else:
            self.add_point(point, int(self.mode.get() == "positive"))

    def motion(self, event):
        """框拖动仅预览；释放鼠标后才提交到编辑历史。"""
        if self.busy or self.drag_start is None:
            return
        vp = self.viewports["input"]
        end = vp.to_image(event.x, event.y, clamp=True)
        self.input_canvas.delete("drag")
        self.input_canvas.create_rectangle(*vp.to_canvas(*self.drag_start), *vp.to_canvas(*end),
                                           outline="orange", width=2, tags="drag")

    def release(self, event):
        """允许反向拖框，统一转换成左上、右下 XYXY。"""
        if self.busy or self.drag_start is None:
            return
        end = self.viewports["input"].to_image(event.x, event.y, clamp=True)
        start, self.drag_start = self.drag_start, None
        box = [min(start[0], end[0]), min(start[1], end[1]), max(start[0], end[0]), max(start[1], end[1])]
        if box[2] - box[0] < 1 or box[3] - box[1] < 1:
            self.draw_input()
            return
        self.history.append(self.snapshot())
        self.box, self.annotation_source = box, "operator_input"
        self.changed()

    def right_click(self, event):
        """右键快捷添加负点；与明确的负点模式作用相同。"""
        if not self.busy:
            point = self.viewports["input"].to_image(event.x, event.y)
            if point is not None:
                self.add_point(point, 0)

    def apply_box(self):
        """精确坐标入口仍经过同一像素合同校验。"""
        if self.busy:
            return
        try:
            value = self.prompts()
            value["box_xyxy_px"] = [float(v.strip()) for v in self.box_text.get().replace("，", ",").split(",")]
            validate_prompts(value, self.session.image)
            self.history.append(self.snapshot())
            self.box, self.annotation_source = value["box_xyxy_px"], "operator_input"
            self.changed()
        except Exception as error:
            messagebox.showerror("坐标无效", str(error))

    def undo(self):
        """撤销只影响当前提示，不删除已保存实验。"""
        if not self.busy and self.history:
            self.box, self.points, self.labels, self.annotation_source = self.history.pop()
            self.changed()

    def clear(self):
        """清空需要显式重新框选，不提供猜测种子框。"""
        if not self.busy:
            self.history.append(self.snapshot())
            self.box, self.points, self.labels = None, [], []
            self.annotation_source = "operator_input"
            self.changed()

    def save(self):
        """保存可重新载入的提示，即使尚未进行推理。"""
        if self.busy:
            return
        try:
            path = self.session.save_annotation(self.prompts())
            self.status.set(f"提示已保存：{path}")
        except Exception as error:
            messagebox.showerror("保存失败", str(error))

    def load(self, value):
        """仅恢复同图提示；不把历史预测当成当前模型的新输出。"""
        validate_prompts(value, self.session.image)
        self.history.append(self.snapshot())
        self.box = list(value["box_xyxy_px"])
        self.points, self.labels = copy.deepcopy(value.get("point_coords_px", [])), list(value.get("point_labels", []))
        self.annotation_source = value["annotation_source"]
        self.changed()

    def load_dialog(self):
        """图片不匹配时拒绝载入并保留当前编辑。"""
        if self.busy:
            return
        path = filedialog.askopenfilename(title="载入同一照片的提示", filetypes=[("JSON", "*.json")])
        if path:
            try:
                self.load(json.loads(Path(path).read_text()))
                self.status.set(f"已载入提示：{path}；点击运行以重新推理。")
            except Exception as error:
                messagebox.showerror("载入失败", str(error))

    def run(self, mode):
        """一次只运行一个实验，避免模型和特征缓存被并发改写。"""
        if self.busy:
            return
        value = self.prompts()
        try:
            used_prompts(value, mode, self.session.image)
        except Exception as error:
            messagebox.showerror("提示无效", str(error))
            return
        self.busy = True
        for button in self.buttons:
            button.configure(state="disabled")
        self.status.set("正在运行 SAM 2，首次需要加载模型；窗口仍可响应。")

        def worker():
            """工作线程仅处理数据，Tk 操作全部经主线程队列返回。"""
            try:
                self.messages.put(("done", mode, self.session.run(value, mode, self.backend_factory)))
            except Exception as error:
                self.messages.put(("error", mode, str(error)))

        threading.Thread(target=worker, daemon=False).start()

    def poll(self):
        """关闭期间等待正在运行的实验落盘，不强行中断后台推理。"""
        try:
            kind, mode, result = self.messages.get_nowait()
        except queue.Empty:
            pass
        else:
            self.busy = False
            for button in self.buttons:
                button.configure(state="normal")
            if kind == "done":
                report, picture, folder = result
                self.results[mode] = report, picture
                self.result_labels[mode].configure(text=f'{report["status"]}；SAM 质量 {report["sam_mask_quality_score"]:.4f}；{report["mask_pixels"]} 像素')
                self.draw_result(mode)
                self.notebook.select(self.tabs[mode])
                self.status.set(f"实验已保存：{folder}；对照页面 index.html 已更新。")
            else:
                self.status.set(result)
                if not self.closing:
                    messagebox.showerror("推理失败", result)
            if self.closing:
                self.root.destroy()
                return
        self.root.after(100, self.poll)

    def close(self):
        """有效提示自动归档；推理期间等待完成后退出。"""
        if self.busy:
            self.closing = True
            self.status.set("正在等待当前实验保存，完成后窗口自动关闭。")
            return
        if self.box is not None:
            self.session.save_annotation(self.prompts())
        self.root.destroy()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import tkinter as tk
from tkinter import messagebox
import json
import subprocess
import signal


class ScriptRunnerUI:
    def __init__(self, master):
        self.master = master
        self.master.title("脚本运行面板")

        # 窗口初始大小与居中逻辑，与原控制面板风格一致
        screen_width = self.master.winfo_screenwidth()
        screen_height = self.master.winfo_screenheight()
        win_width = int(screen_width * 0.4)
        win_height = int(screen_height * 0.8)
        x_coordinate = int((screen_width - win_width) / 2)
        y_coordinate = int((screen_height - win_height) / 2)
        self.master.geometry(f"{win_width}x{win_height}+{x_coordinate}+{y_coordinate}")
        self.master.configure(bg='#1e1e1e')

        # 捕获窗口关闭事件，优雅终止所有脚本
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # ========== 状态变量（完全沿用原逻辑） ==========
        self.selected_script_info = None  # 当前选中项信息
        self.script_buttons = []          # 所有脚本按钮实例
        self.running_processes = {}       # 运行中的进程句柄 {path: popen_object}

        # ========== 构建主界面（完全复刻原脚本区布局） ==========
        self.build_main_ui()

    def build_main_ui(self):
        """完全复刻原脚本运行区结构：顶部控制栏 + 中间滚动列表 + 底部状态条"""
        main_frame = tk.Frame(self.master, bg='#2d2d2d', bd=2, relief=tk.SUNKEN)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 1. 顶部固定控制栏（永远置顶，不随列表滚动）
        self.control_bar = tk.Frame(main_frame, bg='#2d2d2d')
        self.control_bar.pack(side=tk.TOP, fill=tk.X, padx=20, pady=5)

        # 内部容器实现按钮整体居中，与原逻辑完全一致
        inner_bar = tk.Frame(self.control_bar, bg='#2d2d2d')
        inner_bar.pack()

        btn_run_term = tk.Button(inner_bar, text="终端运行", bg="#44475a", fg="white",
                                 font=('DejaVu Sans', 30, 'bold'), width=12,
                                 command=self.run_selected_script_in_terminal)
        btn_run_term.pack(side=tk.LEFT, padx=5, pady=5)

        btn_stop = tk.Button(inner_bar, text="停止", bg="#44475a", fg="white",
                             font=('DejaVu Sans', 30, 'bold'), width=12,
                             command=self.stop_selected_script)
        btn_stop.pack(side=tk.LEFT, padx=5, pady=5)

        # 2. 滚动脚本列表区域
        list_container = tk.Frame(main_frame, bg='#2d2d2d')
        list_container.pack(expand=True, fill=tk.BOTH, padx=20, pady=5)

        self.script_canvas = tk.Canvas(list_container, bg='#2d2d2d', highlightthickness=0)
        scrollbar = tk.Scrollbar(list_container, orient="vertical", command=self.script_canvas.yview)

        # 实际放置按钮的内部容器
        self.script_container = tk.Frame(self.script_canvas, bg='#2d2d2d')
        self.script_container.bind(
            "<Configure>",
            lambda e: self.script_canvas.configure(scrollregion=self.script_canvas.bbox("all"))
        )
        canvas_window = self.script_canvas.create_window((0, 0), window=self.script_container, anchor="nw")
        self.script_canvas.configure(yscrollcommand=scrollbar.set)

        # 强制内部容器宽度跟随画布
        self.script_canvas.bind("<Configure>", lambda e: self.script_canvas.itemconfig(canvas_window, width=e.width))

        # 绑定鼠标滚轮事件（画布和按钮上都支持滚动）
        self.script_canvas.bind("<Button-4>", lambda e: self.script_canvas.yview_scroll(-1, "units"))
        self.script_canvas.bind("<Button-5>", lambda e: self.script_canvas.yview_scroll(1, "units"))

        self.script_canvas.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 3. 底部状态反馈条
        self.script_status_var = tk.StringVar(value="当前无脚本运行")
        status_label = tk.Label(main_frame, textvariable=self.script_status_var,
                                bg='#1e1e1e', fg='#50fa7b', font=('DejaVu Sans Mono', 12), height=2)
        status_label.pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=10)

        # 加载脚本配置并生成按钮
        self.load_and_create_script_buttons()

    def load_and_create_script_buttons(self):
        """解析 scripts.json 生成按钮，逻辑与原代码完全一致"""
        scripts = []
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts.json")
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    scripts = json.load(f)
            except Exception as e:
                print(f"读取脚本配置失败: {e}")
        else:
            # 默认示例脚本，与原代码保持一致
            scripts = [
                {"name": "相机标定脚本", "script": "/path/to/calib.py"},
                {"name": "点云调试接口", "script": "/path/to/debug_pointCloud.py"},
                {"name": "YOLOv5s 预处理", "script": "/path/to/yolo_pre.py"}
            ]

        # 清空已有按钮
        self.script_buttons = []
        for widget in self.script_container.winfo_children():
            widget.destroy()

        for s in scripts:
            name = s["name"]
            path = s.get("script", "")
            # 按钮样式与原代码完全一致：25号字体、高度2、左对齐、灰底白字
            btn = tk.Button(self.script_container, text=name, bg="#44475a", fg="white",
                            font=('DejaVu Sans', 25), height=2, anchor="w", padx=15)
            btn.pack(fill=tk.X, pady=4, expand=True)

            # 按钮上绑定滚轮事件，解决悬停时无法滚动的问题
            btn.bind("<Button-4>", lambda e: self.script_canvas.yview_scroll(-1, "units"))
            btn.bind("<Button-5>", lambda e: self.script_canvas.yview_scroll(1, "units"))

            # 绑定单选点击事件
            btn.config(command=lambda b=btn, n=name, p=path: self.select_script(b, n, p))
            self.script_buttons.append({"btn": btn, "name": name, "path": path})

            # 恢复之前的选中状态
            if self.selected_script_info and self.selected_script_info["path"] == path:
                btn.config(bg="white", fg="black")
                self.selected_script_info["btn"] = btn

    def select_script(self, btn, name, path):
        """脚本单选逻辑，与原代码完全一致"""
        # 全部恢复未选中状态
        for item in self.script_buttons:
            item["btn"].config(bg="#44475a", fg="white")
        # 当前项设为选中态：白底黑字
        btn.config(bg="white", fg="black")
        self.selected_script_info = {"name": name, "path": path, "btn": btn}

        # 更新底部状态栏
        if path in self.running_processes and self.running_processes[path].poll() is None:
            self.script_status_var.set(f"已选中: {name} (终端运行中...)")
        else:
            self.script_status_var.set(f"已选中: {name}")

    def run_selected_script_in_terminal(self):
        """在新终端中运行选中脚本，逻辑与原代码完全一致"""
        if not self.selected_script_info:
            self.script_status_var.set("[提示] 请先在下方列表中选择一个脚本")
            return

        name = self.selected_script_info["name"]
        path = self.selected_script_info["path"]

        if not path or (not os.path.exists(path) and not path.startswith("/path/to")):
            self.script_status_var.set(f"路径无效: {name}")
            return

        # 检查是否已在运行
        if path in self.running_processes and self.running_processes[path].poll() is None:
            self.script_status_var.set(f"[提示] {name} 已经在终端运行中")
            return

        self.script_status_var.set(f"正在打开新终端运行: {name} ...")
        print(f"Terminal running script: {path}")

        try:
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'

            # 根据脚本后缀自动选择运行方式
            if path.endswith('.sh'):
                exec_cmd = f"bash '{path}'; exec bash"
            else:
                exec_cmd = f"python3 '{path}'; exec bash"

            proc = subprocess.Popen(
                ['gnome-terminal', '--disable-factory', '--', 'bash', '-c', exec_cmd],
                env=env,
                preexec_fn=os.setsid  # 创建独立进程组，保证停止按钮可完整终止
            )
            self.running_processes[path] = proc
            self.script_status_var.set(f"[已运行] {name} 已在独立终端启动")

            # 刷新选中状态的运行提示
            self.select_script(self.selected_script_info["btn"], name, path)

        except Exception as e:
            self.script_status_var.set(f"[错误] {name} 启动失败")
            print(str(e))
            messagebox.showerror("启动失败", f"脚本 {name} 启动失败:\n{str(e)}")

    def stop_selected_script(self):
        """停止选中的脚本，终止整个终端进程组，与原逻辑一致"""
        if not self.selected_script_info:
            self.script_status_var.set("[提示] 请先选择要停止的脚本")
            return

        name = self.selected_script_info["name"]
        path = self.selected_script_info["path"]

        if path in self.running_processes and self.running_processes[path].poll() is None:
            proc = self.running_processes[path]
            try:
                # 杀死整个进程组，包含终端窗口和内部运行的子进程
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                self.script_status_var.set(f"[已停止] {name} 进程已强行终止")
                # 刷新选中状态
                self.select_script(self.selected_script_info["btn"], name, path)
            except Exception as e:
                self.script_status_var.set(f"[错误] 停止 {name} 失败")
                print(f"Kill terminal process group failed: {e}")
        else:
            self.script_status_var.set(f"[提示] {name} 当前没有在运行")

    def on_closing(self):
        """窗口关闭时终止所有运行中的脚本，再退出"""
        # 遍历终止所有进程
        for path, proc in self.running_processes.items():
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except Exception:
                    pass
        self.running_processes.clear()
        self.master.destroy()


if __name__ == '__main__':
    root = tk.Tk()
    app = ScriptRunnerUI(root)
    root.mainloop()
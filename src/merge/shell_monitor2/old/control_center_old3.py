#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from PIL import ImageDraw
from PIL import ImageFont
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
from PIL import Image, ImageTk
from std_msgs.msg import String, Int32  # 修改这一行，加上 Int32


class TouchscreenUI:
    def __init__(self, master):
        self.master = master
        self.master.title("ROS 触屏控制面板")
        # 触屏设备建议全屏或固定大尺寸
        # --- 修改：动态计算屏幕 3/4 大小并居中显示 ---
        screen_width = self.master.winfo_screenwidth()
        screen_height = self.master.winfo_screenheight()

        # 计算 3/4 的宽度和高度
        win_width = int(screen_width * 0.75)
        win_height = int(screen_height * 0.75)

        # 计算居中时的左上角坐标
        x_coordinate = int((screen_width - win_width) / 2)
        y_coordinate = int((screen_height - win_height) / 2)

        # 设置窗口大小及初始位置
        self.master.geometry(f"{win_width}x{win_height}+{x_coordinate}+{y_coordinate}")
        # --------------------------------------------

        self.master.configure(bg='#1e1e1e')

        # 初始化 ROS 节点与 Publisher
        rospy.init_node('touch_ui_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/ui_commands', String, queue_size=10)

        # 捕获窗口关闭事件，优雅退出 ROS
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # UI 样式配置
        self.setup_styles()

        # 主布局：分为左、中、右三个区块
        self.main_frame = tk.Frame(self.master, bg='#1e1e1e')
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.main_frame.columnconfigure(0, weight=5)  # 12宫格区
        self.main_frame.columnconfigure(1, weight=4)  # ROS指令区
        # self.main_frame.columnconfigure(2, weight=3)  # 脚本运行区
        self.main_frame.rowconfigure(0, weight=1)

        # 新增状态存储
        self.parsed_plans = {}  # 存放 9 种方案的解析结果
        self.selected_plan_key = None  # 当前选中的方案键值对，例如 (2, 3)
        self.plan_buttons = {}  # 存放表格按钮的实例引用
        # --- 修改结束 ---

        # 构建三大功能块
        self.build_grid_block(self.main_frame, col=0)
        self.build_ros_command_block(self.main_frame, col=1)
        # self.build_script_block(self.main_frame, col=2)

        # --- 修改开始：替换原有的单一 self.database_path ---
        self.save_map_path = r"/home/awwsome/RC/shell_monitor/test/map.txt"

        # 【注意】请将这里的路径替换为你实际的 9 个数据库文件的具体路径
        self.databases = {
            (1, 2): r"./path.txt",
            (1, 3): r"./path.txt",
            (1, 4): r"./path.txt",
            (2, 2): r"./path.txt",
            (2, 3): r"./path_3.txt",
            (2, 4): r"./path_3.txt",
            (3, 2): r"./path_3.txt",
            (3, 3): r"./path_3.txt",
            (3, 4): r"./path_3.txt",
        }
        self.target_path = r"../path/map.txt"

        # 初始化控制状态（默认发送 1）
        self.send_path_value = 1
        self.is_publishing = False  # 控制当前是否处于“正在发送”的激活状态

        # 注册发布者与订阅者
        self.send_path_pub = rospy.Publisher('/scripts/send_path', Int32, queue_size=10)
        self.status_sub = rospy.Subscriber('/scripts/response', Int32, self.status_callback)

        # # 启动程序即刻开始循环发送话题
        # self.start_publish_loop()

    def setup_styles(self):
        """配置大尺寸、适合触屏的 ttk 样式"""
        style = ttk.Style()
        style.theme_use('clam')

        # 状态按钮样式
        style.configure('Status.TButton', font=('DejaVu Sans', 14, 'bold'), padding=15)
        # ROS 命令按钮样式
        style.configure('Cmd.TButton', font=('DejaVu Sans', 16, 'bold'), padding=20)
        # 脚本运行按钮样式
        style.configure('Script.TButton', font=('DejaVu Sans', 14), padding=15)

        # 标题标签样式
        style.configure('Header.TLabel', background='#1e1e1e', foreground='#ffffff', font=('DejaVu Sans', 16, 'bold'))

    # ==========================================
    # 第一块：12宫格状态填涂区
    # ==========================================
    def build_grid_block(self, parent, col):
        frame = tk.Frame(parent, bg='#2d2d2d', bd=2, relief=tk.SUNKEN)
        frame.grid(row=0, column=col, sticky='nsew', padx=10, pady=10)

        ttk.Label(frame, text="梅林输入区", style='Header.TLabel', background='#2d2d2d').pack(pady=10)

        # --- 约束状态常驻显示区（保持在顶部） ---
        constraint_frame = tk.Frame(frame, bg='#2d2d2d')
        constraint_frame.pack(fill=tk.X, pady=5)

        self.pos_constraint_var = tk.StringVar(value="位置约束：不满足")
        self.qty_constraint_var = tk.StringVar(value="数量约束：不满足")

        self.pos_constraint_label = tk.Label(constraint_frame, textvariable=self.pos_constraint_var, fg="#ff5555",
                                             bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold'))
        self.pos_constraint_label.pack(side=tk.LEFT, expand=True)

        self.qty_constraint_label = tk.Label(constraint_frame, textvariable=self.qty_constraint_var, fg="#ff5555",
                                             bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold'))
        self.qty_constraint_label.pack(side=tk.LEFT, expand=True)

        # --- 新增：路径发送状态显示区 ---
        path_status_frame = tk.Frame(frame, bg='#2d2d2d')
        path_status_frame.pack(fill=tk.X, pady=2)

        self.path_status_var = tk.StringVar(value="当前路径：未发送")
        # 初始默认黄色 (#f1fa8c)
        self.path_status_label = tk.Label(path_status_frame, textvariable=self.path_status_var,
                                          fg="#f1fa8c", bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold'))
        self.path_status_label.pack()

        # 记录上一次发送成功的序列，用于防止同一路径重复发送
        self.last_sent_seq = None
        # --------------------------------

        # --- 修改：新增地图与按钮的左右组合容器 ---
        content_container = tk.Frame(frame, bg='#2d2d2d')
        content_container.pack(expand=True, fill=tk.BOTH, pady=10)

        # 12 宫格 Canvas 改为靠左，撑满左侧空间
        self.canvas = tk.Canvas(content_container, bg='#1e1e1e', highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)

        # 按钮功能区改为靠右，并向上向下填充
        btn_frame = tk.Frame(content_container, bg='#2d2d2d')
        btn_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=15)

        self.states_config = [
            {"name": "空", "color": "#ffffff", "rgb": (255, 255, 255)},
            {"name": "R1", "color": "#00ffff", "rgb": (0, 255, 255)},
            {"name": "R2", "color": "#ffff00", "rgb": (255, 255, 0)},
            {"name": "Fake", "color": "#ff0000", "rgb": (255, 0, 0)}
        ]

        # 按钮改为纵向平铺 (side=tk.TOP)，加个 width 确保触屏按压区域足够大
        for idx, state in enumerate(self.states_config):
            btn = tk.Button(btn_frame, text=state["name"], bg=state["color"],
                            font=('DejaVu Sans', 20, 'bold'), width=6,
                            command=lambda i=idx: self.apply_state_to_grid(i))
            btn.pack(side=tk.TOP, expand=True, fill=tk.BOTH, pady=5)

        reset_btn = tk.Button(btn_frame, text="全部\n重置", bg="#44475a", fg="white",
                              font=('DejaVu Sans', 20, 'bold'), width=6,
                              command=self.reset_all_grids)
        reset_btn.pack(side=tk.TOP, expand=True, fill=tk.BOTH, pady=5)

        # 新增：路径规划常驻展示区（放在 content_container 底部）
        # --- 修改开始：重写路径规划常驻展示区为左右分栏 ---
        path_display_frame = tk.Frame(frame, bg='#2d2d2d')
        path_display_frame.pack(fill=tk.X, expand=False, pady=(5, 5), padx=10)

        # 设置左右均等分栏 (1:1)
        path_display_frame.columnconfigure(0, weight=1)
        path_display_frame.columnconfigure(1, weight=1)

        # === 1. 左半边：表格区域 ===
        left_frame = tk.Frame(path_display_frame, bg='#2d2d2d')
        left_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 5))

        ttk.Label(left_frame, text="方案\n选择", style='Header.TLabel',
                  justify=tk.CENTER).pack(side=tk.LEFT, padx=(0, 5))

        table_frame = tk.Frame(left_frame, bg='#2d2d2d')
        table_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        for i in range(4):
            table_frame.columnconfigure(i, weight=1)
            table_frame.rowconfigure(i, weight=1)

        tk.Label(table_frame, text="R1\\R2", bg="#2d2d2d", fg="white", font=('DejaVu Sans', 12, 'bold')).grid(row=0,
                                                                                                              column=0,
                                                                                                              sticky='nsew',
                                                                                                              padx=1,
                                                                                                              pady=1)
        for col_idx, r2_cnt in enumerate([2, 3, 4], start=1):
            tk.Label(table_frame, text=f"{r2_cnt}个", bg="#2d2d2d", fg="white", font=('DejaVu Sans', 12, 'bold')).grid(
                row=0, column=col_idx, sticky='nsew', padx=1, pady=1)

        for row_idx, r1_cnt in enumerate([1, 2, 3], start=1):
            tk.Label(table_frame, text=f"{r1_cnt}个", bg="#2d2d2d", fg="white", font=('DejaVu Sans', 12, 'bold')).grid(
                row=row_idx, column=0, sticky='nsew', padx=1, pady=1)
            for col_idx, r2_cnt in enumerate([2, 3, 4], start=1):
                btn = tk.Button(table_frame, text="-", bg="#44475a", fg="white", font=('DejaVu Sans', 14, 'bold'),
                                command=lambda r1=r1_cnt, r2=r2_cnt: self.on_plan_selected(r1, r2))
                btn.grid(row=row_idx, column=col_idx, sticky='nsew', padx=1, pady=1)
                self.plan_buttons[(r1_cnt, r2_cnt)] = btn

        # === 2. 右半边：路径信息展示区 ===
        right_frame = tk.Frame(path_display_frame, bg='#2d2d2d')
        right_frame.grid(row=0, column=1, sticky='nsew', padx=(5, 0))

        info_container = tk.Frame(right_frame, bg='#2d2d2d')
        info_container.pack(expand=True, fill=tk.X)

        self.r2_path_var = tk.StringVar(value="R2 路径：暂无")
        self.r1_path_var = tk.StringVar(value="R1 夹取：暂无")

        # wraplength=300 防止路径太长导致UI变形，自动换行
        tk.Label(info_container, textvariable=self.r2_path_var, fg="#ffff00", bg="#2d2d2d",
                 font=('DejaVu Sans', 12, 'bold'), width=30, anchor=tk.W, justify=tk.RIGHT, wraplength=1000).pack(fill=tk.X, pady=2)
        tk.Label(info_container, textvariable=self.r1_path_var, fg="#00ffff", bg="#2d2d2d",
                 font=('DejaVu Sans', 12, 'bold'), width=30, anchor=tk.W, justify=tk.RIGHT, wraplength=1000).pack(fill=tk.X, pady=2)
        # --- 修改结束 ---

        # 参数配置（保持不变）
        self.rows, self.cols = 4, 3
        self.cell_w, self.cell_h = 1.0, 1.0
        self.current_cursor_idx = 0

        # 状态数据结构（保持不变）
        self.grid_states = {i: 0 for i in range(12)}
        self.overlay_images = {}
        self.overlay_items = {i: None for i in range(12)}
        self.cursor_rect = None

        # --- 新增：存储解析出的路径与目标动作（内部索引 0-11） ---
        self.r2_robot_path = []
        self.r2_targets = []
        self.r1_targets = []
        # ----------------------------------------------------

        self.set_grid_color_theme("map_blue")

        # 绑定事件
        self.canvas.bind("<Configure>", self.on_canvas_resize)
        self.canvas.bind("<Button-1>", self.on_canvas_click)

    def set_grid_state_visually(self, idx, state_idx):
        """核心绘图：仅更新视觉和数据，不移动光标（用于初始化和重置）"""
        self.grid_states[idx] = state_idx
        # 清除旧的蒙版
        if self.overlay_items[idx]:
            self.canvas.delete(self.overlay_items[idx])
        # 贴上新蒙版
        x1, y1 = self.get_coords_from_index(idx)
        self.overlay_items[idx] = self.canvas.create_image(
            round(x1 + self.cell_w / 8), round(y1 + self.cell_h / 8),
            anchor=tk.NW, image=self.overlay_images[state_idx]
        )

    def check_map_validity(self):
        """需求 2：检查位置与数量约束并更新UI颜色与文本，返回是否全部满足"""
        # 统计各状态数量
        counts = {0: 0, 1: 0, 2: 0, 3: 0}
        for state in self.grid_states.values():
            counts[state] += 1

        # 【数量约束】：空(0)=4, R1(1)=3, R2(2)=4, Fake(3)=1
        qty_ok = (counts[0] == 4 and counts[1] == 3 and counts[2] == 4 and counts[3] == 1)
        if qty_ok:
            self.qty_constraint_var.set("数量约束：满足")
            self.qty_constraint_label.config(fg="#50fa7b")  # 亮绿色
        else:
            self.qty_constraint_var.set("数量约束：不满足")
            self.qty_constraint_label.config(fg="#ff5555")  # 亮红色

        # 【位置约束】
        # Fake不能放 1, 2, 3 -> 对应底层 idx: 0, 1, 2
        # R1不能放 5 和 8 -> 对应底层 idx: 4, 7
        pos_ok = True
        for idx, state in self.grid_states.items():
            if state == 3 and idx in [0, 1, 2]:  # Fake(状态3) 不能在 1, 2, 3
                pos_ok = False
            if state == 1 and idx in [4, 7]:  # R1(状态1) 不能在 5 和 8
                pos_ok = False

        if pos_ok:
            self.pos_constraint_var.set("位置约束：满足")
            self.pos_constraint_label.config(fg="#50fa7b")  # 亮绿色
        else:
            self.pos_constraint_var.set("位置约束：不满足")
            self.pos_constraint_label.config(fg="#ff5555")  # 亮红色

        # --- 修改：判定合法后立刻触发解析，不合法则重置显示与数据 ---
        # --- 修改开始：check_map_validity 的结尾部分 ---
        is_valid = qty_ok and pos_ok
        if is_valid:
            self.update_parsed_path_display()
        else:
            # 清空表格状态与选中的计划
            for btn in getattr(self, 'plan_buttons', {}).values():
                btn.config(text="-", bg="#44475a", state=tk.DISABLED)
            self.selected_plan_key = None

            # <-- 新增以下两行 -->
            self.r2_path_var.set("R2 路径：暂无")
            self.r1_path_var.set("R1 夹取：暂无")

            self.r2_robot_path.clear()
            self.r2_targets.clear()
            self.r1_targets.clear()
            self.redraw_canvas()

        return is_valid
        # --- 修改结束 ---

    # --- 修改开始：替换并新增解析与选择逻辑 ---
    def update_parsed_path_display(self):
        """需求：立刻匹配9个数据库并解析各方案信息，更新表格展示"""
        import re
        self.parsed_plans.clear()

        mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
        current_seq = "".join([mapping[self.grid_states[i]] for i in range(12)])

        first_valid_key = None

        for (r1, r2), db_path in self.databases.items():
            plan_data = {"valid": False, "steps": 0, "r2_path": [], "r2_targets": [], "r1_targets": [], "raw": ""}
            matched_content = None

            if os.path.exists(db_path):
                try:
                    with open(db_path, "r", encoding="utf-8") as f:
                        for line in f:
                            if line.startswith(current_seq):
                                matched_content = line.strip()[12:].strip()
                                break
                except Exception as e:
                    rospy.logerr(f"读取数据库出错 {db_path}: {e}")

            if matched_content:
                processed_str = re.sub(r'\((.*?)\)', lambda m: m.group(0).replace(' ', ','), matched_content)
                steps = processed_str.strip().split()

                if "0" in steps:
                    idx = steps.index("0")
                    r2_part, r1_blocks = steps[:idx], steps[idx + 1:]

                    r2_path = []
                    r2_blocks = []
                    for step in r2_part:
                        if '(' in step:
                            match = re.match(r'(\d+)\((.*)\)', step)
                            if match:
                                r2_path.append(int(match.group(1)) - 1)
                                [r2_blocks.append(int(x) - 1) for x in match.group(2).split(',')]
                            else:
                                match = re.match(r'\((.*)\)', step)
                                [r2_blocks.append(int(x) - 1) for x in match.group(1).split(',')]
                        else:
                            r2_path.append(int(step) - 1)

                    # --- 修改：向 plan_data 字典中追加文字解析结果 ---
                    plan_data.update({
                        "valid": True,
                        "steps": len(r2_part),
                        "r2_path": r2_path,
                        "r2_targets": r2_blocks,
                        "r1_targets": [int(x) - 1 for x in r1_blocks],
                        "r2_str": " → ".join(r2_part) if r2_part else "原地待命",  # <-- 新增
                        "r1_str": " → ".join(r1_blocks) if r1_blocks else "原地待命",  # <-- 新增
                        "raw": matched_content
                    })
                    if first_valid_key is None:
                        first_valid_key = (r1, r2)

            self.parsed_plans[(r1, r2)] = plan_data

            # 更新UI表格表现
            btn = self.plan_buttons.get((r1, r2))
            if btn:
                if plan_data["valid"]:
                    btn.config(text=str(plan_data["steps"]), fg="white", state=tk.NORMAL)
                else:
                    btn.config(text="无解", fg="#ff5555", state=tk.DISABLED)  # 无解红色字体显示并不可点

        # 如果当前没有选中有效方案，自动选中查找到的第一个有效方案
        if first_valid_key and (
                self.selected_plan_key is None or not self.parsed_plans.get(self.selected_plan_key, {}).get(
                "valid")):
            self.on_plan_selected(*first_valid_key)
        elif first_valid_key is None:
            # 9种情况全无解
            self.selected_plan_key = None

            # <-- 新增以下两行 -->
            self.r2_path_var.set("R2 路径：无解")
            self.r1_path_var.set("R1 夹取：无解")

            self.r2_robot_path.clear()
            self.r2_targets.clear()
            self.r1_targets.clear()
            self.redraw_canvas()

    def on_plan_selected(self, r1, r2):
        """点击表格选中某组合方案并高亮"""
        plan = self.parsed_plans.get((r1, r2))
        if not plan or not plan["valid"]:
            return

        self.selected_plan_key = (r1, r2)

        # 刷新所有按钮外观：选中者反色高亮，其余恢复
        for key, btn in self.plan_buttons.items():
            if key == (r1, r2):
                btn.config(bg="#f1fa8c", fg="black")  # 选中反色高亮 (底色黄, 字黑)
            else:
                is_valid = self.parsed_plans.get(key, {}).get("valid", False)
                btn.config(bg="#44475a", fg="white" if is_valid else "#ff5555")

        # 切换地图渲染所需的缓存数据
        self.r2_robot_path = plan["r2_path"]
        self.r2_targets = plan["r2_targets"]
        self.r1_targets = plan["r1_targets"]

        # <-- 新增以下两行 -->
        self.r2_path_var.set(f"R2 路径：{plan['r2_str']}")
        self.r1_path_var.set(f"R1 夹取：{plan['r1_str']}")

        self.redraw_canvas()
    # --- 修改结束 ---


    def set_grid_color_theme(self, theme_name="map_blue"):
        """
        配置 12 宫格的不同纯色搭配方案。
        每种搭配由 12 个颜色的十六进制字符串组成，严格对应索引 0-11 的格子。
        """
        themes = {
            "theme_dark": [  # 纯暗色系
                "#282a36", "#282a36", "#282a36",
                "#282a36", "#282a36", "#282a36",
                "#282a36", "#282a36", "#282a36",
                "#282a36", "#282a36", "#282a36"
            ],
            "map_blue": [  # 200mm: #295210, 400mm: #2A7138, 600mm: #98A650
                "#2A7138", "#295210", "#2A7138",
                "#295210", "#2A7138", "#98A650",
                "#2A7138", "#98A650", "#2A7138",
                "#295210", "#2A7138", "#295210"
            ],
            "map_red": [  # 200mm: #295210, 400mm: #2A7138, 600mm: #98A650
                "#2A7138", "#295210", "#2A7138",
                "#98A650", "#2A7138", "#295210",
                "#2A7138", "#98A650", "#2A7138",
                "#295210", "#2A7138", "#295210"
            ]
        }

        # 加载指定主题，若编码不存在则 fallback 到 theme_dark
        self.current_theme_colors = themes.get(theme_name, themes["theme_dark"])

    def get_coords_from_index(self, idx):
        """将索引 0-11 映射为 UI 的 行、列
        排列规则:
        10 11 12 (row 0)
         7  8  9 (row 1)
         4  5  6 (row 2)
         1  2  3 (row 3)
        """
        row = 3 - (idx // 3)
        col = idx % 3
        x1 = col * self.cell_w
        y1 = row * self.cell_h
        return x1, y1

    def draw_grids(self):
        """绘制网格和编号，并填充纯色"""
        for idx in range(12):
            x1, y1 = self.get_coords_from_index(idx)
            x2, y2 = x1 + self.cell_w, y1 + self.cell_h
            cell_bg_color = self.current_theme_colors[idx]

            # 画底框并填充纯色
            self.canvas.create_rectangle(x1, y1, x2, y2, fill=cell_bg_color, outline="#1e1e1e", width=3)

            # 根据格子高度按比例缩放左下角数字字体
            # num_font_size = max(10, int(self.cell_h * 0.12))
            num_font_size = max(10, int(self.cell_h * 0.06 + self.cell_h * 0.06))
            self.canvas.create_text(round(x1 + self.cell_w / 5), round(y1 + self.cell_h * 4 / 5),
                                    text=str(idx + 1), fill="#ffffff", font=('DejaVu Sans', num_font_size, 'bold'))

    def update_cursor(self):
        """高亮当前选中的格子"""
        if self.cursor_rect:
            self.canvas.delete(self.cursor_rect)
        x1, y1 = self.get_coords_from_index(self.current_cursor_idx)
        x2, y2 = x1 + self.cell_w, y1 + self.cell_h
        self.cursor_rect = self.canvas.create_rectangle(x1 + 2, y1 + 2, x2 - 2, y2 - 2, outline="#f1fa8c", width=5)

    def on_canvas_click(self, event):
        """触屏点击，手动切换焦点"""
        col = int(event.x // self.cell_w)
        row = int(event.y // self.cell_h)
        # 反算索引
        idx = (3 - row) * 3 + col
        if 0 <= idx <= 11:
            self.current_cursor_idx = idx
            self.update_cursor()

    def apply_state_to_grid(self, state_idx):
        """将选定状态填入当前格子，并自动跳向下一格"""
        idx = self.current_cursor_idx

        # --- 新增：如果输入的是 Fake (状态索引为3)，则先清除场上已有的 Fake ---
        if state_idx == 3:
            for i in range(12):
                if self.grid_states[i] == 3:
                    self.set_grid_state_visually(i, 0)  # 将旧的 Fake 置为 0 (空)
        # -------------------------------------------------------------------

        self.set_grid_state_visually(idx, state_idx)

        # 自动切换到下一格 (如果到了12则停住或循环，这里设为循环到0)
        self.current_cursor_idx = (self.current_cursor_idx + 1) % 12
        self.update_cursor()
        self.check_map_validity()  # 修改后即刻检查

        # --- 新增：只要格子状态发生更改，就将 UI 变回未发送（黄色） ---
        if self.grid_states != self.last_sent_seq:
            self.path_status_var.set("当前路径：未发送")
            self.path_status_label.config(fg="#f1fa8c")  # 黄色
        # -----------------------------------------------------------

    def reset_all_grids(self):
        """重置所有格子状态"""
        for idx in range(12):
            self.set_grid_state_visually(idx, 0)

        self.current_cursor_idx = 0
        self.update_cursor()
        self.check_map_validity()  # 重置后即刻检查

        # --- 新增：只要格子状态发生更改，就将 UI 变回未发送（黄色） ---
        self.path_status_var.set("当前路径：未发送")
        self.path_status_label.config(fg="#f1fa8c")  # 黄色
        # -----------------------------------------------------------

    def save_the_map(self):
        """保存地图为txt文件，12个位置按顺序，逗号分隔"""
        try:
            with open(self.save_map_path, "w") as txt_file:
                for idx in range(12):
                    txt_file.write(f"{self.grid_states[idx]}")
                    if idx != 11:
                        txt_file.write(",")
            print("地图文件保存成功")
        except:
            print("地图文件保存失败")

    def generate_overlay_images(self):
        """根据当前格子大小，动态生成并缩放半透明文本蒙版"""
        self.overlay_images = {}
        # 字体大小随格子高度动态缩放
        font_size = max(16, int(self.cell_h * 0.25))
        try:
            my_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", font_size)
        except IOError:
            my_font = ImageFont.load_default()

        text_map = {0: "0", 1: "1", 2: "2", 3: "F"}
        for idx, state in enumerate(self.states_config):
            img = Image.new('RGBA', (round(self.cell_w * 3 / 4), round(self.cell_h * 3 / 4)), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img)
            # 居中对齐微调
            draw.text((int(self.cell_w * 0.18), int(self.cell_h * 0.18)), text_map[idx], fill=state["rgb"],
                      font=my_font)
            self.overlay_images[idx] = ImageTk.PhotoImage(img)

    def on_canvas_resize(self, event):
        """画布尺寸改变时的回调函数，负责重新绘制所有元素"""
        self.canvas_width = event.width
        self.canvas_height = event.height
        self.cell_w = self.canvas_width / self.cols
        self.cell_h = self.canvas_height / self.rows

        # 重新生成对应尺寸的图片并刷新画布
        self.generate_overlay_images()
        self.redraw_canvas()
        self.check_map_validity()

    def redraw_canvas(self):
        """重绘整个画布的内容"""
        self.canvas.delete("all")
        self.cursor_rect = None

        # 1. 绘制纯色底框与左下角编号
        self.draw_grids()

        # --- 修改：弃用图片蒙版，使用原生 create_text 将文字绘制在右侧居中 ---
        font_size = max(16, int(self.cell_h * 0.15))
        # print(font_size)
        text_map = {0: "0", 1: "1", 2: "2", 3: "F"}

        for idx, state_idx in self.grid_states.items():
            x1, y1 = self.get_coords_from_index(idx)
            text_color = self.states_config[state_idx]["color"]

            # 计算坐标：水平靠右（宽度 90% 处，留出边距），垂直居中（高度 50% 处）
            text_x = x1 + self.cell_w * 0.95
            text_y = y1 + self.cell_h / 2

            self.overlay_items[idx] = self.canvas.create_text(
                text_x, text_y,
                text=text_map[state_idx],
                fill=text_color,
                font=('DejaVu Sans', font_size, 'bold'),
                anchor=tk.E  # tk.E 表示以文本的右侧边框作为锚点
            )
        # -----------------------------------------------------------------

        # 3. 恢复高亮光标
        self.update_cursor()

        # 4. 在顶层绘制规划轨迹与动作框
        self.draw_parsed_paths_graphics()

    def draw_parsed_paths_graphics(self):
        """核心绘图：在画布上绘制 R2 蓝色路径线，以及 R1 的蓝色大叉叉和 R2 的黄色目标框"""
        # 依据格子大小计算动态比例因子，保证窗口缩放时路径线和箭头按比例变化
        scale = min(self.cell_w, self.cell_h)
        line_w = max(2, int(scale * 0.05))
        arrow_shape = (max(10, int(scale * 0.15)),  # 箭头长度
                       max(12, int(scale * 0.18)),  # 箭头宽度
                       max(4, int(scale * 0.06)))  # 箭头尾部内收

        box_line_w = max(2, int(scale * 0.03))

        # 1. 绘制 R2 机器人移动路径
        if len(self.r2_robot_path) > 1:
            for i in range(len(self.r2_robot_path) - 1):
                idx1 = self.r2_robot_path[i]
                idx2 = self.r2_robot_path[i + 1]

                x1_f, y1_f = self.get_coords_from_index(idx1)
                x2_f, y2_f = self.get_coords_from_index(idx2)

                cx1, cy1 = x1_f + self.cell_w / 2, y1_f + self.cell_h / 2
                cx2, cy2 = x2_f + self.cell_w / 2, y2_f + self.cell_h / 2

                self.canvas.create_line(cx1, cy1, cx2, cy2, fill="#FFFF00", width=line_w,
                                        arrow=tk.LAST, arrowshape=arrow_shape)

        # 2. 绘制 R2 夹取的方块：修改为【黄色大框】圈住右侧字体区域
        for idx in self.r2_targets:
            x1, y1 = self.get_coords_from_index(idx)
            bx1, by1 = x1 + self.cell_w * 0.65, y1 + self.cell_h * 0.25
            bx2, by2 = x1 + self.cell_w * 0.95, y1 + self.cell_h * 0.75
            self.canvas.create_rectangle(bx1, by1, bx2, by2, outline="#ffff00", width=box_line_w)

        # 3. 绘制 R1 夹取的方块：修改为【蓝色大叉叉】(两条交叉线线覆盖右侧字体区域)
        for idx in self.r1_targets:
            x1, y1 = self.get_coords_from_index(idx)
            bx1, by1 = x1 + self.cell_w * 0.60, y1 + self.cell_h * 0.20
            bx2, by2 = x1 + self.cell_w * 0.98, y1 + self.cell_h * 0.80

            # 绘制对角线1：左上到右下
            self.canvas.create_line(bx1, by1, bx2, by2, fill="#00ffff", width=box_line_w)
            # 绘制对角线2：左下到右上
            self.canvas.create_line(bx1, by2, bx2, by1, fill="#00ffff", width=box_line_w)

    # ==========================================
    # 第二块：ROS 话题实时发布区
    # ==========================================
    def build_ros_command_block(self, parent, col):
        frame = tk.Frame(parent, bg='#2d2d2d', bd=2, relief=tk.SUNKEN)
        frame.grid(row=0, column=col, sticky='nsew', padx=10, pady=10)

        ttk.Label(frame, text="ROS 实时控制区", style='Header.TLabel', background='#2d2d2d').pack(pady=10)

        btn_container = tk.Frame(frame, bg='#2d2d2d')
        btn_container.pack(expand=True, fill=tk.BOTH, padx=20, pady=20)

        commands = [
            {"label": "重定位", "cmd": "relocation", "params": {}},
        ]

        for c in commands:
            bg_color = c.get("color", "#58a6ff")
            btn = tk.Button(btn_container, text=c["label"], bg=bg_color, fg="white",
                            font=('DejaVu Sans', 16, 'bold'),
                            command=lambda cmd=c["cmd"], p=c["params"]: self.send_ros_command(cmd, p))
            # 修改：fill=tk.BOTH, expand=True
            btn.pack(fill=tk.BOTH, expand=True, pady=10)

        # 发送路径按钮也同样修改
        send_path_btn = tk.Button(btn_container, text="发送路径", bg="#2b9348", fg="white",
                                  font=('DejaVu Sans', 16, 'bold'),
                                  command=self.execute_send_path)
        # 修改：fill=tk.BOTH, expand=True
        send_path_btn.pack(fill=tk.BOTH, expand=True, pady=10)

    def start_publish_loop(self):
        """修改：只有当处于激活发送状态时，才高频周期性发送"""
        if self.is_publishing and not rospy.is_shutdown():
            self.send_path_pub.publish(self.send_path_value)
            # 每 100 毫秒（10Hz频率）发送一次
            self.master.after(100, self.start_publish_loop)

    def status_callback(self, msg):
        """修改：收到底层回传的相同数字时，立刻停止发送话题，并交替切换下一次的目标值"""
        if self.is_publishing and msg.data == self.send_path_value:
            self.is_publishing = False  # 【立刻停止发送】

            # 变绿提示已成功同步
            self.path_status_var.set("当前路径：已发送")
            self.path_status_label.config(fg="#50fa7b")  # 亮绿色

            # 交替切换下一次的数值：1 -> 2, 2 -> 1
            self.send_path_value = 2 if self.send_path_value == 1 else 1
            rospy.loginfo(f"接收到匹配应答 {msg.data}，已立刻停止发送。下一次发送值将为: {self.send_path_value}")

    # --- 修改开始：精简和修改 execute_send_path ---
    def execute_send_path(self):
        """发送路径：提取目前选中的方案直接写入"""
        if self.is_publishing:
            messagebox.showwarning("提示", "当前路径正在发送中，请等待底层应答...")
            return

        if not self.check_map_validity():
            messagebox.showwarning("拦截", "当前地图不满足约束条件，禁止发送路径！")
            return

        if not self.selected_plan_key or not self.parsed_plans.get(self.selected_plan_key, {}).get("valid"):
            messagebox.showwarning("拦截", "当前未在表格中选择有效的路径方案！")
            return

        try:
            mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
            current_seq = "".join([mapping[self.grid_states[i]] for i in range(12)])

            # 直接提取之前缓存好的目标行数据
            matched_content = self.parsed_plans[self.selected_plan_key]["raw"]

            # 原封不动写入目标文件
            with open(self.target_path, "w", encoding="utf-8") as f_out:
                f_out.write(matched_content + "\n")

            self.last_sent_seq = current_seq
            self.path_status_var.set(f"当前路径：正在发送 ({self.send_path_value})...")
            self.path_status_label.config(fg="#f1fa8c")  # 恢复黄色准备发送

            self.is_publishing = True
            self.start_publish_loop()

            rospy.loginfo(
                f"梅林序列方案 {self.selected_plan_key} 发送成功，开始常驻发送数字: {self.send_path_value}")

        except Exception as e:
            error_msg = str(e)
            rospy.logerr(f"发送路径失败: {error_msg}")
            messagebox.showerror("失败", f"发送路径发生错误:\n{error_msg}")
    # --- 修改结束 ---

    def send_ros_command(self, cmd_name, params):
        """将指令序列化为 JSON 并通过 std_msgs/String 发送"""
        payload = {
            "cmd": cmd_name,
            "params": params,
            "timestamp": rospy.get_time()
        }
        msg = json.dumps(payload)
        self.cmd_pub.publish(msg)
        rospy.loginfo(f"Published command: {msg}")

    # # ==========================================
    # # 第三块：本地脚本极简运行区
    # # ==========================================
    # def build_script_block(self, parent, col):
    #     frame = tk.Frame(parent, bg='#2d2d2d', bd=2, relief=tk.SUNKEN)
    #     frame.grid(row=0, column=col, sticky='nsew', padx=10, pady=10)
    #
    #     ttk.Label(frame, text="后台脚本执行", style='Header.TLabel', background='#2d2d2d').pack(pady=10)
    #
    #     self.script_container = tk.Frame(frame, bg='#2d2d2d')
    #     self.script_container.pack(expand=True, fill=tk.BOTH, padx=20, pady=10)
    #
    #     # 极简的状态反馈条
    #     self.script_status_var = tk.StringVar()
    #     self.script_status_var.set("当前无脚本运行")
    #     status_label = tk.Label(frame, textvariable=self.script_status_var,
    #                             bg='#1e1e1e', fg='#50fa7b', font=('DejaVu Sans Mono', 12), height=2)
    #     status_label.pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=10)
    #
    #     self.load_and_create_script_buttons()
    #
    # def load_and_create_script_buttons(self):
    #     """解析 scripts.json 并生成大块按钮"""
    #     scripts = []
    #     # 假定与你原项目相同的配置文件逻辑
    #     config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts.json")
    #
    #     if os.path.exists(config_path):
    #         try:
    #             with open(config_path, 'r', encoding='utf-8') as f:
    #                 scripts = json.load(f)
    #         except Exception as e:
    #             rospy.logerr(f"读取脚本配置失败: {e}")
    #     else:
    #         # Fallback 模拟数据
    #         scripts = [
    #             {"name": "相机标定脚本", "script": "/path/to/calib.py"},
    #             {"name": "点云调试接口", "script": "/path/to/debug_pointCloud.py"},
    #             {"name": "YOLOv5s 预处理", "script": "/path/to/yolo_pre.py"}
    #         ]
    #
    #     for s in scripts:
    #         btn = tk.Button(self.script_container, text=s["name"], bg="#44475a", fg="white",
    #                         font=('DejaVu Sans', 14), height=2,
    #                         command=lambda path=s.get("script", ""), name=s["name"]: self.run_script_silently(name,
    #                                                                                                           path))
    #         btn.pack(fill=tk.X, pady=8)
    #
    # def run_script_silently(self, name, path):
    #     """纯后台静默执行脚本，只更新底部状态栏"""
    #     if not path or (not os.path.exists(path) and not path.startswith("/path/to")):
    #         self.script_status_var.set(f"路径无效: {name}")
    #         return
    #
    #     self.script_status_var.set(f"正在运行: {name} ...")
    #     rospy.loginfo(f"Background running script: {path}")
    #
    #     # 模拟启动进程
    #     try:
    #         env = os.environ.copy()
    #         env['PYTHONUNBUFFERED'] = '1'
    #         subprocess.Popen(
    #             [path],
    #             stdout=subprocess.DEVNULL,
    #             stderr=subprocess.DEVNULL,
    #             stdin=subprocess.DEVNULL,
    #             env=env,
    #             preexec_fn=os.setsid
    #         )
    #         self.script_status_var.set(f"[成功] {name} 已在后台启动")
    #     except Exception as e:
    #         self.script_status_var.set(f"[错误] {name} 启动失败")
    #         rospy.logerr(str(e))
    #
    def on_closing(self):
        """窗口关闭处理"""
        rospy.signal_shutdown("UI Window Closed")
        self.master.destroy()


if __name__ == '__main__':
    try:
        root = tk.Tk()
        app = TouchscreenUI(root)
        # Tkinter 主循环会阻塞，正好维持 ROS 节点存活
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
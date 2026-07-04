#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
# 在 import rospy 之前强行指定 ROS 日志配置文件路径
if 'ROS_ETC_DIR' not in os.environ:
    os.environ['ROS_ETC_DIR'] = '/opt/ros/noetic/etc/ros'
import rospy
from PIL import ImageDraw
from PIL import ImageFont
import tkinter as tk
from tkinter import ttk, messagebox
import json
from PIL import Image, ImageTk
from std_msgs.msg import Int32
import subprocess
import signal
from collections import deque

class TouchscreenUI:
    def __init__(self, master):
        self.master = master
        self.master.title("RC 触屏控制面板")
        # 触屏设备建议全屏或固定大尺寸
        # --- 修改：动态计算屏幕 3/4 大小并居中显示 ---
        screen_width = self.master.winfo_screenwidth()
        screen_height = self.master.winfo_screenheight()

        # 计算 3/4 的宽度和高度
        win_width = int(screen_width * 0.85)
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

        # 捕获窗口关闭事件，优雅退出 ROS
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # UI 样式配置
        self.setup_styles()

        # 主布局：分为左、右两个区块
        self.main_frame = tk.Frame(self.master, bg='#1e1e1e')
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.main_frame.columnconfigure(0, weight=15)  # 12宫格区
        self.main_frame.columnconfigure(1, weight=1)  # ROS指令区
        self.main_frame.rowconfigure(0, weight=1)

        # 新增状态存储
        self.parsed_plans = {}  # 存放 9 种方案的解析结果
        self.selected_plan_key = None  # 当前选中的方案键值对，例如 (2, 3)
        self.plan_buttons = {}  # 存放表格按钮的实例引用
        # --- 修改结束 ---

        # --- 新增：红蓝半场初始化变量 ---
        self.is_red_court = True
        self.bg_red = "#FF7F50"  # 偏暗红色调
        self.bg_blue = "#00BFFF"  # 偏暗蓝色调

        # 构建功能块（将左侧区合并为一个带切换功能的复合面板）
        self.build_left_panel(self.main_frame, col=0)
        self.build_ros_command_block(self.main_frame, col=1)
        # self.build_script_block(self.main_frame, col=2) 这行可以删除了

        # 【注意】请将这里的路径替换为你实际的 9 个数据库文件的具体路径
        self.databases = {
            (1, 2): r"./path_12.txt",
            (1, 3): r"./path_13.txt",
            (1, 4): r"./path_14.txt",
            (2, 2): r"./path_22.txt",
            (2, 3): r"./path_23.txt",
            (2, 4): r"./path_24.txt",
            (3, 2): r"./path_32.txt",
            (3, 3): r"./path_33.txt",
            (3, 4): r"./path_34.txt",
        }
        self.target_path = r"../path/map.txt"

        # 初始化控制状态（默认发送 1）
        self.send_path_value = 1
        self.is_publishing = False  # 控制当前是否处于“正在发送”的激活状态

        # ========== 指令队列核心变量 ==========
        self.command_queue = deque()    # 指令队列，存储指令类型标识
        self.current_command = None     # 当前正在执行的指令类型，None表示空闲
        self.current_pub = None         # 当前发送使用的publisher
        self.current_send_value = None  # 当前发送的数值
        self.is_publishing = False      # 发送循环控制标志

        # ========== 路径指令（原有逻辑整合） ==========
        self.send_path_value = 1
        self.send_path_pub = rospy.Publisher('/scripts/send_path', Int32, queue_size=10)

        # ========== 重启雷达指令 ==========
        self.restart_lidar_value = 3  # 用3/4交替，与路径的1/2区分，避免应答冲突
        self.lidar_restart_pub = rospy.Publisher('/scripts/restart_lidar', Int32, queue_size=10)

        # ========== 公共应答订阅（原有不变） ==========
        self.status_sub = rospy.Subscriber('/scripts/response', Int32, self.status_callback)

    def get_current_merlin_seq(self):
        """统一获取序列：底层真实存储状态生成"""
        mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
        # 还原为直接映射，不要在这里做对调，交由 UI 坐标系统一处理
        return "".join([mapping[self.grid_states[i]] for i in range(12)])

    def toggle_court_side(self):
        """处理红蓝半场切换按钮逻辑"""
        self.is_red_court = not self.is_red_court
        self.update_court_ui()

        # 切换半场后映射可能改变，需重新检验地图状态并更新路径展示
        self.check_map_validity()
        self.redraw_canvas()

    def update_court_ui(self):
        """递归刷新梅林输入区背景色，并更新按钮与 12 宫格主题"""
        target_bg = self.bg_red if self.is_red_court else self.bg_blue
        btn_text = "切换为蓝场" if self.is_red_court else "切换为红场"

        self.court_toggle_btn.config(text=btn_text)

        # 递归修改区域内元素的背景色
        def change_bg(widget):
            # 排除由于自带 Style/原生组件而不应强制变色的部分
            if 'ttk' in str(type(widget)) or isinstance(widget, tk.Button):
                return
            # 排除方案表格中代表计算结果的标签 (因为它们自带高亮逻辑)
            if widget in getattr(self, 'plan_buttons', {}).values():
                return

            if isinstance(widget, (tk.Frame, tk.Canvas, tk.Label)):
                try:
                    widget.config(bg=target_bg)
                except tk.TclError:
                    pass

            for child in widget.winfo_children():
                change_bg(child)

        # split_container 为存放宫格与列表的主容器
        if hasattr(self, 'split_container'):
            change_bg(self.split_container)

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
    # 左侧综合面板（包含切换逻辑）
    # ==========================================
    def build_left_panel(self, parent, col):
        left_panel = tk.Frame(parent, bg='#2d2d2d', bd=2, relief=tk.SUNKEN)
        left_panel.grid(row=0, column=col, sticky='nsew', padx=10, pady=10)

        # 标题栏兼切换按钮区 (放大按钮，充当 Tab 标题)
        title_bar = tk.Frame(left_panel, bg='#2d2d2d')
        title_bar.pack(fill=tk.X, pady=10, padx=10)

        self.btn_grid_view = tk.Button(title_bar, text="梅林输入区", font=('DejaVu Sans', 18, 'bold'),
                                       command=lambda: self.switch_left_view('grid'))
        self.btn_grid_view.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 5))

        self.btn_script_view = tk.Button(title_bar, text="脚本运行区", font=('DejaVu Sans', 18, 'bold'),
                                         command=lambda: self.switch_left_view('script'))
        self.btn_script_view.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5, 0))

        # 内容容器
        self.left_content_container = tk.Frame(left_panel, bg='#2d2d2d')
        self.left_content_container.pack(expand=True, fill=tk.BOTH)

        # 预先构建两个子页面
        self.grid_frame = tk.Frame(self.left_content_container, bg='#2d2d2d')
        self.build_grid_block_content(self.grid_frame)

        self.script_frame = tk.Frame(self.left_content_container, bg='#2d2d2d')
        self.build_script_block_content(self.script_frame)

        # 默认显示梅林输入区
        self.switch_left_view('grid')

    def switch_left_view(self, view_name):
        """切换左侧页面显示并更新按钮样式"""
        if view_name == 'grid':
            self.btn_grid_view.config(bg="white", fg="black")  # 选中态：白底黑字
            self.btn_script_view.config(bg="#44475a", fg="white")  # 未选中：灰底白字
            self.script_frame.pack_forget()
            self.grid_frame.pack(expand=True, fill=tk.BOTH)
        else:
            self.btn_grid_view.config(bg="#44475a", fg="white")  # 未选中：灰底白字
            self.btn_script_view.config(bg="white", fg="black")  # 选中态：白底黑字
            self.grid_frame.pack_forget()
            self.script_frame.pack(expand=True, fill=tk.BOTH)

    # ==========================================
    # 第一块内容：12宫格状态填涂区 (原 build_grid_block)
    # ==========================================
    def build_grid_block_content(self, parent):
        frame = parent  # 直接绑定到传入的子页面容器，不再重新创建带有 Sunken 的外框

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

        # 新增一个内部 Frame 来居中对齐两个 Label
        self.inner_status_frame = tk.Frame(path_status_frame, bg='#2d2d2d')
        self.inner_status_frame.pack()

        # 第一部分 Label (主状态：绿色或黄色)
        self.path_status_var_1 = tk.StringVar(value="此地图未发送任何路径")
        self.path_status_label_1 = tk.Label(self.inner_status_frame, textvariable=self.path_status_var_1,
                                            fg="#f1fa8c", bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold'))
        self.path_status_label_1.pack(side=tk.LEFT)

        # 第二部分 Label (附加状态：专门显示黄色的选中未发送)
        self.path_status_var_2 = tk.StringVar(value="")
        self.path_status_label_2 = tk.Label(self.inner_status_frame, textvariable=self.path_status_var_2,
                                            fg="#f1fa8c", bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold'))
        self.path_status_label_2.pack(side=tk.LEFT)

        self.last_sent_seq = None
        self.current_map_sent_plan = None
        # --------------------------------

        # --- 修改：将 split_container 设为实例变量，不再是局部变量 ---
        self.split_container = tk.Frame(frame, bg='#2d2d2d')
        self.split_container.pack(expand=True, fill=tk.BOTH, pady=10)

        # 同样，底下绑定权重的方法也加上 self.
        self.split_container.rowconfigure(0, weight=7, uniform="vert_split")
        self.split_container.rowconfigure(1, weight=3, uniform="vert_split")
        self.split_container.columnconfigure(0, weight=1)

        # 它的子容器实例化时也要引用 self.split_container
        content_container = tk.Frame(self.split_container, bg='#2d2d2d')
        content_container.grid(row=0, column=0, sticky='nsew', pady=(0, 5))

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

        # --- 修改：路径展示区放在第 1 行 (占比 4) ---
        path_display_frame = tk.Frame(self.split_container, bg='#2d2d2d')
        path_display_frame.grid(row=1, column=0, sticky='nsew', pady=(5, 0), padx=10)

        # 核心修改：强制左右均等分栏 1:1，并允许垂直拉伸
        path_display_frame.columnconfigure(0, weight=1, uniform="horiz_split")
        path_display_frame.columnconfigure(1, weight=1, uniform="horiz_split")
        path_display_frame.rowconfigure(0, weight=1)

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

        tk.Label(table_frame, text="R1\\R2", bg="#2d2d2d", fg="white", font=('DejaVu Sans', 20, 'bold')).grid(row=0,
                                                                                                              column=0,
                                                                                                              sticky='nsew',
                                                                                                              padx=1,
                                                                                                              pady=1)
        for col_idx, r2_cnt in enumerate([2, 3, 4], start=1):
            tk.Label(table_frame, text=f"{r2_cnt}个", bg="#2d2d2d", fg="white", font=('DejaVu Sans', 20, 'bold')).grid(
                row=0, column=col_idx, sticky='nsew', padx=1, pady=1)

        for row_idx, r1_cnt in enumerate([1, 2, 3], start=1):
            tk.Label(table_frame, text=f"{r1_cnt}个", bg="#2d2d2d", fg="white", font=('DejaVu Sans', 20, 'bold')).grid(
                row=row_idx, column=0, sticky='nsew', padx=1, pady=1)
            for col_idx, r2_cnt in enumerate([2, 3, 4], start=1):
                # 修改：由 tk.Button 改为 tk.Label，去掉了 command，改用 bind 绑定点击事件
                lbl = tk.Label(table_frame, text="-", bg="#44475a", fg="white", width=5, font=('DejaVu Sans', 30, 'bold'))
                lbl.grid(row=row_idx, column=col_idx, sticky='nsew', padx=1, pady=1)
                lbl.bind("<Button-1>", lambda e, r1=r1_cnt, r2=r2_cnt: self.on_plan_selected(r1, r2))
                self.plan_buttons[(r1_cnt, r2_cnt)] = lbl

        # === 2. 右半边：路径信息展示区 ===
        right_frame = tk.Frame(path_display_frame, bg='#2d2d2d')
        right_frame.grid(row=0, column=1, sticky='nsew', padx=(5, 0))

        # --- 修改：重新配置 right_frame，给上方按钮腾出空间 ---
        right_frame.rowconfigure(0, weight=0)  # 第 0 行放按钮
        right_frame.rowconfigure(1, weight=1)  # 第 1 行放原先的文字信息
        right_frame.columnconfigure(0, weight=1)

        # --- 新增：红蓝半场切换按钮 (放于右上方) ---
        self.court_toggle_btn = tk.Button(right_frame, text="切换为蓝场", font=('DejaVu Sans', 14, 'bold'),
                                          command=self.toggle_court_side)
        self.court_toggle_btn.grid(row=0, column=0, sticky='ne', padx=5, pady=5)

        # 修改：将 info_container 放入第 1 行
        info_container = tk.Frame(right_frame, bg='#2d2d2d')
        info_container.grid(row=1, column=0, sticky='w')

        self.r2_path_var = tk.StringVar(value="R2 路径：暂无")
        self.r1_path_var = tk.StringVar(value="R1 夹取：暂无")

        # anchor=tk.W 确保文字在 Label 宽度内靠左，pady 增加行间距防止拥挤
        tk.Label(info_container, textvariable=self.r2_path_var, fg="#ffff00", bg="#2d2d2d",
                 font=('DejaVu Sans', 20, 'bold'), anchor=tk.W, width=35, justify=tk.LEFT, wraplength=1200).pack(fill=tk.X,
                                                                                                       pady=5)
        tk.Label(info_container, textvariable=self.r1_path_var, fg="#00ffff", bg="#2d2d2d",
                 font=('DejaVu Sans', 20, 'bold'), anchor=tk.W, width=35, justify=tk.LEFT, wraplength=1200).pack(fill=tk.X,
                                                                                                       pady=5)

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

        # --- 新增：强制应用一次初始化红蓝半场样式 ---
        self.update_court_ui()

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
            for lbl in getattr(self, 'plan_buttons', {}).values():
                # 修改：Label 统一恢复为灰色背景、白色字体
                lbl.config(text="-", bg="#44475a", fg="white")
            self.selected_plan_key = None

            self.r2_path_var.set("R2 路径：暂无")
            self.r1_path_var.set("R1 夹取：暂无")

            self.r2_robot_path.clear()
            self.r2_targets.clear()
            self.r1_targets.clear()
            self.redraw_canvas()

        return is_valid

    def update_path_status_display(self):
        """根据当前地图发送状态和表格选中状态，动态更新提示文本与颜色"""
        if self.is_publishing:
            return  # 如果正在等待底层应答，保持“正在发送...”状态不被覆盖

        if not self.current_map_sent_plan:
            self.path_status_var_1.set("此地图未发送任何路径")
            self.path_status_label_1.config(fg="#f1fa8c")  # 黄色
            self.path_status_var_2.set("")  # 清空第二部分
        else:
            sent_r1, sent_r2 = self.current_map_sent_plan
            base_text = f"此地图已发送路径：R1夹{sent_r1}，R2夹{sent_r2}"

            if self.selected_plan_key == self.current_map_sent_plan:
                # 选中的正是已发送的方案
                self.path_status_var_1.set(base_text)
                self.path_status_label_1.config(fg="#50fa7b")  # 第一部分变绿色
                self.path_status_var_2.set("")  # 清空第二部分
            else:
                # 选中的方案与已发送的方案不一致
                self.path_status_var_1.set(base_text)
                self.path_status_label_1.config(fg="#50fa7b")  # 第一部分保持绿色
                self.path_status_var_2.set(" | 选中方案未发送")
                self.path_status_label_2.config(fg="#f1fa8c")  # 第二部分为黄色

    # --- 修改开始：替换并新增解析与选择逻辑 ---
    def update_parsed_path_display(self):
        """需求：立刻匹配9个数据库并解析各方案信息，更新表格展示"""
        import re
        self.parsed_plans.clear()

        # mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
        # current_seq = "".join([mapping[self.grid_states[i]] for i in range(12)])
        # 修改后代码：
        current_seq = self.get_current_merlin_seq()

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
                        "steps": len(r2_path),
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

            self.parsed_plans[(r1, r2)] = plan_data

            # 更新UI表格表现
            lbl = self.plan_buttons.get((r1, r2))
            if lbl:
                if plan_data["valid"]:
                    # 有解时，未选中状态默认为灰色背景、白色字体
                    lbl.config(text=str(plan_data["steps"]), bg="#44475a", fg="white")
                else:
                    # 无解时，固定为灰色背景、红色字体
                    lbl.config(text="无解", bg="#44475a", fg="#ff0000")

        # 如果当前没有选中有效方案，自动选中查找到的第一个有效方案
        if first_valid_key and (
                self.selected_plan_key is None or not self.parsed_plans.get(self.selected_plan_key, {}).get(
                "valid")):
            self.on_plan_selected(*first_valid_key)

        # --- 新增这部分：如果当前已有选中的有效方案，缩放窗口时重新触发高亮以恢复 UI ---
        elif self.selected_plan_key and self.parsed_plans.get(self.selected_plan_key, {}).get("valid"):
            self.on_plan_selected(*self.selected_plan_key)

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
            return  # 拦截：无解时无法选中

        self.selected_plan_key = (r1, r2)

        # 刷新所有文本组件外观
        for key, lbl in self.plan_buttons.items():
            if key == (r1, r2):
                lbl.config(bg="#f1fa8c", fg="black")  # 选中：浅黄色背景，黑色字体
            else:
                is_valid = self.parsed_plans.get(key, {}).get("valid", False)
                # 未选中：有解则灰底白字，无解则灰底红字
                lbl.config(bg="#44475a", fg="white" if is_valid else "#ff0000")

        # 切换地图渲染所需的缓存数据
        self.r2_robot_path = plan["r2_path"]
        self.r2_targets = plan["r2_targets"]
        self.r1_targets = plan["r1_targets"]

        self.r2_path_var.set(f"R2 路径：{plan['r2_str']}")
        self.r1_path_var.set(f"R1 夹取：{plan['r1_str']}")

        self.redraw_canvas()

        self.update_path_status_display()

    def set_grid_color_theme(self, theme_name="map_blue"):
        """
        颜色规则：颜色与显示序号强绑定，红蓝场通用
        序号1-12对应固定色值，位置镜像时颜色自动跟随序号
        """
        # 核心映射：显示序号 → 标准颜色（蓝场基准）
        self.number_color_map = {
            1: "#295210",  # 200mm
            2: "#2A7138",  # 400mm
            3: "#295210",  # 200mm
            4: "#2A7138",  # 400mm
            5: "#98A650",  # 600mm
            6: "#2A7138",  # 400mm
            7: "#295210",  # 200mm
            8: "#2A7138",  # 400mm
            9: "#98A650",  # 600mm
            10: "#2A7138",  # 400mm
            11: "#295210",  # 200mm
            12: "#2A7138"  # 400mm
        }
        # 保留深色主题备用
        self.dark_color = "#282a36"

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

        # --- 新增：红蓝半场 UI 视觉上的左右镜像映射 ---
        if self.is_red_court:
            if col == 0:
                col = 2
            elif col == 2:
                col = 0
        # ------------------------------------------

        x1 = col * self.cell_w
        y1 = row * self.cell_h
        return x1, y1

    def draw_grids(self):
        """绘制网格和编号，颜色与显示序号强绑定"""
        for idx in range(12):
            x1, y1 = self.get_coords_from_index(idx)
            x2, y2 = x1 + self.cell_w, y1 + self.cell_h

            # 核心：当前格子显示的序号就是 idx+1，直接用序号取色
            display_num = idx + 1
            cell_bg_color = self.number_color_map.get(display_num, self.dark_color)

            # 画底框并填充纯色
            self.canvas.create_rectangle(x1, y1, x2, y2, fill=cell_bg_color, outline="#1e1e1e", width=3)

            # 绘制左下角序号
            num_font_size = max(10, int(self.cell_h * 0.06 + self.cell_h * 0.06))
            self.canvas.create_text(
                round(x1 + self.cell_w / 5), round(y1 + self.cell_h * 4 / 5),
                text=str(display_num), fill="#ffffff",
                font=('DejaVu Sans', num_font_size, 'bold')
            )

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

        # --- 新增：触屏点击区域同样进行镜像反算 ---
        if self.is_red_court:
            if col == 0:
                col = 2
            elif col == 2:
                col = 0
        # --------------------------------------

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
        # mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
        # current_seq = "".join([mapping[self.grid_states[i]] for i in range(12)])
        # 修改后代码：
        current_seq = self.get_current_merlin_seq()

        if current_seq != self.last_sent_seq:
            self.current_map_sent_plan = None
            self.update_path_status_display()
        # -----------------------------------------------------------

    def reset_all_grids(self):
        """重置所有格子状态"""
        for idx in range(12):
            self.set_grid_state_visually(idx, 0)

        self.current_cursor_idx = 0
        self.update_cursor()
        self.check_map_validity()  # 重置后即刻检查

        # --- 新增：只要格子状态发生更改，就将 UI 变回未发送（黄色） ---
        self.current_map_sent_plan = None
        self.update_path_status_display()
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

        # 发送路径按钮也同样修改
        send_path_btn = tk.Button(btn_container, text="发送路径", bg="#2b9348", fg="white",
                                  font=('DejaVu Sans', 30, 'bold'),
                                  command=self.execute_send_path)
        # 修改：fill=tk.BOTH, expand=True
        send_path_btn.pack(fill=tk.BOTH, expand=True, pady=10)

        # 重启雷达指令按钮
        restart_lidar_btn = tk.Button(btn_container, text="重启雷达", bg="#e85d04", fg="white",
                                      font=('DejaVu Sans', 30, 'bold'),
                                      command=self.execute_restart_lidar)
        restart_lidar_btn.pack(fill=tk.BOTH, expand=True, pady=10)

    def start_publish_loop(self):
        """通用发送循环：向当前publisher循环发送当前数值（10Hz）"""
        if self.is_publishing and not rospy.is_shutdown() and self.current_pub:
            self.current_pub.publish(self.current_send_value)
            self.master.after(100, self.start_publish_loop)

    def enqueue_command(self, cmd_type):
        """
        指令入队入口
        - 同类型指令正在执行：忽略
        - 同类型指令已在队列：忽略
        - 队列为空且空闲：立即执行
        """
        # 1. 检查是否正在执行同类型指令
        if self.current_command == cmd_type:
            rospy.loginfo(f"指令 [{cmd_type}] 正在执行中，重复请求已忽略")
            return
        # 2. 检查队列中是否已有同类型指令
        if cmd_type in self.command_queue:
            rospy.loginfo(f"指令 [{cmd_type}] 已在队列中，重复入队已忽略")
            return
        # 3. 加入队列
        self.command_queue.append(cmd_type)
        rospy.loginfo(f"指令 [{cmd_type}] 已入队，当前队列长度：{len(self.command_queue)}")
        # 4. 当前空闲则立即触发执行
        if self.current_command is None:
            self.process_next_command()

    def process_next_command(self):
        """调度队列下一条指令：空闲且队列非空时执行"""
        if self.current_command is not None or not self.command_queue:
            return

        cmd_type = self.command_queue.popleft()
        self.current_command = cmd_type
        rospy.loginfo(f"开始执行指令：{cmd_type}")

        # 按指令类型分发执行
        if cmd_type == 'send_path':
            self._start_path_send()
        elif cmd_type == 'restart_lidar':
            self._start_lidar_restart()
        else:
            rospy.logwarn(f"未知指令类型：{cmd_type}，跳过")
            self.current_command = None
            self.process_next_command()

    def status_callback(self, msg):
        """公共应答回调：匹配当前发送值则完成指令，调度下一条"""
        if not self.is_publishing or self.current_command is None:
            return

        # 应答数值匹配，确认指令完成
        if msg.data == self.current_send_value:
            self.is_publishing = False
            rospy.loginfo(f"指令 [{self.current_command}] 收到应答 {msg.data}，执行完成")

            # 按指令类型切换下一次发送值
            if self.current_command == 'send_path':
                self.send_path_value = 2 if self.send_path_value == 1 else 1
                self.current_map_sent_plan = self.selected_plan_key
                self.update_path_status_display()
                rospy.loginfo(f"路径指令下一次发送值：{self.send_path_value}")

            elif self.current_command == 'restart_lidar':
                self.send_test_value = 4 if self.send_test_value == 3 else 3
                rospy.loginfo(f"重启雷达指令下一次发送值：{self.send_test_value}")

            # 复位当前指令，调度下一条
            self.current_command = None
            self.process_next_command()

    # --- 修改开始：精简和修改 execute_send_path ---
    def execute_send_path(self):
        """发送路径按钮点击：前置检查通过后入队"""
        if not self.check_map_validity():
            messagebox.showwarning("拦截", "当前地图不满足约束条件，禁止发送路径！")
            return
        if not self.selected_plan_key or not self.parsed_plans.get(self.selected_plan_key, {}).get("valid"):
            messagebox.showwarning("拦截", "当前未选择有效的路径方案！")
            return
        self.enqueue_command('send_path')

    def _start_path_send(self):
        """路径指令核心执行逻辑：写入文件+启动发送循环"""
        try:
            current_seq = self.get_current_merlin_seq()
            matched_content = self.parsed_plans[self.selected_plan_key]["raw"]
            # 写入目标文件
            with open(self.target_path, "w", encoding="utf-8") as f_out:
                f_out.write(matched_content + "\n")
            self.last_sent_seq = current_seq

            # 更新UI状态
            self.path_status_var_1.set(f"当前路径：正在发送 ({self.send_path_value})...")
            self.path_status_label_1.config(fg="#f1fa8c")
            self.path_status_var_2.set("")

            # 设置当前发送参数
            self.current_pub = self.send_path_pub
            self.current_send_value = self.send_path_value
            self.is_publishing = True
            self.start_publish_loop()
            rospy.loginfo(f"路径方案 {self.selected_plan_key} 开始发送，值：{self.send_path_value}")
        except Exception as e:
            rospy.logerr(f"路径发送启动失败: {str(e)}")
            messagebox.showerror("失败", f"路径发送错误:\n{str(e)}")
            # 异常则复位，继续下一条
            self.is_publishing = False
            self.current_command = None
            self.process_next_command()

    def execute_restart_lidar(self):
        """重启雷达按钮点击：直接入队"""
        self.enqueue_command('restart_lidar')

    def _start_lidar_restart(self):
        """重启雷达指令核心执行逻辑：与路径指令发送机制一致"""
        try:
            rospy.loginfo(f"重启雷达指令开始发送，值：{self.restart_lidar_value}")

            # 设置当前发送参数
            self.current_pub = self.lidar_restart_pub
            self.current_send_value = self.restart_lidar_value
            self.is_publishing = True
            self.start_publish_loop()
        except Exception as e:
            rospy.logerr(f"重启雷达指令发送失败: {str(e)}")
            self.is_publishing = False
            self.current_command = None
            self.process_next_command()

    # ==========================================
    # 第三块内容：本地脚本极简运行区 (升级版)
    # ==========================================
    def build_script_block_content(self, parent):
        frame = parent

        # 1. 顶部固定控制栏 (永远保持在最上方，不随列表滚动)
        self.control_bar = tk.Frame(frame, bg='#2d2d2d')
        self.control_bar.pack(side=tk.TOP, fill=tk.X, padx=20, pady=5)

        # --- 新增：专门用于包裹按钮以实现整体居中的内部容器 ---
        inner_bar = tk.Frame(self.control_bar, bg='#2d2d2d')
        inner_bar.pack()  # 默认不加 fill=tk.X 时，它会在父容器中自动居中
        # --------------------------------------------------

        # 修改：将父容器从 self.control_bar 改为 inner_bar
        btn_run_term = tk.Button(inner_bar, text="终端运行", bg="#44475a", fg="white",
                                 font=('DejaVu Sans', 30, 'bold'), width=12,
                                 command=self.run_selected_script_in_terminal)
        btn_run_term.pack(side=tk.LEFT, padx=5, pady=5)

        # 修改：将父容器从 self.control_bar 改为 inner_bar
        btn_stop = tk.Button(inner_bar, text="停止", bg="#44475a", fg="white",
                             font=('DejaVu Sans', 30, 'bold'), width=12, command=self.stop_selected_script)
        btn_stop.pack(side=tk.LEFT, padx=5, pady=5)

        # 2. 滚动的脚本列表区域
        list_container = tk.Frame(frame, bg='#2d2d2d')
        list_container.pack(expand=True, fill=tk.BOTH, padx=20, pady=5)

        # --- 修改：将 canvas 改为 self.script_canvas ---
        self.script_canvas = tk.Canvas(list_container, bg='#2d2d2d', highlightthickness=0)
        scrollbar = tk.Scrollbar(list_container, orient="vertical", command=self.script_canvas.yview)

        # 实际放置脚本按钮的容器
        self.script_container = tk.Frame(self.script_canvas, bg='#2d2d2d')
        self.script_container.bind(
            "<Configure>",
            lambda e: self.script_canvas.configure(scrollregion=self.script_canvas.bbox("all"))
        )

        canvas_window = self.script_canvas.create_window((0, 0), window=self.script_container, anchor="nw")
        self.script_canvas.configure(yscrollcommand=scrollbar.set)

        # 强制内部容器的宽度等于 Canvas 的宽度
        self.script_canvas.bind("<Configure>", lambda e: self.script_canvas.itemconfig(canvas_window, width=e.width))

        # 绑定 Canvas 自身的鼠标滚轮事件
        self.script_canvas.bind("<Button-4>", lambda e: self.script_canvas.yview_scroll(-1, "units"))
        self.script_canvas.bind("<Button-5>", lambda e: self.script_canvas.yview_scroll(1, "units"))

        self.script_canvas.pack(side=tk.LEFT, expand=True, fill=tk.BOTH)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        # ---------------------------------------------

        # 3. 底部状态反馈条
        self.script_status_var = tk.StringVar(value="当前无脚本运行")
        status_label = tk.Label(frame, textvariable=self.script_status_var,
                                bg='#1e1e1e', fg='#50fa7b', font=('DejaVu Sans Mono', 12), height=2)
        status_label.pack(fill=tk.X, side=tk.BOTTOM, padx=10, pady=10)

        # 初始化控制栏所需的全局状态变量
        self.selected_script_info = None  # 存储当前选中项的信息
        self.script_buttons = []  # 存储所有生成的按钮对象
        self.running_processes = {}  # 存储正在运行的进程句柄 {path: popen_object}

        self.load_and_create_script_buttons()

    def load_and_create_script_buttons(self):
        """解析 scripts.json 并生成大块单选按钮"""
        scripts = []
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scripts.json")

        if os.path.exists(config_path):
            try:
                with open(config_path, 'r', encoding='utf-8') as f:
                    scripts = json.load(f)
            except Exception as e:
                rospy.logerr(f"读取脚本配置失败: {e}")
        else:
            scripts = [
                {"name": "相机标定脚本", "script": "/path/to/calib.py"},
                {"name": "点云调试接口", "script": "/path/to/debug_pointCloud.py"},
                {"name": "YOLOv5s 预处理", "script": "/path/to/yolo_pre.py"}
            ]

        # 清空已有列表
        self.script_buttons = []
        for widget in self.script_container.winfo_children():
            widget.destroy()

        for s in scripts:
            name = s["name"]
            path = s.get("script", "")

            # 初始化为未选中态：灰底白字
            btn = tk.Button(self.script_container, text=name, bg="#44475a", fg="white",
                            font=('DejaVu Sans', 25), height=2, anchor="w", padx=15)
            btn.pack(fill=tk.X, pady=4, expand=True)

            # --- 新增：解决鼠标悬停在按钮上时无法滚动的问题 ---
            btn.bind("<Button-4>", lambda e: self.script_canvas.yview_scroll(-1, "units"))
            btn.bind("<Button-5>", lambda e: self.script_canvas.yview_scroll(1, "units"))
            # ----------------------------------------------

            # 绑定单选点击事件
            btn.config(command=lambda b=btn, n=name, p=path: self.select_script(b, n, p))
            self.script_buttons.append({"btn": btn, "name": name, "path": path})

            # 如果之前有选中的路径，保持选中显示状态 (白底黑字)
            if self.selected_script_info and self.selected_script_info["path"] == path:
                btn.config(bg="white", fg="black")
                self.selected_script_info["btn"] = btn

    def select_script(self, btn, name, path):
        """处理脚本选中状态切换"""
        # 全部恢复为未选中灰底白字
        for item in self.script_buttons:
            item["btn"].config(bg="#44475a", fg="white")

        # 当前项变白底黑字
        btn.config(bg="white", fg="black")
        self.selected_script_info = {"name": name, "path": path, "btn": btn}

        # 更新状态栏提示
        if path in self.running_processes and self.running_processes[path].poll() is None:
            self.script_status_var.set(f"已选中: {name} (终端运行中...)")
        else:
            self.script_status_var.set(f"已选中: {name}")

    def run_selected_script_in_terminal(self):
        """【控制栏功能】在新终端中运行选中的脚本"""
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
        rospy.loginfo(f"Terminal running script: {path}")

        try:
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'

            # --- 修改：根据脚本后缀自动识别是用 bash 还是 python3 运行 ---
            if path.endswith('.sh'):
                exec_cmd = f"bash '{path}'; exec bash"
            else:
                exec_cmd = f"python3 '{path}'; exec bash"
            # ---------------------------------------------------------

            proc = subprocess.Popen(
                ['gnome-terminal', '--disable-factory', '--', 'bash', '-c', exec_cmd],
                env=env,
                preexec_fn=os.setsid  # 创建独立进程组，停止按钮依然有效
            )

            self.running_processes[path] = proc
            self.script_status_var.set(f"[已运行] {name} 已在独立终端启动")
        except Exception as e:
            self.script_status_var.set(f"[错误] {name} 启动失败")
            rospy.logerr(str(e))

    def stop_selected_script(self):
        """【控制栏功能】结束当前选中脚本所开启的终端程序"""
        if not self.selected_script_info:
            self.script_status_var.set("[提示] 请先选择要停止的脚本")
            return

        name = self.selected_script_info["name"]
        path = self.selected_script_info["path"]

        if path in self.running_processes and self.running_processes[path].poll() is None:
            proc = self.running_processes[path]
            try:
                # 杀死整个进程组（包括 gnome-terminal 窗口以及在里面运行的 python 子进程）
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                self.script_status_var.set(f"[已停止] {name} 进程已强行终止")
            except Exception as e:
                self.script_status_var.set(f"[错误] 停止 {name} 失败")
                rospy.logerr(f"Kill terminal process group failed: {e}")
        else:
            self.script_status_var.set(f"[提示] {name} 当前没有在运行")

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
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
import time

class TouchscreenUI:
    def __init__(self, master):
        self.master = master
        self.master.title("RC 触屏控制面板")
        # 触屏设备建议全屏或固定大尺寸
        # --- 修改：动态计算屏幕 3/4 大小并居中显示 ---
        screen_width = self.master.winfo_screenwidth()
        screen_height = self.master.winfo_screenheight()
        # 计算 3/4 的宽度和高度
        win_width = int(screen_width * 0.9)
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

        # ========== 新增：ROS 全局状态显示变量 ==========
        self.ros_status_vars = {
            "path": tk.StringVar(value="路径指令：未发送"),
            "param": tk.StringVar(value="参数指令：未发送"),
            "lidar": tk.StringVar(value="雷达重启：未执行"),
            "common": tk.StringVar(value="系统状态：空闲")
        }
        # 存储所有页面的标签实例，用于多页面同步更新颜色
        self.ros_status_labels = {
            "path": [],
            "param": [],
            "lidar": [],
            "common": []
        }
        # ==============================================

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
        self.bg_red = "#FFADA3"  # 偏暗红色调
        self.bg_blue = "#BBEFF2"  # 偏暗蓝色调
        # ===================== 新增参数配置模块变量 =====================
        # 参数json与输出txt路径，自行修改
        self.params_config_path = r"./params_config.json"
        self.params_target_txt = r"../path/parameter.txt"
        self.params_meta = {}  # 存储参数元信息 {key:{"name":"中文","range":(min,max)}}
        self.params_dict = {}  # 当前选中参数值，默认0
        self.param_blocks = {}  # UI按钮缓存
        self.CMD_TYPE_PARAM = "send_param"
        # 参数ROS发布器，5/6交替
        self.send_param_pub = rospy.Publisher('/scripts/send_param', Int32, queue_size=1)
        self.send_param_value = 5
        # 加载参数配置
        self._load_params_json()

        # ===================== 新增：设备自检区模块变量 =====================
        self.check_tasks_meta = {
            "side_cam": {"name": "两侧相机自检", "topic": "/scripts/check_camera_side"},
            "front_cam": {"name": "前向相机自检", "topic": "/scripts/check_camera_front"},
            "lidar": {"name": "雷达自检", "topic": "/scripts/check_lidar"}
        }
        # 状态机: 0(空闲), 1(开指令发送中), 2(自检中), 3(关指令发送中), 4(自检已完成)
        self.check_states = {k: 0 for k in self.check_tasks_meta.keys()}
        self.check_ui_elements = {}
        self.check_pubs = {
            k: rospy.Publisher(v["topic"], Int32, queue_size=1)
            for k, v in self.check_tasks_meta.items()
        }
        # ==================================================================

        # === 新增：自动识别并关联参数区的红蓝半场，统一默认值为红半区 ===
        self.court_param_key = None
        for k, v in self.params_meta.items():
            if any(kw in v["name"] for kw in ["半场", "红蓝", "分场", "场地"]):
                self.court_param_key = k
                break

        self.is_red_court = True  # 梅林区默认红半区
        if self.court_param_key:
            for val, text in self.params_meta[self.court_param_key]["map"].items():
                if "红" in text:
                    self.params_dict[self.court_param_key] = val  # 参数区同步默认红半区
                    break

        # ==================================================================
        # 构建功能块（将左侧区合并为一个带切换功能的复合面板）
        self.build_left_panel(self.main_frame, col=0)
        # 【注意】请将这里的路径替换为你实际的 9 个数据库文件的具体路径
        self.databases = {
            (1, 2): r"./official_path/path_12.txt",
            (1, 3): r"./official_path/path_13.txt",
            (1, 4): r"./official_path/path_14.txt",
            (2, 2): r"./official_path/path_22.txt",
            (2, 3): r"./official_path/path_23.txt",
            (2, 4): r"./official_path/path_24.txt",
            (3, 2): r"./official_path/path_32.txt",
            (3, 3): r"./official_path/path_33.txt",
            (3, 4): r"./official_path/path_34.txt",
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
        self.send_path_pub = rospy.Publisher('/scripts/send_path', Int32, queue_size=1)
        # ========== 重启雷达指令 ==========
        self.restart_lidar_value = 3  # 用3/4交替，与路径的1/2区分，避免应答冲突
        self.lidar_restart_pub = rospy.Publisher('/scripts/restart_lidar', Int32, queue_size=1)
        # --- 新增：雷达2相关变量与双重重启模式标记 ---
        self.restart_lidar2_value = 3
        self.lidar2_restart_pub = rospy.Publisher('/scripts/restart_lidar2', Int32, queue_size=1)
        self.lidar_restart_mode = 3  # 默认模式：1 表示重启单个，2 表示重启两个
        self.lidar_dual_stage = 0  # 双雷达任务阶段记录 (0:空闲, 1:等待雷达1应答, 2:等待雷达2应答)
        # ---------------------------------------------

        # 新增：重启雷达防误触 - 时间戳队列
        self.lidar_click_timestamps = deque()
        # 新增：雷达状态自动复位定时器句柄
        self.lidar_reset_timer = None
        # ========== 公共应答订阅（原有不变） ==========
        self.status_sub = rospy.Subscriber('/scripts/response', Int32, self.status_callback)

        self.build_ros_command_block(self.main_frame, col=1)

    # ===================== 新增：ROS 状态栏通用方法 =====================
    def build_ros_status_bar(self, parent):
        """构建可复用的ROS话题发送状态栏，自动绑定全局状态变量"""
        status_frame = tk.Frame(parent, bg='#2d2d2d')
        status_frame.pack(fill=tk.X, pady=3)

        # 内部容器居中排列
        inner_frame = tk.Frame(status_frame, bg='#2d2d2d')
        inner_frame.pack()

        # 按顺序创建四类状态标签
        for key, var in self.ros_status_vars.items():
            lbl = tk.Label(inner_frame, textvariable=var, bg="#2d2d2d",
                           fg="#f1fa8c", font=('DejaVu Sans', 25, 'bold'))
            lbl.pack(side=tk.LEFT, padx=12)
            self.ros_status_labels[key].append(lbl)

        return status_frame

    def update_ros_status(self, status_type, text, color):
        """
        统一更新指定类型的ROS状态显示（多页面同步）
        :param status_type: path / param / lidar / common
        :param text: 显示文本
        :param color: 字体颜色：成功#50fa7b / 警告#f1fa8c / 错误#ff5555
        """
        if status_type not in self.ros_status_vars:
            return
        self.ros_status_vars[status_type].set(text)
        # 同步更新所有页面的对应标签颜色
        for lbl in self.ros_status_labels[status_type]:
            lbl.config(fg=color)
    # ==================================================================

    def get_current_merlin_seq(self):
        """统一获取序列：底层真实存储状态生成"""
        mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
        # 还原为直接映射，不要在这里做对调，交由 UI 坐标系统一处理
        return "".join([mapping[self.grid_states[i]] for i in range(12)])

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
            # 解决鼠标悬停在按钮上时无法滚动的问题
            btn.bind("<Button-4>", lambda e: self.script_canvas.yview_scroll(-1, "units"))
            btn.bind("<Button-5>", lambda e: self.script_canvas.yview_scroll(1, "units"))
            # 绑定单选点击事件
            btn.config(command=lambda b=btn, n=name, p=path: self.select_script(b, n, p))
            self.script_buttons.append({"btn": btn, "name": name, "path": path})
            # 如果之前有选中的路径，保持选中显示状态
            if self.selected_script_info and self.selected_script_info["path"] == path:
                btn.config(bg="white", fg="black")
                self.selected_script_info["btn"] = btn

    def _load_params_json(self):
        """读取参数配置json，初始化参数默认值：配置default则用default，否则默认0"""
        if not os.path.exists(self.params_config_path):
            rospy.logerr(f"参数配置文件不存在：{self.params_config_path}")
            return
        try:
            with open(self.params_config_path, "r", encoding="utf-8") as f:
                raw_cfg = json.load(f)
            for k, v in raw_cfg.items():
                name = v.get("name")
                rng = v.get("range")
                map_dict = v.get("map", {})
                # 转换map的key为整数，保证和参数值类型一致
                map_dict = {int(key): val for key, val in map_dict.items()}

                if not name or not isinstance(rng, (list, tuple)) or len(rng) != 2:
                    rospy.logwarn(f"参数{k}配置非法，跳过")
                    continue
                minv, maxv = int(rng[0]), int(rng[1])

                # 新增：读取默认值，未配置则默认 0
                default_val = int(v.get("default", minv))
                # 范围校验：默认值越界则警告并回退为 0
                if not (minv <= default_val <= maxv):
                    rospy.logwarn(f"参数 {k} 的默认值 {default_val} 超出范围 [{minv}, {maxv}]，已回退为 0")
                    default_val = minv

                self.params_meta[k] = {
                    "name": name,
                    "range": (minv, maxv),
                    "map": map_dict
                }
                self.params_dict[k] = default_val
        except Exception as e:
            rospy.logerr(f"加载参数json失败：{str(e)}")

    # --- 2. 添加为 TouchscreenUI 类的类方法 ---
    def _load_monitor_json(self):
        """读取监视区配置 JSON"""
        if not os.path.exists(self.monitor_config_path):
            rospy.logwarn(f"监视区配置文件不存在：{self.monitor_config_path}")
            return
        try:
            with open(self.monitor_config_path, "r", encoding="utf-8") as f:
                self.monitor_meta = json.load(f)
        except Exception as e:
            rospy.logerr(f"加载监视区json失败：{str(e)}")

    def toggle_court_side(self):
        """处理红蓝半场切换按钮逻辑"""
        self.is_red_court = not self.is_red_court
        self.update_court_ui()

        # === 新增：联动更新参数选择区 ===
        if getattr(self, 'court_param_key', None):
            for val, text in self.params_meta[self.court_param_key]["map"].items():
                if (self.is_red_court and "红" in text) or (not self.is_red_court and "蓝" in text):
                    self.params_dict[self.court_param_key] = val
                    # 刷新参数区按钮的高亮样式
                    for v, btn in self.param_blocks[self.court_param_key].items():
                        if v == val:
                            btn.config(bg="#f1fa8c", fg="black")
                        else:
                            btn.config(bg="#44475a", fg="white")
                    break
        # ================================

        # 新增：已输入的方块不镜像
        for i in range(0,12,3):
            # print(i)
            temp = self.grid_states[i]
            self.grid_states[i] = self.grid_states[i+2]
            self.grid_states[i+2] = temp

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
        self.btn_grid_view = tk.Button(title_bar, text="梅林输入区", font=('DejaVu Sans', 50, 'bold'),
                                       command=lambda: self.switch_left_view('grid'))
        self.btn_grid_view.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0, 5))
        # 新增参数配置Tab按钮
        self.btn_param_view = tk.Button(title_bar, text="参数配置区", font=('DejaVu Sans', 50, 'bold'),
                                        command=lambda: self.switch_left_view('param'))
        self.btn_param_view.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5, 0))
        self.btn_check_view = tk.Button(title_bar, text="设备自检区", font=('DejaVu Sans', 50, 'bold'),
                                        command=lambda: self.switch_left_view('check'))
        self.btn_check_view.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5, 0))

        # 内容容器
        self.left_content_container = tk.Frame(left_panel, bg='#2d2d2d')
        self.left_content_container.pack(expand=True, fill=tk.BOTH)
        # 预先构建两个子页面
        self.grid_frame = tk.Frame(self.left_content_container, bg='#2d2d2d')
        self.build_grid_block_content(self.grid_frame)
        # 新增参数配置页面容器
        self.param_frame = tk.Frame(self.left_content_container, bg='#2d2d2d')
        self.build_param_block_content(self.param_frame)
        # 设备自检区
        self.check_frame = tk.Frame(self.left_content_container, bg='#2d2d2d')
        self.build_check_block_content(self.check_frame)

        # 默认显示梅林输入区
        self.switch_left_view('grid')

    def switch_left_view(self, view_name):
        """切换左侧页面显示并更新按钮样式"""
        def reset_btn_style(btn):
            btn.config(bg="#44475a", fg="white")

        def select_btn_style(btn):
            btn.config(bg="white", fg="black")

        # 全部重置样式
        reset_btn_style(self.btn_grid_view)
        reset_btn_style(self.btn_param_view)
        reset_btn_style(self.btn_check_view)
        # 全部隐藏
        self.grid_frame.pack_forget()
        self.param_frame.pack_forget()
        self.check_frame.pack_forget()
        # 切换视图
        if view_name == 'grid':
            select_btn_style(self.btn_grid_view)
            self.grid_frame.pack(expand=True, fill=tk.BOTH)
        elif view_name == 'script':
            select_btn_style(self.btn_script_view)
            self.script_frame.pack(expand=True, fill=tk.BOTH)
        elif view_name == 'param':
            select_btn_style(self.btn_param_view)
            self.param_frame.pack(expand=True, fill=tk.BOTH)
        elif view_name == 'check':
            select_btn_style(self.btn_check_view)
            self.check_frame.pack(expand=True, fill=tk.BOTH)

    def build_param_block_content(self, parent):
        """参数配置页面：顶部常驻ROS状态栏，下方流式布局参数块，自动换行+垂直滚动"""
        # 顶部常驻ROS状态栏
        self.build_ros_status_bar(parent)

        # 滚动区域外层容器
        wrap_frame = tk.Frame(parent, bg="#2d2d2d")
        wrap_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 垂直滚动条
        scrollbar = tk.Scrollbar(wrap_frame, orient=tk.VERTICAL, width=70)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # 滚动画布核心容器
        self.param_canvas = tk.Canvas(wrap_frame, bg="#2d2d2d", highlightthickness=0,
                                      yscrollcommand=scrollbar.set)
        self.param_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.param_canvas.yview)

        # 内部流式容器：存放所有参数块
        self.param_flow_frame = tk.Frame(self.param_canvas, bg="#2d2d2d")
        self.param_flow_window = self.param_canvas.create_window(
            (0, 0), window=self.param_flow_frame, anchor="nw"
        )

        # 存储所有参数块实例，用于重排
        self.param_block_widgets = []

        # ========== 核心修改：移除基于屏幕宽度的像素乘法，改为固定合理尺寸 ==========
        # 响应式布局应依靠 _on_param_canvas_resize 自动改变列数，而不是无脑放大单块
        self.param_block_width = 700  # 统一设定为合理的固定宽度
        self.param_block_height = 250  # 统一设定为合理的固定高度
        self.param_block_gap = 20  # 块之间横向和纵向间距
        # ====================================================================

        # 生成所有参数块
        param_keys = list(self.params_meta.keys())
        for key in param_keys:
            block = self._create_single_param_widget(self.param_flow_frame, key)
            self.param_block_widgets.append(block)

        # 绑定画布尺寸变化：宽度变化时重排参数块
        self.param_canvas.bind("<Configure>", self._on_param_canvas_resize)
        # 内部容器高度变化时，自动更新滚动范围
        self.param_flow_frame.bind(
            "<Configure>",
            lambda e: self.param_canvas.config(scrollregion=self.param_canvas.bbox("all"))
        )

        # 绑定鼠标滚轮
        self.param_canvas.bind("<Button-4>", lambda e: self.param_canvas.yview_scroll(-1, "units"))
        self.param_canvas.bind("<Button-5>", lambda e: self.param_canvas.yview_scroll(1, "units"))

        # 初始化完成后手动触发一次重排
        self.master.update_idletasks()
        self._on_param_canvas_resize(
            type("Event", (), {"width": self.param_canvas.winfo_width()})()
        )

    def _create_single_param_widget(self, parent_container, param_key):
        """单个参数UI块，使用标准系统字号以适配不同设备"""
        meta = self.params_meta[param_key]
        name = meta["name"]
        minv, maxv = meta["range"]
        map_dict = meta["map"]

        # ========== 核心修改：使用固定的标准字号 ==========
        # 让 Tkinter 依据系统 DPI 自行处理字体渲染，避免在不同设备上忽大忽小
        name_font_size = 25
        btn_font_size = 45
        # ===============================================

        block = tk.Frame(
            parent_container, bg="#383838", bd=2, relief=tk.RAISED,
            width=self.param_block_width, height=self.param_block_height
        )
        block.pack_propagate(False)  # 强制宽高生效

        # 参数名称
        tk.Label(
            block, text=name, bg="#383838", fg="white",
            font=("DejaVu Sans", name_font_size, "bold")
        ).pack(anchor="w", padx=10, pady=6)

        # 参数按钮行
        btn_wrap = tk.Frame(block, bg="#383838")
        btn_wrap.pack(anchor="w", padx=10, pady=8)

        self.param_blocks[param_key] = {}
        for val in range(minv, maxv + 1):
            txt = map_dict.get(val, str(val))
            btn = tk.Button(
                btn_wrap, text=txt, font=("DejaVu Sans", btn_font_size, "bold"),
                command=lambda v=val, k=param_key: self._on_param_select(k, v)
            )
            btn.pack(side=tk.LEFT, padx=6, pady=4)
            self.param_blocks[param_key][val] = btn
            if val == self.params_dict[param_key]:
                btn.config(bg="#f1fa8c", fg="black")
            else:
                btn.config(bg="#44475a", fg="white")

        # 滚轮穿透
        def _scroll(e):
            self.param_canvas.yview_scroll(-1 if e.num == 4 else 1, "units")

        block.bind("<Button-4>", _scroll)
        block.bind("<Button-5>", _scroll)
        btn_wrap.bind("<Button-4>", _scroll)
        btn_wrap.bind("<Button-5>", _scroll)

        return block

    def _on_param_select(self, param_key, val):
        """点击参数值切换选中状态"""
        self.params_dict[param_key] = val
        for v, btn in self.param_blocks[param_key].items():
            if v == val:
                btn.config(bg="#f1fa8c", fg="black")
            else:
                btn.config(bg="#44475a", fg="white")

        # === 新增：联动更新梅林输入区红蓝半场 ===
        if getattr(self, 'court_param_key', None) and param_key == self.court_param_key:
            text = self.params_meta[param_key]["map"].get(val, "")
            if "红" in text:
                self.is_red_court = True
            elif "蓝" in text:
                self.is_red_court = False

            # 触发梅林UI刷新与地图重绘
            self.update_court_ui()
            self.check_map_validity()
            self.redraw_canvas()
        # ======================================

    def _on_param_canvas_resize(self, event):
        """画布宽度变化时，强制分为3列，并自动三等分宽度"""
        canvas_width = event.width
        if canvas_width <= 1:
            return  # 宽度异常时跳过，避免初始渲染报错

        cols = 2

        # 配置内部容器的 3 列具有相同的权重和统一的分组，实现底层网格的绝对三等分
        for i in range(cols):
            self.param_flow_frame.columnconfigure(i, weight=1, uniform="param_cols")

        # 计算三等分后每个块应有的实际宽度（单列总宽减去安全边距，防止组件互相粘连）
        new_block_width = max(150, (canvas_width // cols) - self.param_block_gap)

        # 清空原有布局
        for widget in self.param_block_widgets:
            widget.grid_forget()

        # 动态重设宽度并执行流式重排
        for idx, widget in enumerate(self.param_block_widgets):
            # 将方块宽度重置为三等分计算出的宽度
            widget.config(width=new_block_width)

            row = idx // cols
            col = idx % cols
            # sticky="n" 配合列均分，会自动将方块在其所属的三等分区域内水平居中
            widget.grid(
                row=row,
                column=col,
                pady=10,
                sticky="n"
            )

        # 同步内部窗口宽度，撑满画布
        self.param_canvas.itemconfig(self.param_flow_window, width=canvas_width)

    # ==========================================
    # 第一块内容：12宫格状态填涂区 (原 build_grid_block)
    # ==========================================
    def build_grid_block_content(self, parent):
        frame = parent  # 直接绑定到传入的子页面容器，不再重新创建带有 Sunken 的外框

        # --- 修改：ROS状态栏移至最顶部（原路径状态区改造而来）---
        self.build_ros_status_bar(frame)

        # --- 约束状态常驻显示区（下移至状态栏下方） ---
        constraint_frame = tk.Frame(frame, bg='#2d2d2d')
        constraint_frame.pack(fill=tk.X, pady=5)
        self.pos_constraint_var = tk.StringVar(value="位置约束：不满足")
        self.qty_constraint_var = tk.StringVar(value="数量约束：不满足")
        self.pos_constraint_label = tk.Label(constraint_frame, textvariable=self.pos_constraint_var, fg="#ff5555",
                                             bg="#2d2d2d", font=('DejaVu Sans', 20, 'bold'))
        self.pos_constraint_label.pack(side=tk.LEFT, expand=True)
        self.qty_constraint_label = tk.Label(constraint_frame, textvariable=self.qty_constraint_var, fg="#ff5555",
                                             bg="#2d2d2d", font=('DejaVu Sans', 20, 'bold'))
        self.qty_constraint_label.pack(side=tk.LEFT, expand=True)

        # 保留原有状态变量
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
                            font=('DejaVu Sans', 25, 'bold'), width=6,
                            command=lambda i=idx: self.apply_state_to_grid(i))
            btn.pack(side=tk.TOP, expand=True, fill=tk.BOTH, pady=5)
        reset_btn = tk.Button(btn_frame, text="全部\n重置", bg="#44475a", fg="white",
                              font=('DejaVu Sans', 25, 'bold'), width=6,
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
        tk.Label(table_frame, text="R1\\R2", bg="#2d2d2d", fg="black", font=('DejaVu Sans', 25, 'bold')).grid(row=0,
                                                                                                              column=0,
                                                                                                              sticky='nsew',
                                                                                                              padx=1,
                                                                                                              pady=1)
        for col_idx, r2_cnt in enumerate([2, 3, 4], start=1):
            tk.Label(table_frame, text=f"{r2_cnt}个", bg="#2d2d2d", fg="black", font=('DejaVu Sans', 25, 'bold')).grid(
                row=0, column=col_idx, sticky='nsew', padx=1, pady=1)
        for row_idx, r1_cnt in enumerate([1, 2, 3], start=1):
            tk.Label(table_frame, text=f"{r1_cnt}个", bg="#2d2d2d", fg="black", font=('DejaVu Sans', 25, 'bold')).grid(
                row=row_idx, column=0, sticky='nsew', padx=1, pady=1)
            for col_idx, r2_cnt in enumerate([2, 3, 4], start=1):
                # 修改：由 tk.Button 改为 tk.Label，去掉了 command，改用 bind 绑定点击事件
                lbl = tk.Label(table_frame, text="-", bg="#44475a", fg="white", width=5, font=('DejaVu Sans', 40, 'bold'))
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
        self.court_toggle_btn = tk.Button(right_frame, text="切换为蓝场", font=('DejaVu Sans', 30, 'bold'),
                                          command=self.toggle_court_side)
        self.court_toggle_btn.grid(row=0, column=0, sticky='ne', padx=5, pady=5)
        # 修改：将 info_container 放入第 1 行
        info_container = tk.Frame(right_frame, bg='#2d2d2d')
        info_container.grid(row=1, column=0, sticky='w')
        self.r2_path_var = tk.StringVar(value="R2 路径：暂无")
        self.r1_path_var = tk.StringVar(value="R1 夹取：暂无")
        # anchor=tk.W 确保文字在 Label 宽度内靠左，pady 增加行间距防止拥挤
        tk.Label(info_container, textvariable=self.r2_path_var, fg="#000000", bg="#2d2d2d",
                 font=('DejaVu Sans', 30, 'bold'), anchor=tk.W, width=35, justify=tk.LEFT, wraplength=700).pack(fill=tk.X,
                                                                                                       pady=5)
        tk.Label(info_container, textvariable=self.r1_path_var, fg="#0021FA", bg="#2d2d2d",
                 font=('DejaVu Sans', 30, 'bold'), anchor=tk.W, width=35, justify=tk.LEFT, wraplength=1200).pack(fill=tk.X,
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
        """根据当前地图发送状态和表格选中状态，动态更新路径指令状态栏"""
        # 修改：路径指令执行中 或 排队中，都不刷新状态，避免排队提示被重绘逻辑覆盖
        if (self.is_publishing and self.current_command == 'send_path') or ('send_path' in self.command_queue):
            return
        if not self.current_map_sent_plan:
            self.update_ros_status("path", "路径指令：未发送", "#f1fa8c")
        else:
            sent_r1, sent_r2 = self.current_map_sent_plan
            base_text = f"路径指令：已发送R1={sent_r1}R2={sent_r2}"
            if self.selected_plan_key == self.current_map_sent_plan:
                self.update_ros_status("path", base_text, "#50fa7b")
            else:
                self.update_ros_status("path", f"{base_text} | 选中方案未发送", "#f1fa8c")

    # --- 修改开始：替换并新增解析与选择逻辑 ---
    def update_parsed_path_display(self):
        """需求：立刻匹配9个数据库并解析各方案信息，更新表格展示"""
        import re
        self.parsed_plans.clear()
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
                        "r2_str": " → ".join(r2_part) if r2_part else "原地待命",
                        "r1_str": " → ".join(r1_blocks) if r1_blocks else "原地待命",
                        "raw": matched_content
                    })
                    if first_valid_key is None:
                        first_valid_key = (r1, r2)
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
            num_font_size = max(35, int(self.cell_h * 0.06 + self.cell_h * 0.06))
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
        font_size = max(50, int(self.cell_h * 0.15))
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
        send_path_btn.pack(fill=tk.BOTH, expand=True, pady=10)
        # 重启雷达指令按钮
        # --- 修改：定制化的重启雷达复合按钮区域（下方三按钮横向平分） ---
        lidar_frame = tk.Frame(btn_container, bg='#2d2d2d')
        lidar_frame.pack(fill=tk.BOTH, expand=True, pady=10)

        # 上方：重启执行大按钮
        restart_lidar_btn = tk.Button(lidar_frame, text="重启雷达", bg="#e85d04", fg="white",
                                      font=('DejaVu Sans', 30, 'bold'),
                                      command=self.execute_restart_lidar)
        restart_lidar_btn.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(0, 5))

        # 下方：模式选择区 (横向排列)
        mode_frame = tk.Frame(lidar_frame, bg='#2d2d2d')
        mode_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(5, 0))

        # 子按钮：1
        self.btn_lidar_mode1 = tk.Button(mode_frame, text="1", font=('DejaVu Sans', 18, 'bold'),
                                         command=lambda: self.set_lidar_mode(1))
        self.btn_lidar_mode1.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        # 子按钮：2
        self.btn_lidar_mode2 = tk.Button(mode_frame, text="2", font=('DejaVu Sans', 18, 'bold'),
                                         command=lambda: self.set_lidar_mode(2))
        self.btn_lidar_mode2.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        # 子按钮：both
        self.btn_lidar_both = tk.Button(mode_frame, text="all", font=('DejaVu Sans', 30, 'bold'),
                                        command=lambda: self.set_lidar_mode(3))
        self.btn_lidar_both.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 默认高亮 both 按钮
        self.set_lidar_mode(3)
        # ------------------------------------------------------------------

        # 新增发送参数按钮
        send_param_btn = tk.Button(btn_container, text="发送参数", bg="#0088ff", fg="white",
                                   font=('DejaVu Sans', 30, 'bold'),
                                   command=self.execute_send_param)
        send_param_btn.pack(fill=tk.BOTH, expand=True, pady=10)

    def set_lidar_mode(self, mode):
        """处理雷达重启模式的切换与高亮（发送或排队中不可更改）"""
        if self.current_command == 'restart_lidar' or 'restart_lidar' in self.command_queue:
            rospy.logwarn("指令执行/排队中，禁止切换雷达重启模式")
            return

        self.lidar_restart_mode = mode

        # 统一重置所有颜色
        self.btn_lidar_mode1.config(bg="#44475a", fg="white")
        self.btn_lidar_mode2.config(bg="#44475a", fg="white")
        self.btn_lidar_both.config(bg="#44475a", fg="white")

        # 根据选中的 mode 设置高亮
        if mode == 1:
            self.btn_lidar_mode1.config(bg="#f1fa8c", fg="black")
        elif mode == 2:
            self.btn_lidar_mode2.config(bg="#f1fa8c", fg="black")
        elif mode == 3:
            self.btn_lidar_both.config(bg="#f1fa8c", fg="black")

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

        # ========== 新增：非空闲时更新状态栏为排队中 ==========
        if self.current_command is not None:
            status_map = {
                'send_path': ('path', '路径指令：排队等待中'),
                'restart_lidar': ('lidar', '雷达重启：排队等待中'),
                self.CMD_TYPE_PARAM: ('param', '参数指令：排队等待中')
            }
            if cmd_type in status_map:
                key, text = status_map[cmd_type]
                self.update_ros_status(key, text, "#f1fa8c")
        # ======================================================

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
        # 更新系统状态
        self.update_ros_status("common", "系统状态：执行中", "#f1fa8c")
        # 按指令类型分发执行
        if cmd_type == 'send_path':
            self._start_path_send()
        elif cmd_type == 'restart_lidar':
            self._start_lidar_restart()
        elif cmd_type == self.CMD_TYPE_PARAM:
            self._start_param_send()
            # === 修改：在 process_next_command 消费 check_ 指令时，更新对应的具体发送状态 ===
        elif cmd_type.startswith("check_"):
            parts = cmd_type.split("_")
            action = parts[-1]
            task_key = "_".join(parts[1:-1])

            self.current_pub = self.check_pubs[task_key]
            self.current_send_value = 1 if action == 'on' else 2

            if action == 'on':
                self.check_states[task_key] = 1
                self.update_check_ui(task_key, "#BFB936", "发送自检\n指令中")  # 加入换行
            else:
                self.check_states[task_key] = 3
                self.update_check_ui(task_key, "#BFB936", "关闭自检\n发送中")  # 加入换行

            self.is_publishing = True
            self.start_publish_loop()
        # ========================================================================
        else:
            rospy.logwarn(f"未知指令类型：{cmd_type}，跳过")
            self.current_command = None
            self.update_ros_status("common", "系统状态：空闲", "#50fa7b")
            self.process_next_command()

    def status_callback(self, msg):
        """公共应答回调：匹配当前发送值则完成指令，调度下一条"""
        if not self.is_publishing or self.current_command is None:
            return
        # 应答数值匹配，确认指令完成
        if msg.data == self.current_send_value:
            self.is_publishing = False
            rospy.loginfo(f"指令 [{self.current_command}] 收到应答 {msg.data}，执行完成")
            # 按指令类型切换下一次发送值，并更新状态栏
            if self.current_command == 'send_path':
                self.send_path_value = 2 if self.send_path_value == 1 else 1
                self.current_map_sent_plan = self.selected_plan_key
                self.update_path_status_display()
                rospy.loginfo(f"路径指令下一次发送值：{self.send_path_value}")
            elif self.current_command == 'restart_lidar':
                if self.lidar_restart_mode == 3 and self.lidar_dual_stage == 1:
                    # 【both模式阶段1完成】：雷达1收到应答，自动开始发送雷达2
                    self.restart_lidar_value = 4 if self.restart_lidar_value == 3 else 3
                    self.lidar_dual_stage = 2

                    rospy.loginfo("both模式：雷达1应答成功，转入雷达2信号发送流程")
                    self.update_ros_status("lidar", "雷达重启：雷达1成功，正发送雷达2...", "#f1fa8c")

                    self.current_pub = self.lidar2_restart_pub
                    self.current_send_value = self.restart_lidar2_value
                    self.is_publishing = True
                    self.start_publish_loop()
                    return  # 提前 return 拦截，不结束本次队列任务

                # 以下为各模式完全结束时的信号翻转逻辑
                if self.lidar_restart_mode == 1:
                    self.restart_lidar_value = 4 if self.restart_lidar_value == 3 else 3
                elif self.lidar_restart_mode == 2:
                    self.restart_lidar2_value = 4 if self.restart_lidar2_value == 3 else 3
                elif self.lidar_restart_mode == 3:
                    self.restart_lidar2_value = 4 if self.restart_lidar2_value == 3 else 3

                self.lidar_dual_stage = 0
                rospy.loginfo(
                    f"重启雷达任务完成。下一次发送值：(雷达1->{self.restart_lidar_value}, 雷达2->{self.restart_lidar2_value})")
                self.update_ros_status("lidar", "雷达重启：全部执行成功", "#50fa7b")
            elif self.current_command == 'send_param':
                self.send_param_value = 6 if self.send_param_value == 5 else 5
                rospy.loginfo(f"参数指令下一次发送值：{self.send_param_value}")
                self.update_ros_status("param", "参数指令：发送成功", "#50fa7b")
            elif self.current_command.startswith("check_"):
                parts = self.current_command.split("_")
                action = parts[-1]
                task_key = "_".join(parts[1:-1])

                if action == 'on':
                    self.check_states[task_key] = 2
                    self.update_check_ui(task_key, "#006AFF", "自检中")  # 蓝色
                else:
                    self.check_states[task_key] = 4
                    self.update_check_ui(task_key, "#299629", "自检已完成")  # 绿色
            # 复位当前指令，调度下一条
            self.current_command = None
            self.update_ros_status("common", "系统状态：空闲", "#50fa7b")
            self.process_next_command()

    # --- 修改：路径发送逻辑关联状态栏 ---
    def execute_send_path(self):
        """发送路径按钮点击：前置检查通过后入队"""
        if not self.check_map_validity():
            # self.update_ros_status("path", "路径指令：地图不满足约束，禁止发送", "#f1fa8c")
            messagebox.showwarning("拦截", "当前地图不满足约束条件，禁止发送路径！")
            return
        if not self.selected_plan_key or not self.parsed_plans.get(self.selected_plan_key, {}).get("valid"):
            # self.update_ros_status("path", "路径指令：未选择有效方案，禁止发送", "#f1fa8c")
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
            # 更新状态栏
            self.update_ros_status("path", f"路径指令：正在发送", "#f1fa8c")
            # 设置当前发送参数
            self.current_pub = self.send_path_pub
            self.current_send_value = self.send_path_value
            self.is_publishing = True
            self.start_publish_loop()
            rospy.loginfo(f"路径方案 {self.selected_plan_key} 开始发送，值：{self.send_path_value}")
        except Exception as e:
            rospy.logerr(f"路径发送启动失败: {str(e)}")
            self.update_ros_status("path", f"路径指令：发送失败 {str(e)}", "#ff5555")
            messagebox.showerror("失败", f"路径发送错误:\n{str(e)}")
            # 异常则复位，继续下一条
            self.is_publishing = False
            self.current_command = None
            self.update_ros_status("common", "系统状态：空闲", "#50fa7b")
            self.process_next_command()

    def execute_restart_lidar(self):
        """重启雷达：2秒内3连击防误触；超时未达标自动恢复未执行状态；发送/排队中静默拦截"""
        # 发送中或排队中，直接静默拦截，不响应、不改状态
        if self.current_command == 'restart_lidar' or 'restart_lidar' in self.command_queue:
            return

        now = time.time()
        # 清理2秒窗口外的过期记录
        while self.lidar_click_timestamps and now - self.lidar_click_timestamps[0] > 2:
            self.lidar_click_timestamps.popleft()

        self.lidar_click_timestamps.append(now)
        click_count = len(self.lidar_click_timestamps)

        if click_count >= 3:
            # 达标触发：取消超时定时器，执行指令
            if self.lidar_reset_timer:
                self.master.after_cancel(self.lidar_reset_timer)
                self.lidar_reset_timer = None
            self.lidar_click_timestamps.clear()
            self.enqueue_command('restart_lidar')
        else:
            # 未达标：刷新超时定时器，2秒后自动恢复未执行状态
            if self.lidar_reset_timer:
                self.master.after_cancel(self.lidar_reset_timer)
            self.lidar_reset_timer = self.master.after(2000, self._reset_lidar_click_status)
            # 实时提示剩余点击次数
            remain = 3 - click_count
            self.update_ros_status("lidar", f"雷达重启：再点{remain}次确认", "#f1fa8c")

    def _reset_lidar_click_status(self):
        """2秒超时自动恢复雷达状态栏并清空点击记录"""
        self.lidar_click_timestamps.clear()
        self.lidar_reset_timer = None
        self.update_ros_status("lidar", "雷达重启：未执行", "#f1fa8c")

    def _start_lidar_restart(self):
        """重启雷达指令核心执行逻辑：区分 1, 2, both 模式的启动参数"""
        try:
            # 只有 both 模式 (3) 需要走多阶段状态机控制
            self.lidar_dual_stage = 1 if self.lidar_restart_mode == 3 else 0

            if self.lidar_restart_mode == 1:
                mode_str = "雷达1"
                self.current_pub = self.lidar_restart_pub
                self.current_send_value = self.restart_lidar_value
            elif self.lidar_restart_mode == 2:
                mode_str = "雷达2"
                self.current_pub = self.lidar2_restart_pub
                self.current_send_value = self.restart_lidar2_value
            else:
                mode_str = "双雷达both (当前第1个)"
                self.current_pub = self.lidar_restart_pub
                self.current_send_value = self.restart_lidar_value

            rospy.loginfo(f"重启雷达指令({mode_str})开始发送")
            self.update_ros_status("lidar", f"雷达重启：正在发送{mode_str}...", "#f1fa8c")

            self.is_publishing = True
            self.start_publish_loop()
        except Exception as e:
            rospy.logerr(f"重启雷达指令发送失败: {str(e)}")
            self.update_ros_status("lidar", f"雷达重启：执行失败 {str(e)}", "#ff5555")
            self.is_publishing = False
            self.current_command = None
            self.update_ros_status("common", "系统状态：空闲", "#50fa7b")
            self.process_next_command()

    def execute_send_param(self):
        """参数发送按钮点击入口，入队"""
        self.enqueue_command(self.CMD_TYPE_PARAM)

    def _start_param_send(self):
        """参数发送核心逻辑：写txt + 循环发布"""
        try:
            # 按json顺序取出数值，空格分隔写入文件
            vals = [str(self.params_dict[k]) for k in self.params_meta.keys()]
            with open(self.params_target_txt, "w", encoding="utf-8") as f:
                f.write(" ".join(vals))
            rospy.loginfo(f"参数写入文件{self.params_target_txt}, 值列表：{vals}")
            # 更新状态栏
            self.update_ros_status("param", f"参数指令：正在发送", "#f1fa8c")
            # 配置发布器与发送值
            self.current_pub = self.send_param_pub
            self.current_send_value = self.send_param_value
            self.is_publishing = True
            self.start_publish_loop()
        except Exception as e:
            err_msg = f"参数发送失败：{str(e)}"
            rospy.logerr(err_msg)
            self.update_ros_status("param", f"参数指令：发送失败 {str(e)}", "#ff5555")
            messagebox.showerror("参数发送错误", err_msg)
            self.is_publishing = False
            self.current_command = None
            self.update_ros_status("common", "系统状态：空闲", "#50fa7b")
            self.process_next_command()

    # ==========================================
    #  第三块内容：设备自检区
    # ==========================================
    def build_check_block_content(self, parent):
        self.build_ros_status_bar(parent)

        wrap_frame = tk.Frame(parent, bg="#2d2d2d")
        wrap_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        scrollbar = tk.Scrollbar(wrap_frame, orient=tk.VERTICAL, width=70)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.check_canvas = tk.Canvas(wrap_frame, bg="#2d2d2d", highlightthickness=0, yscrollcommand=scrollbar.set)
        self.check_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.check_canvas.yview)

        self.check_flow_frame = tk.Frame(self.check_canvas, bg="#2d2d2d")
        self.check_flow_window = self.check_canvas.create_window((0, 0), window=self.check_flow_frame, anchor="nw")

        self.check_block_widgets = []
        for key in self.check_tasks_meta.keys():
            block = self._create_single_check_widget(self.check_flow_frame, key)
            self.check_block_widgets.append(block)

        self.check_canvas.bind("<Configure>", self._on_check_canvas_resize)
        self.check_flow_frame.bind("<Configure>",
                                   lambda e: self.check_canvas.config(scrollregion=self.check_canvas.bbox("all")))
        self.check_canvas.bind("<Button-4>", lambda e: self.check_canvas.yview_scroll(-1, "units"))
        self.check_canvas.bind("<Button-5>", lambda e: self.check_canvas.yview_scroll(1, "units"))

    def _create_single_check_widget(self, parent_container, task_key):
        meta = self.check_tasks_meta[task_key]
        block = tk.Frame(parent_container, bg="#383838", bd=2, relief=tk.RAISED, height=300)
        block.pack_propagate(False)

        title_lbl = tk.Label(block, text=meta["name"], bg="#383838", fg="white", font=("DejaVu Sans", 25, "bold"))
        title_lbl.pack(anchor="nw", padx=10, pady=6)

        content_frame = tk.Frame(block, bg="#383838")
        content_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        btn_frame = tk.Frame(content_frame, bg="#383838")
        btn_frame.pack(side=tk.LEFT)
        btn_on = tk.Button(btn_frame, text="开", font=("DejaVu Sans", 50, "bold"), width=3, height=5,
                           command=lambda k=task_key: self.execute_check_cmd(k, 'on'))
        btn_on.pack(side=tk.LEFT, padx=5)
        btn_off = tk.Button(btn_frame, text="关", font=("DejaVu Sans", 50, "bold"), width=3, height=5,
                            command=lambda k=task_key: self.execute_check_cmd(k, 'off'))
        btn_off.pack(side=tk.LEFT, padx=5)

        status_var = tk.StringVar(value="未执行")
        status_lbl = tk.Label(content_frame, textvariable=status_var, bg="#383838", fg="white",
                              font=("DejaVu Sans", 30, "bold"), justify=tk.RIGHT)
        status_lbl.pack(side=tk.RIGHT, padx=20)

        self.check_ui_elements[task_key] = {
            "block": block, "title": title_lbl, "content_frame": content_frame,
            "btn_frame": btn_frame, "status_var": status_var, "status_lbl": status_lbl
        }

        def _scroll(e): self.check_canvas.yview_scroll(-1 if e.num == 4 else 1, "units")

        for w in (block, content_frame, btn_frame):
            w.bind("<Button-4>", _scroll)
            w.bind("<Button-5>", _scroll)
        return block

    def _on_check_canvas_resize(self, event):
        canvas_width = event.width
        if canvas_width <= 1: return
        cols = 2
        for i in range(cols): self.check_flow_frame.columnconfigure(i, weight=1, uniform="check_cols")
        new_block_width = max(150, (canvas_width // cols) - 20)

        for widget in self.check_block_widgets: widget.grid_forget()
        for idx, widget in enumerate(self.check_block_widgets):
            widget.config(width=new_block_width)
            widget.grid(row=idx // cols, column=idx % cols, pady=10, sticky="n")
        self.check_canvas.itemconfig(self.check_flow_window, width=canvas_width)

    def update_check_ui(self, task_key, bg_color, text, text_color="white"):
        elems = self.check_ui_elements[task_key]
        elems["status_var"].set(text)
        elems["status_lbl"].config(fg=text_color)
        for w in (elems["block"], elems["title"], elems["content_frame"], elems["btn_frame"], elems["status_lbl"]):
            w.config(bg=bg_color)

    def execute_check_cmd(self, task_key, action):
        """点击开/关的逻辑拦截器"""
        state = self.check_states[task_key]
        if action == 'on' and state not in [0, 4]: return  # 仅空闲或已完成时可按开
        if action == 'off' and state != 2: return  # 仅自检中可按关

        # === 修改：如果当前 ROS 正在发送/等待其他指令，先显示等待发送 ===
        if self.is_publishing:
            # 状态机：5 代表处于排队等待状态
            self.check_states[task_key] = 5
            txt = "等待\n发送中" if action == 'on' else "关闭自检\n等待中"
            self.update_check_ui(task_key, "#BFB936", txt)  # 变黄
        # =========================================================

        self.enqueue_command(f"check_{task_key}_{action}")
    # ==========================================================================

    # =========================================
    # 数据监视区
    # =========================================
    def build_monitor_block_content(self, parent):
        """监视区页面：顶部常驻ROS状态栏，下方流式布局显示块"""
        self.build_ros_status_bar(parent)

        # === 新增：精度控制行 ===
        precision_frame = tk.Frame(parent, bg="#2d2d2d")
        precision_frame.pack(fill=tk.X, padx=10)

        # 靠左写文字“精度：”
        tk.Label(precision_frame, text="精度：", bg="#2d2d2d", fg="white", font=("DejaVu Sans", 30, "bold")).pack(
            side=tk.LEFT)

        # 中间放置精度选择按钮
        precisions = ["2", "3", "4", "5"]
        for p in precisions:
            btn = tk.Button(
                precision_frame, text=f".{p}", font=("DejaVu Sans", 35, "bold"),
                bg="#44475a", fg="white", width=4,
                command=lambda val=p: self._change_precision(val)
            )
            btn.pack(side=tk.LEFT, padx=5)
            self.precision_btns[p] = btn

        # 最后放一个字符 f
        tk.Label(precision_frame, text="f", bg="#2d2d2d", fg="white", font=("DejaVu Sans", 30, "bold")).pack(
            side=tk.LEFT, padx=2)

        # 默认激活选中的高亮样式
        self._change_precision(f"{self.monitor_precision}")
        # =======================

        # --- 下方保持原本的 wrap_frame、Canvas 和滚动条布局不变 ---
        wrap_frame = tk.Frame(parent, bg="#2d2d2d")
        wrap_frame.pack(fill=tk.BOTH, expand=True, padx=10)

        scrollbar = tk.Scrollbar(wrap_frame, orient=tk.VERTICAL, width=70)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.monitor_canvas = tk.Canvas(wrap_frame, bg="#2d2d2d", highlightthickness=0, yscrollcommand=scrollbar.set)
        self.monitor_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.monitor_canvas.yview)

        self.monitor_flow_frame = tk.Frame(self.monitor_canvas, bg="#2d2d2d")
        self.monitor_flow_window = self.monitor_canvas.create_window((0, 0), window=self.monitor_flow_frame,
                                                                     anchor="nw")

        self.monitor_block_widgets = []

        # 根据 json 生成支持的模块块
        for item in self.monitor_meta:
            if item.get("type") not in ["nav_msgs/Odometry", "sensor_msgs/Imu"]:
                continue  # 暂不认识的话题类型不创建块
            block = self._create_single_monitor_widget(self.monitor_flow_frame, item)
            self.monitor_block_widgets.append(block)

        self.monitor_canvas.bind("<Configure>", self._on_monitor_canvas_resize)
        self.monitor_flow_frame.bind(
            "<Configure>",
            lambda e: self.monitor_canvas.config(scrollregion=self.monitor_canvas.bbox("all"))
        )

        self.monitor_canvas.bind("<Button-4>", lambda e: self.monitor_canvas.yview_scroll(-1, "units"))
        self.monitor_canvas.bind("<Button-5>", lambda e: self.monitor_canvas.yview_scroll(1, "units"))

    def _create_single_monitor_widget(self, parent_container, item):
        """单个监视UI块与ROS订阅逻辑"""
        topic_name = item["topic"]
        msg_type = item["type"]

        # 1. 从 JSON 中读取设置的采样频率，若不存在则默认 30Hz
        freq = item.get("frequency", 30.0)

        block = tk.Frame(parent_container, bg="#383838", bd=2, relief=tk.RAISED, height=250)
        block.pack_propagate(False)

        tk.Label(block, text=item["name"], bg="#383838", fg="white", font=("DejaVu Sans", 25, "bold")).pack(anchor="w",
                                                                                                            padx=10,
                                                                                                            pady=6)

        data_var = tk.StringVar(value="等待数据接入...")
        self.monitor_vars[topic_name] = data_var

        data_label = tk.Label(
            block, textvariable=data_var, bg="#282a36", fg="white",
            font=("DejaVu Sans", 20, "bold"), justify=tk.LEFT, anchor="nw"
        )
        data_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        self.monitor_labels[topic_name] = data_label
        self.monitor_last_update[topic_name] = 0.0
        self.monitor_last_process_time[topic_name] = 0.0  # 初始化单话题处理时间

        # 2. 带有频率控制与动态精度的 Odometry 回调
        if msg_type == "nav_msgs/Odometry":
            def odom_cb(msg, var=data_var, top=topic_name, f=freq):
                current_time = time.time()
                # 按照频率周期拦截采样 (周期 = 1.0 / 频率)
                if current_time - self.monitor_last_process_time[top] < (1.0 / f):
                    return

                pos = msg.pose.pose.position
                q = msg.pose.pose.orientation
                r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])

                # 获取当前选中的精度
                prec = self.monitor_precision
                text = (f"XYZ: {pos.x:.{prec}f}, {pos.y:.{prec}f}, {pos.z:.{prec}f}\n"
                        f"RPY: {math.degrees(r):.{prec}f}°, {math.degrees(p):.{prec}f}°, {math.degrees(y):.{prec}f}°")
                var.set(text)

                self.monitor_last_update[top] = current_time
                self.monitor_last_process_time[top] = current_time

            sub = rospy.Subscriber(topic_name, Odometry, odom_cb, queue_size=1)
            self.monitor_subs.append(sub)

        # 3. 带有频率控制与动态精度的 Imu 回调
        elif msg_type == "sensor_msgs/Imu":
            def imu_cb(msg, var=data_var, top=topic_name, f=freq):
                current_time = time.time()
                # 按照频率周期拦截采样
                if current_time - self.monitor_last_process_time[top] < (1.0 / f):
                    return

                q = msg.orientation
                r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
                w = msg.angular_velocity
                a = msg.linear_acceleration

                # 获取当前选中的精度
                prec = self.monitor_precision
                text = (f"角速度: {w.x:.{prec}f}, {w.y:.{prec}f}, {w.z:.{prec}f}\n"
                        f"线加速: {a.x:.{prec}f}, {a.y:.{prec}f}, {a.z:.{prec}f}\n"
                        f"RPY姿态角: {math.degrees(r):.{prec}f}°, {math.degrees(p):.{prec}f}°, {math.degrees(y):.{prec}f}°")
                var.set(text)

                self.monitor_last_update[top] = current_time
                self.monitor_last_process_time[top] = current_time

            sub = rospy.Subscriber(topic_name, Imu, imu_cb, queue_size=1)
            self.monitor_subs.append(sub)

        # 滚轮穿透绑定
        def _scroll(e):
            self.monitor_canvas.yview_scroll(-1 if e.num == 4 else 1, "units")

        block.bind("<Button-4>", _scroll)
        block.bind("<Button-5>", _scroll)
        for child in block.winfo_children():
            child.bind("<Button-4>", _scroll)
            child.bind("<Button-5>", _scroll)

        return block

    def _on_monitor_canvas_resize(self, event):
        """画布宽度变化时自动两等分布局"""
        canvas_width = event.width
        if canvas_width <= 1: return
        cols = 2
        for i in range(cols):
            self.monitor_flow_frame.columnconfigure(i, weight=1, uniform="monitor_cols")

        new_block_width = max(150, (canvas_width // cols) - 20)
        for widget in self.monitor_block_widgets:
            widget.grid_forget()
        for idx, widget in enumerate(self.monitor_block_widgets):
            widget.config(width=new_block_width)
            widget.grid(row=idx // cols, column=idx % cols, pady=10, sticky="n")
        self.monitor_canvas.itemconfig(self.monitor_flow_window, width=canvas_width)

    def _check_monitor_timeout(self):
        """主线程定时器：根据时间戳安全更新 UI 字体颜色"""
        current_time = time.time()
        timeout_threshold = 0.2  # 超时时间(秒)，大于1秒未更新即判定为断流，变回白色

        for topic, last_time in self.monitor_last_update.items():
            if last_time == 0.0:
                continue  # 还没收到过任何数据，保持白色不变

            label = self.monitor_labels[topic]

            # 判断是否超时
            if (current_time - last_time) > timeout_threshold:
                if label.cget("fg") != "white":
                    label.config(fg="white")
            else:
                if label.cget("fg") != "#50fa7b":
                    label.config(fg="#50fa7b")  # 活跃状态为绿色

        # 每 200 毫秒循环自检一次，不会阻塞主线程
        self.master.after(200, self._check_monitor_timeout)

    # --- 添加为 TouchscreenUI 类的类方法 ---
    def _change_precision(self, val):
        """切换全局显示精度并更新按钮高亮"""
        self.monitor_precision = int(val)
        for p, btn in self.precision_btns.items():
            if p == val:
                btn.config(bg="white", fg="black")  # 选中高亮
            else:
                btn.config(bg="#44475a", fg="white")  # 恢复暗色

    # ==========================================
    # 第三块内容：本地脚本极简运行区 (升级版)
    # ==========================================
    def build_script_block_content(self, parent):
        # 新增：顶部常驻ROS状态栏
        self.build_ros_status_bar(parent)

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
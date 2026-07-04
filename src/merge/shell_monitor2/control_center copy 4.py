#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from PIL import ImageDraw
from PIL import ImageFont
from std_msgs.msg import String
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import subprocess
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

        # 构建三大功能块
        self.build_grid_block(self.main_frame, col=0)
        self.build_ros_command_block(self.main_frame, col=1)
        # self.build_script_block(self.main_frame, col=2)

        self.save_map_path = r"/home/awwsome/RC/shell_monitor/test/map.txt"

        self.database_path = r"./path.txt"
        self.target_path = r"../path/map.txt"

        # 初始化控制状态（默认发送 1）
        self.send_path_value = 1

        # 注册发布者与订阅者
        self.send_path_pub = rospy.Publisher('/scripts/send_path', Int32, queue_size=10)
        self.status_sub = rospy.Subscriber('/scripts/response', Int32, self.status_callback)

        # 启动程序即刻开始循环发送话题
        self.start_publish_loop()

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
        
        self.pos_constraint_label = tk.Label(constraint_frame, textvariable=self.pos_constraint_var, fg="#ff5555", bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold'))
        self.pos_constraint_label.pack(side=tk.LEFT, expand=True)
        
        self.qty_constraint_label = tk.Label(constraint_frame, textvariable=self.qty_constraint_var, fg="#ff5555", bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold'))
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
        path_display_frame = tk.Frame(frame, bg='#2d2d2d')
        path_display_frame.pack(fill=tk.X, pady=5, padx=10)
        
        self.r2_path_var = tk.StringVar(value="R2 路径规划：暂无")
        self.r1_path_var = tk.StringVar(value="R1 路径规划：暂无")
        
        # R2 在上，R1 在下
        tk.Label(path_display_frame, textvariable=self.r2_path_var, fg="#8be9fd", bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold')).pack(anchor=tk.W, pady=2)
        tk.Label(path_display_frame, textvariable=self.r1_path_var, fg="#ffb86c", bg="#2d2d2d", font=('DejaVu Sans', 14, 'bold')).pack(anchor=tk.W, pady=2)

        # ---------------------------------------------------------------------

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
        is_valid = qty_ok and pos_ok
        if is_valid:
            self.update_parsed_path_display()
        else:
            self.r2_path_var.set("R2 路径规划：暂无")
            self.r1_path_var.set("R1 路径规划：暂无")
            self.r2_robot_path.clear()
            self.r2_targets.clear()
            self.r1_targets.clear()
            self.redraw_canvas()

        return is_valid
    
    def update_parsed_path_display(self):
        """需求：立刻匹配数据库并解析 R2 与 R1 的路径规划信息，并提取图形化格位"""
        import re
        
        # 每次清空上一次的图形线框记录
        self.r2_robot_path.clear()
        self.r2_targets.clear()
        self.r1_targets.clear()

        mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
        current_seq = "".join([mapping[self.grid_states[i]] for i in range(12)])
        
        if not os.path.exists(self.database_path):
            self.r2_path_var.set("R2 路径规划：[文件丢失] 找不到数据库")
            self.r1_path_var.set("R1 路径规划：[文件丢失] 找不到数据库")
            self.redraw_canvas()
            return
            
        matched_content = None
        try:
            with open(self.database_path, "r", encoding="utf-8") as f:
                for line in f:
                    line_str = line.strip()
                    if not line_str:
                        continue
                    if line_str.startswith(current_seq):
                        matched_content = line_str[12:].strip()
                        break
        except Exception as e:
            rospy.logerr(f"读取数据库出错: {e}")
            return
            
        if matched_content:
            tokens = matched_content.split()
            if '0' in tokens:
                split_idx = tokens.index('0')
                r2_tokens = tokens[:split_idx]
                r1_tokens = tokens[split_idx+1:]
            else:
                parts = matched_content.split('0')
                r2_tokens = parts[0].split() if len(parts) > 0 else []
                r1_tokens = parts[1].split() if len(parts) > 1 else []
                
            # --- 新增：解析具体格位用于画布渲染 (字符转为 0-11 索引) ---
            # R2 解析：形式如 "3" 或 "3(5)"
            for token in r2_tokens:
                match = re.match(r"(\d+)(?:\((\d+)\))?", token)
                if match:
                    r_pos = int(match.group(1)) - 1
                    if 0 <= r_pos < 12:
                        self.r2_robot_path.append(r_pos)
                    if match.group(2): # 如果有括号
                        t_pos = int(match.group(2)) - 1
                        if 0 <= t_pos < 12:
                            self.r2_targets.append(t_pos)
                            
            # R1 解析：每一个都是要夹的方块位置
            for token in r1_tokens:
                if token.isdigit():
                    t_pos = int(token) - 1
                    if 0 <= t_pos < 12:
                        self.r1_targets.append(t_pos)
            # ---------------------------------------------------------
            
            r2_str = " → ".join(r2_tokens) if r2_tokens else "原地待命"
            r1_str = " → ".join(r1_tokens) if r1_tokens else "原地待命"
            
            self.r2_path_var.set(f"R2 路径规划：{r2_str}")
            self.r1_path_var.set(f"R1 路径规划：{r1_str}")
        else:
            self.r2_path_var.set("R2 路径规划：数据库中无匹配序列")
            self.r1_path_var.set("R1 路径规划：数据库中无匹配序列")

        # 触发重绘以渲染路径和线框
        self.redraw_canvas()

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
            "map_blue": [   # 200mm: #295210, 400mm: #2A7138, 600mm: #98A650
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
            num_font_size = max(10, int(self.cell_h * 0.12))
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

        text_map = {0: "0", 1: "R1", 2: "R2", 3: "F"}
        for idx, state in enumerate(self.states_config):
            img = Image.new('RGBA', (round(self.cell_w * 3 / 4), round(self.cell_h * 3 / 4)), (0, 0, 0, 0))
            draw = ImageDraw.Draw(img)
            # 居中对齐微调
            draw.text((int(self.cell_w * 0.18), int(self.cell_h * 0.18)), text_map[idx], fill=state["rgb"], font=my_font)
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
            print(font_size)
            text_map = {0: "0", 1: "R1", 2: "R2", 3: "F"}
            
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
        """核心绘图：在画布上绘制 R2 蓝色路径线，以及 R1/R2 的动作方块目标框"""
        # 1. 绘制 R2 机器人移动路径（蓝色带箭头曲线/直线连接中心点）
        if len(self.r2_robot_path) > 1:
            for i in range(len(self.r2_robot_path) - 1):
                idx1 = self.r2_robot_path[i]
                idx2 = self.r2_robot_path[i+1]
                
                x1_f, y1_f = self.get_coords_from_index(idx1)
                x2_f, y2_f = self.get_coords_from_index(idx2)
                
                # 计算两个格子的中心点坐标
                cx1, cy1 = x1_f + self.cell_w / 2, y1_f + self.cell_h / 2
                cx2, cy2 = x2_f + self.cell_w / 2, y2_f + self.cell_h / 2
                
                # 绘制蓝色带箭头线段（使用标准浅蓝色 #00ffff 凸显）
                self.canvas.create_line(cx1, cy1, cx2, cy2, fill="#00ffff", width=30, 
                                        arrow=tk.LAST, arrowshape=(15, 18, 6))

        # 2. 绘制 R2 夹取的方块（蓝色框住中心字周围区域，收缩到 0.15 范围）
        for idx in self.r2_targets:
            x1, y1 = self.get_coords_from_index(idx)
            bx1, by1 = x1 + self.cell_w * 0.15, y1 + self.cell_h * 0.15
            bx2, by2 = x1 + self.cell_w * 0.85, y1 + self.cell_h * 0.85
            self.canvas.create_rectangle(bx1, by1, bx2, by2, outline="#00ffff", width=4)

        # 3. 绘制 R1 夹取的方块（橙色框住，收缩范围设为 0.1，防止两者重合时外框重叠）
        for idx in self.r1_targets:
            x1, y1 = self.get_coords_from_index(idx)
            bx1, by1 = x1 + self.cell_w * 0.08, y1 + self.cell_h * 0.08
            bx2, by2 = x1 + self.cell_w * 0.92, y1 + self.cell_h * 0.92
            self.canvas.create_rectangle(bx1, by1, bx2, by2, outline="#ffb86c", width=4)

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
        """需求 2：程序启动后一直高频周期性发送指定状态值"""
        if not rospy.is_shutdown():
            self.send_path_pub.publish(self.send_path_value)
            # 每 100 毫秒（10Hz频率）发送一次，维持常驻发送
            self.master.after(100, self.start_publish_loop)

    def status_callback(self, msg):
        """需求 3：订阅状态话题，收到 2 时将发送状态重置为 1"""
        if msg.data == 2:
            self.send_path_value = 1
            # --- 新增：收到底层应答后变绿 ---
            self.path_status_var.set("当前路径：已发送")
            self.path_status_label.config(fg="#50fa7b")  # 亮绿色
            # -----------------------------

    def execute_send_path(self):
        """需求 4：解析当前序列，匹配数据库文件并写入新路径"""
        # --- 新增：发送前的约束强校验 ---
        if not self.check_map_validity():
            messagebox.showwarning("拦截", "当前地图不满足约束条件，禁止发送路径！")
            return
        # -----------------------------

        try:
            # 映射当前梅林序列 (UI: 0=空, 1=R1, 2=R2, 3=Fake -> 文件: 4, 1, 2, 3)
            mapping = {0: '4', 1: '1', 2: '2', 3: '3'}
            current_seq = "".join([mapping[self.grid_states[i]] for i in range(12)])

            # # --- 新增：防重复发送拦截 ---
            # if current_seq == self.last_sent_seq:
            #     messagebox.showwarning("拦截", "当前路径与上次发送的路径相同，请勿重复发送！")
            #     return
            # # --------------------------

            if not os.path.exists(self.database_path):
                raise FileNotFoundError(f"找不到梅林数据库文件: {self.database_path}")

            matched_content = None
            with open(self.database_path, "r", encoding="utf-8") as f:
                for line in f:
                    line_str = line.strip()
                    if not line_str:
                        continue
                    # 匹配前12位序列
                    if line_str.startswith(current_seq):
                        rem = line_str[12:]
                        # 去掉序列后的第一个空格，保留其余全部信息
                        if rem.startswith(" "):
                            matched_content = rem[1:]
                        else:
                            matched_content = rem
                        break

            if matched_content is None:
                raise ValueError(f"数据库中未匹配到当前序列: {current_seq}")

            # 原封不动写入目标文件
            with open(self.target_path, "w", encoding="utf-8") as f_out:
                f_out.write(matched_content + "\n")

            # 成功处理：逻辑切换为 2，并弹出提示
            self.send_path_value = 2
            self.last_sent_seq = current_seq  # 记录当前发送的序列
            # messagebox.showinfo("成功", "路径匹配并发送成功,等待应答...")
            self.path_status_var.set("当前路径：正在发送...")
            rospy.loginfo("梅林序列匹配成功，数据已成功写入目标路径。")

        except Exception as e:
            # 失败处理：弹窗警告并打印报错日志
            error_msg = str(e)
            rospy.logerr(f"发送路径失败: {error_msg}")
            messagebox.showerror("失败", f"发送路径发生错误:\n{error_msg}")

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
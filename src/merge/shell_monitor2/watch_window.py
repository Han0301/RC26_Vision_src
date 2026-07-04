#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
# 在 import rospy 之前强行指定 ROS 日志配置文件路径
if 'ROS_ETC_DIR' not in os.environ:
    os.environ['ROS_ETC_DIR'] = '/opt/ros/noetic/etc/ros'
import rospy
import tkinter as tk
import json
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math


class MonitorStandalone:
    def __init__(self, master):
        self.master = master
        self.master.title("数据监视区")
        # 窗口尺寸逻辑与原代码保持一致
        screen_width = self.master.winfo_screenwidth()
        screen_height = self.master.winfo_screenheight()
        win_width = int(screen_width * 0.8)
        win_height = int(screen_height * 0.5)
        x_coordinate = int((screen_width - win_width) / 2)
        y_coordinate = int((screen_height - win_height) / 2)
        self.master.geometry(f"{win_width}x{win_height}+{x_coordinate}+{y_coordinate}")
        self.master.configure(bg='#1e1e1e')

        # 初始化 ROS 节点
        rospy.init_node('monitor_only_node', anonymous=True)
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

        # ========== 原代码：监视区全部变量 一字未改 ==========
        self.monitor_config_path = r"./monitor_config.json"
        self.monitor_meta = []
        self.monitor_subs = []  # 存储 ROS 订阅者句柄，防止被垃圾回收
        self.monitor_vars = {}  # 存储话题名称对应的 StringVar
        self.monitor_labels = {}  # 存储 Label 引用，用于修改颜色
        self.monitor_last_update = {}  # 存储各个话题最后一次收到数据的时间戳
        self.monitor_precision = 3  # 默认显示精度（小数点后2位）
        self.precision_btns = {}  # 存储精度选择按钮句柄
        self.monitor_last_process_time = {}  # 存储每个话题上一次真正处理/更新的时间戳

        # 原代码执行顺序
        self._load_monitor_json()
        self.build_monitor_block_content(self.master)
        self._check_monitor_timeout()  # 启动颜色监控循环定时器

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

    # ===================== 原代码：监视区所有方法 一字未改 =====================
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

    def build_monitor_block_content(self, parent):
        """监视区页面：顶部常驻ROS状态栏，下方流式布局显示块"""
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

    def _change_precision(self, val):
        """切换全局显示精度并更新按钮高亮"""
        self.monitor_precision = int(val)
        for p, btn in self.precision_btns.items():
            if p == val:
                btn.config(bg="white", fg="black")  # 选中高亮
            else:
                btn.config(bg="#44475a", fg="white")  # 恢复暗色
    # ==================================================================

    def on_closing(self):
        """窗口关闭处理"""
        rospy.signal_shutdown("UI Window Closed")
        self.master.destroy()


if __name__ == '__main__':
    try:
        root = tk.Tk()
        app = MonitorStandalone(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext, font, filedialog, colorchooser
import subprocess
import threading
import os
import platform
from time import strftime
import re
import signal
import sys
import json
import queue
import psutil
import time
import select
import fcntl
import traceback
import faulthandler
import logging
from datetime import datetime

# 启用段错误追踪
faulthandler.enable()

# 设置日志记录
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'logs')
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, f'debug_{datetime.now().strftime("%Y%m%d_%H%M%S")}.log')

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(threadName)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s',
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler(sys.stdout)
    ]
)

logger = logging.getLogger(__name__)


# 装饰器用于追踪函数调用
def debug_trace(func):
    def wrapper(*args, **kwargs):
        logger.debug(f"Entering {func.__name__}")
        try:
            result = func(*args, **kwargs)
            logger.debug(f"Exiting {func.__name__} successfully")
            return result
        except Exception as e:
            logger.error(f"Error in {func.__name__}: {str(e)}")
            logger.error(traceback.format_exc())
            raise

    return wrapper


# 线程安全的GUI更新
def safe_gui_update(widget, update_func, *args, **kwargs):
    """确保GUI更新在主线程中进行"""
    try:
        if threading.current_thread() is threading.main_thread():
            update_func(*args, **kwargs)
        else:
            widget.after(0, update_func, *args, **kwargs)
    except Exception as e:
        logger.error(f"GUI update error: {str(e)}")


class ScriptConsoleTab(ttk.Frame):
    """单独的脚本控制台标签页 - 优化文本显示并确保可复制"""

    def __init__(self, master, script_name, script_path, app):
        logger.info(f"Creating console tab for {script_name}")
        super().__init__(master)
        self.app = app
        self.script_name = script_name
        self.script_path = script_path
        self.running = False
        self.process = None
        self.autoscroll = tk.BooleanVar(value=True)
        self.timestamps = tk.BooleanVar(value=False)
        self.output_buffer = queue.Queue()
        self.flush_interval = 100  # ms
        self.max_flush_lines = 500
        self._flushing = False
        self._destroyed = False  # 添加销毁标志

        # 将这些属性初始化移到 create_header() 之前
        self.press_start_time = None
        self.long_press_threshold = 800  # 长按阈值，单位毫秒
        self.is_long_press = False
        self.press_timer = None
        self.background_processes = []  # 后台进程列表
        self.terminal_processes = []  # 终端进程列表
        self.script_processes = []  # 脚本进程列表（通过终端启动的）

        self.after(self.flush_interval, self.flush_console_buffer)

        # 配置标签页样式
        self.configure(padding=(10, 5, 10, 10))
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        # 创建控制台标题栏
        self.create_header()

        # 创建控制台区域 - 确保文本可复制
        self.create_console()

        # 添加初始输出
        self.update_console(f"=== {self.script_name} 控制台 ===\n")
        self.update_console(f"脚本路径: {self.script_path}\n\n")

        # 添加 ANSI 颜色映射
        self.ansi_colors = {
            '30': '#000000',  # 黑
            '31': '#ff5555',  # 红
            '32': '#50fa7b',  # 绿
            '33': '#f1fa8c',  # 黄
            '34': '#bd93f9',  # 蓝
            '35': '#ff79c6',  # 品红
            '36': '#8be9fd',  # 青
            '37': '#f8f8f2',  # 白
            '90': '#44475a',  # 亮黑
            '91': '#ff6e6e',  # 亮红
            '92': '#69ff94',  # 亮绿
            '93': '#ffffa5',  # 亮黄
            '94': '#d6acff',  # 亮蓝
            '95': '#ff92df',  # 亮品红
            '96': '#a4ffff',  # 亮青
            '97': '#ffffff',  # 亮白
        }

        # 创建文本标签用于不同颜色
        self.setup_color_tags()

    def destroy(self):
        """重写destroy方法以清理所有进程"""
        logger.info(f"Destroying console tab for {self.script_name}")
        self._destroyed = True

        # 停止所有相关进程
        if (self.running or self.background_processes or self.terminal_processes):
            self.stop_script()

        super().destroy()

    @debug_trace
    def clean_rosmon_output(self, text):
        """清理rosmon特殊输出"""
        # 移除常见的控制序列
        text = re.sub(r'\x1B\[[?]?[0-9;]*[hlr]', '', text)  # 光标控制
        text = re.sub(r'\x1B\([B0]', '', text)  # 字符集切换
        text = re.sub(r'\x1B[78DHM]', '', text)  # 其他单字符序列

        # 移除重复的框线字符
        text = re.sub(r'▂{3,}', '─' * 80, text)  # 将多个▂替换为横线

        return text

    def setup_color_tags(self):
        """设置颜色标签 - 增强版本"""
        try:
            # 原有的标准颜色
            for code, color in self.ansi_colors.items():
                self.console.tag_config(f'fg_{code}', foreground=color)
                bg_code = str(int(code) + 10) if int(code) < 40 else code
                self.console.tag_config(f'bg_{bg_code}', background=color)

            # 特殊样式
            self.console.tag_config('bold', font=('DejaVu Sans Mono', 10, 'bold'))
            self.console.tag_config('underline', underline=True)

            # 动态创建 RGB 颜色标签的方法
            self._rgb_tags = set()
        except Exception as e:
            logger.error(f"Error setting up color tags: {str(e)}")

    @debug_trace
    def parse_ansi_text(self, text):
        """解析 ANSI 转义序列并返回带标签的文本段 - 增强支持 rosmon"""
        # 更全面的 ANSI 转义序列正则表达式
        ansi_escape = re.compile(
            r'\x1B'  # ESC
            r'(?:'  # 开始非捕获组
            r'\[([0-9;]*[0-9]*)([ABCDEFGHJKSTfmnsulh])|'  # CSI序列
            r'\]([0-9;]*);([^\x07\x1B]*)\x07|'  # OSC序列
            r'\([B0]|'  # 字符集选择
            r'[78DHM]|'  # 单字符序列
            r'\[[?]?[0-9;]*[hl]'  # DEC私有模式
            r')'  # 结束非捕获组
        )

        # 同时过滤掉特殊的控制字符
        text = text.replace('\x1B(B', '')  # 移除字符集切换
        text = re.sub(r'\x1B\[[\?]?[0-9;]*[hl]', '', text)  # 移除光标显示/隐藏
        segments = []
        last_end = 0
        current_tags = []

        for match in ansi_escape.finditer(text):
            # 添加之前的普通文本
            if match.start() > last_end:
                segments.append((text[last_end:match.start()], tuple(current_tags)))

            codes = match.group(1)
            command = match.group(2)

            # 处理光标控制序列（过滤掉，GUI 中不需要）
            if command in ['A', 'B', 'C', 'D', 'G', 'K', 'H']:
                last_end = match.end()
                continue

            # 处理颜色序列
            if command == 'm' and codes:
                code_list = codes.split(';')
                i = 0
                while i < len(code_list):
                    code = code_list[i]

                    if code == '0':  # 重置
                        current_tags = []
                    elif code == '1':  # 粗体
                        if 'bold' not in current_tags:
                            current_tags.append('bold')
                    elif code == '4':  # 下划线
                        if 'underline' not in current_tags:
                            current_tags.append('underline')
                    elif code == '38':  # 前景色
                        if i + 2 < len(code_list) and code_list[i + 1] == '2':
                            # 24位颜色: 38;2;r;g;b
                            if i + 4 < len(code_list):
                                try:
                                    r, g, b = int(code_list[i + 2]), int(code_list[i + 3]), int(code_list[i + 4])
                                    color_hex = f"#{r:02x}{g:02x}{b:02x}"
                                    current_tags = [tag for tag in current_tags if not tag.startswith('fg_')]
                                    current_tags.append(f'fg_rgb_{color_hex}')
                                except ValueError:
                                    logger.warning(f"Invalid RGB values: {code_list[i + 2:i + 5]}")
                                i += 4
                            else:
                                i += 1
                        else:
                            i += 1
                    elif code in self.ansi_colors:  # 标准 16 色前景色
                        current_tags = [tag for tag in current_tags if not tag.startswith('fg_')]
                        current_tags.append(f'fg_{code}')
                    i += 1

            last_end = match.end()

        # 添加剩余的文本
        if last_end < len(text):
            segments.append((text[last_end:], tuple(current_tags)))

        return segments

    def create_rgb_tag_if_needed(self, tag_name, color_hex):
        """动态创建 RGB 颜色标签"""
        if tag_name not in self._rgb_tags:
            try:
                if tag_name.startswith('fg_rgb_'):
                    self.console.tag_config(tag_name, foreground=color_hex)
                elif tag_name.startswith('bg_rgb_'):
                    self.console.tag_config(tag_name, background=color_hex)
                self._rgb_tags.add(tag_name)
            except tk.TclError as e:
                logger.warning(f"Invalid color {color_hex}: {str(e)}")

    def update_console_with_color(self, text):
        """更新控制台，保留颜色 - 增强版本"""
        if not text or self._destroyed:
            return

        # 使用线程安全的方式更新
        safe_gui_update(self, self._update_console_with_color_impl, text)

    def _update_console_with_color_impl(self, text):
        """实际的控制台更新实现"""
        try:
            if self._destroyed or not self.console.winfo_exists():
                return

            text = self.clean_rosmon_output(text)
            segments = self.parse_ansi_text(text)

            self.console.config(state='normal')

            for segment_text, tags in segments:
                if self.timestamps.get() and segment_text.strip():
                    timestamp = f"[{strftime('%H:%M:%S')}] "
                    self.console.insert(tk.END, timestamp)

                if tags:
                    # 为 RGB 颜色创建标签
                    for tag in tags:
                        if tag.startswith(('fg_rgb_', 'bg_rgb_')):
                            color_hex = tag.split('_')[-1]
                            self.create_rgb_tag_if_needed(tag, color_hex)

                    self.console.insert(tk.END, segment_text, tags)
                else:
                    self.console.insert(tk.END, segment_text)

            if self.autoscroll.get():
                self.console.see(tk.END)

            self.console.config(state='disabled')
        except Exception as e:
            logger.error(f"Error updating console with color: {str(e)}")

    def on_terminal_btn_press(self, event):
        """独立终端按钮按下事件"""
        import time
        self.press_start_time = time.time() * 1000  # 记录按下时间
        self.is_long_press = False  # 重置长按标志
        self.long_press_executed = False  # 新增：长按是否已执行标志

        # 设置按钮按下状态
        self.terminal_btn.state(['pressed'])

        # 启动长按检测定时器
        self.press_timer = self.after(self.long_press_threshold, self.on_long_press_detected)

        logger.debug("Terminal button pressed")

    def create_header(self):
        """创建控制台标题栏"""
        header = ttk.Frame(self)
        header.grid(row=0, column=0, sticky='ew', pady=(0, 10))

        # 脚本名称标签
        name_label = ttk.Label(
            header,
            text=self.script_name,
            font=('DejaVu Sans Mono', 10, 'bold')
        )
        name_label.pack(side=tk.LEFT, padx=(0, 15))

        # 路径标签
        path_label = ttk.Label(
            header,
            text=self.script_path,
            font=('DejaVu Sans Mono', 8)
        )
        path_label.pack(side=tk.LEFT)

        # 按钮容器
        btn_frame = ttk.Frame(header)
        btn_frame.pack(side=tk.RIGHT)

        self.terminal_btn = ttk.Button(
            btn_frame,
            text="终端运行",
            command=self.run_in_terminal,
            width=12
        )
        self.terminal_btn.pack(side=tk.LEFT, padx=5)

        # 运行按钮
        self.run_btn = ttk.Button(
            btn_frame,
            text="运行脚本",
            command=self.run_script,
            width=12
        )
        self.run_btn.pack(side=tk.LEFT, padx=5)

        # 停止按钮
        self.stop_btn = ttk.Button(
            btn_frame,
            text="停止",
            command=self.stop_script,
            state=tk.DISABLED,
            width=8
        )
        self.stop_btn.pack(side=tk.LEFT, padx=5)

        # 清空按钮
        clear_btn = ttk.Button(
            btn_frame,
            text="清空",
            command=self.clear_console,
            width=8
        )
        clear_btn.pack(side=tk.LEFT, padx=5)

        # 绑定鼠标事件而不是command
        self.terminal_btn.bind("<Button-1>", self.on_terminal_btn_press)
        self.terminal_btn.bind("<ButtonRelease-1>", self.on_terminal_btn_release)
        self.terminal_btn.bind("<Leave>", self.on_terminal_btn_leave)
        # 导出日志按钮
        export_btn = ttk.Button(
            btn_frame,
            text="导出日志",
            command=self.export_log,
            width=12
        )
        export_btn.pack(side=tk.LEFT, padx=5)

        # 自动滚动复选框
        autoscroll_check = ttk.Checkbutton(
            btn_frame,
            text="自动滚动",
            variable=self.autoscroll
        )
        autoscroll_check.pack(side=tk.LEFT, padx=(10, 0))

        # 时间戳复选框
        timestamps_check = ttk.Checkbutton(
            btn_frame,
            text="时间戳",
            variable=self.timestamps
        )
        timestamps_check.pack(side=tk.LEFT, padx=(10, 0))

        # 分隔线
        sep = ttk.Separator(btn_frame, orient=tk.VERTICAL)
        sep.pack(side=tk.LEFT, padx=(10, 5), fill='y', pady=4)

        # 关闭按钮
        close_btn = ttk.Button(
            btn_frame,
            text="×",
            command=self.close_this_tab,
            width=3
        )
        close_btn.pack(side=tk.LEFT)

    def on_terminal_btn_release(self, event):
        """独立终端按钮释放事件"""
        if self.press_start_time is None:
            return

        import time
        current_time = time.time() * 1000
        press_duration = current_time - self.press_start_time

        # 取消长按定时器
        if self.press_timer:
            self.after_cancel(self.press_timer)
            self.press_timer = None

        # 恢复按钮状态
        self.terminal_btn.state(['!pressed'])

        logger.debug(f"Terminal button released after {press_duration}ms")

        # 只有在没有执行过长按操作时才执行短按操作
        if not self.long_press_executed and not self.is_long_press and press_duration < self.long_press_threshold:
            # 短按：在终端中运行
            self.after(10, self.run_in_terminal)  # 延迟执行避免按钮状态问题
            self.show_action_feedback("短按 - 在终端中运行", "#58a6ff")

        # 重置状态
        self.press_start_time = None
        self.is_long_press = False
        self.long_press_executed = False

    def on_terminal_btn_leave(self, event):
        """鼠标离开按钮时取消长按检测"""
        if self.press_timer:
            self.after_cancel(self.press_timer)
            self.press_timer = None

        # 恢复按钮状态
        self.terminal_btn.state(['!pressed'])

        # 重置状态
        self.press_start_time = None
        self.is_long_press = False
        self.long_press_executed = False

    def on_long_press_detected(self):
        """检测到长按 - 立即执行后台运行"""
        self.is_long_press = True
        self.long_press_executed = True  # 标记长按已执行
        logger.debug("Long press detected on terminal button - executing background run")

        # 视觉反馈：改变按钮样式提示长按
        original_style = str(self.terminal_btn['style'])

        # 创建长按状态的样式
        style = ttk.Style()
        style.configure('LongPress.TButton',
                        background='#50fa7b',  # 绿色背景
                        foreground='#0d1117')  # 深色文字

        self.terminal_btn.configure(style='LongPress.TButton')

        # 1秒后恢复原样式
        self.after(1000, lambda: self.terminal_btn.configure(style=original_style))

        # 立即执行后台运行
        self.show_action_feedback("正在后台运行", "#50fa7b")
        self.after(10, self.run_in_background)  # 稍微延迟以确保UI更新

    def show_action_feedback(self, message, color):
        """显示操作反馈信息"""
        # 在控制台中显示反馈
        self.update_console(f"\n[{message}]\n")

        # 更新应用状态
        self.app.update_status(f"{self.script_name}: {message}")

        # 创建临时的视觉反馈标签
        if hasattr(self, 'feedback_label'):
            self.feedback_label.destroy()

        self.feedback_label = ttk.Label(
            self,
            text=message,
            foreground=color,
            background="#161b22",
            font=('DejaVu Sans', 9, 'bold'),
            padding=(8, 4)
        )

        # 显示在按钮下方
        self.feedback_label.place(relx=1.0, rely=0.0, anchor="ne", x=-10, y=50)

        # 3秒后自动消失（延长显示时间，因为用户可能还在按住按钮）
        self.after(3000, lambda: self.feedback_label.destroy() if hasattr(self, 'feedback_label') else None)

    @debug_trace
    def run_in_background(self):
        """在后台静默运行脚本"""
        if not os.path.exists(self.script_path):
            messagebox.showerror("错误", f"脚本不存在: {self.script_path}")
            return
        logger.info(f"Running script in background: {self.script_name}")
        try:
            # 确保脚本有执行权限
            if platform.system() != 'Windows':
                os.chmod(self.script_path, 0o755)
            # 设置环境变量
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            # 根据操作系统静默运行
            system = platform.system()
            if system == 'Windows':
                process = subprocess.Popen(
                    [self.script_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    stdin=subprocess.DEVNULL,
                    env=env,
                    creationflags=subprocess.CREATE_NO_WINDOW | subprocess.CREATE_NEW_PROCESS_GROUP
                )
            else:
                process = subprocess.Popen(
                    ['nohup', self.script_path],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    stdin=subprocess.DEVNULL,
                    env=env,
                    preexec_fn=os.setsid
                )
            # 记录后台进程信息
            self.background_processes.append({
                'process': process,
                'pid': process.pid,
                'start_time': time.time(),
                'script_name': self.script_name,
                'script_path': self.script_path
            })
            self.update_console(f"\n[脚本已在后台启动，PID: {process.pid}]\n")
            self.app.update_status(f"{self.script_name} 已在后台运行 (PID: {process.pid})")

            # === 新增：更新按钮状态 ===
            self.stop_btn.config(state=tk.NORMAL)
            self.app.notebook.tab(self, text=f"⚫ {self.script_name}")  # 用黑色圆点表示后台运行

            # 启动监控线程检查进程状态
            threading.Thread(
                target=self.monitor_background_process,
                args=(process,),
                daemon=True,
                name=f"BackgroundMonitor-{self.script_name}"
            ).start()
        except Exception as e:
            logger.error(f"Error running script in background: {str(e)}")
            self.update_console(f"\n[错误: 无法在后台运行脚本 - {str(e)}]\n")
            self.app.update_status(f"后台运行失败: {str(e)}")
            messagebox.showerror("错误", f"无法在后台运行脚本:\n{str(e)}")

    def monitor_background_process(self, process):
        """监控后台进程状态"""
        try:
            return_code = process.wait()  # 等待进程结束
            # 更新控制台
            if return_code == 0:
                self.update_console(f"\n[后台脚本执行完成，PID: {process.pid}]\n")
                self.app.update_status(f"{self.script_name} 后台执行完成")
            else:
                self.update_console(f"\n[后台脚本执行失败，PID: {process.pid}，返回码: {return_code}]\n")
                self.app.update_status(f"{self.script_name} 后台执行失败 (代码: {return_code})")
            # 从进程列表中移除
            if hasattr(self, 'background_processes'):
                self.background_processes = [
                    p for p in self.background_processes
                    if p['pid'] != process.pid
                ]

            # === 新增：检查是否需要更新按钮状态 ===
            # 如果没有任何进程在运行，恢复按钮状态
            if (not self.background_processes and
                    not self.terminal_processes and
                    not self.running):
                safe_gui_update(self, self._restore_button_states)

        except Exception as e:
            logger.error(f"Error monitoring background process: {str(e)}")
            self.update_console(f"\n[后台进程监控错误: {str(e)}]\n")

    def _restore_button_states(self):
        """恢复按钮到初始状态"""
        try:
            if not self._destroyed and self.stop_btn.winfo_exists():
                self.stop_btn.config(state=tk.DISABLED)
            if not self._destroyed and self.run_btn.winfo_exists():
                self.run_btn.config(state=tk.NORMAL)
            if not self._destroyed and self.winfo_exists():
                self.app.notebook.tab(self, text=self.script_name)
        except Exception as e:
            logger.error(f"Error restoring button states: {str(e)}")

    def show_background_processes(self):
        """显示当前运行的所有进程"""
        all_processes = []

        # 应用内进程
        if self.running and self.process:
            all_processes.append({
                'type': '应用内',
                'pid': self.process.pid,
                'status': '运行中',
                'duration': time.time() - getattr(self, 'start_time', time.time())
            })

        # 后台进程
        for proc_info in self.background_processes:
            status = '运行中' if proc_info['process'].poll() is None else '已结束'
            all_processes.append({
                'type': '后台',
                'pid': proc_info['pid'],
                'status': status,
                'duration': time.time() - proc_info['start_time']
            })

        # 终端进程
        for term_info in self.terminal_processes:
            if term_info.get('process'):
                status = '运行中' if term_info['process'].poll() is None else '已结束'
            else:
                status = '运行中'  # macOS标签页

            all_processes.append({
                'type': f"终端({term_info.get('terminal_type', 'unknown')})",
                'pid': term_info.get('pid', term_info.get('tab_id', 'N/A')),
                'status': status,
                'duration': time.time() - term_info['start_time']
            })

        if not all_processes:
            self.update_console("\n[没有运行中的进程]\n")
            return

        self.update_console("\n=== 当前进程状态 ===\n")
        for proc in all_processes:
            duration_str = f"{int(proc['duration'])}秒"
            self.update_console(
                f"{proc['type']:12} | "
                f"PID: {str(proc['pid']):8} | "
                f"状态: {proc['status']:6} | "
                f"运行时间: {duration_str}\n"
            )
        self.update_console("====================\n\n")

    def monitor_terminal_process(self, terminal_process, terminal_type):
        """监控终端进程状态"""
        try:
            return_code = terminal_process.wait()

            # 从终端进程列表中移除
            self.terminal_processes = [
                p for p in self.terminal_processes
                if p.get('pid') != terminal_process.pid
            ]

            self.update_console(f"\n[{terminal_type} 终端已关闭，PID: {terminal_process.pid}]\n")

        except Exception as e:
            logger.error(f"Error monitoring terminal process: {str(e)}")

    @debug_trace
    def run_in_terminal(self):
        """在独立终端中运行脚本（短按功能）- 增加进程追踪"""
        if not os.path.exists(self.script_path):
            messagebox.showerror("错误", f"脚本不存在: {self.script_path}")
            return

        logger.info(f"Opening script in terminal: {self.script_name}")

        try:
            # 确保脚本有执行权限
            if platform.system() != 'Windows':
                os.chmod(self.script_path, 0o755)

            # 根据操作系统选择终端
            system = platform.system()
            terminal_process = None

            if system == 'Linux':
                # 尝试多种常见的Linux终端
                terminals = [
                    # 添加进程组ID以便于终止整个进程树
                    ['gnome-terminal', '--', 'bash', '-c',
                     f'echo "PID:$$"; {self.script_path}; echo -e "\\n按任意键退出..."; read -n1'],
                    ['konsole', '-e', 'bash', '-c',
                     f'echo "PID:$$"; {self.script_path}; echo -e "\\n按任意键退出..."; read -n1'],
                    ['xfce4-terminal', '-e',
                     f'bash -c "echo \\"PID:$$\\"; {self.script_path}; echo -e \\"\\n按任意键退出...\\"; read -n1"'],
                    ['mate-terminal', '-e',
                     f'bash -c "echo \\"PID:$$\\"; {self.script_path}; echo -e \\"\\n按任意键退出...\\"; read -n1"'],
                    ['terminator', '-e',
                     f'bash -c "echo \\"PID:$$\\"; {self.script_path}; echo -e \\"\\n按任意键退出...\\"; read -n1"'],
                    ['xterm', '-e',
                     f'bash -c "echo \\"PID:$$\\"; {self.script_path}; echo -e \\"\\n按任意键退出...\\"; read -n1"']
                ]

                for terminal_cmd in terminals:
                    try:
                        terminal_process = subprocess.Popen(
                            terminal_cmd,
                            preexec_fn=os.setsid  # 创建新的进程组
                        )

                        # 记录终端进程
                        self.terminal_processes.append({
                            'process': terminal_process,
                            'pid': terminal_process.pid,
                            'terminal_type': terminal_cmd[0],
                            'start_time': time.time(),
                            'script_path': self.script_path
                        })

                        self.update_console(f"\n[已在 {terminal_cmd[0]} 中打开脚本，终端PID: {terminal_process.pid}]\n")
                        self.app.update_status(f"已在独立终端中运行 {self.script_name}")

                        # === 新增：更新按钮状态 ===
                        self.stop_btn.config(state=tk.NORMAL)
                        self.app.notebook.tab(self, text=f"◉ {self.script_name}")  # 显示运行状态

                        # 启动监控线程
                        threading.Thread(
                            target=self.monitor_terminal_process,
                            args=(terminal_process, terminal_cmd[0]),
                            daemon=True,
                            name=f"TerminalMonitor-{self.script_name}"
                        ).start()

                        return
                    except (subprocess.SubprocessError, FileNotFoundError):
                        continue

            elif system == 'Darwin':  # macOS
                apple_script = f'''
                    tell application "Terminal"
                        activate
                        set newTab to do script "echo \\"PID:$$\\"; cd {os.path.dirname(self.script_path)} && {self.script_path}; echo -e '\\n按任意键退出...'; read -n1"
                        return id of newTab
                    end tell
                    '''
                try:
                    result = subprocess.run(['osascript', '-e', apple_script],
                                            capture_output=True, text=True)
                    if result.returncode == 0:
                        tab_id = result.stdout.strip()

                        # 对于macOS，我们记录AppleScript标识符
                        self.terminal_processes.append({
                            'process': None,  # AppleScript不返回进程对象
                            'pid': None,
                            'terminal_type': 'Terminal.app',
                            'tab_id': tab_id,
                            'start_time': time.time(),
                            'script_path': self.script_path
                        })

                        self.update_console(f"\n[已在 Terminal.app 中打开脚本，标签ID: {tab_id}]\n")
                        self.app.update_status(f"已在独立终端中运行 {self.script_name}")

                        # === 新增：更新按钮状态 ===
                        self.stop_btn.config(state=tk.NORMAL)
                        self.app.notebook.tab(self, text=f"◉ {self.script_name}")  # 显示运行状态

                        return
                except Exception as e:
                    logger.error(f"macOS terminal error: {str(e)}")

            elif system == 'Windows':
                temp_bat = os.path.join(os.path.dirname(self.script_path), f"temp_run_{os.getpid()}.bat")
                with open(temp_bat, 'w', encoding='utf-8') as f:
                    f.write(f'@echo off\n')
                    f.write(f'echo PID:%PPID%\n')  # 显示父进程ID
                    f.write(f'cd /d "{os.path.dirname(self.script_path)}"\n')
                    f.write(f'"{self.script_path}"\n')
                    f.write(f'echo.\n')
                    f.write(f'echo 按任意键退出...\n')
                    f.write(f'pause >nul\n')
                    f.write(f'del "%~f0"\n')

                # 启动cmd窗口
                terminal_process = subprocess.Popen(
                    ['cmd', '/c', 'start', f'"{self.script_name}"', temp_bat],
                    creationflags=subprocess.CREATE_NEW_PROCESS_GROUP
                )

                # 记录进程信息
                self.terminal_processes.append({
                    'process': terminal_process,
                    'pid': terminal_process.pid,
                    'terminal_type': 'cmd',
                    'start_time': time.time(),
                    'script_path': self.script_path,
                    'temp_file': temp_bat
                })

                self.update_console(f"\n[已在 CMD 窗口中打开脚本，进程PID: {terminal_process.pid}]\n")
                self.app.update_status(f"已在独立终端中运行 {self.script_name}")

                # === 新增：更新按钮状态 ===
                self.stop_btn.config(state=tk.NORMAL)
                self.app.notebook.tab(self, text=f"◉ {self.script_name}")  # 显示运行状态

                return

            raise Exception("无法找到合适的终端程序")

        except Exception as e:
            logger.error(f"Error opening terminal: {str(e)}")
            self.update_console(f"\n[错误: 无法在独立终端中打开脚本 - {str(e)}]\n")
            self.app.update_status(f"打开终端失败: {str(e)}")
            messagebox.showerror("错误", f"无法在独立终端中运行脚本:\n{str(e)}")

    def update_button_states(self):
        """统一的按钮状态管理方法"""
        has_running_processes = (
                self.running or  # 应用内运行
                self.background_processes or  # 后台进程
                self.terminal_processes  # 终端进程
        )

        if has_running_processes:
            self.stop_btn.config(state=tk.NORMAL)
            # 标签页显示运行状态
            if self.running:
                self.app.notebook.tab(self, text=f"● {self.script_name}")  # 应用内运行
            else:
                self.app.notebook.tab(self, text=f"◉ {self.script_name}")  # 外部运行
        else:
            self.stop_btn.config(state=tk.DISABLED)
            self.run_btn.config(state=tk.NORMAL)
            self.app.notebook.tab(self, text=self.script_name)  # 恢复正常状态

    def create_console(self):
        """创建控制台输出区域 - 确保文本可复制"""
        self.console_frame = ttk.Frame(self)
        self.console_frame.grid(row=1, column=0, sticky='nsew')
        self.console_frame.columnconfigure(0, weight=1)
        self.console_frame.rowconfigure(0, weight=1)

        # 创建文本框架确保正确布局
        text_frame = ttk.Frame(self.console_frame)
        text_frame.grid(row=0, column=0, sticky='nsew')
        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)

        # 控制台文本区域 - GitHub Dark 颜色
        console_bg = '#0d1117'
        console_fg = '#c9d1d9'
        console_select_bg = '#58a6ff'
        console_insert_bg = 'white'

        self.console = scrolledtext.ScrolledText(
            text_frame,
            state='disabled',
            wrap=tk.WORD,
            font=('DejaVu Sans Mono', 10),
            background=console_bg,
            foreground=console_fg,
            padx=12,
            pady=12,
            insertbackground=console_insert_bg,
            selectbackground=console_select_bg,
            selectforeground=console_bg,
            undo=False,
            maxundo=-1,
            blockcursor=False,
            relief='flat',
            borderwidth=0,
            exportselection=True,
        )
        self.console.grid(row=0, column=0, sticky='nsew')

        self.set_tab_width(4)
        self.create_context_menu()

        # 添加滚动条
        scrollbar = ttk.Scrollbar(text_frame, orient='vertical', command=self.console.yview)
        scrollbar.grid(row=0, column=1, sticky='ns')
        self.console.config(yscrollcommand=scrollbar.set)
        style = ttk.Style()
        style.configure("Vertical.TScrollbar", background='#161b22', troughcolor='#0d1117', bordercolor='#30363d',
                        arrowcolor='#c9d1d9')
        style.map("Vertical.TScrollbar", background=[('active', '#21262d')])

        text_frame.columnconfigure(0, weight=1)
        text_frame.rowconfigure(0, weight=1)

    def create_context_menu(self):
        """创建右键菜单以支持复制操作"""
        menu_bg = '#161b22'
        menu_fg = '#c9d1d9'
        menu_active_bg = '#58a6ff'
        menu_active_fg = '#0d1117'

        self.context_menu = tk.Menu(
            self.console,
            tearoff=0,
            background=menu_bg,
            foreground=menu_fg,
            activebackground=menu_active_bg,
            activeforeground=menu_active_fg,
            bd=0
        )
        self.context_menu.add_command(label="复制", command=self.copy_selected_text)
        self.context_menu.add_separator()
        self.context_menu.add_command(label="清空控制台", command=self.clear_console)
        self.context_menu.add_separator()
        # === 新增：后台进程管理选项 ===
        self.context_menu.add_command(label="查看后台进程", command=self.show_background_processes)
        # === 新增结束 ===

        self.console.bind("<Button-3>", self.show_context_menu)
        self.console.bind("<Control-c>", self.copy_selected_text)
        self.console.bind("<Control-C>", self.copy_selected_text)
        self.console.bind("<Button-1>", self.hide_context_menu)

    def show_context_menu(self, event):
        """显示右键菜单，菜单不要紧贴鼠标指针"""
        try:
            offset_x, offset_y = 10, 10
            self.context_menu.tk_popup(event.x_root + offset_x, event.y_root + offset_y)
        finally:
            self.context_menu.grab_release()

    def hide_context_menu(self, event=None):
        """左键点击时取消右键菜单显示"""
        try:
            self.context_menu.unpost()
        except:
            pass

    def copy_selected_text(self, event=None):
        """复制选中的文本到剪贴板"""
        try:
            if self.console.tag_ranges(tk.SEL):
                selected_text = self.console.get(tk.SEL_FIRST, tk.SEL_LAST)
                self.console.clipboard_clear()
                self.console.clipboard_append(selected_text)
                self.show_copy_confirmation()
        except tk.TclError:
            pass
        return "break"

    def show_copy_confirmation(self):
        """显示复制确认提示"""
        confirm_label = ttk.Label(
            self.console_frame,
            text="✓ 文本已复制到剪贴板",
            foreground="#3fb950",
            background="#161b22",
            padding=(8, 4),
            font=('DejaVu Sans', 8)
        )
        confirm_label.place(relx=1.0, rely=1.0, anchor="se", x=-10, y=-5)
        self.after(1500, confirm_label.destroy)

    def set_tab_width(self, num_spaces):
        """设置制表符宽度为指定空格数"""
        font_name = 'DejaVu Sans Mono'
        font_size = 10

        try:
            temp_font = tk.font.Font(family=font_name, size=font_size)
            space_width = temp_font.measure(' ')
            tab_width = space_width * num_spaces
            self.console.config(tabs=(tab_width,))
        except:
            self.console.config(tabs=60)

    @debug_trace
    def run_script(self):
        """运行脚本"""
        if not os.path.exists(self.script_path):
            messagebox.showerror("错误", f"脚本不存在: {self.script_path}")
            return

        if self.running:
            self.update_console("脚本已经在运行中...\n")
            return

        self.update_console(f"\n=== 开始执行: {self.script_name} ===\n")
        self.running = True
        self.run_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.NORMAL)
        self.app.notebook.tab(self, text=f"● {self.script_name}")

        threading.Thread(
            target=self.execute_script,
            daemon=True,
            name=f"ScriptExecutor-{self.script_name}"
        ).start()

    @debug_trace
    def execute_script(self):
        """执行脚本并捕获输出 - 修复 rosmon 段错误"""
        logger.info(f"Starting script execution for {self.script_name}")
        master_fd = None
        try:
            if platform.system() != 'Windows':
                os.chmod(self.script_path, 0o755)

            # 设置环境变量 - 移除可能导致冲突的设置
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            env['PYTHONIOENCODING'] = 'utf-8'

            # 对于 rosmon，使用更保守的环境设置
            if 'rosmon' in self.script_path.lower():
                # 不强制设置 TERM 和颜色模式，让 rosmon 自己决定
                env.pop('ROSMON_COLOR_MODE', None)
                # 使用系统默认的 TERM 设置
                if 'TERM' not in env:
                    env['TERM'] = 'xterm'
                logger.info("Using conservative settings for rosmon")
            else:
                # 非 rosmon 脚本可以使用完整的颜色支持
                env['ROSMON_COLOR_MODE'] = 'truecolor'
                env['TERM'] = 'xterm-256color'
                env['LANG'] = 'zh_CN.UTF-8'
                env['LC_ALL'] = 'zh_CN.UTF-8'

            kwargs = {
                'env': env,
                'preexec_fn': os.setsid if platform.system() != 'Windows' else None
            }

            # 暂时禁用 PTY 处理 rosmon，使用标准方法
            # 这可以避免 PTY 相关的段错误
            logger.info("Using standard execution method for all scripts")
            self.execute_script_standard()

            # 如果需要重新启用 PTY 处理，请使用以下代码替换上面的行：
            # self.execute_script_with_fallback()

            if self.process:
                return_code = self.process.poll()

                if return_code == 0:
                    self.update_console(f"\n=== {self.script_name} 执行成功 ===\n\n")
                    self.app.update_status(f"{self.script_name} 执行成功")
                else:
                    self.update_console(f"\n!!! {self.script_name} 执行失败 (返回码: {return_code}) !!!\n\n")
                    self.app.update_status(f"{self.script_name} 执行失败 (代码: {return_code})")

        except Exception as e:
            logger.error(f"Script execution error: {str(e)}")
            logger.error(traceback.format_exc())
            self.update_console(f"错误: {str(e)}\n")
            self.app.update_status(f"执行出错: {str(e)}")
        finally:
            # 清理资源
            if master_fd is not None:
                try:
                    os.close(master_fd)
                    logger.debug("Closed master FD")
                except:
                    pass

            self.running = False
            self.process = None

            # 使用线程安全的方式更新UI
            def update_ui():
                if not self._destroyed:
                    if self.run_btn.winfo_exists():
                        self.run_btn.config(state=tk.NORMAL)
                    if self.stop_btn.winfo_exists():
                        self.stop_btn.config(state=tk.DISABLED)
                    if self.winfo_exists():
                        self.app.notebook.tab(self, text=self.script_name)

            safe_gui_update(self, update_ui)

    @debug_trace
    def execute_script_with_fallback(self):
        """带回退机制的脚本执行方法"""
        logger.info(f"Attempting PTY execution for {self.script_name}")

        # 首先尝试使用标准方法执行
        try:
            self.execute_script_standard()
            return
        except Exception as e:
            logger.warning(f"Standard execution failed: {str(e)}")

        # 如果标准方法失败，且是 rosmon，尝试简化的 PTY 方法
        if 'rosmon' in self.script_path.lower() and platform.system() != 'Windows':
            try:
                logger.info("Attempting simplified PTY execution for rosmon")
                self.execute_script_simple_pty()
            except Exception as e:
                logger.error(f"PTY execution also failed: {str(e)}")
                # 最后的回退：直接使用 subprocess.run
                self.execute_script_basic()

    @debug_trace
    def execute_script_simple_pty(self):
        """简化的 PTY 执行方法（仅用于 rosmon 回退）"""
        try:
            import pty
            import select

            master_fd, slave_fd = pty.openpty()

            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'
            # 不设置特殊的 rosmon 环境变量

            self.process = subprocess.Popen(
                [self.script_path],
                stdout=slave_fd,
                stderr=slave_fd,
                stdin=slave_fd,
                text=True,
                env=env,
                preexec_fn=os.setsid
            )

            os.close(slave_fd)  # 立即关闭子进程端

            # 设置非阻塞读取，超时时间更短
            fcntl.fcntl(master_fd, fcntl.F_SETFL, os.O_NONBLOCK)

            while True:
                if self._destroyed or not self.running:
                    break

                try:
                    # 更短的超时时间
                    ready, _, _ = select.select([master_fd], [], [], 0.05)
                    if ready:
                        data = os.read(master_fd, 1024).decode('utf-8', errors='replace')
                        if data:
                            # 使用普通的控制台更新，不解析 ANSI
                            self.update_console(data)

                    # 检查进程状态
                    if self.process.poll() is not None:
                        break

                except OSError as e:
                    if e.errno != 11:  # 不是 EAGAIN
                        logger.warning(f"PTY read error: {str(e)}")
                        break
                except Exception as e:
                    logger.error(f"Unexpected error in PTY loop: {str(e)}")
                    break

        finally:
            if 'master_fd' in locals():
                try:
                    os.close(master_fd)
                except:
                    pass

    @debug_trace
    def execute_script_basic(self):
        """最基本的脚本执行方法（最后的回退）"""
        logger.info("Using basic execution method as final fallback")

        try:
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'

            # 使用最简单的方式执行
            result = subprocess.run(
                [self.script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                env=env,
                timeout=300  # 5分钟超时
            )

            # 一次性输出所有结果
            if result.stdout:
                self.update_console(result.stdout)

            # 设置虚拟的 process 对象用于返回码检查
            class MockProcess:
                def __init__(self, returncode):
                    self.returncode = returncode

                def poll(self):
                    return self.returncode

            self.process = MockProcess(result.returncode)

        except subprocess.TimeoutExpired:
            self.update_console("\n[脚本执行超时]\n")
            self.process = MockProcess(124)  # 超时返回码
        except Exception as e:
            self.update_console(f"\n[执行失败: {str(e)}]\n")
            self.process = MockProcess(1)  # 错误返回码

    @debug_trace
    def execute_script_standard(self):
        """标准脚本执行方法（用作回退）"""
        logger.info(f"Using standard execution method for {self.script_name}")
        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'

        self.process = subprocess.Popen(
            [self.script_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=0,  # 无缓冲
            shell=True,
            encoding='utf-8',
            errors='replace',
            env=env,
            preexec_fn=os.setsid if platform.system() != 'Windows' else None
        )

        # 使用非阻塞读取
        if platform.system() != 'Windows':
            fcntl.fcntl(self.process.stdout.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)

        output_buffer = ""
        while True:
            if self._destroyed:
                break

            try:
                if platform.system() != 'Windows':
                    ready, _, _ = select.select([self.process.stdout], [], [], 0.1)
                    if ready:
                        char = self.process.stdout.read(1)
                        if char:
                            output_buffer += char
                            # 遇到换行符或缓冲区满时输出
                            if char == '\n' or len(output_buffer) > 1024:
                                self.update_console_with_color(output_buffer)
                                output_buffer = ""
                else:
                    # Windows 环境的处理
                    line = self.process.stdout.readline()
                    if line:
                        self.update_console_with_color(line)

            except (IOError, OSError) as e:
                if e.errno == 11:  # EAGAIN
                    pass
                else:
                    logger.error(f"IO error in standard execution: {str(e)}")

            # 检查进程是否结束
            if self.process.poll() is not None:
                # 输出剩余缓冲区内容
                if output_buffer:
                    self.update_console_with_color(output_buffer)
                # 读取任何剩余输出
                try:
                    remaining = self.process.stdout.read()
                    if remaining:
                        self.update_console_with_color(remaining)
                except:
                    pass
                break

            # 检查是否被要求停止
            if not self.running:
                break

    def remove_ansi_escape(self, text):
        """移除ANSI转义序列"""
        ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        return ansi_escape.sub('', text)

    def kill_processes_by_script_name(self):
        """通过脚本名称查找并终止相关进程"""
        try:
            import psutil
            script_name = os.path.basename(self.script_path)
            killed_processes = []

            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    # 检查进程命令行是否包含脚本路径
                    cmdline = proc.info.get('cmdline', [])
                    if cmdline and any(self.script_path in arg for arg in cmdline):
                        proc.kill()
                        killed_processes.append(f"脚本进程 (PID: {proc.info['pid']})")
                        logger.info(f"Killed script process by name: {proc.info['pid']}")
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            if killed_processes:
                self.update_console(f"通过进程名查找并停止:\n")
                for proc in killed_processes:
                    self.update_console(f"  ✓ {proc}\n")

        except ImportError:
            # 如果没有psutil，使用系统命令
            try:
                script_name = os.path.basename(self.script_path)
                if platform.system() != 'Windows':
                    # Unix系统使用pkill
                    subprocess.run(['pkill', '-f', self.script_path], capture_output=True)
                    self.update_console(f"  ✓ 使用pkill停止相关进程\n")
            except Exception as e:
                logger.debug(f"Could not kill processes by name: {str(e)}")

    @debug_trace
    def stop_script(self):
        """停止正在运行的脚本 - 包括应用内、终端和后台进程"""
        logger.info(f"Stopping all processes for script {self.script_name}")

        stopped_processes = []
        failed_processes = []

        try:
            # 1. 停止应用内运行的脚本
            if self.running and self.process:
                try:
                    pid = self.process.pid
                    if platform.system() == 'Windows':
                        subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)],
                                       capture_output=True)
                    else:
                        os.killpg(os.getpgid(pid), signal.SIGTERM)
                    stopped_processes.append(f"应用内脚本 (PID: {pid})")
                    logger.info(f"Stopped internal script process: {pid}")
                except Exception as e:
                    failed_processes.append(f"应用内脚本: {str(e)}")
                    logger.error(f"Failed to stop internal script: {str(e)}")
                finally:
                    self.running = False
                    self.process = None

            # 2. 停止后台进程
            background_count = len(self.background_processes)
            for proc_info in self.background_processes[:]:  # 使用切片避免修改正在迭代的列表
                try:
                    process = proc_info['process']
                    pid = proc_info['pid']

                    if process.poll() is None:  # 进程仍在运行
                        if platform.system() == 'Windows':
                            subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)],
                                           capture_output=True)
                        else:
                            os.killpg(os.getpgid(pid), signal.SIGTERM)

                        stopped_processes.append(f"后台进程 (PID: {pid})")
                        logger.info(f"Stopped background process: {pid}")

                    # 从列表中移除
                    self.background_processes.remove(proc_info)

                except Exception as e:
                    failed_processes.append(f"后台进程 {proc_info.get('pid', 'unknown')}: {str(e)}")
                    logger.error(f"Failed to stop background process: {str(e)}")

            # 3. 停止终端进程
            terminal_count = len(self.terminal_processes)
            for term_info in self.terminal_processes[:]:
                try:
                    if platform.system() == 'Darwin' and 'tab_id' in term_info:
                        # macOS: 关闭Terminal.app标签页
                        apple_script = f'''
                        tell application "Terminal"
                            close tab id {term_info['tab_id']}
                        end tell
                        '''
                        subprocess.run(['osascript', '-e', apple_script],
                                       capture_output=True)
                        stopped_processes.append(f"终端标签页 (ID: {term_info['tab_id']})")

                    elif term_info.get('process'):
                        # Linux/Windows: 终止进程组
                        process = term_info['process']
                        pid = term_info['pid']

                        if process.poll() is None:  # 进程仍在运行
                            if platform.system() == 'Windows':
                                subprocess.run(["taskkill", "/F", "/T", "/PID", str(pid)],
                                               capture_output=True)
                            else:
                                # 终止整个进程组，包括终端和其中运行的脚本
                                try:
                                    os.killpg(os.getpgid(pid), signal.SIGTERM)
                                    time.sleep(0.5)  # 给进程一些时间优雅退出
                                    if process.poll() is None:
                                        os.killpg(os.getpgid(pid), signal.SIGKILL)  # 强制终止
                                except ProcessLookupError:
                                    pass  # 进程已经不存在

                            stopped_processes.append(f"终端进程 (PID: {pid})")

                    # 清理临时文件（Windows）
                    if 'temp_file' in term_info:
                        try:
                            if os.path.exists(term_info['temp_file']):
                                os.remove(term_info['temp_file'])
                        except:
                            pass

                    # 从列表中移除
                    self.terminal_processes.remove(term_info)
                    logger.info(f"Stopped terminal process: {term_info.get('pid', 'N/A')}")

                except Exception as e:
                    failed_processes.append(f"终端进程 {term_info.get('pid', 'unknown')}: {str(e)}")
                    logger.error(f"Failed to stop terminal process: {str(e)}")

            # 4. 尝试通过进程名查找并终止相关脚本进程
            self.kill_processes_by_script_name()

            # 5. 更新UI状态
            def update_ui():
                if not self._destroyed:
                    if self.run_btn.winfo_exists():
                        self.run_btn.config(state=tk.NORMAL)
                    if self.stop_btn.winfo_exists():
                        self.stop_btn.config(state=tk.DISABLED)
                    if self.winfo_exists():
                        self.app.notebook.tab(self, text=self.script_name)

            safe_gui_update(self, update_ui)

            # 6. 显示停止结果
            total_stopped = len(stopped_processes)
            result_msg = f"\n=== 停止脚本完成 ===\n"

            if stopped_processes:
                result_msg += f"成功停止 {total_stopped} 个进程:\n"
                for proc in stopped_processes:
                    result_msg += f"  ✓ {proc}\n"

            if failed_processes:
                result_msg += f"\n停止失败 {len(failed_processes)} 个进程:\n"
                for proc in failed_processes:
                    result_msg += f"  ✗ {proc}\n"

            if not stopped_processes and not failed_processes:
                result_msg += "没有发现运行中的进程\n"

            result_msg += f"===================\n\n"

            self.update_console(result_msg)

            # 更新状态栏
            if total_stopped > 0:
                self.app.update_status(f"{self.script_name} 已停止 ({total_stopped} 个进程)")
            else:
                self.app.update_status(f"{self.script_name} 已停止")

        except Exception as e:
            logger.error(f"Error in stop_script: {str(e)}")
            self.update_console(f"\n[停止脚本时发生错误: {str(e)}]\n\n")
            self.app.update_status(f"停止脚本失败: {str(e)}")

    def update_console(self, text):
        """将输出放入缓冲区，由定时器批量刷新到控件"""
        if text and not self._destroyed:
            self.output_buffer.put(text)

    def flush_console_buffer(self):
        """定时批量刷新输出到控件，支持颜色"""
        if self._destroyed:
            return

        if not self._flushing:
            self._flushing = True
            lines = []
            try:
                for _ in range(self.max_flush_lines):
                    lines.append(self.output_buffer.get_nowait())
            except queue.Empty:
                pass

            if lines:
                def update_impl():
                    if self._destroyed or not self.console.winfo_exists():
                        return

                    self.console.config(state='normal')
                    for text in lines:
                        # 解析并显示带颜色的文本
                        segments = self.parse_ansi_text(text)
                        for segment_text, tags in segments:
                            if self.timestamps.get() and segment_text.strip():
                                timestamp = f"[{strftime('%H:%M:%S')}] "
                                self.console.insert(tk.END, timestamp)

                            if tags:
                                self.console.insert(tk.END, segment_text, tags)
                            else:
                                self.console.insert(tk.END, segment_text)

                        if self.autoscroll.get():
                            self.console.see(tk.END)

                    self.console.config(state='disabled')
                    self.console.update_idletasks()

                safe_gui_update(self, update_impl)

            self._flushing = False

        if not self._destroyed:
            self.after(self.flush_interval, self.flush_console_buffer)

    def clear_console(self):
        """清空控制台"""

        def update_impl():
            self.console.config(state='normal')
            self.console.delete(1.0, tk.END)
            self.console.config(state='disabled')

        safe_gui_update(self, update_impl)
        self.update_console(f"控制台已清空\n")

    def export_log(self):
        """导出控制台内容到文件"""
        log_content = self.console.get(1.0, tk.END)
        if not log_content.strip():
            messagebox.showinfo("信息", "控制台为空，无需导出。")
            return

        try:
            default_filename = f"{self.script_name}_{strftime('%Y%m%d_%H%M%S')}.log"
            filepath = filedialog.asksaveasfilename(
                initialfile=default_filename,
                defaultextension=".log",
                filetypes=[("Log files", "*.log"), ("Text files", "*.txt"), ("All files", "*.*")]
            )
            if filepath:
                with open(filepath, 'w', encoding='utf-8') as f:
                    f.write(log_content)
                self.app.update_status(f"日志已导出到 {os.path.basename(filepath)}")
        except Exception as e:
            messagebox.showerror("导出失败", f"无法保存日志文件：\n{e}")

    def close_this_tab(self):
        """请求主应用关闭此标签页"""
        self.app.close_tab(self)


class ScriptRunnerApp:
    def __init__(self, master):
        logger.info("Initializing ScriptRunnerApp")
        self.master = master
        master.title("脚本运行器 - 调试版本")
        master.geometry("1000x700")
        master.minsize(900, 600)

        self.performance_mode = False
        self.set_window_icon()
        self.set_theme()

        self.main_frame = ttk.Frame(master)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=15, pady=15)

        self.create_header()
        self.create_script_panel()
        self.create_console_tabs()
        self.create_status_bar()

        self.scripts = []
        self.tabs = []

        self.load_scripts()
        self.update_time()
        self.setup_exit_handler()

        # 添加调试信息显示
        self.show_debug_info()

    def show_debug_info(self):
        """显示调试信息"""
        debug_info = f"""
调试模式已启用
日志文件: {LOG_FILE}
Python版本: {sys.version}
平台: {platform.platform()}
进程ID: {os.getpid()}
线程: {threading.current_thread().name}
"""
        logger.info(debug_info)
        self.update_status(f"调试模式 - 日志: {os.path.basename(LOG_FILE)}")

    def setup_exit_handler(self):
        """设置退出处理程序"""
        self.master.protocol("WM_DELETE_WINDOW", self.cleanup_and_exit)
        if platform.system() != 'Windows':
            # 暂时禁用信号处理器，看是否是崩溃原因
            # signal.signal(signal.SIGINT, self.signal_handler)
            # signal.signal(signal.SIGTERM, self.signal_handler)
            pass

    def signal_handler(self, signum, frame):
        """处理终止信号"""
        logger.info(f"Received signal {signum}")
        self.cleanup_and_exit()

    @debug_trace
    def cleanup_and_exit(self):
        """清理资源并退出应用"""
        logger.info("Starting cleanup and exit")
        for tab in self.tabs:
            if tab.running:
                self.update_status(f"正在停止 {tab.script_name}...")
                tab.stop_script()
        logger.info("Destroying main window")
        self.master.destroy()
        logger.info("Exiting application")
        sys.exit(0)

    def set_window_icon(self):
        """尝试设置窗口图标"""
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            icon_path = os.path.join(script_dir, "icons", "terminal.png")
            if os.path.exists(icon_path):
                self.master.iconphoto(True, tk.PhotoImage(file=icon_path))
        except Exception as e:
            logger.warning(f"无法设置图标: {str(e)}")

    def set_theme(self):
        """设置 GitHub Dark 主题"""
        bg_color = '#0d1117'
        frame_bg_color = '#161b22'
        text_color = '#c9d1d9'
        border_color = '#30363d'
        accent_color = '#58a6ff'
        button_bg = '#21262d'
        button_hover_bg = '#30363d'
        header_fg = '#f0f6fc'

        style = ttk.Style()
        style.theme_use('clam')

        style.configure('.',
                        background=bg_color,
                        foreground=text_color,
                        font=('DejaVu Sans', 9))

        self.master.configure(background=bg_color)
        style.configure('TFrame', background=bg_color)
        style.configure('TLabelframe',
                        background=bg_color,
                        foreground=text_color,
                        bordercolor=border_color)
        style.configure('TLabelframe.Label',
                        background=bg_color,
                        foreground=text_color,
                        font=('DejaVu Sans', 10, 'bold'))

        style.configure('Header.TFrame', background=frame_bg_color)
        style.configure('Header.TLabel',
                        background=frame_bg_color,
                        foreground=header_fg,
                        font=('DejaVu Sans', 11, 'bold'))

        style.configure('StatusBar.TLabel',
                        background=frame_bg_color,
                        foreground=text_color,
                        font=('DejaVu Sans Mono', 9))

        style.configure('TButton',
                        background=button_bg,
                        foreground=text_color,
                        bordercolor=border_color,
                        padding=(8, 5),
                        font=('DejaVu Sans', 18),
                        relief='flat')
        style.map('TButton',
                  background=[('active', button_hover_bg), ('pressed', accent_color)],
                  foreground=[('pressed', bg_color)])

        style.configure('TCheckbutton',
                        background=bg_color,
                        foreground=text_color)
        style.map('TCheckbutton',
                  background=[('active', bg_color)],
                  indicatorbackground=[('selected', accent_color), ('active', button_hover_bg)],
                  indicatorforeground=[('selected', bg_color)])

        style.configure('TNotebook',
                        background=bg_color,
                        bordercolor=border_color)
        style.configure('TNotebook.Tab',
                        background=button_bg,
                        foreground=text_color,
                        padding=(10, 5),
                        font=('DejaVu Sans', 9))
        style.map('TNotebook.Tab',
                  background=[('selected', bg_color)],
                  foreground=[('selected', accent_color)],
                  expand=[('selected', [1, 1, 1, 0])])

    def create_header(self):
        """创建简化标题栏"""
        header_frame = ttk.Frame(self.main_frame, style='Header.TFrame')
        header_frame.pack(fill=tk.X, pady=(0, 10))

        title_label = ttk.Label(
            header_frame,
            text="Shell 脚本监视器 - 调试版本",
            style='Header.TLabel'
        )
        title_label.pack(side=tk.LEFT, padx=10, pady=8)

        button_frame = ttk.Frame(header_frame, style='Header.TFrame')
        button_frame.pack(side=tk.RIGHT, padx=10, pady=4)

        # 添加查看日志按钮
        log_btn = ttk.Button(
            button_frame,
            text="查看日志",
            command=self.open_log_file,
            width=10
        )
        log_btn.pack(side=tk.RIGHT, padx=5)

        manage_btn = ttk.Button(
            button_frame,
            text="管理脚本",
            command=self.open_script_manager,
            width=12
        )
        manage_btn.pack(side=tk.RIGHT, padx=5)

        exit_btn = ttk.Button(
            button_frame,
            text="退出",
            command=self.cleanup_and_exit,
            width=8
        )
        exit_btn.pack(side=tk.RIGHT, padx=5)

        refresh_btn = ttk.Button(
            button_frame,
            text="刷新脚本",
            command=self.refresh_scripts,
            width=10
        )
        refresh_btn.pack(side=tk.RIGHT, padx=5)

    def open_log_file(self):
        """打开日志文件"""
        try:
            if platform.system() == 'Windows':
                os.startfile(LOG_FILE)
            elif platform.system() == 'Darwin':  # macOS
                subprocess.call(['open', LOG_FILE])
            else:  # Linux
                subprocess.call(['xdg-open', LOG_FILE])
        except Exception as e:
            messagebox.showerror("错误", f"无法打开日志文件: {str(e)}")

    def create_script_panel(self):
        """创建脚本选择面板（可折叠/展开）"""
        self.scripts_collapsed = False
        self.script_panel = ttk.LabelFrame(
            self.main_frame,
            text="脚本列表",
            padding=(10, 8, 10, 10)
        )
        self.script_panel.pack(fill=tk.X, pady=(0, 10))

        # 标题栏和折叠按钮
        title_frame = ttk.Frame(self.script_panel)
        title_frame.pack(fill=tk.X, side=tk.TOP, anchor='w')
        title_label = ttk.Label(title_frame, text="脚本列表", font=('DejaVu Sans', 10, 'bold'))
        title_label.pack(side=tk.LEFT)
        self.collapse_btn = ttk.Button(
            title_frame,
            text="▲",
            width=2,
            command=self.toggle_scripts_panel
        )
        self.collapse_btn.pack(side=tk.LEFT, padx=(8, 0))

        self.btn_container = ttk.Frame(self.script_panel)
        self.btn_container.pack(fill=tk.BOTH, expand=True)

        for i in range(4):
            self.btn_container.columnconfigure(i, weight=1)
        for i in range(2):
            self.btn_container.rowconfigure(i, weight=1)

    def toggle_scripts_panel(self):
        """折叠或展开脚本列表区域"""
        if self.scripts_collapsed:
            self.btn_container.pack(fill=tk.BOTH, expand=True)
            self.collapse_btn.config(text="▲")
            self.scripts_collapsed = False
        else:
            self.btn_container.pack_forget()
            self.collapse_btn.config(text="▼")
            self.scripts_collapsed = True

    def create_console_tabs(self):
        """创建控制台标签页容器"""
        tabs_frame = ttk.LabelFrame(
            self.main_frame,
            text="脚本控制台",
            padding=(8, 4, 8, 8)
        )
        tabs_frame.pack(fill=tk.BOTH, expand=True)
        tabs_frame.columnconfigure(0, weight=1)
        tabs_frame.rowconfigure(0, weight=1)

        self.notebook = ttk.Notebook(tabs_frame)
        self.notebook.grid(row=0, column=0, sticky='nsew')

        tabs_frame.columnconfigure(0, weight=1)
        tabs_frame.rowconfigure(0, weight=1)
        self.create_welcome_tab()

    @debug_trace
    def create_welcome_tab(self):
        """创建欢迎标签页"""
        self.welcome_frame = ttk.Frame(self.notebook, padding=15)
        self.notebook.add(self.welcome_frame, text="欢迎")

        welcome_label = ttk.Label(
            self.welcome_frame,
            text="脚本运行器 - 调试版本",
            font=('DejaVu Sans', 16, 'bold'),
        )
        welcome_label.pack(pady=(10, 5), anchor='center')

        desc_text = (
            f"调试模式已启用\n"
            f"日志文件: {LOG_FILE}\n\n"
            "by HM_ZC\n\n"
        )
        desc_label = ttk.Label(
            self.welcome_frame,
            text=desc_text,
            font=('DejaVu Sans', 10),
            justify=tk.CENTER
        )
        desc_label.pack(pady=8, padx=10, anchor='center')

        # 系统静态信息
        sys_info = (
            f"系统: {platform.system()} {platform.release()}\n"
            f"Python版本: {platform.python_version()}\n"
            f"Python路径: {sys.executable}\n"
            f"CPU核心数: {psutil.cpu_count(logical=True)} (物理: {psutil.cpu_count(logical=False)})\n"
            f"内存总量: {round(psutil.virtual_memory().total / (1024 ** 3), 2)} GB\n"
            f"磁盘总量: {round(psutil.disk_usage('/').total / (1024 ** 3), 2)} GB\n"
            f"主机名: {platform.node()}\n"
            f"当前用户: {os.getlogin()}\n"
        )
        sys_label = ttk.Label(
            self.welcome_frame,
            text=sys_info,
            font=('DejaVu Sans Mono', 9),
            justify=tk.CENTER
        )
        sys_label.pack(pady=(15, 5), anchor='center')

        # 图形化资源区
        self.resource_frame = ttk.LabelFrame(self.welcome_frame, text="实时系统资源占用", padding=18)
        self.resource_frame.pack(pady=(10, 5), padx=5, anchor='center')
        self.resource_frame.columnconfigure(0, weight=1)
        self.resource_frame.columnconfigure(1, weight=1)
        self.resource_frame.columnconfigure(2, weight=1)
        self.resource_frame.columnconfigure(3, weight=1)

        # 白色背景的黑色进度条样式
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Black.Horizontal.TProgressbar',
                        troughcolor='#fff',
                        background='#111',
                        bordercolor='#333',
                        lightcolor='#111',
                        darkcolor='#111',
                        thickness=18)
        style.configure('BlackSmall.Horizontal.TProgressbar',
                        troughcolor='#fff',
                        background='#111',
                        bordercolor='#333',
                        lightcolor='#111',
                        darkcolor='#111',
                        thickness=10)

        # 进度条和数值标签
        self.cpu_bar = ttk.Progressbar(self.resource_frame, length=220, maximum=100,
                                       style='Black.Horizontal.TProgressbar')
        self.cpu_bar.grid(row=0, column=0, padx=10, pady=6)
        self.cpu_label = ttk.Label(self.resource_frame, text="CPU", font=('DejaVu Sans Mono', 10, 'bold'))
        self.cpu_label.grid(row=0, column=1, sticky='w')
        self.cpu_val = ttk.Label(self.resource_frame, text="", font=('DejaVu Sans Mono', 10))
        self.cpu_val.grid(row=0, column=2, sticky='w')

        self.mem_bar = ttk.Progressbar(self.resource_frame, length=220, maximum=100,
                                       style='Black.Horizontal.TProgressbar')
        self.mem_bar.grid(row=1, column=0, padx=10, pady=6)
        self.mem_label = ttk.Label(self.resource_frame, text="内存", font=('DejaVu Sans Mono', 10, 'bold'))
        self.mem_label.grid(row=1, column=1, sticky='w')
        self.mem_val = ttk.Label(self.resource_frame, text="", font=('DejaVu Sans Mono', 10))
        self.mem_val.grid(row=1, column=2, sticky='w')

        self.swap_bar = ttk.Progressbar(self.resource_frame, length=220, maximum=100,
                                        style='Black.Horizontal.TProgressbar')
        self.swap_bar.grid(row=2, column=0, padx=10, pady=6)
        self.swap_label = ttk.Label(self.resource_frame, text="Swap", font=('DejaVu Sans Mono', 10, 'bold'))
        self.swap_label.grid(row=2, column=1, sticky='w')
        self.swap_val = ttk.Label(self.resource_frame, text="", font=('DejaVu Sans Mono', 10))
        self.swap_val.grid(row=2, column=2, sticky='w')

        self.disk_bar = ttk.Progressbar(self.resource_frame, length=220, maximum=100,
                                        style='Black.Horizontal.TProgressbar')
        self.disk_bar.grid(row=3, column=0, padx=10, pady=6)
        self.disk_label = ttk.Label(self.resource_frame, text="磁盘", font=('DejaVu Sans Mono', 10, 'bold'))
        self.disk_label.grid(row=3, column=1, sticky='w')
        self.disk_val = ttk.Label(self.resource_frame, text="", font=('DejaVu Sans Mono', 10))
        self.disk_val.grid(row=3, column=2, sticky='w')

        # CPU每核进度条区
        self.core_bars = []
        self.core_vals = []
        self.core_labels = []
        self.core_frame = ttk.Frame(self.resource_frame)
        self.core_frame.grid(row=4, column=0, columnspan=4, pady=(8, 2), sticky='nsew')

        cpu_count = psutil.cpu_count(logical=True)
        for i in range(cpu_count):
            bar = ttk.Progressbar(self.core_frame, length=80, maximum=100, style='BlackSmall.Horizontal.TProgressbar')
            bar.grid(row=0, column=i, padx=4)
            self.core_bars.append(bar)
            val = ttk.Label(self.core_frame, text="", font=('DejaVu Sans Mono', 8))
            val.grid(row=1, column=i)
            self.core_vals.append(val)
            label = ttk.Label(self.core_frame, text=f"核{i + 1}", font=('DejaVu Sans Mono', 8))
            label.grid(row=2, column=i)
            self.core_labels.append(label)

        # 有线网卡流量区
        self.eth_label = ttk.Label(self.resource_frame, text="", font=('DejaVu Sans Mono', 12, 'bold'),
                                   foreground="#58a6ff", anchor='center', justify=tk.CENTER)
        self.eth_label.grid(row=5, column=0, columnspan=4, pady=(16, 2), sticky='nsew')

        # GPU占用区
        self.gpu_label = ttk.Label(self.resource_frame, text="", font=('DejaVu Sans Mono', 11, 'bold'),
                                   foreground="#e67e22", anchor='center', justify=tk.CENTER)
        self.gpu_label.grid(row=6, column=0, columnspan=4, pady=(12, 2), sticky='nsew')

        psutil.cpu_percent(interval=None, percpu=True)  # 预热
        self.update_resource_info()

    @debug_trace
    def update_resource_info(self):
        """定时刷新系统资源占用信息（图形化美化显示+GPU）"""
        try:
            cpu_total = psutil.cpu_percent(interval=None)
            cpu_percents = psutil.cpu_percent(interval=None, percpu=True)
            mem = psutil.virtual_memory()
            swap = psutil.swap_memory()
            disk = psutil.disk_usage('/')

            # 只显示有线网卡流量和速率
            eth_info = ""
            try:
                netifs = psutil.net_if_stats()
                netio = psutil.net_io_counters(pernic=True)
                for name in netifs:
                    if (name.startswith('eth') or name.startswith('enp')) and netifs[name].isup:
                        eth = netio.get(name)
                        if eth:
                            if not hasattr(self, '_last_eth_bytes'):
                                self._last_eth_bytes = {}
                                self._last_eth_time = {}
                            now = time.time()
                            last_bytes = self._last_eth_bytes.get(name, (eth.bytes_recv, eth.bytes_sent))
                            last_time = self._last_eth_time.get(name, now)
                            interval = max(now - last_time, 1e-3)
                            rx_speed = (eth.bytes_recv - last_bytes[0]) / interval / 1024  # KB/s
                            tx_speed = (eth.bytes_sent - last_bytes[1]) / interval / 1024
                            eth_info += (f"[有线网卡:{name}] 收:{eth.bytes_recv // (1024 ** 2)}MB({rx_speed:.1f}KB/s) "
                                         f"发:{eth.bytes_sent // (1024 ** 2)}MB({tx_speed:.1f}KB/s)  ")
                            self._last_eth_bytes[name] = (eth.bytes_recv, eth.bytes_sent)
                            self._last_eth_time[name] = now
            except Exception as e:
                logger.warning(f"Error getting network info: {str(e)}")

            # 更新进度条和数值
            self.cpu_bar['value'] = cpu_total
            self.cpu_val.config(text=f"{cpu_total:.1f}%")
            self.mem_bar['value'] = mem.percent
            self.mem_val.config(text=f"{mem.percent:.1f}%  {mem.used // (1024 ** 2)}MB/{mem.total // (1024 ** 2)}MB")
            self.swap_bar['value'] = swap.percent
            self.swap_val.config(
                text=f"{swap.percent:.1f}%  {swap.used // (1024 ** 2)}MB/{swap.total // (1024 ** 2)}MB")
            self.disk_bar['value'] = disk.percent
            self.disk_val.config(text=f"{disk.percent:.1f}%  {disk.used // (1024 ** 3)}G/{disk.total // (1024 ** 3)}G")

            # 每核进度条
            for i, p in enumerate(cpu_percents):
                if i < len(self.core_bars):
                    self.core_bars[i]['value'] = p
                    self.core_vals[i].config(text=f"{p:.1f}%")

            self.eth_label.config(text=eth_info if eth_info else "无有线网卡连接")

            # GPU占用
            gpu_text = self.get_gpu_usage_text()
            self.gpu_label.config(text=gpu_text)

        except Exception as e:
            logger.error(f"Error updating resource info: {str(e)}")

        self.master.after(1000, self.update_resource_info)

    def get_gpu_usage_text(self):
        """获取GPU占用信息（优先NVIDIA, 其次AMD, 再次Intel, 否则提示）"""
        # NVIDIA
        try:
            import subprocess
            result = subprocess.run(
                ['nvidia-smi', '--query-gpu=utilization.gpu,memory.used,memory.total', '--format=csv,noheader,nounits'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                if lines:
                    util, mem_used, mem_total = lines[0].split(',')
                    return f"[NVIDIA GPU] 占用:{util.strip()}%  显存:{mem_used.strip()}MB/{mem_total.strip()}MB"
        except Exception:
            pass

        # AMD GPU (rocm-smi or radeontop)
        try:
            import subprocess
            # rocm-smi
            result = subprocess.run(['rocm-smi', '--showuse'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=1)
            if result.returncode == 0 and 'GPU use' in result.stdout:
                import re
                m = re.search(r'GPU use:\s*([0-9]+)%', result.stdout)
                if m:
                    return f"[AMD GPU] 占用:{m.group(1)}%"
            # radeontop (需root)
            result = subprocess.run(['bash', '-c', "timeout 0.5s radeontop -d - -l 1 | grep 'gpu ' | head -n1"],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1)
            if result.returncode == 0 and 'gpu ' in result.stdout:
                import re
                m = re.search(r'gpu\s*([0-9.]+)%', result.stdout)
                if m:
                    return f"[AMD GPU] 占用:{m.group(1)}%"
        except Exception:
            pass

        # Intel GPU (需安装 intel-gpu-tools)
        try:
            import subprocess
            result = subprocess.run(
                ['bash', '-c', 'timeout 0.5s intel_gpu_top -J -s 1000 | grep "gpu busy" | head -n1'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=1)
            if result.returncode == 0 and 'gpu busy' in result.stdout:
                import re
                m = re.search(r'gpu busy":\s*([0-9.]+)', result.stdout)
                if m:
                    return f"[Intel GPU] 占用:{m.group(1)}%"
        except Exception:
            pass

        return "无GPU或不支持/未安装驱动"

    def create_status_bar(self):
        """创建状态栏"""
        status_bar = ttk.Frame(self.main_frame)
        status_bar.pack(fill=tk.X, pady=(8, 0))

        self.status_var = tk.StringVar(value="就绪 - 等待操作")
        status_label = ttk.Label(
            status_bar,
            textvariable=self.status_var,
            style='StatusBar.TLabel',
            font=('DejaVu Sans Mono', 9),
            padding=(8, 4, 8, 4),
            anchor=tk.W
        )
        status_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

        self.time_var = tk.StringVar()
        time_label = ttk.Label(
            status_bar,
            textvariable=self.time_var,
            style='StatusBar.TLabel',
            font=('DejaVu Sans Mono', 9),
            padding=(8, 4, 8, 4),
            anchor=tk.E
        )
        time_label.pack(side=tk.RIGHT)

    def get_config_path(self):
        """获取配置文件路径"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(script_dir, "scripts.json")

    @debug_trace
    def load_scripts(self):
        """从 JSON 文件加载脚本配置"""
        try:
            config_path = self.get_config_path()
            if not os.path.exists(config_path):
                self.scripts = []
                with open(config_path, 'w', encoding='utf-8') as f:
                    json.dump([], f, indent=4)
                self.update_status("未找到 scripts.json，已创建新文件")
                self.create_script_buttons()
                return

            with open(config_path, 'r', encoding='utf-8') as f:
                self.scripts = json.load(f)

            script_dir = os.path.dirname(os.path.abspath(__file__))
            for script_config in self.scripts:
                if not os.path.isabs(script_config['script']):
                    if not script_config['script'].startswith('scripts/'):
                        scripts_dir_path = os.path.join(script_dir, "scripts")
                        script_config['path'] = os.path.join(scripts_dir_path, script_config['script'])
                    else:
                        script_config['path'] = os.path.join(script_dir, script_config['script'])
                else:
                    script_config['path'] = script_config['script']

            self.update_status(f"成功加载 {len(self.scripts)} 个脚本")
            self.create_script_buttons()

        except json.JSONDecodeError:
            self.update_status("错误: scripts.json 文件格式无效")
            messagebox.showerror("配置错误", "scripts.json 文件格式无效。\n请检查文件内容是否为合法的 JSON。")
            self.scripts = []
        except Exception as e:
            logger.error(f"Error loading scripts: {str(e)}")
            self.update_status(f"加载脚本时发生未知错误: {e}")
            messagebox.showerror("加载错误", f"加载脚本时发生未知错误:\n{e}")
            self.scripts = []
            self.create_script_buttons()

    def get_or_create_button_style(self, color):
        """获取或创建带颜色的按钮样式"""
        style_name = f"{color.replace('#', '')}.TButton"
        style = ttk.Style()
        style.configure(style_name, foreground=color)
        return style_name

    def refresh_scripts(self):
        """刷新脚本列表"""
        self.update_status("正在刷新脚本列表...")
        self.load_scripts()
        self.update_status("脚本列表已刷新")

    def open_script_manager(self):
        """打开脚本管理窗口"""
        ScriptManager(self.master, self)

    def save_scripts(self):
        """保存脚本配置到 JSON 文件"""
        try:
            config_path = self.get_config_path()
            scripts_to_save = []
            for script in self.scripts:
                script_copy = script.copy()
                script_copy.pop('path', None)
                scripts_to_save.append(script_copy)

            with open(config_path, 'w', encoding='utf-8') as f:
                json.dump(scripts_to_save, f, indent=4, ensure_ascii=False)

            self.update_status("脚本配置已保存")
            self.refresh_scripts()
        except Exception as e:
            logger.error(f"Error saving scripts: {str(e)}")
            self.update_status(f"保存配置失败: {e}")
            messagebox.showerror("保存失败", f"无法保存脚本配置文件：\n{e}")

    def create_script_buttons(self):
        """创建脚本按钮"""
        for widget in self.btn_container.winfo_children():
            widget.destroy()

        row, col = 0, 0
        for script_config in self.scripts:
            style_name = 'TButton'
            if 'color' in script_config and script_config['color']:
                style_name = self.get_or_create_button_style(script_config['color'])

            btn = ttk.Button(
                self.btn_container,
                text=script_config['name'],
                command=lambda sc=script_config: self.open_script_console(sc['name'], sc['path']),
                width=18,
                style=style_name
            )
            btn.grid(row=row, column=col, padx=5, pady=5, sticky='nsew')

            # === 新增：为按钮添加右键菜单 ===
            # 创建右键菜单
            context_menu = tk.Menu(btn, tearoff=0,
                                   background='#161b22',
                                   foreground='#c9d1d9',
                                   activebackground='#58a6ff',
                                   activeforeground='#0d1117')
            context_menu.add_command(
                label="在应用内运行",
                command=lambda sc=script_config: self.open_script_console(sc['name'], sc['path'])
            )
            context_menu.add_separator()
            context_menu.add_command(
                label="在独立终端运行",
                command=lambda sc=script_config: self.run_script_in_terminal(sc['name'], sc['path'])
            )

            # 绑定右键事件
            def show_context_menu(event, menu=context_menu):
                try:
                    menu.tk_popup(event.x_root + 5, event.y_root + 5)
                finally:
                    menu.grab_release()

            btn.bind("<Button-3>", show_context_menu)
            # === 新增结束 ===

            col += 1
            if col >= 4:
                col = 0
                row += 1

    @debug_trace
    def run_script_in_terminal(self, script_name, script_path):
        """直接在独立终端中运行脚本（从脚本列表）"""
        if not os.path.exists(script_path):
            messagebox.showerror("错误", f"脚本不存在: {script_path}")
            return

        logger.info(f"Opening script in terminal from list: {script_name}")

        try:
            # 确保脚本有执行权限
            if platform.system() != 'Windows':
                os.chmod(script_path, 0o755)

            # 复用 ScriptConsoleTab 中的终端运行逻辑
            system = platform.system()

            if system == 'Linux':
                terminals = [
                    ['gnome-terminal', '--', 'bash', '-c', f'{script_path}; echo -e "\\n按任意键退出..."; read -n1'],
                    ['konsole', '-e', 'bash', '-c', f'{script_path}; echo -e "\\n按任意键退出..."; read -n1'],
                    ['xfce4-terminal', '-e', f'bash -c "{script_path}; echo -e \\"\\n按任意键退出...\\"; read -n1"'],
                    ['xterm', '-e', f'bash -c "{script_path}; echo -e \\"\\n按任意键退出...\\"; read -n1"']
                ]

                for terminal_cmd in terminals:
                    try:
                        subprocess.Popen(terminal_cmd)
                        self.update_status(f"已在独立终端中运行 {script_name}")
                        return
                    except:
                        continue

            elif system == 'Darwin':
                apple_script = f'''
                tell application "Terminal"
                    activate
                    do script "cd {os.path.dirname(script_path)} && {script_path}; echo -e '\\n按任意键退出...'; read -n1"
                end tell
                '''
                subprocess.Popen(['osascript', '-e', apple_script])
                self.update_status(f"已在独立终端中运行 {script_name}")
                return

            elif system == 'Windows':
                temp_bat = os.path.join(os.path.dirname(script_path), f"temp_run_{os.getpid()}.bat")
                with open(temp_bat, 'w', encoding='utf-8') as f:
                    f.write(f'@echo off\n')
                    f.write(f'cd /d "{os.path.dirname(script_path)}"\n')
                    f.write(f'"{script_path}"\n')
                    f.write(f'echo.\n')
                    f.write(f'echo 按任意键退出...\n')
                    f.write(f'pause >nul\n')
                    f.write(f'del "%~f0"\n')

                subprocess.Popen(['cmd', '/c', 'start', f'"{script_name}"', temp_bat])
                self.update_status(f"已在独立终端中运行 {script_name}")
                return

            raise Exception("无法找到合适的终端程序")

        except Exception as e:
            logger.error(f"Error opening terminal: {str(e)}")
            self.update_status(f"打开终端失败: {str(e)}")
            messagebox.showerror("错误", f"无法在独立终端中运行脚本:\n{str(e)}")

    @debug_trace
    def open_script_console(self, script_name, script_path):
        """打开脚本控制台标签页"""
        logger.info(f"Opening console for {script_name}")
        for tab_id in self.notebook.tabs():
            tab = self.notebook.nametowidget(tab_id)
            if hasattr(tab, 'script_name') and tab.script_name == script_name:
                self.notebook.select(tab_id)
                return

        tab = ScriptConsoleTab(
            self.notebook,
            script_name,
            script_path,
            self
        )
        self.notebook.add(tab, text=script_name)
        self.notebook.select(tab)
        self.tabs.append(tab)
        self.update_status(f"已打开 {script_name} 控制台")
        self.master.update_idletasks()

    @debug_trace
    def close_tab(self, tab_widget):
        """处理关闭脚本控制台标签页的逻辑"""
        logger.info(f"Closing tab for {tab_widget.script_name}")
        if tab_widget.running:
            if not messagebox.askyesno(
                    "确认关闭",
                    f"脚本 '{tab_widget.script_name}' 仍在运行中。\n"
                    "关闭此标签页将终止该脚本。\n\n"
                    "您确定要继续吗？",
                    parent=self.master
            ):
                return
            tab_widget.stop_script()

        tab_name = tab_widget.script_name
        self.notebook.forget(tab_widget)
        if tab_widget in self.tabs:
            self.tabs.remove(tab_widget)

        self.update_status(f"已关闭 '{tab_name}' 标签页")

    def update_status(self, message):
        """更新状态栏消息"""
        self.status_var.set(message)
        if len(message) > 120:
            self.status_var.set(message[:120] + "...")
        self.master.update_idletasks()

    def update_time(self):
        """更新时间显示"""
        current_time = strftime('%H:%M:%S')
        self.time_var.set(current_time)
        self.master.after(1000, self.update_time)


class ScriptManager(tk.Toplevel):
    """脚本管理窗口"""

    def __init__(self, master, app):
        super().__init__(master)
        self.app = app
        self.transient(master)
        self.title("脚本管理器")
        self.geometry("750x500")
        self.configure(background='#0d1117')
        self.protocol("WM_DELETE_WINDOW", self.close_manager)

        style = ttk.Style(self)
        style.configure("Manager.TFrame", background='#0d1117')
        style.configure("Manager.TLabel", background='#0d1117')
        style.configure("Manager.TButton", padding=5)
        style.configure("Manager.TEntry", padding=5)

        main_frame = ttk.Frame(self, padding=15, style="Manager.TFrame")
        main_frame.pack(fill=tk.BOTH, expand=True)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)

        self.create_listbox(main_frame)
        self.create_form(main_frame)
        self.create_buttons(main_frame)

        self.populate_list()
        self.listbox.bind("<<ListboxSelect>>", self.on_select)

    def create_listbox(self, parent):
        list_frame = ttk.LabelFrame(parent, text="脚本列表", padding=10)
        list_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 10))
        list_frame.columnconfigure(0, weight=1)
        list_frame.rowconfigure(0, weight=1)

        self.listbox = tk.Listbox(
            list_frame,
            background="#161b22",
            foreground="#c9d1d9",
            selectbackground="#58a6ff",
            selectforeground="#0d1117",
            borderwidth=0,
            highlightthickness=0,
            font=('DejaVu Sans Mono', 10)
        )
        self.listbox.grid(row=0, column=0, sticky="nsew")

        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.listbox.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.listbox.config(yscrollcommand=scrollbar.set)

    def create_form(self, parent):
        form_frame = ttk.LabelFrame(parent, text="编辑脚本", padding=10)
        form_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        form_frame.columnconfigure(1, weight=1)

        ttk.Label(form_frame, text="名称:", style="Manager.TLabel").grid(row=0, column=0, sticky="w", pady=5)
        self.name_var = tk.StringVar()
        self.name_entry = ttk.Entry(form_frame, textvariable=self.name_var, style="Manager.TEntry")
        self.name_entry.grid(row=0, column=1, sticky="ew")

        ttk.Label(form_frame, text="脚本:", style="Manager.TLabel").grid(row=1, column=0, sticky="w", pady=5)
        self.script_var = tk.StringVar()
        script_entry_frame = ttk.Frame(form_frame, style="Manager.TFrame")
        script_entry_frame.grid(row=1, column=1, sticky="ew")
        script_entry_frame.columnconfigure(0, weight=1)
        ttk.Entry(script_entry_frame, textvariable=self.script_var, style="Manager.TEntry").grid(row=0, column=0,
                                                                                                 sticky="ew")
        ttk.Button(script_entry_frame, text="...", command=self.browse_script, width=3).grid(row=0, column=1,
                                                                                             padx=(5, 0))

        ttk.Label(form_frame, text="颜色:", style="Manager.TLabel").grid(row=2, column=0, sticky="w", pady=5)
        self.color_var = tk.StringVar()
        color_frame = ttk.Frame(form_frame, style="Manager.TFrame")
        color_frame.grid(row=2, column=1, sticky="ew")
        color_frame.columnconfigure(0, weight=1)
        self.color_entry = ttk.Entry(color_frame, textvariable=self.color_var, style="Manager.TEntry")
        self.color_entry.grid(row=0, column=0, sticky="ew")
        self.color_preview = tk.Label(color_frame, text="  ", background=self.color_var.get() or "grey")
        self.color_preview.grid(row=0, column=1, padx=(5, 0))
        ttk.Button(color_frame, text="选取", command=self.pick_color, width=5).grid(row=0, column=2, padx=(5, 0))
        self.color_var.trace("w", self.update_color_preview)

    def create_buttons(self, parent):
        button_frame = ttk.Frame(parent, style="Manager.TFrame")
        button_frame.grid(row=2, column=0, columnspan=2, sticky="e", pady=(10, 0))

        ttk.Button(button_frame, text="新增", command=self.add_script, style="Manager.TButton").pack(side=tk.LEFT,
                                                                                                     padx=5)
        ttk.Button(button_frame, text="保存", command=self.save_script, style="Manager.TButton").pack(side=tk.LEFT,
                                                                                                      padx=5)
        ttk.Button(button_frame, text="删除", command=self.delete_script, style="Manager.TButton").pack(side=tk.LEFT,
                                                                                                        padx=5)
        ttk.Button(button_frame, text="关闭", command=self.close_manager, style="Manager.TButton").pack(side=tk.LEFT,
                                                                                                        padx=(20, 5))

    def populate_list(self):
        self.listbox.delete(0, tk.END)
        for script in self.app.scripts:
            self.listbox.insert(tk.END, script['name'])

    def on_select(self, event=None):
        indices = self.listbox.curselection()
        if not indices:
            return

        index = indices[0]
        script = self.app.scripts[index]
        self.name_var.set(script.get('name', ''))
        self.script_var.set(script.get('script', ''))
        self.color_var.set(script.get('color', ''))

    def clear_form(self):
        self.name_var.set("")
        self.script_var.set("")
        self.color_var.set("")
        self.listbox.selection_clear(0, tk.END)

    def add_script(self):
        self.clear_form()
        self.name_entry.focus()

    def save_script(self):
        name = self.name_var.get().strip()
        script_path = self.script_var.get().strip()
        if not name or not script_path:
            messagebox.showerror("错误", "脚本名称和路径不能为空。")
            return

        new_script_data = {
            "name": name,
            "script": script_path,
            "color": self.color_var.get().strip()
        }

        indices = self.listbox.curselection()
        if indices:
            index = indices[0]
            self.app.scripts[index] = new_script_data
        else:
            self.app.scripts.append(new_script_data)

        self.app.save_scripts()
        self.populate_list()
        self.clear_form()

    def delete_script(self):
        indices = self.listbox.curselection()
        if not indices:
            messagebox.showwarning("警告", "请先选择一个要删除的脚本。")
            return

        index = indices[0]
        script_name = self.app.scripts[index]['name']
        if messagebox.askyesno("确认删除", f"确定要删除脚本 '{script_name}' 吗？"):
            self.app.scripts.pop(index)
            self.app.save_scripts()
            self.populate_list()
            self.clear_form()

    def browse_script(self):
        script_dir = os.path.join(os.path.dirname(self.app.get_config_path()), "scripts")
        if not os.path.exists(script_dir):
            os.makedirs(script_dir)

        filepath = filedialog.askopenfilename(
            initialdir=script_dir,
            title="选择脚本文件",
            filetypes=(("Shell Scripts", "*.sh"), ("All files", "*.*"))
        )
        if filepath:
            base_dir = os.path.dirname(self.app.get_config_path())
            try:
                relative_path = os.path.relpath(filepath, base_dir)
                self.script_var.set(relative_path.replace("\\", "/"))
            except ValueError:
                self.script_var.set(filepath)

    def pick_color(self):
        color_code = colorchooser.askcolor(title="选取颜色")
        if color_code and color_code[1]:
            self.color_var.set(color_code[1])

    def update_color_preview(self, *args):
        color = self.color_var.get()
        try:
            self.color_preview.config(background=color)
        except tk.TclError:
            self.color_preview.config(background="grey")

    def close_manager(self):
        self.destroy()


if __name__ == "__main__":
    logger.info("Starting application")
    root = tk.Tk()
    os.environ['TK_SILENCE_DEPRECATION'] = '1'

    try:
        default_font = tk.font.nametofont("TkDefaultFont")
        default_font.configure(family="DejaVu Sans", size=9)
        text_font = tk.font.nametofont("TkTextFont")
        text_font.configure(family="DejaVu Sans Mono", size=10)
        fixed_font = tk.font.nametofont("TkFixedFont")
        fixed_font.configure(family="DejaVu Sans Mono", size=10)
    except Exception as e:
        logger.error(f"字体配置错误: {e}")

    app = ScriptRunnerApp(root)
    try:
        logger.info("Starting main loop")
        root.mainloop()
    except KeyboardInterrupt:
        logger.info("Received keyboard interrupt")
        app.cleanup_and_exit()
    except Exception as e:
        logger.error(f"Unhandled exception: {str(e)}")
        logger.error(traceback.format_exc())
        raise

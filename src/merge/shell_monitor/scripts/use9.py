# 导入tkinter库和消息框
import tkinter as tk
from tkinter import messagebox
# 导入Pillow库处理图片
from PIL import Image, ImageTk
import os

# 全局变量（保留核心配置，删除无用的逐个选值变量）
SAVE_FILE = "./../../path/map.txt"  # 保存的文件名，固定为map.txt
BG_IMAGE_PATH = "background.jpg"    # 背景图路径（脚本同目录）

def save_to_txt(input_var, entry_widget):
    """
    新版保存函数：一次性输入校验 + 保存
    :param input_var: 输入框绑定的变量
    :param entry_widget: 输入框组件（用于错误时保持焦点）
    """
    # 获取输入内容并去除首尾空格
    content = input_var.get().strip()
    
    # 🔍 严格校验规则
    # 1. 校验长度必须为12位
    if len(content) != 12:
        messagebox.showerror("输入错误", f"必须输入12位数字！\n当前长度：{len(content)}位")
        entry_widget.focus()  # 聚焦输入框，方便修改
        return
    
    # 2. 校验是否为纯数字 + 每个数字在1~9之间
    if not content.isdigit():
        messagebox.showerror("输入错误", "只能输入数字！不允许空格、字母、符号")
        entry_widget.focus()
        return
    
    for char in content:
        if not (1 <= int(char) <= 9):
            messagebox.showerror("输入错误", "数字必须是 1~9 之间！")
            entry_widget.focus()
            return

    # ✅ 校验通过，写入文件
    try:
        # 自动创建目录（防止路径不存在报错）
        os.makedirs(os.path.dirname(SAVE_FILE), exist_ok=True)
        with open(SAVE_FILE, "w", encoding="utf-8") as f:
            f.write(content)
        messagebox.showinfo("保存成功", f"已保存到【{SAVE_FILE}】\n内容：{content}")
        entry_widget.focus()
    except Exception as e:
        messagebox.showerror("保存失败", f"文件写入错误：{str(e)}")

def create_gui():
    """主界面：一次性输入12位数字 + 居中布局 + 背景图"""
    root = tk.Tk()
    root.title("12位数字一次性输入器")
    root.geometry("640x480")
    root.resizable(False, False)
    root_bg = root.cget("bg")  # 主窗口原生背景色

    # 背景图片逻辑（完全保留）
    bg_label = None
    if os.path.exists(BG_IMAGE_PATH):
        image = Image.open(BG_IMAGE_PATH)
        image = image.resize((640, 480), Image.LANCZOS)
        bg_image = ImageTk.PhotoImage(image)
        bg_label = tk.Label(root, image=bg_image, bg=root_bg)
        bg_label.place(x=0, y=0, relwidth=1, relheight=1)
        bg_label.image = bg_image
        main_container = bg_label
    else:
        messagebox.showwarning("图片缺失", f"未找到背景图片【{BG_IMAGE_PATH}】，请放在脚本同目录！")
        main_container = root

    # 提示文字（保留风格）
    tip_label = tk.Label(
        main_container,
        text="一次性输入12位数字（1~9），按Enter键保存：",
        font=("Arial", 12), bg=root_bg
    )
    tip_label.place(relx=0.5, rely=0.3, anchor=tk.CENTER)  # 居中显示

    # ===================== 核心修改：一次性输入框 =====================
    # 输入框变量
    input_var = tk.StringVar()
    # 创建输入框（大字体、居中、宽度适配12位数字）
    entry = tk.Entry(
        main_container,
        textvariable=input_var,
        font=("Arial", 18, "bold"),  # 大字体方便输入
        justify=tk.CENTER,           # 文字居中
        width=15                     # 宽度足够放12位数字
    )
    entry.place(relx=0.5, rely=0.4, anchor=tk.CENTER)  # 界面居中
    entry.focus()  # 默认聚焦输入框
    # =================================================================

    # Enter键绑定保存功能（保留）
    root.bind("<Return>", lambda event: save_to_txt(input_var, entry))
    root.mainloop()

# 程序入口
if __name__ == "__main__":
    create_gui()


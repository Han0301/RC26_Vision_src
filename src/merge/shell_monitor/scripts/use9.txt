# 导入tkinter库和消息框
import tkinter as tk
from tkinter import messagebox
# 新增：导入Pillow库处理图片（缩放、格式兼容）
from PIL import Image, ImageTk
import os

# 全局变量：保留原始核心变量，删除随机色相关，新增高亮配置
var_list = []    # 存储tk.StringVar，记录每个位置选中值
label_list = []  # 存储Canvas对象（仍用原名，不改动其他逻辑）
SAVE_FILE = "./../../path/map.txt"  # 保存的文件名，固定为map.txt
POSITIONS = [12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]  # 原下标顺序
BG_IMAGE_PATH = "background.jpg"  # 背景图路径（脚本同目录）
# ⚡ 高亮配置：固定蓝色+粗边框+常亮
HIGHLIGHT_COLOR = "#0099ff"  # 醒目天蓝色（可改其他蓝色，如#0066cc深蓝）
BORDER_WIDTH = 4             # 边框宽度，调大更显眼（推荐4-6）
WHITE = "#ffffff"            # 原始边框白色

def choose_value(pos, var):
    """点击选值弹窗：完全保留原始代码，一字未改"""
    root = tk._default_root
    choose_win = tk.Toplevel(root)
    choose_win.title(f"选择位置 {pos} 的值")
    choose_win.geometry("280x150")
    choose_win.resizable(False, False)
    choose_win.transient(root)

    def set_val(v):
        var.set(v)
        choose_win.destroy()

    tip_label = tk.Label(choose_win, text=f"请选择位置 {pos} 的值", font=("Arial", 12))
    tip_label.pack(pady=10)
    btn_frame = tk.Frame(choose_win)
    btn_frame.pack()
    for num in ["1", "2", "3", "4"]:
        tk.Button(
            btn_frame,
            text=num, font=("Arial", 14, "bold"), width=3,
            command=lambda n=num: set_val(n)
        ).pack(side=tk.LEFT, padx=6)

    choose_win.update_idletasks()
    x = root.winfo_x() + (root.winfo_width() - choose_win.winfo_width()) // 2
    y = root.winfo_y() + (root.winfo_height() - choose_win.winfo_height()) // 2
    choose_win.geometry(f"+{x}+{y}")
    choose_win.grab_set()

def save_to_txt():
    """Enter保存：修正后的顺序（1→12）+ 保留标红逻辑，完全不变"""
    # 第一步：先按原顺序做非空校验，保证报错的pos位置准确
    for idx, var in enumerate(var_list):
        val = var.get().strip()
        pos = POSITIONS[idx]
        if not val:
            messagebox.showerror("输入错误", f"位置 {pos} 尚未选择值！请点击选择1/2/3/4")
            # 适配Canvas文字标红
            label_list[idx].itemconfig(label_list[idx].text_id, fill="red")
            label_list[idx].after(1000, lambda c=label_list[idx]: c.itemconfig(c.text_id, fill="black"))
            return
    
    # 第二步：校验通过，逆序遍历var_list拼接内容，实现保存顺序1→2→…→12
    result_str = ""
    for var in reversed(var_list):
        result_str += var.get().strip()

    try:
        with open(SAVE_FILE, "w", encoding="utf-8") as f:
            f.write(result_str)
        messagebox.showinfo("保存成功", f"已保存到【{SAVE_FILE}】\n内容：{result_str}")
    except Exception as e:
        messagebox.showerror("保存失败", f"文件写入错误：{str(e)}")

# ⚡ 核心高亮函数：输入后蓝色粗边框常亮，无值恢复白色
def highlight_border(canvas):
    # 未激活/无值时，恢复白色粗边框
    if not canvas.anim_active or canvas.get_var().get().strip() == "":
        canvas.reset_border()
        return
    # 输入后设置整个边框为固定蓝色（持续常亮，无循环）
    canvas.itemconfig(canvas.top_line, fill=HIGHLIGHT_COLOR)
    canvas.itemconfig(canvas.right_line, fill=HIGHLIGHT_COLOR)
    canvas.itemconfig(canvas.bottom_line, fill=HIGHLIGHT_COLOR)
    canvas.itemconfig(canvas.left_line, fill=HIGHLIGHT_COLOR)

def create_gui():
    """主界面：输入后蓝色粗边框常亮+4×3居中+文字居中+纯透明+下标黑色+保存顺序正确"""
    root = tk.Tk()
    root.title("12位置选择器（蓝色粗边框常亮）")
    root.geometry("640x480")
    root.resizable(False, False)
    root_bg = root.cget("bg")  # 主窗口原生背景色，避报错

    # 背景图片逻辑：修复后的正确代码，完全不变
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

    # 提示文字：完全保留原始布局
    tip_label = tk.Label(
        main_container,
        text="点击网格选择值（下标对应图片位置）：",
        font=("Arial", 10), bg=root_bg
    )
    tip_label.grid(row=0, column=0, columnspan=3, pady=10)

    # 3列权重配置，整体居中：完全保留
    main_container.columnconfigure(0, weight=1)
    main_container.columnconfigure(1, weight=1)
    main_container.columnconfigure(2, weight=1)

    # 循环创建4×3方块：仅修改边框宽度，其余逻辑完全不变
    for i in range(12):
        r = i // 3
        c = i % 3
        pos = POSITIONS[i]
        val_var = tk.StringVar()
        val_var.set("")
        var_list.append(val_var)

        # Canvas基础设置：去掉原生边框，手绘边框，完全不变
        pos_canvas = tk.Canvas(
            main_container,
            width=60, height=40,
            highlightthickness=0,
            bg=root_bg,
            bd=0, relief="flat"
        )
        pos_canvas.grid(row=r*2 + 2, column=c, padx=10, pady=5)
        label_list.append(pos_canvas)

        # 动画属性：仅保留激活开关和值绑定，完全不变
        pos_canvas.anim_active = False
        pos_canvas.set_var = lambda v=val_var: setattr(pos_canvas, "var", v)
        pos_canvas.get_var = lambda: getattr(pos_canvas, "var", None)
        pos_canvas.set_var(val_var)

        # ✅ 关键修改：手绘边框用全局BORDER_WIDTH，宽度调大，初始白色
        pos_canvas.top_line = pos_canvas.create_line(0, 0, 60, 0, width=BORDER_WIDTH, fill=WHITE)
        pos_canvas.right_line = pos_canvas.create_line(60, 0, 60, 40, width=BORDER_WIDTH, fill=WHITE)
        pos_canvas.bottom_line = pos_canvas.create_line(60, 40, 0, 40, width=BORDER_WIDTH, fill=WHITE)
        pos_canvas.left_line = pos_canvas.create_line(0, 40, 0, 0, width=BORDER_WIDTH, fill=WHITE)

        # 重置边框方法：恢复白色粗边框，完全不变
        def reset_border(canvas=pos_canvas):
            canvas.itemconfig(canvas.top_line, fill=WHITE)
            canvas.itemconfig(canvas.right_line, fill=WHITE)
            canvas.itemconfig(canvas.bottom_line, fill=WHITE)
            canvas.itemconfig(canvas.left_line, fill=WHITE)
        pos_canvas.reset_border = reset_border

        # 文字居中：完全保留原始逻辑
        text_id = pos_canvas.create_text(30, 20, text="", font=("Arial", 14), fill="black", anchor=tk.CENTER)
        pos_canvas.text_id = text_id

        # 高亮触发逻辑：输入值激活常亮，无值停止，完全不变
        def update_text(*args, canvas=pos_canvas, var=val_var):
            canvas.itemconfig(canvas.text_id, text=var.get())
            if var.get().strip() != "":
                if not canvas.anim_active:
                    canvas.anim_active = True
                    highlight_border(canvas)  # 一次调用，持续常亮
            else:
                canvas.anim_active = False
        val_var.trace_add("write", update_text)

        # 点击事件+鼠标手型：完全保留原始逻辑
        pos_canvas.bind("<Button-1>", lambda event, p=pos, v=val_var: choose_value(p, v))
        pos_canvas.config(cursor="hand2")

        # 下标标签：黑色、原布局，完全保留
        idx_label = tk.Label(
            main_container,
            text=str(pos), font=("Arial", 10), fg="black", bg=root_bg
        )
        idx_label.grid(row=r*2 + 3, column=c, pady=2)

    # Enter键绑定：完全保留原始逻辑
    root.bind("<Return>", lambda event: save_to_txt())
    root.mainloop()

# 程序入口：完全保留
if __name__ == "__main__":
    create_gui()


# -*- coding: utf-8 -*-
"""
文本文件短行统计工具
功能：读取txt文件，统计字符数少于12的行，并显示具体行号和内容
"""

import os
import sys


def count_short_lines(file_path, char_limit=12):
    """
    统计文件中字符数少于等于指定限制的行

    参数:
        file_path (str): 文本文件路径
        char_limit (int): 字符数限制，默认为12

    返回:
        tuple: (总共有多少行, 短行数量, 短行列表)
    """
    total_lines = 0
    short_line_count = 0
    short_lines = []

    try:
        # 使用utf-8编码打开文件，同时处理可能的编码错误
        with open(file_path, 'r', encoding='utf-8', errors='replace') as file:
            for line_num, line in enumerate(file, start=1):
                total_lines += 1
                # 去掉末尾的换行符后再计算字符数
                # 如果需要包含换行符，请使用 len(line)
                line_content = line.rstrip('\n')
                char_count = len(line_content)

                if char_count <= char_limit:
                    short_line_count += 1
                    short_lines.append((line_num, char_count, line_content))

        return total_lines, short_line_count, short_lines

    except FileNotFoundError:
        print(f"错误：找不到文件 '{file_path}'")
        return None
    except PermissionError:
        print(f"错误：没有权限读取文件 '{file_path}'")
        return None
    except Exception as e:
        print(f"读取文件时发生错误：{str(e)}")
        return None


def main(file_path:str):
    # 检查文件是否存在
    if not os.path.isfile(file_path):
        print(f"错误：'{file_path}' 不是一个有效的文件路径")
        return

    # 统计短行
    result = count_short_lines(file_path)

    if result is None:
        return

    total_lines, short_line_count, short_lines = result

    # 显示结果
    print("\n" + "=" * 50)
    print(f"文件分析结果：{os.path.basename(file_path)}")
    print("=" * 50)
    print(f"文件总行数：{total_lines}")
    print(f"字符数少于12的行数：{short_line_count}")
    print(f"占比：{short_line_count / total_lines * 100:.2f}%" if total_lines > 0 else "占比：0%")
    print("-" * 50)

    if short_line_count > 0:
        print("\n具体短行信息：")
        print("行号\t字符数\t内容")
        print("-" * 50)
        for line_num, char_count, content in short_lines:
            # 处理空行的显示
            display_content = content if content else "(空行)"
            print(f"{line_num}\t{char_count}\t{display_content}")
    else:
        print("\n该文件中没有字符数少于12的行。")

    print("\n" + "=" * 50)


if __name__ == "__main__":
    txt_path = r"/home/awwsome/RC/robocon_ws4/src/merge/shell_monitor2/path_23.txt"
    main(txt_path)
#!/bin/bash
# 核心：获取脚本本身所在的绝对目录（解决相对路径基准问题）
SCRIPT_DIR=$(cd $(dirname $(realpath $0)) && pwd)
# 切换到脚本所在目录（此时脚本和py同目录，就能找到use6.py了）
cd $SCRIPT_DIR

cd ..
cd ..
cd ..
cd ..
source ./devel/setup.bash
sleep 1
roscore



#!/bin/bash

echo "================================================"
echo "  无人机综合控制系统 - Web GUI"
echo "================================================"
echo ""

# 检查依赖
echo "检查Python依赖..."
python3 -c "import flask" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  Flask未安装，正在安装依赖..."
    pip install -r requirements_gui.txt
fi

echo ""
echo "启动Web服务器..."
echo ""
echo "访问地址："
echo "  本地: http://localhost:5000"
echo "  网络: http://$(hostname -I | awk '{print $1}'):5000"
echo ""
echo "按 Ctrl+C 停止服务器"
echo "================================================"
echo ""

# 切换到脚本所在目录
cd "$(dirname "$0")"

# 启动GUI
python3 drone_control_gui.py

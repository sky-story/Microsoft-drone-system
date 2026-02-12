#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 清屏
clear

# 显示Logo
echo -e "${CYAN}"
echo "  ╔═══════════════════════════════════════════════════════════╗"
echo "  ║                                                           ║"
echo "  ║        🚁  无人机综合控制系统  🚁                          ║"
echo "  ║                                                           ║"
echo "  ║           Web GUI - 启动菜单                              ║"
echo "  ║                                                           ║"
echo "  ╚═══════════════════════════════════════════════════════════╝"
echo -e "${NC}"
echo ""

# 检查Python
echo -e "${YELLOW}检查Python环境...${NC}"
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ Python3 未安装${NC}"
    exit 1
fi
echo -e "${GREEN}✅ Python3: $(python3 --version)${NC}"
echo ""

# 主菜单
while true; do
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${PURPLE}请选择启动模式：${NC}"
    echo ""
    echo -e "  ${GREEN}1${NC} - 🎭 演示模式 ${YELLOW}(推荐新手)${NC}"
    echo "      • 无需真实无人机"
    echo "      • 自动模拟完整任务流程"
    echo "      • 用于学习和演示"
    echo ""
    echo -e "  ${GREEN}2${NC} - 🚀 正式模式"
    echo "      • 连接真实无人机"
    echo "      • 实际控制飞行"
    echo "      • 生产环境使用"
    echo ""
    echo -e "  ${GREEN}3${NC} - 🔧 检查依赖"
    echo "      • 检查Python包是否安装"
    echo "      • 验证环境配置"
    echo ""
    echo -e "  ${GREEN}4${NC} - 📚 查看文档"
    echo "      • 使用指南"
    echo "      • 快速参考"
    echo ""
    echo -e "  ${GREEN}5${NC} - 📦 安装依赖"
    echo "      • 安装所需Python包"
    echo ""
    echo -e "  ${GREEN}0${NC} - 退出"
    echo ""
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -n "请输入选项 [0-5]: "
    
    read choice
    echo ""
    
    case $choice in
        1)
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${GREEN}🎭 启动演示模式${NC}"
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo ""
            echo "正在启动演示模式..."
            echo "• 会自动模拟无人机连接"
            echo "• 会自动运行一次完整任务"
            echo "• 浏览器打开: http://localhost:5000"
            echo ""
            echo -e "${YELLOW}按 Ctrl+C 停止服务器${NC}"
            echo ""
            sleep 2
            cd "$(dirname "$0")"
            python3 test_gui_demo.py
            break
            ;;
        2)
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${GREEN}🚀 启动正式模式${NC}"
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo ""
            echo "正在启动正式模式..."
            echo "• 需要连接真实无人机"
            echo "• 确保无人机WiFi已连接"
            echo "• 浏览器打开: http://localhost:5000"
            echo ""
            echo -e "${YELLOW}按 Ctrl+C 停止服务器${NC}"
            echo ""
            sleep 2
            cd "$(dirname "$0")"
            ./start_gui.sh
            break
            ;;
        3)
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${GREEN}🔧 检查依赖${NC}"
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo ""
            cd "$(dirname "$0")"
            python3 check_dependencies.py
            echo ""
            echo -e "${YELLOW}按回车键继续...${NC}"
            read
            clear
            ;;
        4)
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${GREEN}📚 查看文档${NC}"
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo ""
            cd "$(dirname "$0")"
            cat README.md | less
            
            echo ""
            echo -e "${YELLOW}按回车键继续...${NC}"
            read
            clear
            ;;
        5)
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo -e "${GREEN}📦 安装依赖${NC}"
            echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
            echo ""
            echo "正在安装Python依赖..."
            echo ""
            cd "$(dirname "$0")"
            pip install -r requirements_gui.txt
            echo ""
            echo -e "${GREEN}✅ 依赖安装完成${NC}"
            echo ""
            echo -e "${YELLOW}按回车键继续...${NC}"
            read
            clear
            ;;
        0)
            echo -e "${GREEN}👋 再见！${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}❌ 无效选项，请重新选择${NC}"
            echo ""
            sleep 1
            ;;
    esac
done

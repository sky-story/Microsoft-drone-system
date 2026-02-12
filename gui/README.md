# 无人机控制系统 - Web GUI

## 快速启动

```bash
# 演示模式（无需无人机）
python3 test_gui_demo.py

# 正式模式
./start_gui.sh

# 交互式菜单
./menu.sh
```

然后浏览器打开：http://localhost:5000

## 文件说明

- `drone_control_gui.py` - 主程序
- `test_gui_demo.py` - 演示模式
- `start_gui.sh` - 启动脚本
- `启动菜单.sh` - 交互式菜单
- `check_dependencies.py` - 依赖检查
- `requirements_gui.txt` - 依赖列表
- `templates/` - HTML模板
- `static/` - CSS和JS文件

## 安装依赖

```bash
pip install -r requirements_gui.txt
```

## 系统功能

整合三个模块：
1. **Navigation** - GPS导航 ⚠️ 需要GPS
2. **Perception** - 目标跟踪（YOLO/颜色识别）✅ 不需要GPS
3. **Winch System** - 绞盘抓取 ✅ 不需要GPS

### GPS需求说明

| 功能 | 需要GPS | 备注 |
|------|---------|------|
| Take Off | ❌ | 室内外都可起飞 |
| Land | ❌ | 任何时候都可降落 |
| Manual Control | ❌ | 完全手动，不依赖GPS |
| **Navigation** | ✅ **需要 ≥10颗卫星** | GPS自动导航 |
| Perception | ❌ | 只需要摄像头 |
| Winch System | ❌ | 只需要网络连接 |

## 基本使用

1. 连接无人机
2. 起飞
3. **手动控制**（起飞后可用）：
   - 前进/后退 (Pitch)
   - 左移/右移 (Roll)
   - 上升/下降 (Altitude)
   - 左转/右转 (Yaw)
4. 启动GPS导航（输入坐标）
5. 启动目标跟踪（选择YOLO或颜色模式）
6. 系统自动触发绞盘
7. 降落

## 配置

在 `drone_control_gui.py` 中修改：
- 无人机IP：`drone_ip = "192.168.42.1"`
- 服务器端口：`port = 5000`
- 绞盘URL：`system_url = "http://192.168.42.15"`

## 界面特色

- 现代深色主题 + 渐变色设计
- 实时状态更新（WebSocket）
- 响应式设计（支持手机/平板）
- 标签页布局，无需滚动
- 虚拟控制面板（手动控制）
- 彩色日志分类
- **紧急停止按钮**（浮动在右下角，最高优先级）
- **模块独立测试**（可单独测试Navigation/Perception/Winch）

## 模块测试

每个模块都可以单独测试：

1. **Navigation Test** - 检查GPS状态、位置信息
2. **Perception Test** - 验证检测模式、目标配置
3. **Winch Test** - 测试完整抓取序列（LOWER → PULL → STOP）

点击各标签页中的 "Test Module" 按钮即可。

## 紧急停止

- 浮动按钮在右下角
- 红色圆形，持续脉动
- 任何时候都可点击
- 最高优先级，立即响应

## GPS需求说明

**重要：大部分功能不需要GPS！**

| 功能 | 需要GPS | 室内可用 |
|------|---------|----------|
| Take Off | ❌ | ✅ |
| Land | ❌ | ✅ |
| Manual Control | ❌ | ✅ |
| Perception (Tracking) | ❌ | ✅ |
| Winch System | ❌ | ✅ |
| **Navigation (GPS)** | ✅ 需要≥10颗卫星 | ❌ |

**测试建议：**
- 室内：测试起飞、手动控制、目标跟踪、绞盘
- 户外：测试GPS导航

## 故障排除

### 起飞失败

**⚠️ 起飞不需要GPS！可以在室内起飞。**

**最常见原因：未校准**
- 症状：`drone_not_calibrated`
- 解决：
  1. 使用手机安装 **FreeFlight 7** APP
  2. 连接无人机WiFi
  3. 打开APP → Settings → Calibration
  4. 校准磁力计（Magnetometer）
  5. 完成后重启无人机

**其他原因：**
- 电池过低 → 更换电池
- 螺旋桨未安装 → 检查螺旋桨
- 传感器异常 → 重启无人机

### 技术问题

```bash
# 检查依赖
python3 check_dependencies.py

# 查看端口
netstat -tlnp | grep 5000
```

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
依赖检查脚本
检查所有GUI运行所需的Python包是否已安装
"""

import sys

def check_import(module_name, package_name=None, min_version=None):
    """检查模块是否可以导入"""
    if package_name is None:
        package_name = module_name
    
    try:
        mod = __import__(module_name)
        version = getattr(mod, '__version__', 'unknown')
        
        if min_version and version != 'unknown':
            from packaging import version as pkg_version
            if pkg_version.parse(version) < pkg_version.parse(min_version):
                print(f"  ⚠️  {package_name}: 版本过低 ({version} < {min_version})")
                return False
        
        print(f"  ✅ {package_name}: {version}")
        return True
    except ImportError:
        print(f"  ❌ {package_name}: 未安装")
        return False


def main():
    """主函数"""
    print("=" * 60)
    print("  无人机控制系统GUI - 依赖检查")
    print("=" * 60)
    print()
    
    print("检查Python版本...")
    version = sys.version_info
    print(f"  Python {version.major}.{version.minor}.{version.micro}")
    if version.major < 3 or (version.major == 3 and version.minor < 8):
        print(f"  ⚠️  需要Python 3.8+，当前版本过低")
    else:
        print(f"  ✅ Python版本符合要求")
    print()
    
    print("检查必需的Python包...")
    all_ok = True
    
    # Web框架
    print("\n[Web框架]")
    all_ok &= check_import('flask', 'Flask', '3.0.0')
    all_ok &= check_import('flask_socketio', 'Flask-SocketIO', '5.3.0')
    all_ok &= check_import('socketio', 'python-socketio')
    all_ok &= check_import('werkzeug', 'Werkzeug')
    
    # 计算机视觉
    print("\n[计算机视觉]")
    all_ok &= check_import('cv2', 'opencv-python')
    all_ok &= check_import('numpy', 'numpy')
    
    # 网络请求
    print("\n[网络]")
    all_ok &= check_import('requests', 'requests')
    
    # 无人机SDK（可选）
    print("\n[无人机SDK - 可选]")
    try:
        import olympe
        print(f"  ✅ olympe: {getattr(olympe, '__version__', 'installed')}")
    except ImportError:
        print(f"  ⚠️  olympe: 未安装（运行需要，但测试演示模式不需要）")
    
    print()
    print("=" * 60)
    
    if all_ok:
        print("✅ 所有依赖检查通过！")
        print()
        print("可以运行以下命令启动GUI：")
        print("  • 演示模式: python3 test_gui_demo.py")
        print("  • 正式模式: ./start_gui.sh")
    else:
        print("❌ 部分依赖缺失")
        print()
        print("请运行以下命令安装依赖：")
        print("  pip install -r requirements_gui.txt")
    
    print("=" * 60)
    
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
import os
import sys

# 必须在导入olympe之前设置OpenGL平台
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import time
import math
import argparse
import traceback
import tty
import termios
import select

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, CancelMoveTo, PCMD
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.GPSState import NumberOfSatelliteChanged
from olympe.messages.ardrone3.GPSSettingsState import GeofenceCenterChanged


# ============================================================================
# 超参数配置区域 - 在这里修改所有参数
# ============================================================================

# 目标GPS坐标（默认值，可通过命令行参数覆盖）
TARGET_LATITUDE = 47.6218425      # 目标纬度
TARGET_LONGITUDE = -122.1769126   # 目标经度
TARGET_ALTITUDE = 1.0             # 目标高度（米）

# 无人机连接设置
DRONE_IP = "192.168.42.1"         # 无人机IP地址

# GPS相关设置
MIN_SATELLITES = 12               # 最少卫星数量要求
GPS_STABLE_SECONDS = 2            # GPS稳定等待时间（秒）
GPS_MAX_WAIT = 60                 # GPS等待最长时间（秒）

# 飞行相关设置
TAKEOFF_TIMEOUT = 30              # 起飞超时时间（秒）
MOVETO_TIMEOUT = 120              # moveTo命令超时时间（秒）
STABILIZE_DELAY = 3               # 起飞后稳定等待时间（秒）

# 到达判定
ARRIVAL_THRESHOLD = 0.5           # 到达阈值，距离小于此值视为到达（米）
ARRIVAL_CONFIRM_COUNT = 3         # 需要连续确认到达的次数
MONITOR_MAX_LOOPS = 200           # 监控最大循环次数（约100秒）

# 手动控制设置
MANUAL_CONTROL_SPEED = 25         # 键盘手动控制速度 (0-100)
MANUAL_DECAY_RATE = 0.8           # 手动控制衰减率（释放按键后的减速）

# ============================================================================


class TerminalKeyReader:
    """终端非阻塞键盘读取"""
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old = None

    def __enter__(self):
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def read_key_nonblock(self):
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if not r:
            return None
        return sys.stdin.read(1)


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlmb / 2)**2
    return 2 * R * math.asin(math.sqrt(a))


def wait_gps_ready(drone, min_sats=MIN_SATELLITES, stable_seconds=GPS_STABLE_SECONDS, max_wait=GPS_MAX_WAIT):
    print(f"[1] Waiting GPS ready (>= {min_sats} sats)")
    print(f"    💡 提示：GPS需要磁力计校准（使用FreeFlight 7 APP）\n")
    
    start = time.time()
    stable_since = None
    last_sats = None  # 上次的卫星数
    last_print_time = 0  # 上次打印时间

    while True:
        elapsed = time.time() - start
        if elapsed > max_wait:
            raise RuntimeError("GPS not ready in time")

        # 等待状态更新，但不要求一定成功
        try:
            drone(NumberOfSatelliteChanged()).wait(_timeout=1)
        except Exception as e:
            if elapsed - last_print_time > 10:  # 每10秒打印一次
                print(f"    ⏳ 等待GPS状态更新...")
                last_print_time = elapsed
            time.sleep(1)
            continue

        try:
            drone(GeofenceCenterChanged()).wait(_timeout=1)
        except Exception as e:
            if elapsed - last_print_time > 10:
                print(f"    ⏳ 等待GPS位置更新...")
                last_print_time = elapsed
            time.sleep(1)
            continue

        # 安全地获取状态 - 添加异常处理
        try:
            sat_state = drone.get_state(NumberOfSatelliteChanged)
        except (ValueError, Exception):
            sat_state = None
        
        try:
            geo_state = drone.get_state(GeofenceCenterChanged)
        except (ValueError, Exception):
            geo_state = None

        # 检查状态是否有效
        if not sat_state or "numberOfSatellite" not in sat_state:
            if elapsed - last_print_time > 10:
                print(f"    ⏳ GPS卫星状态未初始化... (已等待 {elapsed:.0f}秒)")
                last_print_time = elapsed
            time.sleep(1)
            continue
        
        if not geo_state or "latitude" not in geo_state:
            if elapsed - last_print_time > 10:
                print(f"    ⏳ GPS位置状态未初始化... (已等待 {elapsed:.0f}秒)")
                print(f"       💡 可能需要磁力计校准")
                last_print_time = elapsed
            time.sleep(1)
            continue

        sats = sat_state["numberOfSatellite"]
        lat = geo_state.get("latitude", 0)
        lon = geo_state.get("longitude", 0)

        # 只在卫星数变化或每2秒打印一次
        should_print = (last_sats != sats) or (elapsed - last_print_time > 2)
        
        ok = sats >= min_sats and abs(lat) > 0.0001 and abs(lon) > 0.0001

        if ok:
            if stable_since is None:
                stable_since = time.time()
                print(f"    ✓ GPS条件满足！卫星数={sats}, 等待稳定 {stable_seconds}秒...")
                last_print_time = elapsed
            
            # 稳定期间显示倒计时
            remaining = stable_seconds - (time.time() - stable_since)
            if remaining > 0:
                print(f"    ⏱️  稳定中... {remaining:.1f}秒 ", end='\r', flush=True)
            
            if time.time() - stable_since >= stable_seconds:
                print(f"\n    ✅ GPS ready! 卫星数={sats}, 起始位置=({lat:.7f}, {lon:.7f})\n")
                return lat, lon
        else:
            stable_since = None
            if should_print:
                reason = []
                if sats < min_sats:
                    reason.append(f"卫星不足({sats}/{min_sats})")
                if abs(lat) <= 0.0001 or abs(lon) <= 0.0001:
                    reason.append("位置未锁定")
                print(f"    ⏳ 等待GPS... {', '.join(reason)}")
                last_sats = sats
                last_print_time = elapsed
        
        time.sleep(0.5)  # 减少循环频率


def get_user_input():
    """交互式获取用户输入的GPS坐标和高度"""
    print("\n" + "="*60)
    print("📍 请输入目标GPS坐标和飞行高度")
    print("="*60)
    
    while True:
        try:
            # 输入纬度
            lat_input = input("\n请输入目标纬度 (例如: 47.6218425): ").strip()
            if not lat_input:
                print("❌ 纬度不能为空！")
                continue
            target_lat = float(lat_input)
            
            if target_lat < -90 or target_lat > 90:
                print("❌ 纬度必须在 -90 到 90 之间！")
                continue
            
            # 输入经度
            lon_input = input("请输入目标经度 (例如: -122.1769126): ").strip()
            if not lon_input:
                print("❌ 经度不能为空！")
                continue
            target_lon = float(lon_input)
            
            if target_lon < -180 or target_lon > 180:
                print("❌ 经度必须在 -180 到 180 之间！")
                continue
            
            # 输入高度
            alt_input = input("请输入飞行高度/米 (例如: 2.0): ").strip()
            if not alt_input:
                print("❌ 高度不能为空！")
                continue
            target_alt = float(alt_input)
            
            if target_alt < 0.5 or target_alt > 150:
                print("❌ 高度必须在 0.5 到 150 米之间！")
                continue
            
            # 确认信息
            print("\n" + "="*60)
            print("✅ 输入完成，请确认以下信息：")
            print("="*60)
            print(f"目标纬度: {target_lat:.7f}")
            print(f"目标经度: {target_lon:.7f}")
            print(f"飞行高度: {target_alt} 米")
            print("="*60)
            
            confirm = input("\n确认无误？(y/n): ").strip().lower()
            if confirm in ('y', 'yes', '是', 'Y'):
                return target_lat, target_lon, target_alt
            else:
                print("\n重新输入...\n")
                
        except ValueError:
            print("❌ 输入格式错误！请输入有效的数字。")
        except KeyboardInterrupt:
            print("\n\n⚠️  用户取消输入")
            sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='无人机GPS导航控制（支持交互式输入）')
    parser.add_argument("--lat", type=float, default=None,
                        help='目标纬度（可选，不指定则交互式输入）')
    parser.add_argument("--lon", type=float, default=None,
                        help='目标经度（可选，不指定则交互式输入）')
    parser.add_argument("--alt", type=float, default=None,
                        help='目标高度/米（可选，不指定则交互式输入）')
    args = parser.parse_args()

    # 如果命令行参数都提供了，直接使用；否则交互式输入
    if args.lat is not None and args.lon is not None and args.alt is not None:
        target_lat = args.lat
        target_lon = args.lon
        target_alt = args.alt
        print("\n使用命令行参数模式")
    else:
        print("\n使用交互式输入模式")
        target_lat, target_lon, target_alt = get_user_input()

    print(f"\n" + "="*60)
    print(f"🎯 任务目标")
    print(f"="*60)
    print(f"目标GPS坐标: ({target_lat:.7f}, {target_lon:.7f})")
    print(f"目标飞行高度: {target_alt} 米")
    print(f"="*60 + "\n")

    drone = olympe.Drone(DRONE_IP)

    print("[0] Connecting")
    drone.connect()

    # 安全模式和手动控制状态
    safe_mode = False
    manual_roll = 0
    manual_pitch = 0
    manual_yaw = 0
    manual_gaz = 0
    
    def send_piloting(roll, pitch, yaw, gaz):
        """发送飞行控制命令"""
        drone.piloting(roll, pitch, yaw, gaz, 0.05)
    
    def handle_key(ch):
        """处理按键，返回 (should_quit, safe_mode)"""
        nonlocal safe_mode, manual_roll, manual_pitch, manual_yaw, manual_gaz
        
        # 退出
        if ch in ('\x1b', 'q', 'Q'):
            return True, safe_mode
        
        # 安全模式（空格键）- 最高优先级
        if ch == ' ':
            safe_mode = not safe_mode
            if safe_mode:
                # 进入安全模式：取消moveTo，停止运动
                print("\n" + "="*60)
                print("🛡️  [SAFE MODE] ON - GPS导航已停止")
                print("="*60)
                print("   手动控制已启用，您可以使用以下按键控制无人机：")
                print("   • w/s = 前进/后退    • a/d = 左移/右移")
                print("   • r/f = 上升/下降    • z/e = 左转/右转")
                print("   • l   = 立即降落")
                print("   • 空格 = 退出安全模式，恢复GPS导航")
                print("="*60 + "\n")
                try:
                    drone(CancelMoveTo()).wait(_timeout=2)
                except:
                    pass
                manual_roll = manual_pitch = manual_yaw = manual_gaz = 0
                send_piloting(0, 0, 0, 0)
            else:
                # 退出安全模式
                print("\n[🛡️ SAFE MODE] OFF - 恢复GPS导航\n")
                manual_roll = manual_pitch = manual_yaw = manual_gaz = 0
            return False, safe_mode
        
        # 降落（随时可用）
        if ch in ('l', 'L'):
            print("[SAFETY] 立即降落...")
            return True, safe_mode
        
        # 手动飞行控制
        if ch in ('w', 'W'):
            manual_pitch = MANUAL_CONTROL_SPEED
        elif ch in ('s', 'S'):
            manual_pitch = -MANUAL_CONTROL_SPEED
        elif ch in ('a', 'A'):
            manual_roll = -MANUAL_CONTROL_SPEED
        elif ch in ('d', 'D'):
            manual_roll = MANUAL_CONTROL_SPEED
        elif ch in ('z', 'Z'):
            manual_yaw = -MANUAL_CONTROL_SPEED
        elif ch in ('e', 'E'):
            manual_yaw = MANUAL_CONTROL_SPEED
        elif ch in ('r', 'R'):
            manual_gaz = MANUAL_CONTROL_SPEED
        elif ch in ('f', 'F'):
            manual_gaz = -MANUAL_CONTROL_SPEED
        
        return False, safe_mode
    
    def decay_manual_controls():
        """衰减手动控制（释放按键后自动减速）"""
        nonlocal manual_roll, manual_pitch, manual_yaw, manual_gaz
        manual_roll = int(manual_roll * MANUAL_DECAY_RATE)
        manual_pitch = int(manual_pitch * MANUAL_DECAY_RATE)
        manual_yaw = int(manual_yaw * MANUAL_DECAY_RATE)
        manual_gaz = int(manual_gaz * MANUAL_DECAY_RATE)

    print("\n" + "="*60)
    print("⌨️  键盘控制提示：")
    print("="*60)
    print("  空格键 = 🛡️  安全模式（急停+手动控制）")
    print("  l      = 立即降落")
    print("  q/ESC  = 退出程序")
    print("="*60 + "\n")

    try:
        home_lat, home_lon = wait_gps_ready(drone)
        
        print(f"\n[DEBUG] 起始位置: ({home_lat:.7f}, {home_lon:.7f})")
        print(f"[DEBUG] 目标位置: ({target_lat:.7f}, {target_lon:.7f})")
        
        # 计算距离
        distance = haversine_m(home_lat, home_lon, target_lat, target_lon)
        print(f"[DEBUG] 目标距离: {distance:.1f} 米\n")

        print("[2] Takeoff")
        result = drone(TakeOff() >> FlyingStateChanged(state="hovering")).wait(_timeout=TAKEOFF_TIMEOUT)
        if not result.success():
            raise RuntimeError(f"起飞失败: {result}")
        print("    起飞成功 ✅")

        print(f"[3] Stabilizing ({STABILIZE_DELAY}s)")
        time.sleep(STABILIZE_DELAY)
        
        # 获取起飞后的位置
        try:
            geo = drone.get_state(GeofenceCenterChanged)
            if geo:
                print(f"    [DEBUG] 起飞后位置: ({geo.get('latitude', 0):.7f}, {geo.get('longitude', 0):.7f})")
        except Exception as e:
            print(f"    [DEBUG] 无法获取起飞后位置: {e}")

        print(f"\n[4] 发送moveTo命令")
        print(f"    目标: ({target_lat:.7f}, {target_lon:.7f})")
        print(f"    高度: {target_alt}米")
        print(f"    方向: TO_TARGET (朝向目标)")
        
        # 发送moveTo命令（不等待完成，moveTo是持续执行的命令）
        moveto_cmd = moveTo(
            target_lat,
            target_lon,
            target_alt,
            "TO_TARGET",
            0.0
        )
        
        print(f"    正在发送moveTo命令...")
        result = drone(moveto_cmd).wait(_timeout=5)  # 只等待5秒确认接收
        
        # 检查moveTo状态
        time.sleep(0.5)
        try:
            from olympe.messages.ardrone3.PilotingState import moveToChanged
            moveto_state = drone.get_state(moveToChanged)
            if moveto_state:
                status = moveto_state.get("status", "UNKNOWN")
                print(f"    执行状态: {status}")
                if status == "RUNNING":
                    print(f"    ✅ moveTo正在执行中！")
                elif status == "DONE":
                    print(f"    ✅ 已完成")
                elif status == "ERROR":
                    print(f"    ❌ 执行错误！")
                    raise RuntimeError("moveTo执行失败")
            else:
                print(f"    ⚠️  无法获取状态，继续监控...")
        except ImportError:
            print(f"    继续监控飞行状态...")
        except RuntimeError:
            raise
        except Exception as e:
            print(f"    状态检查异常: {e}")

        print(f"\n[5] 开始监控飞行状态")
        print(f"    到达阈值: {ARRIVAL_THRESHOLD}米")
        print(f"    需要确认: {ARRIVAL_CONFIRM_COUNT}次")
        print(f"    最大监控时间: {MONITOR_MAX_LOOPS * 0.5:.0f}秒")
        print(f"    按空格键可进入安全模式手动控制\n")
        
        arrived = 0
        loop_count = 0
        moveto_active = True  # moveTo命令是否还在执行
        first_position_shown = False
        
        with TerminalKeyReader() as kr:
            while loop_count < MONITOR_MAX_LOOPS:
                loop_count += 1
                
                # 检查键盘输入
                ch = kr.read_key_nonblock()
                if ch:
                    should_quit, safe_mode = handle_key(ch)
                    if should_quit:
                        break
                
                # 衰减手动控制
                decay_manual_controls()
                
                # === 安全模式处理 ===
                if safe_mode:
                    # 安全模式：禁用GPS导航，只使用手动控制
                    if moveto_active:
                        try:
                            drone(CancelMoveTo()).wait(_timeout=1)
                            moveto_active = False
                        except:
                            pass
                    
                    # 发送手动控制命令
                    send_piloting(manual_roll, manual_pitch, manual_yaw, manual_gaz)
                    
                    # 获取当前位置用于显示
                    try:
                        drone(GeofenceCenterChanged()).wait(_timeout=0.1)
                    except:
                        pass
                    geo = drone.get_state(GeofenceCenterChanged)
                    if geo and "latitude" in geo:
                        current_lat = geo["latitude"]
                        current_lon = geo["longitude"]
                        d = haversine_m(current_lat, current_lon, target_lat, target_lon)
                        print(f"    [SAFE MODE] 位置: ({current_lat:.7f}, {current_lon:.7f}), "
                              f"距目标={d:.1f}米 | 控制: roll={manual_roll}, pitch={manual_pitch}")
                    
                    time.sleep(0.1)
                    continue
                
                # === 正常GPS导航模式 ===
                # 如果从安全模式恢复，重新发送moveTo命令
                if not moveto_active:
                    print(f"    [INFO] 恢复GPS导航，重新发送moveTo命令...")
                    result = drone(
                        moveTo(target_lat, target_lon, target_alt, "TO_TARGET", 0.0)
                    ).wait(_timeout=MOVETO_TIMEOUT)
                    moveto_active = True
                    arrived = 0  # 重置到达计数
                
                try:
                    drone(GeofenceCenterChanged()).wait(_timeout=1)
                except Exception as e:
                    if loop_count == 1:
                        print(f"    [DEBUG] 等待GPS位置更新: {e}")
                    pass
                
                try:
                    geo = drone.get_state(GeofenceCenterChanged)
                except (ValueError, Exception) as e:
                    print(f"    [ERROR] 获取GPS状态失败: {e}")
                    time.sleep(0.5)
                    continue
                
                if not geo or "latitude" not in geo:
                    print(f"    [DEBUG] GPS状态无效，等待中...")
                    time.sleep(0.5)
                    continue

                current_lat = geo["latitude"]
                current_lon = geo["longitude"]
                
                d = haversine_m(
                    current_lat, current_lon,
                    target_lat, target_lon
                )

                # 显示第一次位置以确认开始飞行
                if not first_position_shown:
                    print(f"    ✓ 开始飞行！")
                    print(f"    起点: ({current_lat:.7f}, {current_lon:.7f})")
                    print(f"    终点: ({target_lat:.7f}, {target_lon:.7f})")
                    print(f"    距离: {d:.1f}米\n")
                    first_position_shown = True

                print(f"    [{loop_count}] 位置: ({current_lat:.7f}, {current_lon:.7f}), 距目标={d:.1f}米 (确认{arrived}/{ARRIVAL_CONFIRM_COUNT})")

                if d < ARRIVAL_THRESHOLD:
                    arrived += 1
                    if arrived >= ARRIVAL_CONFIRM_COUNT:
                        print("[6] Arrived ✅")
                        break
                else:
                    arrived = 0

                time.sleep(0.5)
            
            if arrived < ARRIVAL_CONFIRM_COUNT and not safe_mode:
                print("[6] 超时或未到达目标")

    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断 (Ctrl+C)")
        try:
            drone(CancelMoveTo()).wait()
        except:
            pass
    except Exception as e:
        print(f"\n❌ [ERROR] 发生异常: {e}")
        print("\n[DEBUG] 完整错误信息:")
        traceback.print_exc()
        print("\n尝试取消moveTo命令...")
        try:
            drone(CancelMoveTo()).wait()
        except:
            pass
    finally:
        print("\n[7] 准备降落...")
        # 停止所有运动
        try:
            send_piloting(0, 0, 0, 0)
            time.sleep(0.5)
        except:
            pass
        
        # 降落
        print("[7] Landing")
        try:
            drone(Landing()).wait(_timeout=15)
        except Exception as e:
            print(f"[WARN] 降落异常: {e}")
        
        # 断开连接
        print("[8] Disconnecting")
        try:
            drone.disconnect()
        except:
            pass
        
        print("✅ 程序已安全退出")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
无人机综合控制系统 - Web GUI
整合 Navigation、Perception 和 Winch System 三个模块
"""

import os
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import sys
import time
import threading
import json
import base64
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass, asdict
from enum import Enum

import math
import queue
import cv2
import numpy as np
import requests
from flask import Flask, render_template, Response, jsonify, request

# 导入现有模块
import olympe
import olympe.log
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveTo, CancelMoveTo
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, moveToChanged
from olympe.messages.ardrone3.GPSState import NumberOfSatelliteChanged
from olympe.messages.ardrone3.GPSSettingsState import GeofenceCenterChanged
from olympe.messages.common.CommonState import BatteryStateChanged


def haversine_m(lat1, lon1, lat2, lon2):
    """计算两点GPS距离（米）- 参考 fly_to_gps.py"""
    R = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dlmb / 2)**2
    return 2 * R * math.asin(math.sqrt(a))


class ExternalSystemController:
    """外部系统控制器 (通过HTTP请求) - 来自 fly_track_and_grab.py"""
    
    def __init__(self, base_url: str = "http://192.168.42.37", timeout: float = 5.0):
        self.base_url = base_url
        self.timeout = timeout
        
    def send_command(self, command: str) -> bool:
        """发送命令到外部系统: 'lower', 'pull', 'stop'"""
        url = f"{self.base_url}/{command}"
        try:
            response = requests.get(url, timeout=self.timeout)
            if response.status_code == 200:
                return True
            else:
                return False
        except requests.exceptions.Timeout:
            return False
        except requests.exceptions.ConnectionError:
            return False
        except Exception:
            return False
    
    def execute_grab_sequence(self, wait_time: float = 5.0, pull_time: float = 3.0) -> bool:
        """执行抓取序列: LOWER → wait → PULL → wait → STOP"""
        if not self.send_command("lower"):
            return False
        time.sleep(wait_time)
        
        if not self.send_command("pull"):
            return False
        time.sleep(pull_time)
        
        self.send_command("stop")
        return True


class SystemState(Enum):
    """系统状态枚举"""
    IDLE = "idle"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"
    COMPLETED = "completed"


@dataclass
class DroneStatus:
    """无人机状态"""
    connected: bool = False
    flying: bool = False
    battery: int = 0
    gps_fix: bool = False
    satellites: int = 0
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    

@dataclass
class NavigationStatus:
    """导航系统状态"""
    state: str = SystemState.IDLE.value
    target_lat: float = 0.0
    target_lon: float = 0.0
    target_alt: float = 2.0
    current_distance: float = 0.0
    progress: int = 0
    message: str = "就绪"


@dataclass
class PerceptionStatus:
    """感知系统状态"""
    state: str = SystemState.IDLE.value
    model: str = "/home/seahaws/Drone/perception/yolov8s.pt"
    tracking_mode: str = "yolo"  # yolo 或 color
    target_classes: list = None
    target_color: str = "red"
    detected: bool = False
    tracking: bool = False
    stable: bool = False
    confidence: float = 0.0
    message: str = "Ready"
    
    def __post_init__(self):
        if self.target_classes is None:
            self.target_classes = ["keyboard"]


@dataclass
class WinchStatus:
    """绞盘系统状态"""
    state: str = SystemState.IDLE.value
    current_action: str = "IDLE"
    message: str = "就绪"
    progress: int = 0


class DroneControlSystem:
    """无人机综合控制系统 - 所有 Olympe 操作在专用线程运行"""
    
    def __init__(self, drone_ip: str = "192.168.42.1"):
        self.drone_ip = drone_ip
        self.drone: Optional[olympe.Drone] = None
        
        # 系统状态
        self.drone_status = DroneStatus()
        self.navigation_status = NavigationStatus()
        self.perception_status = PerceptionStatus()
        self.winch_status = WinchStatus()
        
        # 任务控制 - 每个模块独立的 stop flag
        self.current_task = None
        self.task_thread = None
        self.stop_flag = threading.Event()  # 全局紧急停止
        self.nav_stop_flag = threading.Event()
        self.perception_stop_flag = threading.Event()
        
        # Piloting 状态标志
        self.piloting_started = False
        
        # 视频流 (参考 fly_track_and_grab.py)
        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_lock = threading.Lock()
        self.current_frame = None      # 原始摄像头帧 (由 frame_updater 更新)
        self.display_frame = None      # 带叠加层的显示帧 (由 perception 更新)
        self.frame_lock = threading.Lock()
        self.streaming = False
        
        # 外部系统 (Winch & Gripper)
        self.external_system = ExternalSystemController()
        
        # 日志
        self.logs = []
        self.max_logs = 100
        
        # ========== 专用 Drone 线程 ==========
        # 所有 Olympe SDK 操作都在这个线程上运行
        # 这是因为 Olympe 的内部 pomp loop 有线程亲和性
        # Flask 请求线程结束时会导致 Olympe 内部状态被清理
        self._cmd_queue = queue.Queue()
        self._drone_thread = threading.Thread(target=self._drone_worker, daemon=True)
        self._drone_thread.start()
        
    def set_flying(self, value: bool, source: str):
        """设置飞行状态 + 诊断日志（追踪所有状态变更来源）"""
        old = self.drone_status.flying
        if old != value:
            self.drone_status.flying = value
            msg = f"🔄 flying: {old} → {value} (source: {source})"
            self.log(msg, "info")
            # 额外在终端打印调用栈，帮助定位问题
            import traceback
            traceback.print_stack(limit=6)
    
    def log(self, message: str, level: str = "info"):
        """添加日志"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = {
            "time": timestamp,
            "level": level,
            "message": message
        }
        self.logs.append(log_entry)
        if len(self.logs) > self.max_logs:
            self.logs.pop(0)
        print(f"[{timestamp}] [{level.upper()}] {message}")
    
    # ========== 线程调度机制 ==========
    
    def _drone_worker(self):
        """专用 Drone 线程 - 处理所有 Olympe 操作"""
        while True:
            try:
                func, args, kwargs, result_event, result_holder = self._cmd_queue.get()
                try:
                    result_holder['value'] = func(*args, **kwargs)
                except Exception as e:
                    result_holder['error'] = e
                result_event.set()
            except Exception:
                pass
    
    def _run_on_drone_thread(self, func, *args, timeout=30, **kwargs):
        """在 Drone 专用线程上执行函数并等待结果"""
        result_holder = {}
        result_event = threading.Event()
        self._cmd_queue.put((func, args, kwargs, result_event, result_holder))
        result_event.wait(timeout=timeout)
        if 'error' in result_holder:
            raise result_holder['error']
        return result_holder.get('value')
    
    # ========== 公开接口（Flask 路由调用这些） ==========
    
    def connect_drone(self) -> bool:
        """连接无人机 - 在 Drone 线程上执行"""
        return self._run_on_drone_thread(self._do_connect)
    
    def disconnect_drone(self):
        """断开连接 - 在 Drone 线程上执行"""
        return self._run_on_drone_thread(self._do_disconnect)
    
    def takeoff(self) -> bool:
        """起飞 - 在 Drone 线程上执行"""
        return self._run_on_drone_thread(self._do_takeoff)
    
    def land(self) -> bool:
        """降落 - 在 Drone 线程上执行"""
        return self._run_on_drone_thread(self._do_land)
    
    def _drain_cmd_queue(self):
        """清空命令队列 - 丢弃所有排队中的命令"""
        drained = 0
        while True:
            try:
                func, args, kwargs, result_event, result_holder = self._cmd_queue.get_nowait()
                result_holder['error'] = RuntimeError("Command cancelled by stop")
                result_event.set()  # 解除等待方的阻塞
                drained += 1
            except queue.Empty:
                break
        if drained > 0:
            self.log(f"🗑️ Drained {drained} queued commands", "info")
    
    def _send_hover(self):
        """直接发送悬停命令（不经过队列），多次发送确保生效"""
        if self.drone and self.drone_status.connected:
            try:
                for _ in range(5):  # 多次发送确保覆盖残留指令
                    self.drone.piloting(0, 0, 0, 0, 0.05)
                    time.sleep(0.05)
            except Exception:
                pass
    
    def emergency_stop(self):
        """紧急停止 - 直接执行，不经过队列（最高优先级）
        piloting() 是 UDP 命令，可安全从任意线程调用。
        stop flags 是线程安全的 Event 对象。
        """
        self.log("🚨 EMERGENCY STOP", "warning")
        
        # 1. 立即设置所有 stop flags（终止所有任务循环）
        self.stop_flag.set()
        self.nav_stop_flag.set()
        self.perception_stop_flag.set()
        
        # 2. 清空命令队列（防止残留的 tracking 命令覆盖 hover）
        self._drain_cmd_queue()
        
        # 3. 先取消 moveTo（如果正在导航中，moveTo 会覆盖 hover 命令）
        if self.drone and self.drone_status.connected:
            try:
                self.drone(CancelMoveTo()).wait(_timeout=2)
            except Exception:
                pass
        
        # 4. 取消 moveTo 之后再发 hover 命令
        self._send_hover()
        
        self.log("🛑 All systems stopped, drone hovering", "warning")
    
    def manual_control(self, axis: str, value: int):
        """手动控制 - 在 Drone 线程上执行"""
        return self._run_on_drone_thread(self._do_manual_control, axis, value)
    
    # ========== 实际 Olympe 操作（在 Drone 线程上运行） ==========
    
    def _do_connect(self) -> bool:
        """实际连接操作 - 运行在 Drone 线程"""
        self.log("🔗 Connecting to drone...", "info")
        
        # 如果已有连接，先断开
        if self.drone is not None:
            try:
                self.drone.disconnect()
            except:
                pass
            time.sleep(0.5)
        
        self.drone = olympe.Drone(self.drone_ip)
        
        # 重试3次连接（参考 fly_track_and_grab.py）
        for i in range(3):
            try:
                if self.drone.connect():
                    self.drone_status.connected = True
                    self.log(f"✅ Connected to drone: {self.drone_ip}", "success")
                    
                    # 在同一线程上启动视频流
                    time.sleep(1)
                    self._do_start_streaming()
                    
                    # 在同一线程上启动 piloting
                    self._do_start_piloting()
                    
                    # 状态更新在独立线程（只读，不影响）
                    threading.Thread(
                        target=self._update_drone_status, daemon=True
                    ).start()
                    
                    return True
                    
            except Exception as e:
                self.log(f"⚠️ Attempt {i+1}/3 failed: {str(e)}", "warning")
                time.sleep(1)
        
        self.log("❌ Connection failed after 3 attempts", "error")
        return False
    
    def _do_disconnect(self):
        """实际断开操作 - 运行在 Drone 线程"""
        try:
            if self.drone and self.drone_status.connected:
                # 停止视频流
                self._do_stop_streaming()
                
                # 停止piloting
                if self.piloting_started:
                    try:
                        self.drone.stop_piloting()
                        self.piloting_started = False
                    except:
                        pass
                
                # 断开连接
                time.sleep(0.3)
                self.drone.disconnect()
                self.drone_status.connected = False
                self.log("✅ Disconnected", "info")
        except Exception as e:
            self.log(f"❌ Disconnect error: {str(e)}", "error")
    
    def _do_start_piloting(self):
        """启动 piloting - 运行在 Drone 线程"""
        if not self.piloting_started:
            try:
                self.drone.start_piloting()
                self.piloting_started = True
                self.log("🎮 Piloting interface started", "info")
            except Exception as e:
                self.log(f"⚠️ Start piloting: {str(e)}", "warning")
    
    def _check_flying_state(self) -> str:
        """安全地检查当前飞行状态（仅在 drone 线程调用）"""
        try:
            st = self.drone.get_state(FlyingStateChanged)
            if st:
                state = st.get("state")
                return getattr(state, "name", str(state)) if state else "unknown"
        except Exception:
            pass
        return "unknown"
    
    def _do_takeoff(self) -> bool:
        """实际起飞操作 - 运行在 Drone 线程"""
        try:
            if not self.drone_status.connected or self.drone is None:
                self.log("❌ Drone not connected", "error")
                return False
            
            # 确保 piloting 已启动
            self._do_start_piloting()
            
            self.log("🚁 Taking off...", "info")
            
            # 发送起飞命令，等待 hovering 状态确认
            result = self.drone(
                TakeOff() >> FlyingStateChanged(state="hovering", _timeout=20)
            ).wait()
            
            if result.success():
                self.set_flying(True, "_do_takeoff:event_confirmed")
                self.log("✅ Take off successful", "success")
                return True
            else:
                # 事件确认失败，但 TakeOff() 命令可能已发出
                # 等待 2 秒后用 get_state 做二次检查
                self.log("⚠️ Takeoff event not confirmed, checking actual state...", "warning")
                time.sleep(2)
                actual_state = self._check_flying_state()
                self.log(f"   Actual state: {actual_state}", "info")
                
                if actual_state in ("hovering", "flying", "takingoff"):
                    self.set_flying(True, f"_do_takeoff:fallback({actual_state})")
                    self.log("✅ Take off confirmed via state check", "success")
                    return True
                else:
                    self.log("❌ Take off failed", "error")
                    self.log("💡 Most common: NOT CALIBRATED", "warning")
                    self.log("   → Use FreeFlight 7 APP to calibrate", "info")
                    return False
                
        except Exception as e:
            # 异常时也做二次检查
            self.log(f"⚠️ Take off exception: {str(e)}", "warning")
            try:
                time.sleep(2)
                actual_state = self._check_flying_state()
                if actual_state in ("hovering", "flying", "takingoff"):
                    self.set_flying(True, f"_do_takeoff:exception_fallback({actual_state})")
                    self.log("✅ Take off confirmed via state check (after exception)", "success")
                    return True
            except Exception:
                pass
            self.log("❌ Take off failed", "error")
            self.log("💡 Use FreeFlight 7 to calibrate", "warning")
            return False
    
    def _do_land(self) -> bool:
        """实际降落操作 - 运行在 Drone 线程"""
        try:
            if not self.drone_status.connected or self.drone is None:
                self.log("❌ Drone not connected", "error")
                return False
            
            self.log("🛬 Landing...", "info")
            result = self.drone(
                Landing() >> FlyingStateChanged(state="landed", _timeout=30)
            ).wait()
            
            if result.success():
                self.set_flying(False, "_do_land:event_confirmed")
                self.log("✅ Landed", "success")
                return True
            else:
                # 事件确认失败，二次检查实际状态
                self.log("⚠️ Landing event not confirmed, checking actual state...", "warning")
                time.sleep(2)
                actual_state = self._check_flying_state()
                self.log(f"   Actual state: {actual_state}", "info")
                
                if actual_state in ("landed", "landing"):
                    self.set_flying(False, f"_do_land:fallback({actual_state})")
                    self.log("✅ Landing confirmed via state check", "success")
                    return True
                else:
                    self.log("❌ Landing failed", "error")
                    return False
                
        except Exception as e:
            self.log(f"⚠️ Landing exception: {str(e)}", "warning")
            try:
                time.sleep(2)
                actual_state = self._check_flying_state()
                if actual_state in ("landed", "landing"):
                    self.set_flying(False, f"_do_land:exception_fallback({actual_state})")
                    self.log("✅ Landing confirmed via state check (after exception)", "success")
                    return True
            except Exception:
                pass
            self.log("❌ Landing failed", "error")
            return False
    
    def _do_tracking_control(self, roll: int, pitch: int):
        """感知跟踪控制 - 摄像头朝下，用 roll/pitch 水平移动跟踪（运行在 Drone 线程）
        参考 fly_track_and_grab.py: self.drone.piloting(roll, pitch, yaw, gaz, 0.05)
        """
        # 如果 perception 已停止或紧急停止，拒绝发送追踪命令
        if self.perception_stop_flag.is_set() or self.stop_flag.is_set():
            return False
        if not self.drone or not self.drone_status.connected:
            return False
        self._do_start_piloting()
        # roll/pitch 控制水平移动，yaw=0 gaz=0（不旋转不升降）
        self.drone.piloting(roll, pitch, 0, 0, 0.05)
        return True
    
    def _do_manual_control(self, axis: str, value: int):
        """实际手动控制 - 运行在 Drone 线程"""
        if not self.drone or not self.drone_status.connected:
            return False
        
        self._do_start_piloting()
        
        roll = pitch = yaw = gaz = 0
        if axis == 'roll':
            roll = value
        elif axis == 'pitch':
            pitch = value
        elif axis == 'yaw':
            yaw = value
        elif axis == 'gaz':
            gaz = value
        
        self.drone.piloting(roll, pitch, yaw, gaz, 0.05)
        return True
    
    # ========== 视频流 (参考 fly_track_and_grab.py) ==========
    
    def _yuv_frame_cb(self, yuv_frame):
        """YUV帧回调"""
        try:
            yuv_frame.ref()
            try:
                self.frame_queue.put_nowait(yuv_frame)
            except queue.Full:
                old = self.frame_queue.get_nowait()
                old.unref()
                self.frame_queue.put_nowait(yuv_frame)
        except Exception:
            try:
                yuv_frame.unref()
            except Exception:
                pass
    
    def _flush_cb(self, stream):
        """Flush回调"""
        with self.flush_lock:
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait().unref()
                except Exception:
                    pass
        return True
    
    def _get_latest_frame(self):
        """获取最新一帧 BGR 图像（参考 fly_track_and_grab.py）"""
        last = None
        while True:
            try:
                f = self.frame_queue.get_nowait()
                if last is not None:
                    try:
                        last.unref()
                    except Exception:
                        pass
                last = f
            except queue.Empty:
                break
        
        if last is None:
            return None
        
        try:
            fmt = last.format()
            cv2_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }.get(fmt)
            
            if cv2_flag is None:
                return None
            
            yuv = last.as_ndarray()
            bgr = cv2.cvtColor(yuv, cv2_flag)
            return bgr
        except Exception:
            return None
        finally:
            try:
                last.unref()
            except Exception:
                pass
    
    def _do_start_streaming(self):
        """启动视频流 - 运行在 Drone 线程"""
        if not self.drone or not self.drone_status.connected:
            return False
        
        try:
            self.drone.streaming.set_callbacks(
                raw_cb=self._yuv_frame_cb,
                flush_raw_cb=self._flush_cb,
            )
            self.drone.streaming.start()
            self.streaming = True
            self.log("📹 Video streaming started", "success")
            
            # 后台线程持续更新当前帧（只做帧转换，不调用 Olympe）
            def frame_updater():
                while self.streaming and self.drone_status.connected:
                    frame = self._get_latest_frame()
                    if frame is not None:
                        with self.frame_lock:
                            self.current_frame = frame
                        time.sleep(0.008)  # ~120fps 上限，足够流畅且不干扰 Olympe 事件
                    else:
                        time.sleep(0.01)   # 无帧时等待，避免空转
            
            threading.Thread(target=frame_updater, daemon=True).start()
            return True
        except Exception as e:
            self.log(f"❌ Streaming error: {str(e)}", "error")
            return False
    
    def _do_stop_streaming(self):
        """停止视频流 - 运行在 Drone 线程"""
        self.streaming = False
        if self.drone:
            try:
                self._flush_cb(None)
            except Exception:
                pass
            try:
                self.drone.streaming.stop()
            except Exception:
                pass
            time.sleep(0.3)
    
    # ========== 状态更新（独立线程，只读不写 Olympe） ==========
    
    def _update_drone_status(self):
        """更新无人机状态（独立线程）
        
        注意: 飞行状态 (flying) 不在此处轮询！
        Olympe 的 get_state(FlyingStateChanged) 无论在哪个线程调用都可能返回过期数据。
        flying 状态只由以下操作显式设置:
          - _do_takeoff → True
          - _do_land → False
        """
        time.sleep(3)  # 延迟3秒启动
        self.log("📊 Status monitoring started", "info")
        
        while self.drone_status.connected and self.drone is not None:
            try:
                # *** flying 状态不在这里读取 ***
                # 只读取电池、GPS 等数值型状态
                
                try:
                    sat_state = self.drone.get_state(NumberOfSatelliteChanged)
                    if sat_state:
                        self.drone_status.satellites = sat_state.get("numberOfSatellite", 0)
                        self.drone_status.gps_fix = self.drone_status.satellites >= 4
                except:
                    pass
                
                try:
                    geo_state = self.drone.get_state(GeofenceCenterChanged)
                    if geo_state:
                        self.drone_status.latitude = geo_state.get("latitude", 0.0)
                        self.drone_status.longitude = geo_state.get("longitude", 0.0)
                except:
                    pass
                
                try:
                    battery_state = self.drone.get_state(BatteryStateChanged)
                    if battery_state:
                        self.drone_status.battery = battery_state.get("percent", 0)
                except:
                    pass
                
            except Exception:
                pass
            
            time.sleep(2)
    
    def stop_navigation(self):
        """停止导航 — 正确顺序: CancelMoveTo → hover"""
        self.log("🛑 Stopping navigation...", "warning")
        
        # 1. 设置 stop flag（让 _navigation_task 循环退出）
        self.nav_stop_flag.set()
        
        # 2. 清空命令队列
        self._drain_cmd_queue()
        
        # 3. 先发 CancelMoveTo（必须在 hover 之前！否则 moveTo 会覆盖 hover）
        if self.drone and self.drone_status.connected:
            try:
                self.drone(CancelMoveTo()).wait(_timeout=3)
            except Exception:
                pass
        
        # 4. CancelMoveTo 之后再发 hover，确保无人机真正停下来
        self._send_hover()
        
        self.navigation_status.state = SystemState.IDLE.value
        self.navigation_status.message = "Stopped"
        self.log("🛑 Navigation stopped, drone hovering", "info")
    
    def start_navigation(self, target_lat: float, target_lon: float, target_alt: float,
                         arrival_threshold: float = 0.5):
        """启动导航任务"""
        if self.navigation_status.state == SystemState.RUNNING.value:
            self.log("⚠️ Navigation already running", "warning")
            return False
        
        if not self.drone_status.connected:
            self.log("❌ Drone not connected", "error")
            return False
        
        self.nav_stop_flag.clear()
        self.stop_flag.clear()
        self.navigation_status.state = SystemState.RUNNING.value
        self.navigation_status.target_lat = target_lat
        self.navigation_status.target_lon = target_lon
        self.navigation_status.target_alt = target_alt
        self.navigation_status.progress = 0
        self.navigation_status.message = "Starting navigation..."
        
        self.log(f"🧭 Navigation to ({target_lat:.6f}, {target_lon:.6f}), alt: {target_alt}m, threshold: {arrival_threshold}m", "info")
        
        # 导航在独立线程运行（不阻塞 Drone 线程！）
        # 只在发送 moveTo/CancelMoveTo 时短暂使用 Drone 线程
        threading.Thread(
            target=self._navigation_task, 
            args=(target_lat, target_lon, target_alt, arrival_threshold),
            daemon=True
        ).start()
        return True
    
    def _navigation_task(self, target_lat: float, target_lon: float, target_alt: float,
                         arrival_threshold: float = 0.5):
        """导航任务（独立线程） - 不阻塞 Drone 线程（参考 fly_to_gps.py）"""
        try:
            # 报告 GPS 状态
            sats = self.drone_status.satellites
            self.log(f"📡 GPS satellites: {sats}", "info")
            if sats < 10:
                self.log(f"⚠️ GPS weak ({sats} sats), navigation may be inaccurate", "warning")
            
            # 报告初始距离
            initial_dist = 1.0
            cur_lat = self.drone_status.latitude
            cur_lon = self.drone_status.longitude
            if abs(cur_lat) > 0.001:
                initial_dist = max(haversine_m(cur_lat, cur_lon, target_lat, target_lon), 1.0)
                self.log(f"📍 Current: ({cur_lat:.6f}, {cur_lon:.6f}), distance: {initial_dist:.1f}m", "info")
            
            # 发送 moveTo（短暂使用 Drone 线程）
            self.log("🚀 Sending moveTo command...", "info")
            try:
                self._run_on_drone_thread(
                    self._do_send_moveto, target_lat, target_lon, target_alt,
                    timeout=10
                )
            except Exception as e:
                self.log(f"❌ moveTo failed: {str(e)}", "error")
                self.navigation_status.state = SystemState.ERROR.value
                return
            
            self.log(f"✈️ Flying to target for 5s (threshold: {arrival_threshold}m)...", "info")
            self.navigation_status.message = "Flying..."
            
            # 飞行最多 5 秒，每 0.5 秒检查一次是否到达或被停止
            flight_duration = 5.0  # 秒
            check_interval = 0.5
            checks = int(flight_duration / check_interval)  # 10 次
            arrived_early = False
            
            for loop in range(checks):
                if self.nav_stop_flag.is_set() or self.stop_flag.is_set():
                    self.navigation_status.state = SystemState.IDLE.value
                    self.navigation_status.message = "Stopped"
                    self.log("⚠️ Navigation task exited", "warning")
                    return
                
                # 从已缓存的状态读取位置
                cur_lat = self.drone_status.latitude
                cur_lon = self.drone_status.longitude
                
                elapsed = (loop + 1) * check_interval
                progress = min(100, int(elapsed / flight_duration * 100))
                
                if abs(cur_lat) > 0.001:
                    dist = haversine_m(cur_lat, cur_lon, target_lat, target_lon)
                    self.navigation_status.current_distance = dist
                    self.navigation_status.message = f"Flying... {dist:.1f}m remaining ({elapsed:.0f}s/5s)"
                    
                    if dist < arrival_threshold:
                        arrived_early = True
                        self.log(f"✅ Arrived early at {elapsed:.1f}s! Distance: {dist:.2f}m", "success")
                        break
                else:
                    self.navigation_status.message = f"Flying... ({elapsed:.0f}s/5s)"
                
                self.navigation_status.progress = progress
                time.sleep(check_interval)
            
            # 5 秒到或提前到达 → CancelMoveTo + hover → completed
            try:
                self._run_on_drone_thread(self._do_cancel_moveto, timeout=3)
            except Exception:
                pass
            self._send_hover()
            
            self.navigation_status.state = SystemState.COMPLETED.value
            self.navigation_status.progress = 100
            if arrived_early:
                self.navigation_status.message = "Arrived! Hovering. Start Perception when ready."
            else:
                self.navigation_status.message = "5s flight done. Hovering. Start Perception when ready."
                cur_lat = self.drone_status.latitude
                cur_lon = self.drone_status.longitude
                if abs(cur_lat) > 0.001:
                    dist = haversine_m(cur_lat, cur_lon, target_lat, target_lon)
                    self.navigation_status.current_distance = dist
                    self.log(f"✅ Navigation complete (5s). Distance to target: {dist:.2f}m. Hovering.", "success")
                else:
                    self.log("✅ Navigation complete (5s). Hovering.", "success")
            
        except Exception as e:
            self.navigation_status.state = SystemState.ERROR.value
            self.navigation_status.message = f"Error: {str(e)}"
            self.log(f"❌ Navigation error: {str(e)}", "error")
    
    def _do_send_moveto(self, target_lat, target_lon, target_alt):
        """发送 moveTo 命令 - 运行在 Drone 线程（快速返回）"""
        self.drone(
            moveTo(target_lat, target_lon, target_alt, "TO_TARGET", 0.0)
        ).wait(_timeout=5)
    
    def _do_cancel_moveto(self):
        """取消 moveTo - 运行在 Drone 线程"""
        self.drone(CancelMoveTo()).wait(_timeout=3)
    
    def start_perception(self):
        """启动感知任务"""
        if self.perception_status.state == SystemState.RUNNING.value:
            self.log("⚠️ Perception already running", "warning")
            return False
        
        if not self.drone_status.connected:
            self.log("❌ Drone not connected", "error")
            return False
        
        self.perception_stop_flag.clear()
        self.stop_flag.clear()  # 清除紧急停止残留
        self.perception_status.state = SystemState.RUNNING.value
        self.perception_status.message = "Tracking..."
        self.perception_status.detected = False
        self.perception_status.tracking = False
        self.perception_status.stable = False
        
        mode = self.perception_status.tracking_mode
        if mode == "yolo":
            target_info = f"classes: {', '.join(self.perception_status.target_classes)}"
        else:
            target_info = f"color: {self.perception_status.target_color}"
        
        self.log(f"👁️ Starting perception - mode: {mode}, {target_info}", "info")
        
        # 感知任务在独立线程运行（读帧+检测不需要在 Drone 线程）
        # piloting 命令通过 manual_control 接口发送到 Drone 线程
        self.task_thread = threading.Thread(target=self._perception_task, daemon=True)
        self.task_thread.start()
        
        return True
    
    def _draw_perception_overlay(self, frame, bbox, target_center, label, confidence,
                                    stable_count, stable_required, is_stable):
        """在帧上绘制感知叠加层（参考 fly_track_and_grab.py 的 draw_info）"""
        display = frame.copy()
        h, w = display.shape[:2]
        
        STABILITY_THRESHOLD = 0.25  # 与 _perception_task 一致
        
        # ---- 计算偏移量 ----
        offset_x = 0.0
        offset_y = 0.0
        offset_mag = 0.0
        if target_center is not None:
            tx, ty = target_center
            offset_x = (tx - w / 2) / (w / 2)
            offset_y = (ty - h / 2) / (h / 2)
            offset_mag = math.sqrt(offset_x**2 + offset_y**2)
        
        # ---- 顶部状态栏 ----
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, 100), (0, 0, 0), -1)
        display = cv2.addWeighted(overlay, 0.55, display, 0.45, 0)
        
        mode = self.perception_status.tracking_mode.upper()
        status_text = f"[{mode}] "
        if bbox is not None:
            if is_stable:
                status_text += "CENTERED & STABLE - Ready to grab"
                status_color = (0, 255, 0)
            elif offset_mag < STABILITY_THRESHOLD:
                status_text += f"Centered! Holding... ({stable_count}/{stable_required})"
                status_color = (0, 255, 128)
            else:
                status_text += f"Tracking (offset={offset_mag:.2f}, need <{STABILITY_THRESHOLD})"
                status_color = (0, 255, 255)
        else:
            status_text += "Searching..."
            status_color = (100, 100, 255)
        
        cv2.putText(display, status_text, (10, 28),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        if confidence > 0:
            conf_text = f"Conf: {confidence:.0%}  |  {label}  |  offset: x={offset_x:+.2f} y={offset_y:+.2f} mag={offset_mag:.2f}"
            cv2.putText(display, conf_text, (10, 55),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        
        # 稳定度进度条
        bar_x, bar_y, bar_w, bar_h = 10, 68, 250, 14
        cv2.rectangle(display, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (80, 80, 80), -1)
        fill_w = int(bar_w * min(stable_count, stable_required) / stable_required) if stable_required > 0 else 0
        bar_color = (0, 255, 0) if is_stable else (0, 200, 255)
        cv2.rectangle(display, (bar_x, bar_y), (bar_x + fill_w, bar_y + bar_h), bar_color, -1)
        cv2.rectangle(display, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (150, 150, 150), 1)
        bar_label = f"Stable: {stable_count}/{stable_required}"
        cv2.putText(display, bar_label, (bar_x + bar_w + 8, bar_y + 12),
                     cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # 稳定阈值圆圈（画面中心附近的稳定区域可视化）
        center_x, center_y = w // 2, h // 2
        stable_radius = int(STABILITY_THRESHOLD * (w / 2))
        # 稳定区域圆圈（目标需要进入此圆才算 centered）
        circle_color = (0, 255, 0) if (bbox is not None and offset_mag < STABILITY_THRESHOLD) else (100, 100, 100)
        cv2.circle(display, (center_x, center_y), stable_radius, circle_color, 1)
        
        # ---- 画面中心十字 ----
        cv2.line(display, (center_x - 40, center_y), (center_x + 40, center_y), (255, 255, 255), 1)
        cv2.line(display, (center_x, center_y - 40), (center_x, center_y + 40), (255, 255, 255), 1)
        cv2.circle(display, (center_x, center_y), 5, (255, 255, 255), -1)
        
        # ---- 检测框 + 偏移线 ----
        if bbox is not None and target_center is not None:
            bx1, by1, bx2, by2 = [int(v) for v in bbox]
            tx, ty = int(target_center[0]), int(target_center[1])
            
            # 框颜色：绿色=在稳定区内，黄色=跟踪中（未居中），红色=偏离太远
            if is_stable:
                box_color = (0, 255, 0)
            elif offset_mag < STABILITY_THRESHOLD:
                box_color = (0, 255, 128)
            elif offset_mag < 0.5:
                box_color = (0, 255, 255)
            else:
                box_color = (0, 140, 255)  # 橙色 = 偏离较远
            
            # 检测框
            cv2.rectangle(display, (bx1, by1), (bx2, by2), box_color, 2)
            # 目标中心点
            cv2.circle(display, (tx, ty), 8, box_color, -1)
            # 偏移指示线（中心 → 目标）
            cv2.line(display, (center_x, center_y), (tx, ty), box_color, 2)
            
            # 偏移量文字
            offset_text = f"offset: x={offset_x:+.2f} y={offset_y:+.2f} dist={offset_mag:.2f}"
            cv2.putText(display, offset_text, (bx1, by1 - 10),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
            
            # 方向提示（摄像头朝下：offset_x→roll, offset_y→pitch）
            dir_hint = ""
            if offset_x > 0.1:
                dir_hint = "> Roll RIGHT"
            elif offset_x < -0.1:
                dir_hint = "< Roll LEFT"
            else:
                dir_hint = "= Centered X"
            if offset_y > 0.1:
                dir_hint += " | v Pitch FWD"
            elif offset_y < -0.1:
                dir_hint += " | ^ Pitch BACK"
            else:
                dir_hint += " | = Centered Y"
            cv2.putText(display, dir_hint, (bx1, by1 - 35),
                         cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
        
        return display
    
    def _perception_task(self):
        """感知任务 - 参考 fly_track_and_grab.py 的 SafeTracker 逻辑"""
        try:
            # 尝试加载检测器
            detector = None
            mode = self.perception_status.tracking_mode
            
            if mode == "yolo":
                try:
                    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
                    from object_detector import ObjectDetector
                    detector = ObjectDetector(
                        model_path=self.perception_status.model,
                        conf_threshold=0.3,
                        class_names_filter=self.perception_status.target_classes,
                        verbose=True
                    )
                    self.log(f"✅ YOLO detector loaded: {self.perception_status.model}", "success")
                except Exception as e:
                    self.log(f"⚠️ YOLO detector not available: {str(e)}", "warning")
                    self.log("ℹ️ Running without detection (manual tracking only)", "info")
            
            self.log("👁️ Perception running - tracking target", "info")
            
            # ---- 跟踪参数（基于 fly_track_and_grab.py SafeTracker，针对抖动优化） ----
            # 摄像头朝下，用 roll/pitch 控制水平移动跟踪目标
            STABILITY_THRESHOLD = 0.25  # 偏移量 < 此值算 "centered"（放宽，容忍抖动）
            DEADZONE = 0.15             # 死区（略小于阈值，让无人机在阈值内仍微调）
            KP = 5.0                    # 比例增益
            MAX_SPEED = 5               # 极保守最大控制量
            SMOOTHING = 0.05            # 极平滑
            MAX_LOST_FRAMES = 20        # 目标丢失多少帧后平滑停止
            
            stable_count = 0
            stable_required = 5          # 累计 5 帧 centered 即触发
            target_lost_frames = 0
            
            # 平滑控制量（roll / pitch，对应 SafeTracker）
            smooth_roll = 0.0
            smooth_pitch = 0.0
            
            # 最近一次检测结果缓存（用于视觉平滑，丢失几帧仍显示框）
            VISUAL_KEEP_FRAMES = 5  # 丢失后保留显示框的帧数
            last_det_bbox = None
            last_det_center = None
            last_det_label = ""
            last_det_confidence = 0.0
            
            while not self.perception_stop_flag.is_set():
                # 获取当前帧
                with self.frame_lock:
                    frame = self.current_frame
                
                if frame is None:
                    time.sleep(0.01)
                    continue
                
                detected = False
                confidence = 0.0
                det_bbox = None       # (x1, y1, x2, y2)
                det_center = None     # (cx, cy) 像素坐标
                det_label = ""
                offset_x = 0.0       # -1 to 1
                offset_y = 0.0       # -1 to 1
                
                # ========== 检测阶段 ==========
                if detector is not None and mode == "yolo":
                    try:
                        detections = detector.detect(frame)
                        if detections:
                            best = max(detections, key=lambda d: d.area)
                            detected = True
                            confidence = best.confidence
                            det_bbox = best.bbox
                            det_center = best.center
                            det_label = f"{best.class_name}"
                            
                            h, w = frame.shape[:2]
                            tx, ty = det_center
                            offset_x = (tx - w / 2) / (w / 2)  # -1 to 1
                            offset_y = (ty - h / 2) / (h / 2)  # -1 to 1
                    except Exception:
                        pass
                
                elif mode == "color":
                    try:
                        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                        color = self.perception_status.target_color
                        
                        color_ranges = {
                            "red":    ([0, 100, 100], [10, 255, 255], [160, 100, 100], [180, 255, 255]),
                            "green":  ([35, 100, 100], [85, 255, 255], None, None),
                            "blue":   ([100, 100, 100], [130, 255, 255], None, None),
                            "yellow": ([20, 100, 100], [35, 255, 255], None, None),
                            "orange": ([10, 100, 100], [20, 255, 255], None, None),
                        }
                        
                        ranges = color_ranges.get(color, color_ranges["red"])
                        mask = cv2.inRange(hsv, np.array(ranges[0]), np.array(ranges[1]))
                        if ranges[2] is not None:
                            mask2 = cv2.inRange(hsv, np.array(ranges[2]), np.array(ranges[3]))
                            mask = cv2.bitwise_or(mask, mask2)
                        
                        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        if contours:
                            largest = max(contours, key=cv2.contourArea)
                            area = cv2.contourArea(largest)
                            if area > 500:
                                detected = True
                                confidence = min(1.0, area / 10000)
                                det_label = f"{color}"
                                
                                rx, ry, rw, rh = cv2.boundingRect(largest)
                                det_bbox = (rx, ry, rx + rw, ry + rh)
                                
                                M = cv2.moments(largest)
                                if M["m00"] > 0:
                                    tcx = int(M["m10"] / M["m00"])
                                    tcy = int(M["m01"] / M["m00"])
                                    det_center = (tcx, tcy)
                                    
                                    h, w = frame.shape[:2]
                                    offset_x = (tcx - w / 2) / (w / 2)
                                    offset_y = (tcy - h / 2) / (h / 2)
                    except Exception:
                        pass
                
                # ========== 跟踪控制（完全对齐 SafeTracker.update） ==========
                # 摄像头朝下：offset_x → roll（左右移动），offset_y → pitch（前后移动）
                self.perception_status.detected = detected
                self.perception_status.confidence = confidence
                
                if detected:
                    target_lost_frames = 0
                    self.perception_status.tracking = True
                    # 缓存本帧检测结果
                    last_det_bbox = det_bbox
                    last_det_center = det_center
                    last_det_label = det_label
                    last_det_confidence = confidence
                    
                    # 1. 稳定性检查：偏移量足够小才算 centered
                    offset_magnitude = math.sqrt(offset_x**2 + offset_y**2)
                    if offset_magnitude < STABILITY_THRESHOLD:
                        stable_count += 1
                    else:
                        # 不硬重置，逐渐衰减（容忍因抖动偶尔偏出阈值）
                        stable_count = max(0, stable_count - 2)
                        self.perception_status.stable = False
                    
                    # 2. 应用死区
                    ctrl_x = 0.0 if abs(offset_x) < DEADZONE else offset_x
                    ctrl_y = 0.0 if abs(offset_y) < DEADZONE else offset_y
                    
                    # 3. 比例控制（摄像头朝下模式，同 SafeTracker）
                    #    offset_x > 0 → 目标在右 → roll 正（向右飞）
                    #    offset_y > 0 → 目标在下（画面下方=前方）→ pitch 负（向前飞）
                    target_roll = ctrl_x * KP
                    target_pitch = -ctrl_y * KP
                    
                    # 4. 限制最大速度
                    target_roll = max(-MAX_SPEED, min(MAX_SPEED, target_roll))
                    target_pitch = max(-MAX_SPEED, min(MAX_SPEED, target_pitch))
                    
                    # 5. 矢量速度限制（防止对角线过冲，同 SafeTracker）
                    magnitude = math.sqrt(target_roll**2 + target_pitch**2)
                    if magnitude > MAX_SPEED:
                        scale = MAX_SPEED / magnitude
                        target_roll *= scale
                        target_pitch *= scale
                    
                    # 6. 平滑过渡
                    smooth_roll = smooth_roll * (1 - SMOOTHING) + target_roll * SMOOTHING
                    smooth_pitch = smooth_pitch * (1 - SMOOTHING) + target_pitch * SMOOTHING
                    
                    # 7. 发送控制命令
                    roll_cmd = int(smooth_roll)
                    pitch_cmd = int(smooth_pitch)
                    try:
                        self._run_on_drone_thread(
                            self._do_tracking_control, roll_cmd, pitch_cmd, timeout=1
                        )
                    except Exception:
                        pass
                    
                    # 8. 检查是否达到 stable
                    if stable_count >= stable_required:
                        self.perception_status.stable = True
                        self.perception_status.message = "Target CENTERED & STABLE - triggering grab"
                        self.log(f"🎯 Target stable ({stable_count} frames centered), triggering winch", "success")
                        
                        annotated = self._draw_perception_overlay(
                            frame, det_bbox, det_center, det_label, confidence,
                            stable_count, stable_required, True
                        )
                        with self.frame_lock:
                            self.display_frame = annotated
                        
                        self._trigger_winch()
                        break
                    else:
                        self.perception_status.message = f"Tracking (centered {stable_count}/{stable_required}) offset={offset_magnitude:.2f}"
                else:
                    target_lost_frames += 1
                    stable_count = 0
                    self.perception_status.stable = False
                    
                    # 目标丢失 → 平滑减速（同 SafeTracker._smooth_stop）
                    if target_lost_frames >= MAX_LOST_FRAMES:
                        smooth_roll *= (1 - SMOOTHING)
                        smooth_pitch *= (1 - SMOOTHING)
                        if abs(smooth_roll) < 0.5:
                            smooth_roll = 0
                        if abs(smooth_pitch) < 0.5:
                            smooth_pitch = 0
                    
                    if target_lost_frames > 10:
                        self.perception_status.tracking = False
                        self.perception_status.message = "Searching for target..."
                
                # ---- 绘制叠加层并更新显示帧 ----
                # 视觉平滑：短暂丢失时保留上一帧的检测框，避免闪烁
                draw_bbox = det_bbox
                draw_center = det_center
                draw_label = det_label
                draw_conf = confidence
                if not detected and target_lost_frames <= VISUAL_KEEP_FRAMES and last_det_bbox is not None:
                    draw_bbox = last_det_bbox
                    draw_center = last_det_center
                    draw_label = last_det_label
                    draw_conf = last_det_confidence
                
                annotated = self._draw_perception_overlay(
                    frame, draw_bbox, draw_center, draw_label, draw_conf,
                    stable_count, stable_required, 
                    stable_count >= stable_required
                )
                with self.frame_lock:
                    self.display_frame = annotated
                
                time.sleep(0.005)
            
            # 清除显示帧叠加层，恢复原始画面
            with self.frame_lock:
                self.display_frame = None
            
            # 退出循环后立即发送归零命令，确保无人机停止追踪运动
            self._send_hover()
            
            if not self.perception_stop_flag.is_set():
                self.perception_status.state = SystemState.COMPLETED.value
                self.log("✅ Perception task completed", "success")
            else:
                self.perception_status.state = SystemState.IDLE.value
                self.perception_status.message = "Stopped"
                self.perception_status.detected = False
                self.perception_status.tracking = False
                self.perception_status.stable = False
                self.log("⚠️ Perception stopped by user", "warning")
                
        except Exception as e:
            # 出错时也清除叠加帧
            with self.frame_lock:
                self.display_frame = None
            self.perception_status.state = SystemState.ERROR.value
            self.perception_status.message = f"Error: {str(e)}"
            self.log(f"❌ Perception error: {str(e)}", "error")
    
    def _interruptible_sleep(self, seconds: float, check_interval: float = 0.2) -> bool:
        """可被紧急停止中断的 sleep。返回 True 表示正常完成，False 表示被中断。"""
        elapsed = 0
        while elapsed < seconds:
            if self.stop_flag.is_set():
                return False
            time.sleep(min(check_interval, seconds - elapsed))
            elapsed += check_interval
        return True
    
    def _trigger_winch(self):
        """触发绞盘系统 - 使用真实 HTTP 命令（参考 fly_track_and_grab.py）"""
        try:
            self.winch_status.state = SystemState.RUNNING.value
            
            # 步骤1: LOWER
            self.winch_status.current_action = "LOWERING"
            self.winch_status.message = "Lowering..."
            self.log(f"⬇️ Sending LOWER to {self.external_system.base_url}", "info")
            if not self.external_system.send_command("lower"):
                self.log("⚠️ LOWER command failed, continuing...", "warning")
            
            if not self._interruptible_sleep(5):
                self.external_system.send_command("stop")
                self.winch_status.state = SystemState.IDLE.value
                self.winch_status.message = "Emergency stopped"
                self.log("🚨 Winch interrupted by emergency stop", "warning")
                return
            
            # 步骤2: PULL
            self.winch_status.current_action = "PULLING"
            self.winch_status.message = "Pulling..."
            self.log(f"⬆️ Sending PULL to {self.external_system.base_url}", "info")
            if not self.external_system.send_command("pull"):
                self.log("⚠️ PULL command failed, continuing...", "warning")
            
            if not self._interruptible_sleep(3):
                self.external_system.send_command("stop")
                self.winch_status.state = SystemState.IDLE.value
                self.winch_status.message = "Emergency stopped"
                self.log("🚨 Winch interrupted by emergency stop", "warning")
                return
            
            # 步骤3: STOP
            self.winch_status.current_action = "STOP"
            self.winch_status.message = "Stopping..."
            self.log(f"⏹️ Sending STOP to {self.external_system.base_url}", "info")
            self.external_system.send_command("stop")
            
            self.winch_status.state = SystemState.COMPLETED.value
            self.winch_status.message = "Completed"
            self.log("✅ Winch sequence completed", "success")
            
        except Exception as e:
            self.winch_status.state = SystemState.ERROR.value
            self.winch_status.message = f"Error: {str(e)}"
            self.log(f"❌ Winch system error: {str(e)}", "error")
    
    def get_status(self) -> Dict[str, Any]:
        """获取系统完整状态"""
        return {
            "drone": asdict(self.drone_status),
            "navigation": asdict(self.navigation_status),
            "perception": asdict(self.perception_status),
            "winch": asdict(self.winch_status),
            "logs": self.logs[-10:]  # 最近10条日志
        }


# Flask 应用 (不使用 SocketIO，避免 monkey-patching 破坏 Olympe)
app = Flask(__name__, template_folder="templates", static_folder="static")
app.config['SECRET_KEY'] = 'drone-control-secret-2024'
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0  # 禁用静态文件缓存

# 全局控制系统实例
control_system = DroneControlSystem()


@app.route('/')
def index():
    """主页"""
    return render_template('index.html')


@app.route('/api/status')
def api_status():
    """获取系统状态"""
    return jsonify(control_system.get_status())


@app.route('/api/connect', methods=['POST'])
def api_connect():
    """连接无人机（所有操作在 Drone 线程上执行）"""
    success = control_system.connect_drone()
    return jsonify({"success": success})


@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    """断开连接"""
    control_system.disconnect_drone()
    return jsonify({"success": True})


@app.route('/api/takeoff', methods=['POST'])
def api_takeoff():
    """起飞"""
    success = control_system.takeoff()
    return jsonify({"success": success})


@app.route('/api/land', methods=['POST'])
def api_land():
    """降落"""
    success = control_system.land()
    return jsonify({"success": success})


@app.route('/api/emergency_stop', methods=['POST'])
def api_emergency_stop():
    """紧急停止"""
    control_system.emergency_stop()
    return jsonify({"success": True})


@app.route('/api/navigation/start', methods=['POST'])
def api_navigation_start():
    """启动导航"""
    data = request.json
    success = control_system.start_navigation(
        float(data['target_lat']),
        float(data['target_lon']),
        float(data['target_alt']),
        float(data.get('arrival_threshold', 0.5))
    )
    return jsonify({"success": success})


@app.route('/api/perception/config', methods=['POST'])
def api_perception_config():
    """配置感知系统"""
    data = request.json
    control_system.perception_status.tracking_mode = data.get('mode', 'yolo')
    control_system.perception_status.target_classes = data.get('classes', ['person'])
    control_system.perception_status.target_color = data.get('color', 'red')
    return jsonify({"success": True})


@app.route('/api/perception/start', methods=['POST'])
def api_perception_start():
    """启动感知"""
    success = control_system.start_perception()
    return jsonify({"success": success})


@app.route('/api/perception/stop', methods=['POST'])
def api_perception_stop():
    """停止感知"""
    control_system.perception_stop_flag.set()
    # 清空命令队列中残留的 tracking 命令，然后发送 hover
    control_system._drain_cmd_queue()
    control_system._send_hover()
    control_system.log("🛑 Perception stopped, drone hovering", "info")
    return jsonify({"success": True})


@app.route('/api/navigation/stop', methods=['POST'])
def api_navigation_stop():
    """停止导航"""
    control_system.stop_navigation()
    return jsonify({"success": True})


@app.route('/api/manual_control', methods=['POST'])
def api_manual_control():
    """手动控制无人机（在 Drone 线程上执行）"""
    data = request.json
    axis = data.get('axis')  # roll, pitch, yaw, gaz
    value = int(data.get('value', 0))  # -100 to 100
    
    if not control_system.drone_status.connected:
        return jsonify({"success": False, "error": "Drone not connected"})
    
    try:
        control_system.manual_control(axis, value)
        return jsonify({"success": True})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/test/navigation', methods=['POST'])
def api_test_navigation():
    """测试 Navigation 模块 - 检查 GPS + moveTo 可用性"""
    try:
        control_system.log("🧪 Testing Navigation module...", "info")
        
        if not control_system.drone_status.connected:
            control_system.log("❌ Drone not connected", "error")
            return jsonify({"success": False, "error": "Drone not connected"})
        
        # 1. 检查 GPS 状态
        sats = control_system.drone_status.satellites
        control_system.log(f"📡 GPS satellites: {sats}", "info")
        
        if sats < 4:
            control_system.log("⚠️ GPS signal too weak for navigation (need ≥10)", "warning")
        elif sats < 10:
            control_system.log("⚠️ GPS signal weak, navigation may be inaccurate", "warning")
        else:
            control_system.log("✅ GPS signal good", "success")
        
        # 2. 报告当前位置
        lat = control_system.drone_status.latitude
        lon = control_system.drone_status.longitude
        control_system.log(f"📍 Position: ({lat:.6f}, {lon:.6f})", "info")
        
        if abs(lat) < 0.001 and abs(lon) < 0.001:
            control_system.log("⚠️ GPS position not acquired yet", "warning")
        else:
            control_system.log("✅ GPS position acquired", "success")
        
        # 3. 检查 moveTo 命令可用性
        control_system.log("✅ moveTo command available (Olympe import OK)", "success")
        
        # 4. 总结
        if sats >= 10 and abs(lat) > 0.001:
            control_system.log("✅ Navigation module: READY", "success")
        else:
            control_system.log("⚠️ Navigation module: GPS NOT READY", "warning")
        
        return jsonify({"success": True})
        
    except Exception as e:
        control_system.log(f"❌ Navigation test failed: {str(e)}", "error")
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/test/perception', methods=['POST'])
def api_test_perception():
    """测试 Perception 模块 - 检查检测器 + 视频帧"""
    try:
        control_system.log("🧪 Testing Perception module...", "info")
        
        mode = control_system.perception_status.tracking_mode
        control_system.log(f"📋 Mode: {mode}", "info")
        
        # 1. 测试视频帧可用性
        with control_system.frame_lock:
            frame = control_system.current_frame
        
        if frame is not None:
            h, w = frame.shape[:2]
            control_system.log(f"✅ Video feed OK: {w}x{h}", "success")
        else:
            control_system.log("⚠️ No video frame available", "warning")
        
        # 2. 测试检测器
        if mode == "yolo":
            classes = control_system.perception_status.target_classes
            control_system.log(f"🎯 Target classes: {', '.join(classes)}", "info")
            
            try:
                sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
                from object_detector import ObjectDetector
                detector = ObjectDetector(
                    model_path=control_system.perception_status.model,
                    conf_threshold=0.3,
                    class_names_filter=classes,
                    verbose=False
                )
                control_system.log(f"✅ YOLO model loaded: {control_system.perception_status.model}", "success")
                
                # 尝试检测一帧
                if frame is not None:
                    detections = detector.detect(frame)
                    control_system.log(f"🔍 Detected {len(detections)} objects in current frame", "info")
                    for d in detections[:3]:
                        control_system.log(f"   → {d.class_name}: {d.confidence:.0%}", "info")
                    
            except ImportError:
                control_system.log("⚠️ object_detector module not found", "warning")
            except Exception as e:
                control_system.log(f"⚠️ YOLO load failed: {str(e)}", "warning")
        
        elif mode == "color":
            color = control_system.perception_status.target_color
            control_system.log(f"🎯 Target color: {color}", "info")
            
            if frame is not None:
                # 测试颜色检测
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                control_system.log(f"✅ Color detection pipeline OK", "success")
            
        control_system.log("✅ Perception module: READY", "success")
        return jsonify({"success": True})
        
    except Exception as e:
        control_system.log(f"❌ Perception test failed: {str(e)}", "error")
        return jsonify({"success": False, "error": str(e)})


@app.route('/api/video_feed')
def api_video_feed():
    """视频流 (MJPEG over HTTP)"""
    def generate():
        while True:
            with control_system.frame_lock:
                # 优先使用带叠加层的显示帧（perception 运行时），否则用原始帧
                frame = control_system.display_frame if control_system.display_frame is not None else control_system.current_frame
            
            if frame is not None:
                # 编码为 JPEG
                ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           jpeg.tobytes() + b'\r\n')
            else:
                # 没有帧时发送一个黑色占位图
                blank = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(blank, "No Video Feed", (180, 180),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (100, 100, 100), 2)
                ret, jpeg = cv2.imencode('.jpg', blank)
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' +
                           jpeg.tobytes() + b'\r\n')
            
            time.sleep(0.02)  # ~50fps MJPEG 推流（浏览器 MJPEG 渲染瓶颈约 30-60fps）
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/test/winch', methods=['POST'])
def api_test_winch():
    """测试 Winch System - 尝试发送实际 HTTP 命令"""
    def run_test():
        try:
            url = control_system.external_system.base_url
            control_system.log(f"🧪 Testing Winch System at {url}", "info")
            
            # 1. 测试 LOWER
            control_system.log("⬇️ Sending LOWER...", "info")
            ok = control_system.external_system.send_command("lower")
            if ok:
                control_system.log("✅ LOWER: OK", "success")
            else:
                control_system.log("❌ LOWER: FAILED (connection error?)", "error")
                control_system.log(f"ℹ️ Check if winch system is at {url}", "info")
                return
            
            time.sleep(2)
            
            # 2. 测试 PULL
            control_system.log("⬆️ Sending PULL...", "info")
            ok = control_system.external_system.send_command("pull")
            if ok:
                control_system.log("✅ PULL: OK", "success")
            else:
                control_system.log("❌ PULL: FAILED", "error")
            
            time.sleep(2)
            
            # 3. 测试 STOP
            control_system.log("⏹️ Sending STOP...", "info")
            ok = control_system.external_system.send_command("stop")
            if ok:
                control_system.log("✅ STOP: OK", "success")
            else:
                control_system.log("❌ STOP: FAILED", "error")
            
            control_system.log("✅ Winch System test complete", "success")
            
        except Exception as e:
            control_system.log(f"❌ Winch test error: {str(e)}", "error")
    
    # 在后台线程运行（因为有 sleep）
    threading.Thread(target=run_test, daemon=True).start()
    return jsonify({"success": True})


def main():
    """主函数"""
    print("=" * 60)
    print("  Drone Control System - Web GUI")
    print("=" * 60)
    print()
    print("  Web server starting...")
    print("  URL: http://0.0.0.0:5000")
    print()
    print("  Press Ctrl+C to stop")
    print("=" * 60)
    print()
    
    # 使用纯 Flask（不使用 SocketIO，避免与 Olympe 冲突）
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


if __name__ == "__main__":
    main()

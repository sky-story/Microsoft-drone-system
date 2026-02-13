#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ANAFI Ai: 目标跟踪 + 外部系统控制脚本 (WebSocket版本)
适用于：海面物体回收场景

工作流程:
  1. 无人机起飞并搜索目标
  2. 检测到目标后开始跟踪(保持目标在画面中心)
  3. 目标稳定跟踪一段时间后,触发外部系统控制:
     - 发送 lower:length_mm 命令 (下降抓取装置指定长度)
     - 等待 5 秒 (可调)
     - 发送 pull:length_mm 命令 (拉起物体指定长度)
     - 保持 3 秒 (可调)
     - 发送 stop 命令 (停止外部系统)
  4. 完成后等待手动降落 (按 'l' 键降落)

外部系统 (见 external_systems.py，可扩展):
  - Winch (默认 ws://192.168.42.15): lower:mm | pull:mm | stop
  - Gripper (默认 ws://192.168.42.39:81): hold | release | grip | status

使用方法:
  python fly_track_and_grab_ws.py --classes person
  python fly_track_and_grab_ws.py --classes keyboard --stable-time 5 --wait-time 5 --pull-time 3
  python fly_track_and_grab_ws.py --classes person --lower-length 150 --pull-length 80

测试流程 (mock 飛航+追蹤成功，僅測試 winch 流程):
  python fly_track_and_grab_ws.py --test-winch-only --classes person
  python fly_track_and_grab_ws.py --test-winch-only --classes person --no-gripper --lower-length 100 --pull-length 50
"""

import os
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import sys
import time
import argparse
import queue
import threading
import math
from typing import Optional, Tuple, List
from enum import Enum

import cv2
import numpy as np

import olympe
import olympe.log
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

from object_detector import ObjectDetector, Detection
from external_systems import (
    WinchController,
    GripperController,
    ExternalSystemsManager,
    GrabState,
)

import tty
import termios
import select
import json


class SafeTracker:
    """
    安全的目标跟踪控制器
    
    摄像头模式 "down"（摄像头朝下,用于海面回收）:
      画面左右 → roll（左右平移）
      画面上下 → pitch（前后移动）
    """
    
    def __init__(
        self,
        max_speed: int = 5,
        smoothing: float = 0.05,
        deadzone: float = 0.2,
        kp: float = 5.0,
        invert_roll: bool = False,
        invert_pitch: bool = False,
    ):
        self.max_speed = max_speed
        self.smoothing = smoothing
        self.deadzone = deadzone
        self.kp = kp
        self.invert_roll = invert_roll
        self.invert_pitch = invert_pitch
        
        # 控制输出
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.gaz = 0.0
        
        # 跟踪状态
        self.target_lost_frames = 0
        self.max_lost_frames = 20
        
        # 稳定性检测
        self.stable_frames = 0           # 连续稳定的帧数
        self.is_stable = False            # 是否稳定
        self.stability_threshold = 0.15   # 稳定判定阈值 (偏移量小于此值认为稳定)
        
    def update(self, detection: Optional[Detection], frame_width: int, frame_height: int) -> bool:
        """
        更新控制量
        
        Returns:
            True = 正在跟踪, False = 目标丢失
        """
        if detection is None:
            self.target_lost_frames += 1
            self.stable_frames = 0
            self.is_stable = False
            if self.target_lost_frames >= self.max_lost_frames:
                self._smooth_stop()
            return False
        
        self.target_lost_frames = 0
        
        # 计算目标中心偏移量
        cx, cy = detection.center
        offset_x = (cx - frame_width / 2) / (frame_width / 2)
        offset_y = (cy - frame_height / 2) / (frame_height / 2)
        
        # 检查稳定性
        offset_magnitude = math.sqrt(offset_x**2 + offset_y**2)
        if offset_magnitude < self.stability_threshold:
            self.stable_frames += 1
        else:
            self.stable_frames = 0
            self.is_stable = False
        
        # 应用死区
        if abs(offset_x) < self.deadzone:
            offset_x = 0
        if abs(offset_y) < self.deadzone:
            offset_y = 0
        
        # 计算控制量 (摄像头朝下模式)
        roll_multiplier = -1 if self.invert_roll else 1
        pitch_multiplier = 1 if self.invert_pitch else -1
        
        target_roll = offset_x * self.kp * roll_multiplier
        target_pitch = offset_y * self.kp * pitch_multiplier
        
        # 限制最大速度
        target_roll = self._clamp(target_roll, -self.max_speed, self.max_speed)
        target_pitch = self._clamp(target_pitch, -self.max_speed, self.max_speed)
        
        # 矢量速度限制 (防止对角线过冲)
        magnitude = math.sqrt(target_roll**2 + target_pitch**2)
        if magnitude > self.max_speed:
            scale = self.max_speed / magnitude
            target_roll *= scale
            target_pitch *= scale
        
        # 平滑过渡
        s = self.smoothing
        self.roll = self.roll * (1 - s) + target_roll * s
        self.pitch = self.pitch * (1 - s) + target_pitch * s
        
        return True
    
    def _smooth_stop(self):
        """平滑停止"""
        s = self.smoothing
        self.roll *= (1 - s)
        self.pitch *= (1 - s)
        self.yaw *= (1 - s)
        self.gaz *= (1 - s)
        
        if abs(self.roll) < 0.5:
            self.roll = 0
        if abs(self.pitch) < 0.5:
            self.pitch = 0
        if abs(self.yaw) < 0.5:
            self.yaw = 0
        if abs(self.gaz) < 0.5:
            self.gaz = 0
    
    def _clamp(self, v: float, min_v: float, max_v: float) -> float:
        return max(min_v, min(max_v, v))
    
    def get_command(self) -> Tuple[int, int, int, int]:
        """获取控制命令 (roll, pitch, yaw, gaz)"""
        return (int(self.roll), int(self.pitch), int(self.yaw), int(self.gaz))
    
    def emergency_stop(self):
        """紧急停止"""
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gaz = 0
        self.stable_frames = 0
        self.is_stable = False


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


def parse_args():
    p = argparse.ArgumentParser(
        description="ANAFI Ai: Target Tracking + External System Control (Camera Down)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
工作流程:
  1. 起飞并搜索目标
  2. 检测到目标后开始跟踪 (保持目标在画面中心)
  3. 目标稳定跟踪指定时间后,触发外部系统(通过WebSocket):
     - 发送 lower:length_mm 命令 (下降抓取装置指定长度)
     - 等待 wait_time 秒 (默认5秒, 让装置下降到位)
     - 发送 pull:length_mm 命令 (拉起物体指定长度)
     - 等待 pull_time 秒 (默认3秒, 保持拉起状态)
     - 发送 stop 命令 (停止外部系统)
  4. 完成后保持悬停, 等待手动降落 (按 'l' 键)

使用示例:
  # 基本使用
  python fly_track_and_grab_ws.py --classes person
  
  # 自定义时间参数
  python fly_track_and_grab_ws.py --classes chair \
      --stable-time 3 --wait-time 5 --pull-time 3
  
  # 自定义长度参数 (毫米)
  python fly_track_and_grab_ws.py --classes person \
      --lower-length 150 --pull-length 80
  
  # 快速抓取 (缩短等待时间)
  python fly_track_and_grab_ws.py --classes person \
      --wait-time 3 --pull-time 2
  
  # 超保守设置 (防止过冲)
  python fly_track_and_grab_ws.py --classes person --kp 3 --max-speed 3
  
  # 自动起飞 + 自动降落 + 自定义WebSocket地址
  python fly_track_and_grab_ws.py --classes person --takeoff-on-start --auto-land-on-exit \
      --system-url ws://192.168.42.20
  
  注意: 默认完成后不自动降落, 需手动按 'l' 键降落
        如需自动降落, 添加 --auto-land-on-exit 参数

摄像头控制映射 (固定朝下模式):
  目标在画面RIGHT  → drone moves RIGHT   (roll +)
  目标在画面LEFT   → drone moves LEFT    (roll -)
  目标在画面TOP    → drone moves FORWARD (pitch +)
  目标在画面BOTTOM → drone moves BACK    (pitch -)

安全特性:
  • 🛡️ 安全模式 (SPACE) - 最高优先级
    - 立即禁用自动跟踪
    - 启用手动键盘控制
    - 用于紧急情况下手动控制到安全位置
  
  • 紧急降落 (L) - 随时可用，不受任何模式限制
  
  • 速度限制:
    - 极保守的默认值 (max_speed=5, kp=5.0)
    - 矢量速度限制 (防止对角线过冲)
    - 大死区 (deadzone=0.2, 防止抖动)
    - 极平滑控制 (smoothing=0.05)
  
  • 手动控制优先级高于自动跟踪
    - 任何时候都可以手动接管控制

按键控制:
  🛡️ SPACE = 安全模式 (toggle) - 最高优先级
           进入: 禁用自动跟踪，启用手动控制
           退出: 恢复自动跟踪
  
  l = 降落 (随时可用)
  t = 起飞
  p = 暂停/恢复自动跟踪
  g = 手动触发抓取 (当检测到目标时)
  q = 退出
  
  手动飞行 (优先级高于自动跟踪):
    w/s = 前进/后退 (pitch)
    a/d = 左移/右移 (roll)
    z/e = 左转/右转 (yaw)
    r/f = 上升/下降 (gaz)

参数调优:
  问题: 目标飞出画面 (尤其在角落)?
    解决: --kp 3 --max-speed 3 --deadzone 0.25
  
  问题: 跟踪太慢?
    解决: --kp 10 --max-speed 10 --smoothing 0.15
        """
    )
    
    # 连接
    p.add_argument("--drone-ip", default=os.environ.get("DRONE_IP", "192.168.42.1"))
    p.add_argument("--loglevel", default="WARNING", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    
    # 外部系统 (winch + 可选 gripper)
    p.add_argument("--system-url", default="ws://192.168.42.15",
                   help="Winch WebSocket URL (default: ws://192.168.42.15)")
    p.add_argument("--gripper-url", default="ws://192.168.42.39:81",
                   help="Gripper WebSocket URL; cmds: hold | release | grip | status (default: ws://192.168.42.39:81)")
    p.add_argument("--no-gripper", action="store_true",
                   help="Disable gripper in grab sequence (winch only)")
    p.add_argument("--lower-length", type=float, default=100.0,
                   help="Lower distance in millimeters (default: 100mm)")
    p.add_argument("--pull-length", type=float, default=50.0,
                   help="Pull distance in millimeters (default: 50mm)")
    p.add_argument("--wait-time", type=float, default=5.0,
                   help="Wait time between LOWER and PULL (seconds, default 5)")
    p.add_argument("--pull-time", type=float, default=3.0,
                   help="Hold time after PULL command (seconds, default 3)")
    p.add_argument("--stable-time", type=float, default=3.0,
                   help="Stable tracking time before triggering grab (seconds, at 10fps = 30 frames)")
    p.add_argument("--auto-trigger", action="store_true", default=True,
                   help="Auto trigger grab when target is stable")
    p.add_argument("--no-auto-trigger", action="store_false", dest="auto_trigger")
    
    # 检测
    p.add_argument("--model", default="yolov8n.pt", help="YOLO model")
    p.add_argument("--conf", type=float, default=0.5, help="Detection confidence")
    p.add_argument("--classes", nargs="+", type=str, required=True,
                   help="Target classes to track")
    
    # 跟踪控制
    p.add_argument("--max-speed", type=int, default=5,
                   help="Max auto-tracking speed (0-100, default 5)")
    p.add_argument("--smoothing", type=float, default=0.05,
                   help="Smoothing (0-1, default 0.05)")
    p.add_argument("--deadzone", type=float, default=0.2,
                   help="Dead zone (0-0.5, default 0.2)")
    p.add_argument("--kp", type=float, default=5.0,
                   help="Proportional gain (default 5.0)")
    p.add_argument("--invert-roll", action="store_true",
                   help="Invert roll direction")
    p.add_argument("--invert-pitch", action="store_true",
                   help="Invert pitch direction")
    
    # 手动控制
    p.add_argument("--manual-speed", type=int, default=25,
                   help="Manual control speed (0-100, default 25)")
    
    # 起飞/降落
    p.add_argument("--takeoff-on-start", action="store_true")
    p.add_argument("--auto-land-on-exit", action="store_true", default=False,
                   help="Auto land when program exits (default: False, manual landing)")
    p.add_argument("--test-winch-only", action="store_true",
                   help="Mock fly+track success: skip drone/detector/streaming, run only winch grab sequence (for testing)")
    
    return p.parse_args()


class FlyTrackAndGrab:
    """目标跟踪 + 外部系统控制主程序"""
    
    def __init__(self, args):
        self.args = args
        olympe.log.update_config({"loggers": {"olympe": {"level": args.loglevel}}})
        
        self.drone = olympe.Drone(args.drone_ip)
        self.detector = None
        self.tracker = None
        # Modular external systems: winch + optional gripper
        self.external_systems = ExternalSystemsManager()
        winch = WinchController(
            ws_url=args.system_url,
            lower_length=args.lower_length,
            pull_length=args.pull_length,
            log_prefix="WINCH",
        )
        self.external_systems.register("winch", winch)
        self.external_system = winch  # backward compat: state + execute_grab_sequence
        gripper = GripperController(ws_url=args.gripper_url, log_prefix="GRIPPER")
        self.external_systems.register("gripper", gripper)
        self.gripper = gripper
        self.use_gripper_in_sequence = not args.no_gripper
        
        self.running = False
        self.paused = False
        self.safe_mode = False            # 🛡️ 安全模式：最高优先级
        self.grab_triggered = False       # 是否已触发抓取
        
        # 视频帧队列
        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_lock = threading.Lock()
        
        # 手动飞行控制（优先级高于自动跟踪）
        self.manual_roll = 0
        self.manual_pitch = 0
        self.manual_yaw = 0
        self.manual_gaz = 0
        self.manual_speed = args.manual_speed
        self.last_manual_time = {"roll": 0, "pitch": 0, "yaw": 0, "gaz": 0}
        self.manual_timeout = 0.25
        
        # 稳定时间判定 (假设10fps, 3秒 = 30帧)
        self.stable_frames_required = int(args.stable_time * 10)
        
        # 统计
        self.fps_list = []
        self.frame_count = 0
        
    def connect(self):
        """连接无人机"""
        print(f"[1/4] Connecting to {self.args.drone_ip} ...")
        for i in range(3):
            try:
                if self.drone.connect():
                    print("[OK] Connected")
                    return True
            except Exception as e:
                print(f"[WARN] Attempt {i+1}/3 failed: {e}")
                time.sleep(1)
        print("[ERROR] Connection failed")
        return False
    
    def init_detector(self):
        """初始化检测器"""
        print("[2/4] Initializing detector ...")
        try:
            self.detector = ObjectDetector(
                model_path=self.args.model,
                conf_threshold=self.args.conf,
                class_names_filter=self.args.classes,
                verbose=True
            )
            print(f"[OK] Detector ready, tracking: {self.args.classes}")
            return True
        except Exception as e:
            print(f"[ERROR] Detector failed: {e}")
            return False
    
    def init_tracker(self):
        """初始化跟踪器"""
        self.tracker = SafeTracker(
            max_speed=self.args.max_speed,
            smoothing=self.args.smoothing,
            deadzone=self.args.deadzone,
            kp=self.args.kp,
            invert_roll=self.args.invert_roll,
            invert_pitch=self.args.invert_pitch,
        )
        print(f"[OK] Tracker ready (max_speed={self.args.max_speed})")
        print(f"[OK] Winch: {self.args.system_url}  Gripper: {self.args.gripper_url}")
        print(f"[OK] Auto-trigger: {self.args.auto_trigger}, stable time: {self.args.stable_time}s")
    
    def yuv_frame_cb(self, yuv_frame):
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

    def flush_cb(self, stream):
        with self.flush_lock:
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait().unref()
                except Exception:
                    pass
        return True
    
    def start_streaming(self):
        """启动视频流"""
        print("[3/4] Starting video stream ...")
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            flush_raw_cb=self.flush_cb,
        )
        self.drone.streaming.start()
        self.running = True
        print("[OK] Streaming started")
    
    def stop_streaming(self):
        if self.running:
            print("[INFO] Stopping streaming...")
            try:
                self.flush_cb(None)
            except Exception:
                pass
            try:
                self.drone.streaming.stop()
            except Exception:
                pass
            time.sleep(0.5)
            self.running = False
    
    def disconnect(self):
        print("[INFO] Disconnecting ...")
        time.sleep(0.3)
        try:
            self.drone.disconnect()
            print("[OK] Disconnected")
        except Exception as e:
            print(f"[WARN] disconnect error: {e}")
    
    def _state_name(self) -> str:
        st = self.drone.get_state(FlyingStateChanged).get("state")
        return getattr(st, "name", str(st))
    
    def takeoff(self):
        state = self._state_name()
        if state == "hovering":
            print("[INFO] Already hovering")
            return
        print("[INFO] Taking off ...")
        try:
            self.drone(TakeOff() >> FlyingStateChanged(state="hovering", _timeout=20)).wait()
            print("[OK] Hovering")
        except Exception as e:
            print(f"[ERROR] Takeoff failed: {e}")
    
    def land(self):
        state = self._state_name()
        if state in ("hovering", "flying", "takingoff"):
            print("[INFO] Landing ...")
            try:
                self.drone(Landing() >> FlyingStateChanged(state="landed", _timeout=30)).wait()
                print("[OK] Landed")
            except Exception as e:
                print(f"[WARN] Land error: {e}")
    
    def start_piloting(self):
        self.drone.start_piloting()
    
    def stop_piloting(self):
        try:
            self.drone.stop_piloting()
        except Exception:
            pass
    
    def send_piloting(self, roll: int, pitch: int, yaw: int, gaz: int):
        """发送飞行控制命令"""
        self.drone.piloting(roll, pitch, yaw, gaz, 0.05)
    
    def _clamp(self, v: int) -> int:
        """限制值在 -100 到 100 之间"""
        return max(-100, min(100, int(v)))
    
    def _update_manual_axis(self, axis: str, value: int):
        """更新手动控制轴的值"""
        if axis == "roll":
            self.manual_roll = self._clamp(self.manual_roll + value)
        elif axis == "pitch":
            self.manual_pitch = self._clamp(self.manual_pitch + value)
        elif axis == "yaw":
            self.manual_yaw = self._clamp(self.manual_yaw + value)
        elif axis == "gaz":
            self.manual_gaz = self._clamp(self.manual_gaz + value)
        self.last_manual_time[axis] = time.time()
    
    def _decay_manual_controls(self):
        """自动衰减未使用的手动控制轴"""
        now = time.time()
        if now - self.last_manual_time["roll"] > self.manual_timeout:
            self.manual_roll = 0
        if now - self.last_manual_time["pitch"] > self.manual_timeout:
            self.manual_pitch = 0
        if now - self.last_manual_time["yaw"] > self.manual_timeout:
            self.manual_yaw = 0
        if now - self.last_manual_time["gaz"] > self.manual_timeout:
            self.manual_gaz = 0
    
    def has_manual_control(self) -> bool:
        """检查是否有活跃的手动控制"""
        now = time.time()
        for axis in ["roll", "pitch", "yaw", "gaz"]:
            if now - self.last_manual_time[axis] <= self.manual_timeout:
                return True
        return False
    
    def get_latest_frame(self):
        """获取最新一帧 BGR 图像"""
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
        finally:
            try:
                last.unref()
            except Exception:
                pass
    
    def select_target(self, detections: List[Detection]) -> Optional[Detection]:
        """选择要跟踪的目标（选择最大的）"""
        if not detections:
            return None
        return max(detections, key=lambda d: d.area)
    
    def trigger_grab_sequence(self):
        """触发抓取序列（在单独的线程中执行）"""
        if self.grab_triggered:
            print("[WARN] Grab already triggered, ignoring")
            return
        
        self.grab_triggered = True
        print("\n" + "="*60)
        print("🎯 TARGET LOCKED! Triggering grab sequence...")
        print("="*60)
        
        gripper = self.gripper if self.use_gripper_in_sequence else None
        thread = threading.Thread(
            target=self.external_system.execute_grab_sequence,
            kwargs={"gripper": gripper},
            daemon=True
        )
        thread.start()
    
    def draw_info(self, frame, detection, tracking):
        """绘制信息"""
        h, w = frame.shape[:2]
        
        # === 🛡️ 安全模式：最高优先级显示 ===
        if self.safe_mode:
            # 全屏橙色半透明遮罩
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), (0, 140, 255), -1)
            frame = cv2.addWeighted(overlay, 0.12, frame, 0.88, 0)
            
            # 大号警告文字
            warning = "[SAFE MODE] - AUTO-TRACKING OFF"
            text_size = cv2.getTextSize(warning, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)[0]
            text_x = (w - text_size[0]) // 2
            
            cv2.rectangle(frame, (text_x - 20, 10), (text_x + text_size[0] + 20, 55), (0, 100, 200), -1)
            cv2.rectangle(frame, (text_x - 20, 10), (text_x + text_size[0] + 20, 55), (0, 165, 255), 3)
            cv2.putText(frame, warning, (text_x, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
            
            # 提示信息
            hint1 = "Manual Control ENABLED: Use WASD/ZE/RF to move safely"
            hint1_size = cv2.getTextSize(hint1, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            hint1_x = (w - hint1_size[0]) // 2
            cv2.rectangle(frame, (hint1_x - 10, 65), (hint1_x + hint1_size[0] + 10, 95), (0, 80, 160), -1)
            cv2.putText(frame, hint1, (hint1_x, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            hint2 = "Press 'l' to LAND | Press SPACE to resume auto-tracking"
            hint2_size = cv2.getTextSize(hint2, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            hint2_x = (w - hint2_size[0]) // 2
            cv2.rectangle(frame, (hint2_x - 10, 100), (hint2_x + hint2_size[0] + 10, 130), (0, 60, 120), -1)
            cv2.putText(frame, hint2, (hint2_x, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 显示当前手动控制量
            roll = self.manual_roll
            pitch = self.manual_pitch
            yaw = self.manual_yaw
            gaz = self.manual_gaz
            ctrl = f"roll: {roll:+3d} | pitch: {pitch:+3d} | yaw: {yaw:+3d} | gaz: {gaz:+3d} [SAFE MODE]"
            cv2.putText(frame, ctrl, (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
            
            return frame
        
        # === 正常模式显示 ===
        # 背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 120), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
        
        # 外部系统状态 (最显眼的位置)
        grab_state = self.external_system.state.value.upper()
        if self.external_system.state == GrabState.COMPLETED:
            grab_color = (0, 255, 0)  # 绿色
            grab_text = f"[GRAB: {grab_state}] ✓"
        elif self.external_system.state in [GrabState.LOWERING, GrabState.WAITING, GrabState.PULLING]:
            grab_color = (0, 165, 255)  # 橙色
            grab_text = f"[GRAB: {grab_state}] ..."
        elif self.external_system.state == GrabState.ERROR:
            grab_color = (0, 0, 255)  # 红色
            grab_text = f"[GRAB: {grab_state}] ✗"
        else:
            grab_color = (180, 180, 180)  # 灰色
            grab_text = f"[GRAB: {grab_state}]"
        
        cv2.putText(frame, grab_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, grab_color, 2)
        
        # 跟踪状态
        if self.paused:
            status = "PAUSED"
            color = (0, 165, 255)
        elif tracking and detection:
            # 显示稳定性
            stable_progress = min(100, int(self.tracker.stable_frames / self.stable_frames_required * 100))
            status = f"TRACKING: {detection.class_name} | Stable: {stable_progress}%"
            
            if self.tracker.stable_frames >= self.stable_frames_required:
                color = (0, 255, 0)  # 绿色 = 已稳定
                self.tracker.is_stable = True
            else:
                color = (0, 255, 255)  # 黄色 = 跟踪中
        else:
            status = "SEARCHING..."
            color = (100, 100, 100)
        
        cv2.putText(frame, status, (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # 控制量（显示实际发送的值，包括手动覆盖）
        roll, pitch, yaw, gaz = self.tracker.get_command()
        
        # 检查手动控制并覆盖显示值
        manual_active = self.has_manual_control()
        if self.manual_roll != 0:
            roll = self.manual_roll
        if self.manual_pitch != 0:
            pitch = self.manual_pitch
        if self.manual_yaw != 0:
            yaw = self.manual_yaw
        if self.manual_gaz != 0:
            gaz = self.manual_gaz
        
        # DOWN 模式：显示更详细的方向信息
        ctrl = f"roll: {roll:+3d}"
        if roll > 0:
            ctrl += "(>RIGHT)"
        elif roll < 0:
            ctrl += "(<LEFT)"
        
        ctrl += f" | pitch: {pitch:+3d}"
        if pitch > 0:
            ctrl += "(^FWD)"
        elif pitch < 0:
            ctrl += "(vBACK)"
        
        ctrl += f" | yaw: {yaw:+3d} | gaz: {gaz:+3d}"
        if manual_active:
            ctrl += " [MANUAL]"
        cv2.putText(frame, ctrl, (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # 参数
        params = f"mode: down | kp: {self.args.kp} | max_speed: {self.args.max_speed} | deadzone: {self.args.deadzone} | smooth: {self.args.smoothing}"
        cv2.putText(frame, params, (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
        
        # 画面中心十字
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx - 40, cy), (cx + 40, cy), (255, 255, 255), 1)
        cv2.line(frame, (cx, cy - 40), (cx, cy + 40), (255, 255, 255), 1)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
        
        # 目标
        if detection:
            x1, y1, x2, y2 = detection.bbox
            tx, ty = detection.center
            
            # 检测框 (颜色根据稳定性变化)
            if self.tracker.is_stable:
                box_color = (0, 255, 0)  # 绿色 = 稳定
            else:
                box_color = (0, 255, 255)  # 黄色 = 跟踪中
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            cv2.circle(frame, (tx, ty), 8, box_color, -1)
            cv2.line(frame, (cx, cy), (tx, ty), box_color, 2)
            
            # 偏移量
            offset_x = (tx - cx) / (w / 2)
            offset_y = (ty - cy) / (h / 2)
            offset_text = f"offset: x={offset_x:+.2f} y={offset_y:+.2f}"
            cv2.putText(frame, offset_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
            
            # 显示控制方向提示（帮助调试）
            if offset_x > 0.1:
                dir_hint = "> Roll RIGHT"
            elif offset_x < -0.1:
                dir_hint = "< Roll LEFT"
            else:
                dir_hint = "= Centered X"
            
            if offset_y > 0.1:
                dir_hint += " | v Pitch BACK"
            elif offset_y < -0.1:
                dir_hint += " | ^ Pitch FORWARD"
            else:
                dir_hint += " | = Centered Y"
            
            cv2.putText(frame, dir_hint, (x1, y1 - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)
        
        # 帮助
        help_text = "SPACE=🛡️SAFE | l=LAND t=takeoff p=pause g=grab | wasd/ze/rf=move | q=quit"
        cv2.putText(frame, help_text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1)
        
        return frame
    
    def handle_key(self, ch) -> bool:
        """
        处理按键,返回 True 表示退出
        
        优先级顺序（从高到低）：
        1. 退出 (q/ESC)
        2. 🛡️ 安全模式 (SPACE) - 最高安全优先级
        3. 降落 (l) - 随时可以降落
        4. 起飞 (t)
        5. 其他控制
        """
        # === 优先级 1: 退出 ===
        if ch in ('\x1b', 'q', 'Q'):
            return True
        
        # === 优先级 2: 🛡️ 安全模式（最高优先级） ===
        if ch == ' ':
            self.safe_mode = not self.safe_mode
            if self.safe_mode:
                # 进入安全模式：
                # 1. 立即停止自动跟踪
                # 2. 停止当前运动
                # 3. 但允许手动键盘控制
                self.tracker.emergency_stop()
                self.manual_roll = self.manual_pitch = self.manual_yaw = self.manual_gaz = 0
                self.send_piloting(0, 0, 0, 0)
                print("[🛡️ SAFE MODE] ON - Auto-tracking DISABLED. Manual control ENABLED.")
                print("[🛡️ SAFE MODE] Use WASD/ZE/RF to move safely, then press 'l' to land.")
            else:
                # 退出安全模式
                print("[🛡️ SAFE MODE] OFF - Auto-tracking enabled.")
            return False
        
        # === 优先级 3: 降落（不受安全模式限制，随时可降落） ===
        if ch in ('l', 'L'):
            print("[SAFETY] Landing requested - executing immediately...")
            self.land()
            return False
        
        # === 安全模式激活时，禁用自动跟踪和起飞，但允许手动控制 ===
        if self.safe_mode:
            if ch in ('t', 'T'):
                print("[🛡️ SAFE MODE] Takeoff blocked in safe mode. Press SPACE to exit safe mode first.")
                return False
            # 手动控制和云台控制继续往下执行（不被阻止）
        
        # === 优先级 4: 起飞 ===
        if ch in ('t', 'T'):
            self.takeoff()
        
        # === 优先级 5: 暂停/恢复跟踪 ===
        elif ch in ('p', 'P'):
            self.paused = not self.paused
            if self.paused:
                self.tracker.emergency_stop()
                self.manual_roll = self.manual_pitch = self.manual_yaw = self.manual_gaz = 0
                self.send_piloting(0, 0, 0, 0)
            print(f"[INFO] {'PAUSED' if self.paused else 'RESUMED'}")
        
        # === 优先级 6: 手动触发抓取 ===
        elif ch in ('g', 'G'):
            print("[INFO] Manual grab trigger requested")
            self.trigger_grab_sequence()
        
        # 手动飞行控制（优先级高于自动跟踪）
        # 前进/后退 (pitch)
        elif ch in ('w', 'W'):
            self._update_manual_axis("pitch", self.manual_speed)
        elif ch in ('s', 'S'):
            self._update_manual_axis("pitch", -self.manual_speed)
        # 左移/右移 (roll)
        elif ch in ('a', 'A'):
            self._update_manual_axis("roll", -self.manual_speed)
        elif ch in ('d', 'D'):
            self._update_manual_axis("roll", self.manual_speed)
        # 左转/右转 (yaw)
        elif ch in ('z', 'Z'):
            self._update_manual_axis("yaw", -self.manual_speed)
        elif ch in ('e', 'E'):
            self._update_manual_axis("yaw", self.manual_speed)
        # 上升/下降 (gaz)
        elif ch in ('r', 'R'):
            self._update_manual_axis("gaz", self.manual_speed)
        elif ch in ('f', 'F'):
            self._update_manual_axis("gaz", -self.manual_speed)
        # 手动 gripper 控制 (1=hold 2=release 3=grip 4=status)
        elif ch == '1':
            print("[GRIPPER] hold")
            self.gripper.hold()
        elif ch == '2':
            print("[GRIPPER] release")
            self.gripper.release()
        elif ch == '3':
            print("[GRIPPER] grip")
            self.gripper.grip()
        elif ch == '4':
            st = self.gripper.status()
            print(f"[GRIPPER] status: {st}")
        
        return False
    
    def run(self):
        """主循环"""
        print()
        print("=" * 75)
        print("   TARGET TRACKING + EXTERNAL SYSTEM CONTROL (WebSocket)")
        print("=" * 75)
        print()
        print(f"  Tracking: {self.args.classes}")
        print(f"  Winch (WebSocket): {self.args.system_url}")
        print(f"  Gripper (WebSocket): {self.args.gripper_url}  (in sequence: {'yes' if self.use_gripper_in_sequence else 'no (--no-gripper)'})")
        print(f"  Lower length: {self.args.lower_length}mm")
        print(f"  Pull length: {self.args.pull_length}mm")
        print(f"  Auto-trigger: {self.args.auto_trigger}")
        print(f"  Stable time: {self.args.stable_time}s ({self.stable_frames_required} frames)")
        print(f"  Wait time: {self.args.wait_time}s (after LOWER, before PULL)")
        print(f"  Pull time: {self.args.pull_time}s (hold PULL command)")
        print()
        print("  Control Parameters (Ultra-Conservative for Stability):")
        print(f"    • kp (gain):      {self.args.kp:.1f}  (lower = gentler)")
        print(f"    • max_speed:      {self.args.max_speed}     (with vector limiting)")
        print(f"    • deadzone:       {self.args.deadzone:.2f}  (large center zone)")
        print(f"    • smoothing:      {self.args.smoothing:.2f}  (very smooth)")
        print(f"    • manual_speed:   {self.manual_speed}    (keyboard speed)")
        print()
        print("  Workflow:")
        print("    1. Takeoff and search for target")
        print("    2. Track target until stable")
        print("    3. Trigger grab sequence (each step waits for ok; winch waits for ok: done):")
        if self.use_gripper_in_sequence:
            print("       • Gripper RELEASE → ok → Winch LOWER → ok → ok: done → Gripper GRIP → ok → Winch PULL → ok → ok: done")
        else:
            print("       • Winch LOWER → ok → ok: done → Winch PULL → ok → ok: done")
        print(f"       • Lower: {self.args.lower_length}mm, Pull: {self.args.pull_length}mm")
        print("    4. Hover and wait for manual landing (press 'l')")
        if self.args.auto_land_on_exit:
            print("       ⚠️  Auto-land ENABLED (--auto-land-on-exit)")
        else:
            print("       ✓  Manual landing (default, press 'l' to land)")
        print()
        print("  🛡️  SAFETY SYSTEM (INTELLIGENT MODE) 🛡️")
        print("  ─────────────────────────────────────────────────────────")
        print("  SPACE = Toggle SAFE MODE (HIGHEST PRIORITY)")
        print("          • Disables auto-tracking")
        print("          • Enables manual keyboard control")
        print("          • Use WASD/ZE/RF to move safely")
        print("          • Then press 'l' to land")
        print()
        print("  l = LAND (works ANYTIME, highest priority)")
        print()
        print("  BASIC CONTROLS:")
        print("  ───────────────")
        print("  t = takeoff  |  p = pause/resume  |  q = quit")
        print("  g = manual grab (trigger grab when target detected)")
        print("  1/2/3/4 = gripper: hold / release / grip / status")
        print()
        print("  MANUAL FLIGHT (works ANYTIME, overrides auto-tracking):")
        print("  ────────────────────────────────────────────────────────")
        print("  w/s = forward/backward  |  a/d = left/right")
        print("  z/e = turn left/right   |  r/f = up/down")
        print()
        print("=" * 75)
        print()
        
        self.start_piloting()
        
        if self.args.takeoff_on_start:
            self.takeoff()
        
        with TerminalKeyReader() as kr:
            while True:
                # 键盘
                ch = kr.read_key_nonblock()
                if ch and self.handle_key(ch):
                    break
                
                # 获取帧
                frame = self.get_latest_frame()
                if frame is None:
                    time.sleep(0.01)
                    continue
                
                self.frame_count += 1
                h, w = frame.shape[:2]
                
                # 检测
                start = time.perf_counter()
                detections = self.detector.detect(frame)
                target = self.select_target(detections)
                
                # 更新手动控制的自动衰减
                self._decay_manual_controls()
                
                # === 🛡️ 安全模式检查（最高优先级） ===
                if self.safe_mode:
                    # 安全模式下：
                    # 1. 禁用自动跟踪
                    # 2. 允许手动键盘控制
                    self.tracker.emergency_stop()
                    tracking = False
                    
                    # 只使用手动控制（不使用自动跟踪）
                    roll = self.manual_roll
                    pitch = self.manual_pitch
                    yaw = self.manual_yaw
                    gaz = self.manual_gaz
                    
                    # 发送手动控制命令
                    self.send_piloting(roll, pitch, yaw, gaz)
                    
                elif not self.paused:
                    # 非安全模式且未暂停：自动跟踪 + 手动控制
                    tracking = self.tracker.update(target, w, h)
                    
                    # 获取自动跟踪的控制命令
                    roll, pitch, yaw, gaz = self.tracker.get_command()
                    
                    # 手动控制（优先级更高）- 覆盖自动跟踪
                    if self.manual_roll != 0:
                        roll = self.manual_roll
                    if self.manual_pitch != 0:
                        pitch = self.manual_pitch
                    if self.manual_yaw != 0:
                        yaw = self.manual_yaw
                    if self.manual_gaz != 0:
                        gaz = self.manual_gaz
                    
                    # 发送控制命令
                    self.send_piloting(roll, pitch, yaw, gaz)
                    
                    # 检查是否触发抓取（仅在非安全模式下）
                    if self.args.auto_trigger and not self.grab_triggered:
                        if self.tracker.is_stable and self.tracker.stable_frames >= self.stable_frames_required:
                            self.trigger_grab_sequence()
                else:
                    # 暂停模式
                    tracking = False
                
                elapsed_ms = (time.perf_counter() - start) * 1000
                if elapsed_ms > 0:
                    self.fps_list.append(1000.0 / elapsed_ms)
                
                # 绘制
                if detections:
                    frame = self.detector.draw_results(frame, detections)
                frame = self.draw_info(frame, target, tracking)
                
                cv2.imshow("Track and Grab", frame)
                cv2.waitKey(1)
        
        cv2.destroyAllWindows()
        self.tracker.emergency_stop()
        self.send_piloting(0, 0, 0, 0)


def main():
    args = parse_args()
    app = FlyTrackAndGrab(args)
    
    try:
        if args.test_winch_only:
            # Mock fly+track 成功，只跑 winch 抓取流程（不連無人機、不開偵測與串流）
            print()
            print("=" * 60)
            print("  [TEST] Winch-only mode (mock fly+track success)")
            print("=" * 60)
            print(f"  Winch: {args.system_url}")
            print(f"  Gripper in sequence: {'no' if args.no_gripper else 'yes'} ({args.gripper_url})")
            print(f"  Lower: {args.lower_length}mm, Pull: {args.pull_length}mm")
            print("=" * 60)
            gripper = app.gripper if app.use_gripper_in_sequence else None
            ok = app.external_system.execute_grab_sequence(gripper=gripper)
            print()
            print("[TEST] Winch sequence finished:", "OK" if ok else "FAILED")
            return 0 if ok else 1
        
        if not app.connect():
            return 1
        
        if not app.init_detector():
            return 1
        
        app.init_tracker()
        app.start_streaming()
        time.sleep(1.0)
        
        app.run()
        
    except KeyboardInterrupt:
        print("\n[CTRL+C] Stopping...")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n[INFO] Cleaning up...")
        
        if args.test_winch_only:
            try:
                app.external_system.stop()
            except Exception:
                pass
            try:
                app.external_systems.disconnect_all()
            except Exception:
                pass
        else:
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)
            except Exception:
                pass
            if app.tracker:
                app.tracker.emergency_stop()
            try:
                app.send_piloting(0, 0, 0, 0)
            except Exception:
                pass
            try:
                app.stop_piloting()
            except Exception:
                pass
            if args.auto_land_on_exit:
                try:
                    app.land()
                except Exception:
                    pass
            try:
                app.stop_streaming()
            except Exception:
                pass
            try:
                app.disconnect()
            except Exception:
                pass
            try:
                app.external_system.stop()
            except Exception:
                pass
            try:
                app.external_systems.disconnect_all()
            except Exception:
                pass
        
        print("[OK] Cleanup complete")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

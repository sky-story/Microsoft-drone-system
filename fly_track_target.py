#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ANAFI Ai: 目标跟踪脚本（摄像头朝下版本）
适用于：海面物体回收等场景，摄像头固定朝下

原理：
  - 摄像头朝下，检测画面中的目标
  - 通过机身移动（roll/pitch）使目标居中
  - 不移动云台，云台保持固定角度

安全特性：
  - 极低的最大速度（默认 10）
  - 平滑控制，避免突然运动
  - 死区防抖
  - 紧急停止按键
  - 自动降落

使用方法：
  python fly_track_target.py --classes person
  python fly_track_target.py --classes chair --max-speed 8
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

import cv2
import numpy as np

import olympe
import olympe.log
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages import gimbal

from object_detector import ObjectDetector, Detection

import tty
import termios
import select


class SafeTracker:
    """
    安全的目标跟踪控制器
    
    支持两种摄像头模式：
    
    模式 "down"（摄像头朝下，用于海面回收）：
      画面左右 → roll（左右平移）
      画面上下 → pitch（前后移动）
      
    模式 "forward"（摄像头平视，用于测试）：
      画面左右 → yaw（左右旋转）
      画面上下 → gaz（上下升降）
    """
    
    def __init__(
        self,
        camera_mode: str = "down",     # "forward" 或 "down"，默认 down（摄像头朝下）
        max_speed: int = 5,            # 最大速度（非常保守，防止过冲）
        smoothing: float = 0.05,       # 平滑系数（越小越平滑，0.05=极平滑）
        deadzone: float = 0.2,         # 死区（大死区防止抖动和过度反应）
        kp: float = 5.0,               # 比例增益（低增益防止过度反应）
        invert_roll: bool = False,     # 反转 roll 方向
        invert_pitch: bool = False,    # 反转 pitch 方向
    ):
        # 模式
        self.camera_mode = camera_mode
        
        # 安全参数
        self.max_speed = max_speed
        self.smoothing = smoothing
        self.deadzone = deadzone
        self.kp = kp
        
        # 控制方向反转（用于调试）
        self.invert_roll = invert_roll
        self.invert_pitch = invert_pitch
        
        # 当前控制输出（4个轴）
        self.roll = 0.0   # 左右平移
        self.pitch = 0.0  # 前后移动
        self.yaw = 0.0    # 左右旋转
        self.gaz = 0.0    # 上下升降
        
        # 跟踪状态
        self.target_lost_frames = 0
        self.max_lost_frames = 20  # 丢失目标多少帧后停止
        
    def update(self, detection: Optional[Detection], frame_width: int, frame_height: int) -> bool:
        """
        更新控制量
        
        Args:
            detection: 检测结果
            frame_width: 画面宽度
            frame_height: 画面高度
            
        Returns:
            True = 正在跟踪, False = 目标丢失
        """
        if detection is None:
            self.target_lost_frames += 1
            if self.target_lost_frames >= self.max_lost_frames:
                # 丢失太久，平滑停止
                self._smooth_stop()
            return False
        
        self.target_lost_frames = 0
        
        # 计算目标中心
        cx, cy = detection.center
        
        # 计算偏移量（-1 到 +1）
        # offset_x: 正 = 目标在右边
        # offset_y: 正 = 目标在下边
        offset_x = (cx - frame_width / 2) / (frame_width / 2)
        offset_y = (cy - frame_height / 2) / (frame_height / 2)
        
        # 应用死区
        if abs(offset_x) < self.deadzone:
            offset_x = 0
        if abs(offset_y) < self.deadzone:
            offset_y = 0
        
        # 根据摄像头模式计算控制量
        target_roll = 0.0
        target_pitch = 0.0
        target_yaw = 0.0
        target_gaz = 0.0
        
        if self.camera_mode == "down":
            # 摄像头朝下：画面左右→roll，画面上下→pitch
            # 
            # 标准映射：
            # offset_x > 0 = 目标在画面右侧 → roll + (向右移动)
            # offset_x < 0 = 目标在画面左侧 → roll - (向左移动)
            # offset_y < 0 = 目标在画面上方 → pitch + (向前移动)
            # offset_y > 0 = 目标在画面下方 → pitch - (向后移动)
            roll_multiplier = -1 if self.invert_roll else 1
            pitch_multiplier = 1 if self.invert_pitch else -1
            
            target_roll = offset_x * self.kp * roll_multiplier
            target_pitch = offset_y * self.kp * pitch_multiplier
        else:
            # 摄像头平视(forward)：画面左右→yaw，画面上下→gaz
            target_yaw = offset_x * self.kp        # 目标在右边 → 向右旋转
            target_gaz = -offset_y * self.kp       # 目标在上方 → 向上升
        
        # 限制最大速度（每个轴独立限制）
        target_roll = self._clamp(target_roll, -self.max_speed, self.max_speed)
        target_pitch = self._clamp(target_pitch, -self.max_speed, self.max_speed)
        target_yaw = self._clamp(target_yaw, -self.max_speed, self.max_speed)
        target_gaz = self._clamp(target_gaz, -self.max_speed, self.max_speed)
        
        # 重要优化：当目标在角落时（同时有roll和pitch），限制总速度
        # 这防止在对角线方向上速度过大导致过冲
        if self.camera_mode == "down":
            # 计算 roll 和 pitch 的矢量长度
            magnitude = math.sqrt(target_roll**2 + target_pitch**2)
            if magnitude > self.max_speed:
                # 等比例缩放，保持方向但限制总速度
                scale = self.max_speed / magnitude
                target_roll *= scale
                target_pitch *= scale
        else:
            # forward 模式：限制 yaw 和 gaz 的组合速度
            magnitude = math.sqrt(target_yaw**2 + target_gaz**2)
            if magnitude > self.max_speed:
                scale = self.max_speed / magnitude
                target_yaw *= scale
                target_gaz *= scale
        
        # 平滑过渡
        s = self.smoothing
        self.roll = self.roll * (1 - s) + target_roll * s
        self.pitch = self.pitch * (1 - s) + target_pitch * s
        self.yaw = self.yaw * (1 - s) + target_yaw * s
        self.gaz = self.gaz * (1 - s) + target_gaz * s
        
        return True
    
    def _smooth_stop(self):
        """平滑停止"""
        s = self.smoothing
        self.roll *= (1 - s)
        self.pitch *= (1 - s)
        self.yaw *= (1 - s)
        self.gaz *= (1 - s)
        
        # 接近零时直接置零
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
        description="ANAFI Ai: Safe Target Tracking (Camera Down)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Usage Examples:
  # Basic usage (very conservative defaults to prevent overshoot)
  python fly_track_target.py --classes chair
  
  # Ultra gentle (if target still flies out, especially at corners)
  python fly_track_target.py --classes person --kp 3 --max-speed 3 --deadzone 0.25
  
  # More aggressive tracking (if too slow to follow)
  python fly_track_target.py --classes person --kp 10 --max-speed 10 --smoothing 0.15

Camera Modes:
  --camera-mode down (default, camera looking down 90°):
    Target on RIGHT  -> drone moves RIGHT   (roll +)
    Target on LEFT   -> drone moves LEFT    (roll -)
    Target on TOP    -> drone moves FORWARD (pitch +)
    Target on BOTTOM -> drone moves BACK    (pitch -)

  --camera-mode forward (camera level, for testing):
    Target on RIGHT  -> drone turns RIGHT (yaw +)
    Target on LEFT   -> drone turns LEFT  (yaw -)
    Target on TOP    -> drone goes UP     (gaz +)
    Target on BOTTOM -> drone goes DOWN   (gaz -)

Safety & Tuning:
  - Ultra-conservative defaults to prevent overshoot (especially at corners!)
  - Default: kp=5.0, max_speed=5, deadzone=0.2, smoothing=0.05
  - Vector speed limiting: prevents diagonal overshoot at corners
  - Large dead zone prevents jittering and over-correction
  
  Problem: Target flies out at corners (e.g., top-right)?
    Solution: --kp 3 --max-speed 3 --deadzone 0.25
  
  Problem: Tracking too slow?
    Solution: --kp 10 --max-speed 10 --smoothing 0.15
  
  - Press SPACE for safe mode
  - Auto land on exit

Keys (Priority Order):
  SPACE = 🛡️ SAFE MODE (toggle) - HIGHEST PRIORITY
          When ON: Auto-tracking DISABLED, manual control ENABLED
          Use WASD/ZE/RF to move to safe landing position
          Press SPACE again to resume auto-tracking
  
  l = LAND - ALWAYS AVAILABLE (works anytime)
  
  t = takeoff (disabled in safe mode)
  p = pause/resume tracking
  q/ESC = quit

Manual Flight Control (ALWAYS available, priority over auto-tracking):
  w / s = forward / backward (pitch + / -)
  a / d = left / right (roll - / +)
  z / e = turn left / turn right (yaw - / +)
  r / f = up / down (gaz + / -)

Gimbal Control:
  i = gimbal up (pitch +)
  k = gimbal down (pitch -)
  j = gimbal left (yaw -)
  u = gimbal right (yaw +)
  o = stop gimbal
        """
    )
    
    # 连接
    p.add_argument("--drone-ip", default=os.environ.get("DRONE_IP", "192.168.42.1"))
    p.add_argument("--loglevel", default="WARNING", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    
    # 检测
    p.add_argument("--model", default="yolov8n.pt", help="YOLO model")
    p.add_argument("--conf", type=float, default=0.5, help="Detection confidence")
    p.add_argument("--classes", nargs="+", type=str, required=True,
                   help="Target classes to track (e.g., --classes person)")
    
    # 跟踪控制 - 安全参数
    p.add_argument("--max-speed", type=int, default=5,
                   help="Max speed (0-100, default 5, VERY SAFE to prevent overshoot)")
    p.add_argument("--smoothing", type=float, default=0.05,
                   help="Smoothing (0-1, lower=smoother, default 0.05 for very stable tracking)")
    p.add_argument("--deadzone", type=float, default=0.2,
                   help="Dead zone (0-0.5, default 0.2, large center area prevents jittering)")
    p.add_argument("--kp", type=float, default=5.0,
                   help="Proportional gain (default 5.0, lower=gentler response, prevents overshoot)")
    
    # 摄像头模式
    p.add_argument("--camera-mode", choices=["forward", "down"], default="down",
                   help="Camera mode: 'forward'=level view (yaw+gaz), 'down'=looking down (roll+pitch). Default: down")
    
    # 控制方向反转（用于调试和修正）
    p.add_argument("--invert-roll", action="store_true",
                   help="Invert roll direction (for debugging DOWN mode)")
    p.add_argument("--invert-pitch", action="store_true",
                   help="Invert pitch direction (for debugging DOWN mode)")
    
    # 云台
    p.add_argument("--gimbal-speed", type=float, default=0.15,
                   help="Gimbal control speed (default 0.15)")
    
    # 手动飞行控制
    p.add_argument("--manual-speed", type=int, default=25,
                   help="Manual control speed (0-100, default 25)")
    
    # 起飞/降落
    p.add_argument("--takeoff-on-start", action="store_true")
    p.add_argument("--auto-land-on-exit", action="store_true", default=True)
    p.add_argument("--no-auto-land", action="store_false", dest="auto_land_on_exit")
    
    return p.parse_args()


class FlyTrackTarget:
    """目标跟踪主程序"""
    
    def __init__(self, args):
        self.args = args
        olympe.log.update_config({"loggers": {"olympe": {"level": args.loglevel}}})
        
        self.drone = olympe.Drone(args.drone_ip)
        self.detector = None
        self.tracker = None
        self.running = False
        self.paused = False
        self.safe_mode = False  # 安全模式：最高优先级，禁用所有控制
        
        # 视频帧队列
        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_lock = threading.Lock()
        
        # 云台控制
        self.gimbal_pitch_speed = 0.0
        self.gimbal_yaw_speed = 0.0
        self.gimbal_speed = args.gimbal_speed
        
        # 手动飞行控制（优先级高于自动跟踪）
        self.manual_roll = 0
        self.manual_pitch = 0
        self.manual_yaw = 0
        self.manual_gaz = 0
        self.manual_speed = args.manual_speed  # 手动控制速度
        self.last_manual_time = {"roll": 0, "pitch": 0, "yaw": 0, "gaz": 0}
        self.manual_timeout = 0.25  # 松开按键后多久自动回零
        
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
            camera_mode=self.args.camera_mode,
            max_speed=self.args.max_speed,
            smoothing=self.args.smoothing,
            deadzone=self.args.deadzone,
            kp=self.args.kp,
            invert_roll=self.args.invert_roll,
            invert_pitch=self.args.invert_pitch,
        )
        mode_desc = "yaw+gaz" if self.args.camera_mode == "forward" else "roll+pitch"
        invert_info = ""
        if self.args.invert_roll or self.args.invert_pitch:
            invert_info = f" [invert: roll={self.args.invert_roll}, pitch={self.args.invert_pitch}]"
        print(f"[OK] Tracker ready (mode={self.args.camera_mode} [{mode_desc}], max_speed={self.args.max_speed}){invert_info}")
    
    # ---------- 视频回调 ----------
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
                # 先清空队列
                self.flush_cb(None)
            except Exception:
                pass
            
            try:
                self.drone.streaming.stop()
            except Exception as e:
                print(f"[WARN] streaming.stop error: {e}")
            
            # 等待一下让资源释放
            time.sleep(0.5)
            self.running = False
            print("[OK] Streaming stopped")
    
    def disconnect(self):
        print("[INFO] Disconnecting ...")
        # 等待一下确保流已停止
        time.sleep(0.3)
        try:
            self.drone.disconnect()
            print("[OK] Disconnected")
        except Exception as e:
            print(f"[WARN] disconnect error: {e}")
    
    # ---------- 飞行控制 ----------
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
        """
        发送飞行控制命令
        
        Args:
            roll: 左右平移 (-100 ~ +100)，正=右移
            pitch: 前后移动 (-100 ~ +100)，正=前进
            yaw: 左右旋转 (-100 ~ +100)，正=右转
            gaz: 上下升降 (-100 ~ +100)，正=上升
        """
        self.drone.piloting(roll, pitch, yaw, gaz, 0.05)
    
    def send_gimbal_velocity(self, pitch_speed: float, yaw_speed: float):
        """发送云台速度控制命令"""
        try:
            self.drone(
                gimbal.set_target(
                    gimbal_id=0,
                    control_mode="velocity",
                    yaw_frame_of_reference="relative",
                    yaw=float(yaw_speed),
                    pitch_frame_of_reference="relative",
                    pitch=float(pitch_speed),
                    roll_frame_of_reference="none",
                    roll=0.0,
                )
            )
        except Exception as e:
            print(f"[WARN] Gimbal error: {e}")
    
    def stop_gimbal(self):
        """停止云台运动"""
        self.gimbal_pitch_speed = 0.0
        self.gimbal_yaw_speed = 0.0
        self.send_gimbal_velocity(0.0, 0.0)
    
    # ---------- 获取帧 ----------
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
    
    def draw_info(self, frame, detection, tracking):
        """绘制信息"""
        h, w = frame.shape[:2]
        
        # === 安全模式：醒目的橙色警告（最高优先级显示） ===
        if self.safe_mode:
            # 全屏橙色半透明遮罩
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, h), (0, 140, 255), -1)
            frame = cv2.addWeighted(overlay, 0.12, frame, 0.88, 0)
            
            # 大号警告文字
            warning = "[SAFE MODE] - AUTO-TRACKING OFF"
            text_size = cv2.getTextSize(warning, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 3)[0]
            text_x = (w - text_size[0]) // 2
            
            # 橙色背景
            cv2.rectangle(frame, (text_x - 20, 10), (text_x + text_size[0] + 20, 55), (0, 100, 200), -1)
            cv2.rectangle(frame, (text_x - 20, 10), (text_x + text_size[0] + 20, 55), (0, 165, 255), 3)
            
            # 白色警告文字
            cv2.putText(frame, warning, (text_x, 42), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
            
            # 提示信息
            hint1 = "Manual Control ENABLED: Use WASD/ZE/RF to move to safe position"
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
            ctrl = f"roll: {roll:+3d} | pitch: {pitch:+3d} | yaw: {yaw:+3d} | gaz: {gaz:+3d} [SAFE MODE - MANUAL ONLY]"
            cv2.putText(frame, ctrl, (10, h - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
            
            return frame
        
        # === 正常模式显示 ===
        # 半透明背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 90), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
        
        # 状态
        if self.paused:
            status = "PAUSED - press 'p' to resume"
            color = (0, 165, 255)
        elif tracking and detection:
            status = f"TRACKING: {detection.class_name} ({detection.confidence:.2f})"
            color = (0, 255, 0)
        else:
            status = "SEARCHING... no target"
            color = (0, 255, 255)
        
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
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
        
        if self.args.camera_mode == "forward":
            ctrl = f"yaw: {yaw:+3d} | gaz: {gaz:+3d} | roll: {roll:+3d} | pitch: {pitch:+3d}"
        else:
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
        
        # 显示手动控制状态
        if manual_active:
            ctrl += " [MANUAL]"
        cv2.putText(frame, ctrl, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # 模式和安全参数
        mode_str = "forward" if self.args.camera_mode == "forward" else "down"
        safety = f"mode: {mode_str} | kp: {self.args.kp} | max_speed: {self.args.max_speed} | deadzone: {self.args.deadzone} | smooth: {self.args.smoothing}"
        cv2.putText(frame, safety, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
        
        # 画面中心十字
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx - 40, cy), (cx + 40, cy), (255, 255, 255), 1)
        cv2.line(frame, (cx, cy - 40), (cx, cy + 40), (255, 255, 255), 1)
        cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)
        
        # 目标和连线
        if detection:
            x1, y1, x2, y2 = detection.bbox
            tx, ty = detection.center
            
            # 检测框
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 目标中心
            cv2.circle(frame, (tx, ty), 8, (0, 255, 0), -1)
            
            # 连线（中心到目标）
            cv2.line(frame, (cx, cy), (tx, ty), (0, 255, 255), 2)
            
            # 显示偏移量和控制方向
            offset_x = (tx - cx) / (w / 2)
            offset_y = (ty - cy) / (h / 2)
            offset_text = f"offset: x={offset_x:+.2f} y={offset_y:+.2f}"
            cv2.putText(frame, offset_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # 显示控制方向提示（帮助调试）
            if self.args.camera_mode == "down":
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
        help_text = "SPACE=[SAFE]:stop_track+manual | l=LAND t=takeoff p=pause | wasd/ze/rf=move | ikju=gimbal"
        cv2.putText(frame, help_text, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (150, 150, 150), 1)
        
        return frame
    
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
    
    def handle_key(self, ch) -> bool:
        """
        处理按键，返回 True 表示退出
        
        优先级顺序（从高到低）：
        1. 退出 (q/ESC)
        2. 安全模式 (SPACE) - 最高安全优先级
        3. 降落 (l) - 随时可以降落
        4. 起飞 (t)
        5. 其他控制
        """
        # === 优先级 1: 退出 ===
        if ch in ('\x1b', 'q', 'Q'):
            return True
        
        # === 优先级 2: 安全模式（最高优先级） ===
        if ch == ' ':
            self.safe_mode = not self.safe_mode
            if self.safe_mode:
                # 进入安全模式：
                # 1. 立即停止自动跟踪
                # 2. 停止当前运动
                # 3. 但允许手动键盘控制（可以移动到安全降落点）
                self.tracker.emergency_stop()
                self.manual_roll = self.manual_pitch = self.manual_yaw = self.manual_gaz = 0
                self.send_piloting(0, 0, 0, 0)
                self.stop_gimbal()
                print("[🛡️ SAFE MODE] ON - Auto-tracking DISABLED. Manual control ENABLED.")
                print("[🛡️ SAFE MODE] Use WASD/ZE/RF to move to safe landing position, then press 'l' to land.")
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
            # 在安全模式下，只禁用起飞
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
        
        # 云台控制
        elif ch in ('i', 'I'):
            # 云台向上（pitch +）
            self.gimbal_pitch_speed = self.gimbal_speed
            self.send_gimbal_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ('k', 'K'):
            # 云台向下（pitch -）
            self.gimbal_pitch_speed = -self.gimbal_speed
            self.send_gimbal_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ('j', 'J'):
            # 云台向左（yaw -）
            self.gimbal_yaw_speed = -self.gimbal_speed
            self.send_gimbal_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ('u', 'U'):
            # 云台向右（yaw +）
            self.gimbal_yaw_speed = self.gimbal_speed
            self.send_gimbal_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ('o', 'O'):
            # 停止云台
            self.stop_gimbal()
            print("[INFO] Gimbal stopped")
        
        return False
    
    def run(self):
        """主循环"""
        print()
        print("=" * 75)
        print("   TARGET TRACKING WITH INTELLIGENT SAFETY SYSTEM")
        print("=" * 75)
        print()
        print(f"  Tracking: {self.args.classes}")
        print(f"  Camera mode: {self.args.camera_mode}")
        if self.args.camera_mode == "forward":
            print("    -> yaw (turn) + gaz (up/down)")
        else:
            print("    -> roll (strafe) + pitch (fwd/back)")
        print()
        print("  Control Parameters (Ultra-Conservative for Stability):")
        print(f"    • kp (gain):      {self.args.kp:.1f}  (lower = gentler, prevents overshoot)")
        print(f"    • max_speed:      {self.args.max_speed}     (speed limit, with vector limiting)")
        print(f"    • deadzone:       {self.args.deadzone:.2f}  (large center zone, stable)")
        print(f"    • smoothing:      {self.args.smoothing:.2f}  (very low = very smooth)")
        print(f"    • manual_speed:   {self.args.manual_speed}    (keyboard control speed)")
        print()
        print("  ⚠️  Corner Overshoot Prevention ENABLED")
        print("      → When target at corner, total speed is limited")
        print()
        print("  💡 Target flies out (especially at corners)? → --kp 3 --max-speed 3")
        print("  💡 Tracking too slow? → --kp 10 --max-speed 10")
        print()
        print("  🛡️  SAFETY SYSTEM (INTELLIGENT MODE) 🛡️")
        print("  ─────────────────────────────────────────────────────────")
        print("  SPACE = Toggle SAFE MODE")
        print("          • Disables auto-tracking (stops following target)")
        print("          • Enables manual keyboard control")
        print("          • Use WASD/ZE/RF to move to safe landing position")
        print("          • Then press 'l' to land safely")
        print()
        print("  l = LAND (works ANYTIME, highest priority for safety)")
        print()
        print("  BASIC CONTROLS:")
        print("  ───────────────")
        print("  t = takeoff")
        print("  p = pause/resume auto-tracking")
        print("  q = quit program")
        print()
        print("  MANUAL FLIGHT (works ANYTIME, overrides auto-tracking):")
        print("  ────────────────────────────────────────────────────────")
        print("  w/s = forward/backward  |  a/d = left/right")
        print("  z/e = turn left/right   |  r/f = up/down")
        print()
        print("  GIMBAL CONTROL:")
        print("  ───────────────")
        print("  i/k = pitch up/down  |  j/u = yaw left/right  |  o = stop")
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
                
                # 选择目标
                target = self.select_target(detections)
                
                # 更新手动控制的自动衰减
                self._decay_manual_controls()
                
                # === 安全模式检查（最高优先级） ===
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
                
                cv2.imshow("Target Tracking (Camera Down)", frame)
                cv2.waitKey(1)
        
        cv2.destroyAllWindows()
        
        # 确保停止
        self.tracker.emergency_stop()
        self.send_piloting(0, 0, 0, 0)


def main():
    args = parse_args()
    app = FlyTrackTarget(args)
    
    try:
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
        
        # 1. 销毁 OpenCV 窗口
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)  # 让窗口有机会关闭
        except Exception:
            pass
        
        # 2. 停止跟踪和飞行控制
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
        
        # 3. 自动降落（如果设置了）
        if args.auto_land_on_exit:
            try:
                app.land()
            except Exception:
                pass
        
        # 4. 停止视频流
        try:
            app.stop_streaming()
        except Exception:
            pass
        
        # 5. 断开连接
        try:
            app.disconnect()
        except Exception:
            pass
        
        print("[OK] Cleanup complete")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

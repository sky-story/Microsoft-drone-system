#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
橙色物体颜色追踪 - 完整测试脚本（含无人机起飞/降落/手动控制/紧急停止）
========================================================================
参考 fly_track_and_grab.py 的 SafeTracker + 安全模式 + WASD 手动控制，
在此基础上增加了 HSV 颜色检测、EMA 平滑、形态学滤波等增强。

三种模式：
  calibrate  — 连接无人机看画面 + 调 HSV 参数（不发飞控命令）
  track      — 完整追踪：起飞/降落/手动控制/自动追踪（参考 fly_track_and_grab.py）
  webcam     — 离线用笔记本摄像头调参（--webcam）

用法：
  python test_color_tracking.py --mode calibrate
  python test_color_tracking.py --mode track
  python test_color_tracking.py --webcam

按键（track 模式完整列表）：
  ================================================================
  最高优先级：
    SPACE = 紧急停止（立即停止追踪 + 悬停，进入安全模式）
    L     = 降落（随时可用，不受任何模式限制）
    Q/ESC = 退出（自动悬停+断开连接）
  ----------------------------------------------------------------
  飞行控制：
    1     = 起飞（约1m悬停，起飞后用 R/F 调高度）
    T     = 开启/关闭自动追踪（需先起飞并到合适高度）
  ----------------------------------------------------------------
  手动飞行（优先级高于自动追踪，松开后 0.25s 自动归零）：
    W/S   = 前进/后退 (pitch)
    A/D   = 左移/右移 (roll)
    Z/E   = 左转/右转 (yaw)
    R/F   = 上升/下降 (gaz)  ← 起飞后用这个调节高度
  ----------------------------------------------------------------
  HSV 校准：
    P     = 保存 HSV 参数
    O     = 重置 HSV 为默认橙色
  ================================================================
"""
import os
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import argparse
import json
import math
import queue
import time
import threading
from pathlib import Path

import cv2
import numpy as np

# ========== 默认橙色 HSV 范围 ==========
# 目标：亮橙色（高饱和度、高亮度）
# 排除：木头/棕色（低饱和度、低亮度的暗淡橙色）
# S_min=150 过滤掉暗淡/灰蒙的颜色，V_min=120 过滤掉阴影/暗处
DEFAULT_HSV = {
    "h_min": 5,   "h_max": 25,
    "s_min": 178, "s_max": 255,
    "v_min": 120, "v_max": 255,
}

# ========== 追踪参数 ==========
# 基于 fly_track_and_grab.py SafeTracker 默认值，针对颜色追踪场景做防过冲调优。
# fly_track_and_grab.py 建议防过冲：--kp 3 --max-speed 3 --deadzone 0.25
STABILITY_THRESHOLD = 0.25   # 偏移量 < 此值算 "centered"
DEADZONE = 0.20              # 死区（放大→靠近中心时更早停止，减少来回震荡）
KP = 3.0                     # 比例增益（降低→减少过冲）
MAX_SPEED = 3                # 最大控制量（降低→限制最大飞行速度）
SMOOTHING_CTRL = 0.10        # 控制量平滑系数（提高→更快响应方向变化，减少惯性过冲）
MAX_LOST_FRAMES = 20         # 目标丢失多少帧后渐进减速
MANUAL_SPEED = 25            # 手动控制速度
MANUAL_TIMEOUT = 0.25        # 手动控制松开后归零的超时

# 颜色检测增强参数（SafeTracker 无此功能，为本脚本新增）
EMA_ALPHA = 0.4              # 中心点 EMA 平滑系数
MIN_AREA_RATIO = 0.001       # 最小轮廓面积占帧面积比例
MORPH_KERNEL_SIZE = 7        # 形态学核大小
GAUSSIAN_BLUR_SIZE = 5       # 高斯模糊核大小


# =====================================================================
#  Parrot Anafi 无人机连接（参考 drone_control_gui.py + fly_track_and_grab.py）
# =====================================================================

class DroneVideoSource:
    """通过 Olympe 连接 Parrot Anafi，提供视频帧 + 完整飞行控制"""

    def __init__(self, ip: str = "192.168.42.1"):
        self.ip = ip
        self.drone = None
        self.connected = False
        self.piloting_started = False
        self._olympe = None
        self._FlyingStateChanged = None
        self._TakeOff = None
        self._Landing = None

        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_lock = threading.Lock()
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.streaming = False

    def connect(self) -> bool:
        try:
            import olympe
            import olympe.log
            from olympe.messages.ardrone3.Piloting import TakeOff, Landing
            from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
            olympe.log.update_config({
                "loggers": {
                    "olympe": {"level": "WARNING"},
                    "ulog": {"level": "ERROR"},
                }
            })
            self._olympe = olympe
            self._FlyingStateChanged = FlyingStateChanged
            self._TakeOff = TakeOff
            self._Landing = Landing
        except ImportError:
            print("[ERROR] olympe 未安装，请使用 --webcam 模式")
            return False

        print(f"[INFO] Connecting to drone at {self.ip} ...")
        self.drone = self._olympe.Drone(self.ip)

        for attempt in range(3):
            try:
                if self.drone.connect():
                    self.connected = True
                    print(f"[OK] Connected to drone: {self.ip}")
                    time.sleep(1)
                    self._start_streaming()
                    return True
            except Exception as e:
                print(f"[WARN] Attempt {attempt + 1}/3 failed: {e}")
                time.sleep(1)

        print("[ERROR] Failed to connect after 3 attempts")
        return False

    def disconnect(self):
        self.streaming = False
        if self.drone and self.connected:
            try:
                self._flush_cb(None)
            except Exception:
                pass
            try:
                self.drone.streaming.stop()
            except Exception:
                pass
            if self.piloting_started:
                try:
                    self.drone.stop_piloting()
                except Exception:
                    pass
            time.sleep(0.3)
            try:
                self.drone.disconnect()
            except Exception:
                pass
            self.connected = False
            print("[INFO] Disconnected from drone")

    def start_piloting(self):
        if not self.piloting_started and self.drone and self.connected:
            try:
                self.drone.start_piloting()
                self.piloting_started = True
                print("[INFO] Piloting interface started")
            except Exception as e:
                print(f"[WARN] Start piloting failed: {e}")

    def takeoff(self) -> bool:
        """
        起飞（参考 fly_track_and_grab.py）
        Parrot Anafi TakeOff() 默认悬停在约 1m 高度。
        起飞后用 R/F 键手动调整高度（不做自动爬升，避免阻塞按键响应）。
        """
        if not self.drone or not self.connected:
            print("[ERROR] Drone not connected")
            return False
        self.start_piloting()
        print("[INFO] Taking off ...")
        try:
            result = self.drone(
                self._TakeOff()
                >> self._FlyingStateChanged(state="hovering", _timeout=20)
            ).wait()
            if result.success():
                print("[OK] Hovering at ~1m")
                print("[TIP] Use R key to go UP, F key to go DOWN")
                return True
            else:
                time.sleep(2)
                st = self._get_flying_state()
                if st in ("hovering", "flying", "takingoff"):
                    print(f"[OK] Takeoff confirmed via state check: {st}")
                    print("[TIP] Use R key to go UP, F key to go DOWN")
                    return True
                print("[ERROR] Takeoff failed")
                return False
        except Exception as e:
            print(f"[ERROR] Takeoff exception: {e}")
            return False

    def land(self) -> bool:
        """降落（参考 fly_track_and_grab.py）"""
        if not self.drone or not self.connected:
            print("[ERROR] Drone not connected")
            return False
        print("[INFO] Landing ...")
        try:
            result = self.drone(
                self._Landing()
                >> self._FlyingStateChanged(state="landed", _timeout=30)
            ).wait()
            if result.success():
                print("[OK] Landed")
                return True
            else:
                time.sleep(2)
                st = self._get_flying_state()
                if st in ("landed", "landing"):
                    print(f"[OK] Landing confirmed via state check: {st}")
                    return True
                print("[ERROR] Landing failed")
                return False
        except Exception as e:
            print(f"[ERROR] Landing exception: {e}")
            return False

    def _get_flying_state(self) -> str:
        try:
            st = self.drone.get_state(self._FlyingStateChanged)
            if st:
                state = st.get("state")
                return getattr(state, "name", str(state)) if state else "unknown"
        except Exception:
            pass
        return "unknown"

    def send_piloting(self, roll: int, pitch: int, yaw: int = 0, gaz: int = 0):
        if self.drone and self.connected and self.piloting_started:
            self.drone.piloting(roll, pitch, yaw, gaz, 0.05)

    def hover(self):
        """多次发送悬停确保生效（参考 drone_control_gui.py._send_hover）"""
        if self.drone and self.connected and self.piloting_started:
            for _ in range(5):
                self.drone.piloting(0, 0, 0, 0, 0.05)
                time.sleep(0.05)

    def get_frame(self):
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None

    # ---- 内部视频流方法 ----

    def _yuv_frame_cb(self, yuv_frame):
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
        with self.flush_lock:
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait().unref()
                except Exception:
                    pass
        return True

    def _get_latest_frame(self):
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
                self._olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                self._olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }.get(fmt)
            if cv2_flag is None:
                return None
            yuv = last.as_ndarray()
            return cv2.cvtColor(yuv, cv2_flag)
        except Exception:
            return None
        finally:
            try:
                last.unref()
            except Exception:
                pass

    def _start_streaming(self):
        self.drone.streaming.set_callbacks(
            raw_cb=self._yuv_frame_cb,
            flush_raw_cb=self._flush_cb,
        )
        self.drone.streaming.start()
        self.streaming = True
        print("[INFO] Video streaming started")

        def frame_updater():
            while self.streaming and self.connected:
                frame = self._get_latest_frame()
                if frame is not None:
                    with self.frame_lock:
                        self.current_frame = frame
                    time.sleep(0.008)
                else:
                    time.sleep(0.01)

        threading.Thread(target=frame_updater, daemon=True).start()


class WebcamVideoSource:
    """笔记本摄像头视频源（离线调试用）"""

    def __init__(self, source=0):
        self.source = source
        self.cap = None
        self.connected = False
        self.piloting_started = False

    def connect(self) -> bool:
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            print(f"[ERROR] Cannot open webcam: {self.source}")
            return False
        ret, frame = self.cap.read()
        if ret:
            h, w = frame.shape[:2]
            print(f"[INFO] Webcam opened: {w}x{h}")
            self.connected = True
            return True
        return False

    def disconnect(self):
        if self.cap:
            self.cap.release()
        self.connected = False

    def start_piloting(self):
        self.piloting_started = True

    def takeoff(self) -> bool:
        print("[SIM] Takeoff (simulated)")
        return True

    def land(self) -> bool:
        print("[SIM] Land (simulated)")
        return True

    def send_piloting(self, roll: int, pitch: int, yaw: int = 0, gaz: int = 0):
        pass

    def hover(self):
        pass

    def get_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            return frame if ret else None
        return None


# =====================================================================
#  颜色追踪器（含 SafeTracker 完整控制逻辑 + 颜色检测增强）
# =====================================================================

class ColorTracker:
    """
    橙色物体颜色追踪器

    检测流水线（新增）：GaussianBlur → HSV → inRange → morphologyEx → contour → EMA
    控制流水线（对齐 SafeTracker）：deadzone → P 控制 → 矢量限速 → 平滑 → 丢失渐进减速
    """

    def __init__(self, hsv_params: dict = None):
        self.hsv = hsv_params or DEFAULT_HSV.copy()

        # EMA 平滑中心点
        self._smooth_cx = None
        self._smooth_cy = None

        # 控制量（对齐 SafeTracker 字段名）
        self._smooth_roll = 0.0
        self._smooth_pitch = 0.0

        # 稳定性（对齐 SafeTracker.stable_frames）
        self.stable_count = 0
        self.stable_required = 5

        # 丢失帧计数（对齐 SafeTracker.target_lost_frames / max_lost_frames）
        self.target_lost_frames = 0

        # FPS
        self._last_time = time.time()
        self._fps = 0.0

    def detect(self, frame: np.ndarray):
        """
        检测橙色物体
        返回 (bbox, center, mask, confidence) 或 (None, None, mask, 0)
        """
        h, w = frame.shape[:2]
        min_area = int(h * w * MIN_AREA_RATIO)

        blurred = cv2.GaussianBlur(frame, (GAUSSIAN_BLUR_SIZE, GAUSSIAN_BLUR_SIZE), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        lower = np.array([self.hsv["h_min"], self.hsv["s_min"], self.hsv["v_min"]])
        upper = np.array([self.hsv["h_max"], self.hsv["s_max"], self.hsv["v_max"]])
        mask = cv2.inRange(hsv, lower, upper)

        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            self._smooth_cx = None
            self._smooth_cy = None
            return None, None, mask, 0.0

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < min_area:
            self._smooth_cx = None
            self._smooth_cy = None
            return None, None, mask, 0.0

        rx, ry, rw, rh = cv2.boundingRect(largest)
        bbox = (rx, ry, rx + rw, ry + rh)

        M = cv2.moments(largest)
        if M["m00"] <= 0:
            return None, None, mask, 0.0

        raw_cx = int(M["m10"] / M["m00"])
        raw_cy = int(M["m01"] / M["m00"])

        # EMA 平滑（SafeTracker 无此功能，本脚本改进）
        if self._smooth_cx is None:
            self._smooth_cx = float(raw_cx)
            self._smooth_cy = float(raw_cy)
        else:
            self._smooth_cx = EMA_ALPHA * raw_cx + (1 - EMA_ALPHA) * self._smooth_cx
            self._smooth_cy = EMA_ALPHA * raw_cy + (1 - EMA_ALPHA) * self._smooth_cy

        center = (int(self._smooth_cx), int(self._smooth_cy))
        confidence = min(1.0, area / (h * w * 0.05))

        return bbox, center, mask, confidence

    def compute_control(self, center, frame_shape):
        """
        计算追踪控制量（完全对齐 SafeTracker.update + _smooth_stop 逻辑）
        返回 (roll_cmd, pitch_cmd, offset_x, offset_y, offset_mag, is_stable)
        """
        h, w = frame_shape[:2]

        if center is None:
            # ---- 目标丢失分支（对齐 SafeTracker） ----
            self.target_lost_frames += 1
            self.stable_count = 0

            # 渐进减速：丢失 MAX_LOST_FRAMES 帧后才开始平滑停止
            # 避免短暂遮挡导致突然停止
            if self.target_lost_frames >= MAX_LOST_FRAMES:
                self._smooth_roll *= (1 - SMOOTHING_CTRL)
                self._smooth_pitch *= (1 - SMOOTHING_CTRL)
                if abs(self._smooth_roll) < 0.5:
                    self._smooth_roll = 0
                if abs(self._smooth_pitch) < 0.5:
                    self._smooth_pitch = 0

            return int(self._smooth_roll), int(self._smooth_pitch), 0.0, 0.0, 0.0, False

        # ---- 目标检测到 ----
        self.target_lost_frames = 0

        cx, cy = center
        offset_x = (cx - w / 2) / (w / 2)
        offset_y = (cy - h / 2) / (h / 2)
        offset_mag = math.sqrt(offset_x ** 2 + offset_y ** 2)

        # 稳定性（drone_control_gui.py 渐进衰减，比 SafeTracker 硬重置更抗抖动）
        if offset_mag < STABILITY_THRESHOLD:
            self.stable_count += 1
        else:
            self.stable_count = max(0, self.stable_count - 2)

        is_stable = self.stable_count >= self.stable_required

        # 死区
        ctrl_x = 0.0 if abs(offset_x) < DEADZONE else offset_x
        ctrl_y = 0.0 if abs(offset_y) < DEADZONE else offset_y

        # P 控制（摄像头朝下：offset_x→roll, offset_y→-pitch）
        target_roll = max(-MAX_SPEED, min(MAX_SPEED, ctrl_x * KP))
        target_pitch = max(-MAX_SPEED, min(MAX_SPEED, -ctrl_y * KP))

        # 矢量限速
        mag = math.sqrt(target_roll ** 2 + target_pitch ** 2)
        if mag > MAX_SPEED:
            scale = MAX_SPEED / mag
            target_roll *= scale
            target_pitch *= scale

        # 平滑
        self._smooth_roll = self._smooth_roll * (1 - SMOOTHING_CTRL) + target_roll * SMOOTHING_CTRL
        self._smooth_pitch = self._smooth_pitch * (1 - SMOOTHING_CTRL) + target_pitch * SMOOTHING_CTRL

        return int(self._smooth_roll), int(self._smooth_pitch), offset_x, offset_y, offset_mag, is_stable

    def emergency_stop(self):
        """紧急停止（对齐 SafeTracker.emergency_stop）"""
        self._smooth_roll = 0
        self._smooth_pitch = 0
        self.stable_count = 0
        self.target_lost_frames = 0

    def update_fps(self):
        now = time.time()
        dt = now - self._last_time
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        self._last_time = now
        return self._fps


# =====================================================================
#  手动控制管理器（参考 fly_track_and_grab.py 的手动控制逻辑）
# =====================================================================

class ManualController:
    """
    手动飞行控制（优先级高于自动追踪）
    参考 fly_track_and_grab.py：手动按键覆盖自动跟踪，松开后自动衰减归零
    """

    def __init__(self, speed: int = MANUAL_SPEED, timeout: float = MANUAL_TIMEOUT):
        self.speed = speed
        self.timeout = timeout
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gaz = 0
        self.last_time = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "gaz": 0.0}

    def set_axis(self, axis: str, value: int):
        clamp = lambda v: max(-100, min(100, v))
        if axis == "roll":
            self.roll = clamp(value)
        elif axis == "pitch":
            self.pitch = clamp(value)
        elif axis == "yaw":
            self.yaw = clamp(value)
        elif axis == "gaz":
            self.gaz = clamp(value)
        self.last_time[axis] = time.time()

    def decay(self):
        """自动衰减（松开按键后 timeout 秒归零）"""
        now = time.time()
        if now - self.last_time["roll"] > self.timeout:
            self.roll = 0
        if now - self.last_time["pitch"] > self.timeout:
            self.pitch = 0
        if now - self.last_time["yaw"] > self.timeout:
            self.yaw = 0
        if now - self.last_time["gaz"] > self.timeout:
            self.gaz = 0

    def has_active_input(self) -> bool:
        now = time.time()
        return any(now - self.last_time[a] <= self.timeout for a in self.last_time)

    def reset(self):
        self.roll = self.pitch = self.yaw = self.gaz = 0


# =====================================================================
#  可视化绘制
# =====================================================================

def draw_overlay(frame, bbox, center, confidence, offset_x, offset_y, offset_mag,
                 roll_cmd, pitch_cmd, stable_count, stable_required, is_stable, fps,
                 mode_label="CALIBRATE", tracking_active=False, safe_mode=False,
                 manual_ctrl=None, final_roll=0, final_pitch=0, final_yaw=0, final_gaz=0):
    display = frame.copy()
    h, w = display.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2

    # ---- 安全模式全屏橙色遮罩（最高优先级，参考 fly_track_and_grab.py） ----
    if safe_mode:
        overlay = display.copy()
        cv2.rectangle(overlay, (0, 0), (w, h), (0, 140, 255), -1)
        display = cv2.addWeighted(overlay, 0.12, display, 0.88, 0)

        warning = "[SAFE MODE] TRACKING OFF - HOVER"
        ts = cv2.getTextSize(warning, cv2.FONT_HERSHEY_SIMPLEX, 0.9, 3)[0]
        tx = (w - ts[0]) // 2
        cv2.rectangle(display, (tx - 15, 10), (tx + ts[0] + 15, 50), (0, 100, 200), -1)
        cv2.rectangle(display, (tx - 15, 10), (tx + ts[0] + 15, 50), (0, 165, 255), 3)
        cv2.putText(display, warning, (tx, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 3)

        hint = "WASD=move  L=LAND  SPACE=resume  Q=quit"
        hs = cv2.getTextSize(hint, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)[0]
        hx = (w - hs[0]) // 2
        cv2.putText(display, hint, (hx, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        ctrl = f"roll:{final_roll:+3d}  pitch:{final_pitch:+3d}  yaw:{final_yaw:+3d}  gaz:{final_gaz:+3d}"
        cv2.putText(display, ctrl, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        return display

    # ---- 稳定区域圆圈 ----
    stable_radius = int(STABILITY_THRESHOLD * (w / 2))
    circle_color = (0, 255, 0) if (bbox is not None and offset_mag < STABILITY_THRESHOLD) else (80, 80, 80)
    cv2.circle(display, (cx_frame, cy_frame), stable_radius, circle_color, 2)

    # ---- 中心十字 ----
    cv2.line(display, (cx_frame - 30, cy_frame), (cx_frame + 30, cy_frame), (255, 255, 255), 1)
    cv2.line(display, (cx_frame, cy_frame - 30), (cx_frame, cy_frame + 30), (255, 255, 255), 1)

    # ---- 检测框 ----
    if bbox is not None and center is not None:
        x1, y1, x2, y2 = bbox
        tcx, tcy = center
        if is_stable:
            box_color = (0, 255, 0)
        elif offset_mag < STABILITY_THRESHOLD:
            box_color = (0, 255, 128)
        elif offset_mag < 0.5:
            box_color = (0, 255, 255)
        else:
            box_color = (0, 140, 255)
        cv2.rectangle(display, (x1, y1), (x2, y2), box_color, 2)
        cv2.circle(display, (tcx, tcy), 6, box_color, -1)
        cv2.line(display, (cx_frame, cy_frame), (tcx, tcy), box_color, 2)
        cv2.putText(display, f"orange  conf={confidence:.0%}", (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

    # ---- 顶部状态栏 ----
    overlay = display.copy()
    cv2.rectangle(overlay, (0, 0), (w, 100), (0, 0, 0), -1)
    display = cv2.addWeighted(overlay, 0.6, display, 0.4, 0)

    # 模式 + 追踪状态
    if tracking_active:
        mode_color = (0, 0, 255)
        mode_text = f"[{mode_label}] AUTO-TRACKING ON"
    else:
        mode_color = (255, 200, 0)
        mode_text = f"[{mode_label}] Tracking OFF"

    cv2.putText(display, mode_text, (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 2)

    # 检测状态
    if bbox is not None:
        if is_stable:
            status = "STABLE - Target centered!"
            s_color = (0, 255, 0)
        else:
            status = f"Detected  offset={offset_mag:.2f}  stable={stable_count}/{stable_required}"
            s_color = (0, 255, 255)
    else:
        status = "Searching..."
        s_color = (100, 100, 255)

    cv2.putText(display, status, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, s_color, 2)

    # 控制量（显示最终实际发送值，含手动覆盖）
    manual_tag = " [MANUAL]" if (manual_ctrl and manual_ctrl.has_active_input()) else ""
    ctrl_info = (f"send: roll={final_roll:+d} pitch={final_pitch:+d} "
                 f"yaw={final_yaw:+d} gaz={final_gaz:+d}{manual_tag} | "
                 f"auto: r={roll_cmd:+d} p={pitch_cmd:+d} | FPS={fps:.0f}")
    cv2.putText(display, ctrl_info, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)

    # 稳定度进度条
    bar_x, bar_y, bar_w, bar_h = 10, 75, 200, 12
    cv2.rectangle(display, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (60, 60, 60), -1)
    fill = int(bar_w * min(stable_count, stable_required) / stable_required) if stable_required > 0 else 0
    bar_c = (0, 255, 0) if is_stable else (0, 200, 255)
    cv2.rectangle(display, (bar_x, bar_y), (bar_x + fill, bar_y + bar_h), bar_c, -1)
    cv2.rectangle(display, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (150, 150, 150), 1)
    cv2.putText(display, f"{stable_count}/{stable_required}", (bar_x + bar_w + 5, bar_y + 11),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    # 底部提示
    hints = "SPACE=STOP  L=Land  T=Track  WASD=Move  P=Save  O=Reset  Q=Quit"
    cv2.putText(display, hints, (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

    return display


# =====================================================================
#  UI 辅助
# =====================================================================

def nothing(_):
    pass

def create_trackbars(window_name: str, hsv: dict):
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 400, 300)
    cv2.createTrackbar("H min", window_name, hsv["h_min"], 179, nothing)
    cv2.createTrackbar("H max", window_name, hsv["h_max"], 179, nothing)
    cv2.createTrackbar("S min", window_name, hsv["s_min"], 255, nothing)
    cv2.createTrackbar("S max", window_name, hsv["s_max"], 255, nothing)
    cv2.createTrackbar("V min", window_name, hsv["v_min"], 255, nothing)
    cv2.createTrackbar("V max", window_name, hsv["v_max"], 255, nothing)

def read_trackbars(window_name: str) -> dict:
    return {
        "h_min": cv2.getTrackbarPos("H min", window_name),
        "h_max": cv2.getTrackbarPos("H max", window_name),
        "s_min": cv2.getTrackbarPos("S min", window_name),
        "s_max": cv2.getTrackbarPos("S max", window_name),
        "v_min": cv2.getTrackbarPos("V min", window_name),
        "v_max": cv2.getTrackbarPos("V max", window_name),
    }

def save_params(hsv: dict, path: str = "color_tracking_params.json"):
    with open(path, "w") as f:
        json.dump(hsv, f, indent=2)
    print(f"[INFO] HSV params saved to {path}: {hsv}")

def load_params(path: str = "color_tracking_params.json") -> dict:
    p = Path(path)
    if p.exists():
        with open(p) as f:
            params = json.load(f)
        print(f"[INFO] Loaded HSV params from {path}: {params}")
        return params
    return None


# =====================================================================
#  主函数
# =====================================================================

def main():
    parser = argparse.ArgumentParser(
        description="橙色物体颜色追踪测试（完整飞控 + HSV 校准）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--mode", choices=["calibrate", "track"], default="calibrate",
                        help="calibrate=只看画面, track=完整飞控+追踪")
    parser.add_argument("--ip", default="192.168.42.1")
    parser.add_argument("--webcam", action="store_true",
                        help="用笔记本摄像头（不连无人机）")
    parser.add_argument("--source", default="0",
                        help="webcam 模式的摄像头编号/视频文件")
    parser.add_argument("--no-trackbar", action="store_true")
    parser.add_argument("--params", default="color_tracking_params.json")
    args = parser.parse_args()

    # 加载 HSV 参数
    saved = load_params(args.params)
    hsv_params = saved if saved else DEFAULT_HSV.copy()

    # 创建视频源
    if args.webcam:
        source = int(args.source) if args.source.isdigit() else args.source
        video_src = WebcamVideoSource(source)
        mode_label = "WEBCAM"
        can_fly = False
    else:
        video_src = DroneVideoSource(ip=args.ip)
        mode_label = args.mode.upper()
        can_fly = (args.mode == "track")

    if not video_src.connect():
        return

    if not args.webcam:
        print("[INFO] Waiting for video stream (3s) ...")
        time.sleep(3)

    # 窗口
    win_main = "Color Tracking Test"
    win_mask = "HSV Mask"
    cv2.namedWindow(win_main, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_mask, cv2.WINDOW_NORMAL)

    trackbar_win = "HSV Calibration"
    use_trackbar = not args.no_trackbar
    if use_trackbar:
        create_trackbars(trackbar_win, hsv_params)

    # 初始化
    tracker = ColorTracker(hsv_params)
    manual = ManualController()
    tracking_active = False
    safe_mode = False

    print()
    print("=" * 65)
    print("  Orange Color Tracking Test")
    print(f"  Mode: {mode_label}")
    print("=" * 65)
    if can_fly:
        print("  SPACE  = EMERGENCY STOP (highest priority)")
        print("  L      = Land (always available)")
        print("  1      = Takeoff (~1m), then use R/F to adjust altitude")
        print("  T      = Toggle auto-tracking on/off")
        print("  W/S    = Forward/Backward   A/D = Left/Right")
        print("  Z/E    = Yaw left/right     R/F = Up/Down (adjust altitude!)")
    print("  P      = Save HSV params    O   = Reset HSV")
    print("  Q/ESC  = Quit")
    print("=" * 65)
    if can_fly:
        print()
        print("  Tracking params (ref: SafeTracker):")
        print(f"    KP={KP}  MAX_SPEED={MAX_SPEED}  DEADZONE={DEADZONE}")
        print(f"    SMOOTHING={SMOOTHING_CTRL}  STABILITY={STABILITY_THRESHOLD}")
        print(f"    MANUAL_SPEED={MANUAL_SPEED}  TIMEOUT={MANUAL_TIMEOUT}s")
    print()

    try:
        while True:
            frame = video_src.get_frame()

            # 初始化默认值（无论有没有帧，按键和手动控制都要处理）
            bbox = center = None
            mask = None
            confidence = 0.0
            auto_roll = auto_pitch = 0
            ox = oy = om = 0.0
            stable = False
            fps = tracker.update_fps()

            if frame is not None:
                # 滑块更新
                if use_trackbar:
                    tracker.hsv = read_trackbars(trackbar_win)

                # 检测
                bbox, center, mask, confidence = tracker.detect(frame)

                # 计算自动追踪控制量
                auto_roll, auto_pitch, ox, oy, om, stable = tracker.compute_control(
                    center, frame.shape
                )

            # 手动控制衰减（无论有没有帧都要执行）
            manual.decay()

            # ======== 最终控制量决定（优先级系统） ========
            final_roll = final_pitch = final_yaw = final_gaz = 0

            if safe_mode:
                tracker.emergency_stop()
                final_roll = manual.roll
                final_pitch = manual.pitch
                final_yaw = manual.yaw
                final_gaz = manual.gaz
            elif tracking_active and can_fly:
                # 手动输入覆盖自动
                final_roll = manual.roll if manual.roll != 0 else auto_roll
                final_pitch = manual.pitch if manual.pitch != 0 else auto_pitch
                final_yaw = manual.yaw
                final_gaz = manual.gaz
            else:
                final_roll = manual.roll
                final_pitch = manual.pitch
                final_yaw = manual.yaw
                final_gaz = manual.gaz

            # 发送控制命令
            if can_fly and video_src.piloting_started:
                video_src.send_piloting(final_roll, final_pitch, final_yaw, final_gaz)

            # 绘制显示
            if frame is not None:
                display = draw_overlay(
                    frame, bbox, center, confidence,
                    ox, oy, om, auto_roll, auto_pitch,
                    tracker.stable_count, tracker.stable_required,
                    stable, fps,
                    mode_label=mode_label,
                    tracking_active=tracking_active,
                    safe_mode=safe_mode,
                    manual_ctrl=manual,
                    final_roll=final_roll, final_pitch=final_pitch,
                    final_yaw=final_yaw, final_gaz=final_gaz,
                )

                if mask is not None:
                    mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(mask_colored, contours, -1, (0, 255, 0), 2)
                    if center is not None:
                        cv2.circle(mask_colored, center, 8, (0, 0, 255), -1)
                    cv2.imshow(win_mask, mask_colored)

                cv2.imshow(win_main, display)
            else:
                # 没有帧时显示占位图（确保窗口存在以接收按键）
                placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, "Waiting for video...", (150, 170),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)
                cv2.putText(placeholder, "Press 1=Takeoff  L=Land  Q=Quit", (120, 210),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1)
                cv2.imshow(win_main, placeholder)
                time.sleep(0.02)

            # ======== 键盘事件 ========
            key = cv2.waitKey(1) & 0xFF

            # --- 最高优先级：退出 ---
            if key == ord('q') or key == 27:  # Q / ESC
                if can_fly:
                    tracker.emergency_stop()
                    video_src.hover()
                    print("[INFO] Hover sent before quit")
                break

            # --- 最高优先级：紧急停止 / 安全模式 (SPACE) ---
            elif key == ord(' '):
                if can_fly:
                    safe_mode = not safe_mode
                    if safe_mode:
                        tracking_active = False
                        tracker.emergency_stop()
                        manual.reset()
                        video_src.hover()
                        print("[SAFE MODE] ON - Auto-tracking DISABLED, hover, manual control ENABLED")
                    else:
                        print("[SAFE MODE] OFF - You can now press T to resume tracking")

            # --- 降落 (L) - 不受任何模式限制 ---
            elif key == ord('l') or key == ord('L'):
                if can_fly:
                    tracking_active = False
                    safe_mode = False
                    tracker.emergency_stop()
                    manual.reset()
                    video_src.hover()
                    time.sleep(0.3)
                    video_src.land()

            # --- 起飞 (数字键 1) ---
            elif key == ord('1'):
                if can_fly and not safe_mode:
                    video_src.takeoff()
                elif safe_mode:
                    print("[WARN] Takeoff blocked in SAFE MODE, press SPACE first")

            # --- 追踪开关 (T) ---
            elif key == ord('t') or key == ord('T'):
                if can_fly and not safe_mode:
                    tracking_active = not tracking_active
                    if tracking_active:
                        video_src.start_piloting()
                        print("[INFO] >>> AUTO-TRACKING ON <<<")
                    else:
                        tracker.emergency_stop()
                        video_src.hover()
                        print("[INFO] >>> AUTO-TRACKING OFF - hover <<<")
                elif safe_mode:
                    print("[WARN] Cannot enable tracking in SAFE MODE, press SPACE first")
                elif not can_fly:
                    print("[WARN] Tracking not available in this mode (use --mode track)")

            # --- 手动悬停 (H) ---
            elif key == ord('h') or key == ord('H'):
                if can_fly:
                    tracking_active = False
                    tracker.emergency_stop()
                    manual.reset()
                    video_src.hover()
                    print("[INFO] Manual hover, tracking off")

            # --- 手动飞行 WASD/ZE/RF（大小写均可） ---
            elif key in (ord('w'), ord('W')):
                if can_fly:
                    manual.set_axis("pitch", MANUAL_SPEED)
            elif key in (ord('s'), ord('S')):
                if can_fly:
                    manual.set_axis("pitch", -MANUAL_SPEED)
            elif key in (ord('a'), ord('A')):
                if can_fly:
                    manual.set_axis("roll", -MANUAL_SPEED)
            elif key in (ord('d'), ord('D')):
                if can_fly:
                    manual.set_axis("roll", MANUAL_SPEED)
            elif key in (ord('z'), ord('Z')):
                if can_fly:
                    manual.set_axis("yaw", -MANUAL_SPEED)
            elif key in (ord('e'), ord('E')):
                if can_fly:
                    manual.set_axis("yaw", MANUAL_SPEED)
            elif key in (ord('r'), ord('R')):
                if can_fly:
                    manual.set_axis("gaz", MANUAL_SPEED)
            elif key in (ord('f'), ord('F')):
                if can_fly:
                    manual.set_axis("gaz", -MANUAL_SPEED)

            # --- HSV 参数保存 (P) ---
            elif key == ord('p') or key == ord('P'):
                save_params(tracker.hsv.copy(), args.params)

            # --- HSV 参数重置 (O) ---
            elif key == ord('o') or key == ord('O'):
                tracker.hsv = DEFAULT_HSV.copy()
                if use_trackbar:
                    for k, v in DEFAULT_HSV.items():
                        bar_name = k.replace("_", " ").replace("h ", "H ").replace("s ", "S ").replace("v ", "V ")
                        try:
                            cv2.setTrackbarPos(bar_name, trackbar_win, v)
                        except Exception:
                            pass
                print("[INFO] HSV reset to default orange")

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C")
    finally:
        if can_fly:
            print("[INFO] Cleaning up...")
            tracker.emergency_stop()
            manual.reset()
            try:
                video_src.hover()
            except Exception:
                pass

        video_src.disconnect()
        cv2.destroyAllWindows()

        print(f"\n[INFO] Final HSV params: {tracker.hsv}")
        print("[INFO] Done.")


if __name__ == "__main__":
    main()

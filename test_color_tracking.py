#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
橙色物体颜色追踪 - 最小单元测试脚本（含无人机控制）
=====================================================
用途：连接 Parrot Anafi 无人机，用真实摄像头画面验证 HSV 颜色检测，
      并可选地实际控制无人机追踪橙色目标。

两种模式：
  calibrate  — 只连接无人机看画面 + 调 HSV 参数，不发送任何飞控命令
  track      — 在 calibrate 基础上，实际发送 roll/pitch 控制无人机追踪目标
               （需要先手动起飞，脚本不会自动起飞）

用法：
  # 模式1: 连接无人机，只校准参数（安全，不动无人机）
  python test_color_tracking.py --mode calibrate

  # 模式2: 连接无人机，实际追踪（需要先起飞！按 T 启用追踪）
  python test_color_tracking.py --mode track

  # 离线测试: 用笔记本摄像头调参（不需要无人机）
  python test_color_tracking.py --webcam
  python test_color_tracking.py --webcam --source 1

  # 其他选项
  python test_color_tracking.py --mode calibrate --no-trackbar
  python test_color_tracking.py --mode calibrate --ip 10.202.0.1
"""

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
DEFAULT_HSV = {
    "h_min": 5,   "h_max": 25,
    "s_min": 100, "s_max": 255,
    "v_min": 100, "v_max": 255,
}

# ========== 追踪参数（与 drone_control_gui.py 一致） ==========
STABILITY_THRESHOLD = 0.25   # 偏移量 < 此值算 "centered"
DEADZONE = 0.15              # 死区
KP = 5.0                     # 比例增益
MAX_SPEED = 5                # 最大控制量
SMOOTHING_CTRL = 0.05        # 控制量平滑系数
EMA_ALPHA = 0.4              # 中心点 EMA 平滑系数（0~1，越大越灵敏）
MIN_AREA_RATIO = 0.001       # 最小轮廓面积占帧面积比例
MORPH_KERNEL_SIZE = 7        # 形态学核大小
GAUSSIAN_BLUR_SIZE = 5       # 高斯模糊核大小


# =====================================================================
#  Parrot Anafi 无人机连接（参考 drone_control_gui.py）
# =====================================================================

class DroneVideoSource:
    """通过 Olympe 连接 Parrot Anafi 获取视频帧，并可发送 piloting 命令"""

    def __init__(self, ip: str = "192.168.42.1"):
        self.ip = ip
        self.drone = None
        self.connected = False
        self.piloting_started = False

        # 视频帧队列（与 drone_control_gui.py 完全一致）
        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_lock = threading.Lock()
        self.current_frame = None
        self.frame_lock = threading.Lock()
        self.streaming = False

    def connect(self) -> bool:
        """连接无人机并启动视频流"""
        try:
            import olympe
            import olympe.log
            # 降低 olympe 日志噪音
            olympe.log.update_config({
                "loggers": {
                    "olympe": {"level": "WARNING"},
                    "ulog": {"level": "ERROR"},
                }
            })
        except ImportError:
            print("[ERROR] olympe 未安装，无法连接无人机")
            print("        请使用 --webcam 模式进行离线测试")
            return False

        self._olympe = olympe  # 保存引用

        print(f"[INFO] Connecting to drone at {self.ip} ...")
        self.drone = olympe.Drone(self.ip)

        for attempt in range(3):
            try:
                if self.drone.connect():
                    self.connected = True
                    print(f"[INFO] Connected to drone: {self.ip}")

                    # 启动视频流
                    time.sleep(1)
                    self._start_streaming()
                    return True
            except Exception as e:
                print(f"[WARN] Attempt {attempt + 1}/3 failed: {e}")
                time.sleep(1)

        print("[ERROR] Failed to connect after 3 attempts")
        return False

    def disconnect(self):
        """断开连接"""
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
        """启动 piloting 接口（发送控制命令前需要先调用）"""
        if not self.piloting_started and self.drone and self.connected:
            try:
                self.drone.start_piloting()
                self.piloting_started = True
                print("[INFO] Piloting interface started")
            except Exception as e:
                print(f"[WARN] Start piloting failed: {e}")

    def send_piloting(self, roll: int, pitch: int, yaw: int = 0, gaz: int = 0):
        """发送 piloting 命令（roll/pitch/yaw/gaz）"""
        if self.drone and self.connected and self.piloting_started:
            self.drone.piloting(roll, pitch, yaw, gaz, 0.05)

    def hover(self):
        """发送悬停命令（归零）"""
        if self.drone and self.connected and self.piloting_started:
            for _ in range(5):
                self.drone.piloting(0, 0, 0, 0, 0.05)
                time.sleep(0.05)

    def get_frame(self):
        """获取当前帧（线程安全）"""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None

    # ---- 内部方法（与 drone_control_gui.py 一致） ----

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
    """笔记本摄像头视频源（离线调试用，接口与 DroneVideoSource 一致）"""

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
        print("[ERROR] Cannot read from webcam")
        return False

    def disconnect(self):
        if self.cap:
            self.cap.release()
        self.connected = False

    def start_piloting(self):
        self.piloting_started = True
        print("[INFO] Piloting (simulated) - webcam mode, no drone commands")

    def send_piloting(self, roll: int, pitch: int, yaw: int = 0, gaz: int = 0):
        pass  # 没有真正的无人机，不发送命令

    def hover(self):
        pass

    def get_frame(self):
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            return frame if ret else None
        return None


# =====================================================================
#  颜色追踪器（与之前版本一致，核心检测+控制逻辑）
# =====================================================================

class ColorTracker:
    """橙色物体颜色追踪器（完整处理流水线）"""

    def __init__(self, hsv_params: dict = None):
        self.hsv = hsv_params or DEFAULT_HSV.copy()
        self._smooth_cx = None
        self._smooth_cy = None
        self._smooth_roll = 0.0
        self._smooth_pitch = 0.0
        self.stable_count = 0
        self.stable_required = 5
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
        计算追踪控制量
        返回 (roll_cmd, pitch_cmd, offset_x, offset_y, offset_mag, is_stable)
        """
        h, w = frame_shape[:2]

        if center is None:
            self._smooth_roll *= (1 - SMOOTHING_CTRL)
            self._smooth_pitch *= (1 - SMOOTHING_CTRL)
            self.stable_count = 0
            return 0, 0, 0.0, 0.0, 0.0, False

        cx, cy = center
        offset_x = (cx - w / 2) / (w / 2)
        offset_y = (cy - h / 2) / (h / 2)
        offset_mag = math.sqrt(offset_x ** 2 + offset_y ** 2)

        if offset_mag < STABILITY_THRESHOLD:
            self.stable_count += 1
        else:
            self.stable_count = max(0, self.stable_count - 2)

        is_stable = self.stable_count >= self.stable_required

        ctrl_x = 0.0 if abs(offset_x) < DEADZONE else offset_x
        ctrl_y = 0.0 if abs(offset_y) < DEADZONE else offset_y

        target_roll = max(-MAX_SPEED, min(MAX_SPEED, ctrl_x * KP))
        target_pitch = max(-MAX_SPEED, min(MAX_SPEED, -ctrl_y * KP))

        mag = math.sqrt(target_roll ** 2 + target_pitch ** 2)
        if mag > MAX_SPEED:
            scale = MAX_SPEED / mag
            target_roll *= scale
            target_pitch *= scale

        self._smooth_roll = self._smooth_roll * (1 - SMOOTHING_CTRL) + target_roll * SMOOTHING_CTRL
        self._smooth_pitch = self._smooth_pitch * (1 - SMOOTHING_CTRL) + target_pitch * SMOOTHING_CTRL

        return int(self._smooth_roll), int(self._smooth_pitch), offset_x, offset_y, offset_mag, is_stable

    def update_fps(self):
        now = time.time()
        dt = now - self._last_time
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        self._last_time = now
        return self._fps


# =====================================================================
#  可视化绘制
# =====================================================================

def draw_overlay(frame, bbox, center, confidence, offset_x, offset_y, offset_mag,
                 roll_cmd, pitch_cmd, stable_count, stable_required, is_stable, fps,
                 mode_label="CALIBRATE", tracking_active=False):
    """在帧上绘制追踪可视化叠加层"""
    display = frame.copy()
    h, w = display.shape[:2]
    cx_frame, cy_frame = w // 2, h // 2

    # 稳定区域圆圈
    stable_radius = int(STABILITY_THRESHOLD * (w / 2))
    circle_color = (0, 255, 0) if (bbox is not None and offset_mag < STABILITY_THRESHOLD) else (80, 80, 80)
    cv2.circle(display, (cx_frame, cy_frame), stable_radius, circle_color, 2)

    # 画面中心十字
    cv2.line(display, (cx_frame - 30, cy_frame), (cx_frame + 30, cy_frame), (255, 255, 255), 1)
    cv2.line(display, (cx_frame, cy_frame - 30), (cx_frame, cy_frame + 30), (255, 255, 255), 1)

    # 检测结果
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

        label = f"orange  conf={confidence:.0%}"
        cv2.putText(display, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)

    # 顶部状态栏
    overlay = display.copy()
    cv2.rectangle(overlay, (0, 0), (w, 95), (0, 0, 0), -1)
    display = cv2.addWeighted(overlay, 0.6, display, 0.4, 0)

    # 模式标签
    if tracking_active:
        mode_color = (0, 0, 255)
        mode_text = f"[{mode_label}] TRACKING ACTIVE - Sending commands!"
    else:
        mode_color = (255, 200, 0)
        mode_text = f"[{mode_label}] View only"

    cv2.putText(display, mode_text, (10, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 2)

    if bbox is not None:
        if is_stable:
            status = "STABLE - Target centered!"
            s_color = (0, 255, 0)
        else:
            status = f"Tracking  offset={offset_mag:.2f}  stable={stable_count}/{stable_required}"
            s_color = (0, 255, 255)
    else:
        status = "Searching..."
        s_color = (100, 100, 255)

    cv2.putText(display, status, (10, 42),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, s_color, 2)

    info = f"offset: x={offset_x:+.2f} y={offset_y:+.2f} | roll={roll_cmd:+d} pitch={pitch_cmd:+d} | FPS={fps:.0f}"
    cv2.putText(display, info, (10, 62),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

    # 稳定度进度条
    bar_x, bar_y, bar_w, bar_h = 10, 72, 200, 12
    cv2.rectangle(display, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (60, 60, 60), -1)
    fill = int(bar_w * min(stable_count, stable_required) / stable_required) if stable_required > 0 else 0
    bar_c = (0, 255, 0) if is_stable else (0, 200, 255)
    cv2.rectangle(display, (bar_x, bar_y), (bar_x + fill, bar_y + bar_h), bar_c, -1)
    cv2.rectangle(display, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (150, 150, 150), 1)
    cv2.putText(display, f"{stable_count}/{stable_required}", (bar_x + bar_w + 5, bar_y + 11),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    # 底部操作提示
    hints = "[S]ave  [R]eset  [T]rack on/off  [H]over  [Q]uit"
    cv2.putText(display, hints, (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

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
    print(f"[INFO] HSV params saved to {path}")
    print(f"       {hsv}")


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
    parser = argparse.ArgumentParser(description="橙色物体颜色追踪测试（含无人机控制）")
    parser.add_argument("--mode", choices=["calibrate", "track"], default="calibrate",
                        help="calibrate: 只看画面调参, track: 实际控制无人机追踪")
    parser.add_argument("--ip", default="192.168.42.1",
                        help="无人机 IP 地址 (默认: 192.168.42.1)")
    parser.add_argument("--webcam", action="store_true",
                        help="使用笔记本摄像头（离线调试，不连接无人机）")
    parser.add_argument("--source", default="0",
                        help="webcam 模式下的摄像头编号或视频文件路径")
    parser.add_argument("--no-trackbar", action="store_true",
                        help="不显示 HSV 校准滑块")
    parser.add_argument("--params", default="color_tracking_params.json",
                        help="HSV 参数文件路径")
    args = parser.parse_args()

    # ---- 加载 HSV 参数 ----
    saved = load_params(args.params)
    hsv_params = saved if saved else DEFAULT_HSV.copy()

    # ---- 创建视频源 ----
    if args.webcam:
        source = int(args.source) if args.source.isdigit() else args.source
        video_src = WebcamVideoSource(source)
        mode_label = "WEBCAM"
        can_track = False
        print("[INFO] Webcam mode - no drone connection")
    else:
        video_src = DroneVideoSource(ip=args.ip)
        mode_label = args.mode.upper()
        can_track = (args.mode == "track")

    # ---- 连接 ----
    if not video_src.connect():
        return

    # 等待无人机视频流稳定
    if not args.webcam:
        print("[INFO] Waiting for video stream to stabilize (3s) ...")
        time.sleep(3)

    # ---- 创建窗口 ----
    win_main = "Color Tracking Test"
    win_mask = "HSV Mask"
    cv2.namedWindow(win_main, cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_mask, cv2.WINDOW_NORMAL)

    trackbar_win = "HSV Calibration"
    use_trackbar = not args.no_trackbar
    if use_trackbar:
        create_trackbars(trackbar_win, hsv_params)

    # ---- 初始化追踪器 ----
    tracker = ColorTracker(hsv_params)

    # 追踪控制开关（track 模式下按 T 切换）
    tracking_active = False

    print()
    print("=" * 55)
    print("  Orange Color Tracking Test")
    print(f"  Mode: {mode_label}")
    if can_track:
        print("  WARNING: Track mode - drone WILL move when tracking is ON!")
        print("  Make sure the drone is FLYING before enabling tracking!")
    print("=" * 55)
    print("  [S] Save HSV params        [R] Reset to default")
    print("  [T] Toggle tracking on/off [H] Send hover (stop)")
    print("  [Q] Quit")
    print("=" * 55)
    print()

    try:
        while True:
            # 获取帧
            frame = video_src.get_frame()
            if frame is None:
                time.sleep(0.01)
                continue

            # 从滑块更新 HSV 参数
            if use_trackbar:
                tracker.hsv = read_trackbars(trackbar_win)

            # 检测
            bbox, center, mask, confidence = tracker.detect(frame)

            # 计算控制量
            roll, pitch, ox, oy, om, stable = tracker.compute_control(center, frame.shape)

            # ---- 发送实际控制命令 ----
            if tracking_active and can_track and center is not None:
                video_src.send_piloting(roll, pitch)
            elif tracking_active and can_track and center is None:
                # 目标丢失 → 悬停
                video_src.send_piloting(0, 0)

            # FPS
            fps = tracker.update_fps()

            # 绘制可视化
            display = draw_overlay(
                frame, bbox, center, confidence,
                ox, oy, om, roll, pitch,
                tracker.stable_count, tracker.stable_required,
                stable, fps,
                mode_label=mode_label,
                tracking_active=tracking_active,
            )

            # Mask 可视化
            mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(mask_colored, contours, -1, (0, 255, 0), 2)
            if center is not None:
                cv2.circle(mask_colored, center, 8, (0, 0, 255), -1)

            cv2.imshow(win_main, display)
            cv2.imshow(win_mask, mask_colored)

            # ---- 键盘事件 ----
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q') or key == 27:  # Q / ESC
                if tracking_active:
                    video_src.hover()
                    print("[INFO] Hover sent before quit")
                break

            elif key == ord('s'):
                save_params(tracker.hsv.copy(), args.params)

            elif key == ord('r'):
                tracker.hsv = DEFAULT_HSV.copy()
                if use_trackbar:
                    for k, v in DEFAULT_HSV.items():
                        bar_name = k.replace("_", " ").replace("h ", "H ").replace("s ", "S ").replace("v ", "V ")
                        try:
                            cv2.setTrackbarPos(bar_name, trackbar_win, v)
                        except Exception:
                            pass
                print("[INFO] Reset to default orange HSV params")

            elif key == ord('t'):
                if can_track:
                    tracking_active = not tracking_active
                    if tracking_active:
                        video_src.start_piloting()
                        print("[INFO] >>> TRACKING ON - drone will move! <<<")
                    else:
                        video_src.hover()
                        print("[INFO] >>> TRACKING OFF - hover <<<")
                else:
                    print("[WARN] Tracking not available in this mode")
                    print("       Use --mode track to enable drone control")

            elif key == ord('h'):
                if can_track:
                    video_src.hover()
                    tracking_active = False
                    print("[INFO] Hover command sent, tracking off")

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        # 确保退出时悬停
        if can_track and tracking_active:
            print("[INFO] Sending hover before exit...")
            video_src.hover()

        video_src.disconnect()
        cv2.destroyAllWindows()

        print(f"\n[INFO] Final HSV params: {tracker.hsv}")
        print("[INFO] Done.")


if __name__ == "__main__":
    main()

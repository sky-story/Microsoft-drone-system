#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ANAFI Ai: 边飞边看 + 实时目标检测
基于 fly_watch_stream_ai_keyboard.py 集成目标检测功能

使用方法:
    python fly_watch_detection.py --drone-ip 192.168.42.1

新增功能:
    - 实时 YOLO 目标检测
    - 在视频画面上绘制检测框
    - 可过滤指定类别
"""

# ============================================================
# 0) 必须最早设置 OpenGL 平台
# ============================================================
import os
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")
import sys
import time
import argparse
import tempfile
from pathlib import Path
import queue
import threading

import cv2
import numpy as np

import olympe
import olympe.log
from olympe.video import HudType

from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
from olympe.messages import gimbal

# 终端非阻塞键盘读取
import tty
import termios
import select

# ============================================================
# 导入目标检测模块
# ============================================================
from object_detector import ObjectDetector, Detection


KEYMAP_HELP = r"""
==================== Keyboard Mapping (Terminal) ====================

[安全/退出]
  q 或 ESC      : 退出程序（可选自动降落）
  Ctrl+C        : 退出程序（可选自动降落）

[起飞/降落]
  t             : Takeoff（起飞）
  l             : Land（降落）
  space         : Brake（立即把 roll/pitch/yaw/gaz 置 0）

[飞行控制（持续按住 = 需要键盘自动重复；或连续点按）]
  w / s         : 前进 / 后退   (pitch + / -)
  a / d         : 左移 / 右移   (roll  - / +)
  z / e         : 左转 / 右转   (yaw   - / +)
  r / f         : 上升 / 下降   (gaz   + / -)

[云台（摄像头）角度控制]
  i / k         : 云台 pitch 上 / 下（速度控制）
  j / u         : 云台 yaw 左 / 右（如果支持）
  o             : 云台停止（pitch/yaw 速度置 0）

[目标检测]
  v             : 开启/关闭目标检测
  c             : 切换检测类别过滤 (全部 -> person -> car -> 全部)
=====================================================================
"""


def parse_args():
    p = argparse.ArgumentParser(
        description="ANAFI Ai: 边飞边看 + 实时目标检测"
    )

    p.add_argument("--drone-ip", default=os.environ.get("DRONE_IP", "192.168.42.1"),
                   help="无人机 IP（直连 Wi-Fi 常见：192.168.42.1）。")
    p.add_argument("--retry", type=int, default=5, help="连接重试次数。")

    p.add_argument("--hud", choices=["none", "piloting", "imaging"], default="piloting",
                   help="HUD 叠加。")

    p.add_argument("--record", action="store_true", help="是否录制 mp4。")
    p.add_argument("--outdir", default="", help="录制输出目录。")

    p.add_argument("--takeoff-on-start", action="store_true", help="启动后自动起飞。")
    p.add_argument("--wait-gps", action="store_true", help="起飞前等待 GPS fix。")
    p.add_argument("--gps-timeout", type=int, default=15, help="等待 GPS fix 的超时秒数。")

    p.add_argument("--auto-land-on-exit", action="store_true", default=True,
                   help="退出程序时若在飞行则自动降落。")
    p.add_argument("--no-auto-land-on-exit", action="store_false", dest="auto_land_on_exit")

    p.add_argument("--max-seconds", type=int, default=0, help="程序最大运行秒数。")

    p.add_argument("--step", type=int, default=25, help="每次按键改变的控制量。")
    p.add_argument("--hold-timeout", type=float, default=0.25, help="按键多久没重复就自动回 0。")
    p.add_argument("--send-hz", type=float, default=20.0, help="发送 piloting 指令频率。")

    p.add_argument("--gimbal-speed", type=float, default=0.15, help="云台速度控制的速度值。")

    p.add_argument("--loglevel", choices=["ERROR", "WARNING", "INFO", "DEBUG"], default="INFO",
                   help="Olympe 日志级别。")

    p.add_argument("--window-name", default="ANAFI Ai + Detection", help="OpenCV 窗口名。")
    p.add_argument("--no-opencv", action="store_true", help="不弹 OpenCV 窗口。")

    p.add_argument("--enable-renderer", action="store_true", help="启用 PdrawRenderer。")

    # ============== 目标检测相关参数 ==============
    p.add_argument("--enable-detection", action="store_true", default=True,
                   help="启用目标检测（默认开启）。")
    p.add_argument("--no-detection", action="store_false", dest="enable_detection",
                   help="禁用目标检测。")
    
    p.add_argument("--model", default="yolov8n.pt",
                   help="YOLO 模型路径 (默认 yolov8n.pt，最小最快)。")
    p.add_argument("--conf", type=float, default=0.5,
                   help="检测置信度阈值 (0-1)。")
    p.add_argument("--classes", nargs="+", type=str, default=None,
                   help="只检测指定类别名称（如 --classes person car dog）。")
    p.add_argument("--detect-classes", nargs="*", type=int, default=None,
                   help="只检测指定类别 ID（如 --detect-classes 0 2，优先级低于 --classes）。")
    p.add_argument("--detect-interval", type=int, default=1,
                   help="每隔几帧进行一次检测（1=每帧都检测，2=隔一帧检测）。")

    return p.parse_args()


def hud_type_from_arg(hud: str) -> HudType:
    if hud == "piloting":
        return HudType.PILOTING
    if hud == "imaging":
        return HudType.IMAGING
    return HudType.NONE


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


class FlyWatchDetection:
    """带目标检测的无人机控制类"""
    
    # 预设的类别过滤选项（按 c 键循环切换）
    CLASS_FILTERS = [
        None,           # 全部
        [0],            # person
        [2],            # car
        [0, 2],         # person + car
        [14, 15, 16],   # bird, cat, dog
    ]
    CLASS_FILTER_NAMES = [
        "ALL classes",
        "person only",
        "car only", 
        "person + car",
        "bird/cat/dog",
    ]
    
    def __init__(self, args):
        self.args = args
        olympe.log.update_config({"loggers": {"olympe": {"level": args.loglevel}}})

        self.drone = olympe.Drone(args.drone_ip)

        self.renderer = None
        self.PdrawRenderer = None
        self.running = False
        self.outdir = None

        # piloting axes
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.gaz = 0

        now = time.time()
        self.last_axis_update = {"roll": now, "pitch": now, "yaw": now, "gaz": now}

        self.gimbal_pitch_speed = 0.0
        self.gimbal_yaw_speed = 0.0

        # video frame queue
        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_queue_lock = threading.Lock()

        # ============== 目标检测相关 ==============
        self.detector = None
        self.detection_enabled = args.enable_detection
        self.current_filter_idx = 0  # 当前类别过滤索引
        self.frame_count = 0
        self.last_detections = []    # 保存最近一次的检测结果
        self.fps_history = []        # FPS 历史
        
        if self.detection_enabled:
            self._init_detector()

    def _init_detector(self):
        """初始化目标检测器"""
        print("[INFO] Initializing object detector...")
        try:
            self.detector = ObjectDetector(
                model_path=self.args.model,
                conf_threshold=self.args.conf,
                classes=self.args.detect_classes,
                class_names_filter=self.args.classes,  # 支持类别名称过滤
                verbose=True
            )
            print("[OK] Object detector initialized")
        except Exception as e:
            print(f"[ERROR] Detector init failed: {e}")
            print("[WARN] Detection will be disabled")
            self.detection_enabled = False
            self.detector = None

    # --------- helpers ---------
    def _state_name(self) -> str:
        st = self.drone.get_state(FlyingStateChanged).get("state")
        return getattr(st, "name", str(st))

    # ----------------- 连接 -----------------
    def connect(self):
        print(f"[1/7] Connecting to ANAFI Ai ({self.args.drone_ip}) ...")
        last_err = None
        for i in range(1, self.args.retry + 1):
            try:
                ok = self.drone.connect()
                if ok:
                    print("[OK] Connected.")
                    return
            except Exception as e:
                last_err = e
            print(f"[WARN] connect failed ({i}/{self.args.retry}), retrying ...")
            time.sleep(1.0)

        raise RuntimeError(f"Connect failed after retries. last_err={last_err}")

    def disconnect(self):
        print("[7/7] Disconnecting ...")
        try:
            self.drone.disconnect()
        except Exception as e:
            print(f"[WARN] disconnect error: {e}")
        print("[OK] Disconnected.")

    # ----------------- raw_cb -----------------
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
        with self.flush_queue_lock:
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait().unref()
                except Exception:
                    pass
        return True

    def _display_latest_frame_opencv(self):
        if self.args.no_opencv:
            return

        # 只显示最新一帧
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
            return

        try:
            fmt = last.format()
            cv2_cvt_color_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }.get(fmt, None)

            if cv2_cvt_color_flag is None:
                return

            yuv = last.as_ndarray()
            bgr = cv2.cvtColor(yuv, cv2_cvt_color_flag)
            
            # ============== 目标检测 ==============
            self.frame_count += 1
            detect_this_frame = (
                self.detection_enabled 
                and self.detector is not None
                and (self.frame_count % self.args.detect_interval == 0)
            )
            
            start_time = time.perf_counter()
            
            if detect_this_frame:
                # 运行检测
                self.last_detections = self.detector.detect(bgr)
            
            # 绘制检测结果（使用最近一次的检测结果）
            if self.detection_enabled and self.last_detections:
                bgr = self.detector.draw_results(bgr, self.last_detections)
            
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            
            # 计算并显示 FPS
            if detect_this_frame and elapsed_ms > 0:
                fps = 1000.0 / elapsed_ms
                self.fps_history.append(fps)
                if len(self.fps_history) > 30:
                    self.fps_history.pop(0)
            
            # 绘制信息覆盖层
            bgr = self._draw_info_overlay(bgr)
            
            cv2.imshow(self.args.window_name, bgr)
            cv2.waitKey(1)
            
        finally:
            try:
                last.unref()
            except Exception:
                pass

    def _draw_info_overlay(self, frame: np.ndarray) -> np.ndarray:
        """绘制信息覆盖层"""
        h, w = frame.shape[:2]
        
        # 背景半透明条
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)
        
        # 检测状态 - 显示实际过滤的类别
        if self.detection_enabled and self.detector:
            if self.detector.class_names_filter:
                filter_str = ", ".join(self.detector.class_names_filter)
            elif self.detector.filter_classes:
                # 如果只有 ID，转换为名称
                names = [self.detector.class_names.get(c, str(c)) for c in self.detector.filter_classes]
                filter_str = ", ".join(names)
            else:
                filter_str = "ALL"
            status = f"Detection: ON | {filter_str}"
            color = (0, 255, 0)
        elif self.detection_enabled:
            status = "Detection: ON | ALL"
            color = (0, 255, 0)
        else:
            status = "Detection: OFF (press 'v' to enable)"
            color = (0, 0, 255)
        
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # FPS 和检测数量
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            fps_text = f"Detection FPS: {avg_fps:.1f}"
        else:
            fps_text = "Detection FPS: --"
        
        det_count = len(self.last_detections) if self.detection_enabled else 0
        info_text = f"{fps_text} | Objects: {det_count}"
        cv2.putText(frame, info_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 飞行状态
        try:
            state = self._state_name()
            state_text = f"Drone: {state} | R:{self.roll} P:{self.pitch} Y:{self.yaw} G:{self.gaz}"
            cv2.putText(frame, state_text, (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        except Exception:
            pass
        
        return frame

    # ----------------- 视频流 + 录制 -----------------
    def start_streaming(self):
        print("[2/7] Starting streaming ...")

        if self.args.outdir:
            self.outdir = Path(self.args.outdir).expanduser().resolve()
            self.outdir.mkdir(parents=True, exist_ok=True)
        else:
            self.outdir = Path(tempfile.mkdtemp(prefix="olympe_ai_stream_")).resolve()

        if self.args.record:
            video_path = self.outdir / "streaming.mp4"
            meta_path = self.outdir / "streaming_metadata.json"
            self.drone.streaming.set_output_files(video=str(video_path), metadata=str(meta_path))
            print(f"[INFO] Recording enabled:\n  - {video_path}\n  - {meta_path}")
        else:
            print("[INFO] Recording disabled.")
        print(f"[INFO] Output dir: {self.outdir}")

        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=None,
            start_cb=None,
            end_cb=None,
            flush_raw_cb=self.flush_cb,
        )

        self.drone.streaming.start()
        print("[OK] Streaming started.")

        if self.args.enable_renderer:
            os.environ.setdefault("PYOPENGL_PLATFORM", "glx")
            from olympe.video.renderer import PdrawRenderer
            self.PdrawRenderer = PdrawRenderer

            hud_type = hud_type_from_arg(self.args.hud)
            self.renderer = self.PdrawRenderer(pdraw=self.drone.streaming, hud_type=hud_type)
            print(f"[OK] Renderer started (HUD={self.args.hud}).")
        else:
            print("[INFO] Renderer disabled (using OpenCV window only).")

        self.running = True

    def stop_streaming(self):
        if not self.running:
            return
        print("[6/7] Stopping streaming ...")
        try:
            if self.renderer is not None:
                self.renderer.stop()
                self.renderer = None
        except Exception as e:
            print(f"[WARN] renderer.stop error: {e}")

        try:
            self.drone.streaming.stop()
        except Exception as e:
            print(f"[WARN] streaming.stop error: {e}")

        if not self.args.no_opencv:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

        self.flush_cb({})
        self.running = False
        print("[OK] Streaming stopped.")

    # ----------------- 起飞/降落 -----------------
    def wait_gps_fix_if_needed(self):
        if not self.args.wait_gps:
            return
        print(f"[INFO] Waiting for GPS fix (timeout={self.args.gps_timeout}s) ...")
        try:
            self.drone(GPSFixStateChanged(fixed=1, _timeout=self.args.gps_timeout)).wait()
        except Exception as e:
            print(f"[WARN] GPS wait error (ignored): {e}")

    def takeoff(self):
        state = self._state_name()
        print(f"[INFO] Current state: {state}")
        if state == "hovering":
            print("[INFO] Already hovering.")
            return

        self.wait_gps_fix_if_needed()

        print("[INFO] TakeOff ...")
        ok = self.drone(
            TakeOff()
            >> FlyingStateChanged(state="hovering", _timeout=20)
        ).wait().success()

        if not ok:
            raise RuntimeError("Takeoff failed: not hovering after timeout")
        print("[OK] Hovering.")

    def land_if_flying(self):
        state = self._state_name()
        if state in ("hovering", "flying", "takingoff", "motor_ramping"):
            print("[INFO] Landing ...")
            ok = self.drone(
                Landing()
                >> FlyingStateChanged(state="landed", _timeout=30)
            ).wait().success()
            print(f"[OK] Landed={ok}")
        else:
            print(f"[INFO] Not flying (state={state}), skip landing.")

    # ----------------- manual piloting + 云台 -----------------
    def start_piloting(self):
        self.drone.start_piloting()

    def stop_piloting(self):
        try:
            self.drone.stop_piloting()
        except Exception:
            pass

    def send_piloting(self):
        self.drone.piloting(self.roll, self.pitch, self.yaw, self.gaz, 1.0 / self.args.send_hz)

    def gimbal_set_velocity(self, pitch_speed: float, yaw_speed: float):
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
            ).wait(_timeout=2)
        except Exception as e:
            print(f"[WARN] gimbal control failed (ignored): {e}")

    # ----------------- 键盘处理 -----------------
    def _clamp(self, v: int) -> int:
        return max(-100, min(100, int(v)))

    def _bump_axis(self, axis: str, delta: int):
        if axis == "roll":
            self.roll = self._clamp(self.roll + delta)
        elif axis == "pitch":
            self.pitch = self._clamp(self.pitch + delta)
        elif axis == "yaw":
            self.yaw = self._clamp(self.yaw + delta)
        elif axis == "gaz":
            self.gaz = self._clamp(self.gaz + delta)
        self.last_axis_update[axis] = time.time()

    def _maybe_decay_axes(self):
        now = time.time()
        ht = float(self.args.hold_timeout)

        if now - self.last_axis_update["roll"] > ht:
            self.roll = 0
        if now - self.last_axis_update["pitch"] > ht:
            self.pitch = 0
        if now - self.last_axis_update["yaw"] > ht:
            self.yaw = 0
        if now - self.last_axis_update["gaz"] > ht:
            self.gaz = 0

    def _toggle_detection(self):
        """切换目标检测开关"""
        if self.detector is None:
            print("[WARN] Detector not initialized")
            return
        
        self.detection_enabled = not self.detection_enabled
        status = "ON" if self.detection_enabled else "OFF"
        print(f"[INFO] Detection: {status}")
        
        if not self.detection_enabled:
            self.last_detections = []

    def _cycle_class_filter(self):
        """循环切换类别过滤"""
        if not self.detection_enabled or self.detector is None:
            return
        
        self.current_filter_idx = (self.current_filter_idx + 1) % len(self.CLASS_FILTERS)
        filter_classes = self.CLASS_FILTERS[self.current_filter_idx]
        
        # 更新检测器的类别过滤
        self.detector.filter_classes = filter_classes
        self.detector.class_names_filter = None  # 清除名称过滤，使用 ID 过滤
        
        print(f"[INFO] Filter: {self.CLASS_FILTER_NAMES[self.current_filter_idx]}")
        
        # 清空之前的检测结果
        self.last_detections = []

    def handle_key(self, ch: str):
        """返回 True 表示请求退出"""
        step = int(self.args.step)

        if ch == "\x1b":  # ESC
            return True
        if ch in ("q", "Q"):
            return True

        if ch in ("t", "T"):
            self.takeoff()
            return False
        if ch in ("l", "L"):
            self.land_if_flying()
            return False
        if ch == " ":
            self.roll = self.pitch = self.yaw = self.gaz = 0
            now = time.time()
            for k in self.last_axis_update:
                self.last_axis_update[k] = now
            print("[INFO] Brake: axes -> 0")
            return False

        # axes
        if ch in ("w", "W"):
            self._bump_axis("pitch", +step)
        elif ch in ("s", "S"):
            self._bump_axis("pitch", -step)
        elif ch in ("a", "A"):
            self._bump_axis("roll", -step)
        elif ch in ("d", "D"):
            self._bump_axis("roll", +step)
        elif ch in ("z", "Z"):
            self._bump_axis("yaw", -step)
        elif ch in ("e", "E"):
            self._bump_axis("yaw", +step)
        elif ch in ("r", "R"):
            self._bump_axis("gaz", +step)
        elif ch in ("f", "F"):
            self._bump_axis("gaz", -step)

        # gimbal
        gs = float(self.args.gimbal_speed)
        if ch in ("i", "I"):
            self.gimbal_pitch_speed = +gs
            self.gimbal_set_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ("k", "K"):
            self.gimbal_pitch_speed = -gs
            self.gimbal_set_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ("j", "J"):
            self.gimbal_yaw_speed = -gs
            self.gimbal_set_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ("u", "U"):
            self.gimbal_yaw_speed = +gs
            self.gimbal_set_velocity(self.gimbal_pitch_speed, self.gimbal_yaw_speed)
        elif ch in ("o", "O"):
            self.gimbal_pitch_speed = 0.0
            self.gimbal_yaw_speed = 0.0
            self.gimbal_set_velocity(0.0, 0.0)

        # ============== 目标检测控制 ==============
        elif ch in ("v", "V"):
            self._toggle_detection()
        elif ch in ("c", "C"):
            self._cycle_class_filter()

        return False

    # ----------------- 主循环 -----------------
    def run(self):
        print(KEYMAP_HELP)
        print("[INFO] 默认无限运行直到你按 q/ESC 或 Ctrl+C")

        self.start_piloting()

        if self.args.takeoff_on_start:
            self.takeoff()

        start_time = time.time()
        period = 1.0 / float(self.args.send_hz)
        next_send = time.time()

        with TerminalKeyReader() as kr:
            while True:
                # 1) 键盘
                ch = kr.read_key_nonblock()
                if ch is not None:
                    try:
                        if self.handle_key(ch):
                            print("[INFO] Quit requested.")
                            break
                    except Exception as e:
                        print(f"[WARN] key handling error: {e}")

                # 2) 运行时长限制
                if self.args.max_seconds and self.args.max_seconds > 0:
                    if time.time() - start_time >= self.args.max_seconds:
                        print(f"[INFO] Reached max-seconds={self.args.max_seconds}.")
                        break

                # 3) 自动回零
                self._maybe_decay_axes()

                # 4) 发送控制
                now = time.time()
                if now >= next_send:
                    try:
                        self.send_piloting()
                    except Exception as e:
                        print(f"[WARN] piloting send failed (ignored): {e}")
                    next_send = now + period

                # 5) 显示视频（带目标检测）
                try:
                    self._display_latest_frame_opencv()
                except Exception as e:
                    print(f"[WARN] opencv display error (ignored): {e}")

                time.sleep(0.005)


def main():
    args = parse_args()
    app = FlyWatchDetection(args)

    try:
        app.connect()
        app.start_streaming()
        print("[DBG] gimbal states:", app.drone.get_state(gimbal.attitude))

        app.run()

    except KeyboardInterrupt:
        print("\n[CTRL+C] Interrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        try:
            app.stop_piloting()
        except Exception:
            pass

        if args.auto_land_on_exit:
            try:
                app.land_if_flying()
            except Exception as e:
                print(f"[WARN] auto land failed: {e}")

        try:
            app.stop_streaming()
        except Exception as e:
            print(f"[WARN] stop_streaming error: {e}")

        try:
            app.disconnect()
        except Exception as e:
            print(f"[WARN] disconnect error: {e}")

        if args.record and app.outdir:
            print(f"[DONE] Recording saved under: {app.outdir}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Olympe (ANAFI Ai): 直连 Wi-Fi 边飞边看
- OpenCV 实时显示（raw_cb: olympe.VideoFrame -> as_ndarray + cvtColor）
- 可选 PdrawRenderer（默认关闭，避免 H264/AVCC not supported 影响）
- HUD/录制/终端键盘控制/起飞降落/自动回零
"""

# ============================================================
# 0) 必须最早设置 OpenGL 平台（必须在 import OpenGL / PdrawRenderer 之前）
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
  z / e         : 左转 / 右转   (yaw   - / +)   注意：q 同时是退出键，左转用 z
  r / f         : 上升 / 下降   (gaz   + / -)

[云台（摄像头）角度控制]
  i / k         : 云台 pitch 上 / 下（速度控制）
  j / u         : 云台 yaw 左 / 右（如果支持）
  o             : 云台停止（pitch/yaw 速度置 0）
=====================================================================
"""


def parse_args():
    p = argparse.ArgumentParser(
        description="ANAFI Ai: 直连 Wi-Fi 边飞边看（OpenCV raw_cb + 可选 PdrawRenderer + 录制 + 键盘控制）"
    )

    p.add_argument("--drone-ip", default=os.environ.get("DRONE_IP", "192.168.42.1"),
                   help="无人机 IP（直连 Wi-Fi 常见：192.168.42.1）。也可用环境变量 DRONE_IP。")
    p.add_argument("--retry", type=int, default=5, help="连接重试次数（默认 5）。")

    p.add_argument("--hud", choices=["none", "piloting", "imaging"], default="piloting",
                   help="HUD 叠加（仅对 PdrawRenderer 有意义）：piloting/imaging/none。")

    p.add_argument("--record", action="store_true", help="是否录制 mp4 + metadata.json（由 drone.streaming 输出）。")
    p.add_argument("--outdir", default="", help="录制输出目录；不填则创建临时目录。")

    p.add_argument("--takeoff-on-start", action="store_true", help="启动后自动起飞。")
    p.add_argument("--wait-gps", action="store_true",
                   help="起飞前等待 GPS fix（室内可能等不到；默认不等）。")
    p.add_argument("--gps-timeout", type=int, default=15, help="等待 GPS fix 的超时秒数（默认 15）。")

    # 默认开启自动降落
    p.add_argument("--auto-land-on-exit", action="store_true", default=True,
                   help="退出程序时若在飞行则自动降落（默认开启）。")
    p.add_argument("--no-auto-land-on-exit", action="store_false", dest="auto_land_on_exit",
                   help="退出程序时不自动降落（不推荐）。")

    p.add_argument("--max-seconds", type=int, default=0,
                   help="程序最大运行秒数；0 表示无限（默认）。")

    p.add_argument("--step", type=int, default=25,
                   help="每次按键改变的控制量（roll/pitch/yaw/gaz，范围 1-100，默认 25）。")
    p.add_argument("--hold-timeout", type=float, default=0.25,
                   help="按键多久没重复就把对应轴自动回 0（秒，默认 0.25）。")
    p.add_argument("--send-hz", type=float, default=20.0,
                   help="发送 piloting 指令频率（Hz，默认 20）。")

    p.add_argument("--gimbal-speed", type=float, default=0.15,
                   help="云台速度控制的速度值（默认 0.15）。")

    p.add_argument("--loglevel", choices=["ERROR", "WARNING", "INFO", "DEBUG"], default="INFO",
                   help="Olympe 日志级别：默认 INFO。")

    # 视频显示相关
    p.add_argument("--window-name", default="ANAFI Ai (raw_cb via OpenCV)", help="OpenCV 窗口名。")
    p.add_argument("--no-opencv", action="store_true", help="不弹 OpenCV 窗口（仅跑控制/录制）。")

    # 可选启用 PdrawRenderer（默认关闭）
    p.add_argument("--enable-renderer", action="store_true",
                   help="启用 PdrawRenderer（需要 OpenGL；若你遇到 H264/AVCC not supported，建议别开）。")

    return p.parse_args()


def hud_type_from_arg(hud: str) -> HudType:
    if hud == "piloting":
        return HudType.PILOTING
    if hud == "imaging":
        return HudType.IMAGING
    return HudType.NONE


class TerminalKeyReader:
    """把终端切换到 raw/cbreak mode，支持非阻塞读取单字符。"""
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


class FlyWatchAI:
    def __init__(self, args):
        self.args = args
        olympe.log.update_config({"loggers": {"olympe": {"level": args.loglevel}}})

        self.drone = olympe.Drone(args.drone_ip)

        # PdrawRenderer（可选）
        self.renderer = None
        self.PdrawRenderer = None  # 延迟 import
        self.running = False

        # output dir
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

        # video frame queue (raw_cb)
        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_queue_lock = threading.Lock()

    # --------- helpers ---------
    def _state_name(self) -> str:
        """FlyingStateChanged 的 state 是枚举：统一转成 'landed'/'hovering' 等字符串。"""
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

        raise RuntimeError(
            f"Connect failed after retries. last_err={last_err}\n"
            f"请确认：已连接无人机 Wi-Fi、IP 正确、Ai 已开启 Direct Connection。"
        )

    def disconnect(self):
        print("[7/7] Disconnecting ...")
        try:
            self.drone.disconnect()
        except Exception as e:
            print(f"[WARN] disconnect error: {e}")
        print("[OK] Disconnected.")

    # ----------------- raw_cb（关键修复点） -----------------
    def yuv_frame_cb(self, yuv_frame):
        """
        raw_cb: 每帧都会回调，参数类型是 olympe.VideoFrame
        ✅ 正确取像素：yuv_frame.as_ndarray()
        ❌ 不要用：yuv_frame.yuv_frame()
        """
        try:
            yuv_frame.ref()
            try:
                self.frame_queue.put_nowait(yuv_frame)
            except queue.Full:
                # 丢掉最旧的一帧
                old = self.frame_queue.get_nowait()
                old.unref()
                self.frame_queue.put_nowait(yuv_frame)
        except Exception:
            # 这里不要让异常冒泡到 olympe 线程
            try:
                yuv_frame.unref()
            except Exception:
                pass

    def flush_cb(self, stream):
        # 参考官方/论坛常见写法：遇到 flush 时把队列里未处理帧 unref 掉
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

        # 只显示最新一帧：把队列清空，只留最后取到的 frame
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
            # yuv_frame.format() -> 选择 OpenCV 转换标志
            fmt = last.format()
            cv2_cvt_color_flag = {
                olympe.VDEF_I420: cv2.COLOR_YUV2BGR_I420,
                olympe.VDEF_NV12: cv2.COLOR_YUV2BGR_NV12,
            }.get(fmt, None)

            if cv2_cvt_color_flag is None:
                # 不支持的 raw 格式就不显示（但别崩）
                return

            yuv = last.as_ndarray()             # 2D array: (3*h/2, w) for I420/NV12
            bgr = cv2.cvtColor(yuv, cv2_cvt_color_flag)
            cv2.imshow(self.args.window_name, bgr)
            cv2.waitKey(1)
        finally:
            try:
                last.unref()
            except Exception:
                pass

    # ----------------- 视频流 + 录制 +（可选 Renderer） -----------------
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

        # ✅ 设置回调：raw_cb + flush_raw_cb
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=None,
            start_cb=None,
            end_cb=None,
            flush_raw_cb=self.flush_cb,
        )

        self.drone.streaming.start()
        print("[OK] Streaming started.")

        # 可选 PdrawRenderer（默认关）
        if self.args.enable_renderer:
            # 必须最早设置 OpenGL 平台（在 import PdrawRenderer 前）
            os.environ.setdefault("PYOPENGL_PLATFORM", "glx")
            from olympe.video.renderer import PdrawRenderer  # 延迟 import
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

        # 清理 OpenCV
        if not self.args.no_opencv:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass

        # flush queue
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
                    roll_frame_of_reference="relative",
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

                # 5) 显示视频（OpenCV）
                try:
                    self._display_latest_frame_opencv()
                except Exception as e:
                    # 不要让显示错误影响主循环
                    print(f"[WARN] opencv display error (ignored): {e}")

                time.sleep(0.005)


def main():
    args = parse_args()
    app = FlyWatchAI(args)

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


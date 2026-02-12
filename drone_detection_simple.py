#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简单测试脚本：连接无人机 + 显示画面 + 目标检测

功能：
  - 连接 ANAFI Ai 无人机
  - 获取摄像头实时画面
  - 运行 YOLO 目标检测
  - 显示检测结果

用途：
  验证无人机画面能否正常进行目标检测，
  无需飞行控制功能，只做视觉验证。

使用方法：
  python drone_detection_simple.py
  python drone_detection_simple.py --drone-ip 192.168.42.1
  python drone_detection_simple.py --classes person car
"""

import os
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import sys
import time
import argparse
import queue
import threading

import cv2
import numpy as np

import olympe
import olympe.log

# 导入目标检测模块
from object_detector import ObjectDetector


def parse_args():
    p = argparse.ArgumentParser(
        description="简单测试：无人机画面 + 目标检测"
    )
    p.add_argument("--drone-ip", default=os.environ.get("DRONE_IP", "192.168.42.1"),
                   help="无人机 IP（默认 192.168.42.1）")
    p.add_argument("--model", default="yolov8n.pt",
                   help="YOLO 模型（默认 yolov8n.pt）")
    p.add_argument("--conf", type=float, default=0.5,
                   help="置信度阈值（默认 0.5）")
    p.add_argument("--classes", nargs="+", type=str, default=None,
                   help="只检测指定类别，如: --classes person car dog")
    p.add_argument("--no-detection", action="store_true",
                   help="禁用检测（只显示画面）")
    p.add_argument("--loglevel", default="WARNING",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                   help="日志级别（默认 WARNING）")
    return p.parse_args()


class SimpleDroneDetection:
    """简单的无人机画面检测类"""
    
    def __init__(self, args):
        self.args = args
        olympe.log.update_config({"loggers": {"olympe": {"level": args.loglevel}}})
        
        self.drone = olympe.Drone(args.drone_ip)
        self.detector = None
        self.running = False
        
        # 视频帧队列
        self.frame_queue = queue.Queue(maxsize=5)
        self.flush_lock = threading.Lock()
        
        # 统计
        self.fps_list = []
        self.frame_count = 0
    
    def connect(self):
        """连接无人机"""
        print(f"[1/3] 连接无人机 {self.args.drone_ip} ...")
        
        for i in range(3):
            try:
                if self.drone.connect():
                    print("[OK] 连接成功！")
                    return True
            except Exception as e:
                print(f"[WARN] 连接失败 ({i+1}/3): {e}")
                time.sleep(1)
        
        print("[ERROR] 无法连接无人机")
        print("请检查：")
        print("  1. 已连接无人机 WiFi")
        print("  2. IP 地址正确（尝试 ping 192.168.42.1）")
        print("  3. 无人机已开机")
        return False
    
    def init_detector(self):
        """初始化目标检测器"""
        if self.args.no_detection:
            print("[INFO] 检测已禁用（只显示画面）")
            return
        
        print("[2/3] 初始化目标检测器 ...")
        try:
            self.detector = ObjectDetector(
                model_path=self.args.model,
                conf_threshold=self.args.conf,
                class_names_filter=self.args.classes,
                verbose=True
            )
            print("[OK] 检测器初始化完成")
        except Exception as e:
            print(f"[ERROR] 检测器初始化失败: {e}")
            self.detector = None
    
    # ---------- 视频回调 ----------
    def yuv_frame_cb(self, yuv_frame):
        """视频帧回调"""
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
        """清空帧队列"""
        with self.flush_lock:
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait().unref()
                except Exception:
                    pass
        return True
    
    def start_streaming(self):
        """启动视频流"""
        print("[3/3] 启动视频流 ...")
        
        self.drone.streaming.set_callbacks(
            raw_cb=self.yuv_frame_cb,
            flush_raw_cb=self.flush_cb,
        )
        
        self.drone.streaming.start()
        self.running = True
        print("[OK] 视频流已启动")
    
    def stop_streaming(self):
        """停止视频流"""
        if self.running:
            print("[INFO] 停止视频流 ...")
            try:
                self.drone.streaming.stop()
            except Exception:
                pass
            self.flush_cb(None)
            self.running = False
    
    def disconnect(self):
        """断开连接"""
        print("[INFO] 断开连接 ...")
        try:
            self.drone.disconnect()
        except Exception:
            pass
        print("[OK] 已断开")
    
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
    
    def run(self):
        """主循环"""
        print()
        print("=" * 50)
        print("无人机画面 + 目标检测测试")
        print("=" * 50)
        print()
        print("按键说明：")
        print("  q     - 退出")
        print("  d     - 开启/关闭检测")
        print("  c     - 切换类别过滤")
        print("  s     - 截图")
        print()
        print("-" * 50)
        
        detection_enabled = self.detector is not None
        screenshot_count = 0
        
        while True:
            # 获取帧
            frame = self.get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            
            self.frame_count += 1
            
            # 目标检测
            start = time.perf_counter()
            detections = []
            
            if detection_enabled and self.detector:
                detections = self.detector.detect(frame)
                frame = self.detector.draw_results(frame, detections)
            
            elapsed_ms = (time.perf_counter() - start) * 1000
            if elapsed_ms > 0:
                self.fps_list.append(1000.0 / elapsed_ms)
                if len(self.fps_list) > 30:
                    self.fps_list.pop(0)
            
            # 绘制信息
            frame = self._draw_info(frame, detections, detection_enabled)
            
            # 显示
            cv2.imshow("Drone Detection Test (q=quit)", frame)
            
            # 键盘处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("\n[INFO] 退出...")
                break
            elif key == ord('d'):
                if self.detector:
                    detection_enabled = not detection_enabled
                    print(f"[INFO] 检测: {'开启' if detection_enabled else '关闭'}")
            elif key == ord('c'):
                if self.detector and detection_enabled:
                    if self.detector.filter_classes:
                        self.detector.clear_filter()
                    else:
                        self.detector.set_filter_by_names(["person", "car"])
            elif key == ord('s'):
                screenshot_count += 1
                filename = f"drone_screenshot_{screenshot_count}.jpg"
                cv2.imwrite(filename, frame)
                print(f"[INFO] 截图: {filename}")
        
        cv2.destroyAllWindows()
        
        # 打印统计
        if self.fps_list:
            avg_fps = sum(self.fps_list) / len(self.fps_list)
            print(f"\n平均检测 FPS: {avg_fps:.1f}")
        print(f"总帧数: {self.frame_count}")
    
    def _draw_info(self, frame, detections, detection_enabled):
        """绘制信息覆盖层"""
        h, w = frame.shape[:2]
        
        # 半透明背景
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 80), (0, 0, 0), -1)
        frame = cv2.addWeighted(overlay, 0.6, frame, 0.4, 0)
        
        # 状态 (使用英文，OpenCV 不支持中文)
        if detection_enabled and self.detector:
            if self.detector.class_names_filter:
                status = f"Detect: {', '.join(self.detector.class_names_filter)}"
            else:
                status = "Detect: ALL classes"
            color = (0, 255, 0)
        else:
            status = "Detect: OFF (press 'd' to enable)"
            color = (0, 165, 255)
        
        cv2.putText(frame, status, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # FPS 和检测数
        if self.fps_list:
            avg_fps = sum(self.fps_list) / len(self.fps_list)
            fps_text = f"FPS: {avg_fps:.1f}"
        else:
            fps_text = "FPS: --"
        
        info = f"{fps_text} | Objects: {len(detections)}"
        cv2.putText(frame, info, (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # 帧计数
        cv2.putText(frame, f"Frame: {self.frame_count}", (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        
        return frame


def main():
    args = parse_args()
    app = SimpleDroneDetection(args)
    
    try:
        # 1. 连接
        if not app.connect():
            return 1
        
        # 2. 初始化检测器
        app.init_detector()
        
        # 3. 启动视频流
        app.start_streaming()
        
        # 4. 等待一下让视频稳定
        print("[INFO] 等待视频流稳定...")
        time.sleep(1.0)
        
        # 5. 运行主循环
        app.run()
        
    except KeyboardInterrupt:
        print("\n[CTRL+C] 中断")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        app.stop_streaming()
        app.disconnect()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

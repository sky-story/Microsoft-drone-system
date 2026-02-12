#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
目标检测模块 - 轻量级 YOLO 检测器
用于 ANAFI Ai 无人机的实时目标检测

使用方法:
    from object_detector import ObjectDetector
    
    detector = ObjectDetector()  # 默认使用 yolov8n
    results = detector.detect(bgr_frame)
    annotated_frame = detector.draw_results(bgr_frame, results)

支持的预训练模型 (按大小/速度排序):
    - yolov8n: nano (最小最快，推荐无人机使用)
    - yolov8s: small
    - yolov8m: medium
    - yolov8l: large
    - yolov8x: xlarge (最大最准)

后续自定义训练:
    1. 准备数据集 (YOLO 格式)
    2. 训练: yolo train data=your_data.yaml model=yolov8n.pt epochs=100
    3. 使用: detector = ObjectDetector(model_path="runs/detect/train/weights/best.pt")
"""

import time
from pathlib import Path
from typing import List, Dict, Optional, Tuple, Union
from dataclasses import dataclass

import numpy as np
import cv2


@dataclass
class Detection:
    """单个检测结果"""
    class_id: int           # 类别 ID
    class_name: str         # 类别名称
    confidence: float       # 置信度 (0-1)
    bbox: Tuple[int, int, int, int]  # (x1, y1, x2, y2) 边界框
    
    @property
    def center(self) -> Tuple[int, int]:
        """返回边界框中心点"""
        x1, y1, x2, y2 = self.bbox
        return ((x1 + x2) // 2, (y1 + y2) // 2)
    
    @property
    def width(self) -> int:
        return self.bbox[2] - self.bbox[0]
    
    @property
    def height(self) -> int:
        return self.bbox[3] - self.bbox[1]
    
    @property
    def area(self) -> int:
        return self.width * self.height


class ObjectDetector:
    """
    轻量级目标检测器 - 封装 YOLOv8
    
    Args:
        model_path: 模型路径或名称 (如 "yolov8n.pt", "yolov8s.pt", 或自定义路径)
        conf_threshold: 置信度阈值 (0-1)，低于此值的检测会被过滤
        iou_threshold: NMS 的 IOU 阈值
        device: 运行设备 ("cpu", "cuda", "cuda:0", 等)，None 表示自动选择
        classes: 只检测指定的类别 ID 列表，None 表示检测所有类别
        verbose: 是否打印详细信息
    """
    
    # COCO 数据集的 80 个类别名称
    COCO_CLASSES = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat',
        'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat',
        'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack',
        'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
        'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
        'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
        'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
        'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
        'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator',
        'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
    ]
    
    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.45,
        device: Optional[str] = None,
        classes: Optional[List[int]] = None,
        class_names_filter: Optional[List[str]] = None,
        verbose: bool = True
    ):
        """
        Args:
            model_path: 模型路径或名称
            conf_threshold: 置信度阈值 (0-1)
            iou_threshold: NMS 的 IOU 阈值
            device: 运行设备 ("cpu", "cuda" 等)
            classes: 只检测指定的类别 ID 列表
            class_names_filter: 只检测指定的类别名称列表 (如 ["person", "car", "dog"])
            verbose: 是否打印详细信息
        """
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        self.filter_classes = classes
        self.class_names_filter = class_names_filter
        self.verbose = verbose
        self.model = None
        self.class_names = {}
        
        self._load_model(model_path)
    
    def _load_model(self, model_path: str):
        """加载 YOLO 模型"""
        try:
            from ultralytics import YOLO
        except ImportError:
            raise ImportError(
                "请安装 ultralytics: pip install ultralytics\n"
                "或: pip install ultralytics opencv-python"
            )
        
        if self.verbose:
            print(f"[ObjectDetector] 加载模型: {model_path}")
        
        self.model = YOLO(model_path, verbose=self.verbose)
        
        # 获取类别名称
        if hasattr(self.model, 'names'):
            self.class_names = self.model.names
        else:
            # 默认使用 COCO 类别
            self.class_names = {i: name for i, name in enumerate(self.COCO_CLASSES)}
        
        # 如果指定了类别名称过滤，转换为类别 ID
        if self.class_names_filter and not self.filter_classes:
            self.filter_classes = self.names_to_ids(self.class_names_filter)
            if self.verbose and self.filter_classes:
                print(f"[ObjectDetector] 名称过滤转换为 ID: {self.class_names_filter} -> {self.filter_classes}")
        
        if self.verbose:
            print(f"[ObjectDetector] 模型加载完成，共 {len(self.class_names)} 个类别")
            if self.filter_classes:
                filtered_names = [self.class_names.get(c, f"class_{c}") for c in self.filter_classes]
                print(f"[ObjectDetector] 只检测: {filtered_names}")
    
    def names_to_ids(self, names: List[str]) -> List[int]:
        """将类别名称列表转换为类别 ID 列表"""
        ids = []
        names_lower = [n.lower().strip() for n in names]
        for cls_id, cls_name in self.class_names.items():
            if cls_name.lower() in names_lower:
                ids.append(cls_id)
        return ids
    
    def set_filter_by_names(self, names: List[str]):
        """
        通过类别名称设置过滤器
        
        Args:
            names: 类别名称列表，如 ["person", "car", "dog"]
        
        Example:
            detector.set_filter_by_names(["person", "car"])
        """
        self.class_names_filter = names
        self.filter_classes = self.names_to_ids(names) if names else None
        if self.verbose:
            if self.filter_classes:
                print(f"[ObjectDetector] 设置过滤: {names} -> ID {self.filter_classes}")
            else:
                print("[ObjectDetector] 清除过滤，检测所有类别")
    
    def clear_filter(self):
        """清除类别过滤，检测所有类别"""
        self.filter_classes = None
        self.class_names_filter = None
        if self.verbose:
            print("[ObjectDetector] 过滤已清除，检测所有类别")
    
    def detect(self, frame: np.ndarray) -> List[Detection]:
        """
        对单帧图像进行目标检测
        
        Args:
            frame: BGR 格式的 numpy 数组图像 (OpenCV 格式)
        
        Returns:
            检测结果列表
        """
        if frame is None or frame.size == 0:
            return []
        
        # 运行推理
        results = self.model.predict(
            source=frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device,
            classes=self.filter_classes,
            verbose=False
        )
        
        detections = []
        
        if results and len(results) > 0:
            result = results[0]  # 单张图片只有一个结果
            
            if result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes
                
                for i in range(len(boxes)):
                    # 获取边界框坐标 (xyxy 格式)
                    xyxy = boxes.xyxy[i].cpu().numpy()
                    x1, y1, x2, y2 = map(int, xyxy)
                    
                    # 获取置信度和类别
                    conf = float(boxes.conf[i].cpu().numpy())
                    cls_id = int(boxes.cls[i].cpu().numpy())
                    cls_name = self.class_names.get(cls_id, f"class_{cls_id}")
                    
                    detection = Detection(
                        class_id=cls_id,
                        class_name=cls_name,
                        confidence=conf,
                        bbox=(x1, y1, x2, y2)
                    )
                    detections.append(detection)
        
        return detections
    
    def draw_results(
        self,
        frame: np.ndarray,
        detections: List[Detection],
        color: Optional[Tuple[int, int, int]] = None,
        thickness: int = 2,
        font_scale: float = 0.6,
        show_confidence: bool = True,
        show_class: bool = True
    ) -> np.ndarray:
        """
        在图像上绘制检测结果
        
        Args:
            frame: 原始 BGR 图像
            detections: 检测结果列表
            color: 边框颜色 (B, G, R)，None 表示根据类别自动选择
            thickness: 边框粗细
            font_scale: 字体大小
            show_confidence: 是否显示置信度
            show_class: 是否显示类别名称
        
        Returns:
            绘制后的图像 (副本)
        """
        annotated = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            
            # 根据类别 ID 生成颜色
            if color is None:
                # 使用类别 ID 生成稳定的颜色
                np.random.seed(det.class_id)
                box_color = tuple(map(int, np.random.randint(0, 255, 3)))
                np.random.seed(None)  # 重置随机种子
            else:
                box_color = color
            
            # 绘制边界框
            cv2.rectangle(annotated, (x1, y1), (x2, y2), box_color, thickness)
            
            # 构建标签文本
            label_parts = []
            if show_class:
                label_parts.append(det.class_name)
            if show_confidence:
                label_parts.append(f"{det.confidence:.2f}")
            label = " ".join(label_parts)
            
            if label:
                # 计算文本尺寸
                (text_w, text_h), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness
                )
                
                # 绘制标签背景
                cv2.rectangle(
                    annotated,
                    (x1, y1 - text_h - baseline - 5),
                    (x1 + text_w + 5, y1),
                    box_color,
                    -1  # 填充
                )
                
                # 绘制标签文本 (白色)
                cv2.putText(
                    annotated,
                    label,
                    (x1 + 2, y1 - baseline - 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    (255, 255, 255),
                    thickness
                )
        
        return annotated
    
    def detect_and_draw(self, frame: np.ndarray, **draw_kwargs) -> Tuple[np.ndarray, List[Detection]]:
        """
        检测并绘制结果（便捷方法）
        
        Returns:
            (annotated_frame, detections)
        """
        detections = self.detect(frame)
        annotated = self.draw_results(frame, detections, **draw_kwargs)
        return annotated, detections
    
    def filter_by_class(self, detections: List[Detection], class_names: List[str]) -> List[Detection]:
        """
        按类别名称过滤检测结果
        
        Args:
            detections: 检测结果列表
            class_names: 要保留的类别名称列表
        
        Returns:
            过滤后的检测结果
        """
        class_names_lower = [c.lower() for c in class_names]
        return [d for d in detections if d.class_name.lower() in class_names_lower]
    
    def filter_by_confidence(self, detections: List[Detection], min_conf: float) -> List[Detection]:
        """按置信度过滤"""
        return [d for d in detections if d.confidence >= min_conf]
    
    def filter_by_area(self, detections: List[Detection], min_area: int = 0, max_area: int = float('inf')) -> List[Detection]:
        """按面积过滤"""
        return [d for d in detections if min_area <= d.area <= max_area]
    
    def get_largest(self, detections: List[Detection], class_name: Optional[str] = None) -> Optional[Detection]:
        """
        获取最大的检测目标
        
        Args:
            detections: 检测结果列表
            class_name: 可选，只在指定类别中查找
        
        Returns:
            最大的检测结果，如果没有则返回 None
        """
        if class_name:
            detections = self.filter_by_class(detections, [class_name])
        
        if not detections:
            return None
        
        return max(detections, key=lambda d: d.area)
    
    def get_closest_to_center(self, detections: List[Detection], frame_shape: Tuple[int, int, int]) -> Optional[Detection]:
        """
        获取最接近画面中心的检测目标
        
        Args:
            detections: 检测结果列表
            frame_shape: 图像形状 (height, width, channels)
        
        Returns:
            最接近中心的检测结果
        """
        if not detections:
            return None
        
        h, w = frame_shape[:2]
        center = (w // 2, h // 2)
        
        def distance_to_center(det):
            dx = det.center[0] - center[0]
            dy = det.center[1] - center[1]
            return dx * dx + dy * dy
        
        return min(detections, key=distance_to_center)
    
    @property
    def available_classes(self) -> Dict[int, str]:
        """返回可用的类别字典 {id: name}"""
        return self.class_names.copy()
    
    def print_classes(self):
        """打印所有可用类别"""
        print("可检测的类别:")
        for cls_id, cls_name in sorted(self.class_names.items()):
            print(f"  {cls_id:3d}: {cls_name}")


class DetectorBenchmark:
    """检测器性能测试工具"""
    
    def __init__(self, detector: ObjectDetector):
        self.detector = detector
        self.inference_times = []
    
    def benchmark(self, frame: np.ndarray, num_runs: int = 100) -> Dict:
        """
        测试检测速度
        
        Returns:
            包含 fps、avg_ms、min_ms、max_ms 的字典
        """
        self.inference_times = []
        
        # 预热
        for _ in range(5):
            self.detector.detect(frame)
        
        # 正式测试
        for _ in range(num_runs):
            start = time.perf_counter()
            self.detector.detect(frame)
            elapsed = (time.perf_counter() - start) * 1000  # ms
            self.inference_times.append(elapsed)
        
        avg_ms = sum(self.inference_times) / len(self.inference_times)
        
        return {
            "fps": 1000.0 / avg_ms,
            "avg_ms": avg_ms,
            "min_ms": min(self.inference_times),
            "max_ms": max(self.inference_times),
            "num_runs": num_runs
        }


# ============================================================
# 测试代码 + 命令行工具
# ============================================================
def parse_args():
    """解析命令行参数"""
    import argparse
    
    p = argparse.ArgumentParser(
        description="目标检测模块 - 使用摄像头或图片进行目标检测",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 使用默认摄像头，检测所有类别
  python object_detector.py

  # 只检测 person 和 car
  python object_detector.py --classes person car

  # 只检测 dog cat bird，置信度阈值 0.3
  python object_detector.py --classes dog cat bird --conf 0.3

  # 使用图片测试
  python object_detector.py --image test.jpg --classes person

  # 列出所有可检测的类别
  python object_detector.py --list-classes

  # 使用外部摄像头
  python object_detector.py --camera 1

COCO 数据集常用类别:
  人物: person
  交通: car, truck, bus, motorcycle, bicycle, airplane, boat, train
  动物: dog, cat, bird, horse, cow, sheep, elephant, bear, zebra, giraffe
  物品: chair, couch, bed, dining table, tv, laptop, cell phone, bottle, cup
        """
    )
    
    p.add_argument("--image", "-i", type=str, default=None,
                   help="测试图片路径（不指定则使用摄像头）")
    p.add_argument("--camera", "-c", type=int, default=0,
                   help="摄像头编号（默认 0）")
    
    p.add_argument("--model", "-m", default="yolov8n.pt",
                   help="YOLO 模型路径（默认 yolov8n.pt，首次运行会自动下载）")
    p.add_argument("--conf", type=float, default=0.5,
                   help="置信度阈值 0-1（默认 0.5）")
    
    p.add_argument("--classes", nargs="+", type=str, default=None,
                   help="只检测指定的类别名称，如: --classes person car dog")
    
    p.add_argument("--list-classes", action="store_true",
                   help="列出所有可检测的类别后退出")
    
    p.add_argument("--benchmark", action="store_true",
                   help="运行性能测试")
    
    return p.parse_args()


def list_all_classes():
    """列出所有 COCO 类别"""
    print("=" * 60)
    print("COCO 数据集 80 个可检测类别")
    print("=" * 60)
    print()
    
    categories = {
        "人物": ["person"],
        "交通工具": ["bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat"],
        "交通设施": ["traffic light", "fire hydrant", "stop sign", "parking meter"],
        "户外物品": ["bench", "backpack", "umbrella", "handbag", "tie", "suitcase"],
        "动物": ["bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe"],
        "运动器材": ["frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", 
                   "baseball glove", "skateboard", "surfboard", "tennis racket"],
        "餐具": ["bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl"],
        "食物": ["banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", 
                "pizza", "donut", "cake"],
        "家具": ["chair", "couch", "potted plant", "bed", "dining table"],
        "家电": ["tv", "laptop", "mouse", "remote", "keyboard", "cell phone", 
                "microwave", "oven", "toaster", "sink", "refrigerator"],
        "其他": ["toilet", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"],
    }
    
    idx = 0
    for category, items in categories.items():
        print(f"【{category}】")
        for item in items:
            print(f"  {idx:2d}: {item}")
            idx += 1
        print()
    
    print("-" * 60)
    print("使用示例: python object_detector.py --classes person car dog")
    print("-" * 60)


def test_with_webcam(args):
    """使用摄像头测试检测器"""
    print("=" * 60)
    print("目标检测模块测试 - 使用摄像头")
    print("=" * 60)
    
    # 创建检测器
    detector = ObjectDetector(
        model_path=args.model,
        conf_threshold=args.conf,
        class_names_filter=args.classes,  # 使用名称过滤
        verbose=True
    )
    
    if not args.classes:
        # 打印可用类别
        print("\n检测所有类别。常用类别示例:")
        print("  --classes person car        只检测人和车")
        print("  --classes dog cat bird      只检测宠物和鸟")
    
    # 打开摄像头
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"[ERROR] 无法打开摄像头 {args.camera}")
        print("提示: 尝试 --camera 1 或其他编号")
        return
    
    print(f"\n摄像头 {args.camera} 已打开")
    print("按 'q' 退出 | 按 'c' 切换过滤 | 按 's' 截图")
    print("-" * 60)
    
    fps_list = []
    screenshot_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # 检测
        start = time.perf_counter()
        annotated, detections = detector.detect_and_draw(frame)
        elapsed_ms = (time.perf_counter() - start) * 1000
        fps = 1000.0 / elapsed_ms if elapsed_ms > 0 else 0
        fps_list.append(fps)
        
        # 显示信息
        avg_fps = sum(fps_list[-30:]) / len(fps_list[-30:])
        
        # 显示过滤状态
        if detector.class_names_filter:
            filter_text = f"Filter: {', '.join(detector.class_names_filter)}"
        else:
            filter_text = "Filter: ALL"
        
        # 绘制信息条
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], 60), (0, 0, 0), -1)
        cv2.putText(annotated, f"FPS: {avg_fps:.1f} | Objects: {len(detections)}", 
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated, filter_text,
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # 显示
        cv2.imshow("Object Detection (q=quit, c=filter, s=screenshot)", annotated)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            # 切换过滤模式
            if detector.filter_classes:
                detector.clear_filter()
            else:
                detector.set_filter_by_names(["person", "car"])
        elif key == ord('s'):
            # 截图
            screenshot_count += 1
            filename = f"detection_screenshot_{screenshot_count}.jpg"
            cv2.imwrite(filename, annotated)
            print(f"[INFO] 截图已保存: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    if fps_list:
        print(f"\n平均 FPS: {sum(fps_list) / len(fps_list):.1f}")


def test_with_image(args):
    """使用图片测试检测器"""
    print("=" * 60)
    print(f"目标检测模块测试 - 图片: {args.image}")
    print("=" * 60)
    
    frame = cv2.imread(args.image)
    if frame is None:
        print(f"[ERROR] 无法读取图片: {args.image}")
        return
    
    detector = ObjectDetector(
        model_path=args.model,
        conf_threshold=args.conf,
        class_names_filter=args.classes,
        verbose=True
    )
    
    # 性能测试
    if args.benchmark:
        benchmark = DetectorBenchmark(detector)
        results = benchmark.benchmark(frame, num_runs=50)
        print(f"\n性能测试结果:")
        print(f"  平均 FPS: {results['fps']:.1f}")
        print(f"  平均延迟: {results['avg_ms']:.1f} ms")
        print(f"  最小延迟: {results['min_ms']:.1f} ms")
        print(f"  最大延迟: {results['max_ms']:.1f} ms")
    
    # 检测并显示
    annotated, detections = detector.detect_and_draw(frame)
    
    print(f"\n检测到 {len(detections)} 个目标:")
    for det in detections:
        print(f"  - {det.class_name}: {det.confidence:.2f} @ {det.bbox}")
    
    cv2.imshow("Detection Result (press any key to close)", annotated)
    print("\n按任意键关闭")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    args = parse_args()
    
    if args.list_classes:
        list_all_classes()
    elif args.image:
        test_with_image(args)
    else:
        test_with_webcam(args)

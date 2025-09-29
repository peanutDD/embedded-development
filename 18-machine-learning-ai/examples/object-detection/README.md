# 目标检测示例

## 概述

本示例展示了如何在边缘设备上实现高效的目标检测，包括YOLO、SSD、MobileNet-SSD等主流检测算法的实现。支持实时检测、批量处理和多种硬件加速方案。

## 功能特性

- **多算法支持**: YOLOv5、YOLOv8、SSD、MobileNet-SSD等
- **多格式兼容**: TensorFlow Lite、ONNX、TensorRT等格式
- **实时检测**: 摄像头实时目标检测
- **多类别检测**: 支持COCO、Pascal VOC等数据集
- **边界框可视化**: 丰富的检测结果可视化
- **性能监控**: FPS、延迟、检测精度统计

## 项目结构

```
object-detection/
├── README.md                    # 本文件
├── requirements.txt             # Python依赖
├── models/                      # 预训练模型
│   ├── yolov5s.tflite          # YOLOv5s模型
│   ├── yolov8n.onnx            # YOLOv8n模型
│   ├── ssd_mobilenet.tflite    # SSD MobileNet模型
│   └── efficientdet_d0.trt     # EfficientDet模型
├── src/                         # 源代码
│   ├── __init__.py
│   ├── detector.py             # 检测器核心类
│   ├── yolo_detector.py        # YOLO检测器
│   ├── ssd_detector.py         # SSD检测器
│   ├── postprocessing.py       # 后处理模块
│   ├── visualization.py        # 可视化模块
│   └── utils.py                # 工具函数
├── examples/                    # 使用示例
│   ├── basic_detection.py      # 基础检测示例
│   ├── realtime_detection.py   # 实时检测
│   ├── video_detection.py      # 视频检测
│   └── batch_detection.py      # 批量检测
├── data/                        # 测试数据
│   ├── test_images/            # 测试图像
│   ├── test_videos/            # 测试视频
│   └── coco_classes.txt        # COCO类别标签
├── configs/                     # 配置文件
│   ├── yolo_config.yaml        # YOLO配置
│   ├── ssd_config.yaml         # SSD配置
│   └── detection_config.yaml   # 通用检测配置
└── tests/                       # 测试用例
    ├── test_detector.py        # 检测器测试
    ├── test_postprocessing.py  # 后处理测试
    └── test_performance.py     # 性能测试
```

## 快速开始

### 1. 环境准备

```bash
# 安装依赖
pip install -r requirements.txt

# 下载预训练模型
python scripts/download_models.py
```

### 2. 基础使用

```python
from src.detector import EdgeObjectDetector

# 创建检测器
detector = EdgeObjectDetector(
    model_path='models/yolov5s.tflite',
    model_type='yolo',
    device='cpu'
)

# 加载模型
detector.load_model()

# 检测单张图像
import cv2
image = cv2.imread('data/test_images/street.jpg')
detections = detector.detect(image, conf_threshold=0.5)

print(f"检测到 {len(detections)} 个目标:")
for detection in detections:
    print(f"{detection['class_name']}: {detection['confidence']:.3f}")
```

### 3. 实时检测

```bash
python examples/realtime_detection.py --model models/yolov5s.tflite --device cpu
```

### 4. 视频检测

```bash
python examples/video_detection.py --input data/test_videos/traffic.mp4 --output results/output.mp4
```

## 核心组件

### 1. 目标检测器基类

```python
import numpy as np
import cv2
import time
from typing import List, Dict, Any, Tuple
from abc import ABC, abstractmethod

class BaseObjectDetector(ABC):
    """目标检测器基类"""
    
    def __init__(self, model_path: str, device: str = 'cpu'):
        self.model_path = model_path
        self.device = device
        self.model = None
        self.class_names = []
        self.input_shape = None
        self.is_loaded = False
        
        # 检测参数
        self.conf_threshold = 0.5
        self.nms_threshold = 0.4
        self.max_detections = 100
    
    @abstractmethod
    def load_model(self) -> bool:
        """加载模型"""
        pass
    
    @abstractmethod
    def predict(self, input_data: np.ndarray) -> np.ndarray:
        """执行推理"""
        pass
    
    @abstractmethod
    def postprocess(self, predictions: np.ndarray, original_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        """后处理预测结果"""
        pass
    
    def preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, float, int, int]:
        """预处理图像"""
        original_height, original_width = image.shape[:2]
        
        if self.input_shape:
            target_height, target_width = self.input_shape[1:3]
            
            # 等比例缩放
            scale = min(target_width / original_width, target_height / original_height)
            new_width = int(original_width * scale)
            new_height = int(original_height * scale)
            
            # 调整大小
            resized = cv2.resize(image, (new_width, new_height))
            
            # 填充到目标尺寸
            padded = np.full((target_height, target_width, 3), 114, dtype=np.uint8)
            
            # 计算填充位置
            pad_x = (target_width - new_width) // 2
            pad_y = (target_height - new_height) // 2
            
            padded[pad_y:pad_y + new_height, pad_x:pad_x + new_width] = resized
            
            # 归一化
            processed = padded.astype(np.float32) / 255.0
            
            # 添加批次维度
            processed = np.expand_dims(processed, axis=0)
            
            return processed, scale, pad_x, pad_y
        else:
            # 默认处理
            processed = image.astype(np.float32) / 255.0
            processed = np.expand_dims(processed, axis=0)
            return processed, 1.0, 0, 0
    
    def detect(self, image: np.ndarray, conf_threshold: float = None, nms_threshold: float = None) -> List[Dict[str, Any]]:
        """检测目标"""
        if not self.is_loaded:
            raise RuntimeError("模型未加载")
        
        # 更新阈值
        if conf_threshold is not None:
            self.conf_threshold = conf_threshold
        if nms_threshold is not None:
            self.nms_threshold = nms_threshold
        
        # 预处理
        processed_image, scale, pad_x, pad_y = self.preprocess(image)
        
        # 推理
        start_time = time.time()
        predictions = self.predict(processed_image)
        inference_time = (time.time() - start_time) * 1000
        
        # 后处理
        detections = self.postprocess(predictions, image.shape[:2])
        
        # 坐标变换
        for detection in detections:
            detection['inference_time_ms'] = inference_time
            self.transform_coordinates(detection, scale, pad_x, pad_y, image.shape[:2])
        
        return detections
    
    def transform_coordinates(self, detection: Dict[str, Any], scale: float, pad_x: int, pad_y: int, original_shape: Tuple[int, int]):
        """变换坐标到原始图像空间"""
        original_height, original_width = original_shape
        
        bbox = detection['bbox']
        x, y, w, h = bbox
        
        # 反变换
        x = (x - pad_x) / scale
        y = (y - pad_y) / scale
        w = w / scale
        h = h / scale
        
        # 裁剪到图像边界
        x = max(0, min(x, original_width))
        y = max(0, min(y, original_height))
        w = min(w, original_width - x)
        h = min(h, original_height - y)
        
        detection['bbox'] = [int(x), int(y), int(w), int(h)]
    
    def get_class_name(self, class_id: int) -> str:
        """获取类别名称"""
        if class_id < len(self.class_names):
            return self.class_names[class_id]
        else:
            return f'class_{class_id}'
    
    def load_class_names(self, class_file: str = 'data/coco_classes.txt'):
        """加载类别名称"""
        try:
            with open(class_file, 'r', encoding='utf-8') as f:
                self.class_names = [line.strip() for line in f.readlines()]
        except FileNotFoundError:
            # 使用COCO默认类别
            self.class_names = [
                'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
                'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
                'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
                'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
                'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat',
                'baseball glove', 'skateboard', 'surfboard', 'tennis racket',
                'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl',
                'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot',
                'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop',
                'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
                'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock',
                'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush'
            ]

class YOLODetector(BaseObjectDetector):
    """YOLO检测器"""
    
    def load_model(self) -> bool:
        """加载YOLO模型"""
        try:
            import tensorflow as tf
            
            # 创建解释器
            if self.device == 'gpu':
                try:
                    delegate = tf.lite.experimental.load_delegate('libdelegate_gpu.so')
                    self.interpreter = tf.lite.Interpreter(
                        model_path=self.model_path,
                        experimental_delegates=[delegate]
                    )
                except:
                    self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
            elif self.device == 'tpu':
                try:
                    from pycoral.utils import edgetpu
                    self.interpreter = edgetpu.make_interpreter(self.model_path)
                except ImportError:
                    return False
            else:
                self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
            
            self.interpreter.allocate_tensors()
            
            # 获取输入输出信息
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            # 获取输入形状
            self.input_shape = self.input_details[0]['shape']
            
            # 加载类别名称
            self.load_class_names()
            
            self.is_loaded = True
            return True
            
        except Exception as e:
            print(f"YOLO模型加载失败: {e}")
            return False
    
    def predict(self, input_data: np.ndarray) -> np.ndarray:
        """执行推理"""
        # 设置输入
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        
        # 执行推理
        self.interpreter.invoke()
        
        # 获取输出
        output = self.interpreter.get_tensor(self.output_details[0]['index'])
        
        return output
    
    def postprocess(self, predictions: np.ndarray, original_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        """YOLO后处理"""
        detections = []
        
        # YOLO输出格式: [batch, num_detections, 85] (x, y, w, h, conf, class_probs...)
        for detection in predictions[0]:  # 移除批次维度
            # 解析检测结果
            x_center, y_center, width, height = detection[:4]
            confidence = detection[4]
            class_probs = detection[5:]
            
            # 过滤低置信度检测
            if confidence < self.conf_threshold:
                continue
            
            # 获取最佳类别
            class_id = np.argmax(class_probs)
            class_confidence = class_probs[class_id]
            
            # 计算最终置信度
            final_confidence = confidence * class_confidence
            
            if final_confidence < self.conf_threshold:
                continue
            
            # 转换边界框格式 (中心点 -> 左上角)
            x = x_center - width / 2
            y = y_center - height / 2
            
            # 缩放到输入图像尺寸
            if self.input_shape:
                input_height, input_width = self.input_shape[1:3]
                x *= input_width
                y *= input_height
                width *= input_width
                height *= input_height
            
            detections.append({
                'bbox': [x, y, width, height],
                'confidence': float(final_confidence),
                'class_id': int(class_id),
                'class_name': self.get_class_name(class_id)
            })
        
        # 非极大值抑制
        if len(detections) > 0:
            detections = self.apply_nms(detections)
        
        return detections
    
    def apply_nms(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """应用非极大值抑制"""
        if len(detections) == 0:
            return detections
        
        # 提取边界框和置信度
        boxes = []
        confidences = []
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            boxes.append([x, y, w, h])
            confidences.append(detection['confidence'])
        
        # 应用NMS
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences, 
            self.conf_threshold, self.nms_threshold
        )
        
        # 过滤结果
        filtered_detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                filtered_detections.append(detections[i])
        
        return filtered_detections[:self.max_detections]

class SSDDetector(BaseObjectDetector):
    """SSD检测器"""
    
    def load_model(self) -> bool:
        """加载SSD模型"""
        try:
            import tensorflow as tf
            
            # 创建解释器
            self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
            self.interpreter.allocate_tensors()
            
            # 获取输入输出信息
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            # 获取输入形状
            self.input_shape = self.input_details[0]['shape']
            
            # 加载类别名称
            self.load_class_names()
            
            self.is_loaded = True
            return True
            
        except Exception as e:
            print(f"SSD模型加载失败: {e}")
            return False
    
    def predict(self, input_data: np.ndarray) -> Dict[str, np.ndarray]:
        """执行推理"""
        # 设置输入
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        
        # 执行推理
        self.interpreter.invoke()
        
        # 获取输出 (SSD通常有多个输出)
        outputs = {}
        for output_detail in self.output_details:
            output_name = output_detail['name']
            output_data = self.interpreter.get_tensor(output_detail['index'])
            outputs[output_name] = output_data
        
        return outputs
    
    def postprocess(self, predictions: Dict[str, np.ndarray], original_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        """SSD后处理"""
        detections = []
        
        # SSD输出通常包含: boxes, classes, scores, num_detections
        if 'detection_boxes' in predictions:
            boxes = predictions['detection_boxes'][0]  # 移除批次维度
            classes = predictions['detection_classes'][0]
            scores = predictions['detection_scores'][0]
            num_detections = int(predictions['num_detections'][0])
        else:
            # 尝试其他可能的输出名称
            output_keys = list(predictions.keys())
            boxes = predictions[output_keys[0]][0]
            classes = predictions[output_keys[1]][0]
            scores = predictions[output_keys[2]][0]
            num_detections = len(scores)
        
        # 处理检测结果
        for i in range(min(num_detections, len(scores))):
            score = scores[i]
            
            if score < self.conf_threshold:
                continue
            
            # 获取边界框 (归一化坐标)
            y1, x1, y2, x2 = boxes[i]
            
            # 转换为像素坐标
            if self.input_shape:
                input_height, input_width = self.input_shape[1:3]
                x1 *= input_width
                y1 *= input_height
                x2 *= input_width
                y2 *= input_height
            
            # 转换为 (x, y, w, h) 格式
            x = x1
            y = y1
            width = x2 - x1
            height = y2 - y1
            
            # 获取类别ID
            class_id = int(classes[i])
            
            detections.append({
                'bbox': [x, y, width, height],
                'confidence': float(score),
                'class_id': class_id,
                'class_name': self.get_class_name(class_id)
            })
        
        return detections[:self.max_detections]

class EdgeObjectDetector:
    """边缘目标检测器工厂类"""
    
    @staticmethod
    def create_detector(model_path: str, model_type: str, device: str = 'cpu') -> BaseObjectDetector:
        """创建检测器实例"""
        if model_type.lower() == 'yolo':
            return YOLODetector(model_path, device)
        elif model_type.lower() == 'ssd':
            return SSDDetector(model_path, device)
        else:
            raise ValueError(f"不支持的模型类型: {model_type}")
```

### 2. 实时检测

```python
import cv2
import argparse
import time
import numpy as np
from collections import deque
from src.detector import EdgeObjectDetector
from src.visualization import DetectionVisualizer

class RealTimeDetector:
    """实时目标检测器"""
    
    def __init__(self, model_path: str, model_type: str, device: str = 'cpu'):
        self.detector = EdgeObjectDetector.create_detector(model_path, model_type, device)
        self.detector.load_model()
        self.visualizer = DetectionVisualizer()
        
        # 性能统计
        self.fps_history = deque(maxlen=30)
        self.detection_counts = deque(maxlen=100)
        self.inference_times = deque(maxlen=30)
        
        # 检测参数
        self.conf_threshold = 0.5
        self.nms_threshold = 0.4
        
    def run(self, camera_id: int = 0, display_size: tuple = (640, 480)):
        """运行实时检测"""
        # 初始化摄像头
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, display_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, display_size[1])
        
        if not cap.isOpened():
            print("无法打开摄像头")
            return
        
        print("实时目标检测已启动")
        print("按键说明:")
        print("  'q' - 退出")
        print("  's' - 保存当前帧")
        print("  '+' - 增加置信度阈值")
        print("  '-' - 减少置信度阈值")
        print("  'r' - 重置统计信息")
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 执行检测
            try:
                detections = self.detector.detect(
                    frame, 
                    conf_threshold=self.conf_threshold,
                    nms_threshold=self.nms_threshold
                )
                
                # 更新统计信息
                self.detection_counts.append(len(detections))
                if detections:
                    self.inference_times.append(detections[0]['inference_time_ms'])
                
                # 可视化结果
                self.visualizer.draw_detections(frame, detections)
                
            except Exception as e:
                print(f"检测错误: {e}")
                detections = []
            
            # 计算FPS
            frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            if elapsed_time >= 1.0:
                fps = frame_count / elapsed_time
                self.fps_history.append(fps)
                frame_count = 0
                start_time = current_time
            
            # 绘制性能信息和控制界面
            self.draw_performance_info(frame)
            self.draw_control_info(frame)
            
            # 显示结果
            cv2.imshow('Real-time Object Detection', frame)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_frame(frame, detections)
            elif key == ord('+') or key == ord('='):
                self.conf_threshold = min(0.95, self.conf_threshold + 0.05)
                print(f"置信度阈值: {self.conf_threshold:.2f}")
            elif key == ord('-'):
                self.conf_threshold = max(0.05, self.conf_threshold - 0.05)
                print(f"置信度阈值: {self.conf_threshold:.2f}")
            elif key == ord('r'):
                self.reset_statistics()
                print("统计信息已重置")
        
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()
        
        # 打印最终统计
        self.print_final_statistics()
    
    def draw_performance_info(self, frame: np.ndarray):
        """绘制性能信息"""
        height, width = frame.shape[:2]
        
        # 背景框
        cv2.rectangle(frame, (width - 250, 10), (width - 10, 120), (0, 0, 0), -1)
        cv2.rectangle(frame, (width - 250, 10), (width - 10, 120), (255, 255, 255), 2)
        
        y_offset = 30
        
        # FPS信息
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            fps_text = f"FPS: {avg_fps:.1f}"
            cv2.putText(frame, fps_text, (width - 240, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
        
        # 推理时间
        if self.inference_times:
            avg_inference = sum(self.inference_times) / len(self.inference_times)
            inference_text = f"Inference: {avg_inference:.1f}ms"
            cv2.putText(frame, inference_text, (width - 240, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            y_offset += 25
        
        # 检测数量
        if self.detection_counts:
            avg_detections = sum(self.detection_counts) / len(self.detection_counts)
            detection_text = f"Objects: {avg_detections:.1f}"
            cv2.putText(frame, detection_text, (width - 240, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            y_offset += 25
        
        # 置信度阈值
        threshold_text = f"Conf: {self.conf_threshold:.2f}"
        cv2.putText(frame, threshold_text, (width - 240, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    def draw_control_info(self, frame: np.ndarray):
        """绘制控制信息"""
        height, width = frame.shape[:2]
        
        # 背景框
        cv2.rectangle(frame, (10, height - 80), (200, height - 10), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, height - 80), (200, height - 10), (255, 255, 255), 2)
        
        # 控制说明
        controls = ["Q:Quit  S:Save", "+/-:Threshold  R:Reset"]
        y_offset = height - 60
        
        for control in controls:
            cv2.putText(frame, control, (15, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 20
    
    def save_frame(self, frame: np.ndarray, detections: List[Dict[str, Any]]):
        """保存当前帧"""
        timestamp = int(time.time())
        filename = f'detection_{timestamp}.jpg'
        
        # 保存图像
        cv2.imwrite(filename, frame)
        
        # 保存检测结果
        result_filename = f'detection_{timestamp}.json'
        import json
        
        detection_data = {
            'timestamp': timestamp,
            'filename': filename,
            'detections': detections,
            'parameters': {
                'conf_threshold': self.conf_threshold,
                'nms_threshold': self.nms_threshold
            }
        }
        
        with open(result_filename, 'w') as f:
            json.dump(detection_data, f, indent=2)
        
        print(f"保存: {filename}, {result_filename}")
    
    def reset_statistics(self):
        """重置统计信息"""
        self.fps_history.clear()
        self.detection_counts.clear()
        self.inference_times.clear()
    
    def print_final_statistics(self):
        """打印最终统计信息"""
        print("\n=== 检测统计 ===")
        
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            print(f"平均FPS: {avg_fps:.2f}")
        
        if self.inference_times:
            avg_inference = sum(self.inference_times) / len(self.inference_times)
            min_inference = min(self.inference_times)
            max_inference = max(self.inference_times)
            print(f"推理时间: 平均 {avg_inference:.2f}ms, 最小 {min_inference:.2f}ms, 最大 {max_inference:.2f}ms")
        
        if self.detection_counts:
            avg_detections = sum(self.detection_counts) / len(self.detection_counts)
            max_detections = max(self.detection_counts)
            print(f"检测目标: 平均 {avg_detections:.1f}个, 最多 {max_detections}个")

def main():
    parser = argparse.ArgumentParser(description='实时目标检测')
    parser.add_argument('--model', required=True, help='模型文件路径')
    parser.add_argument('--type', default='yolo', choices=['yolo', 'ssd'], help='模型类型')
    parser.add_argument('--device', default='cpu', choices=['cpu', 'gpu', 'tpu'], help='推理设备')
    parser.add_argument('--camera', type=int, default=0, help='摄像头ID')
    parser.add_argument('--size', nargs=2, type=int, default=[640, 480], help='显示尺寸')
    parser.add_argument('--conf', type=float, default=0.5, help='置信度阈值')
    parser.add_argument('--nms', type=float, default=0.4, help='NMS阈值')
    
    args = parser.parse_args()
    
    # 创建实时检测器
    detector = RealTimeDetector(args.model, args.type, args.device)
    detector.conf_threshold = args.conf
    detector.nms_threshold = args.nms
    
    # 运行检测
    detector.run(args.camera, tuple(args.size))

if __name__ == "__main__":
    main()
```

### 3. 可视化模块

```python
import cv2
import numpy as np
import random
from typing import List, Dict, Any

class DetectionVisualizer:
    """检测结果可视化器"""
    
    def __init__(self):
        # 生成类别颜色
        self.colors = self.generate_colors(80)  # COCO有80个类别
        
        # 可视化参数
        self.box_thickness = 2
        self.text_thickness = 2
        self.text_scale = 0.7
        self.text_padding = 5
    
    def generate_colors(self, num_classes: int) -> List[tuple]:
        """生成类别颜色"""
        colors = []
        random.seed(42)  # 固定随机种子以保持颜色一致性
        
        for _ in range(num_classes):
            color = (
                random.randint(0, 255),
                random.randint(0, 255),
                random.randint(0, 255)
            )
            colors.append(color)
        
        return colors
    
    def draw_detections(self, image: np.ndarray, detections: List[Dict[str, Any]], 
                       show_confidence: bool = True, show_class_id: bool = False) -> np.ndarray:
        """绘制检测结果"""
        result_image = image.copy()
        
        for detection in detections:
            self.draw_single_detection(
                result_image, detection, 
                show_confidence, show_class_id
            )
        
        return result_image
    
    def draw_single_detection(self, image: np.ndarray, detection: Dict[str, Any], 
                            show_confidence: bool = True, show_class_id: bool = False):
        """绘制单个检测结果"""
        # 获取检测信息
        bbox = detection['bbox']
        confidence = detection['confidence']
        class_id = detection['class_id']
        class_name = detection['class_name']
        
        x, y, w, h = bbox
        x1, y1 = int(x), int(y)
        x2, y2 = int(x + w), int(y + h)
        
        # 获取颜色
        color = self.colors[class_id % len(self.colors)]
        
        # 绘制边界框
        cv2.rectangle(image, (x1, y1), (x2, y2), color, self.box_thickness)
        
        # 准备标签文本
        label_parts = []
        if show_class_id:
            label_parts.append(f"[{class_id}]")
        label_parts.append(class_name)
        if show_confidence:
            label_parts.append(f"{confidence:.2f}")
        
        label = " ".join(label_parts)
        
        # 计算文本尺寸
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, self.text_scale, self.text_thickness
        )
        
        # 绘制标签背景
        label_y = y1 - text_height - self.text_padding if y1 - text_height - self.text_padding > 0 else y1 + text_height + self.text_padding
        
        cv2.rectangle(
            image,
            (x1, label_y - text_height - self.text_padding),
            (x1 + text_width + 2 * self.text_padding, label_y + self.text_padding),
            color,
            -1
        )
        
        # 绘制标签文本
        cv2.putText(
            image, label,
            (x1 + self.text_padding, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            self.text_scale,
            (255, 255, 255),
            self.text_thickness
        )
    
    def draw_detection_statistics(self, image: np.ndarray, detections: List[Dict[str, Any]]):
        """绘制检测统计信息"""
        # 统计各类别数量
        class_counts = {}
        for detection in detections:
            class_name = detection['class_name']
            class_counts[class_name] = class_counts.get(class_name, 0) + 1
        
        # 绘制统计信息
        height, width = image.shape[:2]
        y_offset = 30
        
        # 背景
        stats_height = len(class_counts) * 25 + 40
        cv2.rectangle(image, (10, 10), (250, stats_height), (0, 0, 0), -1)
        cv2.rectangle(image, (10, 10), (250, stats_height), (255, 255, 255), 2)
        
        # 标题
        cv2.putText(image, "Detection Statistics:", (20, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 30
        
        # 各类别统计
        for class_name, count in sorted(class_counts.items()):
            text = f"{class_name}: {count}"
            cv2.putText(image, text, (20, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 25
    
    def create_detection_grid(self, images: List[np.ndarray], detections_list: List[List[Dict[str, Any]]], 
                            grid_size: tuple = None) -> np.ndarray:
        """创建检测结果网格显示"""
        if not images:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        
        num_images = len(images)
        
        # 自动计算网格大小
        if grid_size is None:
            cols = int(np.ceil(np.sqrt(num_images)))
            rows = int(np.ceil(num_images / cols))
            grid_size = (rows, cols)
        
        rows, cols = grid_size
        
        # 调整图像大小
        target_height = 240
        target_width = 320
        
        grid_images = []
        for i in range(rows * cols):
            if i < len(images):
                # 绘制检测结果
                img_with_detections = self.draw_detections(images[i], detections_list[i])
                
                # 调整大小
                resized = cv2.resize(img_with_detections, (target_width, target_height))
                grid_images.append(resized)
            else:
                # 空白图像
                blank = np.zeros((target_height, target_width, 3), dtype=np.uint8)
                grid_images.append(blank)
        
        # 创建网格
        grid_rows = []
        for row in range(rows):
            row_images = grid_images[row * cols:(row + 1) * cols]
            grid_row = np.hstack(row_images)
            grid_rows.append(grid_row)
        
        grid_image = np.vstack(grid_rows)
        
        return grid_image
    
    def save_detection_video(self, video_path: str, output_path: str, detections_list: List[List[Dict[str, Any]]]):
        """保存检测结果视频"""
        # 打开输入视频
        cap = cv2.VideoCapture(video_path)
        
        if not cap.isOpened():
            print(f"无法打开视频: {video_path}")
            return False
        
        # 获取视频属性
        fps = int(cap.get(cv2.CAP_PROP_FPS))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # 创建输出视频写入器
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        
        frame_idx = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 绘制检测结果
            if frame_idx < len(detections_list):
                frame_with_detections = self.draw_detections(frame, detections_list[frame_idx])
            else:
                frame_with_detections = frame
            
            # 写入帧
            out.write(frame_with_detections)
            frame_idx += 1
        
        # 清理资源
        cap.release()
        out.release()
        
        print(f"检测结果视频已保存: {output_path}")
        return True
```

## 性能优化

### 1. 模型优化
- **轻量级架构**: 使用MobileNet、EfficientDet等轻量级backbone
- **模型量化**: INT8量化减少模型大小和推理时间
- **模型剪枝**: 移除冗余参数以压缩模型

### 2. 推理优化
- **批处理**: 同时处理多张图像
- **多线程**: 并行处理预处理和后处理
- **内存优化**: 重用内存缓冲区

### 3. 算法优化
- **NMS优化**: 使用高效的NMS算法
- **锚框优化**: 针对特定场景优化锚框设置
- **多尺度检测**: 平衡精度和速度

## 应用场景

- **智能监控**: 人员、车辆检测
- **自动驾驶**: 道路目标检测
- **工业检测**: 产品缺陷检测
- **零售分析**: 商品识别和统计
- **安全防护**: 危险物品检测

## 扩展功能

- **目标跟踪**: 结合跟踪算法实现目标跟踪
- **行为分析**: 基于检测结果进行行为识别
- **3D检测**: 扩展到3D目标检测
- **实例分割**: 结合分割算法提供像素级检测

欢迎贡献代码和提出改进建议！
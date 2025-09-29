# 图像分类示例

## 概述

本示例展示了如何在边缘设备上实现高效的图像分类，包括模型加载、预处理、推理和后处理的完整流程。支持多种轻量级模型架构和硬件加速方案。

## 功能特性

- **多模型支持**: MobileNet、EfficientNet、ResNet等轻量级模型
- **多格式兼容**: TensorFlow Lite、ONNX、TensorRT等格式
- **硬件加速**: CPU、GPU、NPU、TPU等硬件平台
- **实时处理**: 摄像头实时图像分类
- **批处理**: 支持批量图像处理
- **性能监控**: 延迟、吞吐量、准确率统计

## 项目结构

```
image-classification/
├── README.md                    # 本文件
├── requirements.txt             # Python依赖
├── models/                      # 预训练模型
│   ├── mobilenet_v2.tflite     # MobileNet V2模型
│   ├── efficientnet_b0.onnx    # EfficientNet B0模型
│   └── resnet18.trt            # ResNet18 TensorRT模型
├── src/                         # 源代码
│   ├── __init__.py
│   ├── classifier.py           # 分类器核心类
│   ├── preprocessing.py        # 图像预处理
│   ├── postprocessing.py       # 结果后处理
│   ├── utils.py                # 工具函数
│   └── benchmark.py            # 性能测试
├── examples/                    # 使用示例
│   ├── basic_classification.py # 基础分类示例
│   ├── realtime_camera.py      # 实时摄像头分类
│   ├── batch_processing.py     # 批量处理示例
│   └── model_comparison.py     # 模型对比
├── data/                        # 测试数据
│   ├── test_images/            # 测试图像
│   └── imagenet_classes.txt    # ImageNet类别标签
├── configs/                     # 配置文件
│   ├── mobilenet_config.yaml   # MobileNet配置
│   ├── efficientnet_config.yaml # EfficientNet配置
│   └── deployment_config.yaml  # 部署配置
└── tests/                       # 测试用例
    ├── test_classifier.py      # 分类器测试
    ├── test_preprocessing.py   # 预处理测试
    └── test_performance.py     # 性能测试
```

## 快速开始

### 1. 环境准备

```bash
# 安装依赖
pip install -r requirements.txt

# 下载预训练模型（可选）
python scripts/download_models.py
```

### 2. 基础使用

```python
from src.classifier import EdgeImageClassifier

# 创建分类器
classifier = EdgeImageClassifier(
    model_path='models/mobilenet_v2.tflite',
    model_format='tflite',
    device='cpu'
)

# 加载模型
classifier.load_model()

# 分类单张图像
import cv2
image = cv2.imread('data/test_images/cat.jpg')
results = classifier.classify(image, top_k=5)

print("分类结果:")
for result in results:
    print(f"{result['class_name']}: {result['confidence']:.3f}")
```

### 3. 实时摄像头分类

```bash
python examples/realtime_camera.py --model models/mobilenet_v2.tflite --device cpu
```

### 4. 批量处理

```bash
python examples/batch_processing.py --input_dir data/test_images --output_dir results
```

## 核心组件

### 1. 图像分类器

```python
import numpy as np
import cv2
import time
from typing import List, Dict, Any, Optional
from abc import ABC, abstractmethod

class BaseImageClassifier(ABC):
    """图像分类器基类"""
    
    def __init__(self, model_path: str, device: str = 'cpu'):
        self.model_path = model_path
        self.device = device
        self.model = None
        self.class_names = []
        self.input_shape = None
        self.is_loaded = False
    
    @abstractmethod
    def load_model(self) -> bool:
        """加载模型"""
        pass
    
    @abstractmethod
    def predict(self, input_data: np.ndarray) -> np.ndarray:
        """执行推理"""
        pass
    
    def preprocess(self, image: np.ndarray) -> np.ndarray:
        """预处理图像"""
        # 调整大小
        if self.input_shape:
            height, width = self.input_shape[1:3]
            image = cv2.resize(image, (width, height))
        
        # 归一化
        image = image.astype(np.float32) / 255.0
        
        # 标准化
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        image = (image - mean) / std
        
        # 添加批次维度
        image = np.expand_dims(image, axis=0)
        
        return image
    
    def postprocess(self, predictions: np.ndarray, top_k: int = 5) -> List[Dict[str, Any]]:
        """后处理预测结果"""
        # 应用softmax
        exp_preds = np.exp(predictions - np.max(predictions))
        probabilities = exp_preds / np.sum(exp_preds)
        
        # 获取top-k结果
        top_indices = np.argsort(probabilities)[-top_k:][::-1]
        
        results = []
        for idx in top_indices:
            results.append({
                'class_id': int(idx),
                'class_name': self.get_class_name(idx),
                'confidence': float(probabilities[idx])
            })
        
        return results
    
    def get_class_name(self, class_id: int) -> str:
        """获取类别名称"""
        if class_id < len(self.class_names):
            return self.class_names[class_id]
        else:
            return f'class_{class_id}'
    
    def classify(self, image: np.ndarray, top_k: int = 5) -> List[Dict[str, Any]]:
        """分类图像"""
        if not self.is_loaded:
            raise RuntimeError("模型未加载")
        
        # 预处理
        processed_image = self.preprocess(image)
        
        # 推理
        start_time = time.time()
        predictions = self.predict(processed_image)
        inference_time = (time.time() - start_time) * 1000
        
        # 后处理
        results = self.postprocess(predictions[0], top_k)
        
        # 添加推理时间
        for result in results:
            result['inference_time_ms'] = inference_time
        
        return results

class TensorFlowLiteClassifier(BaseImageClassifier):
    """TensorFlow Lite分类器"""
    
    def load_model(self) -> bool:
        """加载TensorFlow Lite模型"""
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
            print(f"模型加载失败: {e}")
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
    
    def load_class_names(self):
        """加载类别名称"""
        try:
            with open('data/imagenet_classes.txt', 'r') as f:
                self.class_names = [line.strip() for line in f.readlines()]
        except FileNotFoundError:
            # 使用默认类别名称
            self.class_names = [f'class_{i}' for i in range(1000)]

class ONNXClassifier(BaseImageClassifier):
    """ONNX分类器"""
    
    def load_model(self) -> bool:
        """加载ONNX模型"""
        try:
            import onnxruntime as ort
            
            # 设置执行提供者
            providers = ['CPUExecutionProvider']
            if self.device == 'gpu':
                providers.insert(0, 'CUDAExecutionProvider')
            
            # 创建推理会话
            self.session = ort.InferenceSession(self.model_path, providers=providers)
            
            # 获取输入输出信息
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            self.input_shape = self.session.get_inputs()[0].shape
            
            # 加载类别名称
            self.load_class_names()
            
            self.is_loaded = True
            return True
            
        except Exception as e:
            print(f"ONNX模型加载失败: {e}")
            return False
    
    def predict(self, input_data: np.ndarray) -> np.ndarray:
        """执行推理"""
        result = self.session.run([self.output_name], {self.input_name: input_data})
        return result[0]
    
    def load_class_names(self):
        """加载类别名称"""
        try:
            with open('data/imagenet_classes.txt', 'r') as f:
                self.class_names = [line.strip() for line in f.readlines()]
        except FileNotFoundError:
            self.class_names = [f'class_{i}' for i in range(1000)]

class EdgeImageClassifier:
    """边缘图像分类器工厂类"""
    
    @staticmethod
    def create_classifier(model_path: str, model_format: str, device: str = 'cpu') -> BaseImageClassifier:
        """创建分类器实例"""
        if model_format.lower() == 'tflite':
            return TensorFlowLiteClassifier(model_path, device)
        elif model_format.lower() == 'onnx':
            return ONNXClassifier(model_path, device)
        else:
            raise ValueError(f"不支持的模型格式: {model_format}")
```

### 2. 实时摄像头分类

```python
import cv2
import argparse
import time
from collections import deque
from src.classifier import EdgeImageClassifier

class RealTimeClassifier:
    """实时图像分类器"""
    
    def __init__(self, model_path: str, model_format: str, device: str = 'cpu'):
        self.classifier = EdgeImageClassifier.create_classifier(model_path, model_format, device)
        self.classifier.load_model()
        
        # 性能统计
        self.fps_history = deque(maxlen=30)
        self.inference_times = deque(maxlen=30)
        
    def run(self, camera_id: int = 0, display_size: tuple = (640, 480)):
        """运行实时分类"""
        # 初始化摄像头
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, display_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, display_size[1])
        
        if not cap.isOpened():
            print("无法打开摄像头")
            return
        
        print("按 'q' 退出，按 's' 保存当前帧")
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 执行分类
            try:
                results = self.classifier.classify(frame, top_k=3)
                
                # 更新性能统计
                if results:
                    self.inference_times.append(results[0]['inference_time_ms'])
                
                # 绘制结果
                self.draw_results(frame, results)
                
            except Exception as e:
                print(f"分类错误: {e}")
            
            # 计算FPS
            frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            if elapsed_time >= 1.0:
                fps = frame_count / elapsed_time
                self.fps_history.append(fps)
                frame_count = 0
                start_time = current_time
            
            # 绘制性能信息
            self.draw_performance_info(frame)
            
            # 显示结果
            cv2.imshow('Real-time Classification', frame)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                timestamp = int(time.time())
                filename = f'capture_{timestamp}.jpg'
                cv2.imwrite(filename, frame)
                print(f"保存图像: {filename}")
        
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()
    
    def draw_results(self, frame: np.ndarray, results: List[Dict[str, Any]]):
        """绘制分类结果"""
        y_offset = 30
        
        for i, result in enumerate(results):
            class_name = result['class_name']
            confidence = result['confidence']
            
            # 选择颜色
            if i == 0:
                color = (0, 255, 0)  # 绿色 - 最高置信度
            elif i == 1:
                color = (0, 255, 255)  # 黄色 - 第二高
            else:
                color = (255, 255, 255)  # 白色 - 其他
            
            # 绘制文本
            text = f"{class_name}: {confidence:.3f}"
            cv2.putText(frame, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            
            y_offset += 30
    
    def draw_performance_info(self, frame: np.ndarray):
        """绘制性能信息"""
        height, width = frame.shape[:2]
        
        # FPS信息
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            fps_text = f"FPS: {avg_fps:.1f}"
            cv2.putText(frame, fps_text, (width - 150, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # 推理时间信息
        if self.inference_times:
            avg_inference_time = sum(self.inference_times) / len(self.inference_times)
            inference_text = f"Inference: {avg_inference_time:.1f}ms"
            cv2.putText(frame, inference_text, (width - 200, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

def main():
    parser = argparse.ArgumentParser(description='实时图像分类')
    parser.add_argument('--model', required=True, help='模型文件路径')
    parser.add_argument('--format', default='tflite', choices=['tflite', 'onnx'], help='模型格式')
    parser.add_argument('--device', default='cpu', choices=['cpu', 'gpu', 'tpu'], help='推理设备')
    parser.add_argument('--camera', type=int, default=0, help='摄像头ID')
    parser.add_argument('--size', nargs=2, type=int, default=[640, 480], help='显示尺寸')
    
    args = parser.parse_args()
    
    # 创建实时分类器
    classifier = RealTimeClassifier(args.model, args.format, args.device)
    
    # 运行分类
    classifier.run(args.camera, tuple(args.size))

if __name__ == "__main__":
    main()
```

### 3. 批量处理

```python
import os
import json
import time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Dict, Any
import cv2
import numpy as np
from src.classifier import EdgeImageClassifier

class BatchImageProcessor:
    """批量图像处理器"""
    
    def __init__(self, model_path: str, model_format: str, device: str = 'cpu', num_workers: int = 4):
        self.classifier = EdgeImageClassifier.create_classifier(model_path, model_format, device)
        self.classifier.load_model()
        self.num_workers = num_workers
        
        # 支持的图像格式
        self.supported_formats = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.webp'}
    
    def process_directory(self, input_dir: str, output_dir: str, top_k: int = 5) -> Dict[str, Any]:
        """处理目录中的所有图像"""
        input_path = Path(input_dir)
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        # 获取所有图像文件
        image_files = []
        for ext in self.supported_formats:
            image_files.extend(input_path.glob(f'*{ext}'))
            image_files.extend(input_path.glob(f'*{ext.upper()}'))
        
        if not image_files:
            print(f"在 {input_dir} 中未找到支持的图像文件")
            return {'processed': 0, 'failed': 0, 'results': []}
        
        print(f"找到 {len(image_files)} 个图像文件")
        
        # 批量处理
        results = []
        failed_count = 0
        
        start_time = time.time()
        
        if self.num_workers > 1:
            # 多线程处理
            with ThreadPoolExecutor(max_workers=self.num_workers) as executor:
                future_to_file = {
                    executor.submit(self.process_single_image, str(img_file), top_k): img_file 
                    for img_file in image_files
                }
                
                for future in as_completed(future_to_file):
                    img_file = future_to_file[future]
                    try:
                        result = future.result()
                        if result:
                            result['filename'] = img_file.name
                            results.append(result)
                        else:
                            failed_count += 1
                    except Exception as e:
                        print(f"处理 {img_file.name} 时出错: {e}")
                        failed_count += 1
        else:
            # 单线程处理
            for img_file in image_files:
                try:
                    result = self.process_single_image(str(img_file), top_k)
                    if result:
                        result['filename'] = img_file.name
                        results.append(result)
                    else:
                        failed_count += 1
                except Exception as e:
                    print(f"处理 {img_file.name} 时出错: {e}")
                    failed_count += 1
        
        end_time = time.time()
        total_time = end_time - start_time
        
        # 保存结果
        self.save_results(results, output_path, total_time)
        
        # 生成统计报告
        stats = self.generate_statistics(results, total_time, failed_count)
        
        return stats
    
    def process_single_image(self, image_path: str, top_k: int = 5) -> Dict[str, Any]:
        """处理单张图像"""
        try:
            # 加载图像
            image = cv2.imread(image_path)
            if image is None:
                return None
            
            # 执行分类
            classification_results = self.classifier.classify(image, top_k)
            
            # 获取图像信息
            height, width, channels = image.shape
            file_size = os.path.getsize(image_path)
            
            result = {
                'image_path': image_path,
                'image_info': {
                    'width': width,
                    'height': height,
                    'channels': channels,
                    'file_size_bytes': file_size
                },
                'classifications': classification_results,
                'processing_time_ms': classification_results[0]['inference_time_ms'] if classification_results else 0
            }
            
            return result
            
        except Exception as e:
            print(f"处理图像 {image_path} 时出错: {e}")
            return None
    
    def save_results(self, results: List[Dict[str, Any]], output_path: Path, total_time: float):
        """保存处理结果"""
        # 保存JSON结果
        json_output = {
            'metadata': {
                'total_images': len(results),
                'processing_time_seconds': total_time,
                'average_time_per_image_ms': (total_time * 1000) / len(results) if results else 0,
                'timestamp': time.time()
            },
            'results': results
        }
        
        json_file = output_path / 'classification_results.json'
        with open(json_file, 'w', encoding='utf-8') as f:
            json.dump(json_output, f, indent=2, ensure_ascii=False)
        
        # 保存CSV摘要
        csv_file = output_path / 'classification_summary.csv'
        with open(csv_file, 'w', encoding='utf-8') as f:
            f.write('filename,top1_class,top1_confidence,processing_time_ms\n')
            
            for result in results:
                filename = result['filename']
                if result['classifications']:
                    top1 = result['classifications'][0]
                    class_name = top1['class_name']
                    confidence = top1['confidence']
                    processing_time = result['processing_time_ms']
                    
                    f.write(f'{filename},{class_name},{confidence:.4f},{processing_time:.2f}\n')
        
        print(f"结果已保存到 {output_path}")
    
    def generate_statistics(self, results: List[Dict[str, Any]], total_time: float, failed_count: int) -> Dict[str, Any]:
        """生成统计信息"""
        if not results:
            return {
                'processed': 0,
                'failed': failed_count,
                'total_time_seconds': total_time,
                'average_time_per_image_ms': 0,
                'throughput_images_per_second': 0
            }
        
        # 计算统计信息
        processing_times = [r['processing_time_ms'] for r in results]
        
        stats = {
            'processed': len(results),
            'failed': failed_count,
            'total_time_seconds': total_time,
            'average_time_per_image_ms': sum(processing_times) / len(processing_times),
            'min_time_ms': min(processing_times),
            'max_time_ms': max(processing_times),
            'throughput_images_per_second': len(results) / total_time,
            'success_rate': len(results) / (len(results) + failed_count)
        }
        
        # 类别统计
        class_counts = {}
        for result in results:
            if result['classifications']:
                top1_class = result['classifications'][0]['class_name']
                class_counts[top1_class] = class_counts.get(top1_class, 0) + 1
        
        # 按出现次数排序
        sorted_classes = sorted(class_counts.items(), key=lambda x: x[1], reverse=True)
        stats['top_classes'] = sorted_classes[:10]  # 前10个最常见的类别
        
        return stats

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='批量图像分类')
    parser.add_argument('--model', required=True, help='模型文件路径')
    parser.add_argument('--format', default='tflite', choices=['tflite', 'onnx'], help='模型格式')
    parser.add_argument('--device', default='cpu', choices=['cpu', 'gpu', 'tpu'], help='推理设备')
    parser.add_argument('--input_dir', required=True, help='输入图像目录')
    parser.add_argument('--output_dir', required=True, help='输出结果目录')
    parser.add_argument('--top_k', type=int, default=5, help='返回前K个结果')
    parser.add_argument('--workers', type=int, default=4, help='并行工作线程数')
    
    args = parser.parse_args()
    
    # 创建批量处理器
    processor = BatchImageProcessor(args.model, args.format, args.device, args.workers)
    
    # 处理图像
    stats = processor.process_directory(args.input_dir, args.output_dir, args.top_k)
    
    # 打印统计信息
    print("\n=== 处理统计 ===")
    print(f"成功处理: {stats['processed']} 张图像")
    print(f"处理失败: {stats['failed']} 张图像")
    print(f"成功率: {stats['success_rate']:.2%}")
    print(f"总耗时: {stats['total_time_seconds']:.2f} 秒")
    print(f"平均耗时: {stats['average_time_per_image_ms']:.2f} ms/图像")
    print(f"吞吐量: {stats['throughput_images_per_second']:.2f} 图像/秒")
    
    if 'top_classes' in stats:
        print("\n=== 最常见的类别 ===")
        for class_name, count in stats['top_classes']:
            print(f"{class_name}: {count} 张图像")

if __name__ == "__main__":
    main()
```

## 性能优化

### 1. 模型优化

- **量化**: 将FP32模型转换为INT8以减少模型大小和推理时间
- **剪枝**: 移除不重要的权重以压缩模型
- **知识蒸馏**: 使用大模型训练小模型以保持精度

### 2. 推理优化

- **批处理**: 同时处理多张图像以提高吞吐量
- **预处理优化**: 使用高效的图像处理库
- **内存管理**: 重用内存缓冲区以减少分配开销

### 3. 硬件加速

- **GPU加速**: 使用CUDA或OpenCL加速推理
- **NPU支持**: 利用专用神经网络处理器
- **TPU优化**: 针对Edge TPU优化模型

## 测试和验证

### 1. 功能测试

```bash
python tests/test_classifier.py
```

### 2. 性能测试

```bash
python tests/test_performance.py --model models/mobilenet_v2.tflite
```

### 3. 准确率验证

```bash
python tests/test_accuracy.py --model models/mobilenet_v2.tflite --dataset data/validation
```

## 部署指南

### 1. 边缘设备部署

- **树莓派**: 使用TensorFlow Lite CPU推理
- **Jetson Nano**: 使用TensorRT GPU加速
- **Coral Dev Board**: 使用Edge TPU加速

### 2. 移动设备部署

- **Android**: 使用TensorFlow Lite Android库
- **iOS**: 使用TensorFlow Lite iOS库

### 3. 云边协同

- **边缘推理**: 本地实时分类
- **云端训练**: 定期更新模型
- **结果同步**: 上传分类结果和统计信息

## 常见问题

### Q: 如何提高分类准确率？

A: 
1. 使用更大的预训练模型
2. 针对特定领域进行微调
3. 改进图像预处理流程
4. 增加数据增强

### Q: 如何减少推理延迟？

A: 
1. 使用更轻量级的模型
2. 启用硬件加速
3. 优化图像预处理
4. 使用模型量化

### Q: 如何处理内存不足问题？

A: 
1. 减少批处理大小
2. 使用更小的输入分辨率
3. 启用内存优化选项
4. 使用模型压缩技术

## 扩展功能

- **自定义分类**: 训练特定领域的分类模型
- **多标签分类**: 支持一张图像多个标签
- **层次分类**: 实现分层的类别结构
- **在线学习**: 支持增量学习和模型更新

## 贡献指南

欢迎提交问题报告、功能请求和代码贡献。请遵循项目的编码规范和测试要求。

## 许可证

本项目采用MIT许可证，详见LICENSE文件。
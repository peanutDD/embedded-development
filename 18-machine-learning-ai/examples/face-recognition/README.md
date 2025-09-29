# 人脸识别示例

## 概述

本示例展示了如何在边缘设备上实现高效的人脸识别系统，包括人脸检测、特征提取、身份验证和人脸数据库管理的完整流程。支持实时识别、批量处理和多种安全认证场景。

## 功能特性

- **人脸检测**: 基于MTCNN、RetinaFace等算法的高精度人脸检测
- **特征提取**: 使用FaceNet、ArcFace等模型提取人脸特征
- **身份验证**: 1:1验证和1:N识别两种模式
- **活体检测**: 防止照片、视频等欺骗攻击
- **人脸数据库**: 支持本地人脸数据库管理
- **隐私保护**: 本地处理，保护用户隐私

## 项目结构

```
face-recognition/
├── README.md                    # 本文件
├── requirements.txt             # Python依赖
├── models/                      # 预训练模型
│   ├── face_detection/         # 人脸检测模型
│   │   ├── mtcnn.tflite       # MTCNN模型
│   │   └── retinaface.onnx    # RetinaFace模型
│   ├── face_recognition/       # 人脸识别模型
│   │   ├── facenet.tflite     # FaceNet模型
│   │   └── arcface.onnx       # ArcFace模型
│   └── liveness_detection/     # 活体检测模型
│       └── liveness.tflite    # 活体检测模型
├── src/                         # 源代码
│   ├── __init__.py
│   ├── face_detector.py        # 人脸检测器
│   ├── face_recognizer.py      # 人脸识别器
│   ├── liveness_detector.py    # 活体检测器
│   ├── face_database.py        # 人脸数据库管理
│   ├── face_system.py          # 完整人脸识别系统
│   └── utils.py                # 工具函数
├── examples/                    # 使用示例
│   ├── basic_recognition.py    # 基础识别示例
│   ├── realtime_recognition.py # 实时识别
│   ├── face_registration.py    # 人脸注册
│   ├── batch_recognition.py    # 批量识别
│   └── security_system.py      # 安全系统示例
├── data/                        # 数据目录
│   ├── face_database/          # 人脸数据库
│   ├── test_images/            # 测试图像
│   └── registered_faces/       # 已注册人脸
├── configs/                     # 配置文件
│   ├── detection_config.yaml   # 检测配置
│   ├── recognition_config.yaml # 识别配置
│   └── system_config.yaml      # 系统配置
└── tests/                       # 测试用例
    ├── test_detector.py        # 检测器测试
    ├── test_recognizer.py      # 识别器测试
    └── test_system.py          # 系统测试
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
from src.face_system import FaceRecognitionSystem

# 创建人脸识别系统
face_system = FaceRecognitionSystem(
    detector_model='models/face_detection/mtcnn.tflite',
    recognizer_model='models/face_recognition/facenet.tflite',
    device='cpu'
)

# 初始化系统
face_system.initialize()

# 注册新人脸
import cv2
image = cv2.imread('data/test_images/person1.jpg')
success = face_system.register_face(image, person_id='person_001', name='张三')

# 识别人脸
results = face_system.recognize_faces(image)
for result in results:
    print(f"识别结果: {result['name']}, 置信度: {result['confidence']:.3f}")
```

### 3. 实时人脸识别

```bash
python examples/realtime_recognition.py --camera 0
```

### 4. 批量人脸注册

```bash
python examples/face_registration.py --input_dir data/face_photos --output_db data/face_database
```

## 核心组件

### 1. 人脸检测器

```python
import cv2
import numpy as np
import time
from typing import List, Dict, Any, Tuple
from abc import ABC, abstractmethod

class BaseFaceDetector(ABC):
    """人脸检测器基类"""
    
    def __init__(self, model_path: str, device: str = 'cpu'):
        self.model_path = model_path
        self.device = device
        self.model = None
        self.is_loaded = False
        
        # 检测参数
        self.min_face_size = 20
        self.confidence_threshold = 0.7
        self.nms_threshold = 0.4
    
    @abstractmethod
    def load_model(self) -> bool:
        """加载模型"""
        pass
    
    @abstractmethod
    def detect_faces(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """检测人脸"""
        pass
    
    def preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """预处理图像"""
        # 转换为RGB
        if len(image.shape) == 3 and image.shape[2] == 3:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            image_rgb = image
        
        return image_rgb
    
    def postprocess_detections(self, detections: List[Dict[str, Any]], 
                             original_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        """后处理检测结果"""
        filtered_detections = []
        
        for detection in detections:
            # 过滤小人脸
            bbox = detection['bbox']
            width = bbox[2] - bbox[0]
            height = bbox[3] - bbox[1]
            
            if width >= self.min_face_size and height >= self.min_face_size:
                # 过滤低置信度
                if detection['confidence'] >= self.confidence_threshold:
                    filtered_detections.append(detection)
        
        # 应用NMS
        if len(filtered_detections) > 1:
            filtered_detections = self.apply_nms(filtered_detections)
        
        return filtered_detections
    
    def apply_nms(self, detections: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """应用非极大值抑制"""
        if len(detections) <= 1:
            return detections
        
        # 提取边界框和置信度
        boxes = []
        confidences = []
        
        for detection in detections:
            bbox = detection['bbox']
            x1, y1, x2, y2 = bbox
            boxes.append([x1, y1, x2 - x1, y2 - y1])  # 转换为(x, y, w, h)格式
            confidences.append(detection['confidence'])
        
        # 应用NMS
        indices = cv2.dnn.NMSBoxes(
            boxes, confidences,
            self.confidence_threshold, self.nms_threshold
        )
        
        # 过滤结果
        filtered_detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                filtered_detections.append(detections[i])
        
        return filtered_detections

class MTCNNDetector(BaseFaceDetector):
    """MTCNN人脸检测器"""
    
    def load_model(self) -> bool:
        """加载MTCNN模型"""
        try:
            import tensorflow as tf
            
            # 加载TensorFlow Lite模型
            self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
            self.interpreter.allocate_tensors()
            
            # 获取输入输出信息
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            self.is_loaded = True
            return True
            
        except Exception as e:
            print(f"MTCNN模型加载失败: {e}")
            return False
    
    def detect_faces(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """使用MTCNN检测人脸"""
        if not self.is_loaded:
            raise RuntimeError("模型未加载")
        
        # 预处理
        processed_image = self.preprocess_image(image)
        
        # 多尺度检测
        detections = []
        scales = [0.5, 0.7, 1.0, 1.2, 1.5]
        
        for scale in scales:
            scaled_detections = self.detect_at_scale(processed_image, scale)
            detections.extend(scaled_detections)
        
        # 后处理
        final_detections = self.postprocess_detections(detections, image.shape[:2])
        
        return final_detections
    
    def detect_at_scale(self, image: np.ndarray, scale: float) -> List[Dict[str, Any]]:
        """在特定尺度下检测人脸"""
        height, width = image.shape[:2]
        
        # 缩放图像
        new_height = int(height * scale)
        new_width = int(width * scale)
        
        if new_height < 12 or new_width < 12:  # MTCNN最小输入尺寸
            return []
        
        scaled_image = cv2.resize(image, (new_width, new_height))
        
        # 归一化
        normalized_image = (scaled_image.astype(np.float32) - 127.5) / 128.0
        
        # 添加批次维度
        input_data = np.expand_dims(normalized_image, axis=0)
        
        # 推理
        self.interpreter.set_tensor(self.input_details[0]['index'], input_data)
        self.interpreter.invoke()
        
        # 获取输出
        boxes = self.interpreter.get_tensor(self.output_details[0]['index'])
        scores = self.interpreter.get_tensor(self.output_details[1]['index'])
        landmarks = self.interpreter.get_tensor(self.output_details[2]['index'])
        
        # 解析结果
        detections = []
        for i in range(len(scores[0])):
            if scores[0][i] > self.confidence_threshold:
                # 缩放回原始尺寸
                box = boxes[0][i] / scale
                landmark = landmarks[0][i] / scale
                
                detection = {
                    'bbox': [int(box[0]), int(box[1]), int(box[2]), int(box[3])],
                    'confidence': float(scores[0][i]),
                    'landmarks': landmark.tolist()
                }
                
                detections.append(detection)
        
        return detections

class RetinaFaceDetector(BaseFaceDetector):
    """RetinaFace人脸检测器"""
    
    def load_model(self) -> bool:
        """加载RetinaFace模型"""
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
            self.input_shape = self.session.get_inputs()[0].shape
            
            self.is_loaded = True
            return True
            
        except Exception as e:
            print(f"RetinaFace模型加载失败: {e}")
            return False
    
    def detect_faces(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """使用RetinaFace检测人脸"""
        if not self.is_loaded:
            raise RuntimeError("模型未加载")
        
        # 预处理
        processed_image = self.preprocess_image(image)
        input_data = self.prepare_input(processed_image)
        
        # 推理
        start_time = time.time()
        outputs = self.session.run(None, {self.input_name: input_data})
        inference_time = (time.time() - start_time) * 1000
        
        # 解析输出
        detections = self.parse_outputs(outputs, image.shape[:2])
        
        # 添加推理时间
        for detection in detections:
            detection['inference_time_ms'] = inference_time
        
        # 后处理
        final_detections = self.postprocess_detections(detections, image.shape[:2])
        
        return final_detections
    
    def prepare_input(self, image: np.ndarray) -> np.ndarray:
        """准备模型输入"""
        # 调整大小到模型输入尺寸
        target_size = (640, 640)  # RetinaFace常用输入尺寸
        resized = cv2.resize(image, target_size)
        
        # 归一化
        normalized = resized.astype(np.float32)
        normalized = (normalized - [104, 117, 123]) / 255.0
        
        # 转换维度顺序 (HWC -> CHW)
        normalized = np.transpose(normalized, (2, 0, 1))
        
        # 添加批次维度
        input_data = np.expand_dims(normalized, axis=0)
        
        return input_data
    
    def parse_outputs(self, outputs: List[np.ndarray], original_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        """解析模型输出"""
        detections = []
        
        # RetinaFace输出: [boxes, scores, landmarks]
        boxes = outputs[0][0]  # 移除批次维度
        scores = outputs[1][0]
        landmarks = outputs[2][0]
        
        original_height, original_width = original_shape
        
        for i in range(len(scores)):
            if scores[i] > self.confidence_threshold:
                # 缩放边界框到原始图像尺寸
                box = boxes[i]
                x1 = int(box[0] * original_width / 640)
                y1 = int(box[1] * original_height / 640)
                x2 = int(box[2] * original_width / 640)
                y2 = int(box[3] * original_height / 640)
                
                # 缩放关键点
                landmark = landmarks[i]
                scaled_landmarks = []
                for j in range(0, len(landmark), 2):
                    x = landmark[j] * original_width / 640
                    y = landmark[j + 1] * original_height / 640
                    scaled_landmarks.extend([x, y])
                
                detection = {
                    'bbox': [x1, y1, x2, y2],
                    'confidence': float(scores[i]),
                    'landmarks': scaled_landmarks
                }
                
                detections.append(detection)
        
        return detections
```

### 2. 人脸识别器

```python
import numpy as np
import cv2
from typing import List, Dict, Any, Optional
from sklearn.metrics.pairwise import cosine_similarity

class FaceRecognizer:
    """人脸识别器"""
    
    def __init__(self, model_path: str, device: str = 'cpu'):
        self.model_path = model_path
        self.device = device
        self.model = None
        self.is_loaded = False
        
        # 识别参数
        self.face_size = (112, 112)  # 标准人脸尺寸
        self.similarity_threshold = 0.6
        self.feature_dim = 512  # 特征维度
    
    def load_model(self) -> bool:
        """加载人脸识别模型"""
        try:
            import tensorflow as tf
            
            # 加载TensorFlow Lite模型
            self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
            self.interpreter.allocate_tensors()
            
            # 获取输入输出信息
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            self.is_loaded = True
            return True
            
        except Exception as e:
            print(f"人脸识别模型加载失败: {e}")
            return False
    
    def extract_features(self, face_image: np.ndarray) -> Optional[np.ndarray]:
        """提取人脸特征"""
        if not self.is_loaded:
            raise RuntimeError("模型未加载")
        
        # 预处理人脸图像
        processed_face = self.preprocess_face(face_image)
        
        # 推理
        self.interpreter.set_tensor(self.input_details[0]['index'], processed_face)
        self.interpreter.invoke()
        
        # 获取特征向量
        features = self.interpreter.get_tensor(self.output_details[0]['index'])
        
        # L2归一化
        features = features / np.linalg.norm(features)
        
        return features[0]  # 移除批次维度
    
    def preprocess_face(self, face_image: np.ndarray) -> np.ndarray:
        """预处理人脸图像"""
        # 调整大小
        resized = cv2.resize(face_image, self.face_size)
        
        # 转换为RGB
        if len(resized.shape) == 3 and resized.shape[2] == 3:
            rgb_face = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        else:
            rgb_face = resized
        
        # 归一化
        normalized = rgb_face.astype(np.float32) / 255.0
        
        # 标准化
        mean = np.array([0.5, 0.5, 0.5])
        std = np.array([0.5, 0.5, 0.5])
        standardized = (normalized - mean) / std
        
        # 添加批次维度
        input_data = np.expand_dims(standardized, axis=0)
        
        return input_data
    
    def align_face(self, image: np.ndarray, landmarks: List[float]) -> np.ndarray:
        """人脸对齐"""
        # 提取关键点
        if len(landmarks) >= 10:  # 5个关键点，每个点2个坐标
            left_eye = np.array([landmarks[0], landmarks[1]])
            right_eye = np.array([landmarks[2], landmarks[3]])
            nose = np.array([landmarks[4], landmarks[5]])
            left_mouth = np.array([landmarks[6], landmarks[7]])
            right_mouth = np.array([landmarks[8], landmarks[9]])
        else:
            # 如果关键点不足，返回原图像
            return image
        
        # 计算眼睛中心点
        eye_center = (left_eye + right_eye) / 2
        
        # 计算旋转角度
        eye_direction = right_eye - left_eye
        angle = np.arctan2(eye_direction[1], eye_direction[0]) * 180 / np.pi
        
        # 计算缩放比例
        eye_distance = np.linalg.norm(eye_direction)
        desired_eye_distance = self.face_size[0] * 0.35  # 期望的眼睛距离
        scale = desired_eye_distance / eye_distance
        
        # 计算变换矩阵
        center = tuple(eye_center.astype(int))
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, scale)
        
        # 调整平移量使人脸居中
        tx = self.face_size[0] / 2 - eye_center[0] * scale
        ty = self.face_size[1] / 2 - eye_center[1] * scale
        
        rotation_matrix[0, 2] += tx
        rotation_matrix[1, 2] += ty
        
        # 应用变换
        aligned_face = cv2.warpAffine(image, rotation_matrix, self.face_size)
        
        return aligned_face
    
    def compare_faces(self, features1: np.ndarray, features2: np.ndarray) -> float:
        """比较两个人脸特征"""
        # 计算余弦相似度
        similarity = cosine_similarity([features1], [features2])[0][0]
        
        return float(similarity)
    
    def verify_face(self, face_features: np.ndarray, reference_features: np.ndarray) -> Dict[str, Any]:
        """1:1人脸验证"""
        similarity = self.compare_faces(face_features, reference_features)
        
        is_match = similarity >= self.similarity_threshold
        
        return {
            'is_match': is_match,
            'similarity': similarity,
            'confidence': similarity if is_match else 1 - similarity
        }
    
    def identify_face(self, face_features: np.ndarray, 
                     known_features: Dict[str, np.ndarray]) -> Dict[str, Any]:
        """1:N人脸识别"""
        best_match = None
        best_similarity = 0.0
        
        for person_id, known_feature in known_features.items():
            similarity = self.compare_faces(face_features, known_feature)
            
            if similarity > best_similarity and similarity >= self.similarity_threshold:
                best_similarity = similarity
                best_match = person_id
        
        if best_match:
            return {
                'person_id': best_match,
                'similarity': best_similarity,
                'confidence': best_similarity
            }
        else:
            return {
                'person_id': 'unknown',
                'similarity': 0.0,
                'confidence': 0.0
            }
```

### 3. 人脸数据库管理

```python
import os
import json
import pickle
import numpy as np
from typing import Dict, List, Any, Optional
from datetime import datetime

class FaceDatabase:
    """人脸数据库管理器"""
    
    def __init__(self, database_path: str):
        self.database_path = database_path
        self.features_file = os.path.join(database_path, 'features.pkl')
        self.metadata_file = os.path.join(database_path, 'metadata.json')
        
        # 创建数据库目录
        os.makedirs(database_path, exist_ok=True)
        
        # 加载现有数据
        self.features = self.load_features()
        self.metadata = self.load_metadata()
    
    def load_features(self) -> Dict[str, np.ndarray]:
        """加载特征数据"""
        if os.path.exists(self.features_file):
            try:
                with open(self.features_file, 'rb') as f:
                    return pickle.load(f)
            except Exception as e:
                print(f"加载特征数据失败: {e}")
        
        return {}
    
    def load_metadata(self) -> Dict[str, Dict[str, Any]]:
        """加载元数据"""
        if os.path.exists(self.metadata_file):
            try:
                with open(self.metadata_file, 'r', encoding='utf-8') as f:
                    return json.load(f)
            except Exception as e:
                print(f"加载元数据失败: {e}")
        
        return {}
    
    def save_features(self):
        """保存特征数据"""
        try:
            with open(self.features_file, 'wb') as f:
                pickle.dump(self.features, f)
        except Exception as e:
            print(f"保存特征数据失败: {e}")
    
    def save_metadata(self):
        """保存元数据"""
        try:
            with open(self.metadata_file, 'w', encoding='utf-8') as f:
                json.dump(self.metadata, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"保存元数据失败: {e}")
    
    def add_person(self, person_id: str, name: str, features: np.ndarray, 
                   additional_info: Dict[str, Any] = None) -> bool:
        """添加人员"""
        try:
            # 保存特征
            self.features[person_id] = features
            
            # 保存元数据
            metadata = {
                'name': name,
                'person_id': person_id,
                'created_at': datetime.now().isoformat(),
                'updated_at': datetime.now().isoformat(),
                'feature_dim': len(features),
                'additional_info': additional_info or {}
            }
            
            self.metadata[person_id] = metadata
            
            # 持久化保存
            self.save_features()
            self.save_metadata()
            
            return True
            
        except Exception as e:
            print(f"添加人员失败: {e}")
            return False
    
    def update_person(self, person_id: str, name: str = None, 
                     features: np.ndarray = None, 
                     additional_info: Dict[str, Any] = None) -> bool:
        """更新人员信息"""
        if person_id not in self.metadata:
            print(f"人员 {person_id} 不存在")
            return False
        
        try:
            # 更新特征
            if features is not None:
                self.features[person_id] = features
            
            # 更新元数据
            if name is not None:
                self.metadata[person_id]['name'] = name
            
            if additional_info is not None:
                self.metadata[person_id]['additional_info'].update(additional_info)
            
            self.metadata[person_id]['updated_at'] = datetime.now().isoformat()
            
            # 持久化保存
            self.save_features()
            self.save_metadata()
            
            return True
            
        except Exception as e:
            print(f"更新人员失败: {e}")
            return False
    
    def remove_person(self, person_id: str) -> bool:
        """删除人员"""
        try:
            if person_id in self.features:
                del self.features[person_id]
            
            if person_id in self.metadata:
                del self.metadata[person_id]
            
            # 持久化保存
            self.save_features()
            self.save_metadata()
            
            return True
            
        except Exception as e:
            print(f"删除人员失败: {e}")
            return False
    
    def get_person(self, person_id: str) -> Optional[Dict[str, Any]]:
        """获取人员信息"""
        if person_id in self.metadata:
            person_info = self.metadata[person_id].copy()
            person_info['has_features'] = person_id in self.features
            return person_info
        
        return None
    
    def get_all_persons(self) -> List[Dict[str, Any]]:
        """获取所有人员信息"""
        persons = []
        
        for person_id, metadata in self.metadata.items():
            person_info = metadata.copy()
            person_info['has_features'] = person_id in self.features
            persons.append(person_info)
        
        return persons
    
    def get_features(self, person_id: str) -> Optional[np.ndarray]:
        """获取人员特征"""
        return self.features.get(person_id)
    
    def get_all_features(self) -> Dict[str, np.ndarray]:
        """获取所有特征"""
        return self.features.copy()
    
    def search_by_name(self, name: str) -> List[Dict[str, Any]]:
        """按姓名搜索"""
        results = []
        
        for person_id, metadata in self.metadata.items():
            if name.lower() in metadata['name'].lower():
                person_info = metadata.copy()
                person_info['has_features'] = person_id in self.features
                results.append(person_info)
        
        return results
    
    def get_statistics(self) -> Dict[str, Any]:
        """获取数据库统计信息"""
        total_persons = len(self.metadata)
        persons_with_features = len(self.features)
        
        # 计算特征维度分布
        feature_dims = {}
        for person_id, features in self.features.items():
            dim = len(features)
            feature_dims[dim] = feature_dims.get(dim, 0) + 1
        
        # 计算创建时间分布
        creation_dates = []
        for metadata in self.metadata.values():
            creation_dates.append(metadata['created_at'])
        
        stats = {
            'total_persons': total_persons,
            'persons_with_features': persons_with_features,
            'feature_dimensions': feature_dims,
            'database_size_mb': self.get_database_size(),
            'creation_dates': sorted(creation_dates)
        }
        
        return stats
    
    def get_database_size(self) -> float:
        """获取数据库大小（MB）"""
        total_size = 0
        
        if os.path.exists(self.features_file):
            total_size += os.path.getsize(self.features_file)
        
        if os.path.exists(self.metadata_file):
            total_size += os.path.getsize(self.metadata_file)
        
        return total_size / (1024 * 1024)
    
    def backup_database(self, backup_path: str) -> bool:
        """备份数据库"""
        try:
            import shutil
            
            # 创建备份目录
            os.makedirs(backup_path, exist_ok=True)
            
            # 复制文件
            if os.path.exists(self.features_file):
                shutil.copy2(self.features_file, backup_path)
            
            if os.path.exists(self.metadata_file):
                shutil.copy2(self.metadata_file, backup_path)
            
            # 创建备份信息文件
            backup_info = {
                'backup_time': datetime.now().isoformat(),
                'original_path': self.database_path,
                'total_persons': len(self.metadata),
                'persons_with_features': len(self.features)
            }
            
            backup_info_file = os.path.join(backup_path, 'backup_info.json')
            with open(backup_info_file, 'w', encoding='utf-8') as f:
                json.dump(backup_info, f, indent=2, ensure_ascii=False)
            
            return True
            
        except Exception as e:
            print(f"备份数据库失败: {e}")
            return False
    
    def restore_database(self, backup_path: str) -> bool:
        """恢复数据库"""
        try:
            import shutil
            
            # 检查备份文件
            backup_features = os.path.join(backup_path, 'features.pkl')
            backup_metadata = os.path.join(backup_path, 'metadata.json')
            
            if not os.path.exists(backup_features) or not os.path.exists(backup_metadata):
                print("备份文件不完整")
                return False
            
            # 恢复文件
            shutil.copy2(backup_features, self.features_file)
            shutil.copy2(backup_metadata, self.metadata_file)
            
            # 重新加载数据
            self.features = self.load_features()
            self.metadata = self.load_metadata()
            
            return True
            
        except Exception as e:
            print(f"恢复数据库失败: {e}")
            return False
```

### 4. 完整人脸识别系统

```python
import cv2
import numpy as np
import time
from typing import List, Dict, Any, Optional, Tuple
from src.face_detector import MTCNNDetector, RetinaFaceDetector
from src.face_recognizer import FaceRecognizer
from src.face_database import FaceDatabase

class FaceRecognitionSystem:
    """完整的人脸识别系统"""
    
    def __init__(self, detector_model: str, recognizer_model: str, 
                 database_path: str = 'data/face_database', device: str = 'cpu'):
        
        # 初始化组件
        self.detector = self.create_detector(detector_model, device)
        self.recognizer = FaceRecognizer(recognizer_model, device)
        self.database = FaceDatabase(database_path)
        
        # 系统参数
        self.min_face_size = 50
        self.max_faces_per_image = 10
        self.recognition_threshold = 0.6
        
        # 性能统计
        self.stats = {
            'total_detections': 0,
            'total_recognitions': 0,
            'successful_recognitions': 0,
            'average_detection_time': 0,
            'average_recognition_time': 0
        }
    
    def create_detector(self, model_path: str, device: str):
        """创建人脸检测器"""
        if 'mtcnn' in model_path.lower():
            return MTCNNDetector(model_path, device)
        elif 'retinaface' in model_path.lower():
            return RetinaFaceDetector(model_path, device)
        else:
            # 默认使用MTCNN
            return MTCNNDetector(model_path, device)
    
    def initialize(self) -> bool:
        """初始化系统"""
        try:
            # 加载模型
            if not self.detector.load_model():
                print("人脸检测模型加载失败")
                return False
            
            if not self.recognizer.load_model():
                print("人脸识别模型加载失败")
                return False
            
            print("人脸识别系统初始化成功")
            return True
            
        except Exception as e:
            print(f"系统初始化失败: {e}")
            return False
    
    def detect_and_extract_faces(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """检测并提取人脸"""
        # 检测人脸
        start_time = time.time()
        detections = self.detector.detect_faces(image)
        detection_time = (time.time() - start_time) * 1000
        
        # 更新统计
        self.stats['total_detections'] += len(detections)
        self.stats['average_detection_time'] = (
            (self.stats['average_detection_time'] * (self.stats['total_detections'] - len(detections)) + 
             detection_time) / self.stats['total_detections']
        )
        
        # 提取人脸区域和特征
        faces = []
        for detection in detections[:self.max_faces_per_image]:
            # 提取人脸区域
            bbox = detection['bbox']
            x1, y1, x2, y2 = bbox
            
            # 扩展边界框
            margin = 20
            x1 = max(0, x1 - margin)
            y1 = max(0, y1 - margin)
            x2 = min(image.shape[1], x2 + margin)
            y2 = min(image.shape[0], y2 + margin)
            
            face_image = image[y1:y2, x1:x2]
            
            # 检查人脸大小
            if face_image.shape[0] < self.min_face_size or face_image.shape[1] < self.min_face_size:
                continue
            
            # 人脸对齐（如果有关键点）
            if 'landmarks' in detection:
                aligned_face = self.recognizer.align_face(face_image, detection['landmarks'])
            else:
                aligned_face = face_image
            
            # 提取特征
            start_time = time.time()
            features = self.recognizer.extract_features(aligned_face)
            recognition_time = (time.time() - start_time) * 1000
            
            # 更新统计
            self.stats['total_recognitions'] += 1
            self.stats['average_recognition_time'] = (
                (self.stats['average_recognition_time'] * (self.stats['total_recognitions'] - 1) + 
                 recognition_time) / self.stats['total_recognitions']
            )
            
            face_info = {
                'bbox': [x1, y1, x2, y2],
                'confidence': detection['confidence'],
                'face_image': aligned_face,
                'features': features,
                'landmarks': detection.get('landmarks'),
                'detection_time_ms': detection_time / len(detections),
                'recognition_time_ms': recognition_time
            }
            
            faces.append(face_info)
        
        return faces
    
    def register_face(self, image: np.ndarray, person_id: str, name: str, 
                     additional_info: Dict[str, Any] = None) -> bool:
        """注册新人脸"""
        try:
            # 检测和提取人脸
            faces = self.detect_and_extract_faces(image)
            
            if len(faces) == 0:
                print("未检测到人脸")
                return False
            
            if len(faces) > 1:
                print("检测到多张人脸，请确保图像中只有一张人脸")
                return False
            
            # 获取人脸特征
            face_features = faces[0]['features']
            
            # 检查是否已存在相似人脸
            existing_features = self.database.get_all_features()
            for existing_id, existing_feature in existing_features.items():
                similarity = self.recognizer.compare_faces(face_features, existing_feature)
                if similarity > 0.8:  # 高相似度阈值
                    existing_person = self.database.get_person(existing_id)
                    print(f"检测到相似人脸，已存在人员: {existing_person['name']} (相似度: {similarity:.3f})")
                    return False
            
            # 添加到数据库
            success = self.database.add_person(person_id, name, face_features, additional_info)
            
            if success:
                print(f"成功注册人脸: {name} ({person_id})")
                
                # 保存人脸图像
                face_dir = os.path.join(self.database.database_path, 'face_images')
                os.makedirs(face_dir, exist_ok=True)
                
                face_image_path = os.path.join(face_dir, f"{person_id}.jpg")
                cv2.imwrite(face_image_path, faces[0]['face_image'])
            
            return success
            
        except Exception as e:
            print(f"注册人脸失败: {e}")
            return False
    
    def recognize_faces(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """识别图像中的人脸"""
        try:
            # 检测和提取人脸
            faces = self.detect_and_extract_faces(image)
            
            if len(faces) == 0:
                return []
            
            # 获取数据库中的所有特征
            known_features = self.database.get_all_features()
            
            if len(known_features) == 0:
                print("人脸数据库为空，请先注册人脸")
                return []
            
            # 识别每张人脸
            results = []
            for face in faces:
                face_features = face['features']
                
                # 执行识别
                recognition_result = self.recognizer.identify_face(face_features, known_features)
                
                # 获取人员信息
                if recognition_result['person_id'] != 'unknown':
                    person_info = self.database.get_person(recognition_result['person_id'])
                    name = person_info['name'] if person_info else 'Unknown'
                    
                    # 更新成功识别统计
                    self.stats['successful_recognitions'] += 1
                else:
                    name = 'Unknown'
                
                result = {
                    'bbox': face['bbox'],
                    'person_id': recognition_result['person_id'],
                    'name': name,
                    'confidence': recognition_result['confidence'],
                    'similarity': recognition_result['similarity'],
                    'detection_confidence': face['confidence'],
                    'detection_time_ms': face['detection_time_ms'],
                    'recognition_time_ms': face['recognition_time_ms']
                }
                
                results.append(result)
            
            return results
            
        except Exception as e:
            print(f"人脸识别失败: {e}")
            return []
    
    def verify_face(self, image: np.ndarray, person_id: str) -> Dict[str, Any]:
        """验证特定人员的人脸"""
        try:
            # 获取参考特征
            reference_features = self.database.get_features(person_id)
            if reference_features is None:
                return {
                    'success': False,
                    'error': f'人员 {person_id} 不存在'
                }
            
            # 检测和提取人脸
            faces = self.detect_and_extract_faces(image)
            
            if len(faces) == 0:
                return {
                    'success': False,
                    'error': '未检测到人脸'
                }
            
            if len(faces) > 1:
                return {
                    'success': False,
                    'error': '检测到多张人脸'
                }
            
            # 执行验证
            face_features = faces[0]['features']
            verification_result = self.recognizer.verify_face(face_features, reference_features)
            
            # 获取人员信息
            person_info = self.database.get_person(person_id)
            
            result = {
                'success': True,
                'person_id': person_id,
                'name': person_info['name'] if person_info else 'Unknown',
                'is_match': verification_result['is_match'],
                'similarity': verification_result['similarity'],
                'confidence': verification_result['confidence'],
                'bbox': faces[0]['bbox'],
                'detection_time_ms': faces[0]['detection_time_ms'],
                'recognition_time_ms': faces[0]['recognition_time_ms']
            }
            
            return result
            
        except Exception as e:
            print(f"人脸验证失败: {e}")
            return {
                'success': False,
                'error': str(e)
            }
    
    def get_system_stats(self) -> Dict[str, Any]:
        """获取系统统计信息"""
        db_stats = self.database.get_statistics()
        
        # 计算识别成功率
        success_rate = 0.0
        if self.stats['total_recognitions'] > 0:
            success_rate = self.stats['successful_recognitions'] / self.stats['total_recognitions']
        
        system_stats = {
            'performance': {
                'total_detections': self.stats['total_detections'],
                'total_recognitions': self.stats['total_recognitions'],
                'successful_recognitions': self.stats['successful_recognitions'],
                'success_rate': success_rate,
                'average_detection_time_ms': self.stats['average_detection_time'],
                'average_recognition_time_ms': self.stats['average_recognition_time']
            },
            'database': db_stats,
            'system_config': {
                'min_face_size': self.min_face_size,
                'max_faces_per_image': self.max_faces_per_image,
                'recognition_threshold': self.recognition_threshold,
                'detector_type': type(self.detector).__name__,
                'recognizer_model': self.recognizer.model_path
            }
        }
        
        return system_stats
    
    def reset_stats(self):
        """重置统计信息"""
        self.stats = {
            'total_detections': 0,
            'total_recognitions': 0,
            'successful_recognitions': 0,
            'average_detection_time': 0,
            'average_recognition_time': 0
        }
```

## 实时人脸识别示例

```python
import cv2
import argparse
import time
import numpy as np
from collections import deque
from src.face_system import FaceRecognitionSystem

class RealTimeFaceRecognition:
    """实时人脸识别系统"""
    
    def __init__(self, detector_model: str, recognizer_model: str, 
                 database_path: str, device: str = 'cpu'):
        
        # 初始化人脸识别系统
        self.face_system = FaceRecognitionSystem(
            detector_model, recognizer_model, database_path, device
        )
        
        # 性能统计
        self.fps_history = deque(maxlen=30)
        self.recognition_history = deque(maxlen=100)
        
        # 显示参数
        self.show_landmarks = False
        self.show_confidence = True
        self.show_stats = True
        
    def run(self, camera_id: int = 0, display_size: tuple = (640, 480)):
        """运行实时人脸识别"""
        
        # 初始化系统
        if not self.face_system.initialize():
            print("系统初始化失败")
            return
        
        # 初始化摄像头
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, display_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, display_size[1])
        
        if not cap.isOpened():
            print("无法打开摄像头")
            return
        
        print("实时人脸识别已启动")
        print("按键说明:")
        print("  'q' - 退出")
        print("  's' - 保存当前帧")
        print("  'l' - 切换关键点显示")
        print("  'c' - 切换置信度显示")
        print("  'i' - 显示系统信息")
        print("  'r' - 重置统计信息")
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # 执行人脸识别
            try:
                results = self.face_system.recognize_faces(frame)
                
                # 更新统计
                self.recognition_history.append(len(results))
                
                # 绘制结果
                self.draw_results(frame, results)
                
            except Exception as e:
                print(f"识别错误: {e}")
                results = []
            
            # 计算FPS
            frame_count += 1
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            if elapsed_time >= 1.0:
                fps = frame_count / elapsed_time
                self.fps_history.append(fps)
                frame_count = 0
                start_time = current_time
            
            # 绘制界面元素
            if self.show_stats:
                self.draw_performance_info(frame)
            
            self.draw_control_info(frame)
            
            # 显示结果
            cv2.imshow('Real-time Face Recognition', frame)
            
            # 处理按键
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_frame(frame, results)
            elif key == ord('l'):
                self.show_landmarks = not self.show_landmarks
                print(f"关键点显示: {'开启' if self.show_landmarks else '关闭'}")
            elif key == ord('c'):
                self.show_confidence = not self.show_confidence
                print(f"置信度显示: {'开启' if self.show_confidence else '关闭'}")
            elif key == ord('i'):
                self.print_system_info()
            elif key == ord('r'):
                self.face_system.reset_stats()
                self.fps_history.clear()
                self.recognition_history.clear()
                print("统计信息已重置")
        
        # 清理资源
        cap.release()
        cv2.destroyAllWindows()
        
        # 打印最终统计
        self.print_final_statistics()
    
    def draw_results(self, frame: np.ndarray, results: List[Dict[str, Any]]):
        """绘制识别结果"""
        for result in results:
            bbox = result['bbox']
            name = result['name']
            confidence = result['confidence']
            
            x1, y1, x2, y2 = bbox
            
            # 选择颜色
            if name != 'Unknown':
                color = (0, 255, 0)  # 绿色 - 已知人员
            else:
                color = (0, 0, 255)  # 红色 - 未知人员
            
            # 绘制边界框
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # 准备标签文本
            label_parts = [name]
            if self.show_confidence:
                label_parts.append(f"{confidence:.3f}")
            
            label = " - ".join(label_parts)
            
            # 绘制标签背景
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
            cv2.rectangle(frame, (x1, y1 - 30), (x1 + label_size[0], y1), color, -1)
            
            # 绘制标签文本
            cv2.putText(frame, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    def draw_performance_info(self, frame: np.ndarray):
        """绘制性能信息"""
        height, width = frame.shape[:2]
        
        # 背景框
        cv2.rectangle(frame, (width - 250, 10), (width - 10, 150), (0, 0, 0), -1)
        cv2.rectangle(frame, (width - 250, 10), (width - 10, 150), (255, 255, 255), 2)
        
        y_offset = 30
        
        # FPS信息
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            fps_text = f"FPS: {avg_fps:.1f}"
            cv2.putText(frame, fps_text, (width - 240, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
        
        # 识别统计
        if self.recognition_history:
            total_faces = sum(self.recognition_history)
            avg_faces = total_faces / len(self.recognition_history)
            faces_text = f"Faces: {avg_faces:.1f}"
            cv2.putText(frame, faces_text, (width - 240, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            y_offset += 25
        
        # 系统统计
        stats = self.face_system.get_system_stats()
        success_rate = stats['performance']['success_rate']
        success_text = f"Success: {success_rate:.1%}"
        cv2.putText(frame, success_text, (width - 240, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        y_offset += 25
        
        # 数据库信息
        db_persons = stats['database']['total_persons']
        db_text = f"DB: {db_persons} persons"
        cv2.putText(frame, db_text, (width - 240, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    def draw_control_info(self, frame: np.ndarray):
        """绘制控制信息"""
        height, width = frame.shape[:2]
        
        # 背景框
        cv2.rectangle(frame, (10, height - 100), (250, height - 10), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, height - 100), (250, height - 10), (255, 255, 255), 2)
        
        # 控制说明
        controls = [
            "Q:Quit  S:Save  L:Landmarks",
            "C:Confidence  I:Info  R:Reset"
        ]
        
        y_offset = height - 80
        for control in controls:
            cv2.putText(frame, control, (15, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            y_offset += 20
    
    def save_frame(self, frame: np.ndarray, results: List[Dict[str, Any]]):
        """保存当前帧"""
        timestamp = int(time.time())
        filename = f'face_recognition_{timestamp}.jpg'
        
        # 保存图像
        cv2.imwrite(filename, frame)
        
        # 保存识别结果
        result_filename = f'face_recognition_{timestamp}.json'
        import json
        
        recognition_data = {
            'timestamp': timestamp,
            'filename': filename,
            'results': results,
            'system_stats': self.face_system.get_system_stats()
        }
        
        with open(result_filename, 'w', encoding='utf-8') as f:
            json.dump(recognition_data, f, indent=2, ensure_ascii=False)
        
        print(f"保存: {filename}, {result_filename}")
    
    def print_system_info(self):
        """打印系统信息"""
        stats = self.face_system.get_system_stats()
        
        print("\n=== 系统信息 ===")
        print(f"检测器类型: {stats['system_config']['detector_type']}")
        print(f"识别模型: {stats['system_config']['recognizer_model']}")
        print(f"数据库人员数: {stats['database']['total_persons']}")
        print(f"数据库大小: {stats['database']['database_size_mb']:.2f} MB")
        print(f"总检测次数: {stats['performance']['total_detections']}")
        print(f"总识别次数: {stats['performance']['total_recognitions']}")
        print(f"识别成功率: {stats['performance']['success_rate']:.1%}")
        print(f"平均检测时间: {stats['performance']['average_detection_time_ms']:.2f} ms")
        print(f"平均识别时间: {stats['performance']['average_recognition_time_ms']:.2f} ms")
    
    def print_final_statistics(self):
        """打印最终统计信息"""
        print("\n=== 最终统计 ===")
        
        if self.fps_history:
            avg_fps = sum(self.fps_history) / len(self.fps_history)
            print(f"平均FPS: {avg_fps:.2f}")
        
        if self.recognition_history:
            total_faces = sum(self.recognition_history)
            avg_faces = total_faces / len(self.recognition_history)
            print(f"平均检测人脸数: {avg_faces:.1f}")
        
        # 系统统计
        stats = self.face_system.get_system_stats()
        print(f"总处理帧数: {len(self.recognition_history)}")
        print(f"识别成功率: {stats['performance']['success_rate']:.1%}")

def main():
    parser = argparse.ArgumentParser(description='实时人脸识别')
    parser.add_argument('--detector', required=True, help='人脸检测模型路径')
    parser.add_argument('--recognizer', required=True, help='人脸识别模型路径')
    parser.add_argument('--database', default='data/face_database', help='人脸数据库路径')
    parser.add_argument('--device', default='cpu', choices=['cpu', 'gpu'], help='推理设备')
    parser.add_argument('--camera', type=int, default=0, help='摄像头ID')
    parser.add_argument('--size', nargs=2, type=int, default=[640, 480], help='显示尺寸')
    
    args = parser.parse_args()
    
    # 创建实时人脸识别系统
    recognition_system = RealTimeFaceRecognition(
        args.detector, args.recognizer, args.database, args.device
    )
    
    # 运行识别
    recognition_system.run(args.camera, tuple(args.size))

if __name__ == "__main__":
    main()
```

## 应用场景

- **门禁系统**: 办公楼、住宅区门禁控制
- **考勤系统**: 员工上下班考勤管理
- **安防监控**: 重要区域人员识别和预警
- **移动支付**: 人脸支付验证
- **智能家居**: 家庭成员识别和个性化服务

## 性能优化

### 1. 检测优化
- **多尺度检测**: 提高不同大小人脸的检测率
- **ROI检测**: 在感兴趣区域内检测以提高速度
- **帧间跟踪**: 结合目标跟踪减少重复检测

### 2. 识别优化
- **特征缓存**: 缓存已提取的特征向量
- **批量处理**: 同时处理多张人脸
- **模型量化**: 使用INT8量化模型

### 3. 数据库优化
- **索引优化**: 建立特征向量索引加速搜索
- **分层搜索**: 先粗筛选再精确匹配
- **内存管理**: 合理管理内存使用

## 安全考虑

### 1. 活体检测
- **眨眼检测**: 检测眼部动作
- **头部运动**: 检测头部转动
- **纹理分析**: 分析皮肤纹理真实性

### 2. 隐私保护
- **本地处理**: 所有数据本地处理
- **特征加密**: 加密存储人脸特征
- **访问控制**: 严格的数据访问权限

### 3. 防欺骗攻击
- **多模态验证**: 结合多种生物特征
- **时间戳验证**: 检测重放攻击
- **环境感知**: 检测异常环境条件

## 测试和验证

### 1. 功能测试
```bash
python tests/test_detector.py
python tests/test_recognizer.py
python tests/test_system.py
```

### 2. 性能测试
```bash
python tests/test_performance.py --dataset data/test_dataset
```

### 3. 准确率测试
```bash
python tests/test_accuracy.py --database data/face_database --test_images data/test_images
```

## 部署指南

### 1. 边缘设备部署
- **树莓派**: 使用轻量级模型和CPU推理
- **Jetson Nano**: 利用GPU加速提升性能
- **移动设备**: 使用移动端优化模型

### 2. 生产环境部署
- **负载均衡**: 多实例部署处理高并发
- **监控告警**: 实时监控系统状态
- **数据备份**: 定期备份人脸数据库

## 常见问题

### Q: 如何提高识别准确率？
A: 
1. 使用高质量的训练数据
2. 增加人脸样本的多样性
3. 调整识别阈值参数
4. 使用更先进的识别模型

### Q: 如何处理光照变化？
A: 
1. 使用光照归一化预处理
2. 增加不同光照条件的训练样本
3. 使用对光照鲁棒的特征提取方法

### Q: 如何防止照片欺骗？
A: 
1. 集成活体检测功能
2. 使用3D人脸识别技术
3. 结合多种生物特征验证

## 扩展功能

- **情绪识别**: 基于人脸表情识别情绪状态
- **年龄估计**: 估计人员年龄范围
- **性别识别**: 识别人员性别
- **人脸美化**: 实时人脸美化处理
- **人脸替换**: 实现人脸替换效果

## 贡献指南

欢迎提交问题报告、功能请求和代码贡献。请遵循以下规范：

1. 提交前运行所有测试用例
2. 遵循代码风格规范
3. 添加必要的文档说明
4. 确保向后兼容性

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 相关资源

- [MTCNN论文](https://arxiv.org/abs/1604.02878)
- [FaceNet论文](https://arxiv.org/abs/1503.03832)
- [ArcFace论文](https://arxiv.org/abs/1801.07698)
- [RetinaFace论文](https://arxiv.org/abs/1905.00641)
# 异常检测示例

## 概述

本示例展示了如何在边缘设备上实现高效的异常检测系统，包括时序数据异常检测、图像异常检测和多模态异常检测。支持实时监控、离线分析和自适应学习。

## 功能特性

- **多类型异常检测**: 支持时序数据、图像、音频等多种数据类型
- **实时监控**: 实时数据流异常检测和告警
- **自适应学习**: 在线学习和模型更新
- **多种算法**: 统计方法、机器学习和深度学习算法
- **可视化分析**: 丰富的异常检测结果可视化
- **告警系统**: 多级别异常告警和通知

## 项目结构

```
anomaly-detection/
├── README.md                    # 本文件
├── requirements.txt             # Python依赖
├── models/                      # 预训练模型
│   ├── autoencoder/            # 自编码器模型
│   │   ├── time_series_ae.tflite
│   │   └── image_ae.onnx
│   ├── isolation_forest/       # 孤立森林模型
│   │   └── isolation_forest.pkl
│   └── one_class_svm/          # 单类SVM模型
│       └── one_class_svm.pkl
├── src/                         # 源代码
│   ├── __init__.py
│   ├── base_detector.py        # 异常检测器基类
│   ├── time_series_detector.py # 时序异常检测
│   ├── image_detector.py       # 图像异常检测
│   ├── multimodal_detector.py  # 多模态异常检测
│   ├── online_learner.py       # 在线学习器
│   ├── alert_system.py         # 告警系统
│   └── utils.py                # 工具函数
├── examples/                    # 使用示例
│   ├── basic_detection.py      # 基础异常检测
│   ├── realtime_monitoring.py  # 实时监控
│   ├── industrial_monitoring.py # 工业监控
│   ├── network_monitoring.py   # 网络监控
│   └── iot_monitoring.py       # IoT设备监控
├── data/                        # 数据目录
│   ├── time_series/            # 时序数据
│   ├── images/                 # 图像数据
│   ├── sensor_data/            # 传感器数据
│   └── test_data/              # 测试数据
├── configs/                     # 配置文件
│   ├── detection_config.yaml   # 检测配置
│   ├── alert_config.yaml       # 告警配置
│   └── model_config.yaml       # 模型配置
└── tests/                       # 测试用例
    ├── test_detectors.py       # 检测器测试
    ├── test_online_learning.py # 在线学习测试
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
from src.time_series_detector import TimeSeriesAnomalyDetector

# 创建时序异常检测器
detector = TimeSeriesAnomalyDetector(
    method='autoencoder',
    model_path='models/autoencoder/time_series_ae.tflite',
    threshold=0.1
)

# 训练或加载模型
detector.fit(normal_data)

# 检测异常
import numpy as np
test_data = np.random.random((100, 10))  # 100个时间步，10个特征
anomalies = detector.detect(test_data)

print(f"检测到 {len(anomalies)} 个异常点")
for anomaly in anomalies:
    print(f"时间步 {anomaly['timestamp']}: 异常分数 {anomaly['score']:.3f}")
```

### 3. 实时监控

```bash
python examples/realtime_monitoring.py --config configs/detection_config.yaml
```

### 4. 工业监控示例

```bash
python examples/industrial_monitoring.py --sensors temperature,pressure,vibration --threshold 0.05
```

## 核心组件

### 1. 异常检测器基类

```python
import numpy as np
import time
from abc import ABC, abstractmethod
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

class AnomalyType(Enum):
    """异常类型"""
    POINT = "point"          # 点异常
    CONTEXTUAL = "contextual"  # 上下文异常
    COLLECTIVE = "collective"  # 集体异常

@dataclass
class AnomalyResult:
    """异常检测结果"""
    timestamp: float
    score: float
    is_anomaly: bool
    anomaly_type: AnomalyType
    confidence: float
    details: Dict[str, Any]

class BaseAnomalyDetector(ABC):
    """异常检测器基类"""
    
    def __init__(self, threshold: float = 0.1, window_size: int = 100):
        self.threshold = threshold
        self.window_size = window_size
        self.is_fitted = False
        
        # 统计信息
        self.stats = {
            'total_samples': 0,
            'anomaly_count': 0,
            'false_positives': 0,
            'false_negatives': 0,
            'processing_times': []
        }
        
        # 历史数据缓存
        self.history_buffer = []
        self.max_history = 1000
    
    @abstractmethod
    def fit(self, X: np.ndarray, y: Optional[np.ndarray] = None) -> 'BaseAnomalyDetector':
        """训练模型"""
        pass
    
    @abstractmethod
    def predict_scores(self, X: np.ndarray) -> np.ndarray:
        """预测异常分数"""
        pass
    
    def detect(self, X: np.ndarray) -> List[AnomalyResult]:
        """检测异常"""
        if not self.is_fitted:
            raise RuntimeError("模型未训练，请先调用fit方法")
        
        start_time = time.time()
        
        # 预测异常分数
        scores = self.predict_scores(X)
        
        # 判断异常
        anomalies = []
        for i, score in enumerate(scores):
            is_anomaly = score > self.threshold
            
            if is_anomaly:
                anomaly_type = self.classify_anomaly_type(X[i], i, X)
                confidence = min(1.0, score / self.threshold)
                
                anomaly = AnomalyResult(
                    timestamp=time.time() + i * 0.001,  # 假设1ms间隔
                    score=float(score),
                    is_anomaly=True,
                    anomaly_type=anomaly_type,
                    confidence=confidence,
                    details=self.get_anomaly_details(X[i], score)
                )
                
                anomalies.append(anomaly)
        
        # 更新统计信息
        processing_time = (time.time() - start_time) * 1000
        self.update_stats(len(X), len(anomalies), processing_time)
        
        # 更新历史缓存
        self.update_history(X, scores)
        
        return anomalies
    
    def classify_anomaly_type(self, sample: np.ndarray, index: int, 
                            context: np.ndarray) -> AnomalyType:
        """分类异常类型"""
        # 简单的异常类型分类逻辑
        # 实际应用中可以根据具体需求实现更复杂的分类
        
        if index == 0 or index == len(context) - 1:
            return AnomalyType.POINT
        
        # 检查上下文异常
        window_start = max(0, index - 5)
        window_end = min(len(context), index + 6)
        window_data = context[window_start:window_end]
        
        if len(window_data) > 1:
            window_std = np.std(window_data, axis=0)
            sample_deviation = np.abs(sample - np.mean(window_data, axis=0))
            
            if np.any(sample_deviation > 2 * window_std):
                return AnomalyType.CONTEXTUAL
        
        return AnomalyType.POINT
    
    def get_anomaly_details(self, sample: np.ndarray, score: float) -> Dict[str, Any]:
        """获取异常详细信息"""
        return {
            'sample_shape': sample.shape,
            'sample_mean': float(np.mean(sample)),
            'sample_std': float(np.std(sample)),
            'sample_min': float(np.min(sample)),
            'sample_max': float(np.max(sample)),
            'anomaly_score': float(score)
        }
    
    def update_stats(self, sample_count: int, anomaly_count: int, processing_time: float):
        """更新统计信息"""
        self.stats['total_samples'] += sample_count
        self.stats['anomaly_count'] += anomaly_count
        self.stats['processing_times'].append(processing_time)
        
        # 保持处理时间历史在合理范围内
        if len(self.stats['processing_times']) > 1000:
            self.stats['processing_times'] = self.stats['processing_times'][-1000:]
    
    def update_history(self, X: np.ndarray, scores: np.ndarray):
        """更新历史数据"""
        for i, (sample, score) in enumerate(zip(X, scores)):
            self.history_buffer.append({
                'timestamp': time.time() + i * 0.001,
                'sample': sample.copy(),
                'score': score
            })
        
        # 限制历史缓存大小
        if len(self.history_buffer) > self.max_history:
            self.history_buffer = self.history_buffer[-self.max_history:]
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        anomaly_rate = 0.0
        if self.stats['total_samples'] > 0:
            anomaly_rate = self.stats['anomaly_count'] / self.stats['total_samples']
        
        avg_processing_time = 0.0
        if self.stats['processing_times']:
            avg_processing_time = np.mean(self.stats['processing_times'])
        
        return {
            'total_samples': self.stats['total_samples'],
            'anomaly_count': self.stats['anomaly_count'],
            'anomaly_rate': anomaly_rate,
            'false_positives': self.stats['false_positives'],
            'false_negatives': self.stats['false_negatives'],
            'average_processing_time_ms': avg_processing_time,
            'threshold': self.threshold,
            'is_fitted': self.is_fitted
        }
    
    def update_threshold(self, new_threshold: float):
        """更新异常阈值"""
        self.threshold = max(0.0, min(1.0, new_threshold))
    
    def get_recent_history(self, n: int = 100) -> List[Dict[str, Any]]:
        """获取最近的历史数据"""
        return self.history_buffer[-n:]
```

### 2. 时序异常检测器

```python
import numpy as np
import tensorflow as tf
from sklearn.ensemble import IsolationForest
from sklearn.svm import OneClassSVM
from sklearn.preprocessing import StandardScaler
import joblib

class TimeSeriesAnomalyDetector(BaseAnomalyDetector):
    """时序异常检测器"""
    
    def __init__(self, method: str = 'autoencoder', model_path: str = None, 
                 threshold: float = 0.1, window_size: int = 100):
        super().__init__(threshold, window_size)
        
        self.method = method
        self.model_path = model_path
        self.model = None
        self.scaler = StandardScaler()
        
        # 方法特定参数
        if method == 'isolation_forest':
            self.contamination = 0.1
        elif method == 'one_class_svm':
            self.nu = 0.1
        elif method == 'autoencoder':
            self.reconstruction_threshold = threshold
    
    def fit(self, X: np.ndarray, y: Optional[np.ndarray] = None) -> 'TimeSeriesAnomalyDetector':
        """训练时序异常检测模型"""
        try:
            # 数据预处理
            X_scaled = self.scaler.fit_transform(X)
            
            if self.method == 'isolation_forest':
                self.model = IsolationForest(
                    contamination=self.contamination,
                    random_state=42,
                    n_jobs=-1
                )
                self.model.fit(X_scaled)
                
            elif self.method == 'one_class_svm':
                self.model = OneClassSVM(
                    nu=self.nu,
                    kernel='rbf',
                    gamma='scale'
                )
                self.model.fit(X_scaled)
                
            elif self.method == 'autoencoder':
                if self.model_path and self.model_path.endswith('.tflite'):
                    # 加载预训练的TensorFlow Lite模型
                    self.model = tf.lite.Interpreter(model_path=self.model_path)
                    self.model.allocate_tensors()
                    self.input_details = self.model.get_input_details()
                    self.output_details = self.model.get_output_details()
                else:
                    # 创建简单的自编码器
                    self.model = self.create_autoencoder(X_scaled.shape[1])
                    self.train_autoencoder(X_scaled)
            
            elif self.method == 'statistical':
                # 统计方法：基于Z-score
                self.mean_ = np.mean(X_scaled, axis=0)
                self.std_ = np.std(X_scaled, axis=0)
                self.model = 'statistical'  # 标记
            
            else:
                raise ValueError(f"不支持的方法: {self.method}")
            
            self.is_fitted = True
            print(f"时序异常检测模型训练完成 (方法: {self.method})")
            
            return self
            
        except Exception as e:
            print(f"模型训练失败: {e}")
            raise
    
    def create_autoencoder(self, input_dim: int) -> tf.keras.Model:
        """创建自编码器模型"""
        # 编码器
        encoder_input = tf.keras.Input(shape=(input_dim,))
        encoded = tf.keras.layers.Dense(64, activation='relu')(encoder_input)
        encoded = tf.keras.layers.Dense(32, activation='relu')(encoded)
        encoded = tf.keras.layers.Dense(16, activation='relu')(encoded)
        
        # 解码器
        decoded = tf.keras.layers.Dense(32, activation='relu')(encoded)
        decoded = tf.keras.layers.Dense(64, activation='relu')(decoded)
        decoded = tf.keras.layers.Dense(input_dim, activation='linear')(decoded)
        
        # 自编码器
        autoencoder = tf.keras.Model(encoder_input, decoded)
        autoencoder.compile(optimizer='adam', loss='mse')
        
        return autoencoder
    
    def train_autoencoder(self, X: np.ndarray, epochs: int = 100):
        """训练自编码器"""
        self.model.fit(
            X, X,
            epochs=epochs,
            batch_size=32,
            validation_split=0.2,
            verbose=0,
            shuffle=True
        )
    
    def predict_scores(self, X: np.ndarray) -> np.ndarray:
        """预测异常分数"""
        if not self.is_fitted:
            raise RuntimeError("模型未训练")
        
        # 数据预处理
        X_scaled = self.scaler.transform(X)
        
        if self.method == 'isolation_forest':
            # 孤立森林：异常分数为负值，转换为正值
            scores = -self.model.decision_function(X_scaled)
            # 归一化到[0,1]
            scores = (scores - scores.min()) / (scores.max() - scores.min() + 1e-8)
            
        elif self.method == 'one_class_svm':
            # 单类SVM：异常分数为负值，转换为正值
            scores = -self.model.decision_function(X_scaled)
            # 归一化到[0,1]
            scores = (scores - scores.min()) / (scores.max() - scores.min() + 1e-8)
            
        elif self.method == 'autoencoder':
            if isinstance(self.model, tf.lite.Interpreter):
                # TensorFlow Lite推理
                scores = []
                for sample in X_scaled:
                    sample_input = np.expand_dims(sample, axis=0).astype(np.float32)
                    self.model.set_tensor(self.input_details[0]['index'], sample_input)
                    self.model.invoke()
                    reconstruction = self.model.get_tensor(self.output_details[0]['index'])
                    
                    # 计算重构误差
                    mse = np.mean((sample - reconstruction[0]) ** 2)
                    scores.append(mse)
                
                scores = np.array(scores)
            else:
                # 标准TensorFlow模型
                reconstructions = self.model.predict(X_scaled, verbose=0)
                # 计算重构误差
                scores = np.mean((X_scaled - reconstructions) ** 2, axis=1)
            
        elif self.method == 'statistical':
            # 统计方法：Z-score
            z_scores = np.abs((X_scaled - self.mean_) / (self.std_ + 1e-8))
            scores = np.max(z_scores, axis=1)  # 取最大Z-score作为异常分数
            
        else:
            raise ValueError(f"不支持的方法: {self.method}")
        
        return scores
    
    def detect_online(self, sample: np.ndarray) -> Optional[AnomalyResult]:
        """在线异常检测"""
        if not self.is_fitted:
            return None
        
        # 单样本检测
        sample_2d = sample.reshape(1, -1)
        scores = self.predict_scores(sample_2d)
        score = scores[0]
        
        if score > self.threshold:
            return AnomalyResult(
                timestamp=time.time(),
                score=float(score),
                is_anomaly=True,
                anomaly_type=AnomalyType.POINT,
                confidence=min(1.0, score / self.threshold),
                details=self.get_anomaly_details(sample, score)
            )
        
        return None
    
    def update_model(self, X_new: np.ndarray, learning_rate: float = 0.1):
        """在线更新模型"""
        if not self.is_fitted:
            return
        
        try:
            if self.method == 'statistical':
                # 更新统计参数
                X_new_scaled = self.scaler.transform(X_new)
                
                # 指数移动平均更新
                self.mean_ = (1 - learning_rate) * self.mean_ + learning_rate * np.mean(X_new_scaled, axis=0)
                self.std_ = (1 - learning_rate) * self.std_ + learning_rate * np.std(X_new_scaled, axis=0)
                
            elif self.method == 'autoencoder' and not isinstance(self.model, tf.lite.Interpreter):
                # 在线训练自编码器
                X_new_scaled = self.scaler.transform(X_new)
                self.model.fit(X_new_scaled, X_new_scaled, epochs=1, verbose=0)
            
            # 其他方法暂不支持在线更新
            
        except Exception as e:
            print(f"模型更新失败: {e}")
    
    def save_model(self, path: str):
        """保存模型"""
        try:
            model_data = {
                'method': self.method,
                'threshold': self.threshold,
                'scaler': self.scaler,
                'is_fitted': self.is_fitted
            }
            
            if self.method in ['isolation_forest', 'one_class_svm']:
                model_data['model'] = self.model
            elif self.method == 'statistical':
                model_data['mean_'] = self.mean_
                model_data['std_'] = self.std_
            elif self.method == 'autoencoder' and not isinstance(self.model, tf.lite.Interpreter):
                self.model.save(path + '_autoencoder.h5')
                model_data['autoencoder_path'] = path + '_autoencoder.h5'
            
            joblib.dump(model_data, path)
            print(f"模型已保存到: {path}")
            
        except Exception as e:
            print(f"模型保存失败: {e}")
    
    def load_model(self, path: str):
        """加载模型"""
        try:
            model_data = joblib.load(path)
            
            self.method = model_data['method']
            self.threshold = model_data['threshold']
            self.scaler = model_data['scaler']
            self.is_fitted = model_data['is_fitted']
            
            if self.method in ['isolation_forest', 'one_class_svm']:
                self.model = model_data['model']
            elif self.method == 'statistical':
                self.mean_ = model_data['mean_']
                self.std_ = model_data['std_']
            elif self.method == 'autoencoder' and 'autoencoder_path' in model_data:
                self.model = tf.keras.models.load_model(model_data['autoencoder_path'])
            
            print(f"模型已从 {path} 加载")
            
        except Exception as e:
            print(f"模型加载失败: {e}")
```

### 3. 图像异常检测器

```python
import cv2
import numpy as np
import tensorflow as tf
from typing import List, Dict, Any, Optional, Tuple

class ImageAnomalyDetector(BaseAnomalyDetector):
    """图像异常检测器"""
    
    def __init__(self, method: str = 'autoencoder', model_path: str = None,
                 threshold: float = 0.1, input_size: Tuple[int, int] = (224, 224)):
        super().__init__(threshold)
        
        self.method = method
        self.model_path = model_path
        self.input_size = input_size
        self.model = None
        
        # 图像预处理参数
        self.normalize = True
        self.augment = False
    
    def fit(self, X: np.ndarray, y: Optional[np.ndarray] = None) -> 'ImageAnomalyDetector':
        """训练图像异常检测模型"""
        try:
            # 预处理图像数据
            X_processed = self.preprocess_images(X)
            
            if self.method == 'autoencoder':
                if self.model_path and self.model_path.endswith('.tflite'):
                    # 加载预训练模型
                    self.model = tf.lite.Interpreter(model_path=self.model_path)
                    self.model.allocate_tensors()
                    self.input_details = self.model.get_input_details()
                    self.output_details = self.model.get_output_details()
                else:
                    # 创建卷积自编码器
                    self.model = self.create_conv_autoencoder()
                    self.train_autoencoder(X_processed)
            
            elif self.method == 'vae':
                # 变分自编码器
                self.model = self.create_vae()
                self.train_vae(X_processed)
            
            elif self.method == 'statistical':
                # 统计方法：基于像素统计
                self.pixel_mean = np.mean(X_processed, axis=0)
                self.pixel_std = np.std(X_processed, axis=0)
                self.model = 'statistical'
            
            else:
                raise ValueError(f"不支持的方法: {self.method}")
            
            self.is_fitted = True
            print(f"图像异常检测模型训练完成 (方法: {self.method})")
            
            return self
            
        except Exception as e:
            print(f"图像异常检测模型训练失败: {e}")
            raise
    
    def preprocess_images(self, images: np.ndarray) -> np.ndarray:
        """预处理图像"""
        processed_images = []
        
        for img in images:
            # 调整大小
            if len(img.shape) == 3:
                resized = cv2.resize(img, self.input_size)
            else:
                resized = cv2.resize(img, self.input_size)
                resized = np.expand_dims(resized, axis=-1)
            
            # 归一化
            if self.normalize:
                resized = resized.astype(np.float32) / 255.0
            
            processed_images.append(resized)
        
        return np.array(processed_images)
    
    def create_conv_autoencoder(self) -> tf.keras.Model:
        """创建卷积自编码器"""
        input_shape = (*self.input_size, 3)
        
        # 编码器
        encoder_input = tf.keras.Input(shape=input_shape)
        
        # 下采样
        x = tf.keras.layers.Conv2D(32, 3, activation='relu', padding='same')(encoder_input)
        x = tf.keras.layers.MaxPooling2D(2, padding='same')(x)
        x = tf.keras.layers.Conv2D(64, 3, activation='relu', padding='same')(x)
        x = tf.keras.layers.MaxPooling2D(2, padding='same')(x)
        x = tf.keras.layers.Conv2D(128, 3, activation='relu', padding='same')(x)
        encoded = tf.keras.layers.MaxPooling2D(2, padding='same')(x)
        
        # 解码器
        x = tf.keras.layers.Conv2D(128, 3, activation='relu', padding='same')(encoded)
        x = tf.keras.layers.UpSampling2D(2)(x)
        x = tf.keras.layers.Conv2D(64, 3, activation='relu', padding='same')(x)
        x = tf.keras.layers.UpSampling2D(2)(x)
        x = tf.keras.layers.Conv2D(32, 3, activation='relu', padding='same')(x)
        x = tf.keras.layers.UpSampling2D(2)(x)
        decoded = tf.keras.layers.Conv2D(3, 3, activation='sigmoid', padding='same')(x)
        
        # 自编码器
        autoencoder = tf.keras.Model(encoder_input, decoded)
        autoencoder.compile(optimizer='adam', loss='mse')
        
        return autoencoder
    
    def train_autoencoder(self, X: np.ndarray, epochs: int = 50):
        """训练卷积自编码器"""
        self.model.fit(
            X, X,
            epochs=epochs,
            batch_size=16,
            validation_split=0.2,
            verbose=1,
            shuffle=True
        )
    
    def predict_scores(self, X: np.ndarray) -> np.ndarray:
        """预测图像异常分数"""
        if not self.is_fitted:
            raise RuntimeError("模型未训练")
        
        # 预处理图像
        X_processed = self.preprocess_images(X)
        
        if self.method == 'autoencoder':
            if isinstance(self.model, tf.lite.Interpreter):
                # TensorFlow Lite推理
                scores = []
                for img in X_processed:
                    img_input = np.expand_dims(img, axis=0).astype(np.float32)
                    self.model.set_tensor(self.input_details[0]['index'], img_input)
                    self.model.invoke()
                    reconstruction = self.model.get_tensor(self.output_details[0]['index'])
                    
                    # 计算重构误差
                    mse = np.mean((img - reconstruction[0]) ** 2)
                    scores.append(mse)
                
                scores = np.array(scores)
            else:
                # 标准TensorFlow模型
                reconstructions = self.model.predict(X_processed, verbose=0)
                # 计算重构误差
                scores = np.mean((X_processed - reconstructions) ** 2, axis=(1, 2, 3))
        
        elif self.method == 'statistical':
            # 统计方法
            scores = []
            for img in X_processed:
                # 计算像素级Z-score
                z_scores = np.abs((img - self.pixel_mean) / (self.pixel_std + 1e-8))
                # 取平均Z-score作为异常分数
                score = np.mean(z_scores)
                scores.append(score)
            
            scores = np.array(scores)
        
        else:
            raise ValueError(f"不支持的方法: {self.method}")
        
        return scores
    
    def detect_regions(self, image: np.ndarray, patch_size: int = 64, 
                      stride: int = 32) -> List[Dict[str, Any]]:
        """检测图像中的异常区域"""
        if not self.is_fitted:
            raise RuntimeError("模型未训练")
        
        height, width = image.shape[:2]
        anomaly_regions = []
        
        # 滑动窗口检测
        for y in range(0, height - patch_size + 1, stride):
            for x in range(0, width - patch_size + 1, stride):
                # 提取图像块
                patch = image[y:y+patch_size, x:x+patch_size]
                
                # 检测异常
                patch_array = np.expand_dims(patch, axis=0)
                scores = self.predict_scores(patch_array)
                score = scores[0]
                
                if score > self.threshold:
                    anomaly_regions.append({
                        'bbox': [x, y, patch_size, patch_size],
                        'score': float(score),
                        'confidence': min(1.0, score / self.threshold)
                    })
        
        # 非极大值抑制
        if len(anomaly_regions) > 1:
            anomaly_regions = self.apply_nms(anomaly_regions)
        
        return anomaly_regions
    
    def apply_nms(self, regions: List[Dict[str, Any]], 
                  iou_threshold: float = 0.5) -> List[Dict[str, Any]]:
        """应用非极大值抑制"""
        if len(regions) <= 1:
            return regions
        
        # 按分数排序
        regions = sorted(regions, key=lambda x: x['score'], reverse=True)
        
        filtered_regions = []
        
        while regions:
            # 选择分数最高的区域
            current = regions.pop(0)
            filtered_regions.append(current)
            
            # 移除与当前区域重叠度高的区域
            remaining = []
            for region in regions:
                iou = self.calculate_iou(current['bbox'], region['bbox'])
                if iou < iou_threshold:
                    remaining.append(region)
            
            regions = remaining
        
        return filtered_regions
    
    def calculate_iou(self, bbox1: List[int], bbox2: List[int]) -> float:
        """计算两个边界框的IoU"""
        x1, y1, w1, h1 = bbox1
        x2, y2, w2, h2 = bbox2
        
        # 计算交集
        x_left = max(x1, x2)
        y_top = max(y1, y2)
        x_right = min(x1 + w1, x2 + w2)
        y_bottom = min(y1 + h1, y2 + h2)
        
        if x_right < x_left or y_bottom < y_top:
            return 0.0
        
        intersection = (x_right - x_left) * (y_bottom - y_top)
        
        # 计算并集
        area1 = w1 * h1
        area2 = w2 * h2
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def visualize_anomalies(self, image: np.ndarray, 
                          anomaly_regions: List[Dict[str, Any]]) -> np.ndarray:
        """可视化异常区域"""
        result_image = image.copy()
        
        for region in anomaly_regions:
            x, y, w, h = region['bbox']
            score = region['score']
            confidence = region['confidence']
            
            # 绘制边界框
            color = (0, 0, 255)  # 红色
            cv2.rectangle(result_image, (x, y), (x + w, y + h), color, 2)
            
            # 绘制分数
            label = f"Score: {score:.3f}"
            cv2.putText(result_image, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return result_image
```

### 4. 实时监控系统

```python
import threading
import queue
import time
import json
from typing import Dict, List, Any, Optional
from dataclasses import asdict
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

class RealTimeMonitor:
    """实时异常监控系统"""
    
    def __init__(self, detectors: Dict[str, BaseAnomalyDetector], 
                 alert_system: Optional['AlertSystem'] = None):
        self.detectors = detectors
        self.alert_system = alert_system
        
        # 数据队列
        self.data_queue = queue.Queue(maxsize=1000)
        self.result_queue = queue.Queue(maxsize=1000)
        
        # 监控状态
        self.is_monitoring = False
        self.monitor_thread = None
        
        # 历史数据
        self.history_data = {name: deque(maxlen=1000) for name in detectors.keys()}
        self.anomaly_history = deque(maxlen=100)
        
        # 统计信息
        self.stats = {
            'total_samples': 0,
            'total_anomalies': 0,
            'detector_stats': {name: detector.get_stats() for name, detector in detectors.items()}
        }
    
    def start_monitoring(self):
        """启动实时监控"""
        if self.is_monitoring:
            print("监控已在运行中")
            return
        
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        print("实时监控已启动")
    
    def stop_monitoring(self):
        """停止监控"""
        self.is_monitoring = False
        
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
        
        print("实时监控已停止")
    
    def add_data(self, detector_name: str, data: np.ndarray, metadata: Dict[str, Any] = None):
        """添加监控数据"""
        if not self.is_monitoring:
            return
        
        try:
            data_item = {
                'detector_name': detector_name,
                'data': data,
                'timestamp': time.time(),
                'metadata': metadata or {}
            }
            
            self.data_queue.put(data_item, timeout=1)
            
        except queue.Full:
            print("数据队列已满，丢弃数据")
    
    def _monitor_loop(self):
        """监控主循环"""
        while self.is_monitoring:
            try:
                # 获取数据
                data_item = self.data_queue.get(timeout=1)
                
                # 执行异常检测
                self._process_data_item(data_item)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"监控处理错误: {e}")
    
    def _process_data_item(self, data_item: Dict[str, Any]):
        """处理单个数据项"""
        detector_name = data_item['detector_name']
        data = data_item['data']
        timestamp = data_item['timestamp']
        metadata = data_item['metadata']
        
        if detector_name not in self.detectors:
            print(f"未知的检测器: {detector_name}")
            return
        
        detector = self.detectors[detector_name]
        
        try:
            # 执行异常检测
            anomalies = detector.detect(data)
            
            # 更新历史数据
            self.history_data[detector_name].append({
                'timestamp': timestamp,
                'data_shape': data.shape,
                'anomaly_count': len(anomalies),
                'metadata': metadata
            })
            
            # 处理异常结果
            if anomalies:
                self._handle_anomalies(detector_name, anomalies, metadata)
            
            # 更新统计信息
            self.stats['total_samples'] += len(data)
            self.stats['total_anomalies'] += len(anomalies)
            self.stats['detector_stats'][detector_name] = detector.get_stats()
            
            # 将结果放入结果队列
            result_item = {
                'detector_name': detector_name,
                'timestamp': timestamp,
                'anomalies': [asdict(anomaly) for anomaly in anomalies],
                'metadata': metadata
            }
            
            try:
                self.result_queue.put(result_item, timeout=0.1)
            except queue.Full:
                # 如果结果队列满了，移除最旧的结果
                try:
                    self.result_queue.get_nowait()
                    self.result_queue.put(result_item, timeout=0.1)
                except queue.Empty:
                    pass
            
        except Exception as e:
            print(f"数据处理错误 ({detector_name}): {e}")
    
    def _handle_anomalies(self, detector_name: str, anomalies: List[AnomalyResult], 
                         metadata: Dict[str, Any]):
        """处理检测到的异常"""
        for anomaly in anomalies:
            # 记录异常历史
            anomaly_record = {
                'detector_name': detector_name,
                'timestamp': anomaly.timestamp,
                'score': anomaly.score,
                'anomaly_type': anomaly.anomaly_type.value,
                'confidence': anomaly.confidence,
                'metadata': metadata
            }
            
            self.anomaly_history.append(anomaly_record)
            
            # 发送告警
            if self.alert_system:
                self.alert_system.send_alert(detector_name, anomaly, metadata)
            
            # 打印异常信息
            print(f"🚨 异常检测 [{detector_name}]: "
                  f"分数={anomaly.score:.3f}, "
                  f"类型={anomaly.anomaly_type.value}, "
                  f"置信度={anomaly.confidence:.3f}")
    
    def get_recent_results(self, n: int = 10) -> List[Dict[str, Any]]:
        """获取最近的检测结果"""
        results = []
        temp_queue = queue.Queue()
        
        # 从结果队列中获取数据
        while not self.result_queue.empty() and len(results) < n:
            try:
                result = self.result_queue.get_nowait()
                results.append(result)
                temp_queue.put(result)
            except queue.Empty:
                break
        
        # 将数据放回队列
        while not temp_queue.empty():
            self.result_queue.put(temp_queue.get())
        
        return sorted(results, key=lambda x: x['timestamp'], reverse=True)[:n]
    
    def get_anomaly_summary(self, time_window: float = 3600) -> Dict[str, Any]:
        """获取异常摘要（默认1小时窗口）"""
        current_time = time.time()
        cutoff_time = current_time - time_window
        
        # 过滤时间窗口内的异常
        recent_anomalies = [
            anomaly for anomaly in self.anomaly_history 
            if anomaly['timestamp'] >= cutoff_time
        ]
        
        # 按检测器分组统计
        detector_summary = {}
        for detector_name in self.detectors.keys():
            detector_anomalies = [
                anomaly for anomaly in recent_anomalies 
                if anomaly['detector_name'] == detector_name
            ]
            
            if detector_anomalies:
                scores = [anomaly['score'] for anomaly in detector_anomalies]
                detector_summary[detector_name] = {
                    'count': len(detector_anomalies),
                    'avg_score': np.mean(scores),
                    'max_score': np.max(scores),
                    'min_score': np.min(scores)
                }
            else:
                detector_summary[detector_name] = {
                    'count': 0,
                    'avg_score': 0.0,
                    'max_score': 0.0,
                    'min_score': 0.0
                }
        
        return {
            'time_window_hours': time_window / 3600,
            'total_anomalies': len(recent_anomalies),
            'detector_summary': detector_summary,
            'anomaly_rate': len(recent_anomalies) / max(1, self.stats['total_samples']) * 100
        }
    
    def export_data(self, filepath: str, format: str = 'json'):
        """导出监控数据"""
        try:
            export_data = {
                'timestamp': time.time(),
                'stats': self.stats,
                'anomaly_history': list(self.anomaly_history),
                'detector_configs': {
                    name: {
                        'threshold': detector.threshold,
                        'method': getattr(detector, 'method', 'unknown'),
                        'stats': detector.get_stats()
                    }
                    for name, detector in self.detectors.items()
                }
            }
            
            if format == 'json':
                with open(filepath, 'w', encoding='utf-8') as f:
                    json.dump(export_data, f, indent=2, ensure_ascii=False, default=str)
            
            print(f"监控数据已导出到: {filepath}")
            
        except Exception as e:
            print(f"数据导出失败: {e}")
    
    def create_dashboard(self):
        """创建实时监控仪表板"""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('实时异常监控仪表板')
        
        # 异常数量趋势
        ax1 = axes[0, 0]
        ax1.set_title('异常数量趋势')
        ax1.set_xlabel('时间')
        ax1.set_ylabel('异常数量')
        
        # 异常分数分布
        ax2 = axes[0, 1]
        ax2.set_title('异常分数分布')
        ax2.set_xlabel('异常分数')
        ax2.set_ylabel('频次')
        
        # 检测器性能
        ax3 = axes[1, 0]
        ax3.set_title('检测器性能')
        ax3.set_xlabel('检测器')
        ax3.set_ylabel('异常率 (%)')
        
        # 系统状态
        ax4 = axes[1, 1]
        ax4.set_title('系统状态')
        ax4.axis('off')
        
        def update_dashboard(frame):
            # 清除所有子图
            for ax in axes.flat:
                ax.clear()
            
            # 重新设置标题和标签
            axes[0, 0].set_title('异常数量趋势')
            axes[0, 1].set_title('异常分数分布')
            axes[1, 0].set_title('检测器性能')
            axes[1, 1].set_title('系统状态')
            axes[1, 1].axis('off')
            
            # 更新数据
            if self.anomaly_history:
                # 异常数量趋势
                timestamps = [anomaly['timestamp'] for anomaly in self.anomaly_history]
                if timestamps:
                    axes[0, 0].plot(timestamps, range(1, len(timestamps) + 1))
                
                # 异常分数分布
                scores = [anomaly['score'] for anomaly in self.anomaly_history]
                if scores:
                    axes[0, 1].hist(scores, bins=20, alpha=0.7)
                
                # 检测器性能
                detector_names = list(self.detectors.keys())
                anomaly_rates = []
                
                for name in detector_names:
                    detector_anomalies = [
                        anomaly for anomaly in self.anomaly_history 
                        if anomaly['detector_name'] == name
                    ]
                    rate = len(detector_anomalies) / max(1, len(self.anomaly_history)) * 100
                    anomaly_rates.append(rate)
                
                axes[1, 0].bar(detector_names, anomaly_rates)
                axes[1, 0].tick_params(axis='x', rotation=45)
            
            # 系统状态文本
            status_text = f"监控状态: {'运行中' if self.is_monitoring else '已停止'}\n"
            status_text += f"总样本数: {self.stats['total_samples']}\n"
            status_text += f"总异常数: {self.stats['total_anomalies']}\n"
            status_text += f"异常率: {self.stats['total_anomalies'] / max(1, self.stats['total_samples']) * 100:.2f}%"
            
            axes[1, 1].text(0.1, 0.5, status_text, fontsize=12, verticalalignment='center')
            
            plt.tight_layout()
        
        # 创建动画
        ani = animation.FuncAnimation(fig, update_dashboard, interval=2000, cache_frame_data=False)
        
        return fig, ani

# 使用示例
def main():
    # 创建检测器
    time_series_detector = TimeSeriesAnomalyDetector(method='isolation_forest')
    image_detector = ImageAnomalyDetector(method='autoencoder')
    
    detectors = {
        'time_series': time_series_detector,
        'image': image_detector
    }
    
    # 创建监控系统
    monitor = RealTimeMonitor(detectors)
    
    # 启动监控
    monitor.start_monitoring()
    
    try:
        # 模拟数据流
        for i in range(100):
            # 时序数据
            ts_data = np.random.random((10, 5))
            if i % 20 == 0:  # 注入异常
                ts_data += 5
            
            monitor.add_data('time_series', ts_data, {'source': 'sensor_1'})
            
            # 图像数据
            img_data = np.random.random((2, 64, 64, 3))
            monitor.add_data('image', img_data, {'source': 'camera_1'})
            
            time.sleep(0.1)
        
        # 获取结果
        results = monitor.get_recent_results(5)
        summary = monitor.get_anomaly_summary()
        
        print("最近结果:", len(results))
        print("异常摘要:", summary)
        
    finally:
        monitor.stop_monitoring()

if __name__ == "__main__":
    main()
```

## 应用场景

- **工业监控**: 设备状态监控和故障预警
- **网络安全**: 网络流量异常检测
- **金融风控**: 交易异常和欺诈检测
- **IoT监控**: 物联网设备异常监控
- **质量控制**: 产品质量异常检测
- **环境监测**: 环境参数异常告警

## 性能优化

### 1. 算法优化
- **增量学习**: 支持在线模型更新
- **集成方法**: 多种算法融合提高准确率
- **自适应阈值**: 动态调整异常阈值

### 2. 系统优化
- **并行处理**: 多线程异常检测
- **缓存机制**: 缓存计算结果
- **内存管理**: 优化内存使用

### 3. 部署优化
- **模型压缩**: 减少模型大小
- **硬件加速**: 利用GPU/NPU加速
- **边缘计算**: 本地化处理减少延迟

## 常见问题

### Q: 如何选择合适的异常检测算法？
A: 
1. 数据类型：时序数据用统计方法，图像用深度学习
2. 数据量：小数据集用传统方法，大数据集用深度学习
3. 实时性要求：实时检测用轻量级算法
4. 准确率要求：高准确率用集成方法

### Q: 如何处理误报问题？
A: 
1. 调整检测阈值
2. 使用多种算法投票
3. 结合领域知识过滤
4. 持续学习和模型更新

### Q: 如何评估检测效果？
A: 
1. 准确率、召回率、F1分数
2. ROC曲线和AUC值
3. 误报率和漏报率
4. 实际业务指标

## 扩展功能

- **多模态融合**: 结合多种数据类型检测
- **因果分析**: 分析异常产生原因
- **预测性维护**: 基于异常趋势预测故障
- **自动化响应**: 异常自动处理和恢复
- **可解释性**: 提供异常检测解释

欢迎贡献代码和提出改进建议！
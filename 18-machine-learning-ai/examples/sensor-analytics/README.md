# 传感器数据分析示例

## 概述

本示例展示了如何在边缘设备上实现高效的传感器数据分析系统，包括数据采集、预处理、特征提取、模式识别和预测分析。支持多种传感器类型和实时数据处理。

## 功能特性

- **多传感器支持**: 温度、湿度、压力、加速度、陀螺仪等多种传感器
- **实时数据处理**: 流式数据处理和实时分析
- **智能预处理**: 噪声滤波、数据校准和异常值处理
- **特征工程**: 自动特征提取和选择
- **模式识别**: 传感器数据模式分类和识别
- **预测分析**: 基于历史数据的趋势预测

## 项目结构

```
sensor-analytics/
├── README.md                    # 本文件
├── requirements.txt             # Python依赖
├── models/                      # 预训练模型
│   ├── classification/         # 分类模型
│   │   ├── activity_classifier.tflite
│   │   └── gesture_classifier.onnx
│   ├── regression/             # 回归模型
│   │   ├── temperature_predictor.tflite
│   │   └── pressure_predictor.pkl
│   └── clustering/             # 聚类模型
│       └── sensor_clustering.pkl
├── src/                         # 源代码
│   ├── __init__.py
│   ├── sensor_manager.py       # 传感器管理器
│   ├── data_processor.py       # 数据处理器
│   ├── feature_extractor.py    # 特征提取器
│   ├── pattern_recognizer.py   # 模式识别器
│   ├── predictor.py            # 预测器
│   ├── analytics_engine.py     # 分析引擎
│   └── utils.py                # 工具函数
├── examples/                    # 使用示例
│   ├── basic_analytics.py      # 基础分析示例
│   ├── realtime_monitoring.py  # 实时监控
│   ├── activity_recognition.py # 活动识别
│   ├── environmental_monitoring.py # 环境监测
│   └── predictive_maintenance.py # 预测性维护
├── data/                        # 数据目录
│   ├── sensor_data/            # 传感器数据
│   ├── calibration/            # 校准数据
│   ├── models/                 # 训练好的模型
│   └── test_data/              # 测试数据
├── configs/                     # 配置文件
│   ├── sensor_config.yaml      # 传感器配置
│   ├── processing_config.yaml  # 处理配置
│   └── analytics_config.yaml   # 分析配置
└── tests/                       # 测试用例
    ├── test_sensor_manager.py  # 传感器管理测试
    ├── test_data_processor.py  # 数据处理测试
    └── test_analytics.py       # 分析测试
```

## 快速开始

### 1. 环境准备

```bash
# 安装依赖
pip install -r requirements.txt

# 安装传感器库（根据硬件平台）
# 树莓派
sudo apt-get install python3-smbus i2c-tools
# 或者使用pip安装
pip install adafruit-circuitpython-* RPi.GPIO
```

### 2. 基础使用

```python
from src.analytics_engine import SensorAnalyticsEngine

# 创建传感器分析引擎
engine = SensorAnalyticsEngine(
    config_path='configs/analytics_config.yaml'
)

# 初始化引擎
engine.initialize()

# 添加传感器数据
import numpy as np
sensor_data = {
    'temperature': np.array([25.1, 25.3, 25.2, 25.4]),
    'humidity': np.array([60.2, 60.5, 60.1, 60.3]),
    'pressure': np.array([1013.2, 1013.1, 1013.3, 1013.0])
}

# 执行分析
results = engine.analyze(sensor_data)

print("分析结果:")
for sensor, result in results.items():
    print(f"{sensor}: {result}")
```

### 3. 实时监控

```bash
python examples/realtime_monitoring.py --sensors temperature,humidity,pressure --interval 1
```

### 4. 活动识别

```bash
python examples/activity_recognition.py --model models/classification/activity_classifier.tflite
```

## 核心组件

### 1. 传感器管理器

```python
import time
import threading
import queue
import numpy as np
from typing import Dict, List, Any, Optional, Callable
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

class SensorType(Enum):
    """传感器类型"""
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"
    PRESSURE = "pressure"
    ACCELEROMETER = "accelerometer"
    GYROSCOPE = "gyroscope"
    MAGNETOMETER = "magnetometer"
    LIGHT = "light"
    SOUND = "sound"
    GAS = "gas"
    PROXIMITY = "proximity"

@dataclass
class SensorReading:
    """传感器读数"""
    sensor_id: str
    sensor_type: SensorType
    value: np.ndarray
    timestamp: float
    unit: str
    quality: float = 1.0  # 数据质量评分 0-1
    metadata: Dict[str, Any] = None

class BaseSensor(ABC):
    """传感器基类"""
    
    def __init__(self, sensor_id: str, sensor_type: SensorType, 
                 sampling_rate: float = 1.0, calibration_data: Dict[str, Any] = None):
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type
        self.sampling_rate = sampling_rate
        self.calibration_data = calibration_data or {}
        
        # 传感器状态
        self.is_active = False
        self.last_reading = None
        self.error_count = 0
        self.total_readings = 0
    
    @abstractmethod
    def read_raw(self) -> Any:
        """读取原始传感器数据"""
        pass
    
    def read(self) -> Optional[SensorReading]:
        """读取并处理传感器数据"""
        try:
            # 读取原始数据
            raw_data = self.read_raw()
            
            if raw_data is None:
                self.error_count += 1
                return None
            
            # 数据处理和校准
            processed_data = self.process_data(raw_data)
            
            # 创建传感器读数
            reading = SensorReading(
                sensor_id=self.sensor_id,
                sensor_type=self.sensor_type,
                value=processed_data,
                timestamp=time.time(),
                unit=self.get_unit(),
                quality=self.assess_quality(processed_data),
                metadata=self.get_metadata()
            )
            
            self.last_reading = reading
            self.total_readings += 1
            
            return reading
            
        except Exception as e:
            print(f"传感器 {self.sensor_id} 读取错误: {e}")
            self.error_count += 1
            return None
    
    def process_data(self, raw_data: Any) -> np.ndarray:
        """处理和校准数据"""
        # 转换为numpy数组
        if not isinstance(raw_data, np.ndarray):
            if isinstance(raw_data, (list, tuple)):
                data = np.array(raw_data, dtype=np.float32)
            else:
                data = np.array([raw_data], dtype=np.float32)
        else:
            data = raw_data.astype(np.float32)
        
        # 应用校准
        if 'offset' in self.calibration_data:
            data += self.calibration_data['offset']
        
        if 'scale' in self.calibration_data:
            data *= self.calibration_data['scale']
        
        return data
    
    def assess_quality(self, data: np.ndarray) -> float:
        """评估数据质量"""
        # 简单的质量评估
        if len(data) == 0:
            return 0.0
        
        # 检查异常值
        if np.any(np.isnan(data)) or np.any(np.isinf(data)):
            return 0.0
        
        # 检查数据范围
        expected_range = self.get_expected_range()
        if expected_range:
            min_val, max_val = expected_range
            if np.any(data < min_val) or np.any(data > max_val):
                return 0.5  # 部分质量
        
        return 1.0  # 良好质量
    
    @abstractmethod
    def get_unit(self) -> str:
        """获取测量单位"""
        pass
    
    @abstractmethod
    def get_expected_range(self) -> Optional[tuple]:
        """获取期望的数据范围"""
        pass
    
    def get_metadata(self) -> Dict[str, Any]:
        """获取元数据"""
        return {
            'sampling_rate': self.sampling_rate,
            'error_rate': self.error_count / max(1, self.total_readings),
            'total_readings': self.total_readings
        }
    
    def calibrate(self, reference_values: List[float], measured_values: List[float]):
        """校准传感器"""
        if len(reference_values) != len(measured_values):
            raise ValueError("参考值和测量值数量不匹配")
        
        ref_array = np.array(reference_values)
        meas_array = np.array(measured_values)
        
        # 线性校准: y = ax + b
        coeffs = np.polyfit(meas_array, ref_array, 1)
        
        self.calibration_data['scale'] = coeffs[0]
        self.calibration_data['offset'] = coeffs[1]
        
        print(f"传感器 {self.sensor_id} 校准完成: scale={coeffs[0]:.4f}, offset={coeffs[1]:.4f}")

class TemperatureSensor(BaseSensor):
    """温度传感器"""
    
    def __init__(self, sensor_id: str, pin: int = None, **kwargs):
        super().__init__(sensor_id, SensorType.TEMPERATURE, **kwargs)
        self.pin = pin
    
    def read_raw(self) -> float:
        """读取原始温度数据"""
        # 模拟温度读取（实际应用中需要根据具体传感器实现）
        # 例如：DS18B20, DHT22等
        import random
        return 20.0 + random.gauss(0, 2.0)  # 模拟温度 20±2°C
    
    def get_unit(self) -> str:
        return "°C"
    
    def get_expected_range(self) -> tuple:
        return (-40.0, 85.0)  # 典型温度传感器范围

class AccelerometerSensor(BaseSensor):
    """加速度传感器"""
    
    def __init__(self, sensor_id: str, **kwargs):
        super().__init__(sensor_id, SensorType.ACCELEROMETER, **kwargs)
    
    def read_raw(self) -> List[float]:
        """读取原始加速度数据"""
        # 模拟3轴加速度数据
        import random
        return [
            random.gauss(0, 0.1),  # X轴
            random.gauss(0, 0.1),  # Y轴
            random.gauss(9.8, 0.1) # Z轴（重力）
        ]
    
    def get_unit(self) -> str:
        return "m/s²"
    
    def get_expected_range(self) -> tuple:
        return (-20.0, 20.0)  # ±2g范围

class SensorManager:
    """传感器管理器"""
    
    def __init__(self):
        self.sensors: Dict[str, BaseSensor] = {}
        self.data_callbacks: List[Callable] = []
        
        # 数据采集
        self.is_collecting = False
        self.collection_thread = None
        self.data_queue = queue.Queue(maxsize=1000)
        
        # 统计信息
        self.stats = {
            'total_readings': 0,
            'error_count': 0,
            'sensor_stats': {}
        }
    
    def add_sensor(self, sensor: BaseSensor):
        """添加传感器"""
        self.sensors[sensor.sensor_id] = sensor
        self.stats['sensor_stats'][sensor.sensor_id] = {
            'readings': 0,
            'errors': 0,
            'last_reading_time': None
        }
        
        print(f"传感器已添加: {sensor.sensor_id} ({sensor.sensor_type.value})")
    
    def remove_sensor(self, sensor_id: str):
        """移除传感器"""
        if sensor_id in self.sensors:
            del self.sensors[sensor_id]
            del self.stats['sensor_stats'][sensor_id]
            print(f"传感器已移除: {sensor_id}")
    
    def add_data_callback(self, callback: Callable):
        """添加数据回调函数"""
        self.data_callbacks.append(callback)
    
    def start_collection(self, interval: float = 1.0):
        """开始数据采集"""
        if self.is_collecting:
            print("数据采集已在运行")
            return
        
        self.is_collecting = True
        self.collection_thread = threading.Thread(
            target=self._collection_loop,
            args=(interval,)
        )
        self.collection_thread.daemon = True
        self.collection_thread.start()
        
        print(f"数据采集已启动，采集间隔: {interval}秒")
    
    def stop_collection(self):
        """停止数据采集"""
        self.is_collecting = False
        
        if self.collection_thread:
            self.collection_thread.join(timeout=5)
        
        print("数据采集已停止")
    
    def _collection_loop(self, interval: float):
        """数据采集循环"""
        while self.is_collecting:
            start_time = time.time()
            
            # 读取所有传感器数据
            readings = {}
            
            for sensor_id, sensor in self.sensors.items():
                reading = sensor.read()
                
                if reading:
                    readings[sensor_id] = reading
                    
                    # 更新统计
                    self.stats['total_readings'] += 1
                    self.stats['sensor_stats'][sensor_id]['readings'] += 1
                    self.stats['sensor_stats'][sensor_id]['last_reading_time'] = reading.timestamp
                else:
                    # 读取失败
                    self.stats['error_count'] += 1
                    self.stats['sensor_stats'][sensor_id]['errors'] += 1
            
            # 如果有数据，放入队列并调用回调
            if readings:
                try:
                    self.data_queue.put(readings, timeout=0.1)
                except queue.Full:
                    print("数据队列已满，丢弃数据")
                
                # 调用回调函数
                for callback in self.data_callbacks:
                    try:
                        callback(readings)
                    except Exception as e:
                        print(f"回调函数执行错误: {e}")
            
            # 控制采集间隔
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    def get_latest_readings(self, sensor_ids: List[str] = None) -> Dict[str, SensorReading]:
        """获取最新读数"""
        readings = {}
        
        target_sensors = sensor_ids if sensor_ids else list(self.sensors.keys())
        
        for sensor_id in target_sensors:
            if sensor_id in self.sensors:
                sensor = self.sensors[sensor_id]
                if sensor.last_reading:
                    readings[sensor_id] = sensor.last_reading
        
        return readings
    
    def get_sensor_stats(self) -> Dict[str, Any]:
        """获取传感器统计信息"""
        return {
            'total_sensors': len(self.sensors),
            'active_sensors': sum(1 for s in self.sensors.values() if s.is_active),
            'total_readings': self.stats['total_readings'],
            'error_count': self.stats['error_count'],
            'error_rate': self.stats['error_count'] / max(1, self.stats['total_readings']),
            'sensor_details': self.stats['sensor_stats'].copy()
        }
    
    def calibrate_sensor(self, sensor_id: str, reference_values: List[float], 
                        measured_values: List[float]):
        """校准指定传感器"""
        if sensor_id not in self.sensors:
            raise ValueError(f"传感器 {sensor_id} 不存在")
        
        sensor = self.sensors[sensor_id]
        sensor.calibrate(reference_values, measured_values)
```

### 2. 数据处理器

```python
import numpy as np
import scipy.signal
from scipy import stats
from typing import Dict, List, Any, Optional, Tuple
from collections import deque
import warnings

class SensorDataProcessor:
    """传感器数据处理器"""
    
    def __init__(self, window_size: int = 100, sampling_rate: float = 1.0):
        self.window_size = window_size
        self.sampling_rate = sampling_rate
        
        # 数据缓冲区
        self.data_buffers: Dict[str, deque] = {}
        
        # 滤波器参数
        self.filter_configs = {
            'lowpass': {'cutoff': 0.1, 'order': 4},
            'highpass': {'cutoff': 0.01, 'order': 2},
            'bandpass': {'low': 0.01, 'high': 0.1, 'order': 4}
        }
        
        # 异常检测参数
        self.outlier_threshold = 3.0  # Z-score阈值
        self.outlier_method = 'zscore'  # 'zscore', 'iqr', 'isolation'
    
    def add_data(self, sensor_id: str, data: np.ndarray, timestamp: float = None):
        """添加传感器数据到缓冲区"""
        if sensor_id not in self.data_buffers:
            self.data_buffers[sensor_id] = deque(maxlen=self.window_size)
        
        # 确保数据是1D数组
        if data.ndim > 1:
            data = data.flatten()
        
        # 添加时间戳信息
        if timestamp is None:
            timestamp = time.time()
        
        for value in data:
            self.data_buffers[sensor_id].append({
                'value': value,
                'timestamp': timestamp
            })
    
    def get_buffer_data(self, sensor_id: str, as_array: bool = True) -> Any:
        """获取缓冲区数据"""
        if sensor_id not in self.data_buffers:
            return np.array([]) if as_array else []
        
        buffer = self.data_buffers[sensor_id]
        
        if as_array:
            values = [item['value'] for item in buffer]
            return np.array(values)
        else:
            return list(buffer)
    
    def remove_outliers(self, data: np.ndarray, method: str = None) -> Tuple[np.ndarray, np.ndarray]:
        """移除异常值"""
        if len(data) == 0:
            return data, np.array([], dtype=bool)
        
        method = method or self.outlier_method
        
        if method == 'zscore':
            z_scores = np.abs(stats.zscore(data))
            outlier_mask = z_scores > self.outlier_threshold
            
        elif method == 'iqr':
            Q1 = np.percentile(data, 25)
            Q3 = np.percentile(data, 75)
            IQR = Q3 - Q1
            lower_bound = Q1 - 1.5 * IQR
            upper_bound = Q3 + 1.5 * IQR
            outlier_mask = (data < lower_bound) | (data > upper_bound)
            
        elif method == 'isolation':
            from sklearn.ensemble import IsolationForest
            
            if len(data) < 10:  # 数据太少，跳过异常检测
                return data, np.array([False] * len(data))
            
            iso_forest = IsolationForest(contamination=0.1, random_state=42)
            outlier_pred = iso_forest.fit_predict(data.reshape(-1, 1))
            outlier_mask = outlier_pred == -1
            
        else:
            raise ValueError(f"不支持的异常检测方法: {method}")
        
        # 返回清理后的数据和异常值掩码
        clean_data = data[~outlier_mask]
        
        return clean_data, outlier_mask
    
    def apply_filter(self, data: np.ndarray, filter_type: str = 'lowpass', 
                    **kwargs) -> np.ndarray:
        """应用数字滤波器"""
        if len(data) < 10:  # 数据太少，跳过滤波
            return data
        
        # 获取滤波器配置
        config = self.filter_configs.get(filter_type, {})
        config.update(kwargs)
        
        try:
            if filter_type == 'lowpass':
                cutoff = config.get('cutoff', 0.1)
                order = config.get('order', 4)
                
                # 设计低通滤波器
                nyquist = 0.5 * self.sampling_rate
                normal_cutoff = cutoff / nyquist
                b, a = scipy.signal.butter(order, normal_cutoff, btype='low', analog=False)
                
                # 应用滤波器
                filtered_data = scipy.signal.filtfilt(b, a, data)
                
            elif filter_type == 'highpass':
                cutoff = config.get('cutoff', 0.01)
                order = config.get('order', 2)
                
                nyquist = 0.5 * self.sampling_rate
                normal_cutoff = cutoff / nyquist
                b, a = scipy.signal.butter(order, normal_cutoff, btype='high', analog=False)
                
                filtered_data = scipy.signal.filtfilt(b, a, data)
                
            elif filter_type == 'bandpass':
                low = config.get('low', 0.01)
                high = config.get('high', 0.1)
                order = config.get('order', 4)
                
                nyquist = 0.5 * self.sampling_rate
                low_normal = low / nyquist
                high_normal = high / nyquist
                
                b, a = scipy.signal.butter(order, [low_normal, high_normal], 
                                         btype='band', analog=False)
                
                filtered_data = scipy.signal.filtfilt(b, a, data)
                
            elif filter_type == 'median':
                kernel_size = config.get('kernel_size', 5)
                filtered_data = scipy.signal.medfilt(data, kernel_size=kernel_size)
                
            elif filter_type == 'savgol':
                window_length = config.get('window_length', 11)
                polyorder = config.get('polyorder', 3)
                
                # 确保窗口长度是奇数且小于数据长度
                window_length = min(window_length, len(data))
                if window_length % 2 == 0:
                    window_length -= 1
                window_length = max(3, window_length)
                
                polyorder = min(polyorder, window_length - 1)
                
                filtered_data = scipy.signal.savgol_filter(data, window_length, polyorder)
                
            else:
                raise ValueError(f"不支持的滤波器类型: {filter_type}")
            
            return filtered_data
            
        except Exception as e:
            warnings.warn(f"滤波器应用失败: {e}")
            return data
    
    def smooth_data(self, data: np.ndarray, method: str = 'moving_average', 
                   window: int = 5) -> np.ndarray:
        """数据平滑"""
        if len(data) < window:
            return data
        
        if method == 'moving_average':
            # 移动平均
            smoothed = np.convolve(data, np.ones(window)/window, mode='same')
            
        elif method == 'exponential':
            # 指数平滑
            alpha = 2.0 / (window + 1)
            smoothed = np.zeros_like(data)
            smoothed[0] = data[0]
            
            for i in range(1, len(data)):
                smoothed[i] = alpha * data[i] + (1 - alpha) * smoothed[i-1]
                
        elif method == 'gaussian':
            # 高斯平滑
            sigma = window / 3.0
            smoothed = scipy.ndimage.gaussian_filter1d(data, sigma)
            
        else:
            raise ValueError(f"不支持的平滑方法: {method}")
        
        return smoothed
    
    def resample_data(self, data: np.ndarray, timestamps: np.ndarray, 
                     target_rate: float) -> Tuple[np.ndarray, np.ndarray]:
        """重采样数据"""
        if len(data) != len(timestamps):
            raise ValueError("数据和时间戳长度不匹配")
        
        if len(data) < 2:
            return data, timestamps
        
        # 计算目标时间点
        start_time = timestamps[0]
        end_time = timestamps[-1]
        duration = end_time - start_time
        
        num_samples = int(duration * target_rate) + 1
        target_timestamps = np.linspace(start_time, end_time, num_samples)
        
        # 插值重采样
        resampled_data = np.interp(target_timestamps, timestamps, data)
        
        return resampled_data, target_timestamps
    
    def detect_peaks(self, data: np.ndarray, **kwargs) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
        """检测峰值"""
        # 设置默认参数
        height = kwargs.get('height', None)
        threshold = kwargs.get('threshold', None)
        distance = kwargs.get('distance', 1)
        prominence = kwargs.get('prominence', None)
        width = kwargs.get('width', None)
        
        # 检测峰值
        peaks, properties = scipy.signal.find_peaks(
            data,
            height=height,
            threshold=threshold,
            distance=distance,
            prominence=prominence,
            width=width
        )
        
        return peaks, properties
    
    def calculate_statistics(self, data: np.ndarray) -> Dict[str, float]:
        """计算统计特征"""
        if len(data) == 0:
            return {}
        
        statistics = {
            'mean': float(np.mean(data)),
            'std': float(np.std(data)),
            'var': float(np.var(data)),
            'min': float(np.min(data)),
            'max': float(np.max(data)),
            'median': float(np.median(data)),
            'q25': float(np.percentile(data, 25)),
            'q75': float(np.percentile(data, 75)),
            'skewness': float(stats.skew(data)),
            'kurtosis': float(stats.kurtosis(data)),
            'rms': float(np.sqrt(np.mean(data**2))),
            'peak_to_peak': float(np.ptp(data))
        }
        
        return statistics
    
    def process_sensor_data(self, sensor_id: str, 
                          processing_config: Dict[str, Any] = None) -> Dict[str, Any]:
        """处理指定传感器的数据"""
        # 获取原始数据
        raw_data = self.get_buffer_data(sensor_id)
        
        if len(raw_data) == 0:
            return {'error': '无数据可处理'}
        
        processing_config = processing_config or {}
        results = {'sensor_id': sensor_id, 'original_length': len(raw_data)}
        
        # 1. 异常值检测和移除
        if processing_config.get('remove_outliers', True):
            clean_data, outlier_mask = self.remove_outliers(
                raw_data, 
                method=processing_config.get('outlier_method', self.outlier_method)
            )
            results['outliers_removed'] = int(np.sum(outlier_mask))
            results['outlier_percentage'] = float(np.sum(outlier_mask) / len(raw_data) * 100)
        else:
            clean_data = raw_data
            results['outliers_removed'] = 0
        
        # 2. 滤波
        if processing_config.get('apply_filter', False):
            filter_type = processing_config.get('filter_type', 'lowpass')
            filter_params = processing_config.get('filter_params', {})
            
            filtered_data = self.apply_filter(clean_data, filter_type, **filter_params)
            results['filter_applied'] = filter_type
        else:
            filtered_data = clean_data
        
        # 3. 平滑
        if processing_config.get('smooth_data', False):
            smooth_method = processing_config.get('smooth_method', 'moving_average')
            smooth_window = processing_config.get('smooth_window', 5)
            
            smoothed_data = self.smooth_data(filtered_data, smooth_method, smooth_window)
            results['smoothing_applied'] = smooth_method
        else:
            smoothed_data = filtered_data
        
        # 4. 统计特征计算
        if processing_config.get('calculate_stats', True):
            stats = self.calculate_statistics(smoothed_data)
            results['statistics'] = stats
        
        # 5. 峰值检测
        if processing_config.get('detect_peaks', False):
            peak_params = processing_config.get('peak_params', {})
            peaks, peak_properties = self.detect_peaks(smoothed_data, **peak_params)
            
            results['peaks'] = {
                'indices': peaks.tolist(),
                'count': len(peaks),
                'properties': {k: v.tolist() if isinstance(v, np.ndarray) else v 
                             for k, v in peak_properties.items()}
            }
        
        # 6. 保存处理后的数据
        results['processed_data'] = smoothed_data
        results['processing_timestamp'] = time.time()
        
        return results
    
    def batch_process(self, sensor_ids: List[str] = None, 
                     processing_config: Dict[str, Any] = None) -> Dict[str, Any]:
        """批量处理多个传感器数据"""
        if sensor_ids is None:
            sensor_ids = list(self.data_buffers.keys())
        
        results = {}
        
        for sensor_id in sensor_ids:
            if sensor_id in self.data_buffers:
                try:
                    result = self.process_sensor_data(sensor_id, processing_config)
                    results[sensor_id] = result
                except Exception as e:
                    results[sensor_id] = {'error': str(e)}
        
        return results
    
    def get_processing_summary(self) -> Dict[str, Any]:
        """获取处理摘要"""
        summary = {
            'total_sensors': len(self.data_buffers),
            'window_size': self.window_size,
            'sampling_rate': self.sampling_rate,
            'filter_configs': self.filter_configs,
            'outlier_threshold': self.outlier_threshold,
            'sensor_buffer_status': {}
        }
        
        for sensor_id, buffer in self.data_buffers.items():
            summary['sensor_buffer_status'][sensor_id] = {
                'buffer_length': len(buffer),
                'buffer_full': len(buffer) == self.window_size,
                'latest_timestamp': buffer[-1]['timestamp'] if buffer else None
            }
        
        return summary
```

### 3. 特征提取器

```python
import numpy as np
import scipy.fft
from scipy import signal
from typing import Dict, List, Any, Optional
import warnings

class SensorFeatureExtractor:
    """传感器特征提取器"""
    
    def __init__(self, sampling_rate: float = 1.0):
        self.sampling_rate = sampling_rate
        
        # 特征提取配置
        self.time_domain_features = [
            'mean', 'std', 'var', 'min', 'max', 'median', 'q25', 'q75',
            'skewness', 'kurtosis', 'rms', 'peak_to_peak', 'crest_factor',
            'shape_factor', 'impulse_factor', 'clearance_factor'
        ]
        
        self.frequency_domain_features = [
            'spectral_centroid', 'spectral_bandwidth', 'spectral_rolloff',
            'spectral_flux', 'spectral_flatness', 'dominant_frequency',
            'frequency_std', 'power_spectral_density'
        ]
        
        self.time_frequency_features = [
            'wavelet_energy', 'wavelet_entropy', 'stft_features'
        ]
    
    def extract_time_domain_features(self, data: np.ndarray) -> Dict[str, float]:
        """提取时域特征"""
        if len(data) == 0:
            return {}
        
        features = {}
        
        try:
            # 基本统计特征
            features['mean'] = float(np.mean(data))
            features['std'] = float(np.std(data))
            features['var'] = float(np.var(data))
            features['min'] = float(np.min(data))
            features['max'] = float(np.max(data))
            features['median'] = float(np.median(data))
            features['q25'] = float(np.percentile(data, 25))
            features['q75'] = float(np.percentile(data, 75))
            
            # 高阶统计特征
            from scipy import stats
            features['skewness'] = float(stats.skew(data))
            features['kurtosis'] = float(stats.kurtosis(data))
            
            # RMS和峰峰值
            features['rms'] = float(np.sqrt(np.mean(data**2)))
            features['peak_to_peak'] = float(np.ptp(data))
            
            # 形状因子
            if features['mean'] != 0:
                features['crest_factor'] = features['max'] / features['rms']
                features['shape_factor'] = features['rms'] / np.mean(np.abs(data))
                features['impulse_factor'] = features['max'] / np.mean(np.abs(data))
                features['clearance_factor'] = features['max'] / (np.mean(np.sqrt(np.abs(data))))**2
            
            # 零交叉率
            zero_crossings = np.where(np.diff(np.signbit(data)))[0]
            features['zero_crossing_rate'] = float(len(zero_crossings) / len(data))
            
            # 能量
            features['energy'] = float(np.sum(data**2))
            
            # 平均绝对偏差
            features['mean_absolute_deviation'] = float(np.mean(np.abs(data - features['mean'])))
            
        except Exception as e:
            warnings.warn(f"时域特征提取部分失败: {e}")
        
        return features
    
    def extract_frequency_domain_features(self, data: np.ndarray) -> Dict[str, float]:
        """提取频域特征"""
        if len(data) < 4:  # 需要足够的数据点进行FFT
            return {}
        
        features = {}
        
        try:
            # 计算FFT
            fft_data = scipy.fft.fft(data)
            freqs = scipy.fft.fftfreq(len(data), 1/self.sampling_rate)
            
            # 只取正频率部分
            positive_freqs = freqs[:len(freqs)//2]
            magnitude = np.abs(fft_data[:len(fft_data)//2])
            
            if len(magnitude) == 0 or np.sum(magnitude) == 0:
                return features
            
            # 功率谱密度
            psd = magnitude**2
            
            # 谱质心
            features['spectral_centroid'] = float(np.sum(positive_freqs * magnitude) / np.sum(magnitude))
            
            # 谱带宽
            centroid = features['spectral_centroid']
            features['spectral_bandwidth'] = float(
                np.sqrt(np.sum(((positive_freqs - centroid)**2) * magnitude) / np.sum(magnitude))
            )
            
            # 谱滚降点（85%能量所在频率）
            cumulative_magnitude = np.cumsum(magnitude)
            rolloff_threshold = 0.85 * cumulative_magnitude[-1]
            rolloff_index = np.where(cumulative_magnitude >= rolloff_threshold)[0]
            if len(rolloff_index) > 0:
                features['spectral_rolloff'] = float(positive_freqs[rolloff_index[0]])
            
            # 谱通量（相邻帧间的谱变化）
            if hasattr(self, '_prev_magnitude'):
                spectral_flux = np.sum((magnitude - self._prev_magnitude)**2)
                features['spectral_flux'] = float(spectral_flux)
            self._prev_magnitude = magnitude.copy()
            
            # 谱平坦度
            geometric_mean = np.exp(np.mean(np.log(magnitude + 1e-10)))
            arithmetic_mean = np.mean(magnitude)
            if arithmetic_mean > 0:
                features['spectral_flatness'] = float(geometric_mean / arithmetic_mean)
            
            # 主导频率
            dominant_freq_index = np.argmax(magnitude)
            features['dominant_frequency'] = float(positive_freqs[dominant_freq_index])
            
            # 频率标准差
            features['frequency_std'] = float(
                np.sqrt(np.sum(((positive_freqs - features['spectral_centroid'])**2) * magnitude) / np.sum(magnitude))
            )
            
            # 功率谱密度特征
            features['total_power'] = float(np.sum(psd))
            features['mean_power'] = float(np.mean(psd))
            features['max_power'] = float(np.max(psd))
            
            # 频带能量比
            nyquist = self.sampling_rate / 2
            low_freq_mask = positive_freqs <= nyquist * 0.1
            mid_freq_mask = (positive_freqs > nyquist * 0.1) & (positive_freqs <= nyquist * 0.4)
            high_freq_mask = positive_freqs > nyquist * 0.4
            
            total_energy = np.sum(psd)
            if total_energy > 0:
                features['low_freq_energy_ratio'] = float(np.sum(psd[low_freq_mask]) / total_energy)
                features['mid_freq_energy_ratio'] = float(np.sum(psd[mid_freq_mask]) / total_energy)
                features['high_freq_energy_ratio'] = float(np.sum(psd[high_freq_mask]) / total_energy)
            
        except Exception as e:
            warnings.warn(f"频域特征提取部分失败: {e}")
        
        return features
    
    def extract_wavelet_features(self, data: np.ndarray, wavelet: str = 'db4', 
                                levels: int = 4) -> Dict[str, float]:
        """提取小波特征"""
        if len(data) < 8:  # 需要足够的数据点进行小波变换
            return {}
        
        features = {}
        
        try:
            import pywt
            
            # 小波分解
            coeffs = pywt.wavedec(data, wavelet, level=levels)
            
            # 各层能量
            for i, coeff in enumerate(coeffs):
                energy = np.sum(coeff**2)
                features[f'wavelet_energy_level_{i}'] = float(energy)
            
            # 总能量
            total_energy = sum(features[f'wavelet_energy_level_{i}'] for i in range(len(coeffs)))
            features['wavelet_total_energy'] = float(total_energy)
            
            # 相对能量
            if total_energy > 0:
                for i in range(len(coeffs)):
                    features[f'wavelet_relative_energy_level_{i}'] = float(
                        features[f'wavelet_energy_level_{i}'] / total_energy
                    )
            
            # 小波熵
            if total_energy > 0:
                relative_energies = [features[f'wavelet_energy_level_{i}'] / total_energy 
                                   for i in range(len(coeffs))]
                entropy = -np.sum([e * np.log2(e + 1e-10) for e in relative_energies if e > 0])
                features['wavelet_entropy'] = float(entropy)
            
        except ImportError:
            warnings.warn("PyWavelets未安装，跳过小波特征提取")
        except Exception as e:
            warnings.warn(f"小波特征提取失败: {e}")
        
        return features
    
    def extract_stft_features(self, data: np.ndarray, nperseg: int = None) -> Dict[str, float]:
        """提取短时傅里叶变换特征"""
        if len(data) < 16:
            return {}
        
        features = {}
        
        try:
            # 设置STFT参数
            if nperseg is None:
                nperseg = min(256, len(data) // 4)
            
            # 计算STFT
            f, t, Zxx = signal.stft(data, fs=self.sampling_rate, nperseg=nperseg)
            
            # 幅度谱
            magnitude = np.abs(Zxx)
            
            # 时间-频率特征
            features['stft_mean_magnitude'] = float(np.mean(magnitude))
            features['stft_std_magnitude'] = float(np.std(magnitude))
            features['stft_max_magnitude'] = float(np.max(magnitude))
            
            # 频谱质心随时间的变化
            spectral_centroids = []
            for i in range(magnitude.shape[1]):
                if np.sum(magnitude[:, i]) > 0:
                    centroid = np.sum(f * magnitude[:, i]) / np.sum(magnitude[:, i])
                    spectral_centroids.append(centroid)
            
            if spectral_centroids:
                features['stft_spectral_centroid_mean'] = float(np.mean(spectral_centroids))
                features['stft_spectral_centroid_std'] = float(np.std(spectral_centroids))
            
            # 频谱带宽随时间的变化
            spectral_bandwidths = []
            for i in range(magnitude.shape[1]):
                if np.sum(magnitude[:, i]) > 0:
                    centroid = np.sum(f * magnitude[:, i]) / np.sum(magnitude[:, i])
                    bandwidth = np.sqrt(np.sum(((f - centroid)**2) * magnitude[:, i]) / np.sum(magnitude[:, i]))
                    spectral_bandwidths.append(bandwidth)
            
            if spectral_bandwidths:
                features['stft_spectral_bandwidth_mean'] = float(np.mean(spectral_bandwidths))
                features['stft_spectral_bandwidth_std'] = float(np.std(spectral_bandwidths))
            
        except Exception as e:
            warnings.warn(f"STFT特征提取失败: {e}")
        
        return features
    
    def extract_all_features(self, data: np.ndarray, 
                           feature_types: List[str] = None) -> Dict[str, float]:
        """提取所有类型的特征"""
        if feature_types is None:
            feature_types = ['time_domain', 'frequency_domain', 'wavelet', 'stft']
        
        all_features = {}
        
        # 时域特征
        if 'time_domain' in feature_types:
            time_features = self.extract_time_domain_features(data)
            all_features.update(time_features)
        
        # 频域特征
        if 'frequency_domain' in feature_types:
            freq_features = self.extract_frequency_domain_features(data)
            all_features.update(freq_features)
        
        # 小波特征
        if 'wavelet' in feature_types:
            wavelet_features = self.extract_wavelet_features(data)
            all_features.update(wavelet_features)
        
        # STFT特征
        if 'stft' in feature_types:
            stft_features = self.extract_stft_features(data)
            all_features.update(stft_features)
        
        return all_features
    
    def extract_features_from_multiple_sensors(self, 
                                             sensor_data: Dict[str, np.ndarray],
                                             feature_types: List[str] = None) -> Dict[str, Dict[str, float]]:
        """从多个传感器提取特征"""
        results = {}
        
        for sensor_id, data in sensor_data.items():
            try:
                features = self.extract_all_features(data, feature_types)
                results[sensor_id] = features
            except Exception as e:
                warnings.warn(f"传感器 {sensor_id} 特征提取失败: {e}")
                results[sensor_id] = {}
        
        return results
    
    def get_feature_importance(self, features: Dict[str, float], 
                             target: np.ndarray = None) -> Dict[str, float]:
        """计算特征重要性"""
        if target is None or len(features) == 0:
            return {}
        
        try:
            from sklearn.feature_selection import mutual_info_regression
            from sklearn.preprocessing import StandardScaler
            
            # 准备特征矩阵
            feature_names = list(features.keys())
            feature_values = np.array(list(features.values())).reshape(1, -1)
            
            # 如果只有一个样本，无法计算互信息
            if len(target) == 1:
                return {name: 1.0 for name in feature_names}
            
            # 标准化特征
            scaler = StandardScaler()
            feature_values_scaled = scaler.fit_transform(feature_values.T).T
            
            # 计算互信息
            mi_scores = mutual_info_regression(feature_values_scaled.T, target)
            
            # 归一化重要性分数
            if np.sum(mi_scores) > 0:
                mi_scores = mi_scores / np.sum(mi_scores)
            
            importance = dict(zip(feature_names, mi_scores))
            
            return importance
            
        except Exception as e:
            warnings.warn(f"特征重要性计算失败: {e}")
            return {}
```

## 应用场景

- **环境监测**: 温湿度、空气质量、噪声监测
- **工业4.0**: 设备状态监控、预测性维护
- **智能农业**: 土壤监测、作物生长环境分析
- **健康监护**: 生理参数监测、运动分析
- **智能建筑**: 能耗监测、舒适度控制
- **交通监控**: 车辆检测、交通流量分析

## 性能优化

### 1. 数据处理优化
- **流式处理**: 实时数据流处理
- **并行计算**: 多传感器并行分析
- **缓存机制**: 智能数据缓存策略

### 2. 算法优化
- **特征选择**: 自动特征选择和降维
- **模型压缩**: 轻量级机器学习模型
- **增量学习**: 在线模型更新

### 3. 硬件优化
- **边缘计算**: 本地化数据处理
- **低功耗设计**: 优化功耗管理
- **硬件加速**: 利用专用芯片加速

## 常见问题

### Q: 如何处理传感器数据噪声？
A: 
1. 使用数字滤波器（低通、高通、带通）
2. 应用数据平滑算法
3. 异常值检测和移除
4. 传感器校准和标定

### Q: 如何选择合适的特征？
A: 
1. 根据应用场景选择相关特征
2. 使用特征重要性分析
3. 应用降维技术
4. 交叉验证特征效果

### Q: 如何提高实时性能？
A: 
1. 优化数据处理算法
2. 使用轻量级模型
3. 并行处理多传感器数据
4. 硬件加速计算

## 扩展功能

- **传感器融合**: 多传感器数据融合分析
- **自适应校准**: 自动传感器校准
- **边缘智能**: 本地化机器学习
- **数字孪生**: 传感器数据驱动的数字孪生
- **预测分析**: 基于历史数据的趋势预测

欢迎贡献代码和提出改进建议！
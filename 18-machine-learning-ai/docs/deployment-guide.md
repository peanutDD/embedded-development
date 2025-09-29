# 部署最佳实践

## 概述

边缘AI模型的部署是一个复杂的过程，涉及模型转换、硬件适配、性能优化、监控管理等多个环节。本指南提供了从开发到生产的完整部署最佳实践。

## 部署架构

### 1. 分层架构

```python
import json
import logging
import time
from abc import ABC, abstractmethod
from typing import Dict, Any, List, Optional
from dataclasses import dataclass
from enum import Enum

class DeploymentStage(Enum):
    """部署阶段"""
    DEVELOPMENT = "development"
    TESTING = "testing"
    STAGING = "staging"
    PRODUCTION = "production"

class DeviceType(Enum):
    """设备类型"""
    CPU = "cpu"
    GPU = "gpu"
    NPU = "npu"
    TPU = "tpu"
    MOBILE = "mobile"
    EMBEDDED = "embedded"

@dataclass
class DeploymentConfig:
    """部署配置"""
    model_path: str
    model_format: str  # tflite, onnx, tensorrt, etc.
    device_type: DeviceType
    batch_size: int = 1
    max_latency_ms: float = 100.0
    memory_limit_mb: int = 512
    cpu_threads: int = 0
    gpu_memory_fraction: float = 0.8
    optimization_level: int = 1
    enable_profiling: bool = False
    log_level: str = "INFO"

class ModelInterface(ABC):
    """模型接口抽象类"""
    
    @abstractmethod
    def load_model(self, model_path: str) -> bool:
        """加载模型"""
        pass
    
    @abstractmethod
    def predict(self, input_data: Any) -> Any:
        """执行推理"""
        pass
    
    @abstractmethod
    def get_model_info(self) -> Dict[str, Any]:
        """获取模型信息"""
        pass
    
    @abstractmethod
    def cleanup(self):
        """清理资源"""
        pass

class EdgeAIDeploymentManager:
    """边缘AI部署管理器"""
    
    def __init__(self, config: DeploymentConfig):
        self.config = config
        self.model = None
        self.performance_monitor = PerformanceMonitor()
        self.health_checker = HealthChecker()
        self.logger = self.setup_logging()
        
    def setup_logging(self):
        """设置日志"""
        logging.basicConfig(
            level=getattr(logging, self.config.log_level),
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        return logging.getLogger(__name__)
    
    def deploy_model(self) -> bool:
        """部署模型"""
        try:
            self.logger.info("开始部署模型...")
            
            # 1. 验证配置
            if not self.validate_config():
                return False
            
            # 2. 创建模型实例
            self.model = self.create_model_instance()
            if not self.model:
                return False
            
            # 3. 加载模型
            if not self.model.load_model(self.config.model_path):
                return False
            
            # 4. 性能验证
            if not self.validate_performance():
                return False
            
            # 5. 启动监控
            self.start_monitoring()
            
            self.logger.info("模型部署成功")
            return True
            
        except Exception as e:
            self.logger.error(f"模型部署失败: {e}")
            return False
    
    def validate_config(self) -> bool:
        """验证配置"""
        try:
            # 检查模型文件
            import os
            if not os.path.exists(self.config.model_path):
                self.logger.error(f"模型文件不存在: {self.config.model_path}")
                return False
            
            # 检查设备可用性
            if not self.check_device_availability():
                return False
            
            # 检查资源限制
            if not self.check_resource_limits():
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"配置验证失败: {e}")
            return False
    
    def check_device_availability(self) -> bool:
        """检查设备可用性"""
        device_type = self.config.device_type
        
        if device_type == DeviceType.GPU:
            return self.check_gpu_availability()
        elif device_type == DeviceType.NPU:
            return self.check_npu_availability()
        elif device_type == DeviceType.TPU:
            return self.check_tpu_availability()
        else:
            return True  # CPU总是可用
    
    def check_gpu_availability(self) -> bool:
        """检查GPU可用性"""
        try:
            import tensorflow as tf
            gpus = tf.config.list_physical_devices('GPU')
            
            if not gpus:
                self.logger.warning("未检测到GPU设备")
                return False
            
            # 设置GPU内存增长
            for gpu in gpus:
                tf.config.experimental.set_memory_growth(gpu, True)
            
            self.logger.info(f"检测到 {len(gpus)} 个GPU设备")
            return True
            
        except Exception as e:
            self.logger.error(f"GPU检查失败: {e}")
            return False
    
    def check_npu_availability(self) -> bool:
        """检查NPU可用性"""
        # 这里需要根据具体的NPU SDK实现
        self.logger.info("NPU可用性检查")
        return True
    
    def check_tpu_availability(self) -> bool:
        """检查TPU可用性"""
        try:
            from pycoral.utils import edgetpu
            devices = edgetpu.list_edge_tpus()
            
            if not devices:
                self.logger.warning("未检测到Edge TPU设备")
                return False
            
            self.logger.info(f"检测到 {len(devices)} 个Edge TPU设备")
            return True
            
        except ImportError:
            self.logger.error("PyCoral库未安装")
            return False
    
    def check_resource_limits(self) -> bool:
        """检查资源限制"""
        import psutil
        
        # 检查内存
        available_memory = psutil.virtual_memory().available / (1024 * 1024)  # MB
        if available_memory < self.config.memory_limit_mb:
            self.logger.warning(f"可用内存不足: {available_memory:.0f}MB < {self.config.memory_limit_mb}MB")
            return False
        
        # 检查CPU
        cpu_count = psutil.cpu_count()
        if self.config.cpu_threads > cpu_count:
            self.logger.warning(f"CPU线程数超限: {self.config.cpu_threads} > {cpu_count}")
            self.config.cpu_threads = cpu_count
        
        return True
    
    def create_model_instance(self) -> Optional[ModelInterface]:
        """创建模型实例"""
        model_format = self.config.model_format.lower()
        
        if model_format == 'tflite':
            return TensorFlowLiteModel(self.config)
        elif model_format == 'onnx':
            return ONNXModel(self.config)
        elif model_format == 'tensorrt':
            return TensorRTModel(self.config)
        elif model_format == 'openvino':
            return OpenVINOModel(self.config)
        else:
            self.logger.error(f"不支持的模型格式: {model_format}")
            return None
    
    def validate_performance(self) -> bool:
        """验证性能"""
        if not self.model:
            return False
        
        try:
            # 创建测试数据
            model_info = self.model.get_model_info()
            input_shape = model_info.get('input_shape')
            
            if not input_shape:
                self.logger.warning("无法获取输入形状，跳过性能验证")
                return True
            
            test_data = self.create_test_data(input_shape)
            
            # 预热
            for _ in range(5):
                _ = self.model.predict(test_data)
            
            # 性能测试
            latencies = []
            num_runs = 20
            
            for _ in range(num_runs):
                start_time = time.time()
                _ = self.model.predict(test_data)
                end_time = time.time()
                
                latency_ms = (end_time - start_time) * 1000
                latencies.append(latency_ms)
            
            avg_latency = sum(latencies) / len(latencies)
            p95_latency = sorted(latencies)[int(0.95 * len(latencies))]
            
            self.logger.info(f"平均延迟: {avg_latency:.2f}ms")
            self.logger.info(f"P95延迟: {p95_latency:.2f}ms")
            
            # 检查是否满足延迟要求
            if p95_latency > self.config.max_latency_ms:
                self.logger.warning(f"延迟超限: {p95_latency:.2f}ms > {self.config.max_latency_ms}ms")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"性能验证失败: {e}")
            return False
    
    def create_test_data(self, input_shape):
        """创建测试数据"""
        import numpy as np
        
        if isinstance(input_shape, list) and len(input_shape) > 0:
            # 处理批次维度
            if input_shape[0] is None or input_shape[0] == -1:
                shape = [1] + input_shape[1:]
            else:
                shape = input_shape
            
            return np.random.random(shape).astype(np.float32)
        else:
            # 默认形状
            return np.random.random((1, 224, 224, 3)).astype(np.float32)
    
    def start_monitoring(self):
        """启动监控"""
        self.performance_monitor.start()
        self.health_checker.start()
        self.logger.info("监控系统已启动")
    
    def predict(self, input_data: Any) -> Dict[str, Any]:
        """执行推理"""
        if not self.model:
            raise RuntimeError("模型未部署")
        
        start_time = time.time()
        
        try:
            # 执行推理
            result = self.model.predict(input_data)
            
            end_time = time.time()
            latency_ms = (end_time - start_time) * 1000
            
            # 记录性能指标
            self.performance_monitor.record_inference(latency_ms)
            
            return {
                'result': result,
                'latency_ms': latency_ms,
                'timestamp': time.time()
            }
            
        except Exception as e:
            self.logger.error(f"推理失败: {e}")
            raise
    
    def get_status(self) -> Dict[str, Any]:
        """获取部署状态"""
        status = {
            'model_loaded': self.model is not None,
            'config': self.config.__dict__,
            'performance_stats': self.performance_monitor.get_stats(),
            'health_status': self.health_checker.get_status(),
            'system_resources': self.get_system_resources()
        }
        
        if self.model:
            status['model_info'] = self.model.get_model_info()
        
        return status
    
    def get_system_resources(self) -> Dict[str, Any]:
        """获取系统资源信息"""
        import psutil
        
        return {
            'cpu_usage_percent': psutil.cpu_percent(interval=1),
            'memory_usage_percent': psutil.virtual_memory().percent,
            'available_memory_mb': psutil.virtual_memory().available / (1024 * 1024),
            'disk_usage_percent': psutil.disk_usage('/').percent
        }
    
    def shutdown(self):
        """关闭部署"""
        self.logger.info("正在关闭部署...")
        
        if self.model:
            self.model.cleanup()
        
        self.performance_monitor.stop()
        self.health_checker.stop()
        
        self.logger.info("部署已关闭")
```

### 2. 模型适配器

```python
class TensorFlowLiteModel(ModelInterface):
    """TensorFlow Lite模型适配器"""
    
    def __init__(self, config: DeploymentConfig):
        self.config = config
        self.interpreter = None
        self.input_details = None
        self.output_details = None
    
    def load_model(self, model_path: str) -> bool:
        """加载TensorFlow Lite模型"""
        try:
            import tensorflow as tf
            
            # 创建解释器
            if self.config.device_type == DeviceType.GPU:
                # 尝试使用GPU委托
                try:
                    delegate = tf.lite.experimental.load_delegate('libdelegate_gpu.so')
                    self.interpreter = tf.lite.Interpreter(
                        model_path=model_path,
                        experimental_delegates=[delegate]
                    )
                except:
                    # 回退到CPU
                    self.interpreter = tf.lite.Interpreter(
                        model_path=model_path,
                        num_threads=self.config.cpu_threads
                    )
            elif self.config.device_type == DeviceType.TPU:
                # Edge TPU委托
                try:
                    from pycoral.utils import edgetpu
                    self.interpreter = edgetpu.make_interpreter(model_path)
                except ImportError:
                    return False
            else:
                # CPU
                self.interpreter = tf.lite.Interpreter(
                    model_path=model_path,
                    num_threads=self.config.cpu_threads
                )
            
            self.interpreter.allocate_tensors()
            
            # 获取输入输出详情
            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()
            
            return True
            
        except Exception as e:
            print(f"TensorFlow Lite模型加载失败: {e}")
            return False
    
    def predict(self, input_data: Any) -> Any:
        """执行推理"""
        if not self.interpreter:
            raise RuntimeError("模型未加载")
        
        # 设置输入数据
        for i, input_detail in enumerate(self.input_details):
            if isinstance(input_data, list):
                self.interpreter.set_tensor(input_detail['index'], input_data[i])
            else:
                self.interpreter.set_tensor(input_detail['index'], input_data)
        
        # 执行推理
        self.interpreter.invoke()
        
        # 获取输出结果
        outputs = []
        for output_detail in self.output_details:
            output = self.interpreter.get_tensor(output_detail['index'])
            outputs.append(output)
        
        return outputs[0] if len(outputs) == 1 else outputs
    
    def get_model_info(self) -> Dict[str, Any]:
        """获取模型信息"""
        if not self.interpreter:
            return {}
        
        info = {
            'input_details': [],
            'output_details': [],
            'model_size_bytes': 0
        }
        
        for input_detail in self.input_details:
            info['input_details'].append({
                'name': input_detail['name'],
                'shape': input_detail['shape'].tolist(),
                'dtype': str(input_detail['dtype'])
            })
        
        for output_detail in self.output_details:
            info['output_details'].append({
                'name': output_detail['name'],
                'shape': output_detail['shape'].tolist(),
                'dtype': str(output_detail['dtype'])
            })
        
        # 获取模型大小
        import os
        if os.path.exists(self.config.model_path):
            info['model_size_bytes'] = os.path.getsize(self.config.model_path)
        
        return info
    
    def cleanup(self):
        """清理资源"""
        if self.interpreter:
            del self.interpreter
            self.interpreter = None

class ONNXModel(ModelInterface):
    """ONNX模型适配器"""
    
    def __init__(self, config: DeploymentConfig):
        self.config = config
        self.session = None
        self.input_names = None
        self.output_names = None
    
    def load_model(self, model_path: str) -> bool:
        """加载ONNX模型"""
        try:
            import onnxruntime as ort
            
            # 设置会话选项
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            
            if self.config.cpu_threads > 0:
                sess_options.intra_op_num_threads = self.config.cpu_threads
            
            # 设置执行提供者
            providers = ['CPUExecutionProvider']
            
            if self.config.device_type == DeviceType.GPU:
                providers.insert(0, 'CUDAExecutionProvider')
            elif self.config.device_type == DeviceType.NPU:
                # 添加NPU提供者（如果可用）
                providers.insert(0, 'NPUExecutionProvider')
            
            # 创建推理会话
            self.session = ort.InferenceSession(
                model_path,
                sess_options=sess_options,
                providers=providers
            )
            
            # 获取输入输出名称
            self.input_names = [input.name for input in self.session.get_inputs()]
            self.output_names = [output.name for output in self.session.get_outputs()]
            
            return True
            
        except Exception as e:
            print(f"ONNX模型加载失败: {e}")
            return False
    
    def predict(self, input_data: Any) -> Any:
        """执行推理"""
        if not self.session:
            raise RuntimeError("模型未加载")
        
        # 准备输入数据
        if isinstance(input_data, dict):
            input_dict = input_data
        elif isinstance(input_data, list):
            input_dict = {name: data for name, data in zip(self.input_names, input_data)}
        else:
            input_dict = {self.input_names[0]: input_data}
        
        # 执行推理
        outputs = self.session.run(self.output_names, input_dict)
        
        return outputs[0] if len(outputs) == 1 else outputs
    
    def get_model_info(self) -> Dict[str, Any]:
        """获取模型信息"""
        if not self.session:
            return {}
        
        info = {
            'input_details': [],
            'output_details': [],
            'providers': self.session.get_providers()
        }
        
        for input_meta in self.session.get_inputs():
            info['input_details'].append({
                'name': input_meta.name,
                'shape': input_meta.shape,
                'dtype': input_meta.type
            })
        
        for output_meta in self.session.get_outputs():
            info['output_details'].append({
                'name': output_meta.name,
                'shape': output_meta.shape,
                'dtype': output_meta.type
            })
        
        return info
    
    def cleanup(self):
        """清理资源"""
        if self.session:
            del self.session
            self.session = None

class TensorRTModel(ModelInterface):
    """TensorRT模型适配器"""
    
    def __init__(self, config: DeploymentConfig):
        self.config = config
        self.engine = None
        self.context = None
        self.bindings = None
        self.stream = None
    
    def load_model(self, model_path: str) -> bool:
        """加载TensorRT模型"""
        try:
            import tensorrt as trt
            import pycuda.driver as cuda
            import pycuda.autoinit
            
            # 创建logger
            TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
            
            # 加载引擎
            with open(model_path, 'rb') as f:
                engine_data = f.read()
            
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(engine_data)
            
            if not self.engine:
                return False
            
            # 创建执行上下文
            self.context = self.engine.create_execution_context()
            
            # 创建CUDA流
            self.stream = cuda.Stream()
            
            # 准备绑定
            self.setup_bindings()
            
            return True
            
        except Exception as e:
            print(f"TensorRT模型加载失败: {e}")
            return False
    
    def setup_bindings(self):
        """设置绑定"""
        import pycuda.driver as cuda
        
        self.bindings = []
        self.inputs = []
        self.outputs = []
        
        for i in range(self.engine.num_bindings):
            binding_name = self.engine.get_binding_name(i)
            size = trt.volume(self.engine.get_binding_shape(i)) * self.engine.max_batch_size
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            
            # 分配设备内存
            device_mem = cuda.mem_alloc(size * dtype().itemsize)
            self.bindings.append(int(device_mem))
            
            if self.engine.binding_is_input(i):
                self.inputs.append({
                    'name': binding_name,
                    'device_mem': device_mem,
                    'size': size,
                    'dtype': dtype
                })
            else:
                self.outputs.append({
                    'name': binding_name,
                    'device_mem': device_mem,
                    'size': size,
                    'dtype': dtype
                })
    
    def predict(self, input_data: Any) -> Any:
        """执行推理"""
        if not self.context:
            raise RuntimeError("模型未加载")
        
        import pycuda.driver as cuda
        import numpy as np
        
        # 复制输入数据到GPU
        if isinstance(input_data, list):
            for i, (data, input_info) in enumerate(zip(input_data, self.inputs)):
                cuda.memcpy_htod_async(input_info['device_mem'], data, self.stream)
        else:
            cuda.memcpy_htod_async(self.inputs[0]['device_mem'], input_data, self.stream)
        
        # 执行推理
        self.context.execute_async_v2(bindings=self.bindings, stream_handle=self.stream.handle)
        
        # 复制输出数据到CPU
        outputs = []
        for output_info in self.outputs:
            output = np.empty(output_info['size'], dtype=output_info['dtype'])
            cuda.memcpy_dtoh_async(output, output_info['device_mem'], self.stream)
            outputs.append(output)
        
        # 同步流
        self.stream.synchronize()
        
        return outputs[0] if len(outputs) == 1 else outputs
    
    def get_model_info(self) -> Dict[str, Any]:
        """获取模型信息"""
        if not self.engine:
            return {}
        
        info = {
            'max_batch_size': self.engine.max_batch_size,
            'num_bindings': self.engine.num_bindings,
            'input_details': [],
            'output_details': []
        }
        
        for input_info in self.inputs:
            info['input_details'].append({
                'name': input_info['name'],
                'size': input_info['size'],
                'dtype': str(input_info['dtype'])
            })
        
        for output_info in self.outputs:
            info['output_details'].append({
                'name': output_info['name'],
                'size': output_info['size'],
                'dtype': str(output_info['dtype'])
            })
        
        return info
    
    def cleanup(self):
        """清理资源"""
        if self.context:
            del self.context
        if self.engine:
            del self.engine
        if self.stream:
            del self.stream
```

### 3. 性能监控

```python
import threading
import time
from collections import deque
from typing import Dict, List

class PerformanceMonitor:
    """性能监控器"""
    
    def __init__(self, window_size: int = 1000):
        self.window_size = window_size
        self.latencies = deque(maxlen=window_size)
        self.throughput_window = deque(maxlen=60)  # 1分钟窗口
        self.error_count = 0
        self.total_requests = 0
        self.start_time = time.time()
        self.lock = threading.Lock()
        self.monitoring = False
        self.monitor_thread = None
    
    def start(self):
        """启动监控"""
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
    
    def stop(self):
        """停止监控"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join()
    
    def record_inference(self, latency_ms: float, success: bool = True):
        """记录推理指标"""
        with self.lock:
            self.latencies.append(latency_ms)
            self.total_requests += 1
            
            if not success:
                self.error_count += 1
    
    def _monitor_loop(self):
        """监控循环"""
        last_request_count = 0
        
        while self.monitoring:
            time.sleep(1)  # 每秒更新一次
            
            with self.lock:
                current_requests = self.total_requests
                throughput = current_requests - last_request_count
                self.throughput_window.append(throughput)
                last_request_count = current_requests
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self.lock:
            if not self.latencies:
                return {
                    'total_requests': 0,
                    'error_rate': 0.0,
                    'avg_latency_ms': 0.0,
                    'p95_latency_ms': 0.0,
                    'p99_latency_ms': 0.0,
                    'throughput_rps': 0.0,
                    'uptime_seconds': time.time() - self.start_time
                }
            
            latencies_sorted = sorted(self.latencies)
            n = len(latencies_sorted)
            
            stats = {
                'total_requests': self.total_requests,
                'error_count': self.error_count,
                'error_rate': self.error_count / max(self.total_requests, 1),
                'avg_latency_ms': sum(self.latencies) / len(self.latencies),
                'min_latency_ms': min(self.latencies),
                'max_latency_ms': max(self.latencies),
                'p50_latency_ms': latencies_sorted[int(0.5 * n)],
                'p95_latency_ms': latencies_sorted[int(0.95 * n)],
                'p99_latency_ms': latencies_sorted[int(0.99 * n)],
                'throughput_rps': sum(self.throughput_window) / max(len(self.throughput_window), 1),
                'uptime_seconds': time.time() - self.start_time
            }
            
            return stats

class HealthChecker:
    """健康检查器"""
    
    def __init__(self, check_interval: int = 30):
        self.check_interval = check_interval
        self.health_status = {
            'status': 'unknown',
            'last_check': None,
            'checks': {}
        }
        self.checking = False
        self.check_thread = None
    
    def start(self):
        """启动健康检查"""
        self.checking = True
        self.check_thread = threading.Thread(target=self._check_loop)
        self.check_thread.daemon = True
        self.check_thread.start()
    
    def stop(self):
        """停止健康检查"""
        self.checking = False
        if self.check_thread:
            self.check_thread.join()
    
    def _check_loop(self):
        """检查循环"""
        while self.checking:
            self.perform_health_check()
            time.sleep(self.check_interval)
    
    def perform_health_check(self):
        """执行健康检查"""
        checks = {
            'memory': self.check_memory(),
            'disk': self.check_disk(),
            'cpu': self.check_cpu(),
            'model': self.check_model_health()
        }
        
        # 确定整体状态
        all_healthy = all(check['status'] == 'healthy' for check in checks.values())
        overall_status = 'healthy' if all_healthy else 'unhealthy'
        
        self.health_status = {
            'status': overall_status,
            'last_check': time.time(),
            'checks': checks
        }
    
    def check_memory(self) -> Dict[str, Any]:
        """检查内存使用"""
        import psutil
        
        memory = psutil.virtual_memory()
        usage_percent = memory.percent
        
        status = 'healthy' if usage_percent < 90 else 'unhealthy'
        
        return {
            'status': status,
            'usage_percent': usage_percent,
            'available_mb': memory.available / (1024 * 1024)
        }
    
    def check_disk(self) -> Dict[str, Any]:
        """检查磁盘使用"""
        import psutil
        
        disk = psutil.disk_usage('/')
        usage_percent = (disk.used / disk.total) * 100
        
        status = 'healthy' if usage_percent < 95 else 'unhealthy'
        
        return {
            'status': status,
            'usage_percent': usage_percent,
            'free_gb': disk.free / (1024 ** 3)
        }
    
    def check_cpu(self) -> Dict[str, Any]:
        """检查CPU使用"""
        import psutil
        
        cpu_percent = psutil.cpu_percent(interval=1)
        
        status = 'healthy' if cpu_percent < 95 else 'unhealthy'
        
        return {
            'status': status,
            'usage_percent': cpu_percent,
            'load_average': psutil.getloadavg() if hasattr(psutil, 'getloadavg') else None
        }
    
    def check_model_health(self) -> Dict[str, Any]:
        """检查模型健康状态"""
        # 这里可以添加模型特定的健康检查
        # 例如：测试推理、检查模型文件完整性等
        
        return {
            'status': 'healthy',
            'message': 'Model is responsive'
        }
    
    def get_status(self) -> Dict[str, Any]:
        """获取健康状态"""
        return self.health_status.copy()
```

## 部署流程

### 1. 自动化部署

```python
class AutoDeploymentPipeline:
    """自动化部署流水线"""
    
    def __init__(self, config_path: str):
        self.config = self.load_config(config_path)
        self.deployment_manager = None
        self.logger = logging.getLogger(__name__)
    
    def load_config(self, config_path: str) -> Dict[str, Any]:
        """加载配置文件"""
        with open(config_path, 'r') as f:
            return json.load(f)
    
    def validate_environment(self) -> bool:
        """验证部署环境"""
        try:
            # 检查依赖
            required_packages = self.config.get('required_packages', [])
            for package in required_packages:
                try:
                    __import__(package)
                except ImportError:
                    self.logger.error(f"缺少依赖包: {package}")
                    return False
            
            # 检查硬件
            device_type = DeviceType(self.config['deployment']['device_type'])
            if not self.check_hardware_compatibility(device_type):
                return False
            
            # 检查模型文件
            model_path = self.config['model']['path']
            if not os.path.exists(model_path):
                self.logger.error(f"模型文件不存在: {model_path}")
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"环境验证失败: {e}")
            return False
    
    def check_hardware_compatibility(self, device_type: DeviceType) -> bool:
        """检查硬件兼容性"""
        if device_type == DeviceType.GPU:
            try:
                import tensorflow as tf
                gpus = tf.config.list_physical_devices('GPU')
                return len(gpus) > 0
            except:
                return False
        
        elif device_type == DeviceType.TPU:
            try:
                from pycoral.utils import edgetpu
                devices = edgetpu.list_edge_tpus()
                return len(devices) > 0
            except:
                return False
        
        return True  # CPU总是兼容
    
    def prepare_model(self) -> bool:
        """准备模型"""
        try:
            model_config = self.config['model']
            
            # 模型优化
            if model_config.get('optimize', False):
                optimized_path = self.optimize_model(
                    model_config['path'],
                    model_config.get('optimization_config', {})
                )
                model_config['path'] = optimized_path
            
            # 模型验证
            if not self.validate_model(model_config['path']):
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"模型准备失败: {e}")
            return False
    
    def optimize_model(self, model_path: str, optimization_config: Dict[str, Any]) -> str:
        """优化模型"""
        # 这里实现模型优化逻辑
        # 例如：量化、剪枝等
        
        optimized_path = model_path.replace('.', '_optimized.')
        
        # 示例：TensorFlow Lite量化
        if model_path.endswith('.tflite') and optimization_config.get('quantize', False):
            # 实现量化逻辑
            pass
        
        return optimized_path
    
    def validate_model(self, model_path: str) -> bool:
        """验证模型"""
        try:
            # 尝试加载模型
            deployment_config = DeploymentConfig(
                model_path=model_path,
                model_format=self.config['model']['format'],
                device_type=DeviceType(self.config['deployment']['device_type'])
            )
            
            temp_manager = EdgeAIDeploymentManager(deployment_config)
            model = temp_manager.create_model_instance()
            
            if not model or not model.load_model(model_path):
                return False
            
            # 测试推理
            model_info = model.get_model_info()
            if model_info.get('input_details'):
                input_shape = model_info['input_details'][0]['shape']
                test_data = temp_manager.create_test_data(input_shape)
                result = model.predict(test_data)
                
                if result is None:
                    return False
            
            model.cleanup()
            return True
            
        except Exception as e:
            self.logger.error(f"模型验证失败: {e}")
            return False
    
    def deploy(self) -> bool:
        """执行部署"""
        try:
            # 1. 验证环境
            if not self.validate_environment():
                return False
            
            # 2. 准备模型
            if not self.prepare_model():
                return False
            
            # 3. 创建部署配置
            deployment_config = self.create_deployment_config()
            
            # 4. 创建部署管理器
            self.deployment_manager = EdgeAIDeploymentManager(deployment_config)
            
            # 5. 部署模型
            if not self.deployment_manager.deploy_model():
                return False
            
            # 6. 运行部署后测试
            if not self.run_post_deployment_tests():
                return False
            
            self.logger.info("部署成功完成")
            return True
            
        except Exception as e:
            self.logger.error(f"部署失败: {e}")
            return False
    
    def create_deployment_config(self) -> DeploymentConfig:
        """创建部署配置"""
        model_config = self.config['model']
        deployment_config = self.config['deployment']
        
        return DeploymentConfig(
            model_path=model_config['path'],
            model_format=model_config['format'],
            device_type=DeviceType(deployment_config['device_type']),
            batch_size=deployment_config.get('batch_size', 1),
            max_latency_ms=deployment_config.get('max_latency_ms', 100.0),
            memory_limit_mb=deployment_config.get('memory_limit_mb', 512),
            cpu_threads=deployment_config.get('cpu_threads', 0),
            optimization_level=deployment_config.get('optimization_level', 1),
            enable_profiling=deployment_config.get('enable_profiling', False),
            log_level=deployment_config.get('log_level', 'INFO')
        )
    
    def run_post_deployment_tests(self) -> bool:
        """运行部署后测试"""
        try:
            test_config = self.config.get('tests', {})
            
            # 性能测试
            if test_config.get('performance_test', True):
                if not self.run_performance_test():
                    return False
            
            # 功能测试
            if test_config.get('functional_test', True):
                if not self.run_functional_test():
                    return False
            
            # 负载测试
            if test_config.get('load_test', False):
                if not self.run_load_test():
                    return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"部署后测试失败: {e}")
            return False
    
    def run_performance_test(self) -> bool:
        """运行性能测试"""
        try:
            # 创建测试数据
            model_info = self.deployment_manager.model.get_model_info()
            input_shape = model_info['input_details'][0]['shape']
            test_data = self.deployment_manager.create_test_data(input_shape)
            
            # 性能测试
            latencies = []
            num_runs = 50
            
            for _ in range(num_runs):
                result = self.deployment_manager.predict(test_data)
                latencies.append(result['latency_ms'])
            
            # 分析结果
            avg_latency = sum(latencies) / len(latencies)
            p95_latency = sorted(latencies)[int(0.95 * len(latencies))]
            
            max_latency = self.deployment_manager.config.max_latency_ms
            
            if p95_latency > max_latency:
                self.logger.error(f"性能测试失败: P95延迟 {p95_latency:.2f}ms > {max_latency}ms")
                return False
            
            self.logger.info(f"性能测试通过: 平均延迟 {avg_latency:.2f}ms, P95延迟 {p95_latency:.2f}ms")
            return True
            
        except Exception as e:
            self.logger.error(f"性能测试失败: {e}")
            return False
    
    def run_functional_test(self) -> bool:
        """运行功能测试"""
        try:
            # 测试基本推理功能
            model_info = self.deployment_manager.model.get_model_info()
            input_shape = model_info['input_details'][0]['shape']
            test_data = self.deployment_manager.create_test_data(input_shape)
            
            result = self.deployment_manager.predict(test_data)
            
            if result is None or 'result' not in result:
                self.logger.error("功能测试失败: 推理返回空结果")
                return False
            
            # 测试状态获取
            status = self.deployment_manager.get_status()
            if not status['model_loaded']:
                self.logger.error("功能测试失败: 模型状态异常")
                return False
            
            self.logger.info("功能测试通过")
            return True
            
        except Exception as e:
            self.logger.error(f"功能测试失败: {e}")
            return False
    
    def run_load_test(self) -> bool:
        """运行负载测试"""
        try:
            import concurrent.futures
            import threading
            
            # 负载测试参数
            num_threads = 10
            requests_per_thread = 20
            
            # 创建测试数据
            model_info = self.deployment_manager.model.get_model_info()
            input_shape = model_info['input_details'][0]['shape']
            test_data = self.deployment_manager.create_test_data(input_shape)
            
            def worker():
                """工作线程"""
                results = []
                for _ in range(requests_per_thread):
                    try:
                        result = self.deployment_manager.predict(test_data)
                        results.append(result['latency_ms'])
                    except Exception as e:
                        results.append(None)
                return results
            
            # 执行负载测试
            with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
                futures = [executor.submit(worker) for _ in range(num_threads)]
                
                all_results = []
                for future in concurrent.futures.as_completed(futures):
                    thread_results = future.result()
                    all_results.extend(thread_results)
            
            # 分析结果
            successful_requests = [r for r in all_results if r is not None]
            success_rate = len(successful_requests) / len(all_results)
            
            if success_rate < 0.95:  # 95%成功率阈值
                self.logger.error(f"负载测试失败: 成功率 {success_rate:.2%} < 95%")
                return False
            
            avg_latency = sum(successful_requests) / len(successful_requests)
            self.logger.info(f"负载测试通过: 成功率 {success_rate:.2%}, 平均延迟 {avg_latency:.2f}ms")
            
            return True
            
        except Exception as e:
            self.logger.error(f"负载测试失败: {e}")
            return False
    
    def get_deployment_status(self) -> Dict[str, Any]:
        """获取部署状态"""
        if self.deployment_manager:
            return self.deployment_manager.get_status()
        else:
            return {'status': 'not_deployed'}
    
    def shutdown(self):
        """关闭部署"""
        if self.deployment_manager:
            self.deployment_manager.shutdown()

# 使用示例
if __name__ == "__main__":
    # 配置文件示例
    config = {
        "model": {
            "path": "model.tflite",
            "format": "tflite",
            "optimize": True,
            "optimization_config": {
                "quantize": True
            }
        },
        "deployment": {
            "device_type": "cpu",
            "batch_size": 1,
            "max_latency_ms": 50.0,
            "memory_limit_mb": 256,
            "cpu_threads": 4,
            "optimization_level": 2,
            "enable_profiling": False,
            "log_level": "INFO"
        },
        "tests": {
            "performance_test": True,
            "functional_test": True,
            "load_test": True
        },
        "required_packages": [
            "tensorflow",
            "numpy",
            "psutil"
        ]
    }
    
    # 保存配置文件
    with open('deployment_config.json', 'w') as f:
        json.dump(config, f, indent=2)
    
    # 执行部署
    pipeline = AutoDeploymentPipeline('deployment_config.json')
    
    if pipeline.deploy():
        print("部署成功!")
        
        # 获取状态
        status = pipeline.get_deployment_status()
        print(f"部署状态: {json.dumps(status, indent=2)}")
        
        # 运行一段时间后关闭
        time.sleep(60)
        pipeline.shutdown()
    else:
        print("部署失败!")
```

## 总结

边缘AI部署的最佳实践包括：

1. **分层架构设计**: 清晰的接口抽象和模块化设计
2. **多格式支持**: 支持TensorFlow Lite、ONNX、TensorRT等多种模型格式
3. **硬件适配**: 针对不同硬件平台的优化适配
4. **性能监控**: 实时监控推理性能和系统健康状态
5. **自动化部署**: 完整的自动化部署流水线
6. **测试验证**: 全面的功能、性能和负载测试
7. **配置管理**: 灵活的配置管理和环境适配

通过遵循这些最佳实践，可以构建稳定、高效、可维护的边缘AI部署系统。
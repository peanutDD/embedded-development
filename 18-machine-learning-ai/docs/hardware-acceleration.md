# 硬件加速指南

## 概述

硬件加速是边缘AI性能优化的关键技术，通过利用专用硬件（GPU、NPU、TPU、DSP等）来加速机器学习推理。本指南详细介绍了各种硬件加速方案的特点、使用方法和优化策略。

## 硬件加速平台

### 1. GPU加速

GPU（图形处理单元）具有大量并行计算核心，非常适合深度学习的矩阵运算。

#### NVIDIA GPU加速

```python
import tensorflow as tf
import numpy as np
import time

class NVIDIAGPUAccelerator:
    """NVIDIA GPU加速器"""
    
    def __init__(self):
        self.setup_gpu()
        self.available_gpus = self.get_available_gpus()
        
    def setup_gpu(self):
        """设置GPU配置"""
        gpus = tf.config.list_physical_devices('GPU')
        if gpus:
            try:
                # 启用内存增长
                for gpu in gpus:
                    tf.config.experimental.set_memory_growth(gpu, True)
                
                # 设置虚拟GPU（可选）
                tf.config.experimental.set_virtual_device_configuration(
                    gpus[0],
                    [tf.config.experimental.VirtualDeviceConfiguration(memory_limit=1024)]
                )
                
                print(f"GPU配置完成，可用GPU数量: {len(gpus)}")
                
            except RuntimeError as e:
                print(f"GPU配置错误: {e}")
        else:
            print("未检测到可用GPU")
    
    def get_available_gpus(self):
        """获取可用GPU信息"""
        gpus = tf.config.list_physical_devices('GPU')
        gpu_info = []
        
        for i, gpu in enumerate(gpus):
            gpu_details = tf.config.experimental.get_device_details(gpu)
            gpu_info.append({
                'id': i,
                'name': gpu_details.get('device_name', 'Unknown'),
                'compute_capability': gpu_details.get('compute_capability', 'Unknown'),
                'memory_limit': gpu_details.get('memory_limit', 'Unknown')
            })
        
        return gpu_info
    
    def create_gpu_model(self, model_architecture):
        """创建GPU优化模型"""
        with tf.device('/GPU:0'):
            model = tf.keras.Sequential([
                tf.keras.layers.Dense(512, activation='relu', input_shape=(784,)),
                tf.keras.layers.Dropout(0.2),
                tf.keras.layers.Dense(256, activation='relu'),
                tf.keras.layers.Dropout(0.2),
                tf.keras.layers.Dense(10, activation='softmax')
            ])
            
            # 使用混合精度训练
            model.compile(
                optimizer=tf.keras.optimizers.Adam(),
                loss='sparse_categorical_crossentropy',
                metrics=['accuracy']
            )
        
        return model
    
    def benchmark_gpu_inference(self, model, test_data, batch_sizes=[1, 8, 16, 32]):
        """GPU推理性能测试"""
        results = {}
        
        for batch_size in batch_sizes:
            # 准备测试数据
            test_batch = test_data[:batch_size]
            
            # 预热
            for _ in range(10):
                _ = model.predict(test_batch, verbose=0)
            
            # 性能测试
            start_time = time.time()
            num_runs = 100
            
            for _ in range(num_runs):
                predictions = model.predict(test_batch, verbose=0)
            
            end_time = time.time()
            
            total_time = end_time - start_time
            avg_time_per_batch = total_time / num_runs * 1000  # ms
            throughput = (batch_size * num_runs) / total_time  # samples/sec
            
            results[batch_size] = {
                'avg_latency_ms': avg_time_per_batch,
                'throughput_samples_per_sec': throughput,
                'memory_usage_mb': self.get_gpu_memory_usage()
            }
        
        return results
    
    def get_gpu_memory_usage(self):
        """获取GPU内存使用情况"""
        try:
            import nvidia_ml_py3 as nvml
            nvml.nvmlInit()
            
            handle = nvml.nvmlDeviceGetHandleByIndex(0)
            memory_info = nvml.nvmlDeviceGetMemoryInfo(handle)
            
            return {
                'used_mb': memory_info.used / (1024 * 1024),
                'free_mb': memory_info.free / (1024 * 1024),
                'total_mb': memory_info.total / (1024 * 1024)
            }
        except:
            return {'error': 'Unable to get GPU memory info'}
    
    def optimize_for_inference(self, model):
        """推理优化"""
        # 转换为TensorRT
        try:
            from tensorflow.python.compiler.tensorrt import trt_convert as trt
            
            converter = trt.TrtGraphConverterV2(
                input_saved_model_dir=None,
                input_saved_model_tags=None,
                input_saved_model_signature_key=None,
                use_dynamic_shape=True,
                dynamic_shape_profile_strategy='Range',
                max_workspace_size_bytes=1 << 30,  # 1GB
                precision_mode=trt.TrtPrecisionMode.FP16,
                minimum_segment_size=3,
                maximum_cached_engines=1,
                use_calibration=False
            )
            
            # 这里需要保存模型后再转换
            model.save('temp_model')
            converter = trt.TrtGraphConverterV2(input_saved_model_dir='temp_model')
            converter.convert()
            
            optimized_model = converter.save('optimized_model')
            
            return optimized_model
            
        except ImportError:
            print("TensorRT未安装，跳过TensorRT优化")
            return model

# ARM Mali GPU加速
class ARMMaliGPUAccelerator:
    """ARM Mali GPU加速器"""
    
    def __init__(self):
        self.setup_opencl()
    
    def setup_opencl(self):
        """设置OpenCL环境"""
        try:
            import pyopencl as cl
            
            # 获取平台和设备
            platforms = cl.get_platforms()
            self.devices = []
            
            for platform in platforms:
                devices = platform.get_devices(cl.device_type.GPU)
                self.devices.extend(devices)
            
            if self.devices:
                self.context = cl.Context(self.devices)
                self.queue = cl.CommandQueue(self.context)
                print(f"OpenCL GPU设备数量: {len(self.devices)}")
            else:
                print("未找到OpenCL GPU设备")
                
        except ImportError:
            print("PyOpenCL未安装")
    
    def create_tflite_gpu_delegate(self):
        """创建TensorFlow Lite GPU委托"""
        try:
            # ARM GPU委托
            delegate = tf.lite.experimental.load_delegate('libdelegate_gpu.so')
            return delegate
        except:
            print("GPU委托加载失败，使用CPU")
            return None
    
    def optimize_model_for_mali(self, model_path):
        """为Mali GPU优化模型"""
        # 加载模型
        interpreter = tf.lite.Interpreter(model_path=model_path)
        
        # 尝试使用GPU委托
        gpu_delegate = self.create_tflite_gpu_delegate()
        if gpu_delegate:
            interpreter = tf.lite.Interpreter(
                model_path=model_path,
                experimental_delegates=[gpu_delegate]
            )
        
        interpreter.allocate_tensors()
        return interpreter
```

### 2. NPU加速

NPU（神经网络处理单元）是专门为AI推理设计的处理器。

#### 华为昇腾NPU

```python
class HuaweiAscendNPU:
    """华为昇腾NPU加速器"""
    
    def __init__(self, device_id=0):
        self.device_id = device_id
        self.setup_ascend()
    
    def setup_ascend(self):
        """设置昇腾环境"""
        try:
            import acl
            
            # 初始化ACL
            ret = acl.init()
            if ret != 0:
                raise RuntimeError(f"ACL初始化失败: {ret}")
            
            # 设置设备
            ret = acl.rt.set_device(self.device_id)
            if ret != 0:
                raise RuntimeError(f"设备设置失败: {ret}")
            
            # 创建上下文
            self.context, ret = acl.rt.create_context(self.device_id)
            if ret != 0:
                raise RuntimeError(f"上下文创建失败: {ret}")
            
            print(f"昇腾NPU {self.device_id} 初始化成功")
            
        except ImportError:
            print("昇腾ACL库未安装")
    
    def load_om_model(self, model_path):
        """加载OM模型"""
        try:
            import acl
            
            # 加载模型
            model_id, ret = acl.mdl.load_from_file(model_path)
            if ret != 0:
                raise RuntimeError(f"模型加载失败: {ret}")
            
            # 获取模型描述
            model_desc = acl.mdl.create_desc()
            ret = acl.mdl.get_desc(model_desc, model_id)
            if ret != 0:
                raise RuntimeError(f"模型描述获取失败: {ret}")
            
            return {
                'model_id': model_id,
                'model_desc': model_desc,
                'input_num': acl.mdl.get_num_inputs(model_desc),
                'output_num': acl.mdl.get_num_outputs(model_desc)
            }
            
        except Exception as e:
            print(f"OM模型加载错误: {e}")
            return None
    
    def create_data_buffer(self, size):
        """创建数据缓冲区"""
        try:
            import acl
            
            # 在设备上分配内存
            device_ptr, ret = acl.rt.malloc(size, acl.rt.ACL_MEM_MALLOC_HUGE_FIRST)
            if ret != 0:
                raise RuntimeError(f"设备内存分配失败: {ret}")
            
            return device_ptr
            
        except Exception as e:
            print(f"缓冲区创建错误: {e}")
            return None
    
    def inference(self, model_info, input_data):
        """执行推理"""
        try:
            import acl
            
            model_id = model_info['model_id']
            model_desc = model_info['model_desc']
            
            # 准备输入数据
            input_dataset = acl.mdl.create_dataset()
            
            for i in range(model_info['input_num']):
                input_size = acl.mdl.get_input_size_by_index(model_desc, i)
                input_buffer = self.create_data_buffer(input_size)
                
                # 复制数据到设备
                ret = acl.rt.memcpy(input_buffer, input_size, 
                                   input_data[i].ctypes.data, input_size,
                                   acl.rt.ACL_MEMCPY_HOST_TO_DEVICE)
                if ret != 0:
                    raise RuntimeError(f"数据复制失败: {ret}")
                
                data_buffer = acl.create_data_buffer(input_buffer, input_size)
                acl.mdl.add_dataset_buffer(input_dataset, data_buffer)
            
            # 准备输出数据
            output_dataset = acl.mdl.create_dataset()
            output_buffers = []
            
            for i in range(model_info['output_num']):
                output_size = acl.mdl.get_output_size_by_index(model_desc, i)
                output_buffer = self.create_data_buffer(output_size)
                output_buffers.append(output_buffer)
                
                data_buffer = acl.create_data_buffer(output_buffer, output_size)
                acl.mdl.add_dataset_buffer(output_dataset, data_buffer)
            
            # 执行推理
            ret = acl.mdl.execute(model_id, input_dataset, output_dataset)
            if ret != 0:
                raise RuntimeError(f"推理执行失败: {ret}")
            
            # 获取输出结果
            results = []
            for i, output_buffer in enumerate(output_buffers):
                output_size = acl.mdl.get_output_size_by_index(model_desc, i)
                output_data = np.zeros(output_size, dtype=np.uint8)
                
                ret = acl.rt.memcpy(output_data.ctypes.data, output_size,
                                   output_buffer, output_size,
                                   acl.rt.ACL_MEMCPY_DEVICE_TO_HOST)
                if ret != 0:
                    raise RuntimeError(f"结果复制失败: {ret}")
                
                results.append(output_data)
            
            # 清理资源
            acl.mdl.destroy_dataset(input_dataset)
            acl.mdl.destroy_dataset(output_dataset)
            
            return results
            
        except Exception as e:
            print(f"推理执行错误: {e}")
            return None

# 高通Hexagon DSP
class QualcommHexagonDSP:
    """高通Hexagon DSP加速器"""
    
    def __init__(self):
        self.setup_hexagon()
    
    def setup_hexagon(self):
        """设置Hexagon DSP环境"""
        try:
            # 这里需要高通的SNPE SDK
            import snpe
            
            self.snpe_available = True
            print("Hexagon DSP环境设置成功")
            
        except ImportError:
            print("SNPE SDK未安装")
            self.snpe_available = False
    
    def load_dlc_model(self, model_path):
        """加载DLC模型"""
        if not self.snpe_available:
            return None
        
        try:
            import snpe
            
            # 创建SNPE网络
            container = snpe.modeltools.load_container(model_path)
            
            # 设置运行时
            runtime = snpe.snpe_runtime.Runtime.DSP
            
            # 创建网络
            network = snpe.snpe_network.SNPENetwork(container, runtime)
            
            return network
            
        except Exception as e:
            print(f"DLC模型加载错误: {e}")
            return None
    
    def optimize_for_hexagon(self, model_path, input_shape):
        """为Hexagon DSP优化模型"""
        # 这里需要使用SNPE工具链进行模型转换和优化
        optimization_config = {
            'quantization': 'int8',
            'target_runtime': 'dsp',
            'input_shape': input_shape,
            'optimization_level': 'high'
        }
        
        return optimization_config
```

### 3. TPU加速

TPU（张量处理单元）是Google专门为机器学习设计的ASIC芯片。

#### Google Coral Edge TPU

```python
class GoogleCoralTPU:
    """Google Coral Edge TPU加速器"""
    
    def __init__(self):
        self.setup_coral()
    
    def setup_coral(self):
        """设置Coral TPU环境"""
        try:
            from pycoral.utils import edgetpu
            from pycoral.adapters import common
            from pycoral.adapters import classify
            from pycoral.adapters import detect
            
            self.edgetpu = edgetpu
            self.common = common
            self.classify = classify
            self.detect = detect
            
            # 检测可用的Edge TPU设备
            devices = edgetpu.list_edge_tpus()
            print(f"检测到 {len(devices)} 个Edge TPU设备")
            
            for i, device in enumerate(devices):
                print(f"设备 {i}: {device}")
            
            self.coral_available = len(devices) > 0
            
        except ImportError:
            print("PyCoral库未安装")
            self.coral_available = False
    
    def load_tflite_model(self, model_path):
        """加载Edge TPU优化的TensorFlow Lite模型"""
        if not self.coral_available:
            return None
        
        try:
            # 创建Edge TPU解释器
            interpreter = self.edgetpu.make_interpreter(model_path)
            interpreter.allocate_tensors()
            
            return interpreter
            
        except Exception as e:
            print(f"Edge TPU模型加载错误: {e}")
            return None
    
    def classify_image(self, interpreter, image, top_k=5):
        """图像分类"""
        if not self.coral_available:
            return None
        
        try:
            # 预处理图像
            size = self.common.input_size(interpreter)
            image_resized = image.resize(size, Image.ANTIALIAS)
            
            # 设置输入
            self.common.set_input(interpreter, image_resized)
            
            # 执行推理
            interpreter.invoke()
            
            # 获取分类结果
            classes = self.classify.get_classes(interpreter, top_k=top_k)
            
            return classes
            
        except Exception as e:
            print(f"图像分类错误: {e}")
            return None
    
    def detect_objects(self, interpreter, image, threshold=0.5):
        """目标检测"""
        if not self.coral_available:
            return None
        
        try:
            # 预处理图像
            size = self.common.input_size(interpreter)
            image_resized = image.resize(size, Image.ANTIALIAS)
            
            # 设置输入
            self.common.set_input(interpreter, image_resized)
            
            # 执行推理
            interpreter.invoke()
            
            # 获取检测结果
            objects = self.detect.get_objects(interpreter, threshold=threshold)
            
            return objects
            
        except Exception as e:
            print(f"目标检测错误: {e}")
            return None
    
    def benchmark_tpu_performance(self, interpreter, test_data, num_runs=100):
        """TPU性能基准测试"""
        if not self.coral_available:
            return None
        
        try:
            # 预热
            for _ in range(10):
                self.common.set_input(interpreter, test_data)
                interpreter.invoke()
            
            # 性能测试
            start_time = time.time()
            
            for _ in range(num_runs):
                self.common.set_input(interpreter, test_data)
                interpreter.invoke()
            
            end_time = time.time()
            
            total_time = end_time - start_time
            avg_latency = total_time / num_runs * 1000  # ms
            throughput = num_runs / total_time  # inferences/sec
            
            return {
                'avg_latency_ms': avg_latency,
                'throughput_inferences_per_sec': throughput,
                'total_time_sec': total_time
            }
            
        except Exception as e:
            print(f"性能测试错误: {e}")
            return None
    
    def convert_model_for_tpu(self, tflite_model_path, output_path):
        """转换模型为Edge TPU格式"""
        try:
            import subprocess
            
            # 使用Edge TPU编译器
            cmd = [
                'edgetpu_compiler',
                tflite_model_path,
                '-o', output_path
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                print("模型转换成功")
                return True
            else:
                print(f"模型转换失败: {result.stderr}")
                return False
                
        except Exception as e:
            print(f"模型转换错误: {e}")
            return False
```

### 4. FPGA加速

FPGA（现场可编程门阵列）提供了灵活的硬件加速方案。

#### Intel FPGA加速

```python
class IntelFPGAAccelerator:
    """Intel FPGA加速器"""
    
    def __init__(self):
        self.setup_opencl_fpga()
    
    def setup_opencl_fpga(self):
        """设置OpenCL FPGA环境"""
        try:
            import pyopencl as cl
            
            # 查找FPGA平台
            platforms = cl.get_platforms()
            fpga_platform = None
            
            for platform in platforms:
                if 'FPGA' in platform.name or 'Altera' in platform.name:
                    fpga_platform = platform
                    break
            
            if fpga_platform:
                # 获取FPGA设备
                fpga_devices = fpga_platform.get_devices(cl.device_type.ACCELERATOR)
                
                if fpga_devices:
                    self.context = cl.Context(fpga_devices)
                    self.queue = cl.CommandQueue(self.context)
                    self.device = fpga_devices[0]
                    
                    print(f"FPGA设备: {self.device.name}")
                    print(f"全局内存: {self.device.global_mem_size / (1024**3):.2f} GB")
                    
                    self.fpga_available = True
                else:
                    print("未找到FPGA设备")
                    self.fpga_available = False
            else:
                print("未找到FPGA平台")
                self.fpga_available = False
                
        except ImportError:
            print("PyOpenCL未安装")
            self.fpga_available = False
    
    def load_aocx_kernel(self, kernel_path):
        """加载AOCX内核"""
        if not self.fpga_available:
            return None
        
        try:
            import pyopencl as cl
            
            # 读取内核文件
            with open(kernel_path, 'rb') as f:
                kernel_binary = f.read()
            
            # 创建程序
            program = cl.Program(self.context, [self.device], [kernel_binary])
            program.build()
            
            return program
            
        except Exception as e:
            print(f"内核加载错误: {e}")
            return None
    
    def create_convolution_kernel(self):
        """创建卷积内核"""
        kernel_source = """
        __kernel void convolution2d(
            __global const float* input,
            __global const float* weights,
            __global float* output,
            const int input_height,
            const int input_width,
            const int input_channels,
            const int output_channels,
            const int kernel_size,
            const int stride,
            const int padding
        ) {
            int gid_x = get_global_id(0);
            int gid_y = get_global_id(1);
            int gid_z = get_global_id(2);
            
            if (gid_x >= (input_width - kernel_size + 2*padding)/stride + 1 ||
                gid_y >= (input_height - kernel_size + 2*padding)/stride + 1 ||
                gid_z >= output_channels) {
                return;
            }
            
            float sum = 0.0f;
            
            for (int kh = 0; kh < kernel_size; kh++) {
                for (int kw = 0; kw < kernel_size; kw++) {
                    for (int ic = 0; ic < input_channels; ic++) {
                        int input_y = gid_y * stride - padding + kh;
                        int input_x = gid_x * stride - padding + kw;
                        
                        if (input_y >= 0 && input_y < input_height &&
                            input_x >= 0 && input_x < input_width) {
                            
                            int input_idx = ic * input_height * input_width +
                                          input_y * input_width + input_x;
                            int weight_idx = gid_z * input_channels * kernel_size * kernel_size +
                                           ic * kernel_size * kernel_size +
                                           kh * kernel_size + kw;
                            
                            sum += input[input_idx] * weights[weight_idx];
                        }
                    }
                }
            }
            
            int output_height = (input_height - kernel_size + 2*padding)/stride + 1;
            int output_width = (input_width - kernel_size + 2*padding)/stride + 1;
            int output_idx = gid_z * output_height * output_width +
                           gid_y * output_width + gid_x;
            
            output[output_idx] = sum;
        }
        """
        
        return kernel_source
    
    def execute_convolution(self, input_data, weights, kernel_params):
        """执行卷积运算"""
        if not self.fpga_available:
            return None
        
        try:
            import pyopencl as cl
            
            # 创建内核
            kernel_source = self.create_convolution_kernel()
            program = cl.Program(self.context, kernel_source).build()
            kernel = program.convolution2d
            
            # 创建缓冲区
            input_buffer = cl.Buffer(self.context, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR, hostbuf=input_data)
            weights_buffer = cl.Buffer(self.context, cl.mem_flags.READ_ONLY | cl.mem_flags.COPY_HOST_PTR, hostbuf=weights)
            
            # 计算输出大小
            output_height = (kernel_params['input_height'] - kernel_params['kernel_size'] + 2*kernel_params['padding']) // kernel_params['stride'] + 1
            output_width = (kernel_params['input_width'] - kernel_params['kernel_size'] + 2*kernel_params['padding']) // kernel_params['stride'] + 1
            output_size = output_height * output_width * kernel_params['output_channels']
            
            output_buffer = cl.Buffer(self.context, cl.mem_flags.WRITE_ONLY, size=output_size * 4)  # float32
            
            # 设置内核参数
            kernel.set_args(
                input_buffer, weights_buffer, output_buffer,
                np.int32(kernel_params['input_height']),
                np.int32(kernel_params['input_width']),
                np.int32(kernel_params['input_channels']),
                np.int32(kernel_params['output_channels']),
                np.int32(kernel_params['kernel_size']),
                np.int32(kernel_params['stride']),
                np.int32(kernel_params['padding'])
            )
            
            # 执行内核
            global_size = (output_width, output_height, kernel_params['output_channels'])
            cl.enqueue_nd_range_kernel(self.queue, kernel, global_size, None)
            
            # 读取结果
            output_data = np.empty(output_size, dtype=np.float32)
            cl.enqueue_copy(self.queue, output_data, output_buffer)
            
            return output_data.reshape(kernel_params['output_channels'], output_height, output_width)
            
        except Exception as e:
            print(f"卷积执行错误: {e}")
            return None
```

## 性能优化策略

### 1. 内存优化

```python
class MemoryOptimizer:
    """内存优化器"""
    
    def __init__(self, device_type='gpu'):
        self.device_type = device_type
        self.memory_pool = {}
    
    def create_memory_pool(self, pool_size_mb=512):
        """创建内存池"""
        if self.device_type == 'gpu':
            return self.create_gpu_memory_pool(pool_size_mb)
        elif self.device_type == 'cpu':
            return self.create_cpu_memory_pool(pool_size_mb)
    
    def create_gpu_memory_pool(self, pool_size_mb):
        """创建GPU内存池"""
        try:
            import cupy as cp
            
            # 设置内存池大小
            pool_size_bytes = pool_size_mb * 1024 * 1024
            mempool = cp.get_default_memory_pool()
            mempool.set_limit(size=pool_size_bytes)
            
            self.memory_pool['gpu'] = mempool
            
            print(f"GPU内存池创建成功，大小: {pool_size_mb} MB")
            return mempool
            
        except ImportError:
            print("CuPy未安装，无法创建GPU内存池")
            return None
    
    def create_cpu_memory_pool(self, pool_size_mb):
        """创建CPU内存池"""
        # 简单的CPU内存池实现
        pool_size_bytes = pool_size_mb * 1024 * 1024
        
        self.memory_pool['cpu'] = {
            'size': pool_size_bytes,
            'allocated': 0,
            'buffers': {}
        }
        
        print(f"CPU内存池创建成功，大小: {pool_size_mb} MB")
        return self.memory_pool['cpu']
    
    def allocate_buffer(self, size, dtype=np.float32):
        """分配缓冲区"""
        buffer_id = f"buffer_{len(self.memory_pool.get('buffers', {}))}"
        
        if self.device_type == 'gpu':
            return self.allocate_gpu_buffer(buffer_id, size, dtype)
        else:
            return self.allocate_cpu_buffer(buffer_id, size, dtype)
    
    def allocate_gpu_buffer(self, buffer_id, size, dtype):
        """分配GPU缓冲区"""
        try:
            import cupy as cp
            
            buffer = cp.zeros(size, dtype=dtype)
            
            if 'gpu' not in self.memory_pool:
                self.memory_pool['gpu'] = {}
            
            self.memory_pool['gpu'][buffer_id] = buffer
            
            return buffer_id, buffer
            
        except Exception as e:
            print(f"GPU缓冲区分配失败: {e}")
            return None, None
    
    def allocate_cpu_buffer(self, buffer_id, size, dtype):
        """分配CPU缓冲区"""
        try:
            buffer = np.zeros(size, dtype=dtype)
            buffer_size = buffer.nbytes
            
            pool = self.memory_pool['cpu']
            
            if pool['allocated'] + buffer_size > pool['size']:
                print("内存池空间不足")
                return None, None
            
            pool['buffers'][buffer_id] = buffer
            pool['allocated'] += buffer_size
            
            return buffer_id, buffer
            
        except Exception as e:
            print(f"CPU缓冲区分配失败: {e}")
            return None, None
    
    def deallocate_buffer(self, buffer_id):
        """释放缓冲区"""
        if self.device_type == 'gpu':
            if 'gpu' in self.memory_pool and buffer_id in self.memory_pool['gpu']:
                del self.memory_pool['gpu'][buffer_id]
        else:
            pool = self.memory_pool['cpu']
            if buffer_id in pool['buffers']:
                buffer_size = pool['buffers'][buffer_id].nbytes
                del pool['buffers'][buffer_id]
                pool['allocated'] -= buffer_size
    
    def get_memory_usage(self):
        """获取内存使用情况"""
        if self.device_type == 'gpu':
            try:
                import cupy as cp
                mempool = cp.get_default_memory_pool()
                
                return {
                    'used_bytes': mempool.used_bytes(),
                    'total_bytes': mempool.total_bytes(),
                    'usage_percent': mempool.used_bytes() / mempool.total_bytes() * 100
                }
            except:
                return {'error': 'Unable to get GPU memory usage'}
        else:
            pool = self.memory_pool['cpu']
            return {
                'allocated_bytes': pool['allocated'],
                'total_bytes': pool['size'],
                'usage_percent': pool['allocated'] / pool['size'] * 100
            }
```

### 2. 批处理优化

```python
class BatchProcessor:
    """批处理优化器"""
    
    def __init__(self, model, device_type='gpu', max_batch_size=32):
        self.model = model
        self.device_type = device_type
        self.max_batch_size = max_batch_size
        self.batch_queue = []
    
    def add_to_batch(self, input_data, callback=None):
        """添加到批处理队列"""
        self.batch_queue.append({
            'data': input_data,
            'callback': callback,
            'timestamp': time.time()
        })
        
        # 检查是否需要处理批次
        if len(self.batch_queue) >= self.max_batch_size:
            return self.process_batch()
        
        return None
    
    def process_batch(self, force=False):
        """处理批次"""
        if not self.batch_queue:
            return []
        
        if not force and len(self.batch_queue) < self.max_batch_size:
            return []
        
        # 准备批次数据
        batch_data = []
        callbacks = []
        
        current_batch = self.batch_queue[:self.max_batch_size]
        self.batch_queue = self.batch_queue[self.max_batch_size:]
        
        for item in current_batch:
            batch_data.append(item['data'])
            callbacks.append(item['callback'])
        
        # 执行批处理推理
        batch_input = np.stack(batch_data)
        
        start_time = time.time()
        batch_results = self.model.predict(batch_input)
        end_time = time.time()
        
        # 处理结果
        results = []
        for i, (result, callback) in enumerate(zip(batch_results, callbacks)):
            result_item = {
                'result': result,
                'latency_ms': (end_time - start_time) * 1000 / len(batch_results),
                'batch_size': len(batch_results)
            }
            
            if callback:
                callback(result_item)
            
            results.append(result_item)
        
        return results
    
    def flush_batch(self):
        """强制处理剩余批次"""
        return self.process_batch(force=True)
    
    def adaptive_batch_size(self, target_latency_ms=100):
        """自适应批次大小"""
        # 测试不同批次大小的性能
        test_data = np.random.random((self.max_batch_size, *self.model.input_shape[1:]))
        
        best_batch_size = 1
        best_throughput = 0
        
        for batch_size in [1, 2, 4, 8, 16, 32]:
            if batch_size > self.max_batch_size:
                break
            
            # 测试当前批次大小
            batch_input = test_data[:batch_size]
            
            # 预热
            for _ in range(5):
                _ = self.model.predict(batch_input)
            
            # 性能测试
            start_time = time.time()
            num_runs = 20
            
            for _ in range(num_runs):
                _ = self.model.predict(batch_input)
            
            end_time = time.time()
            
            total_time = end_time - start_time
            avg_latency = total_time / num_runs * 1000  # ms
            throughput = batch_size * num_runs / total_time  # samples/sec
            
            print(f"批次大小 {batch_size}: 延迟 {avg_latency:.2f}ms, 吞吐量 {throughput:.2f} samples/sec")
            
            # 检查是否满足延迟要求
            if avg_latency <= target_latency_ms and throughput > best_throughput:
                best_batch_size = batch_size
                best_throughput = throughput
        
        self.max_batch_size = best_batch_size
        print(f"选择最优批次大小: {best_batch_size}")
        
        return best_batch_size
```

### 3. 流水线优化

```python
class PipelineOptimizer:
    """流水线优化器"""
    
    def __init__(self, stages, device_assignments=None):
        self.stages = stages
        self.device_assignments = device_assignments or ['cpu'] * len(stages)
        self.setup_pipeline()
    
    def setup_pipeline(self):
        """设置流水线"""
        import threading
        import queue
        
        self.queues = [queue.Queue(maxsize=10) for _ in range(len(self.stages) + 1)]
        self.workers = []
        
        for i, (stage, device) in enumerate(zip(self.stages, self.device_assignments)):
            worker = threading.Thread(
                target=self.stage_worker,
                args=(i, stage, device, self.queues[i], self.queues[i + 1])
            )
            worker.daemon = True
            self.workers.append(worker)
    
    def stage_worker(self, stage_id, stage_func, device, input_queue, output_queue):
        """流水线阶段工作线程"""
        # 设置设备上下文
        if device == 'gpu':
            self.setup_gpu_context()
        
        while True:
            try:
                # 获取输入数据
                data = input_queue.get(timeout=1.0)
                if data is None:  # 结束信号
                    output_queue.put(None)
                    break
                
                # 处理数据
                start_time = time.time()
                result = stage_func(data)
                end_time = time.time()
                
                # 添加性能信息
                result_with_stats = {
                    'data': result,
                    'stage_id': stage_id,
                    'processing_time_ms': (end_time - start_time) * 1000,
                    'device': device
                }
                
                # 输出结果
                output_queue.put(result_with_stats)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"阶段 {stage_id} 处理错误: {e}")
    
    def setup_gpu_context(self):
        """设置GPU上下文"""
        try:
            import tensorflow as tf
            
            # 设置GPU内存增长
            gpus = tf.config.list_physical_devices('GPU')
            if gpus:
                tf.config.experimental.set_memory_growth(gpus[0], True)
        except:
            pass
    
    def start_pipeline(self):
        """启动流水线"""
        for worker in self.workers:
            worker.start()
    
    def process_data(self, input_data):
        """处理数据"""
        # 将数据放入第一个队列
        self.queues[0].put(input_data)
        
        # 从最后一个队列获取结果
        result = self.queues[-1].get()
        
        return result
    
    def stop_pipeline(self):
        """停止流水线"""
        # 发送结束信号
        self.queues[0].put(None)
        
        # 等待所有工作线程结束
        for worker in self.workers:
            worker.join()
    
    def get_pipeline_stats(self):
        """获取流水线统计信息"""
        stats = {
            'stages': len(self.stages),
            'queue_sizes': [q.qsize() for q in self.queues],
            'device_assignments': self.device_assignments
        }
        
        return stats

# 使用示例
def preprocess_stage(data):
    """预处理阶段"""
    # 模拟预处理
    time.sleep(0.01)
    return data * 2

def inference_stage(data):
    """推理阶段"""
    # 模拟推理
    time.sleep(0.05)
    return data + 1

def postprocess_stage(data):
    """后处理阶段"""
    # 模拟后处理
    time.sleep(0.01)
    return data / 2

# 创建流水线
stages = [preprocess_stage, inference_stage, postprocess_stage]
device_assignments = ['cpu', 'gpu', 'cpu']

pipeline = PipelineOptimizer(stages, device_assignments)
pipeline.start_pipeline()

# 处理数据
test_data = np.random.random((224, 224, 3))
result = pipeline.process_data(test_data)

print(f"流水线处理结果: {result}")

# 获取统计信息
stats = pipeline.get_pipeline_stats()
print(f"流水线统计: {stats}")

# 停止流水线
pipeline.stop_pipeline()
```

## 总结

硬件加速是边缘AI性能优化的关键技术，主要要点包括：

1. **硬件选择**: 根据应用需求选择合适的加速硬件（GPU、NPU、TPU、FPGA等）
2. **模型优化**: 针对目标硬件优化模型结构和参数
3. **内存管理**: 高效的内存分配和管理策略
4. **批处理**: 通过批处理提高硬件利用率
5. **流水线**: 多阶段流水线并行处理

通过合理的硬件加速策略，可以显著提升边缘AI应用的性能和效率。
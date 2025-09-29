# 语音识别示例

## 概述

本示例展示了如何在边缘设备上实现高效的语音识别系统，包括音频预处理、特征提取、语音识别和后处理的完整流程。支持实时识别、离线处理和多种语言模型。

## 功能特性

- **实时语音识别**: 支持麦克风实时语音输入识别
- **离线处理**: 完全本地化处理，保护隐私
- **多语言支持**: 支持中文、英文等多种语言
- **噪声抑制**: 内置噪声抑制和回声消除
- **语音活动检测**: 自动检测语音起止点
- **关键词检测**: 支持特定关键词唤醒和检测

## 项目结构

```
speech-recognition/
├── README.md                    # 本文件
├── requirements.txt             # Python依赖
├── models/                      # 预训练模型
│   ├── acoustic_model/         # 声学模型
│   │   ├── wav2vec2.tflite    # Wav2Vec2模型
│   │   └── deepspeech.onnx    # DeepSpeech模型
│   ├── language_model/         # 语言模型
│   │   ├── chinese_lm.arpa    # 中文语言模型
│   │   └── english_lm.arpa    # 英文语言模型
│   └── keyword_spotting/       # 关键词检测
│       └── keyword_model.tflite
├── src/                         # 源代码
│   ├── __init__.py
│   ├── audio_processor.py      # 音频处理器
│   ├── feature_extractor.py    # 特征提取器
│   ├── speech_recognizer.py    # 语音识别器
│   ├── language_model.py       # 语言模型
│   ├── keyword_spotter.py      # 关键词检测器
│   └── utils.py                # 工具函数
├── examples/                    # 使用示例
│   ├── basic_recognition.py    # 基础识别示例
│   ├── realtime_recognition.py # 实时识别
│   ├── file_recognition.py     # 文件识别
│   ├── keyword_detection.py    # 关键词检测
│   └── voice_assistant.py      # 语音助手
├── data/                        # 数据目录
│   ├── audio_samples/          # 音频样本
│   ├── vocabulary/             # 词汇表
│   └── test_recordings/        # 测试录音
├── configs/                     # 配置文件
│   ├── recognition_config.yaml # 识别配置
│   ├── audio_config.yaml       # 音频配置
│   └── model_config.yaml       # 模型配置
└── tests/                       # 测试用例
    ├── test_audio_processor.py # 音频处理测试
    ├── test_recognizer.py      # 识别器测试
    └── test_performance.py     # 性能测试
```

## 快速开始

### 1. 环境准备

```bash
# 安装依赖
pip install -r requirements.txt

# 安装音频处理库
sudo apt-get install portaudio19-dev  # Linux
# brew install portaudio  # macOS

# 下载预训练模型
python scripts/download_models.py
```

### 2. 基础使用

```python
from src.speech_recognizer import EdgeSpeechRecognizer

# 创建语音识别器
recognizer = EdgeSpeechRecognizer(
    model_path='models/acoustic_model/wav2vec2.tflite',
    language='zh-cn',
    device='cpu'
)

# 初始化识别器
recognizer.initialize()

# 识别音频文件
import librosa
audio, sr = librosa.load('data/audio_samples/test.wav', sr=16000)
result = recognizer.recognize(audio)

print(f"识别结果: {result['text']}")
print(f"置信度: {result['confidence']:.3f}")
```

### 3. 实时语音识别

```bash
python examples/realtime_recognition.py --model models/acoustic_model/wav2vec2.tflite --language zh-cn
```

### 4. 关键词检测

```bash
python examples/keyword_detection.py --keywords "你好" "小助手" --model models/keyword_spotting/keyword_model.tflite
```

## 核心组件

### 1. 音频处理器

```python
import numpy as np
import librosa
import scipy.signal
from typing import Tuple, Optional
import pyaudio
import threading
import queue

class AudioProcessor:
    """音频处理器"""
    
    def __init__(self, sample_rate: int = 16000, chunk_size: int = 1024):
        self.sample_rate = sample_rate
        self.chunk_size = chunk_size
        
        # 音频参数
        self.channels = 1
        self.format = pyaudio.paFloat32
        
        # 预处理参数
        self.preemphasis_coeff = 0.97
        self.frame_length = 400  # 25ms at 16kHz
        self.frame_step = 160    # 10ms at 16kHz
        
        # 噪声抑制参数
        self.noise_reduction_enabled = True
        self.noise_profile = None
        
        # 语音活动检测参数
        self.vad_threshold = 0.01
        self.min_speech_duration = 0.5  # 最小语音持续时间
        self.max_silence_duration = 1.0  # 最大静音持续时间
    
    def preprocess_audio(self, audio: np.ndarray) -> np.ndarray:
        """预处理音频信号"""
        # 归一化
        audio = audio / np.max(np.abs(audio))
        
        # 预加重
        if self.preemphasis_coeff > 0:
            audio = np.append(audio[0], audio[1:] - self.preemphasis_coeff * audio[:-1])
        
        # 噪声抑制
        if self.noise_reduction_enabled and self.noise_profile is not None:
            audio = self.reduce_noise(audio)
        
        return audio
    
    def reduce_noise(self, audio: np.ndarray) -> np.ndarray:
        """噪声抑制"""
        # 简单的谱减法噪声抑制
        # 计算短时傅里叶变换
        f, t, stft = scipy.signal.stft(audio, fs=self.sample_rate, 
                                      nperseg=self.frame_length, 
                                      noverlap=self.frame_length - self.frame_step)
        
        # 计算功率谱
        magnitude = np.abs(stft)
        phase = np.angle(stft)
        
        # 估计噪声功率谱
        if self.noise_profile is None:
            # 使用前几帧作为噪声估计
            noise_frames = min(10, magnitude.shape[1])
            self.noise_profile = np.mean(magnitude[:, :noise_frames], axis=1, keepdims=True)
        
        # 谱减法
        alpha = 2.0  # 过减因子
        beta = 0.01  # 最小增益
        
        noise_power = self.noise_profile ** 2
        signal_power = magnitude ** 2
        
        # 计算增益
        gain = 1 - alpha * (noise_power / signal_power)
        gain = np.maximum(gain, beta)
        
        # 应用增益
        enhanced_magnitude = magnitude * gain
        
        # 重构信号
        enhanced_stft = enhanced_magnitude * np.exp(1j * phase)
        _, enhanced_audio = scipy.signal.istft(enhanced_stft, fs=self.sample_rate,
                                              nperseg=self.frame_length,
                                              noverlap=self.frame_length - self.frame_step)
        
        return enhanced_audio
    
    def detect_voice_activity(self, audio: np.ndarray) -> Tuple[bool, float]:
        """语音活动检测"""
        # 计算短时能量
        frame_length = int(0.025 * self.sample_rate)  # 25ms
        frame_step = int(0.010 * self.sample_rate)    # 10ms
        
        energy = []
        for i in range(0, len(audio) - frame_length, frame_step):
            frame = audio[i:i + frame_length]
            frame_energy = np.sum(frame ** 2) / len(frame)
            energy.append(frame_energy)
        
        # 计算平均能量
        avg_energy = np.mean(energy)
        
        # 判断是否为语音
        is_speech = avg_energy > self.vad_threshold
        
        return is_speech, avg_energy
    
    def extract_mfcc_features(self, audio: np.ndarray, n_mfcc: int = 13) -> np.ndarray:
        """提取MFCC特征"""
        # 计算MFCC
        mfcc = librosa.feature.mfcc(
            y=audio,
            sr=self.sample_rate,
            n_mfcc=n_mfcc,
            n_fft=512,
            hop_length=self.frame_step,
            win_length=self.frame_length
        )
        
        # 计算一阶和二阶差分
        delta_mfcc = librosa.feature.delta(mfcc)
        delta2_mfcc = librosa.feature.delta(mfcc, order=2)
        
        # 合并特征
        features = np.vstack([mfcc, delta_mfcc, delta2_mfcc])
        
        # 转置以匹配模型输入格式 (time_steps, features)
        features = features.T
        
        return features
    
    def extract_mel_spectrogram(self, audio: np.ndarray, n_mels: int = 80) -> np.ndarray:
        """提取Mel频谱图"""
        mel_spec = librosa.feature.melspectrogram(
            y=audio,
            sr=self.sample_rate,
            n_mels=n_mels,
            n_fft=512,
            hop_length=self.frame_step,
            win_length=self.frame_length
        )
        
        # 转换为对数刻度
        log_mel_spec = librosa.power_to_db(mel_spec, ref=np.max)
        
        # 转置
        log_mel_spec = log_mel_spec.T
        
        return log_mel_spec

class RealTimeAudioCapture:
    """实时音频捕获"""
    
    def __init__(self, processor: AudioProcessor, callback=None):
        self.processor = processor
        self.callback = callback
        
        # PyAudio设置
        self.pyaudio = pyaudio.PyAudio()
        self.stream = None
        
        # 缓冲区
        self.audio_buffer = queue.Queue()
        self.is_recording = False
        
        # 语音检测状态
        self.speech_detected = False
        self.speech_start_time = None
        self.last_speech_time = None
    
    def start_recording(self):
        """开始录音"""
        try:
            self.stream = self.pyaudio.open(
                format=self.processor.format,
                channels=self.processor.channels,
                rate=self.processor.sample_rate,
                input=True,
                frames_per_buffer=self.processor.chunk_size,
                stream_callback=self._audio_callback
            )
            
            self.is_recording = True
            self.stream.start_stream()
            
            print("开始录音...")
            
        except Exception as e:
            print(f"录音启动失败: {e}")
    
    def stop_recording(self):
        """停止录音"""
        self.is_recording = False
        
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        
        self.pyaudio.terminate()
        print("录音已停止")
    
    def _audio_callback(self, in_data, frame_count, time_info, status):
        """音频回调函数"""
        if status:
            print(f"音频状态: {status}")
        
        # 转换音频数据
        audio_data = np.frombuffer(in_data, dtype=np.float32)
        
        # 语音活动检测
        is_speech, energy = self.processor.detect_voice_activity(audio_data)
        
        current_time = time.time()
        
        if is_speech:
            if not self.speech_detected:
                # 语音开始
                self.speech_detected = True
                self.speech_start_time = current_time
                print("检测到语音开始")
            
            self.last_speech_time = current_time
            
            # 添加到缓冲区
            self.audio_buffer.put(audio_data)
            
        else:
            if self.speech_detected:
                # 检查是否语音结束
                silence_duration = current_time - self.last_speech_time
                
                if silence_duration > self.processor.max_silence_duration:
                    # 语音结束
                    self.speech_detected = False
                    print("检测到语音结束")
                    
                    # 处理累积的音频
                    self._process_accumulated_audio()
        
        return (in_data, pyaudio.paContinue)
    
    def _process_accumulated_audio(self):
        """处理累积的音频数据"""
        if self.audio_buffer.empty():
            return
        
        # 合并音频数据
        audio_chunks = []
        while not self.audio_buffer.empty():
            audio_chunks.append(self.audio_buffer.get())
        
        if len(audio_chunks) == 0:
            return
        
        # 合并音频
        full_audio = np.concatenate(audio_chunks)
        
        # 检查最小语音长度
        duration = len(full_audio) / self.processor.sample_rate
        if duration < self.processor.min_speech_duration:
            return
        
        # 预处理音频
        processed_audio = self.processor.preprocess_audio(full_audio)
        
        # 调用回调函数
        if self.callback:
            self.callback(processed_audio)
```

### 2. 语音识别器

```python
import numpy as np
import tensorflow as tf
from typing import Dict, List, Any, Optional
import time

class EdgeSpeechRecognizer:
    """边缘语音识别器"""
    
    def __init__(self, model_path: str, language: str = 'zh-cn', device: str = 'cpu'):
        self.model_path = model_path
        self.language = language
        self.device = device
        
        # 模型组件
        self.acoustic_model = None
        self.language_model = None
        self.vocabulary = None
        
        # 识别参数
        self.beam_width = 5
        self.max_sequence_length = 1000
        self.blank_token_id = 0
        
        # 性能统计
        self.stats = {
            'total_recognitions': 0,
            'successful_recognitions': 0,
            'average_processing_time': 0,
            'average_confidence': 0
        }
    
    def initialize(self) -> bool:
        """初始化识别器"""
        try:
            # 加载声学模型
            if not self.load_acoustic_model():
                return False
            
            # 加载语言模型
            if not self.load_language_model():
                print("警告: 语言模型加载失败，将使用贪婪解码")
            
            # 加载词汇表
            if not self.load_vocabulary():
                print("警告: 词汇表加载失败")
            
            print("语音识别器初始化成功")
            return True
            
        except Exception as e:
            print(f"识别器初始化失败: {e}")
            return False
    
    def load_acoustic_model(self) -> bool:
        """加载声学模型"""
        try:
            if self.model_path.endswith('.tflite'):
                # TensorFlow Lite模型
                self.acoustic_model = tf.lite.Interpreter(model_path=self.model_path)
                self.acoustic_model.allocate_tensors()
                
                # 获取输入输出信息
                self.input_details = self.acoustic_model.get_input_details()
                self.output_details = self.acoustic_model.get_output_details()
                
            elif self.model_path.endswith('.onnx'):
                # ONNX模型
                import onnxruntime as ort
                
                providers = ['CPUExecutionProvider']
                if self.device == 'gpu':
                    providers.insert(0, 'CUDAExecutionProvider')
                
                self.acoustic_model = ort.InferenceSession(self.model_path, providers=providers)
                
            else:
                # 标准TensorFlow模型
                self.acoustic_model = tf.keras.models.load_model(self.model_path)
            
            return True
            
        except Exception as e:
            print(f"声学模型加载失败: {e}")
            return False
    
    def load_language_model(self) -> bool:
        """加载语言模型"""
        try:
            # 这里可以加载n-gram语言模型或神经语言模型
            # 简化实现，暂时跳过
            return True
            
        except Exception as e:
            print(f"语言模型加载失败: {e}")
            return False
    
    def load_vocabulary(self) -> bool:
        """加载词汇表"""
        try:
            vocab_file = f'data/vocabulary/{self.language}_vocab.txt'
            
            self.vocabulary = {}
            self.id_to_token = {}
            
            with open(vocab_file, 'r', encoding='utf-8') as f:
                for idx, line in enumerate(f):
                    token = line.strip()
                    self.vocabulary[token] = idx
                    self.id_to_token[idx] = token
            
            return True
            
        except Exception as e:
            print(f"词汇表加载失败: {e}")
            # 使用默认词汇表
            self.create_default_vocabulary()
            return True
    
    def create_default_vocabulary(self):
        """创建默认词汇表"""
        # 简化的中文字符词汇表
        chars = list("abcdefghijklmnopqrstuvwxyz0123456789 ")
        chars.extend(list("的一是在不了有和人这中大为上个国我以要他时来用们生到作地于出就分对成会可主发年动同工也能下过子说产种面而方后多定行学法所民得经十三之进着等部度家电力里如水化高自二理起小物现实加量都两体制机当使点从业本去把性好应开它合还因由其些然前外天政四日那社义事平形相全表间样与关各重新线内数正心反你明看原又么利比或但质气第向道命此变条只没结解问意建月公无系军很情者最立代想已通并提直题党程展五果料象员革位入常文总次品式活设及管特件长求老头基资边流路级少图山统接知较将组见计别她手角期根论运农指几九区强放决西被干做必战先回则任取据处队南给色光门即保治北造百规热领七海口东导器压志世金增争济阶油思术极交受联什认六共权收证改清己美再采转更单风切打白教速花带安场身车例真务具万每目至达走积示议声报斗完类八离华名确才科张信马节话米整空元况今集温传土许步群广石记需段研界拉林律叫且究观越织装影算低持音众书布复容儿须际商非验连断深难近矿千周委素技备半办青省列习响约支般史感劳便团往酸历市克何除消构府称太准精值号率族维划选标写存候毛亲快效斯院查江型眼王按格养易置派层片始却专状育厂京识适属圆包火住调满县局照参红细引听该铁价严")
        
        self.vocabulary = {'<blank>': 0}
        self.id_to_token = {0: '<blank>'}
        
        for idx, char in enumerate(chars, 1):
            self.vocabulary[char] = idx
            self.id_to_token[idx] = char
    
    def recognize(self, audio: np.ndarray) -> Dict[str, Any]:
        """识别语音"""
        start_time = time.time()
        
        try:
            # 提取特征
            features = self.extract_features(audio)
            
            # 声学模型推理
            logits = self.acoustic_inference(features)
            
            # 解码
            text, confidence = self.decode_logits(logits)
            
            # 计算处理时间
            processing_time = (time.time() - start_time) * 1000
            
            # 更新统计
            self.update_stats(processing_time, confidence, len(text) > 0)
            
            result = {
                'text': text,
                'confidence': confidence,
                'processing_time_ms': processing_time,
                'audio_duration_s': len(audio) / 16000,
                'features_shape': features.shape
            }
            
            return result
            
        except Exception as e:
            print(f"语音识别失败: {e}")
            return {
                'text': '',
                'confidence': 0.0,
                'processing_time_ms': (time.time() - start_time) * 1000,
                'error': str(e)
            }
    
    def extract_features(self, audio: np.ndarray) -> np.ndarray:
        """提取音频特征"""
        # 使用AudioProcessor提取特征
        processor = AudioProcessor()
        
        # 根据模型类型选择特征
        if 'wav2vec' in self.model_path.lower():
            # Wav2Vec2使用原始音频
            # 调整长度到固定大小
            target_length = 16000 * 10  # 10秒
            if len(audio) > target_length:
                audio = audio[:target_length]
            else:
                audio = np.pad(audio, (0, target_length - len(audio)), 'constant')
            
            features = audio.reshape(1, -1)  # (batch, time)
            
        else:
            # 其他模型使用MFCC或Mel频谱图
            features = processor.extract_mfcc_features(audio)
            features = np.expand_dims(features, axis=0)  # 添加批次维度
        
        return features
    
    def acoustic_inference(self, features: np.ndarray) -> np.ndarray:
        """声学模型推理"""
        if isinstance(self.acoustic_model, tf.lite.Interpreter):
            # TensorFlow Lite推理
            self.acoustic_model.set_tensor(self.input_details[0]['index'], features.astype(np.float32))
            self.acoustic_model.invoke()
            logits = self.acoustic_model.get_tensor(self.output_details[0]['index'])
            
        elif hasattr(self.acoustic_model, 'run'):
            # ONNX推理
            input_name = self.acoustic_model.get_inputs()[0].name
            logits = self.acoustic_model.run(None, {input_name: features.astype(np.float32)})[0]
            
        else:
            # TensorFlow推理
            logits = self.acoustic_model.predict(features)
        
        return logits
    
    def decode_logits(self, logits: np.ndarray) -> tuple[str, float]:
        """解码logits为文本"""
        # 移除批次维度
        if len(logits.shape) == 3:
            logits = logits[0]
        
        # 贪婪解码
        predicted_ids = np.argmax(logits, axis=-1)
        
        # CTC解码 - 移除重复和空白标记
        decoded_ids = []
        prev_id = -1
        
        for pred_id in predicted_ids:
            if pred_id != prev_id and pred_id != self.blank_token_id:
                decoded_ids.append(pred_id)
            prev_id = pred_id
        
        # 转换为文本
        text = ""
        confidences = []
        
        for token_id in decoded_ids:
            if token_id in self.id_to_token:
                text += self.id_to_token[token_id]
                # 计算该token的置信度
                token_confidence = np.max(tf.nn.softmax(logits[predicted_ids == token_id]))
                confidences.append(token_confidence)
        
        # 计算平均置信度
        avg_confidence = np.mean(confidences) if confidences else 0.0
        
        return text, float(avg_confidence)
    
    def beam_search_decode(self, logits: np.ndarray) -> tuple[str, float]:
        """束搜索解码"""
        # 简化的束搜索实现
        vocab_size = logits.shape[-1]
        seq_length = logits.shape[0]
        
        # 初始化束
        beams = [{'sequence': [], 'score': 0.0}]
        
        for t in range(seq_length):
            new_beams = []
            
            for beam in beams:
                # 获取当前时间步的概率
                probs = tf.nn.softmax(logits[t]).numpy()
                
                # 选择top-k候选
                top_k_indices = np.argsort(probs)[-self.beam_width:]
                
                for idx in top_k_indices:
                    new_sequence = beam['sequence'] + [idx]
                    new_score = beam['score'] + np.log(probs[idx])
                    
                    new_beams.append({
                        'sequence': new_sequence,
                        'score': new_score
                    })
            
            # 保留最佳的beam_width个候选
            new_beams.sort(key=lambda x: x['score'], reverse=True)
            beams = new_beams[:self.beam_width]
        
        # 选择最佳序列
        best_beam = beams[0]
        
        # CTC后处理
        decoded_sequence = self.ctc_postprocess(best_beam['sequence'])
        
        # 转换为文本
        text = ''.join([self.id_to_token.get(idx, '') for idx in decoded_sequence])
        confidence = np.exp(best_beam['score'] / len(decoded_sequence))
        
        return text, confidence
    
    def ctc_postprocess(self, sequence: List[int]) -> List[int]:
        """CTC后处理"""
        decoded = []
        prev_token = -1
        
        for token in sequence:
            if token != prev_token and token != self.blank_token_id:
                decoded.append(token)
            prev_token = token
        
        return decoded
    
    def update_stats(self, processing_time: float, confidence: float, success: bool):
        """更新统计信息"""
        self.stats['total_recognitions'] += 1
        
        if success:
            self.stats['successful_recognitions'] += 1
        
        # 更新平均处理时间
        total = self.stats['total_recognitions']
        self.stats['average_processing_time'] = (
            (self.stats['average_processing_time'] * (total - 1) + processing_time) / total
        )
        
        # 更新平均置信度
        if success:
            successful = self.stats['successful_recognitions']
            self.stats['average_confidence'] = (
                (self.stats['average_confidence'] * (successful - 1) + confidence) / successful
            )
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        success_rate = 0.0
        if self.stats['total_recognitions'] > 0:
            success_rate = self.stats['successful_recognitions'] / self.stats['total_recognitions']
        
        return {
            'total_recognitions': self.stats['total_recognitions'],
            'successful_recognitions': self.stats['successful_recognitions'],
            'success_rate': success_rate,
            'average_processing_time_ms': self.stats['average_processing_time'],
            'average_confidence': self.stats['average_confidence'],
            'vocabulary_size': len(self.vocabulary),
            'language': self.language,
            'model_path': self.model_path
        }
```

### 3. 关键词检测器

```python
import numpy as np
import tensorflow as tf
from typing import List, Dict, Any, Optional
import time
from collections import deque

class KeywordSpotter:
    """关键词检测器"""
    
    def __init__(self, model_path: str, keywords: List[str], device: str = 'cpu'):
        self.model_path = model_path
        self.keywords = keywords
        self.device = device
        
        # 模型
        self.model = None
        self.is_loaded = False
        
        # 检测参数
        self.detection_threshold = 0.7
        self.window_size = 1.0  # 1秒窗口
        self.hop_size = 0.1     # 100ms跳跃
        
        # 音频缓冲区
        self.audio_buffer = deque(maxlen=int(16000 * self.window_size))
        
        # 检测历史
        self.detection_history = deque(maxlen=10)
        
        # 统计信息
        self.stats = {
            'total_detections': 0,
            'keyword_counts': {kw: 0 for kw in keywords},
            'false_positives': 0,
            'average_confidence': 0
        }
    
    def load_model(self) -> bool:
        """加载关键词检测模型"""
        try:
            if self.model_path.endswith('.tflite'):
                self.model = tf.lite.Interpreter(model_path=self.model_path)
                self.model.allocate_tensors()
                
                self.input_details = self.model.get_input_details()
                self.output_details = self.model.get_output_details()
                
            else:
                self.model = tf.keras.models.load_model(self.model_path)
            
            self.is_loaded = True
            print(f"关键词检测模型加载成功: {len(self.keywords)} 个关键词")
            return True
            
        except Exception as e:
            print(f"关键词检测模型加载失败: {e}")
            return False
    
    def add_audio(self, audio_chunk: np.ndarray) -> Optional[Dict[str, Any]]:
        """添加音频块并检测关键词"""
        if not self.is_loaded:
            return None
        
        # 添加到缓冲区
        self.audio_buffer.extend(audio_chunk)
        
        # 检查缓冲区是否足够
        if len(self.audio_buffer) < int(16000 * self.window_size):
            return None
        
        # 提取当前窗口的音频
        window_audio = np.array(list(self.audio_buffer))
        
        # 执行关键词检测
        detection_result = self.detect_keywords(window_audio)
        
        return detection_result
    
    def detect_keywords(self, audio: np.ndarray) -> Dict[str, Any]:
        """检测关键词"""
        try:
            # 提取特征
            features = self.extract_features(audio)
            
            # 模型推理
            start_time = time.time()
            predictions = self.predict(features)
            inference_time = (time.time() - start_time) * 1000
            
            # 解析结果
            detected_keywords = []
            max_confidence = 0.0
            
            for i, keyword in enumerate(self.keywords):
                confidence = predictions[i] if i < len(predictions) else 0.0
                
                if confidence > self.detection_threshold:
                    detected_keywords.append({
                        'keyword': keyword,
                        'confidence': float(confidence),
                        'timestamp': time.time()
                    })
                    
                    # 更新统计
                    self.stats['keyword_counts'][keyword] += 1
                    max_confidence = max(max_confidence, confidence)
            
            # 更新总体统计
            if detected_keywords:
                self.stats['total_detections'] += 1
                
                # 更新平均置信度
                total = self.stats['total_detections']
                self.stats['average_confidence'] = (
                    (self.stats['average_confidence'] * (total - 1) + max_confidence) / total
                )
            
            # 记录检测历史
            self.detection_history.append({
                'timestamp': time.time(),
                'keywords': detected_keywords,
                'max_confidence': max_confidence,
                'inference_time_ms': inference_time
            })
            
            result = {
                'detected_keywords': detected_keywords,
                'max_confidence': max_confidence,
                'inference_time_ms': inference_time,
                'audio_duration_s': len(audio) / 16000
            }
            
            return result
            
        except Exception as e:
            print(f"关键词检测失败: {e}")
            return {
                'detected_keywords': [],
                'max_confidence': 0.0,
                'inference_time_ms': 0.0,
                'error': str(e)
            }
    
    def extract_features(self, audio: np.ndarray) -> np.ndarray:
        """提取音频特征"""
        # 使用MFCC特征
        processor = AudioProcessor()
        features = processor.extract_mfcc_features(audio, n_mfcc=13)
        
        # 调整到固定长度
        target_length = 100  # 100帧
        if features.shape[0] > target_length:
            features = features[:target_length]
        else:
            # 填充
            padding = target_length - features.shape[0]
            features = np.pad(features, ((0, padding), (0, 0)), 'constant')
        
        # 添加批次维度
        features = np.expand_dims(features, axis=0)
        
        return features
    
    def predict(self, features: np.ndarray) -> np.ndarray:
        """模型预测"""
        if isinstance(self.model, tf.lite.Interpreter):
            # TensorFlow Lite推理
            self.model.set_tensor(self.input_details[0]['index'], features.astype(np.float32))
            self.model.invoke()
            predictions = self.model.get_tensor(self.output_details[0]['index'])
            
        else:
            # TensorFlow推理
            predictions = self.model.predict(features)
        
        # 应用sigmoid激活
        predictions = tf.nn.sigmoid(predictions).numpy()
        
        return predictions[0]  # 移除批次维度
    
    def reset_buffer(self):
        """重置音频缓冲区"""
        self.audio_buffer.clear()
    
    def get_detection_history(self, last_n: int = 10) -> List[Dict[str, Any]]:
        """获取检测历史"""
        return list(self.detection_history)[-last_n:]
    
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        return {
            'total_detections': self.stats['total_detections'],
            'keyword_counts': self.stats['keyword_counts'].copy(),
            'false_positives': self.stats['false_positives'],
            'average_confidence': self.stats['average_confidence'],
            'detection_threshold': self.detection_threshold,
            'keywords': self.keywords.copy()
        }
    
    def update_threshold(self, new_threshold: float):
        """更新检测阈值"""
        self.detection_threshold = max(0.0, min(1.0, new_threshold))
        print(f"检测阈值更新为: {self.detection_threshold}")
    
    def add_keyword(self, keyword: str):
        """添加新关键词"""
        if keyword not in self.keywords:
            self.keywords.append(keyword)
            self.stats['keyword_counts'][keyword] = 0
            print(f"添加新关键词: {keyword}")
    
    def remove_keyword(self, keyword: str):
        """移除关键词"""
        if keyword in self.keywords:
            self.keywords.remove(keyword)
            del self.stats['keyword_counts'][keyword]
            print(f"移除关键词: {keyword}")
```

## 实时语音识别示例

```python
import argparse
import time
import threading
from src.audio_processor import AudioProcessor, RealTimeAudioCapture
from src.speech_recognizer import EdgeSpeechRecognizer
from src.keyword_spotter import KeywordSpotter

class RealTimeSpeechRecognition:
    """实时语音识别系统"""
    
    def __init__(self, recognizer_model: str, language: str = 'zh-cn', 
                 keyword_model: str = None, keywords: List[str] = None):
        
        # 初始化组件
        self.audio_processor = AudioProcessor()
        self.speech_recognizer = EdgeSpeechRecognizer(recognizer_model, language)
        
        # 关键词检测器（可选）
        self.keyword_spotter = None
        if keyword_model and keywords:
            self.keyword_spotter = KeywordSpotter(keyword_model, keywords)
        
        # 音频捕获
        self.audio_capture = RealTimeAudioCapture(
            self.audio_processor, 
            callback=self.process_audio
        )
        
        # 系统状态
        self.is_running = False
        self.recognition_mode = 'continuous'  # 'continuous' 或 'keyword_triggered'
        self.keyword_detected = False
        
        # 结果缓存
        self.recent_results = []
        self.max_results = 10
    
    def initialize(self) -> bool:
        """初始化系统"""
        try:
            # 初始化语音识别器
            if not self.speech_recognizer.initialize():
                return False
            
            # 初始化关键词检测器
            if self.keyword_spotter:
                if not self.keyword_spotter.load_model():
                    print("关键词检测器加载失败，将使用连续识别模式")
                    self.keyword_spotter = None
                else:
                    self.recognition_mode = 'keyword_triggered'
            
            print("实时语音识别系统初始化成功")
            return True
            
        except Exception as e:
            print(f"系统初始化失败: {e}")
            return False
    
    def start(self):
        """启动实时识别"""
        if not self.initialize():
            return
        
        self.is_running = True
        
        print("启动实时语音识别...")
        print(f"识别模式: {self.recognition_mode}")
        print("说话开始识别，按Ctrl+C退出")
        
        # 启动音频捕获
        self.audio_capture.start_recording()
        
        try:
            # 主循环
            while self.is_running:
                time.sleep(0.1)
                
                # 显示最近的识别结果
                self.display_recent_results()
                
        except KeyboardInterrupt:
            print("\n正在停止...")
        
        finally:
            self.stop()
    
    def stop(self):
        """停止识别"""
        self.is_running = False
        self.audio_capture.stop_recording()
        
        # 打印统计信息
        self.print_statistics()
        
        print("实时语音识别已停止")
    
    def process_audio(self, audio: np.ndarray):
        """处理音频数据"""
        if not self.is_running:
            return
        
        try:
            if self.recognition_mode == 'keyword_triggered' and self.keyword_spotter:
                # 关键词触发模式
                self.process_keyword_triggered(audio)
            else:
                # 连续识别模式
                self.process_continuous(audio)
                
        except Exception as e:
            print(f"音频处理错误: {e}")
    
    def process_keyword_triggered(self, audio: np.ndarray):
        """关键词触发模式处理"""
        # 检测关键词
        keyword_result = self.keyword_spotter.add_audio(audio)
        
        if keyword_result and keyword_result['detected_keywords']:
            # 检测到关键词
            detected_kw = keyword_result['detected_keywords'][0]
            print(f"\n🎯 检测到关键词: {detected_kw['keyword']} (置信度: {detected_kw['confidence']:.3f})")
            
            self.keyword_detected = True
            
            # 等待后续语音进行识别
            threading.Timer(0.5, self.trigger_recognition, args=[audio]).start()
    
    def process_continuous(self, audio: np.ndarray):
        """连续识别模式处理"""
        # 直接进行语音识别
        self.perform_recognition(audio)
    
    def trigger_recognition(self, audio: np.ndarray):
        """触发语音识别"""
        if self.keyword_detected:
            self.perform_recognition(audio)
            self.keyword_detected = False
    
    def perform_recognition(self, audio: np.ndarray):
        """执行语音识别"""
        try:
            # 语音识别
            result = self.speech_recognizer.recognize(audio)
            
            if result['text'].strip():
                # 添加时间戳
                result['timestamp'] = time.time()
                result['formatted_time'] = time.strftime('%H:%M:%S')
                
                # 添加到结果缓存
                self.recent_results.append(result)
                if len(self.recent_results) > self.max_results:
                    self.recent_results.pop(0)
                
                # 实时显示结果
                print(f"\n🎤 [{result['formatted_time']}] 识别结果: {result['text']}")
                print(f"   置信度: {result['confidence']:.3f}, 处理时间: {result['processing_time_ms']:.1f}ms")
                
        except Exception as e:
            print(f"语音识别错误: {e}")
    
    def display_recent_results(self):
        """显示最近的识别结果"""
        # 这里可以实现更复杂的显示逻辑
        pass
    
    def print_statistics(self):
        """打印统计信息"""
        print("\n=== 识别统计 ===")
        
        # 语音识别统计
        speech_stats = self.speech_recognizer.get_stats()
        print(f"总识别次数: {speech_stats['total_recognitions']}")
        print(f"成功识别次数: {speech_stats['successful_recognitions']}")
        print(f"识别成功率: {speech_stats['success_rate']:.1%}")
        print(f"平均处理时间: {speech_stats['average_processing_time_ms']:.1f}ms")
        print(f"平均置信度: {speech_stats['average_confidence']:.3f}")
        
        # 关键词检测统计
        if self.keyword_spotter:
            keyword_stats = self.keyword_spotter.get_stats()
            print(f"\n关键词检测次数: {keyword_stats['total_detections']}")
            print("各关键词检测次数:")
            for kw, count in keyword_stats['keyword_counts'].items():
                print(f"  {kw}: {count}")
        
        # 最近识别结果
        if self.recent_results:
            print(f"\n最近 {len(self.recent_results)} 次识别结果:")
            for i, result in enumerate(self.recent_results[-5:], 1):
                print(f"  {i}. [{result['formatted_time']}] {result['text']} (置信度: {result['confidence']:.3f})")

def main():
    parser = argparse.ArgumentParser(description='实时语音识别')
    parser.add_argument('--model', required=True, help='语音识别模型路径')
    parser.add_argument('--language', default='zh-cn', help='识别语言')
    parser.add_argument('--keyword_model', help='关键词检测模型路径')
    parser.add_argument('--keywords', nargs='+', help='关键词列表')
    parser.add_argument('--mode', choices=['continuous', 'keyword_triggered'], 
                       default='continuous', help='识别模式')
    
    args = parser.parse_args()
    
    # 创建实时语音识别系统
    recognition_system = RealTimeSpeechRecognition(
        args.model, args.language, args.keyword_model, args.keywords
    )
    
    # 设置识别模式
    if args.mode == 'keyword_triggered' and not (args.keyword_model and args.keywords):
        print("关键词触发模式需要提供关键词模型和关键词列表")
        return
    
    recognition_system.recognition_mode = args.mode
    
    # 启动识别
    recognition_system.start()

if __name__ == "__main__":
    main()
```

## 应用场景

- **智能音箱**: 语音控制和查询
- **车载系统**: 免提语音操作
- **智能家居**: 语音控制家电设备
- **会议记录**: 实时语音转文字
- **辅助输入**: 语音输入法和听写
- **安防监控**: 语音事件检测

## 性能优化

### 1. 模型优化
- **模型量化**: 使用INT8量化减少模型大小
- **模型剪枝**: 移除不重要的参数
- **知识蒸馏**: 用大模型训练小模型

### 2. 音频处理优化
- **实时处理**: 流式音频处理
- **噪声抑制**: 提高识别准确率
- **回声消除**: 改善音频质量

### 3. 系统优化
- **缓存机制**: 缓存常用词汇和模型
- **并行处理**: 多线程音频处理
- **内存管理**: 优化内存使用

## 常见问题

### Q: 如何提高识别准确率？
A: 
1. 使用高质量的音频输入设备
2. 在安静环境中进行识别
3. 调整识别参数和阈值
4. 使用领域特定的语言模型

### Q: 如何处理方言和口音？
A: 
1. 使用多方言训练的模型
2. 收集特定方言的训练数据
3. 进行模型微调和适应

### Q: 如何降低延迟？
A: 
1. 使用更轻量级的模型
2. 优化音频预处理流程
3. 使用硬件加速
4. 减少网络传输

## 扩展功能

- **多语言识别**: 支持多种语言混合识别
- **说话人识别**: 识别不同说话人
- **情感识别**: 分析语音中的情感
- **语音合成**: 文字转语音功能
- **对话管理**: 多轮对话理解

欢迎贡献代码和提出改进建议！
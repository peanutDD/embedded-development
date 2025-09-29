# 边缘NLP技术

## 概述

自然语言处理（NLP）在边缘设备上的应用正在快速发展，涵盖语音识别、文本分析、机器翻译、对话系统等多个领域。边缘NLP技术能够在保护隐私的同时提供实时的语言理解和生成能力。

## 核心技术

### 1. 语音识别 (Speech Recognition)

语音识别是将语音信号转换为文本的技术，是边缘NLP的重要组成部分。

#### 轻量级语音识别模型

```python
import numpy as np
import librosa
import tensorflow as tf
from scipy.signal import spectrogram

class EdgeSpeechRecognizer:
    """边缘语音识别器"""
    
    def __init__(self, model_path, vocab_path):
        self.model = self.load_model(model_path)
        self.vocab = self.load_vocabulary(vocab_path)
        self.sample_rate = 16000
        self.frame_length = 400  # 25ms at 16kHz
        self.frame_step = 160    # 10ms at 16kHz
        
    def load_model(self, model_path):
        """加载语音识别模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def load_vocabulary(self, vocab_path):
        """加载词汇表"""
        vocab = {}
        with open(vocab_path, 'r', encoding='utf-8') as f:
            for idx, line in enumerate(f):
                char = line.strip()
                vocab[idx] = char
        return vocab
    
    def preprocess_audio(self, audio_data):
        """预处理音频数据"""
        # 确保采样率正确
        if len(audio_data.shape) > 1:
            audio_data = np.mean(audio_data, axis=1)
        
        # 归一化
        audio_data = audio_data / np.max(np.abs(audio_data))
        
        # 提取MFCC特征
        mfcc_features = librosa.feature.mfcc(
            y=audio_data,
            sr=self.sample_rate,
            n_mfcc=13,
            n_fft=512,
            hop_length=self.frame_step,
            win_length=self.frame_length
        )
        
        # 添加一阶和二阶差分
        delta_mfcc = librosa.feature.delta(mfcc_features)
        delta2_mfcc = librosa.feature.delta(mfcc_features, order=2)
        
        # 合并特征
        features = np.vstack([mfcc_features, delta_mfcc, delta2_mfcc])
        
        # 转置以匹配模型输入格式 (time_steps, features)
        features = features.T
        
        return features
    
    def recognize(self, audio_data):
        """执行语音识别"""
        # 预处理
        features = self.preprocess_audio(audio_data)
        
        # 添加批次维度
        input_data = np.expand_dims(features, axis=0).astype(np.float32)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(input_data)
        else:
            predictions = self.model.predict(input_data)
        
        # 解码
        text = self.decode_predictions(predictions)
        
        return text
    
    def predict_tflite(self, input_data):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_data)
        self.model.invoke()
        
        return self.model.get_tensor(output_details[0]['index'])
    
    def decode_predictions(self, predictions):
        """解码预测结果"""
        # CTC解码
        decoded = tf.nn.ctc_greedy_decoder(
            inputs=tf.transpose(predictions, [1, 0, 2]),
            sequence_length=tf.fill([tf.shape(predictions)[0]], tf.shape(predictions)[1])
        )
        
        # 转换为文本
        dense_decoded = tf.sparse.to_dense(decoded[0][0], default_value=-1)
        
        text = ""
        for sequence in dense_decoded.numpy():
            for char_idx in sequence:
                if char_idx >= 0 and char_idx in self.vocab:
                    text += self.vocab[char_idx]
        
        return text.strip()

# 实时语音识别
class RealTimeSpeechRecognizer:
    """实时语音识别器"""
    
    def __init__(self, model_path, vocab_path, chunk_duration=1.0):
        self.recognizer = EdgeSpeechRecognizer(model_path, vocab_path)
        self.chunk_duration = chunk_duration
        self.sample_rate = 16000
        self.chunk_size = int(self.sample_rate * chunk_duration)
        self.audio_buffer = np.array([])
        
    def process_audio_chunk(self, audio_chunk):
        """处理音频块"""
        # 添加到缓冲区
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])
        
        results = []
        
        # 处理完整的块
        while len(self.audio_buffer) >= self.chunk_size:
            # 提取一个块
            chunk = self.audio_buffer[:self.chunk_size]
            self.audio_buffer = self.audio_buffer[self.chunk_size//2:]  # 50%重叠
            
            # 语音活动检测
            if self.is_speech(chunk):
                text = self.recognizer.recognize(chunk)
                if text:
                    results.append({
                        'text': text,
                        'confidence': self.calculate_confidence(chunk),
                        'timestamp': time.time()
                    })
        
        return results
    
    def is_speech(self, audio_chunk, threshold=0.01):
        """简单的语音活动检测"""
        energy = np.mean(audio_chunk ** 2)
        return energy > threshold
    
    def calculate_confidence(self, audio_chunk):
        """计算置信度"""
        # 基于信噪比的简单置信度计算
        signal_power = np.mean(audio_chunk ** 2)
        noise_power = np.mean(audio_chunk[:1000] ** 2)  # 假设前1000个样本是噪声
        
        if noise_power > 0:
            snr = 10 * np.log10(signal_power / noise_power)
            confidence = min(1.0, max(0.0, (snr + 10) / 30))  # 映射到0-1
        else:
            confidence = 1.0
        
        return confidence
```

### 2. 文本分析 (Text Analysis)

文本分析包括情感分析、文本分类、命名实体识别等任务。

#### 轻量级文本分类器

```python
import re
import json
from collections import Counter

class EdgeTextClassifier:
    """边缘文本分类器"""
    
    def __init__(self, model_path, tokenizer_path, max_length=128):
        self.model = self.load_model(model_path)
        self.tokenizer = self.load_tokenizer(tokenizer_path)
        self.max_length = max_length
        
    def load_model(self, model_path):
        """加载文本分类模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def load_tokenizer(self, tokenizer_path):
        """加载分词器"""
        with open(tokenizer_path, 'r', encoding='utf-8') as f:
            tokenizer_config = json.load(f)
        
        return {
            'vocab': tokenizer_config['vocab'],
            'word_index': tokenizer_config['word_index'],
            'oov_token': tokenizer_config.get('oov_token', '<UNK>')
        }
    
    def preprocess_text(self, text):
        """预处理文本"""
        # 清理文本
        text = text.lower()
        text = re.sub(r'[^\w\s]', '', text)  # 移除标点
        text = re.sub(r'\s+', ' ', text)     # 规范化空格
        text = text.strip()
        
        # 分词
        tokens = text.split()
        
        # 转换为索引
        word_index = self.tokenizer['word_index']
        oov_index = word_index.get(self.tokenizer['oov_token'], 1)
        
        indices = []
        for token in tokens:
            indices.append(word_index.get(token, oov_index))
        
        # 截断或填充
        if len(indices) > self.max_length:
            indices = indices[:self.max_length]
        else:
            indices.extend([0] * (self.max_length - len(indices)))
        
        return np.array(indices, dtype=np.int32)
    
    def classify(self, text):
        """文本分类"""
        # 预处理
        input_ids = self.preprocess_text(text)
        input_data = np.expand_dims(input_ids, axis=0).astype(np.float32)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(input_data)
        else:
            predictions = self.model.predict(input_data)
        
        # 后处理
        probabilities = tf.nn.softmax(predictions[0]).numpy()
        predicted_class = np.argmax(probabilities)
        confidence = probabilities[predicted_class]
        
        return {
            'class': int(predicted_class),
            'confidence': float(confidence),
            'probabilities': probabilities.tolist()
        }
    
    def predict_tflite(self, input_data):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_data)
        self.model.invoke()
        
        return self.model.get_tensor(output_details[0]['index'])

# 情感分析器
class EdgeSentimentAnalyzer(EdgeTextClassifier):
    """边缘情感分析器"""
    
    def __init__(self, model_path, tokenizer_path):
        super().__init__(model_path, tokenizer_path)
        self.sentiment_labels = ['negative', 'neutral', 'positive']
    
    def analyze_sentiment(self, text):
        """分析情感"""
        result = self.classify(text)
        
        sentiment_result = {
            'text': text,
            'sentiment': self.sentiment_labels[result['class']],
            'confidence': result['confidence'],
            'scores': {
                'negative': result['probabilities'][0],
                'neutral': result['probabilities'][1],
                'positive': result['probabilities'][2]
            }
        }
        
        return sentiment_result
    
    def batch_analyze(self, texts):
        """批量情感分析"""
        results = []
        for text in texts:
            result = self.analyze_sentiment(text)
            results.append(result)
        return results

# 命名实体识别
class EdgeNER:
    """边缘命名实体识别"""
    
    def __init__(self, model_path, tokenizer_path, label_path):
        self.model = self.load_model(model_path)
        self.tokenizer = self.load_tokenizer(tokenizer_path)
        self.labels = self.load_labels(label_path)
        self.max_length = 128
    
    def load_model(self, model_path):
        """加载NER模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def load_tokenizer(self, tokenizer_path):
        """加载分词器"""
        with open(tokenizer_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    
    def load_labels(self, label_path):
        """加载标签映射"""
        with open(label_path, 'r', encoding='utf-8') as f:
            labels = json.load(f)
        return {int(k): v for k, v in labels.items()}
    
    def tokenize_and_align(self, text):
        """分词并对齐"""
        # 简单的字符级分词
        tokens = list(text)
        
        # 转换为索引
        word_index = self.tokenizer['word_index']
        oov_index = word_index.get('<UNK>', 1)
        
        token_ids = []
        for token in tokens:
            token_ids.append(word_index.get(token, oov_index))
        
        # 截断或填充
        if len(token_ids) > self.max_length:
            token_ids = token_ids[:self.max_length]
            tokens = tokens[:self.max_length]
        else:
            padding_length = self.max_length - len(token_ids)
            token_ids.extend([0] * padding_length)
            tokens.extend([''] * padding_length)
        
        return np.array(token_ids), tokens
    
    def extract_entities(self, text):
        """提取命名实体"""
        # 分词
        token_ids, tokens = self.tokenize_and_align(text)
        input_data = np.expand_dims(token_ids, axis=0).astype(np.float32)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(input_data)
        else:
            predictions = self.model.predict(input_data)
        
        # 解码实体
        entities = self.decode_entities(predictions[0], tokens, text)
        
        return entities
    
    def predict_tflite(self, input_data):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_data)
        self.model.invoke()
        
        return self.model.get_tensor(output_details[0]['index'])
    
    def decode_entities(self, predictions, tokens, original_text):
        """解码实体"""
        predicted_labels = np.argmax(predictions, axis=-1)
        
        entities = []
        current_entity = None
        
        for i, (token, label_id) in enumerate(zip(tokens, predicted_labels)):
            if not token:  # 跳过填充token
                continue
                
            label = self.labels.get(label_id, 'O')
            
            if label.startswith('B-'):  # 实体开始
                if current_entity:
                    entities.append(current_entity)
                
                entity_type = label[2:]
                current_entity = {
                    'text': token,
                    'type': entity_type,
                    'start': i,
                    'end': i + 1,
                    'confidence': float(np.max(predictions[i]))
                }
            
            elif label.startswith('I-') and current_entity:  # 实体继续
                entity_type = label[2:]
                if entity_type == current_entity['type']:
                    current_entity['text'] += token
                    current_entity['end'] = i + 1
                else:
                    entities.append(current_entity)
                    current_entity = None
            
            else:  # O标签或其他
                if current_entity:
                    entities.append(current_entity)
                    current_entity = None
        
        if current_entity:
            entities.append(current_entity)
        
        return entities
```

### 3. 机器翻译 (Machine Translation)

边缘机器翻译能够在本地设备上提供实时翻译服务。

#### 轻量级翻译模型

```python
class EdgeTranslator:
    """边缘机器翻译器"""
    
    def __init__(self, model_path, src_tokenizer_path, tgt_tokenizer_path):
        self.model = self.load_model(model_path)
        self.src_tokenizer = self.load_tokenizer(src_tokenizer_path)
        self.tgt_tokenizer = self.load_tokenizer(tgt_tokenizer_path)
        self.max_length = 64
        
        # 构建目标词汇的反向映射
        self.tgt_index_to_word = {
            v: k for k, v in self.tgt_tokenizer['word_index'].items()
        }
    
    def load_model(self, model_path):
        """加载翻译模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def load_tokenizer(self, tokenizer_path):
        """加载分词器"""
        with open(tokenizer_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    
    def encode_text(self, text, tokenizer):
        """编码文本"""
        # 简单的分词
        tokens = text.lower().split()
        
        # 添加特殊token
        tokens = ['<start>'] + tokens + ['<end>']
        
        # 转换为索引
        word_index = tokenizer['word_index']
        oov_index = word_index.get('<UNK>', 1)
        
        indices = []
        for token in tokens:
            indices.append(word_index.get(token, oov_index))
        
        # 截断或填充
        if len(indices) > self.max_length:
            indices = indices[:self.max_length]
        else:
            indices.extend([0] * (self.max_length - len(indices)))
        
        return np.array(indices, dtype=np.int32)
    
    def decode_sequence(self, sequence):
        """解码序列"""
        words = []
        for idx in sequence:
            if idx == 0:  # 填充token
                break
            elif idx in self.tgt_index_to_word:
                word = self.tgt_index_to_word[idx]
                if word in ['<start>', '<end>']:
                    continue
                words.append(word)
        
        return ' '.join(words)
    
    def translate(self, text):
        """翻译文本"""
        # 编码源文本
        src_sequence = self.encode_text(text, self.src_tokenizer)
        input_data = np.expand_dims(src_sequence, axis=0).astype(np.float32)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(input_data)
        else:
            predictions = self.model.predict(input_data)
        
        # 解码目标序列
        if len(predictions.shape) == 3:  # 序列到序列输出
            predicted_sequence = np.argmax(predictions[0], axis=-1)
        else:  # 直接序列输出
            predicted_sequence = predictions[0]
        
        translated_text = self.decode_sequence(predicted_sequence)
        
        return {
            'source': text,
            'translation': translated_text,
            'confidence': self.calculate_translation_confidence(predictions)
        }
    
    def predict_tflite(self, input_data):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_data)
        self.model.invoke()
        
        return self.model.get_tensor(output_details[0]['index'])
    
    def calculate_translation_confidence(self, predictions):
        """计算翻译置信度"""
        if len(predictions.shape) == 3:
            # 计算每个位置的最大概率的平均值
            max_probs = np.max(tf.nn.softmax(predictions[0]), axis=-1)
            confidence = np.mean(max_probs[max_probs > 0])  # 忽略填充位置
        else:
            confidence = 1.0  # 简化处理
        
        return float(confidence)
    
    def batch_translate(self, texts):
        """批量翻译"""
        results = []
        for text in texts:
            result = self.translate(text)
            results.append(result)
        return results
```

### 4. 对话系统 (Dialogue System)

边缘对话系统能够在本地提供智能对话服务。

#### 轻量级对话机器人

```python
class EdgeChatbot:
    """边缘对话机器人"""
    
    def __init__(self, model_path, tokenizer_path, max_history=5):
        self.model = self.load_model(model_path)
        self.tokenizer = self.load_tokenizer(tokenizer_path)
        self.max_length = 128
        self.max_history = max_history
        self.conversation_history = []
        
        # 构建词汇的反向映射
        self.index_to_word = {
            v: k for k, v in self.tokenizer['word_index'].items()
        }
    
    def load_model(self, model_path):
        """加载对话模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def load_tokenizer(self, tokenizer_path):
        """加载分词器"""
        with open(tokenizer_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    
    def preprocess_input(self, user_input):
        """预处理用户输入"""
        # 构建上下文
        context = []
        
        # 添加历史对话
        for turn in self.conversation_history[-self.max_history:]:
            context.extend(['<user>', turn['user'], '<bot>', turn['bot']])
        
        # 添加当前用户输入
        context.extend(['<user>', user_input, '<bot>'])
        
        # 分词
        tokens = []
        for item in context:
            if item.startswith('<'):
                tokens.append(item)
            else:
                tokens.extend(item.lower().split())
        
        # 转换为索引
        word_index = self.tokenizer['word_index']
        oov_index = word_index.get('<UNK>', 1)
        
        indices = []
        for token in tokens:
            indices.append(word_index.get(token, oov_index))
        
        # 截断或填充
        if len(indices) > self.max_length:
            indices = indices[-self.max_length:]  # 保留最近的上下文
        else:
            indices = [0] * (self.max_length - len(indices)) + indices
        
        return np.array(indices, dtype=np.int32)
    
    def generate_response(self, user_input):
        """生成回复"""
        # 预处理输入
        input_sequence = self.preprocess_input(user_input)
        input_data = np.expand_dims(input_sequence, axis=0).astype(np.float32)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(input_data)
        else:
            predictions = self.model.predict(input_data)
        
        # 解码回复
        response = self.decode_response(predictions)
        
        # 更新对话历史
        self.conversation_history.append({
            'user': user_input,
            'bot': response,
            'timestamp': time.time()
        })
        
        # 限制历史长度
        if len(self.conversation_history) > self.max_history:
            self.conversation_history = self.conversation_history[-self.max_history:]
        
        return {
            'response': response,
            'confidence': self.calculate_response_confidence(predictions),
            'context_length': len(self.conversation_history)
        }
    
    def predict_tflite(self, input_data):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_data)
        self.model.invoke()
        
        return self.model.get_tensor(output_details[0]['index'])
    
    def decode_response(self, predictions):
        """解码回复"""
        if len(predictions.shape) == 3:
            # 序列生成模型
            predicted_sequence = np.argmax(predictions[0], axis=-1)
        else:
            # 分类模型（预定义回复）
            predicted_class = np.argmax(predictions[0])
            return self.get_predefined_response(predicted_class)
        
        # 解码序列
        words = []
        for idx in predicted_sequence:
            if idx == 0:  # 填充token
                continue
            elif idx in self.index_to_word:
                word = self.index_to_word[idx]
                if word == '<end>':
                    break
                elif word not in ['<start>', '<user>', '<bot>']:
                    words.append(word)
        
        return ' '.join(words)
    
    def get_predefined_response(self, class_id):
        """获取预定义回复"""
        responses = [
            "我不太理解你的意思，能再说一遍吗？",
            "这是一个有趣的问题。",
            "我需要更多信息来帮助你。",
            "谢谢你的问题。",
            "我正在学习中，请耐心等待。"
        ]
        
        if class_id < len(responses):
            return responses[class_id]
        else:
            return responses[0]
    
    def calculate_response_confidence(self, predictions):
        """计算回复置信度"""
        if len(predictions.shape) == 3:
            # 序列生成的置信度
            probs = tf.nn.softmax(predictions[0])
            max_probs = np.max(probs, axis=-1)
            confidence = np.mean(max_probs[max_probs > 0])
        else:
            # 分类的置信度
            probs = tf.nn.softmax(predictions[0])
            confidence = np.max(probs)
        
        return float(confidence)
    
    def reset_conversation(self):
        """重置对话历史"""
        self.conversation_history = []
    
    def get_conversation_summary(self):
        """获取对话摘要"""
        if not self.conversation_history:
            return "暂无对话历史"
        
        summary = {
            'total_turns': len(self.conversation_history),
            'start_time': self.conversation_history[0]['timestamp'],
            'last_time': self.conversation_history[-1]['timestamp'],
            'topics': self.extract_topics()
        }
        
        return summary
    
    def extract_topics(self):
        """提取对话主题"""
        # 简单的关键词提取
        all_text = ' '.join([
            turn['user'] + ' ' + turn['bot'] 
            for turn in self.conversation_history
        ])
        
        words = re.findall(r'\w+', all_text.lower())
        word_freq = Counter(words)
        
        # 过滤常见词
        stop_words = {'的', '是', '在', '了', '和', '有', '我', '你', '他', '她', '它'}
        topics = [word for word, freq in word_freq.most_common(10) 
                 if word not in stop_words and len(word) > 1]
        
        return topics[:5]  # 返回前5个主题
```

## 模型优化策略

### 1. 知识蒸馏

```python
class NLPModelDistiller:
    """NLP模型蒸馏器"""
    
    def __init__(self, teacher_model, student_model):
        self.teacher_model = teacher_model
        self.student_model = student_model
    
    def distill_language_model(self, train_dataset, temperature=4.0, alpha=0.7):
        """语言模型蒸馏"""
        
        class LanguageModelDistillationLoss(tf.keras.losses.Loss):
            def __init__(self, temperature, alpha):
                super().__init__()
                self.temperature = temperature
                self.alpha = alpha
            
            def call(self, y_true, y_pred):
                teacher_logits, student_logits = y_pred
                
                # 软目标损失
                teacher_probs = tf.nn.softmax(teacher_logits / self.temperature)
                student_log_probs = tf.nn.log_softmax(student_logits / self.temperature)
                soft_loss = -tf.reduce_sum(teacher_probs * student_log_probs, axis=-1)
                soft_loss = tf.reduce_mean(soft_loss) * (self.temperature ** 2)
                
                # 硬目标损失
                hard_loss = tf.keras.losses.sparse_categorical_crossentropy(
                    y_true, student_logits, from_logits=True
                )
                
                return self.alpha * soft_loss + (1 - self.alpha) * hard_loss
        
        # 创建蒸馏模型
        class DistillationModel(tf.keras.Model):
            def __init__(self, teacher, student):
                super().__init__()
                self.teacher = teacher
                self.student = student
            
            def call(self, x, training=None):
                teacher_logits = self.teacher(x, training=False)
                student_logits = self.student(x, training=training)
                return teacher_logits, student_logits
        
        distillation_model = DistillationModel(self.teacher_model, self.student_model)
        distillation_model.compile(
            optimizer='adam',
            loss=LanguageModelDistillationLoss(temperature, alpha)
        )
        
        return distillation_model
    
    def distill_sequence_model(self, train_dataset, temperature=3.0, alpha=0.5):
        """序列模型蒸馏"""
        
        class SequenceDistillationLoss(tf.keras.losses.Loss):
            def __init__(self, temperature, alpha):
                super().__init__()
                self.temperature = temperature
                self.alpha = alpha
            
            def call(self, y_true, y_pred):
                teacher_outputs, student_outputs = y_pred
                
                # 处理序列输出
                if len(teacher_outputs.shape) == 3:  # (batch, seq_len, vocab)
                    # 软目标损失
                    teacher_probs = tf.nn.softmax(teacher_outputs / self.temperature)
                    student_log_probs = tf.nn.log_softmax(student_outputs / self.temperature)
                    
                    soft_loss = -tf.reduce_sum(teacher_probs * student_log_probs, axis=-1)
                    soft_loss = tf.reduce_mean(soft_loss) * (self.temperature ** 2)
                    
                    # 硬目标损失
                    hard_loss = tf.keras.losses.sparse_categorical_crossentropy(
                        y_true, student_outputs, from_logits=True
                    )
                    hard_loss = tf.reduce_mean(hard_loss)
                    
                else:  # 分类输出
                    soft_loss = tf.keras.losses.KLDivergence()(
                        tf.nn.softmax(teacher_outputs / self.temperature),
                        tf.nn.softmax(student_outputs / self.temperature)
                    ) * (self.temperature ** 2)
                    
                    hard_loss = tf.keras.losses.sparse_categorical_crossentropy(
                        y_true, student_outputs, from_logits=True
                    )
                
                return self.alpha * soft_loss + (1 - self.alpha) * hard_loss
        
        return SequenceDistillationLoss(temperature, alpha)
```

### 2. 量化优化

```python
class NLPQuantizer:
    """NLP模型量化器"""
    
    def __init__(self, model):
        self.model = model
    
    def quantize_embedding_layer(self, embedding_layer, num_bits=8):
        """量化嵌入层"""
        weights = embedding_layer.get_weights()[0]
        
        # 计算量化参数
        min_val = np.min(weights)
        max_val = np.max(weights)
        
        # 量化
        scale = (max_val - min_val) / (2 ** num_bits - 1)
        zero_point = -min_val / scale
        
        quantized_weights = np.round(weights / scale + zero_point)
        quantized_weights = np.clip(quantized_weights, 0, 2 ** num_bits - 1)
        
        # 反量化用于验证
        dequantized_weights = (quantized_weights - zero_point) * scale
        
        return {
            'quantized_weights': quantized_weights.astype(np.uint8),
            'scale': scale,
            'zero_point': zero_point,
            'dequantized_weights': dequantized_weights
        }
    
    def quantize_attention_weights(self, attention_layer, num_bits=8):
        """量化注意力权重"""
        weights = attention_layer.get_weights()
        quantized_weights = []
        quantization_params = []
        
        for weight in weights:
            min_val = np.min(weight)
            max_val = np.max(weight)
            
            scale = (max_val - min_val) / (2 ** num_bits - 1)
            zero_point = -min_val / scale
            
            quantized = np.round(weight / scale + zero_point)
            quantized = np.clip(quantized, 0, 2 ** num_bits - 1)
            
            quantized_weights.append(quantized.astype(np.uint8))
            quantization_params.append({'scale': scale, 'zero_point': zero_point})
        
        return quantized_weights, quantization_params
    
    def create_quantized_model(self, representative_dataset):
        """创建量化模型"""
        def representative_dataset_gen():
            for data in representative_dataset:
                yield [data.astype(np.float32)]
        
        converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
        converter.optimizations = [tf.lite.Optimize.DEFAULT]
        converter.representative_dataset = representative_dataset_gen
        converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
        converter.inference_input_type = tf.int8
        converter.inference_output_type = tf.int8
        
        quantized_tflite_model = converter.convert()
        
        return quantized_tflite_model
```

## 部署和优化

### 1. 边缘部署管理

```python
class EdgeNLPDeployment:
    """边缘NLP部署管理"""
    
    def __init__(self, device_config):
        self.device_config = device_config
        self.models = {}
        self.performance_stats = {}
    
    def deploy_speech_recognition(self, model_path, vocab_path):
        """部署语音识别模型"""
        try:
            recognizer = EdgeSpeechRecognizer(model_path, vocab_path)
            self.models['speech_recognition'] = recognizer
            
            # 性能测试
            test_audio = np.random.random(16000).astype(np.float32)  # 1秒音频
            start_time = time.time()
            result = recognizer.recognize(test_audio)
            end_time = time.time()
            
            self.performance_stats['speech_recognition'] = {
                'latency_ms': (end_time - start_time) * 1000,
                'model_size_mb': os.path.getsize(model_path) / (1024 * 1024),
                'status': 'deployed'
            }
            
            return True
        except Exception as e:
            print(f"语音识别模型部署失败: {e}")
            return False
    
    def deploy_text_classifier(self, model_path, tokenizer_path):
        """部署文本分类模型"""
        try:
            classifier = EdgeTextClassifier(model_path, tokenizer_path)
            self.models['text_classifier'] = classifier
            
            # 性能测试
            test_text = "这是一个测试文本"
            start_time = time.time()
            result = classifier.classify(test_text)
            end_time = time.time()
            
            self.performance_stats['text_classifier'] = {
                'latency_ms': (end_time - start_time) * 1000,
                'model_size_mb': os.path.getsize(model_path) / (1024 * 1024),
                'status': 'deployed'
            }
            
            return True
        except Exception as e:
            print(f"文本分类模型部署失败: {e}")
            return False
    
    def deploy_translator(self, model_path, src_tokenizer_path, tgt_tokenizer_path):
        """部署翻译模型"""
        try:
            translator = EdgeTranslator(model_path, src_tokenizer_path, tgt_tokenizer_path)
            self.models['translator'] = translator
            
            # 性能测试
            test_text = "Hello world"
            start_time = time.time()
            result = translator.translate(test_text)
            end_time = time.time()
            
            self.performance_stats['translator'] = {
                'latency_ms': (end_time - start_time) * 1000,
                'model_size_mb': os.path.getsize(model_path) / (1024 * 1024),
                'status': 'deployed'
            }
            
            return True
        except Exception as e:
            print(f"翻译模型部署失败: {e}")
            return False
    
    def deploy_chatbot(self, model_path, tokenizer_path):
        """部署对话机器人"""
        try:
            chatbot = EdgeChatbot(model_path, tokenizer_path)
            self.models['chatbot'] = chatbot
            
            # 性能测试
            test_input = "你好"
            start_time = time.time()
            result = chatbot.generate_response(test_input)
            end_time = time.time()
            
            self.performance_stats['chatbot'] = {
                'latency_ms': (end_time - start_time) * 1000,
                'model_size_mb': os.path.getsize(model_path) / (1024 * 1024),
                'status': 'deployed'
            }
            
            return True
        except Exception as e:
            print(f"对话机器人部署失败: {e}")
            return False
    
    def get_deployment_status(self):
        """获取部署状态"""
        status = {
            'device_info': self.device_config,
            'deployed_models': list(self.models.keys()),
            'performance_stats': self.performance_stats,
            'total_memory_usage': self.calculate_total_memory_usage(),
            'system_resources': self.get_system_resources()
        }
        
        return status
    
    def calculate_total_memory_usage(self):
        """计算总内存使用"""
        total_size = 0
        for model_name, stats in self.performance_stats.items():
            total_size += stats.get('model_size_mb', 0)
        return total_size
    
    def get_system_resources(self):
        """获取系统资源信息"""
        import psutil
        
        return {
            'cpu_usage': psutil.cpu_percent(),
            'memory_usage': psutil.virtual_memory().percent,
            'available_memory_mb': psutil.virtual_memory().available / (1024 * 1024),
            'disk_usage': psutil.disk_usage('/').percent
        }
    
    def optimize_for_device(self):
        """针对设备优化"""
        device_type = self.device_config.get('type', 'cpu')
        
        if device_type == 'raspberry_pi':
            return self.optimize_for_raspberry_pi()
        elif device_type == 'jetson_nano':
            return self.optimize_for_jetson_nano()
        elif device_type == 'mobile':
            return self.optimize_for_mobile()
        else:
            return self.optimize_for_cpu()
    
    def optimize_for_raspberry_pi(self):
        """树莓派优化"""
        optimizations = {
            'max_threads': 4,
            'memory_limit_mb': 1024,
            'batch_size': 1,
            'use_quantization': True,
            'cache_models': False
        }
        
        return optimizations
    
    def optimize_for_jetson_nano(self):
        """Jetson Nano优化"""
        optimizations = {
            'use_gpu': True,
            'max_threads': 4,
            'memory_limit_mb': 2048,
            'batch_size': 4,
            'use_tensorrt': True,
            'cache_models': True
        }
        
        return optimizations
    
    def optimize_for_mobile(self):
        """移动设备优化"""
        optimizations = {
            'max_threads': 2,
            'memory_limit_mb': 512,
            'batch_size': 1,
            'use_quantization': True,
            'aggressive_optimization': True,
            'cache_models': False
        }
        
        return optimizations

# 使用示例
device_config = {
    'type': 'raspberry_pi',
    'memory_gb': 4,
    'cpu_cores': 4,
    'gpu': False
}

deployment = EdgeNLPDeployment(device_config)

# 部署各种NLP模型
deployment.deploy_speech_recognition('speech_model.tflite', 'vocab.txt')
deployment.deploy_text_classifier('classifier.tflite', 'tokenizer.json')
deployment.deploy_translator('translator.tflite', 'src_tokenizer.json', 'tgt_tokenizer.json')
deployment.deploy_chatbot('chatbot.tflite', 'chat_tokenizer.json')

# 获取部署状态
status = deployment.get_deployment_status()
print(json.dumps(status, indent=2, ensure_ascii=False))

# 获取优化建议
optimizations = deployment.optimize_for_device()
print("优化建议:", optimizations)
```

## 总结

边缘NLP技术为本地化的自然语言处理提供了强大的能力，主要优势包括：

1. **隐私保护**: 数据不需要上传到云端
2. **实时响应**: 无网络延迟，即时处理
3. **成本效益**: 减少云服务费用和带宽消耗
4. **可靠性**: 离线工作能力

关键技术要点：

- **模型压缩**: 通过量化、剪枝、蒸馏等技术减小模型大小
- **推理优化**: 利用硬件加速和算法优化提升性能
- **多任务集成**: 在单一设备上部署多种NLP功能
- **资源管理**: 合理分配计算和存储资源

通过合理的架构设计和优化策略，可以在资源受限的边缘设备上实现高效的NLP应用。
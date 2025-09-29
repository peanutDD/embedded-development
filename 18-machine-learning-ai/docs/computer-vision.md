# 计算机视觉应用

## 概述

计算机视觉是边缘AI最重要的应用领域之一，涵盖图像分类、目标检测、人脸识别、语义分割等多个方向。本文档详细介绍了边缘计算机视觉的核心技术、实现方法和最佳实践。

## 核心技术

### 1. 图像分类 (Image Classification)

图像分类是计算机视觉的基础任务，目标是将输入图像分配到预定义的类别中。

#### 经典架构

```python
import tensorflow as tf
from tensorflow.keras import layers, models

class MobileNetV3Small:
    """轻量级图像分类模型"""
    
    def __init__(self, num_classes=1000, input_shape=(224, 224, 3)):
        self.num_classes = num_classes
        self.input_shape = input_shape
        
    def build_model(self):
        """构建MobileNetV3-Small模型"""
        inputs = tf.keras.Input(shape=self.input_shape)
        
        # 初始卷积层
        x = layers.Conv2D(16, 3, strides=2, padding='same', use_bias=False)(inputs)
        x = layers.BatchNormalization()(x)
        x = self.hard_swish(x)
        
        # Inverted Residual Blocks
        x = self.inverted_residual_block(x, 16, 16, 3, 2, 'RE', False)
        x = self.inverted_residual_block(x, 72, 24, 3, 2, 'RE', False)
        x = self.inverted_residual_block(x, 88, 24, 3, 1, 'RE', False)
        x = self.inverted_residual_block(x, 96, 40, 5, 2, 'HS', True)
        x = self.inverted_residual_block(x, 240, 40, 5, 1, 'HS', True)
        x = self.inverted_residual_block(x, 240, 40, 5, 1, 'HS', True)
        x = self.inverted_residual_block(x, 120, 48, 5, 1, 'HS', True)
        x = self.inverted_residual_block(x, 144, 48, 5, 1, 'HS', True)
        x = self.inverted_residual_block(x, 288, 96, 5, 2, 'HS', True)
        x = self.inverted_residual_block(x, 576, 96, 5, 1, 'HS', True)
        x = self.inverted_residual_block(x, 576, 96, 5, 1, 'HS', True)
        
        # 最终卷积层
        x = layers.Conv2D(576, 1, padding='same', use_bias=False)(x)
        x = layers.BatchNormalization()(x)
        x = self.hard_swish(x)
        
        # 全局平均池化
        x = layers.GlobalAveragePooling2D()(x)
        
        # 分类头
        x = layers.Dense(1024, use_bias=False)(x)
        x = layers.BatchNormalization()(x)
        x = self.hard_swish(x)
        x = layers.Dropout(0.2)(x)
        
        outputs = layers.Dense(self.num_classes, activation='softmax')(x)
        
        return models.Model(inputs, outputs)
    
    def inverted_residual_block(self, x, expand_filters, out_filters, 
                               kernel_size, stride, activation, use_se):
        """倒残差块"""
        input_tensor = x
        
        # Expand
        if expand_filters != x.shape[-1]:
            x = layers.Conv2D(expand_filters, 1, padding='same', use_bias=False)(x)
            x = layers.BatchNormalization()(x)
            x = self.activation_fn(x, activation)
        
        # Depthwise
        x = layers.DepthwiseConv2D(kernel_size, strides=stride, 
                                  padding='same', use_bias=False)(x)
        x = layers.BatchNormalization()(x)
        x = self.activation_fn(x, activation)
        
        # Squeeze and Excitation
        if use_se:
            x = self.squeeze_excitation(x, expand_filters)
        
        # Project
        x = layers.Conv2D(out_filters, 1, padding='same', use_bias=False)(x)
        x = layers.BatchNormalization()(x)
        
        # Residual connection
        if stride == 1 and input_tensor.shape[-1] == out_filters:
            x = layers.Add()([input_tensor, x])
        
        return x
    
    def squeeze_excitation(self, x, filters):
        """SE注意力机制"""
        se = layers.GlobalAveragePooling2D()(x)
        se = layers.Reshape((1, 1, filters))(se)
        se = layers.Dense(filters // 4, activation='relu')(se)
        se = layers.Dense(filters, activation='sigmoid')(se)
        return layers.Multiply()([x, se])
    
    def hard_swish(self, x):
        """Hard Swish激活函数"""
        return x * tf.nn.relu6(x + 3) / 6
    
    def activation_fn(self, x, activation):
        """激活函数选择"""
        if activation == 'HS':
            return self.hard_swish(x)
        elif activation == 'RE':
            return tf.nn.relu(x)
        else:
            return x

# 使用示例
classifier = MobileNetV3Small(num_classes=10)
model = classifier.build_model()
model.summary()
```

#### 边缘优化策略

```python
class EdgeImageClassifier:
    """边缘图像分类器"""
    
    def __init__(self, model_path, input_size=(224, 224)):
        self.input_size = input_size
        self.model = self.load_optimized_model(model_path)
        self.preprocessing_pipeline = self.setup_preprocessing()
        
    def load_optimized_model(self, model_path):
        """加载优化后的模型"""
        if model_path.endswith('.tflite'):
            # TensorFlow Lite模型
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        elif model_path.endswith('.onnx'):
            # ONNX模型
            import onnxruntime as ort
            return ort.InferenceSession(model_path)
        else:
            # 标准TensorFlow模型
            return tf.keras.models.load_model(model_path)
    
    def setup_preprocessing(self):
        """设置预处理管道"""
        def preprocess(image):
            # 调整大小
            image = tf.image.resize(image, self.input_size)
            
            # 归一化
            image = tf.cast(image, tf.float32) / 255.0
            
            # 标准化
            mean = tf.constant([0.485, 0.456, 0.406])
            std = tf.constant([0.229, 0.224, 0.225])
            image = (image - mean) / std
            
            return image
        
        return preprocess
    
    def predict(self, image, top_k=5):
        """执行推理"""
        # 预处理
        processed_image = self.preprocessing_pipeline(image)
        processed_image = tf.expand_dims(processed_image, 0)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(processed_image)
        else:
            predictions = self.model.predict(processed_image)
        
        # 后处理
        top_indices = tf.nn.top_k(predictions[0], k=top_k).indices
        top_scores = tf.nn.top_k(predictions[0], k=top_k).values
        
        results = []
        for i in range(top_k):
            results.append({
                'class_id': int(top_indices[i]),
                'confidence': float(top_scores[i]),
                'class_name': self.get_class_name(int(top_indices[i]))
            })
        
        return results
    
    def predict_tflite(self, input_data):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_data.numpy())
        self.model.invoke()
        
        return [self.model.get_tensor(output_details[0]['index'])]
    
    def get_class_name(self, class_id):
        """获取类别名称"""
        # 这里应该加载实际的类别映射
        imagenet_classes = [
            'tench', 'goldfish', 'great_white_shark', 'tiger_shark',
            # ... 更多类别
        ]
        
        if class_id < len(imagenet_classes):
            return imagenet_classes[class_id]
        else:
            return f'class_{class_id}'
```

### 2. 目标检测 (Object Detection)

目标检测不仅要识别图像中的对象类别，还要定位对象的位置。

#### YOLO系列优化

```python
import numpy as np
import cv2

class YOLOv5EdgeDetector:
    """边缘优化的YOLOv5检测器"""
    
    def __init__(self, model_path, conf_threshold=0.5, nms_threshold=0.4):
        self.conf_threshold = conf_threshold
        self.nms_threshold = nms_threshold
        self.input_size = 640
        
        # 加载模型
        self.model = self.load_model(model_path)
        
        # COCO类别
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus',
            'train', 'truck', 'boat', 'traffic light', 'fire hydrant',
            'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog',
            'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
            'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            # ... 更多类别
        ]
    
    def load_model(self, model_path):
        """加载YOLO模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        elif model_path.endswith('.onnx'):
            import onnxruntime as ort
            return ort.InferenceSession(model_path)
        else:
            return tf.saved_model.load(model_path)
    
    def preprocess(self, image):
        """预处理图像"""
        # 保存原始尺寸
        original_height, original_width = image.shape[:2]
        
        # 等比例缩放
        scale = min(self.input_size / original_width, 
                   self.input_size / original_height)
        
        new_width = int(original_width * scale)
        new_height = int(original_height * scale)
        
        # 调整大小
        resized = cv2.resize(image, (new_width, new_height))
        
        # 填充到目标尺寸
        padded = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        
        # 计算填充位置
        pad_x = (self.input_size - new_width) // 2
        pad_y = (self.input_size - new_height) // 2
        
        padded[pad_y:pad_y + new_height, pad_x:pad_x + new_width] = resized
        
        # 归一化
        padded = padded.astype(np.float32) / 255.0
        
        # 转换为模型输入格式
        input_tensor = np.expand_dims(padded, axis=0)
        
        return input_tensor, scale, pad_x, pad_y
    
    def detect(self, image):
        """执行目标检测"""
        # 预处理
        input_tensor, scale, pad_x, pad_y = self.preprocess(image)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(input_tensor)
        else:
            predictions = self.model(input_tensor)
        
        # 后处理
        detections = self.postprocess(predictions, scale, pad_x, pad_y, 
                                    image.shape[:2])
        
        return detections
    
    def predict_tflite(self, input_tensor):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_tensor)
        self.model.invoke()
        
        outputs = []
        for output_detail in output_details:
            outputs.append(self.model.get_tensor(output_detail['index']))
        
        return outputs[0]  # YOLOv5通常只有一个输出
    
    def postprocess(self, predictions, scale, pad_x, pad_y, original_shape):
        """后处理检测结果"""
        original_height, original_width = original_shape
        
        # 解析预测结果
        boxes = []
        confidences = []
        class_ids = []
        
        for detection in predictions[0]:  # 批次维度为1
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            
            if confidence > self.conf_threshold:
                # 边界框坐标 (中心点格式)
                center_x = detection[0]
                center_y = detection[1]
                width = detection[2]
                height = detection[3]
                
                # 转换为左上角格式
                x = center_x - width / 2
                y = center_y - height / 2
                
                # 坐标反变换
                x = (x - pad_x) / scale
                y = (y - pad_y) / scale
                width = width / scale
                height = height / scale
                
                # 裁剪到图像边界
                x = max(0, min(x, original_width))
                y = max(0, min(y, original_height))
                width = min(width, original_width - x)
                height = min(height, original_height - y)
                
                boxes.append([x, y, width, height])
                confidences.append(float(confidence))
                class_ids.append(class_id)
        
        # 非极大值抑制
        if len(boxes) > 0:
            indices = cv2.dnn.NMSBoxes(boxes, confidences, 
                                     self.conf_threshold, self.nms_threshold)
            
            final_detections = []
            if len(indices) > 0:
                for i in indices.flatten():
                    x, y, w, h = boxes[i]
                    final_detections.append({
                        'bbox': [int(x), int(y), int(w), int(h)],
                        'confidence': confidences[i],
                        'class_id': class_ids[i],
                        'class_name': self.class_names[class_ids[i]]
                    })
            
            return final_detections
        
        return []
    
    def draw_detections(self, image, detections):
        """绘制检测结果"""
        result_image = image.copy()
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class_name']
            
            # 绘制边界框
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 绘制标签
            label = f"{class_name}: {confidence:.2f}"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
            
            cv2.rectangle(result_image, (x, y - label_size[1] - 10), 
                         (x + label_size[0], y), (0, 255, 0), -1)
            
            cv2.putText(result_image, label, (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return result_image

# 使用示例
detector = YOLOv5EdgeDetector('yolov5s.tflite')
image = cv2.imread('test_image.jpg')
detections = detector.detect(image)
result_image = detector.draw_detections(image, detections)
cv2.imshow('Detection Result', result_image)
cv2.waitKey(0)
```

### 3. 人脸识别 (Face Recognition)

人脸识别是边缘AI的重要应用，包括人脸检测、特征提取和身份验证。

#### 轻量级人脸识别系统

```python
class EdgeFaceRecognition:
    """边缘人脸识别系统"""
    
    def __init__(self, face_detector_path, face_encoder_path):
        # 人脸检测器
        self.face_detector = self.load_face_detector(face_detector_path)
        
        # 人脸编码器
        self.face_encoder = self.load_face_encoder(face_encoder_path)
        
        # 已知人脸数据库
        self.known_faces = {}
        self.face_threshold = 0.6
    
    def load_face_detector(self, model_path):
        """加载人脸检测模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def load_face_encoder(self, model_path):
        """加载人脸编码模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def detect_faces(self, image):
        """检测人脸"""
        # 预处理
        input_size = 320
        resized = cv2.resize(image, (input_size, input_size))
        normalized = resized.astype(np.float32) / 255.0
        input_tensor = np.expand_dims(normalized, axis=0)
        
        # 推理
        if isinstance(self.face_detector, tf.lite.Interpreter):
            predictions = self.predict_face_detector_tflite(input_tensor)
        else:
            predictions = self.face_detector.predict(input_tensor)
        
        # 解析检测结果
        faces = self.parse_face_detections(predictions, image.shape)
        
        return faces
    
    def predict_face_detector_tflite(self, input_tensor):
        """TensorFlow Lite人脸检测推理"""
        input_details = self.face_detector.get_input_details()
        output_details = self.face_detector.get_output_details()
        
        self.face_detector.set_tensor(input_details[0]['index'], input_tensor)
        self.face_detector.invoke()
        
        # 获取输出
        boxes = self.face_detector.get_tensor(output_details[0]['index'])
        scores = self.face_detector.get_tensor(output_details[1]['index'])
        
        return {'boxes': boxes, 'scores': scores}
    
    def parse_face_detections(self, predictions, image_shape):
        """解析人脸检测结果"""
        height, width = image_shape[:2]
        faces = []
        
        boxes = predictions['boxes'][0]  # 移除批次维度
        scores = predictions['scores'][0]
        
        for i, score in enumerate(scores):
            if score > 0.5:  # 置信度阈值
                box = boxes[i]
                
                # 转换坐标格式
                y1, x1, y2, x2 = box
                x1 = int(x1 * width)
                y1 = int(y1 * height)
                x2 = int(x2 * width)
                y2 = int(y2 * height)
                
                faces.append({
                    'bbox': [x1, y1, x2 - x1, y2 - y1],
                    'confidence': float(score)
                })
        
        return faces
    
    def extract_face_encoding(self, image, face_bbox):
        """提取人脸特征编码"""
        x, y, w, h = face_bbox
        
        # 裁剪人脸区域
        face_crop = image[y:y+h, x:x+w]
        
        # 预处理
        face_size = 112
        face_resized = cv2.resize(face_crop, (face_size, face_size))
        face_normalized = face_resized.astype(np.float32) / 255.0
        
        # 标准化
        mean = np.array([0.5, 0.5, 0.5])
        std = np.array([0.5, 0.5, 0.5])
        face_standardized = (face_normalized - mean) / std
        
        input_tensor = np.expand_dims(face_standardized, axis=0)
        
        # 推理
        if isinstance(self.face_encoder, tf.lite.Interpreter):
            encoding = self.predict_face_encoder_tflite(input_tensor)
        else:
            encoding = self.face_encoder.predict(input_tensor)
        
        # L2归一化
        encoding = encoding / np.linalg.norm(encoding)
        
        return encoding[0]
    
    def predict_face_encoder_tflite(self, input_tensor):
        """TensorFlow Lite人脸编码推理"""
        input_details = self.face_encoder.get_input_details()
        output_details = self.face_encoder.get_output_details()
        
        self.face_encoder.set_tensor(input_details[0]['index'], input_tensor)
        self.face_encoder.invoke()
        
        return self.face_encoder.get_tensor(output_details[0]['index'])
    
    def add_known_face(self, name, image):
        """添加已知人脸"""
        faces = self.detect_faces(image)
        
        if len(faces) == 1:
            encoding = self.extract_face_encoding(image, faces[0]['bbox'])
            self.known_faces[name] = encoding
            return True
        else:
            print(f"图像中检测到 {len(faces)} 张人脸，需要恰好1张")
            return False
    
    def recognize_faces(self, image):
        """识别图像中的人脸"""
        faces = self.detect_faces(image)
        results = []
        
        for face in faces:
            # 提取特征
            encoding = self.extract_face_encoding(image, face['bbox'])
            
            # 与已知人脸比较
            best_match = None
            min_distance = float('inf')
            
            for name, known_encoding in self.known_faces.items():
                # 计算欧氏距离
                distance = np.linalg.norm(encoding - known_encoding)
                
                if distance < min_distance:
                    min_distance = distance
                    best_match = name
            
            # 判断是否匹配
            if min_distance < self.face_threshold:
                identity = best_match
                confidence = 1 - min_distance
            else:
                identity = "Unknown"
                confidence = 0.0
            
            results.append({
                'bbox': face['bbox'],
                'identity': identity,
                'confidence': confidence,
                'distance': min_distance
            })
        
        return results
    
    def draw_recognition_results(self, image, results):
        """绘制识别结果"""
        result_image = image.copy()
        
        for result in results:
            x, y, w, h = result['bbox']
            identity = result['identity']
            confidence = result['confidence']
            
            # 选择颜色
            color = (0, 255, 0) if identity != "Unknown" else (0, 0, 255)
            
            # 绘制边界框
            cv2.rectangle(result_image, (x, y), (x + w, y + h), color, 2)
            
            # 绘制标签
            if identity != "Unknown":
                label = f"{identity}: {confidence:.2f}"
            else:
                label = "Unknown"
            
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            cv2.rectangle(result_image, (x, y - label_size[1] - 10), 
                         (x + label_size[0], y), color, -1)
            
            cv2.putText(result_image, label, (x, y - 5), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return result_image

# 使用示例
face_recognition = EdgeFaceRecognition(
    'face_detector.tflite', 
    'face_encoder.tflite'
)

# 添加已知人脸
face_recognition.add_known_face('Alice', cv2.imread('alice.jpg'))
face_recognition.add_known_face('Bob', cv2.imread('bob.jpg'))

# 识别人脸
test_image = cv2.imread('test.jpg')
results = face_recognition.recognize_faces(test_image)
result_image = face_recognition.draw_recognition_results(test_image, results)

cv2.imshow('Face Recognition', result_image)
cv2.waitKey(0)
```

### 4. 语义分割 (Semantic Segmentation)

语义分割为图像中的每个像素分配类别标签。

#### 轻量级分割模型

```python
class EdgeSemanticSegmentation:
    """边缘语义分割"""
    
    def __init__(self, model_path, num_classes=21):
        self.num_classes = num_classes
        self.model = self.load_model(model_path)
        self.input_size = (512, 512)
        
        # PASCAL VOC类别
        self.class_names = [
            'background', 'aeroplane', 'bicycle', 'bird', 'boat',
            'bottle', 'bus', 'car', 'cat', 'chair', 'cow',
            'diningtable', 'dog', 'horse', 'motorbike', 'person',
            'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor'
        ]
        
        # 类别颜色映射
        self.colors = self.generate_colors()
    
    def load_model(self, model_path):
        """加载分割模型"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(model_path=model_path)
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def generate_colors(self):
        """生成类别颜色"""
        colors = []
        for i in range(self.num_classes):
            # 使用HSV色彩空间生成不同颜色
            hue = (i * 180) // self.num_classes
            color = cv2.cvtColor(np.uint8([[[hue, 255, 255]]]), cv2.COLOR_HSV2BGR)[0][0]
            colors.append(tuple(map(int, color)))
        return colors
    
    def preprocess(self, image):
        """预处理图像"""
        # 保存原始尺寸
        original_height, original_width = image.shape[:2]
        
        # 调整大小
        resized = cv2.resize(image, self.input_size)
        
        # 归一化
        normalized = resized.astype(np.float32) / 255.0
        
        # 标准化
        mean = np.array([0.485, 0.456, 0.406])
        std = np.array([0.229, 0.224, 0.225])
        standardized = (normalized - mean) / std
        
        # 添加批次维度
        input_tensor = np.expand_dims(standardized, axis=0)
        
        return input_tensor, (original_width, original_height)
    
    def segment(self, image):
        """执行语义分割"""
        # 预处理
        input_tensor, original_size = self.preprocess(image)
        
        # 推理
        if isinstance(self.model, tf.lite.Interpreter):
            predictions = self.predict_tflite(input_tensor)
        else:
            predictions = self.model.predict(input_tensor)
        
        # 后处理
        segmentation_map = self.postprocess(predictions, original_size)
        
        return segmentation_map
    
    def predict_tflite(self, input_tensor):
        """TensorFlow Lite推理"""
        input_details = self.model.get_input_details()
        output_details = self.model.get_output_details()
        
        self.model.set_tensor(input_details[0]['index'], input_tensor)
        self.model.invoke()
        
        return self.model.get_tensor(output_details[0]['index'])
    
    def postprocess(self, predictions, original_size):
        """后处理分割结果"""
        original_width, original_height = original_size
        
        # 获取类别预测
        segmentation = np.argmax(predictions[0], axis=-1)
        
        # 调整回原始尺寸
        segmentation_resized = cv2.resize(
            segmentation.astype(np.uint8), 
            (original_width, original_height), 
            interpolation=cv2.INTER_NEAREST
        )
        
        return segmentation_resized
    
    def create_colored_mask(self, segmentation_map):
        """创建彩色分割掩码"""
        height, width = segmentation_map.shape
        colored_mask = np.zeros((height, width, 3), dtype=np.uint8)
        
        for class_id in range(self.num_classes):
            mask = segmentation_map == class_id
            colored_mask[mask] = self.colors[class_id]
        
        return colored_mask
    
    def overlay_segmentation(self, image, segmentation_map, alpha=0.6):
        """将分割结果叠加到原图"""
        colored_mask = self.create_colored_mask(segmentation_map)
        
        # 叠加
        overlayed = cv2.addWeighted(image, 1 - alpha, colored_mask, alpha, 0)
        
        return overlayed
    
    def get_class_statistics(self, segmentation_map):
        """获取类别统计信息"""
        total_pixels = segmentation_map.size
        statistics = {}
        
        for class_id in range(self.num_classes):
            pixel_count = np.sum(segmentation_map == class_id)
            percentage = (pixel_count / total_pixels) * 100
            
            if pixel_count > 0:
                statistics[self.class_names[class_id]] = {
                    'pixel_count': int(pixel_count),
                    'percentage': round(percentage, 2)
                }
        
        return statistics

# 使用示例
segmenter = EdgeSemanticSegmentation('deeplabv3_mobilenetv2.tflite')

image = cv2.imread('test_image.jpg')
segmentation_map = segmenter.segment(image)

# 创建可视化结果
colored_mask = segmenter.create_colored_mask(segmentation_map)
overlayed_result = segmenter.overlay_segmentation(image, segmentation_map)

# 获取统计信息
statistics = segmenter.get_class_statistics(segmentation_map)
print("类别统计:")
for class_name, stats in statistics.items():
    print(f"{class_name}: {stats['percentage']:.1f}%")

# 显示结果
cv2.imshow('Original', image)
cv2.imshow('Segmentation', colored_mask)
cv2.imshow('Overlay', overlayed_result)
cv2.waitKey(0)
```

## 性能优化策略

### 1. 模型压缩

```python
class ModelCompressor:
    """模型压缩工具"""
    
    def __init__(self, model):
        self.model = model
    
    def quantize_model(self, quantization_type='dynamic'):
        """模型量化"""
        if quantization_type == 'dynamic':
            # 动态量化
            converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
            quantized_model = converter.convert()
            
        elif quantization_type == 'int8':
            # INT8量化
            converter = tf.lite.TFLiteConverter.from_keras_model(self.model)
            converter.optimizations = [tf.lite.Optimize.DEFAULT]
            converter.representative_dataset = self.representative_dataset_gen
            converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
            converter.inference_input_type = tf.int8
            converter.inference_output_type = tf.int8
            quantized_model = converter.convert()
            
        return quantized_model
    
    def representative_dataset_gen(self):
        """代表性数据集生成器"""
        # 这里应该提供真实的代表性数据
        for _ in range(100):
            data = np.random.random((1, 224, 224, 3)).astype(np.float32)
            yield [data]
    
    def prune_model(self, target_sparsity=0.5):
        """模型剪枝"""
        import tensorflow_model_optimization as tfmot
        
        # 定义剪枝参数
        pruning_params = {
            'pruning_schedule': tfmot.sparsity.keras.PolynomialDecay(
                initial_sparsity=0.0,
                final_sparsity=target_sparsity,
                begin_step=0,
                end_step=1000
            )
        }
        
        # 应用剪枝
        pruned_model = tfmot.sparsity.keras.prune_low_magnitude(
            self.model, **pruning_params
        )
        
        return pruned_model
    
    def knowledge_distillation(self, teacher_model, student_model, 
                              train_dataset, temperature=3.0, alpha=0.7):
        """知识蒸馏"""
        class DistillationLoss(tf.keras.losses.Loss):
            def __init__(self, temperature, alpha):
                super().__init__()
                self.temperature = temperature
                self.alpha = alpha
            
            def call(self, y_true, y_pred):
                teacher_pred, student_pred = y_pred
                
                # 软目标损失
                soft_targets = tf.nn.softmax(teacher_pred / self.temperature)
                soft_prob = tf.nn.softmax(student_pred / self.temperature)
                soft_loss = tf.keras.losses.categorical_crossentropy(
                    soft_targets, soft_prob
                )
                
                # 硬目标损失
                hard_loss = tf.keras.losses.categorical_crossentropy(
                    y_true, student_pred
                )
                
                return self.alpha * soft_loss + (1 - self.alpha) * hard_loss
        
        # 创建蒸馏模型
        class DistillationModel(tf.keras.Model):
            def __init__(self, teacher, student):
                super().__init__()
                self.teacher = teacher
                self.student = student
            
            def call(self, x):
                teacher_pred = self.teacher(x, training=False)
                student_pred = self.student(x, training=True)
                return teacher_pred, student_pred
        
        distillation_model = DistillationModel(teacher_model, student_model)
        distillation_model.compile(
            optimizer='adam',
            loss=DistillationLoss(temperature, alpha),
            metrics=['accuracy']
        )
        
        return distillation_model
```

### 2. 推理加速

```python
class InferenceAccelerator:
    """推理加速器"""
    
    def __init__(self, model_path):
        self.model_path = model_path
        self.batch_size = 1
        self.use_gpu = self.check_gpu_availability()
    
    def check_gpu_availability(self):
        """检查GPU可用性"""
        return len(tf.config.list_physical_devices('GPU')) > 0
    
    def setup_gpu_memory_growth(self):
        """设置GPU内存增长"""
        if self.use_gpu:
            gpus = tf.config.list_physical_devices('GPU')
            if gpus:
                try:
                    for gpu in gpus:
                        tf.config.experimental.set_memory_growth(gpu, True)
                except RuntimeError as e:
                    print(f"GPU设置错误: {e}")
    
    def create_optimized_session(self):
        """创建优化的推理会话"""
        if self.model_path.endswith('.onnx'):
            import onnxruntime as ort
            
            # ONNX Runtime优化
            sess_options = ort.SessionOptions()
            sess_options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            sess_options.intra_op_num_threads = 0  # 使用所有可用线程
            
            providers = ['CPUExecutionProvider']
            if self.use_gpu:
                providers.insert(0, 'CUDAExecutionProvider')
            
            session = ort.InferenceSession(
                self.model_path, 
                sess_options=sess_options,
                providers=providers
            )
            
            return session
        
        elif self.model_path.endswith('.tflite'):
            # TensorFlow Lite优化
            interpreter = tf.lite.Interpreter(
                model_path=self.model_path,
                num_threads=0  # 使用所有可用线程
            )
            interpreter.allocate_tensors()
            
            return interpreter
    
    def batch_inference(self, images, batch_size=8):
        """批量推理"""
        results = []
        
        for i in range(0, len(images), batch_size):
            batch = images[i:i + batch_size]
            
            # 预处理批次
            batch_tensor = np.stack([self.preprocess(img) for img in batch])
            
            # 推理
            batch_results = self.model.predict(batch_tensor)
            
            # 后处理
            for j, result in enumerate(batch_results):
                processed_result = self.postprocess(result, batch[j].shape)
                results.append(processed_result)
        
        return results
    
    def pipeline_inference(self, image_stream):
        """流水线推理"""
        import threading
        import queue
        
        # 创建队列
        input_queue = queue.Queue(maxsize=10)
        output_queue = queue.Queue(maxsize=10)
        
        def preprocess_worker():
            """预处理工作线程"""
            for image in image_stream:
                processed = self.preprocess(image)
                input_queue.put((image, processed))
            input_queue.put(None)  # 结束标志
        
        def inference_worker():
            """推理工作线程"""
            while True:
                item = input_queue.get()
                if item is None:
                    output_queue.put(None)
                    break
                
                original_image, processed_image = item
                result = self.model.predict(np.expand_dims(processed_image, 0))
                output_queue.put((original_image, result[0]))
        
        # 启动工作线程
        preprocess_thread = threading.Thread(target=preprocess_worker)
        inference_thread = threading.Thread(target=inference_worker)
        
        preprocess_thread.start()
        inference_thread.start()
        
        # 收集结果
        results = []
        while True:
            item = output_queue.get()
            if item is None:
                break
            
            original_image, inference_result = item
            final_result = self.postprocess(inference_result, original_image.shape)
            results.append(final_result)
        
        # 等待线程结束
        preprocess_thread.join()
        inference_thread.join()
        
        return results
```

## 部署实践

### 1. 边缘设备部署

```python
class EdgeDeployment:
    """边缘设备部署管理"""
    
    def __init__(self, device_type='cpu'):
        self.device_type = device_type
        self.models = {}
        self.performance_monitor = self.setup_monitoring()
    
    def setup_monitoring(self):
        """设置性能监控"""
        return {
            'inference_times': [],
            'memory_usage': [],
            'cpu_usage': [],
            'temperature': []
        }
    
    def deploy_model(self, model_name, model_path, config):
        """部署模型"""
        try:
            # 根据设备类型选择最优配置
            if self.device_type == 'raspberry_pi':
                model = self.deploy_to_raspberry_pi(model_path, config)
            elif self.device_type == 'jetson_nano':
                model = self.deploy_to_jetson_nano(model_path, config)
            elif self.device_type == 'coral_tpu':
                model = self.deploy_to_coral_tpu(model_path, config)
            else:
                model = self.deploy_to_cpu(model_path, config)
            
            self.models[model_name] = {
                'model': model,
                'config': config,
                'stats': {'total_inferences': 0, 'avg_latency': 0}
            }
            
            print(f"模型 {model_name} 部署成功")
            return True
            
        except Exception as e:
            print(f"模型部署失败: {e}")
            return False
    
    def deploy_to_raspberry_pi(self, model_path, config):
        """部署到树莓派"""
        # 树莓派优化配置
        interpreter = tf.lite.Interpreter(
            model_path=model_path,
            num_threads=4  # 树莓派4核心
        )
        interpreter.allocate_tensors()
        
        return interpreter
    
    def deploy_to_jetson_nano(self, model_path, config):
        """部署到Jetson Nano"""
        # Jetson Nano GPU加速
        if model_path.endswith('.trt'):
            # TensorRT引擎
            import tensorrt as trt
            import pycuda.driver as cuda
            
            TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
            
            with open(model_path, 'rb') as f:
                engine_data = f.read()
            
            runtime = trt.Runtime(TRT_LOGGER)
            engine = runtime.deserialize_cuda_engine(engine_data)
            
            return engine
        else:
            # TensorFlow Lite GPU委托
            try:
                delegate = tf.lite.experimental.load_delegate('libedgetpu.so.1')
                interpreter = tf.lite.Interpreter(
                    model_path=model_path,
                    experimental_delegates=[delegate]
                )
            except:
                # 回退到CPU
                interpreter = tf.lite.Interpreter(model_path=model_path)
            
            interpreter.allocate_tensors()
            return interpreter
    
    def deploy_to_coral_tpu(self, model_path, config):
        """部署到Coral TPU"""
        try:
            from pycoral.utils import edgetpu
            from pycoral.adapters import common
            
            # 加载Edge TPU模型
            interpreter = edgetpu.make_interpreter(model_path)
            interpreter.allocate_tensors()
            
            return interpreter
        except ImportError:
            print("Coral TPU库未安装，回退到CPU")
            return self.deploy_to_cpu(model_path, config)
    
    def deploy_to_cpu(self, model_path, config):
        """部署到CPU"""
        if model_path.endswith('.tflite'):
            interpreter = tf.lite.Interpreter(
                model_path=model_path,
                num_threads=config.get('num_threads', 0)
            )
            interpreter.allocate_tensors()
            return interpreter
        else:
            return tf.keras.models.load_model(model_path)
    
    def inference(self, model_name, input_data):
        """执行推理"""
        if model_name not in self.models:
            raise ValueError(f"模型 {model_name} 未部署")
        
        model_info = self.models[model_name]
        model = model_info['model']
        
        # 记录开始时间
        start_time = time.time()
        
        # 执行推理
        if isinstance(model, tf.lite.Interpreter):
            result = self.tflite_inference(model, input_data)
        else:
            result = model.predict(input_data)
        
        # 记录结束时间
        end_time = time.time()
        inference_time = (end_time - start_time) * 1000  # 毫秒
        
        # 更新统计信息
        stats = model_info['stats']
        stats['total_inferences'] += 1
        stats['avg_latency'] = (
            (stats['avg_latency'] * (stats['total_inferences'] - 1) + inference_time) /
            stats['total_inferences']
        )
        
        # 记录性能数据
        self.performance_monitor['inference_times'].append(inference_time)
        
        return result
    
    def tflite_inference(self, interpreter, input_data):
        """TensorFlow Lite推理"""
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
        
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        
        return interpreter.get_tensor(output_details[0]['index'])
    
    def get_performance_report(self):
        """获取性能报告"""
        import psutil
        
        report = {
            'models': {},
            'system': {
                'cpu_usage': psutil.cpu_percent(),
                'memory_usage': psutil.virtual_memory().percent,
                'disk_usage': psutil.disk_usage('/').percent
            }
        }
        
        for model_name, model_info in self.models.items():
            stats = model_info['stats']
            report['models'][model_name] = {
                'total_inferences': stats['total_inferences'],
                'average_latency_ms': round(stats['avg_latency'], 2),
                'throughput_fps': round(1000 / stats['avg_latency'], 2) if stats['avg_latency'] > 0 else 0
            }
        
        return report

# 使用示例
deployment = EdgeDeployment(device_type='raspberry_pi')

# 部署模型
deployment.deploy_model(
    'face_detector', 
    'face_detector.tflite',
    {'num_threads': 4}
)

# 执行推理
test_image = np.random.random((1, 320, 320, 3)).astype(np.float32)
result = deployment.inference('face_detector', test_image)

# 获取性能报告
report = deployment.get_performance_report()
print(json.dumps(report, indent=2))
```

## 总结

边缘计算机视觉应用需要在性能、精度和资源消耗之间找到平衡。通过合理的模型选择、优化策略和部署方案，可以在资源受限的边缘设备上实现高效的计算机视觉功能。

关键要点：

1. **模型选择**: 根据应用场景选择合适的轻量级模型
2. **优化策略**: 采用量化、剪枝、知识蒸馏等技术压缩模型
3. **推理加速**: 利用硬件加速、批处理、流水线等技术提升性能
4. **部署实践**: 针对不同边缘设备采用相应的部署策略
5. **性能监控**: 持续监控和优化系统性能

通过系统性的方法，可以构建高效、可靠的边缘计算机视觉应用。
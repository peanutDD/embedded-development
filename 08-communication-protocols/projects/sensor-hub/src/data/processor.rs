//! 数据处理器模块
//! 
//! 负责对收集到的传感器数据进行处理、过滤、校准和分析。
//! 提供数据质量评估、异常检测、数据融合等功能。

use core::fmt;
use heapless::{Vec, FnvIndexMap};
use libm::{fabsf, sqrtf, powf};
use crate::sensors::{SensorData, SensorType};

/// 处理器错误类型
#[derive(Debug, Clone, PartialEq)]
pub enum ProcessorError {
    /// 数据无效
    InvalidData,
    /// 缓冲区满
    BufferFull,
    /// 配置错误
    ConfigurationError,
    /// 处理超时
    ProcessingTimeout,
    /// 内存不足
    OutOfMemory,
    /// 算法错误
    AlgorithmError,
    /// 校准数据缺失
    CalibrationMissing,
}

impl fmt::Display for ProcessorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ProcessorError::InvalidData => write!(f, "Invalid data"),
            ProcessorError::BufferFull => write!(f, "Processing buffer is full"),
            ProcessorError::ConfigurationError => write!(f, "Configuration error"),
            ProcessorError::ProcessingTimeout => write!(f, "Processing timeout"),
            ProcessorError::OutOfMemory => write!(f, "Out of memory"),
            ProcessorError::AlgorithmError => write!(f, "Algorithm error"),
            ProcessorError::CalibrationMissing => write!(f, "Calibration data missing"),
        }
    }
}

/// 数据质量等级
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataQuality {
    /// 高质量
    High,
    /// 中等质量
    Medium,
    /// 低质量
    Low,
    /// 无效数据
    Invalid,
}

/// 滤波器类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FilterType {
    /// 无滤波
    None,
    /// 移动平均
    MovingAverage,
    /// 指数移动平均
    ExponentialMovingAverage,
    /// 卡尔曼滤波
    Kalman,
    /// 中值滤波
    Median,
    /// 低通滤波
    LowPass,
}

/// 异常检测方法
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnomalyDetectionMethod {
    /// 无检测
    None,
    /// 阈值检测
    Threshold,
    /// 统计检测 (Z-score)
    Statistical,
    /// 滑动窗口检测
    SlidingWindow,
    /// 梯度检测
    Gradient,
}

/// 处理配置
#[derive(Debug, Clone)]
pub struct ProcessingConfig {
    /// 启用的滤波器
    pub filter_type: FilterType,
    /// 滤波器参数
    pub filter_params: FilterParams,
    /// 异常检测方法
    pub anomaly_detection: AnomalyDetectionMethod,
    /// 异常检测参数
    pub anomaly_params: AnomalyParams,
    /// 校准启用
    pub calibration_enabled: bool,
    /// 数据融合启用
    pub fusion_enabled: bool,
    /// 处理窗口大小
    pub window_size: usize,
    /// 质量评估启用
    pub quality_assessment: bool,
}

/// 滤波器参数
#[derive(Debug, Clone)]
pub struct FilterParams {
    /// 移动平均窗口大小
    pub ma_window_size: usize,
    /// 指数移动平均alpha值
    pub ema_alpha: f32,
    /// 卡尔曼滤波参数
    pub kalman_q: f32, // 过程噪声
    pub kalman_r: f32, // 测量噪声
    /// 低通滤波截止频率
    pub lowpass_cutoff: f32,
}

/// 异常检测参数
#[derive(Debug, Clone)]
pub struct AnomalyParams {
    /// 阈值上限
    pub threshold_high: f32,
    /// 阈值下限
    pub threshold_low: f32,
    /// 统计检测的Z-score阈值
    pub z_score_threshold: f32,
    /// 滑动窗口大小
    pub window_size: usize,
    /// 梯度阈值
    pub gradient_threshold: f32,
}

/// 校准数据
#[derive(Debug, Clone)]
pub struct CalibrationData {
    /// 偏移量
    pub offset: f32,
    /// 缩放因子
    pub scale: f32,
    /// 线性校准系数
    pub linear_coeffs: Vec<f32, 4>,
    /// 温度补偿系数
    pub temp_compensation: f32,
}

/// 处理统计信息
#[derive(Debug, Clone, Default)]
pub struct ProcessingStats {
    /// 处理的数据点数
    pub processed_count: u32,
    /// 过滤的数据点数
    pub filtered_count: u32,
    /// 检测到的异常数
    pub anomaly_count: u32,
    /// 校准应用次数
    pub calibration_applied: u32,
    /// 平均处理时间 (微秒)
    pub avg_processing_time_us: u32,
    /// 数据质量分布
    pub quality_distribution: FnvIndexMap<DataQuality, u32, 4>,
}

/// 处理结果
#[derive(Debug, Clone)]
pub struct ProcessingResult {
    /// 处理后的数据
    pub processed_data: SensorData,
    /// 数据质量
    pub quality: DataQuality,
    /// 是否为异常值
    pub is_anomaly: bool,
    /// 置信度 (0.0-1.0)
    pub confidence: f32,
    /// 处理标志
    pub processing_flags: ProcessingFlags,
}

/// 处理标志
#[derive(Debug, Clone, Default)]
pub struct ProcessingFlags {
    /// 已应用滤波
    pub filtered: bool,
    /// 已应用校准
    pub calibrated: bool,
    /// 已进行异常检测
    pub anomaly_checked: bool,
    /// 已进行质量评估
    pub quality_assessed: bool,
    /// 已进行数据融合
    pub fused: bool,
}

/// 数据处理器
pub struct DataProcessor {
    config: ProcessingConfig,
    stats: ProcessingStats,
    calibration_data: FnvIndexMap<SensorType, CalibrationData, 8>,
    history_buffer: FnvIndexMap<SensorType, Vec<f32, 32>, 8>,
    filter_state: FnvIndexMap<SensorType, FilterState, 8>,
}

/// 滤波器状态
#[derive(Debug, Clone)]
struct FilterState {
    /// 移动平均缓冲区
    ma_buffer: Vec<f32, 32>,
    /// 指数移动平均状态
    ema_value: f32,
    /// 卡尔曼滤波状态
    kalman_x: f32,  // 状态估计
    kalman_p: f32,  // 误差协方差
    /// 低通滤波器状态
    lowpass_prev: f32,
}

impl Default for FilterState {
    fn default() -> Self {
        Self {
            ma_buffer: Vec::new(),
            ema_value: 0.0,
            kalman_x: 0.0,
            kalman_p: 1.0,
            lowpass_prev: 0.0,
        }
    }
}

impl Default for ProcessingConfig {
    fn default() -> Self {
        Self {
            filter_type: FilterType::MovingAverage,
            filter_params: FilterParams {
                ma_window_size: 5,
                ema_alpha: 0.1,
                kalman_q: 0.1,
                kalman_r: 0.1,
                lowpass_cutoff: 0.1,
            },
            anomaly_detection: AnomalyDetectionMethod::Threshold,
            anomaly_params: AnomalyParams {
                threshold_high: 100.0,
                threshold_low: -100.0,
                z_score_threshold: 3.0,
                window_size: 10,
                gradient_threshold: 10.0,
            },
            calibration_enabled: true,
            fusion_enabled: false,
            window_size: 16,
            quality_assessment: true,
        }
    }
}

impl DataProcessor {
    /// 创建新的数据处理器
    pub fn new(config: ProcessingConfig) -> Self {
        Self {
            config,
            stats: ProcessingStats::default(),
            calibration_data: FnvIndexMap::new(),
            history_buffer: FnvIndexMap::new(),
            filter_state: FnvIndexMap::new(),
        }
    }
    
    /// 处理单个数据点
    pub fn process_data(&mut self, data: SensorData) -> Result<ProcessingResult, ProcessorError> {
        let mut result = ProcessingResult {
            processed_data: data.clone(),
            quality: DataQuality::High,
            is_anomaly: false,
            confidence: 1.0,
            processing_flags: ProcessingFlags::default(),
        };
        
        // 1. 数据验证
        if !self.validate_data(&data) {
            result.quality = DataQuality::Invalid;
            return Ok(result);
        }
        
        // 2. 校准
        if self.config.calibration_enabled {
            result.processed_data = self.apply_calibration(result.processed_data)?;
            result.processing_flags.calibrated = true;
            self.stats.calibration_applied += 1;
        }
        
        // 3. 滤波
        if self.config.filter_type != FilterType::None {
            result.processed_data.value = self.apply_filter(
                data.sensor_type, 
                result.processed_data.value
            )?;
            result.processing_flags.filtered = true;
            self.stats.filtered_count += 1;
        }
        
        // 4. 异常检测
        if self.config.anomaly_detection != AnomalyDetectionMethod::None {
            result.is_anomaly = self.detect_anomaly(
                data.sensor_type, 
                result.processed_data.value
            )?;
            result.processing_flags.anomaly_checked = true;
            if result.is_anomaly {
                self.stats.anomaly_count += 1;
            }
        }
        
        // 5. 质量评估
        if self.config.quality_assessment {
            result.quality = self.assess_quality(&result.processed_data, result.is_anomaly);
            result.processing_flags.quality_assessed = true;
        }
        
        // 6. 更新历史缓冲区
        self.update_history(data.sensor_type, result.processed_data.value);
        
        // 7. 更新统计信息
        self.stats.processed_count += 1;
        *self.stats.quality_distribution.entry(result.quality).or_insert(0) += 1;
        
        Ok(result)
    }
    
    /// 批量处理数据
    pub fn process_batch(&mut self, data_batch: &[SensorData]) -> Result<Vec<ProcessingResult, 32>, ProcessorError> {
        let mut results = Vec::new();
        
        for data in data_batch {
            let result = self.process_data(data.clone())?;
            if results.push(result).is_err() {
                return Err(ProcessorError::BufferFull);
            }
        }
        
        Ok(results)
    }
    
    /// 数据验证
    fn validate_data(&self, data: &SensorData) -> bool {
        // 检查数值是否为有限数
        if !data.value.is_finite() {
            return false;
        }
        
        // 检查基本范围 (根据传感器类型)
        match data.sensor_type {
            SensorType::Temperature => data.value >= -273.15 && data.value <= 1000.0,
            SensorType::Humidity => data.value >= 0.0 && data.value <= 100.0,
            SensorType::Pressure => data.value >= 0.0 && data.value <= 2000.0,
            SensorType::Light => data.value >= 0.0,
            _ => true,
        }
    }
    
    /// 应用校准
    fn apply_calibration(&self, mut data: SensorData) -> Result<SensorData, ProcessorError> {
        if let Some(cal_data) = self.calibration_data.get(&data.sensor_type) {
            // 应用线性校准: y = ax + b
            data.value = data.value * cal_data.scale + cal_data.offset;
            
            // 应用多项式校准 (如果有系数)
            if !cal_data.linear_coeffs.is_empty() {
                let mut calibrated_value = 0.0;
                for (i, &coeff) in cal_data.linear_coeffs.iter().enumerate() {
                    calibrated_value += coeff * powf(data.value, i as f32);
                }
                data.value = calibrated_value;
            }
        }
        
        Ok(data)
    }
    
    /// 应用滤波器
    fn apply_filter(&mut self, sensor_type: SensorType, value: f32) -> Result<f32, ProcessorError> {
        let filter_state = self.filter_state.entry(sensor_type).or_insert_with(FilterState::default);
        
        match self.config.filter_type {
            FilterType::None => Ok(value),
            FilterType::MovingAverage => self.apply_moving_average(filter_state, value),
            FilterType::ExponentialMovingAverage => self.apply_ema(filter_state, value),
            FilterType::Kalman => self.apply_kalman_filter(filter_state, value),
            FilterType::Median => self.apply_median_filter(filter_state, value),
            FilterType::LowPass => self.apply_lowpass_filter(filter_state, value),
        }
    }
    
    /// 移动平均滤波
    fn apply_moving_average(&self, state: &mut FilterState, value: f32) -> Result<f32, ProcessorError> {
        if state.ma_buffer.push(value).is_err() {
            state.ma_buffer.remove(0);
            let _ = state.ma_buffer.push(value);
        }
        
        let sum: f32 = state.ma_buffer.iter().sum();
        Ok(sum / state.ma_buffer.len() as f32)
    }
    
    /// 指数移动平均滤波
    fn apply_ema(&self, state: &mut FilterState, value: f32) -> Result<f32, ProcessorError> {
        if state.ema_value == 0.0 {
            state.ema_value = value;
        } else {
            state.ema_value = self.config.filter_params.ema_alpha * value + 
                             (1.0 - self.config.filter_params.ema_alpha) * state.ema_value;
        }
        Ok(state.ema_value)
    }
    
    /// 卡尔曼滤波
    fn apply_kalman_filter(&self, state: &mut FilterState, measurement: f32) -> Result<f32, ProcessorError> {
        // 预测步骤
        let predicted_x = state.kalman_x;
        let predicted_p = state.kalman_p + self.config.filter_params.kalman_q;
        
        // 更新步骤
        let kalman_gain = predicted_p / (predicted_p + self.config.filter_params.kalman_r);
        state.kalman_x = predicted_x + kalman_gain * (measurement - predicted_x);
        state.kalman_p = (1.0 - kalman_gain) * predicted_p;
        
        Ok(state.kalman_x)
    }
    
    /// 中值滤波
    fn apply_median_filter(&self, state: &mut FilterState, value: f32) -> Result<f32, ProcessorError> {
        if state.ma_buffer.push(value).is_err() {
            state.ma_buffer.remove(0);
            let _ = state.ma_buffer.push(value);
        }
        
        let mut sorted_values: Vec<f32, 32> = state.ma_buffer.clone();
        sorted_values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
        
        let len = sorted_values.len();
        if len % 2 == 0 {
            Ok((sorted_values[len / 2 - 1] + sorted_values[len / 2]) / 2.0)
        } else {
            Ok(sorted_values[len / 2])
        }
    }
    
    /// 低通滤波
    fn apply_lowpass_filter(&self, state: &mut FilterState, value: f32) -> Result<f32, ProcessorError> {
        let alpha = self.config.filter_params.lowpass_cutoff;
        state.lowpass_prev = alpha * value + (1.0 - alpha) * state.lowpass_prev;
        Ok(state.lowpass_prev)
    }
    
    /// 异常检测
    fn detect_anomaly(&mut self, sensor_type: SensorType, value: f32) -> Result<bool, ProcessorError> {
        match self.config.anomaly_detection {
            AnomalyDetectionMethod::None => Ok(false),
            AnomalyDetectionMethod::Threshold => self.threshold_anomaly_detection(value),
            AnomalyDetectionMethod::Statistical => self.statistical_anomaly_detection(sensor_type, value),
            AnomalyDetectionMethod::SlidingWindow => self.sliding_window_anomaly_detection(sensor_type, value),
            AnomalyDetectionMethod::Gradient => self.gradient_anomaly_detection(sensor_type, value),
        }
    }
    
    /// 阈值异常检测
    fn threshold_anomaly_detection(&self, value: f32) -> Result<bool, ProcessorError> {
        Ok(value > self.config.anomaly_params.threshold_high || 
           value < self.config.anomaly_params.threshold_low)
    }
    
    /// 统计异常检测 (Z-score)
    fn statistical_anomaly_detection(&self, sensor_type: SensorType, value: f32) -> Result<bool, ProcessorError> {
        if let Some(history) = self.history_buffer.get(&sensor_type) {
            if history.len() < 3 {
                return Ok(false); // 数据不足
            }
            
            let mean = history.iter().sum::<f32>() / history.len() as f32;
            let variance = history.iter()
                .map(|&x| (x - mean) * (x - mean))
                .sum::<f32>() / history.len() as f32;
            let std_dev = sqrtf(variance);
            
            if std_dev > 0.0 {
                let z_score = fabsf(value - mean) / std_dev;
                Ok(z_score > self.config.anomaly_params.z_score_threshold)
            } else {
                Ok(false)
            }
        } else {
            Ok(false)
        }
    }
    
    /// 滑动窗口异常检测
    fn sliding_window_anomaly_detection(&self, sensor_type: SensorType, value: f32) -> Result<bool, ProcessorError> {
        if let Some(history) = self.history_buffer.get(&sensor_type) {
            if history.len() < self.config.anomaly_params.window_size {
                return Ok(false);
            }
            
            let window_size = self.config.anomaly_params.window_size.min(history.len());
            let window_start = history.len() - window_size;
            let window_mean = history[window_start..].iter().sum::<f32>() / window_size as f32;
            
            let deviation = fabsf(value - window_mean);
            Ok(deviation > self.config.anomaly_params.threshold_high)
        } else {
            Ok(false)
        }
    }
    
    /// 梯度异常检测
    fn gradient_anomaly_detection(&self, sensor_type: SensorType, value: f32) -> Result<bool, ProcessorError> {
        if let Some(history) = self.history_buffer.get(&sensor_type) {
            if let Some(&last_value) = history.last() {
                let gradient = fabsf(value - last_value);
                Ok(gradient > self.config.anomaly_params.gradient_threshold)
            } else {
                Ok(false)
            }
        } else {
            Ok(false)
        }
    }
    
    /// 质量评估
    fn assess_quality(&self, data: &SensorData, is_anomaly: bool) -> DataQuality {
        if is_anomaly {
            return DataQuality::Low;
        }
        
        // 基于数据特征评估质量
        match data.sensor_type {
            SensorType::Temperature => {
                if data.value >= -40.0 && data.value <= 85.0 {
                    DataQuality::High
                } else if data.value >= -100.0 && data.value <= 200.0 {
                    DataQuality::Medium
                } else {
                    DataQuality::Low
                }
            },
            SensorType::Humidity => {
                if data.value >= 0.0 && data.value <= 100.0 {
                    DataQuality::High
                } else {
                    DataQuality::Invalid
                }
            },
            _ => DataQuality::High,
        }
    }
    
    /// 更新历史缓冲区
    fn update_history(&mut self, sensor_type: SensorType, value: f32) {
        let history = self.history_buffer.entry(sensor_type).or_insert_with(Vec::new);
        
        if history.push(value).is_err() {
            history.remove(0);
            let _ = history.push(value);
        }
    }
    
    /// 设置校准数据
    pub fn set_calibration_data(&mut self, sensor_type: SensorType, cal_data: CalibrationData) {
        self.calibration_data.insert(sensor_type, cal_data);
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> &ProcessingStats {
        &self.stats
    }
    
    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats = ProcessingStats::default();
    }
    
    /// 更新配置
    pub fn update_config(&mut self, config: ProcessingConfig) {
        self.config = config;
    }
    
    /// 获取配置
    pub fn get_config(&self) -> &ProcessingConfig {
        &self.config
    }
    
    /// 清空历史数据
    pub fn clear_history(&mut self) {
        self.history_buffer.clear();
        self.filter_state.clear();
    }
}

/// 处理器工具模块
pub mod processor_utils {
    use super::*;
    
    /// 创建温度传感器校准数据
    pub fn create_temperature_calibration(offset: f32, scale: f32) -> CalibrationData {
        CalibrationData {
            offset,
            scale,
            linear_coeffs: Vec::new(),
            temp_compensation: 0.0,
        }
    }
    
    /// 创建湿度传感器校准数据
    pub fn create_humidity_calibration(offset: f32, scale: f32) -> CalibrationData {
        CalibrationData {
            offset,
            scale,
            linear_coeffs: Vec::new(),
            temp_compensation: 0.0,
        }
    }
    
    /// 验证处理配置
    pub fn validate_processing_config(config: &ProcessingConfig) -> Result<(), ProcessorError> {
        if config.filter_params.ma_window_size == 0 {
            return Err(ProcessorError::ConfigurationError);
        }
        
        if config.filter_params.ema_alpha <= 0.0 || config.filter_params.ema_alpha >= 1.0 {
            return Err(ProcessorError::ConfigurationError);
        }
        
        if config.anomaly_params.window_size == 0 {
            return Err(ProcessorError::ConfigurationError);
        }
        
        Ok(())
    }
    
    /// 计算处理效率
    pub fn calculate_processing_efficiency(stats: &ProcessingStats) -> f32 {
        if stats.processed_count == 0 {
            0.0
        } else {
            let valid_count = stats.processed_count - stats.anomaly_count;
            valid_count as f32 / stats.processed_count as f32 * 100.0
        }
    }
    
    /// 获取质量分布百分比
    pub fn get_quality_distribution_percentage(stats: &ProcessingStats) -> FnvIndexMap<DataQuality, f32, 4> {
        let mut percentages = FnvIndexMap::new();
        let total = stats.processed_count as f32;
        
        if total > 0.0 {
            for (&quality, &count) in &stats.quality_distribution {
                percentages.insert(quality, count as f32 / total * 100.0);
            }
        }
        
        percentages
    }
}
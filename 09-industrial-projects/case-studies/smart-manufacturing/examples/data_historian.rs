//! 数据历史记录系统示例
//! 
//! 这个示例展示了工业数据历史记录系统的基本功能：
//! - 实时数据采集
//! - 数据存储和检索
//! - 数据压缩和归档
//! - 趋势分析

use std::collections::{HashMap, VecDeque};
use std::time::{SystemTime, UNIX_EPOCH};

/// 数据点结构
#[derive(Debug, Clone)]
struct DataPoint {
    timestamp: u64,
    tag_name: String,
    value: f64,
    quality: DataQuality,
}

/// 数据质量枚举
#[derive(Debug, Clone, PartialEq)]
enum DataQuality {
    Good,
    Bad,
    Uncertain,
}

/// 数据标签配置
#[derive(Debug, Clone)]
struct TagConfig {
    name: String,
    description: String,
    unit: String,
    min_value: f64,
    max_value: f64,
    deadband: f64, // 死区值，用于数据压缩
}

/// 历史数据存储
struct DataHistorian {
    tags: HashMap<String, TagConfig>,
    raw_data: HashMap<String, VecDeque<DataPoint>>,
    compressed_data: HashMap<String, VecDeque<DataPoint>>,
    max_raw_points: usize,
    max_compressed_points: usize,
}

impl DataHistorian {
    fn new(max_raw: usize, max_compressed: usize) -> Self {
        Self {
            tags: HashMap::new(),
            raw_data: HashMap::new(),
            compressed_data: HashMap::new(),
            max_raw_points: max_raw,
            max_compressed_points: max_compressed,
        }
    }
    
    /// 添加标签配置
    fn add_tag(&mut self, config: TagConfig) {
        let tag_name = config.name.clone();
        self.tags.insert(tag_name.clone(), config);
        self.raw_data.insert(tag_name.clone(), VecDeque::new());
        self.compressed_data.insert(tag_name, VecDeque::new());
    }
    
    /// 添加数据点
    fn add_data_point(&mut self, mut point: DataPoint) {
        // 验证数据质量
        if let Some(config) = self.tags.get(&point.tag_name) {
            if point.value < config.min_value || point.value > config.max_value {
                point.quality = DataQuality::Bad;
            }
        }
        
        // 存储到原始数据
        if let Some(raw_buffer) = self.raw_data.get_mut(&point.tag_name) {
            raw_buffer.push_back(point.clone());
            
            // 限制原始数据缓冲区大小
            if raw_buffer.len() > self.max_raw_points {
                if let Some(old_point) = raw_buffer.pop_front() {
                    self.compress_data_point(old_point);
                }
            }
        }
    }
    
    /// 数据压缩（死区压缩算法）
    fn compress_data_point(&mut self, point: DataPoint) {
        if let Some(compressed_buffer) = self.compressed_data.get_mut(&point.tag_name) {
            let should_store = if let Some(last_point) = compressed_buffer.back() {
                if let Some(config) = self.tags.get(&point.tag_name) {
                    (point.value - last_point.value).abs() > config.deadband
                } else {
                    true
                }
            } else {
                true
            };
            
            if should_store {
                compressed_buffer.push_back(point);
                
                // 限制压缩数据缓冲区大小
                if compressed_buffer.len() > self.max_compressed_points {
                    compressed_buffer.pop_front();
                }
            }
        }
    }
    
    /// 获取历史数据
    fn get_history(&self, tag_name: &str, start_time: u64, end_time: u64) -> Vec<DataPoint> {
        let mut result = Vec::new();
        
        // 从原始数据中获取
        if let Some(raw_buffer) = self.raw_data.get(tag_name) {
            for point in raw_buffer {
                if point.timestamp >= start_time && point.timestamp <= end_time {
                    result.push(point.clone());
                }
            }
        }
        
        // 从压缩数据中获取
        if let Some(compressed_buffer) = self.compressed_data.get(tag_name) {
            for point in compressed_buffer {
                if point.timestamp >= start_time && point.timestamp <= end_time {
                    result.push(point.clone());
                }
            }
        }
        
        // 按时间戳排序
        result.sort_by_key(|p| p.timestamp);
        result
    }
    
    /// 计算统计信息
    fn calculate_statistics(&self, tag_name: &str, start_time: u64, end_time: u64) -> Option<Statistics> {
        let history = self.get_history(tag_name, start_time, end_time);
        
        if history.is_empty() {
            return None;
        }
        
        let values: Vec<f64> = history.iter()
            .filter(|p| p.quality == DataQuality::Good)
            .map(|p| p.value)
            .collect();
        
        if values.is_empty() {
            return None;
        }
        
        let sum: f64 = values.iter().sum();
        let count = values.len() as f64;
        let average = sum / count;
        
        let min = values.iter().fold(f64::INFINITY, |a, &b| a.min(b));
        let max = values.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b));
        
        Some(Statistics {
            count: count as usize,
            average,
            min,
            max,
            sum,
        })
    }
    
    /// 获取系统状态
    fn get_status(&self) -> SystemStatus {
        let mut total_raw_points = 0;
        let mut total_compressed_points = 0;
        
        for buffer in self.raw_data.values() {
            total_raw_points += buffer.len();
        }
        
        for buffer in self.compressed_data.values() {
            total_compressed_points += buffer.len();
        }
        
        SystemStatus {
            tag_count: self.tags.len(),
            total_raw_points,
            total_compressed_points,
            memory_usage_mb: (total_raw_points + total_compressed_points) * 64 / 1024 / 1024, // 估算
        }
    }
}

/// 统计信息结构
#[derive(Debug)]
struct Statistics {
    count: usize,
    average: f64,
    min: f64,
    max: f64,
    sum: f64,
}

/// 系统状态结构
#[derive(Debug)]
struct SystemStatus {
    tag_count: usize,
    total_raw_points: usize,
    total_compressed_points: usize,
    memory_usage_mb: usize,
}

fn get_current_timestamp() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs()
}

fn main() {
    println!("数据历史记录系统示例");
    
    let mut historian = DataHistorian::new(1000, 10000);
    
    // 添加标签配置
    historian.add_tag(TagConfig {
        name: "Temperature_01".to_string(),
        description: "反应器温度".to_string(),
        unit: "°C".to_string(),
        min_value: -50.0,
        max_value: 200.0,
        deadband: 0.5,
    });
    
    historian.add_tag(TagConfig {
        name: "Pressure_01".to_string(),
        description: "系统压力".to_string(),
        unit: "bar".to_string(),
        min_value: 0.0,
        max_value: 10.0,
        deadband: 0.1,
    });
    
    let start_time = get_current_timestamp();
    
    // 模拟数据采集
    println!("\n开始数据采集...");
    for i in 0..100 {
        let timestamp = start_time + i;
        
        // 模拟温度数据（带噪声）
        let temp_value = 25.0 + 10.0 * (i as f64 * 0.1).sin() + (i % 7) as f64 * 0.2;
        historian.add_data_point(DataPoint {
            timestamp,
            tag_name: "Temperature_01".to_string(),
            value: temp_value,
            quality: DataQuality::Good,
        });
        
        // 模拟压力数据
        let pressure_value = 2.0 + 1.0 * (i as f64 * 0.05).cos();
        historian.add_data_point(DataPoint {
            timestamp,
            tag_name: "Pressure_01".to_string(),
            value: pressure_value,
            quality: DataQuality::Good,
        });
        
        if i % 20 == 0 {
            println!("已采集 {} 个数据点", i * 2);
        }
    }
    
    // 显示系统状态
    let status = historian.get_status();
    println!("\n=== 系统状态 ===");
    println!("{:#?}", status);
    
    // 获取历史数据
    let end_time = get_current_timestamp();
    let temp_history = historian.get_history("Temperature_01", start_time, end_time);
    println!("\n温度历史数据点数: {}", temp_history.len());
    
    // 计算统计信息
    if let Some(temp_stats) = historian.calculate_statistics("Temperature_01", start_time, end_time) {
        println!("\n=== 温度统计信息 ===");
        println!("{:#?}", temp_stats);
    }
    
    if let Some(pressure_stats) = historian.calculate_statistics("Pressure_01", start_time, end_time) {
        println!("\n=== 压力统计信息 ===");
        println!("{:#?}", pressure_stats);
    }
    
    println!("\n数据历史记录系统示例完成");
}
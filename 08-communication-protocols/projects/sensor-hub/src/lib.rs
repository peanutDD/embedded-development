//! 传感器集线器库
//!
//! 这个库提供了一个完整的传感器数据收集、处理和存储系统，
//! 支持多种传感器类型和通信协议。
//!
//! # 主要功能
//!
//! - **传感器管理**: 支持温度、湿度、压力和光照传感器
//! - **通信协议**: 支持I2C和SPI通信协议
//! - **数据处理**: 提供数据收集、处理和存储功能
//! - **错误处理**: 完整的错误处理和恢复机制
//!
//! # 使用示例
//!
//! ```rust
//! use sensor_hub::{
//!     sensors::{SensorManager, SensorConfig},
//!     data::{DataPipeline, DataPipelineConfig},
//!     communication::{I2CManager, SPIManager},
//! };
//!
//! // 创建传感器管理器
//! let mut sensor_manager = SensorManager::new();
//!
//! // 创建数据处理管道
//! let mut data_pipeline = DataPipeline::new(
//!     Default::default(),
//!     Default::default(),
//!     Default::default(),
//! );
//!
//! // 读取传感器数据并处理
//! if let Ok(data) = sensor_manager.read_all_sensors() {
//!     for sensor_data in data {
//!         let _ = data_pipeline.process_sensor_data(sensor_data);
//!     }
//! }
//! ```

#![no_std]
#![deny(missing_docs)]
#![deny(warnings)]

extern crate alloc;

// 重新导出核心类型
pub use heapless;
pub use nb;

/// 传感器模块
/// 
/// 提供各种传感器的驱动和管理功能
pub mod sensors;

/// 通信模块
/// 
/// 提供I2C和SPI通信协议的管理功能
pub mod communication;

/// 数据处理模块
/// 
/// 提供数据收集、处理和存储功能
pub mod data;

// 重新导出常用类型
pub use sensors::{
    SensorManager, SensorConfig, SensorData, SensorType, SensorError,
    TemperatureSensor, HumiditySensor, PressureSensor, LightSensor,
};

pub use communication::{
    I2CManager, SPIManager, CommunicationError,
    I2CConfig, SPIConfig, DeviceAddress,
};

pub use data::{
    DataPipeline, DataPipelineConfig, DataPipelineStats,
    DataCollector, DataProcessor, DataStorage,
    CollectionConfig, ProcessingConfig, StorageConfig,
    SensorData as DataSensorData, DataError,
};

/// 传感器集线器的主要错误类型
#[derive(Debug, Clone)]
pub enum SensorHubError {
    /// 传感器错误
    Sensor(SensorError),
    /// 通信错误
    Communication(CommunicationError),
    /// 数据处理错误
    Data(DataError),
    /// 配置错误
    Configuration(&'static str),
    /// 系统错误
    System(&'static str),
}

impl From<SensorError> for SensorHubError {
    fn from(error: SensorError) -> Self {
        SensorHubError::Sensor(error)
    }
}

impl From<CommunicationError> for SensorHubError {
    fn from(error: CommunicationError) -> Self {
        SensorHubError::Communication(error)
    }
}

impl From<DataError> for SensorHubError {
    fn from(error: DataError) -> Self {
        SensorHubError::Data(error)
    }
}

/// 传感器集线器配置
#[derive(Debug, Clone)]
pub struct SensorHubConfig {
    /// 传感器配置
    pub sensors: SensorConfig,
    /// I2C配置
    pub i2c: I2CConfig,
    /// SPI配置
    pub spi: SPIConfig,
    /// 数据处理管道配置
    pub data_pipeline: DataPipelineConfig,
    /// 采样间隔（毫秒）
    pub sampling_interval_ms: u32,
    /// 是否启用自动校准
    pub auto_calibration: bool,
}

impl Default for SensorHubConfig {
    fn default() -> Self {
        Self {
            sensors: SensorConfig::default(),
            i2c: I2CConfig::default(),
            spi: SPIConfig::default(),
            data_pipeline: DataPipelineConfig::default(),
            sampling_interval_ms: 1000, // 1秒
            auto_calibration: true,
        }
    }
}

/// 传感器集线器统计信息
#[derive(Debug, Clone, Default)]
pub struct SensorHubStats {
    /// 数据处理管道统计
    pub data_pipeline: DataPipelineStats,
    /// 总运行时间（毫秒）
    pub uptime_ms: u64,
    /// 总采样次数
    pub total_samples: u64,
    /// 成功采样次数
    pub successful_samples: u64,
    /// 失败采样次数
    pub failed_samples: u64,
    /// 平均采样间隔（毫秒）
    pub avg_sampling_interval_ms: f32,
}

/// 传感器集线器主结构
/// 
/// 集成所有传感器、通信和数据处理功能的主要接口
pub struct SensorHub {
    sensor_manager: SensorManager,
    data_pipeline: DataPipeline,
    config: SensorHubConfig,
    stats: SensorHubStats,
    last_sample_time: u64,
}

impl SensorHub {
    /// 创建新的传感器集线器
    pub fn new(config: SensorHubConfig) -> Self {
        let sensor_manager = SensorManager::new();
        let data_pipeline = DataPipeline::new(
            config.data_pipeline.collector.clone(),
            config.data_pipeline.processor.clone(),
            config.data_pipeline.storage.clone(),
        );
        
        Self {
            sensor_manager,
            data_pipeline,
            config,
            stats: SensorHubStats::default(),
            last_sample_time: 0,
        }
    }
    
    /// 初始化传感器集线器
    pub fn initialize(&mut self) -> Result<(), SensorHubError> {
        // 初始化传感器管理器
        self.sensor_manager.initialize(self.config.sensors.clone())?;
        
        // 如果启用自动校准，执行校准
        if self.config.auto_calibration {
            self.sensor_manager.calibrate_all_sensors()?;
        }
        
        Ok(())
    }
    
    /// 执行单次采样
    pub fn sample_once(&mut self) -> Result<u32, SensorHubError> {
        self.stats.total_samples += 1;
        
        // 读取所有传感器数据
        match self.sensor_manager.read_all_sensors() {
            Ok(sensor_data) => {
                let mut processed_count = 0;
                
                // 处理每个传感器的数据
                for data in sensor_data {
                    match self.data_pipeline.process_sensor_data(data) {
                        Ok(_) => processed_count += 1,
                        Err(e) => {
                            log::warn!("Failed to process sensor data: {:?}", e);
                        }
                    }
                }
                
                self.stats.successful_samples += 1;
                Ok(processed_count)
            },
            Err(e) => {
                self.stats.failed_samples += 1;
                Err(SensorHubError::Sensor(e))
            }
        }
    }
    
    /// 获取统计信息
    pub fn get_stats(&mut self) -> SensorHubStats {
        self.stats.data_pipeline = self.data_pipeline.get_pipeline_stats();
        self.stats.clone()
    }
    
    /// 更新配置
    pub fn update_config(&mut self, config: SensorHubConfig) -> Result<(), SensorHubError> {
        // 更新传感器配置
        self.sensor_manager.update_config(config.sensors.clone())?;
        
        // 更新数据处理管道配置
        self.data_pipeline.update_config(config.data_pipeline.clone())?;
        
        self.config = config;
        Ok(())
    }
    
    /// 执行维护操作
    pub fn maintenance(&mut self) -> Result<(), SensorHubError> {
        // 清理数据处理管道
        self.data_pipeline.cleanup()?;
        
        // 如果启用自动校准，定期校准传感器
        if self.config.auto_calibration {
            self.sensor_manager.calibrate_all_sensors()?;
        }
        
        Ok(())
    }
    
    /// 查询历史数据
    pub fn query_data(&mut self, condition: data::QueryCondition) -> Result<heapless::Vec<data::DataRecord, 64>, SensorHubError> {
        self.data_pipeline.query_data(condition)
            .map_err(SensorHubError::Data)
    }
    
    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.data_pipeline.reset_stats();
        self.stats = SensorHubStats::default();
    }
    
    /// 获取配置
    pub fn get_config(&self) -> &SensorHubConfig {
        &self.config
    }
}
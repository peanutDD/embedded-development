//! 显示设备模块
//!
//! 本模块提供对各种显示设备的统一抽象和管理功能，包括：
//! - OLED显示屏（基于SSD1306控制器）
//! - LCD显示屏（基于ST7735控制器）
//! - 电子纸显示（基于Waveshare EPD系列）
//!
//! # 特性
//! - 统一的显示设备接口
//! - 多显示设备管理
//! - 自动设备发现和初始化
//! - 显示设备健康监控
//! - 电源管理
//! - 错误恢复机制

use embedded_graphics::prelude::*;
use embedded_hal::delay::DelayNs;
use heapless::{FnvIndexMap, Vec};

pub mod oled;
pub mod lcd;
pub mod epaper;

/// 显示设备类型
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum DisplayType {
    /// OLED显示屏
    Oled,
    /// LCD显示屏
    Lcd,
    /// 电子纸显示
    EPaper,
}

/// 显示设备接口
pub trait Display {
    type Color: PixelColor;
    type Error;
    
    /// 初始化显示设备
    fn init(&mut self) -> Result<(), Self::Error>;
    
    /// 清屏
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error>;
    
    /// 刷新显示缓冲区到屏幕
    fn flush(&mut self) -> Result<(), Self::Error>;
    
    /// 设置像素
    fn set_pixel(&mut self, point: Point, color: Self::Color) -> Result<(), Self::Error>;
    
    /// 获取显示尺寸
    fn size(&self) -> Size;
    
    /// 设置亮度（0-255）
    fn set_brightness(&mut self, brightness: u8) -> Result<(), Self::Error>;
    
    /// 获取当前亮度
    fn get_brightness(&self) -> u8;
    
    /// 设置显示开关
    fn set_display_on(&mut self, on: bool) -> Result<(), Self::Error>;
    
    /// 检查显示设备是否正常工作
    fn is_healthy(&mut self) -> bool;
    
    /// 进入睡眠模式
    fn sleep(&mut self) -> Result<(), Self::Error>;
    
    /// 从睡眠模式唤醒
    fn wake_up(&mut self) -> Result<(), Self::Error>;
    
    /// 获取设备信息
    fn get_info(&self) -> DisplayInfo;
}

/// 显示设备信息
#[derive(Debug, Clone)]
pub struct DisplayInfo {
    /// 设备类型
    pub display_type: DisplayType,
    /// 显示尺寸
    pub size: Size,
    /// 颜色深度（位数）
    pub color_depth: u8,
    /// 支持的最大亮度
    pub max_brightness: u8,
    /// 刷新率（Hz）
    pub refresh_rate: u16,
    /// 设备名称
    pub name: &'static str,
    /// 制造商
    pub manufacturer: &'static str,
    /// 接口类型
    pub interface: InterfaceType,
}

/// 接口类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfaceType {
    /// I2C接口
    I2C,
    /// SPI接口
    SPI,
    /// 并行接口
    Parallel,
}

/// 显示设备配置
#[derive(Debug, Clone)]
pub struct DisplayConfig {
    /// 默认亮度
    pub default_brightness: u8,
    /// 自动睡眠时间（秒，0表示禁用）
    pub auto_sleep_timeout: u32,
    /// 启用健康检查
    pub enable_health_check: bool,
    /// 健康检查间隔（毫秒）
    pub health_check_interval: u32,
    /// 最大重试次数
    pub max_retry_count: u8,
}

impl Default for DisplayConfig {
    fn default() -> Self {
        Self {
            default_brightness: 128,
            auto_sleep_timeout: 300, // 5分钟
            enable_health_check: true,
            health_check_interval: 5000, // 5秒
            max_retry_count: 3,
        }
    }
}

/// 显示设备统计信息
#[derive(Debug, Default, Clone)]
pub struct DisplayStats {
    /// 初始化次数
    pub init_count: u32,
    /// 刷新次数
    pub flush_count: u32,
    /// 错误次数
    pub error_count: u32,
    /// 重试次数
    pub retry_count: u32,
    /// 睡眠次数
    pub sleep_count: u32,
    /// 唤醒次数
    pub wake_count: u32,
    /// 运行时间（毫秒）
    pub uptime_ms: u32,
    /// 最后一次健康检查时间
    pub last_health_check: u32,
}

impl DisplayStats {
    /// 重置统计信息
    pub fn reset(&mut self) {
        *self = Self::default();
    }
    
    /// 计算错误率
    pub fn error_rate(&self) -> f32 {
        if self.flush_count == 0 {
            0.0
        } else {
            self.error_count as f32 / self.flush_count as f32
        }
    }
    
    /// 计算平均刷新间隔（毫秒）
    pub fn average_flush_interval(&self) -> f32 {
        if self.flush_count == 0 {
            0.0
        } else {
            self.uptime_ms as f32 / self.flush_count as f32
        }
    }
}

/// 显示管理器
pub struct DisplayManager {
    /// 注册的显示设备
    displays: FnvIndexMap<DisplayType, Box<dyn Display<Color = BinaryColor, Error = DisplayError>>, 4>,
    /// 显示设备配置
    configs: FnvIndexMap<DisplayType, DisplayConfig, 4>,
    /// 显示设备统计
    stats: FnvIndexMap<DisplayType, DisplayStats, 4>,
    /// 当前活动的显示设备
    active_display: Option<DisplayType>,
    /// 最后一次健康检查时间
    last_health_check: u32,
}

impl DisplayManager {
    /// 创建新的显示管理器
    pub fn new() -> Self {
        Self {
            displays: FnvIndexMap::new(),
            configs: FnvIndexMap::new(),
            stats: FnvIndexMap::new(),
            active_display: None,
            last_health_check: 0,
        }
    }
    
    /// 注册显示设备
    pub fn register_display(
        &mut self,
        display_type: DisplayType,
        display: Box<dyn Display<Color = BinaryColor, Error = DisplayError>>,
    ) -> Result<(), DisplayError> {
        // 初始化显示设备
        let mut display = display;
        display.init().map_err(DisplayError::InitializationFailed)?;
        
        // 注册设备
        self.displays.insert(display_type, display)
            .map_err(|_| DisplayError::RegistrationFailed)?;
        
        // 设置默认配置
        self.configs.insert(display_type, DisplayConfig::default())
            .map_err(|_| DisplayError::ConfigurationFailed)?;
        
        // 初始化统计信息
        let mut stats = DisplayStats::default();
        stats.init_count = 1;
        self.stats.insert(display_type, stats)
            .map_err(|_| DisplayError::StatsFailed)?;
        
        // 如果是第一个设备，设为活动设备
        if self.active_display.is_none() {
            self.active_display = Some(display_type);
        }
        
        Ok(())
    }
    
    /// 注销显示设备
    pub fn unregister_display(&mut self, display_type: DisplayType) -> Result<(), DisplayError> {
        self.displays.remove(&display_type);
        self.configs.remove(&display_type);
        self.stats.remove(&display_type);
        
        // 如果注销的是活动设备，选择新的活动设备
        if self.active_display == Some(display_type) {
            self.active_display = self.displays.keys().next().copied();
        }
        
        Ok(())
    }
    
    /// 获取显示设备（可变引用）
    pub fn get_display_mut(
        &mut self,
        display_type: DisplayType,
    ) -> Result<&mut dyn Display<Color = BinaryColor, Error = DisplayError>, DisplayError> {
        self.displays.get_mut(&display_type)
            .map(|d| d.as_mut())
            .ok_or(DisplayError::DeviceNotFound)
    }
    
    /// 获取显示设备（不可变引用）
    pub fn get_display(
        &self,
        display_type: DisplayType,
    ) -> Result<&dyn Display<Color = BinaryColor, Error = DisplayError>, DisplayError> {
        self.displays.get(&display_type)
            .map(|d| d.as_ref())
            .ok_or(DisplayError::DeviceNotFound)
    }
    
    /// 设置活动显示设备
    pub fn set_active_display(&mut self, display_type: DisplayType) -> Result<(), DisplayError> {
        if self.displays.contains_key(&display_type) {
            self.active_display = Some(display_type);
            Ok(())
        } else {
            Err(DisplayError::DeviceNotFound)
        }
    }
    
    /// 获取活动显示设备
    pub fn get_active_display(&self) -> Option<DisplayType> {
        self.active_display
    }
    
    /// 检查显示设备是否可用
    pub fn is_available(&self, display_type: DisplayType) -> bool {
        self.displays.contains_key(&display_type)
    }
    
    /// 获取所有可用的显示设备
    pub fn get_available_displays(&self) -> Vec<DisplayType, 4> {
        self.displays.keys().copied().collect()
    }
    
    /// 更新显示管理器
    pub fn update(&mut self) -> Result<(), DisplayError> {
        let current_time = self.get_current_time();
        
        // 更新统计信息
        for (_, stats) in self.stats.iter_mut() {
            stats.uptime_ms = current_time;
        }
        
        // 执行健康检查
        if current_time - self.last_health_check > 5000 { // 5秒间隔
            self.perform_health_check()?;
            self.last_health_check = current_time;
        }
        
        // 检查自动睡眠
        self.check_auto_sleep(current_time)?;
        
        Ok(())
    }
    
    /// 执行健康检查
    fn perform_health_check(&mut self) -> Result<(), DisplayError> {
        let display_types: Vec<DisplayType, 4> = self.displays.keys().copied().collect();
        
        for display_type in display_types {
            if let Some(config) = self.configs.get(&display_type) {
                if config.enable_health_check {
                    if let Ok(display) = self.get_display_mut(display_type) {
                        let is_healthy = display.is_healthy();
                        
                        if let Some(stats) = self.stats.get_mut(&display_type) {
                            stats.last_health_check = self.get_current_time();
                            
                            if !is_healthy {
                                stats.error_count += 1;
                                // 尝试恢复
                                self.recover_display(display_type)?;
                            }
                        }
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 恢复显示设备
    pub fn recover_display(&mut self, display_type: DisplayType) -> Result<(), DisplayError> {
        if let Ok(display) = self.get_display_mut(display_type) {
            // 尝试重新初始化
            display.init().map_err(DisplayError::RecoveryFailed)?;
            
            if let Some(stats) = self.stats.get_mut(&display_type) {
                stats.retry_count += 1;
                stats.init_count += 1;
            }
        }
        
        Ok(())
    }
    
    /// 检查自动睡眠
    fn check_auto_sleep(&mut self, current_time: u32) -> Result<(), DisplayError> {
        // 这里应该实现自动睡眠逻辑
        // 基于最后活动时间和配置的超时时间
        Ok(())
    }
    
    /// 获取健康状态
    pub fn get_health_status(&self) -> Vec<(DisplayType, bool), 4> {
        let mut health_status = Vec::new();
        
        for (&display_type, display) in &self.displays {
            // 注意：这里需要可变引用来调用is_healthy，但我们只有不可变引用
            // 在实际实现中，可能需要调整接口设计
            health_status.push((display_type, true)).ok(); // 临时返回true
        }
        
        health_status
    }
    
    /// 重新初始化通信
    pub fn reinitialize_communication(&mut self) -> Result<(), DisplayError> {
        // 重新初始化所有显示设备的通信
        let display_types: Vec<DisplayType, 4> = self.displays.keys().copied().collect();
        
        for display_type in display_types {
            self.recover_display(display_type)?;
        }
        
        Ok(())
    }
    
    /// 重新初始化显示设备
    pub fn reinitialize_displays(&mut self) -> Result<(), DisplayError> {
        self.reinitialize_communication()
    }
    
    /// 获取当前时间（毫秒）
    fn get_current_time(&self) -> u32 {
        // 这里应该返回系统时间，具体实现取决于平台
        // 临时返回0
        0
    }
    
    /// 获取显示设备统计信息
    pub fn get_stats(&self, display_type: DisplayType) -> Option<&DisplayStats> {
        self.stats.get(&display_type)
    }
    
    /// 获取显示设备配置
    pub fn get_config(&self, display_type: DisplayType) -> Option<&DisplayConfig> {
        self.configs.get(&display_type)
    }
    
    /// 更新显示设备配置
    pub fn update_config(
        &mut self,
        display_type: DisplayType,
        config: DisplayConfig,
    ) -> Result<(), DisplayError> {
        self.configs.insert(display_type, config)
            .map_err(|_| DisplayError::ConfigurationFailed)?;
        Ok(())
    }
}

/// 显示错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisplayError {
    /// 通信错误
    Communication,
    /// 初始化失败
    InitializationFailed,
    /// 设备未找到
    DeviceNotFound,
    /// 注册失败
    RegistrationFailed,
    /// 配置失败
    ConfigurationFailed,
    /// 统计失败
    StatsFailed,
    /// 恢复失败
    RecoveryFailed,
    /// 显示错误
    Display,
    /// 无效参数
    InvalidParameter,
    /// 超时
    Timeout,
    /// 内存不足
    OutOfMemory,
}

/// 显示工具函数
pub mod display_utils {
    use super::*;
    
    /// 创建默认OLED配置
    pub fn create_oled_config() -> DisplayConfig {
        DisplayConfig {
            default_brightness: 200,
            auto_sleep_timeout: 600, // 10分钟
            enable_health_check: true,
            health_check_interval: 3000, // 3秒
            max_retry_count: 5,
        }
    }
    
    /// 创建默认LCD配置
    pub fn create_lcd_config() -> DisplayConfig {
        DisplayConfig {
            default_brightness: 255,
            auto_sleep_timeout: 300, // 5分钟
            enable_health_check: true,
            health_check_interval: 5000, // 5秒
            max_retry_count: 3,
        }
    }
    
    /// 创建默认电子纸配置
    pub fn create_epaper_config() -> DisplayConfig {
        DisplayConfig {
            default_brightness: 255, // 电子纸通常不支持亮度调节
            auto_sleep_timeout: 0, // 禁用自动睡眠
            enable_health_check: false, // 电子纸检查较慢
            health_check_interval: 30000, // 30秒
            max_retry_count: 2,
        }
    }
    
    /// 验证显示配置
    pub fn validate_config(config: &DisplayConfig) -> Result<(), DisplayError> {
        if config.default_brightness > 255 {
            return Err(DisplayError::InvalidParameter);
        }
        
        if config.health_check_interval < 1000 {
            return Err(DisplayError::InvalidParameter);
        }
        
        if config.max_retry_count > 10 {
            return Err(DisplayError::InvalidParameter);
        }
        
        Ok(())
    }
    
    /// 计算显示设备效率
    pub fn calculate_efficiency(stats: &DisplayStats) -> f32 {
        if stats.flush_count == 0 {
            0.0
        } else {
            let success_rate = 1.0 - stats.error_rate();
            let retry_penalty = stats.retry_count as f32 / stats.flush_count as f32;
            (success_rate - retry_penalty * 0.1).max(0.0)
        }
    }
    
    /// 获取推荐的健康检查间隔
    pub fn get_recommended_health_check_interval(display_type: DisplayType) -> u32 {
        match display_type {
            DisplayType::Oled => 3000,   // 3秒
            DisplayType::Lcd => 5000,    // 5秒
            DisplayType::EPaper => 30000, // 30秒
        }
    }
}
//! Display Controller Library
//! 
//! 一个用于嵌入式系统的显示控制器库，支持多种显示设备类型和图形操作。
//! 
//! # 特性
//! 
//! - 支持多种显示设备：OLED、LCD、电子纸
//! - 图形绘制功能：基本图形、文本渲染、图像显示
//! - 字体管理：位图字体和矢量字体支持
//! - 高效的内存管理和性能优化
//! - 模块化设计，易于扩展
//! 
//! # 快速开始
//! 
//! ```rust,no_run
//! use display_controller::{
//!     DisplayManager, GraphicsRenderer, FontManager,
//!     displays::{oled::OledI2C, Display},
//!     graphics::{Color, DrawParams},
//! };
//! 
//! // 创建显示管理器
//! let mut display_manager = DisplayManager::new();
//! 
//! // 创建OLED显示器
//! let oled_config = oled::OledConfig::ssd1306_128x64_i2c(0x3C);
//! let mut oled = OledI2C::new(i2c, oled_config);
//! 
//! // 初始化显示器
//! oled.init(&mut delay).unwrap();
//! 
//! // 注册显示器
//! display_manager.register_display("main", Box::new(oled)).unwrap();
//! 
//! // 创建渲染器
//! let mut renderer = GraphicsRenderer::new();
//! 
//! // 绘制矩形
//! let draw_params = DrawParams::default();
//! renderer.draw_rectangle(10, 10, 50, 30, &draw_params).unwrap();
//! ```

#![no_std]
#![deny(unsafe_code)]
#![warn(
    missing_docs,
    rust_2018_idioms,
    missing_debug_implementations,
)]

// 重新导出核心依赖
pub use embedded_hal as hal;
pub use embedded_graphics;
pub use heapless;
pub use nb;

// 公开模块
pub mod displays;
pub mod graphics;

// 重新导出常用类型
pub use displays::{
    Display, DisplayManager, DisplayType, DisplayInfo, DisplayConfig, DisplayError,
};

pub use graphics::{
    Color, ColorFormat, GraphicsError, DrawParams, TextParams, 
    renderer::{GraphicsRenderer, RenderTarget, Transform2D},
    fonts::{FontManager, TextRenderer, FontInfo, FontError},
};

/// 库错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DisplayControllerError {
    /// 显示错误
    Display(DisplayError),
    /// 图形错误
    Graphics(GraphicsError),
    /// 字体错误
    Font(FontError),
    /// 配置错误
    Configuration,
    /// 初始化错误
    Initialization,
    /// 通信错误
    Communication,
    /// 内存不足
    OutOfMemory,
    /// 不支持的操作
    Unsupported,
}

impl From<DisplayError> for DisplayControllerError {
    fn from(err: DisplayError) -> Self {
        DisplayControllerError::Display(err)
    }
}

impl From<GraphicsError> for DisplayControllerError {
    fn from(err: GraphicsError) -> Self {
        DisplayControllerError::Graphics(err)
    }
}

impl From<FontError> for DisplayControllerError {
    fn from(err: FontError) -> Self {
        DisplayControllerError::Font(err)
    }
}

impl core::fmt::Display for DisplayControllerError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            DisplayControllerError::Display(e) => write!(f, "Display error: {}", e),
            DisplayControllerError::Graphics(e) => write!(f, "Graphics error: {}", e),
            DisplayControllerError::Font(e) => write!(f, "Font error: {}", e),
            DisplayControllerError::Configuration => write!(f, "Configuration error"),
            DisplayControllerError::Initialization => write!(f, "Initialization error"),
            DisplayControllerError::Communication => write!(f, "Communication error"),
            DisplayControllerError::OutOfMemory => write!(f, "Out of memory"),
            DisplayControllerError::Unsupported => write!(f, "Unsupported operation"),
        }
    }
}

/// 显示控制器配置
#[derive(Debug, Clone)]
pub struct DisplayControllerConfig {
    /// 默认显示设备
    pub default_display: Option<heapless::String<32>>,
    /// 默认字体
    pub default_font: Option<heapless::String<32>>,
    /// 启用调试模式
    pub debug_mode: bool,
    /// 性能监控
    pub performance_monitoring: bool,
    /// 自动刷新间隔 (毫秒)
    pub auto_refresh_interval: Option<u32>,
    /// 最大显示设备数量
    pub max_displays: usize,
    /// 最大字体数量
    pub max_fonts: usize,
}

impl Default for DisplayControllerConfig {
    fn default() -> Self {
        Self {
            default_display: None,
            default_font: None,
            debug_mode: false,
            performance_monitoring: false,
            auto_refresh_interval: None,
            max_displays: 4,
            max_fonts: 8,
        }
    }
}

/// 显示控制器统计信息
#[derive(Debug, Clone, Copy, Default)]
pub struct DisplayControllerStats {
    /// 总绘制操作数
    pub total_draw_operations: u32,
    /// 总刷新次数
    pub total_refreshes: u32,
    /// 错误计数
    pub error_count: u32,
    /// 平均帧率 (FPS)
    pub average_fps: f32,
    /// 内存使用量 (字节)
    pub memory_usage: usize,
    /// 运行时间 (毫秒)
    pub uptime_ms: u32,
}

impl DisplayControllerStats {
    /// 创建新的统计信息
    pub fn new() -> Self {
        Self::default()
    }

    /// 增加绘制操作计数
    pub fn increment_draw_operations(&mut self) {
        self.total_draw_operations = self.total_draw_operations.saturating_add(1);
    }

    /// 增加刷新计数
    pub fn increment_refreshes(&mut self) {
        self.total_refreshes = self.total_refreshes.saturating_add(1);
    }

    /// 增加错误计数
    pub fn increment_errors(&mut self) {
        self.error_count = self.error_count.saturating_add(1);
    }

    /// 更新帧率
    pub fn update_fps(&mut self, fps: f32) {
        self.average_fps = fps;
    }

    /// 更新内存使用量
    pub fn update_memory_usage(&mut self, usage: usize) {
        self.memory_usage = usage;
    }

    /// 更新运行时间
    pub fn update_uptime(&mut self, uptime_ms: u32) {
        self.uptime_ms = uptime_ms;
    }

    /// 重置统计信息
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

/// 主显示控制器结构
#[derive(Debug)]
pub struct DisplayController {
    /// 显示管理器
    display_manager: DisplayManager,
    /// 图形渲染器
    renderer: GraphicsRenderer,
    /// 字体管理器
    font_manager: FontManager,
    /// 文本渲染器
    text_renderer: TextRenderer,
    /// 配置
    config: DisplayControllerConfig,
    /// 统计信息
    stats: DisplayControllerStats,
    /// 是否已初始化
    initialized: bool,
}

impl DisplayController {
    /// 创建新的显示控制器
    pub fn new(config: DisplayControllerConfig) -> Self {
        Self {
            display_manager: DisplayManager::new(),
            renderer: GraphicsRenderer::new(),
            font_manager: FontManager::new(),
            text_renderer: TextRenderer::new(),
            config,
            stats: DisplayControllerStats::new(),
            initialized: false,
        }
    }

    /// 使用默认配置创建显示控制器
    pub fn with_default_config() -> Self {
        Self::new(DisplayControllerConfig::default())
    }

    /// 初始化显示控制器
    pub fn init(&mut self) -> Result<(), DisplayControllerError> {
        if self.initialized {
            return Ok(());
        }

        // 初始化所有显示设备
        self.display_manager.init_all()?;

        // 加载默认字体
        if let Some(ref font_name) = self.config.default_font.clone() {
            if let Some(font) = self.font_manager.get_font(font_name) {
                // 字体已存在，无需重新加载
            } else {
                // 可以在这里加载默认字体
                let default_font = graphics::fonts::font_utils::create_simple_8x8_font()
                    .map_err(DisplayControllerError::Font)?;
                self.font_manager.load_font(default_font)
                    .map_err(DisplayControllerError::Font)?;
            }
        }

        self.initialized = true;
        Ok(())
    }

    /// 获取显示管理器
    pub fn display_manager(&mut self) -> &mut DisplayManager {
        &mut self.display_manager
    }

    /// 获取图形渲染器
    pub fn renderer(&mut self) -> &mut GraphicsRenderer {
        &mut self.renderer
    }

    /// 获取字体管理器
    pub fn font_manager(&mut self) -> &mut FontManager {
        &mut self.font_manager
    }

    /// 获取文本渲染器
    pub fn text_renderer(&mut self) -> &mut TextRenderer {
        &mut self.text_renderer
    }

    /// 获取配置
    pub fn config(&self) -> &DisplayControllerConfig {
        &self.config
    }

    /// 更新配置
    pub fn update_config(&mut self, config: DisplayControllerConfig) {
        self.config = config;
    }

    /// 获取统计信息
    pub fn stats(&self) -> &DisplayControllerStats {
        &self.stats
    }

    /// 重置统计信息
    pub fn reset_stats(&mut self) {
        self.stats.reset();
    }

    /// 执行系统更新
    pub fn update(&mut self) -> Result<(), DisplayControllerError> {
        if !self.initialized {
            return Err(DisplayControllerError::Initialization);
        }

        // 更新显示管理器
        self.display_manager.update()?;

        // 更新统计信息
        self.stats.increment_draw_operations();

        Ok(())
    }

    /// 刷新所有显示设备
    pub fn refresh_all(&mut self) -> Result<(), DisplayControllerError> {
        self.display_manager.refresh_all()?;
        self.stats.increment_refreshes();
        Ok(())
    }

    /// 清除所有显示设备
    pub fn clear_all(&mut self) -> Result<(), DisplayControllerError> {
        self.display_manager.clear_all()?;
        Ok(())
    }

    /// 执行健康检查
    pub fn health_check(&mut self) -> Result<bool, DisplayControllerError> {
        let health = self.display_manager.health_check_all()?;
        if !health {
            self.stats.increment_errors();
        }
        Ok(health)
    }

    /// 进入睡眠模式
    pub fn sleep(&mut self) -> Result<(), DisplayControllerError> {
        self.display_manager.sleep_all()?;
        Ok(())
    }

    /// 从睡眠模式唤醒
    pub fn wake_up(&mut self) -> Result<(), DisplayControllerError> {
        self.display_manager.wake_up_all()?;
        Ok(())
    }

    /// 获取系统信息
    pub fn get_system_info(&self) -> SystemInfo {
        SystemInfo {
            initialized: self.initialized,
            display_count: self.display_manager.get_display_count(),
            font_count: self.font_manager.list_fonts().len(),
            config: self.config.clone(),
            stats: self.stats,
        }
    }

    /// 执行维护操作
    pub fn maintenance(&mut self) -> Result<(), DisplayControllerError> {
        // 执行显示设备维护
        self.display_manager.maintenance()?;

        // 清理字体缓存（如果需要）
        let (hits, misses) = self.font_manager.get_cache_stats();
        if hits + misses > 1000 && self.font_manager.get_cache_hit_rate() < 0.8 {
            // 缓存命中率低，可能需要优化
        }

        Ok(())
    }
}

impl Default for DisplayController {
    fn default() -> Self {
        Self::with_default_config()
    }
}

/// 系统信息
#[derive(Debug, Clone)]
pub struct SystemInfo {
    /// 是否已初始化
    pub initialized: bool,
    /// 显示设备数量
    pub display_count: usize,
    /// 字体数量
    pub font_count: usize,
    /// 配置信息
    pub config: DisplayControllerConfig,
    /// 统计信息
    pub stats: DisplayControllerStats,
}

/// 便利函数和宏
pub mod prelude {
    //! 预导入模块，包含最常用的类型和函数

    pub use crate::{
        DisplayController, DisplayControllerConfig, DisplayControllerError,
        DisplayManager, GraphicsRenderer, FontManager, TextRenderer,
        Display, DisplayType, DisplayInfo, DisplayConfig,
        Color, ColorFormat, DrawParams, TextParams,
    };

    pub use crate::displays::{
        oled::{OledI2C, OledSPI, OledConfig},
        lcd::{CharacterLcdParallel, GraphicLcdSpi, LcdConfig},
        epaper::{EPaperDisplay, EPaperConfig},
    };

    pub use crate::graphics::{
        renderer::{RenderTarget, Transform2D},
        fonts::{FontInfo, FontType, FontStyle},
    };
}

/// 实用工具函数
pub mod utils {
    use super::*;

    /// 创建默认的显示控制器实例
    pub fn create_default_controller() -> DisplayController {
        DisplayController::with_default_config()
    }

    /// 创建带有调试模式的显示控制器
    pub fn create_debug_controller() -> DisplayController {
        let mut config = DisplayControllerConfig::default();
        config.debug_mode = true;
        config.performance_monitoring = true;
        DisplayController::new(config)
    }

    /// 验证配置
    pub fn validate_config(config: &DisplayControllerConfig) -> Result<(), DisplayControllerError> {
        if config.max_displays == 0 {
            return Err(DisplayControllerError::Configuration);
        }

        if config.max_fonts == 0 {
            return Err(DisplayControllerError::Configuration);
        }

        if let Some(interval) = config.auto_refresh_interval {
            if interval == 0 {
                return Err(DisplayControllerError::Configuration);
            }
        }

        Ok(())
    }

    /// 计算内存使用量
    pub fn calculate_memory_usage(controller: &DisplayController) -> usize {
        let mut total = core::mem::size_of::<DisplayController>();
        
        // 添加显示管理器内存使用
        total += controller.display_manager.get_memory_usage();
        
        // 添加字体管理器内存使用
        for font_name in controller.font_manager.list_fonts() {
            if let Some(font) = controller.font_manager.fonts.get(font_name) {
                total += graphics::fonts::font_utils::calculate_font_memory_usage(font);
            }
        }
        
        total
    }

    /// 获取推荐配置
    pub fn get_recommended_config_for_device(device_type: DisplayType) -> DisplayControllerConfig {
        let mut config = DisplayControllerConfig::default();
        
        match device_type {
            DisplayType::Oled => {
                config.auto_refresh_interval = Some(16); // ~60 FPS
                config.max_displays = 2;
            }
            DisplayType::Lcd => {
                config.auto_refresh_interval = Some(33); // ~30 FPS
                config.max_displays = 1;
            }
            DisplayType::EPaper => {
                config.auto_refresh_interval = Some(1000); // 1 FPS
                config.max_displays = 1;
            }
        }
        
        config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_display_controller_creation() {
        let controller = DisplayController::with_default_config();
        assert!(!controller.initialized);
        assert_eq!(controller.stats.total_draw_operations, 0);
    }

    #[test]
    fn test_config_validation() {
        let mut config = DisplayControllerConfig::default();
        assert!(utils::validate_config(&config).is_ok());
        
        config.max_displays = 0;
        assert!(utils::validate_config(&config).is_err());
    }

    #[test]
    fn test_stats_operations() {
        let mut stats = DisplayControllerStats::new();
        
        stats.increment_draw_operations();
        assert_eq!(stats.total_draw_operations, 1);
        
        stats.increment_refreshes();
        assert_eq!(stats.total_refreshes, 1);
        
        stats.reset();
        assert_eq!(stats.total_draw_operations, 0);
        assert_eq!(stats.total_refreshes, 0);
    }
}
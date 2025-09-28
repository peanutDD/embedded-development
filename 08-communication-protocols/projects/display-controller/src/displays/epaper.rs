//! 电子纸显示屏驱动模块
//!
//! 本模块提供对各种电子纸显示屏的支持，包括黑白、三色和多色电子纸。
//! 支持多种尺寸和不同的刷新模式，适用于低功耗应用场景。
//!
//! # 特性
//! - 支持多种电子纸尺寸（1.54", 2.13", 2.9", 4.2", 7.5"等）
//! - 黑白、红黑白、黄黑白显示模式
//! - 全刷新和部分刷新支持
//! - 低功耗设计
//! - SPI接口通信
//! - 图像缓冲和压缩
//! - 温度补偿
//! - 显示旋转支持

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::{BinaryColor, Rgb888},
    prelude::*,
    text::Text,
    primitives::{Rectangle, PrimitiveStyle},
};
use embedded_hal::{
    delay::DelayNs,
    spi::SpiDevice,
    digital::{InputPin, OutputPin},
};
use heapless::{Vec, String};

use super::{Display, DisplayError, DisplayInfo, DisplayType, InterfaceType};

/// 电子纸显示屏类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EPaperType {
    /// 1.54英寸黑白
    EPD1in54,
    /// 1.54英寸三色（黑红白）
    EPD1in54c,
    /// 2.13英寸黑白
    EPD2in13,
    /// 2.13英寸三色（黑红白）
    EPD2in13c,
    /// 2.9英寸黑白
    EPD2in9,
    /// 2.9英寸三色（黑红白）
    EPD2in9c,
    /// 4.2英寸黑白
    EPD4in2,
    /// 4.2英寸三色（黑红白）
    EPD4in2c,
    /// 7.5英寸黑白
    EPD7in5,
    /// 7.5英寸三色（黑红白）
    EPD7in5c,
}

impl EPaperType {
    /// 获取显示尺寸
    pub fn size(&self) -> Size {
        match self {
            EPaperType::EPD1in54 | EPaperType::EPD1in54c => Size::new(200, 200),
            EPaperType::EPD2in13 | EPaperType::EPD2in13c => Size::new(250, 122),
            EPaperType::EPD2in9 | EPaperType::EPD2in9c => Size::new(296, 128),
            EPaperType::EPD4in2 | EPaperType::EPD4in2c => Size::new(400, 300),
            EPaperType::EPD7in5 | EPaperType::EPD7in5c => Size::new(800, 480),
        }
    }
    
    /// 是否支持彩色
    pub fn is_color(&self) -> bool {
        matches!(self, 
            EPaperType::EPD1in54c | EPaperType::EPD2in13c | 
            EPaperType::EPD2in9c | EPaperType::EPD4in2c | 
            EPaperType::EPD7in5c
        )
    }
    
    /// 获取缓冲区大小（字节）
    pub fn buffer_size(&self) -> usize {
        let size = self.size();
        let pixels = size.width as usize * size.height as usize;
        // 每个像素1位，向上取整到字节
        (pixels + 7) / 8
    }
    
    /// 获取颜色缓冲区大小（仅彩色电子纸）
    pub fn color_buffer_size(&self) -> usize {
        if self.is_color() {
            self.buffer_size()
        } else {
            0
        }
    }
}

/// 电子纸颜色
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EPaperColor {
    /// 白色
    White,
    /// 黑色
    Black,
    /// 红色（仅三色电子纸）
    Red,
    /// 黄色（仅三色电子纸）
    Yellow,
}

impl From<BinaryColor> for EPaperColor {
    fn from(color: BinaryColor) -> Self {
        match color {
            BinaryColor::Off => EPaperColor::White,
            BinaryColor::On => EPaperColor::Black,
        }
    }
}

/// 刷新模式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RefreshMode {
    /// 全刷新（清晰但慢）
    Full,
    /// 快速刷新（快但可能有残影）
    Fast,
    /// 部分刷新（仅更新变化区域）
    Partial,
}

/// 显示旋转
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rotation {
    /// 0度
    Rotate0,
    /// 90度
    Rotate90,
    /// 180度
    Rotate180,
    /// 270度
    Rotate270,
}

/// 电子纸配置
#[derive(Debug, Clone)]
pub struct EPaperConfig {
    /// 电子纸类型
    pub epaper_type: EPaperType,
    /// 刷新模式
    pub refresh_mode: RefreshMode,
    /// 显示旋转
    pub rotation: Rotation,
    /// 温度补偿
    pub temperature_compensation: bool,
    /// 部分刷新支持
    pub partial_refresh_support: bool,
    /// 快速刷新支持
    pub fast_refresh_support: bool,
    /// 自动睡眠延时（毫秒）
    pub auto_sleep_delay_ms: u32,
}

impl Default for EPaperConfig {
    fn default() -> Self {
        Self {
            epaper_type: EPaperType::EPD2in13,
            refresh_mode: RefreshMode::Full,
            rotation: Rotation::Rotate0,
            temperature_compensation: true,
            partial_refresh_support: true,
            fast_refresh_support: false,
            auto_sleep_delay_ms: 10000, // 10秒后自动睡眠
        }
    }
}

/// 电子纸命令（通用）
#[allow(dead_code)]
mod epaper_commands {
    // 基本命令
    pub const DRIVER_OUTPUT_CONTROL: u8 = 0x01;
    pub const BOOSTER_SOFT_START_CONTROL: u8 = 0x0C;
    pub const GATE_SCAN_START_POSITION: u8 = 0x0F;
    pub const DEEP_SLEEP_MODE: u8 = 0x10;
    pub const DATA_ENTRY_MODE_SETTING: u8 = 0x11;
    pub const SW_RESET: u8 = 0x12;
    pub const TEMPERATURE_SENSOR_CONTROL: u8 = 0x1A;
    pub const MASTER_ACTIVATION: u8 = 0x20;
    pub const DISPLAY_UPDATE_CONTROL_1: u8 = 0x21;
    pub const DISPLAY_UPDATE_CONTROL_2: u8 = 0x22;
    pub const WRITE_RAM: u8 = 0x24;
    pub const WRITE_VCOM_REGISTER: u8 = 0x2C;
    pub const WRITE_LUT_REGISTER: u8 = 0x32;
    pub const SET_DUMMY_LINE_PERIOD: u8 = 0x3A;
    pub const SET_GATE_TIME: u8 = 0x3B;
    pub const BORDER_WAVEFORM_CONTROL: u8 = 0x3C;
    pub const SET_RAM_X_ADDRESS_START_END_POSITION: u8 = 0x44;
    pub const SET_RAM_Y_ADDRESS_START_END_POSITION: u8 = 0x45;
    pub const SET_RAM_X_ADDRESS_COUNTER: u8 = 0x4E;
    pub const SET_RAM_Y_ADDRESS_COUNTER: u8 = 0x4F;
    pub const TERMINATE_FRAME_READ_WRITE: u8 = 0xFF;
    
    // 彩色电子纸专用命令
    pub const WRITE_RAM_RED: u8 = 0x26;
    pub const WRITE_RAM_BLACK: u8 = 0x24;
}

/// 2.13英寸电子纸LUT表
#[allow(dead_code)]
mod lut_tables {
    // 全刷新LUT表
    pub const LUT_FULL_UPDATE: [u8; 30] = [
        0x02, 0x02, 0x01, 0x11, 0x12, 0x12, 0x22, 0x22, 0x66, 0x69,
        0x69, 0x59, 0x58, 0x99, 0x99, 0x88, 0x00, 0x00, 0x00, 0x00,
        0xF8, 0xB4, 0x13, 0x51, 0x35, 0x51, 0x51, 0x19, 0x01, 0x00
    ];
    
    // 部分刷新LUT表
    pub const LUT_PARTIAL_UPDATE: [u8; 30] = [
        0x10, 0x18, 0x18, 0x08, 0x18, 0x18, 0x08, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x13, 0x14, 0x44, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ];
    
    // 快速刷新LUT表
    pub const LUT_FAST_UPDATE: [u8; 30] = [
        0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    ];
}

/// 电子纸驱动器
pub struct EPaperDisplay<SPI, CS, DC, RST, BUSY, DELAY> {
    /// SPI接口
    spi: SPI,
    /// 片选引脚
    cs: CS,
    /// 数据/命令选择引脚
    dc: DC,
    /// 复位引脚
    rst: RST,
    /// 忙状态引脚
    busy: BUSY,
    /// 延时提供者
    delay: DELAY,
    /// 配置
    config: EPaperConfig,
    /// 黑白缓冲区
    buffer: Vec<u8, 5000>, // 足够大的缓冲区
    /// 彩色缓冲区（仅彩色电子纸使用）
    color_buffer: Vec<u8, 5000>,
    /// 当前刷新模式
    current_refresh_mode: RefreshMode,
    /// 初始化状态
    initialized: bool,
    /// 睡眠状态
    sleeping: bool,
    /// 最后更新时间（用于自动睡眠）
    last_update_time: u32,
}

impl<SPI, CS, DC, RST, BUSY, DELAY> EPaperDisplay<SPI, CS, DC, RST, BUSY, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    BUSY: InputPin,
    DELAY: DelayNs,
{
    /// 创建新的电子纸显示器实例
    pub fn new(
        spi: SPI,
        cs: CS,
        dc: DC,
        rst: RST,
        busy: BUSY,
        delay: DELAY,
        config: EPaperConfig,
    ) -> Self {
        let buffer_size = config.epaper_type.buffer_size();
        let color_buffer_size = config.epaper_type.color_buffer_size();
        
        let mut buffer = Vec::new();
        buffer.resize(buffer_size, 0xFF).ok(); // 默认白色
        
        let mut color_buffer = Vec::new();
        if color_buffer_size > 0 {
            color_buffer.resize(color_buffer_size, 0xFF).ok(); // 默认白色
        }
        
        Self {
            spi,
            cs,
            dc,
            rst,
            busy,
            delay,
            config,
            buffer,
            color_buffer,
            current_refresh_mode: RefreshMode::Full,
            initialized: false,
            sleeping: false,
            last_update_time: 0,
        }
    }
    
    /// 选择芯片
    fn select(&mut self) -> Result<(), DisplayError> {
        self.cs.set_low().map_err(|_| DisplayError::Communication)
    }
    
    /// 取消选择芯片
    fn deselect(&mut self) -> Result<(), DisplayError> {
        self.cs.set_high().map_err(|_| DisplayError::Communication)
    }
    
    /// 等待忙状态结束
    fn wait_until_idle(&mut self) -> Result<(), DisplayError> {
        let mut timeout = 10000; // 10秒超时
        
        while self.busy.is_high().map_err(|_| DisplayError::Communication)? {
            self.delay.delay_ms(10);
            timeout -= 10;
            if timeout <= 0 {
                return Err(DisplayError::Timeout);
            }
        }
        
        self.delay.delay_ms(200);
        Ok(())
    }
    
    /// 发送命令
    fn send_command(&mut self, command: u8) -> Result<(), DisplayError> {
        self.select()?;
        self.dc.set_low().map_err(|_| DisplayError::Communication)?;
        self.spi.write(&[command]).map_err(|_| DisplayError::Communication)?;
        self.deselect()
    }
    
    /// 发送数据
    fn send_data(&mut self, data: &[u8]) -> Result<(), DisplayError> {
        self.select()?;
        self.dc.set_high().map_err(|_| DisplayError::Communication)?;
        self.spi.write(data).map_err(|_| DisplayError::Communication)?;
        self.deselect()
    }
    
    /// 发送单字节数据
    fn send_data_byte(&mut self, data: u8) -> Result<(), DisplayError> {
        self.send_data(&[data])
    }
    
    /// 硬件复位
    fn hardware_reset(&mut self) -> Result<(), DisplayError> {
        self.rst.set_high().map_err(|_| DisplayError::Communication)?;
        self.delay.delay_ms(200);
        self.rst.set_low().map_err(|_| DisplayError::Communication)?;
        self.delay.delay_ms(2);
        self.rst.set_high().map_err(|_| DisplayError::Communication)?;
        self.delay.delay_ms(200);
        Ok(())
    }
    
    /// 软件复位
    fn software_reset(&mut self) -> Result<(), DisplayError> {
        self.send_command(epaper_commands::SW_RESET)?;
        self.wait_until_idle()
    }
    
    /// 设置LUT表
    fn set_lut(&mut self, lut: &[u8]) -> Result<(), DisplayError> {
        self.send_command(epaper_commands::WRITE_LUT_REGISTER)?;
        self.send_data(lut)
    }
    
    /// 设置内存区域
    fn set_memory_area(&mut self, x_start: u16, y_start: u16, x_end: u16, y_end: u16) -> Result<(), DisplayError> {
        // 设置X地址范围
        self.send_command(epaper_commands::SET_RAM_X_ADDRESS_START_END_POSITION)?;
        self.send_data_byte((x_start >> 3) as u8)?; // X起始地址（字节）
        self.send_data_byte((x_end >> 3) as u8)?;   // X结束地址（字节）
        
        // 设置Y地址范围
        self.send_command(epaper_commands::SET_RAM_Y_ADDRESS_START_END_POSITION)?;
        self.send_data_byte(y_start as u8)?;        // Y起始地址（低字节）
        self.send_data_byte((y_start >> 8) as u8)?; // Y起始地址（高字节）
        self.send_data_byte(y_end as u8)?;          // Y结束地址（低字节）
        self.send_data_byte((y_end >> 8) as u8)?;   // Y结束地址（高字节）
        
        Ok(())
    }
    
    /// 设置内存指针
    fn set_memory_pointer(&mut self, x: u16, y: u16) -> Result<(), DisplayError> {
        // 设置X地址计数器
        self.send_command(epaper_commands::SET_RAM_X_ADDRESS_COUNTER)?;
        self.send_data_byte((x >> 3) as u8)?;
        
        // 设置Y地址计数器
        self.send_command(epaper_commands::SET_RAM_Y_ADDRESS_COUNTER)?;
        self.send_data_byte(y as u8)?;
        self.send_data_byte((y >> 8) as u8)?;
        
        Ok(())
    }
    
    /// 初始化2.13英寸电子纸
    fn initialize_2in13(&mut self) -> Result<(), DisplayError> {
        self.hardware_reset()?;
        self.wait_until_idle()?;
        self.software_reset()?;
        self.wait_until_idle()?;
        
        // 驱动输出控制
        self.send_command(epaper_commands::DRIVER_OUTPUT_CONTROL)?;
        self.send_data(&[0xF9, 0x00, 0x00])?;
        
        // 升压软启动控制
        self.send_command(epaper_commands::BOOSTER_SOFT_START_CONTROL)?;
        self.send_data(&[0xD7, 0xD6, 0x9D])?;
        
        // 写VCOM寄存器
        self.send_command(epaper_commands::WRITE_VCOM_REGISTER)?;
        self.send_data_byte(0xA8)?;
        
        // 设置虚拟线周期
        self.send_command(epaper_commands::SET_DUMMY_LINE_PERIOD)?;
        self.send_data_byte(0x1A)?;
        
        // 设置门时间
        self.send_command(epaper_commands::SET_GATE_TIME)?;
        self.send_data_byte(0x08)?;
        
        // 数据输入模式
        self.send_command(epaper_commands::DATA_ENTRY_MODE_SETTING)?;
        self.send_data_byte(0x03)?;
        
        // 设置LUT表
        match self.current_refresh_mode {
            RefreshMode::Full => self.set_lut(&lut_tables::LUT_FULL_UPDATE)?,
            RefreshMode::Partial => self.set_lut(&lut_tables::LUT_PARTIAL_UPDATE)?,
            RefreshMode::Fast => self.set_lut(&lut_tables::LUT_FAST_UPDATE)?,
        }
        
        Ok(())
    }
    
    /// 初始化电子纸
    fn initialize_epaper(&mut self) -> Result<(), DisplayError> {
        match self.config.epaper_type {
            EPaperType::EPD2in13 | EPaperType::EPD2in13c => self.initialize_2in13(),
            _ => {
                // 其他型号的初始化可以在这里添加
                self.initialize_2in13() // 暂时使用2.13的初始化
            }
        }
    }
    
    /// 设置像素
    pub fn set_pixel(&mut self, x: u16, y: u16, color: EPaperColor) -> Result<(), DisplayError> {
        let size = self.config.epaper_type.size();
        if x >= size.width as u16 || y >= size.height as u16 {
            return Ok(()); // 忽略超出边界的像素
        }
        
        // 应用旋转
        let (x, y) = self.apply_rotation(x, y);
        
        let byte_index = ((y * size.width as u16 + x) / 8) as usize;
        let bit_index = (x % 8) as u8;
        
        if byte_index >= self.buffer.len() {
            return Err(DisplayError::InvalidParameter);
        }
        
        match color {
            EPaperColor::White => {
                self.buffer[byte_index] |= 1 << (7 - bit_index);
            }
            EPaperColor::Black => {
                self.buffer[byte_index] &= !(1 << (7 - bit_index));
            }
            EPaperColor::Red | EPaperColor::Yellow => {
                if self.config.epaper_type.is_color() && !self.color_buffer.is_empty() {
                    // 在黑白缓冲区中设置为白色
                    self.buffer[byte_index] |= 1 << (7 - bit_index);
                    // 在彩色缓冲区中设置颜色
                    self.color_buffer[byte_index] &= !(1 << (7 - bit_index));
                }
            }
        }
        
        Ok(())
    }
    
    /// 应用旋转变换
    fn apply_rotation(&self, x: u16, y: u16) -> (u16, u16) {
        let size = self.config.epaper_type.size();
        match self.config.rotation {
            Rotation::Rotate0 => (x, y),
            Rotation::Rotate90 => (y, size.width as u16 - 1 - x),
            Rotation::Rotate180 => (size.width as u16 - 1 - x, size.height as u16 - 1 - y),
            Rotation::Rotate270 => (size.height as u16 - 1 - y, x),
        }
    }
    
    /// 清除缓冲区
    pub fn clear_buffer(&mut self, color: EPaperColor) {
        let fill_value = match color {
            EPaperColor::White => 0xFF,
            EPaperColor::Black => 0x00,
            _ => 0xFF, // 其他颜色默认为白色
        };
        
        for byte in self.buffer.iter_mut() {
            *byte = fill_value;
        }
        
        if self.config.epaper_type.is_color() {
            let color_fill = match color {
                EPaperColor::Red | EPaperColor::Yellow => 0x00,
                _ => 0xFF,
            };
            
            for byte in self.color_buffer.iter_mut() {
                *byte = color_fill;
            }
        }
    }
    
    /// 绘制矩形
    pub fn draw_rect(&mut self, x: u16, y: u16, width: u16, height: u16, color: EPaperColor, filled: bool) -> Result<(), DisplayError> {
        if filled {
            // 填充矩形
            for dy in 0..height {
                for dx in 0..width {
                    self.set_pixel(x + dx, y + dy, color)?;
                }
            }
        } else {
            // 绘制矩形边框
            // 上边
            for dx in 0..width {
                self.set_pixel(x + dx, y, color)?;
            }
            // 下边
            for dx in 0..width {
                self.set_pixel(x + dx, y + height - 1, color)?;
            }
            // 左边
            for dy in 0..height {
                self.set_pixel(x, y + dy, color)?;
            }
            // 右边
            for dy in 0..height {
                self.set_pixel(x + width - 1, y + dy, color)?;
            }
        }
        Ok(())
    }
    
    /// 绘制文本（简单实现）
    pub fn draw_text(&mut self, text: &str, x: u16, y: u16, color: EPaperColor) -> Result<(), DisplayError> {
        let char_width = 6;
        let char_height = 8;
        
        for (i, ch) in text.chars().enumerate() {
            let char_x = x + (i as u16 * char_width);
            
            // 简单的字符绘制（这里应该使用字体数据）
            if ch != ' ' {
                self.draw_rect(char_x, y, char_width - 1, char_height, color, false)?;
            }
        }
        
        Ok(())
    }
    
    /// 显示缓冲区内容
    pub fn display(&mut self) -> Result<(), DisplayError> {
        if self.sleeping {
            self.wake_up()?;
        }
        
        let size = self.config.epaper_type.size();
        
        // 设置内存区域
        self.set_memory_area(0, 0, size.width as u16 - 1, size.height as u16 - 1)?;
        self.set_memory_pointer(0, 0)?;
        
        // 发送黑白数据
        self.send_command(epaper_commands::WRITE_RAM)?;
        self.send_data(&self.buffer)?;
        
        // 如果是彩色电子纸，发送彩色数据
        if self.config.epaper_type.is_color() && !self.color_buffer.is_empty() {
            self.send_command(epaper_commands::WRITE_RAM_RED)?;
            self.send_data(&self.color_buffer)?;
        }
        
        // 更新显示
        self.update_display()
    }
    
    /// 更新显示
    fn update_display(&mut self) -> Result<(), DisplayError> {
        self.send_command(epaper_commands::DISPLAY_UPDATE_CONTROL_2)?;
        match self.current_refresh_mode {
            RefreshMode::Full => self.send_data_byte(0xC4)?,
            RefreshMode::Partial => self.send_data_byte(0x0C)?,
            RefreshMode::Fast => self.send_data_byte(0xC7)?,
        }
        
        self.send_command(epaper_commands::MASTER_ACTIVATION)?;
        self.send_command(epaper_commands::TERMINATE_FRAME_READ_WRITE)?;
        
        self.wait_until_idle()?;
        
        self.last_update_time = 0; // 这里应该使用实际的时间戳
        Ok(())
    }
    
    /// 设置刷新模式
    pub fn set_refresh_mode(&mut self, mode: RefreshMode) -> Result<(), DisplayError> {
        self.current_refresh_mode = mode;
        
        // 重新设置LUT表
        match mode {
            RefreshMode::Full => self.set_lut(&lut_tables::LUT_FULL_UPDATE),
            RefreshMode::Partial => self.set_lut(&lut_tables::LUT_PARTIAL_UPDATE),
            RefreshMode::Fast => self.set_lut(&lut_tables::LUT_FAST_UPDATE),
        }
    }
    
    /// 部分更新
    pub fn partial_update(&mut self, x: u16, y: u16, width: u16, height: u16) -> Result<(), DisplayError> {
        if !self.config.partial_refresh_support {
            return Err(DisplayError::NotSupported);
        }
        
        let old_mode = self.current_refresh_mode;
        self.set_refresh_mode(RefreshMode::Partial)?;
        
        // 设置部分更新区域
        self.set_memory_area(x, y, x + width - 1, y + height - 1)?;
        self.set_memory_pointer(x, y)?;
        
        // 计算部分缓冲区
        let mut partial_buffer = Vec::<u8, 1000>::new();
        let size = self.config.epaper_type.size();
        
        for dy in 0..height {
            for dx in 0..(width / 8) {
                let buffer_x = (x + dx * 8) / 8;
                let buffer_y = y + dy;
                let buffer_index = (buffer_y * (size.width as u16 / 8) + buffer_x) as usize;
                
                if buffer_index < self.buffer.len() {
                    partial_buffer.push(self.buffer[buffer_index]).ok();
                }
            }
        }
        
        // 发送部分数据
        self.send_command(epaper_commands::WRITE_RAM)?;
        self.send_data(&partial_buffer)?;
        
        self.update_display()?;
        
        // 恢复原来的刷新模式
        self.set_refresh_mode(old_mode)?;
        
        Ok(())
    }
}

// 实现Display trait
impl<SPI, CS, DC, RST, BUSY, DELAY> Display for EPaperDisplay<SPI, CS, DC, RST, BUSY, DELAY>
where
    SPI: SpiDevice,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    BUSY: InputPin,
    DELAY: DelayNs,
{
    type Color = BinaryColor;
    type Error = DisplayError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize_epaper()?;
        self.initialized = true;
        Ok(())
    }
    
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let epaper_color = EPaperColor::from(color);
        self.clear_buffer(epaper_color);
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), Self::Error> {
        self.display()
    }
    
    fn set_pixel(&mut self, point: Point, color: Self::Color) -> Result<(), Self::Error> {
        if point.x >= 0 && point.y >= 0 {
            let epaper_color = EPaperColor::from(color);
            self.set_pixel(point.x as u16, point.y as u16, epaper_color)
        } else {
            Ok(())
        }
    }
    
    fn size(&self) -> Size {
        self.config.epaper_type.size()
    }
    
    fn set_brightness(&mut self, _brightness: u8) -> Result<(), Self::Error> {
        // 电子纸不支持亮度调节
        Err(DisplayError::NotSupported)
    }
    
    fn get_brightness(&self) -> u8 {
        255 // 电子纸没有亮度概念
    }
    
    fn set_display_on(&mut self, _on: bool) -> Result<(), Self::Error> {
        // 电子纸没有开关概念，内容会一直显示
        Ok(())
    }
    
    fn is_healthy(&mut self) -> bool {
        // 检查忙状态引脚是否正常
        self.busy.is_low().unwrap_or(false)
    }
    
    fn sleep(&mut self) -> Result<(), Self::Error> {
        self.send_command(epaper_commands::DEEP_SLEEP_MODE)?;
        self.send_data_byte(0x01)?;
        self.sleeping = true;
        Ok(())
    }
    
    fn wake_up(&mut self) -> Result<(), Self::Error> {
        if self.sleeping {
            self.hardware_reset()?;
            self.initialize_epaper()?;
            self.sleeping = false;
        }
        Ok(())
    }
    
    fn get_info(&self) -> DisplayInfo {
        let type_name = match self.config.epaper_type {
            EPaperType::EPD1in54 => "1.54\" E-Paper",
            EPaperType::EPD1in54c => "1.54\" E-Paper (3-Color)",
            EPaperType::EPD2in13 => "2.13\" E-Paper",
            EPaperType::EPD2in13c => "2.13\" E-Paper (3-Color)",
            EPaperType::EPD2in9 => "2.9\" E-Paper",
            EPaperType::EPD2in9c => "2.9\" E-Paper (3-Color)",
            EPaperType::EPD4in2 => "4.2\" E-Paper",
            EPaperType::EPD4in2c => "4.2\" E-Paper (3-Color)",
            EPaperType::EPD7in5 => "7.5\" E-Paper",
            EPaperType::EPD7in5c => "7.5\" E-Paper (3-Color)",
        };
        
        DisplayInfo {
            display_type: DisplayType::EPaper,
            size: self.config.epaper_type.size(),
            color_depth: if self.config.epaper_type.is_color() { 2 } else { 1 },
            max_brightness: 1, // 电子纸没有亮度概念
            refresh_rate: 0, // 电子纸刷新率很低
            name: type_name,
            manufacturer: "Waveshare",
            interface: InterfaceType::SPI,
        }
    }
}

/// 电子纸工具函数
pub mod epaper_utils {
    use super::*;
    
    /// 创建默认2.13英寸电子纸配置
    pub fn create_2in13_config() -> EPaperConfig {
        EPaperConfig {
            epaper_type: EPaperType::EPD2in13,
            refresh_mode: RefreshMode::Full,
            rotation: Rotation::Rotate0,
            temperature_compensation: true,
            partial_refresh_support: true,
            fast_refresh_support: false,
            auto_sleep_delay_ms: 10000,
        }
    }
    
    /// 创建默认2.13英寸三色电子纸配置
    pub fn create_2in13c_config() -> EPaperConfig {
        EPaperConfig {
            epaper_type: EPaperType::EPD2in13c,
            refresh_mode: RefreshMode::Full,
            rotation: Rotation::Rotate0,
            temperature_compensation: true,
            partial_refresh_support: false, // 三色电子纸通常不支持部分刷新
            fast_refresh_support: false,
            auto_sleep_delay_ms: 10000,
        }
    }
    
    /// 创建默认4.2英寸电子纸配置
    pub fn create_4in2_config() -> EPaperConfig {
        EPaperConfig {
            epaper_type: EPaperType::EPD4in2,
            refresh_mode: RefreshMode::Full,
            rotation: Rotation::Rotate0,
            temperature_compensation: true,
            partial_refresh_support: true,
            fast_refresh_support: false,
            auto_sleep_delay_ms: 15000, // 大屏幕延长睡眠时间
        }
    }
    
    /// 计算刷新时间（毫秒）
    pub fn calculate_refresh_time(epaper_type: EPaperType, refresh_mode: RefreshMode) -> u32 {
        let base_time = match epaper_type {
            EPaperType::EPD1in54 | EPaperType::EPD1in54c => 2000,
            EPaperType::EPD2in13 | EPaperType::EPD2in13c => 2000,
            EPaperType::EPD2in9 | EPaperType::EPD2in9c => 3000,
            EPaperType::EPD4in2 | EPaperType::EPD4in2c => 4000,
            EPaperType::EPD7in5 | EPaperType::EPD7in5c => 6000,
        };
        
        match refresh_mode {
            RefreshMode::Full => base_time,
            RefreshMode::Partial => base_time / 4,
            RefreshMode::Fast => base_time / 10,
        }
    }
    
    /// 验证电子纸配置
    pub fn validate_epaper_config(config: &EPaperConfig) -> Result<(), DisplayError> {
        // 检查三色电子纸的部分刷新支持
        if config.epaper_type.is_color() && config.partial_refresh_support {
            // 大多数三色电子纸不支持部分刷新
            return Err(DisplayError::InvalidParameter);
        }
        
        // 检查快速刷新支持
        if config.refresh_mode == RefreshMode::Fast && !config.fast_refresh_support {
            return Err(DisplayError::NotSupported);
        }
        
        Ok(())
    }
    
    /// 图像抖动处理（用于改善显示效果）
    pub fn apply_dithering(buffer: &mut [u8], width: usize, height: usize) {
        // Floyd-Steinberg抖动算法的简化实现
        for y in 0..height {
            for x in 0..width {
                let byte_index = (y * width + x) / 8;
                let bit_index = (x % 8) as u8;
                
                if byte_index < buffer.len() {
                    let pixel = (buffer[byte_index] >> (7 - bit_index)) & 1;
                    
                    // 简单的抖动处理
                    if pixel == 0 && (x + y) % 2 == 0 {
                        buffer[byte_index] |= 1 << (7 - bit_index);
                    }
                }
            }
        }
    }
    
    /// 计算功耗估算（微安时）
    pub fn estimate_power_consumption(
        epaper_type: EPaperType,
        refresh_mode: RefreshMode,
        refresh_count: u32,
    ) -> u32 {
        let base_consumption = match epaper_type {
            EPaperType::EPD1in54 | EPaperType::EPD1in54c => 100, // 100μAh per refresh
            EPaperType::EPD2in13 | EPaperType::EPD2in13c => 150,
            EPaperType::EPD2in9 | EPaperType::EPD2in9c => 200,
            EPaperType::EPD4in2 | EPaperType::EPD4in2c => 400,
            EPaperType::EPD7in5 | EPaperType::EPD7in5c => 800,
        };
        
        let mode_multiplier = match refresh_mode {
            RefreshMode::Full => 1.0,
            RefreshMode::Partial => 0.3,
            RefreshMode::Fast => 0.1,
        };
        
        (base_consumption as f32 * mode_multiplier * refresh_count as f32) as u32
    }
}
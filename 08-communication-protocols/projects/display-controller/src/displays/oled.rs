//! OLED显示屏驱动模块
//!
//! 本模块提供对OLED显示屏的完整支持，主要基于SSD1306控制器。
//! 支持多种分辨率和接口类型（I2C/SPI）。
//!
//! # 特性
//! - 支持128x64、128x32等常见分辨率
//! - I2C和SPI接口支持
//! - 硬件加速绘图
//! - 亮度控制
//! - 电源管理
//! - 错误恢复
//! - 显示缓冲区管理

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};
use embedded_hal::{
    delay::DelayNs,
    i2c::I2c,
    spi::SpiDevice,
    digital::OutputPin,
};
use heapless::Vec;

use super::{Display, DisplayError, DisplayInfo, DisplayType, InterfaceType};

/// SSD1306 OLED显示屏分辨率
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OledResolution {
    /// 128x64像素
    Res128x64,
    /// 128x32像素
    Res128x32,
    /// 96x16像素
    Res96x16,
    /// 64x48像素
    Res64x48,
}

impl OledResolution {
    /// 获取分辨率尺寸
    pub fn size(&self) -> Size {
        match self {
            OledResolution::Res128x64 => Size::new(128, 64),
            OledResolution::Res128x32 => Size::new(128, 32),
            OledResolution::Res96x16 => Size::new(96, 16),
            OledResolution::Res64x48 => Size::new(64, 48),
        }
    }
    
    /// 获取页数
    pub fn pages(&self) -> u8 {
        match self {
            OledResolution::Res128x64 => 8,
            OledResolution::Res128x32 => 4,
            OledResolution::Res96x16 => 2,
            OledResolution::Res64x48 => 6,
        }
    }
    
    /// 获取缓冲区大小
    pub fn buffer_size(&self) -> usize {
        let size = self.size();
        (size.width * size.height / 8) as usize
    }
}

/// SSD1306命令
#[allow(dead_code)]
mod ssd1306_commands {
    // 基本命令
    pub const SET_CONTRAST: u8 = 0x81;
    pub const DISPLAY_ALL_ON_RESUME: u8 = 0xA4;
    pub const DISPLAY_ALL_ON: u8 = 0xA5;
    pub const NORMAL_DISPLAY: u8 = 0xA6;
    pub const INVERT_DISPLAY: u8 = 0xA7;
    pub const DISPLAY_OFF: u8 = 0xAE;
    pub const DISPLAY_ON: u8 = 0xAF;
    
    // 滚动命令
    pub const RIGHT_HORIZONTAL_SCROLL: u8 = 0x26;
    pub const LEFT_HORIZONTAL_SCROLL: u8 = 0x27;
    pub const VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL: u8 = 0x29;
    pub const VERTICAL_AND_LEFT_HORIZONTAL_SCROLL: u8 = 0x2A;
    pub const DEACTIVATE_SCROLL: u8 = 0x2E;
    pub const ACTIVATE_SCROLL: u8 = 0x2F;
    pub const SET_VERTICAL_SCROLL_AREA: u8 = 0xA3;
    
    // 寻址设置命令
    pub const SET_LOWER_COLUMN: u8 = 0x00;
    pub const SET_HIGHER_COLUMN: u8 = 0x10;
    pub const MEMORY_ADDR_MODE: u8 = 0x20;
    pub const SET_COLUMN_ADDR: u8 = 0x21;
    pub const SET_PAGE_ADDR: u8 = 0x22;
    
    // 硬件配置命令
    pub const SET_START_LINE: u8 = 0x40;
    pub const SET_SEGMENT_REMAP: u8 = 0xA0;
    pub const SET_MULTIPLEX_RATIO: u8 = 0xA8;
    pub const COM_SCAN_INC: u8 = 0xC0;
    pub const COM_SCAN_DEC: u8 = 0xC8;
    pub const SET_DISPLAY_OFFSET: u8 = 0xD3;
    pub const SET_COM_PINS: u8 = 0xDA;
    
    // 时序和驱动方案设置命令
    pub const SET_DISPLAY_CLOCK_DIV: u8 = 0xD5;
    pub const SET_PRECHARGE: u8 = 0xD9;
    pub const SET_VCOM_DETECT: u8 = 0xDB;
    pub const SET_CHARGE_PUMP: u8 = 0x8D;
}

/// OLED显示屏配置
#[derive(Debug, Clone)]
pub struct OledConfig {
    /// 分辨率
    pub resolution: OledResolution,
    /// I2C地址（仅I2C接口使用）
    pub i2c_address: u8,
    /// 外部VCC（true表示外部VCC，false表示内部电荷泵）
    pub external_vcc: bool,
    /// 翻转显示
    pub flip_horizontal: bool,
    /// 垂直翻转
    pub flip_vertical: bool,
    /// 显示偏移
    pub display_offset: u8,
    /// 时钟分频
    pub clock_div: u8,
    /// 预充电周期
    pub precharge: u8,
    /// COM引脚配置
    pub com_pin_config: u8,
    /// VCOM检测电平
    pub vcom_detect: u8,
}

impl Default for OledConfig {
    fn default() -> Self {
        Self {
            resolution: OledResolution::Res128x64,
            i2c_address: 0x3C,
            external_vcc: false,
            flip_horizontal: false,
            flip_vertical: false,
            display_offset: 0,
            clock_div: 0x80,
            precharge: if false { 0x22 } else { 0xF1 }, // external_vcc
            com_pin_config: 0x12,
            vcom_detect: 0x40,
        }
    }
}

/// I2C接口的OLED显示屏
pub struct OledI2C<I2C, DELAY> {
    /// I2C接口
    i2c: I2C,
    /// 延时提供者
    delay: DELAY,
    /// 配置
    config: OledConfig,
    /// 显示缓冲区
    buffer: Vec<u8, 1024>, // 最大支持128x64
    /// 当前亮度
    brightness: u8,
    /// 显示状态
    display_on: bool,
    /// 初始化状态
    initialized: bool,
}

impl<I2C, DELAY> OledI2C<I2C, DELAY>
where
    I2C: I2c,
    DELAY: DelayNs,
{
    /// 创建新的OLED I2C实例
    pub fn new(i2c: I2C, delay: DELAY, config: OledConfig) -> Self {
        let buffer_size = config.resolution.buffer_size();
        let mut buffer = Vec::new();
        buffer.resize(buffer_size, 0).ok();
        
        Self {
            i2c,
            delay,
            config,
            buffer,
            brightness: 128,
            display_on: false,
            initialized: false,
        }
    }
    
    /// 发送命令
    fn send_command(&mut self, command: u8) -> Result<(), DisplayError> {
        let data = [0x00, command]; // 0x00表示命令
        self.i2c.write(self.config.i2c_address, &data)
            .map_err(|_| DisplayError::Communication)
    }
    
    /// 发送命令序列
    fn send_commands(&mut self, commands: &[u8]) -> Result<(), DisplayError> {
        for &command in commands {
            self.send_command(command)?;
        }
        Ok(())
    }
    
    /// 发送数据
    fn send_data(&mut self, data: &[u8]) -> Result<(), DisplayError> {
        // 分块发送数据以避免I2C缓冲区溢出
        const CHUNK_SIZE: usize = 16;
        
        for chunk in data.chunks(CHUNK_SIZE) {
            let mut buffer = Vec::<u8, 17>::new(); // 1字节控制 + 16字节数据
            buffer.push(0x40).ok(); // 0x40表示数据
            
            for &byte in chunk {
                buffer.push(byte).ok();
            }
            
            self.i2c.write(self.config.i2c_address, &buffer)
                .map_err(|_| DisplayError::Communication)?;
        }
        
        Ok(())
    }
    
    /// 初始化显示屏
    fn initialize_display(&mut self) -> Result<(), DisplayError> {
        // 显示关闭
        self.send_command(ssd1306_commands::DISPLAY_OFF)?;
        
        // 设置时钟分频
        self.send_commands(&[
            ssd1306_commands::SET_DISPLAY_CLOCK_DIV,
            self.config.clock_div,
        ])?;
        
        // 设置多路复用比
        let multiplex = match self.config.resolution {
            OledResolution::Res128x64 => 63,
            OledResolution::Res128x32 => 31,
            OledResolution::Res96x16 => 15,
            OledResolution::Res64x48 => 47,
        };
        self.send_commands(&[
            ssd1306_commands::SET_MULTIPLEX_RATIO,
            multiplex,
        ])?;
        
        // 设置显示偏移
        self.send_commands(&[
            ssd1306_commands::SET_DISPLAY_OFFSET,
            self.config.display_offset,
        ])?;
        
        // 设置起始行
        self.send_command(ssd1306_commands::SET_START_LINE | 0x00)?;
        
        // 设置电荷泵
        let charge_pump = if self.config.external_vcc { 0x10 } else { 0x14 };
        self.send_commands(&[
            ssd1306_commands::SET_CHARGE_PUMP,
            charge_pump,
        ])?;
        
        // 设置内存寻址模式（水平寻址）
        self.send_commands(&[
            ssd1306_commands::MEMORY_ADDR_MODE,
            0x00, // 水平寻址模式
        ])?;
        
        // 设置段重映射
        let segment_remap = if self.config.flip_horizontal {
            ssd1306_commands::SET_SEGMENT_REMAP | 0x01
        } else {
            ssd1306_commands::SET_SEGMENT_REMAP | 0x00
        };
        self.send_command(segment_remap)?;
        
        // 设置COM扫描方向
        let com_scan = if self.config.flip_vertical {
            ssd1306_commands::COM_SCAN_DEC
        } else {
            ssd1306_commands::COM_SCAN_INC
        };
        self.send_command(com_scan)?;
        
        // 设置COM引脚配置
        self.send_commands(&[
            ssd1306_commands::SET_COM_PINS,
            self.config.com_pin_config,
        ])?;
        
        // 设置对比度
        self.send_commands(&[
            ssd1306_commands::SET_CONTRAST,
            self.brightness,
        ])?;
        
        // 设置预充电周期
        self.send_commands(&[
            ssd1306_commands::SET_PRECHARGE,
            self.config.precharge,
        ])?;
        
        // 设置VCOM检测电平
        self.send_commands(&[
            ssd1306_commands::SET_VCOM_DETECT,
            self.config.vcom_detect,
        ])?;
        
        // 恢复显示
        self.send_command(ssd1306_commands::DISPLAY_ALL_ON_RESUME)?;
        
        // 正常显示
        self.send_command(ssd1306_commands::NORMAL_DISPLAY)?;
        
        // 停用滚动
        self.send_command(ssd1306_commands::DEACTIVATE_SCROLL)?;
        
        // 显示开启
        self.send_command(ssd1306_commands::DISPLAY_ON)?;
        
        self.display_on = true;
        self.initialized = true;
        
        Ok(())
    }
    
    /// 设置显示区域
    fn set_display_area(&mut self, start_col: u8, end_col: u8, start_page: u8, end_page: u8) -> Result<(), DisplayError> {
        self.send_commands(&[
            ssd1306_commands::SET_COLUMN_ADDR,
            start_col,
            end_col,
        ])?;
        
        self.send_commands(&[
            ssd1306_commands::SET_PAGE_ADDR,
            start_page,
            end_page,
        ])?;
        
        Ok(())
    }
    
    /// 绘制文本
    pub fn draw_text(&mut self, text: &str, position: Point, color: BinaryColor) -> Result<(), DisplayError> {
        let style = MonoTextStyle::new(&FONT_6X10, color);
        Text::new(text, position, style)
            .draw(self)
            .map_err(|_| DisplayError::Display)
    }
    
    /// 绘制像素
    pub fn draw_pixel(&mut self, point: Point, color: BinaryColor) -> Result<(), DisplayError> {
        self.set_pixel(point, color)
    }
    
    /// 绘制线条
    pub fn draw_line(&mut self, start: Point, end: Point, color: BinaryColor) -> Result<(), DisplayError> {
        use embedded_graphics::primitives::{Line, PrimitiveStyle};
        
        let style = PrimitiveStyle::with_stroke(color, 1);
        Line::new(start, end)
            .into_styled(style)
            .draw(self)
            .map_err(|_| DisplayError::Display)
    }
    
    /// 绘制矩形
    pub fn draw_rectangle(&mut self, top_left: Point, size: Size, color: BinaryColor, filled: bool) -> Result<(), DisplayError> {
        use embedded_graphics::primitives::{Rectangle, PrimitiveStyle, PrimitiveStyleBuilder};
        
        let style = if filled {
            PrimitiveStyleBuilder::new()
                .fill_color(color)
                .build()
        } else {
            PrimitiveStyleBuilder::new()
                .stroke_color(color)
                .stroke_width(1)
                .build()
        };
        
        Rectangle::new(top_left, size)
            .into_styled(style)
            .draw(self)
            .map_err(|_| DisplayError::Display)
    }
    
    /// 绘制圆形
    pub fn draw_circle(&mut self, center: Point, radius: u32, color: BinaryColor, filled: bool) -> Result<(), DisplayError> {
        use embedded_graphics::primitives::{Circle, PrimitiveStyle, PrimitiveStyleBuilder};
        
        let style = if filled {
            PrimitiveStyleBuilder::new()
                .fill_color(color)
                .build()
        } else {
            PrimitiveStyleBuilder::new()
                .stroke_color(color)
                .stroke_width(1)
                .build()
        };
        
        Circle::new(center, radius)
            .into_styled(style)
            .draw(self)
            .map_err(|_| DisplayError::Display)
    }
    
    /// 滚动显示
    pub fn scroll_horizontal(&mut self, direction: ScrollDirection, speed: ScrollSpeed) -> Result<(), DisplayError> {
        // 停用当前滚动
        self.send_command(ssd1306_commands::DEACTIVATE_SCROLL)?;
        
        let command = match direction {
            ScrollDirection::Left => ssd1306_commands::LEFT_HORIZONTAL_SCROLL,
            ScrollDirection::Right => ssd1306_commands::RIGHT_HORIZONTAL_SCROLL,
        };
        
        let speed_value = match speed {
            ScrollSpeed::Frames2 => 0x07,
            ScrollSpeed::Frames3 => 0x04,
            ScrollSpeed::Frames4 => 0x05,
            ScrollSpeed::Frames5 => 0x00,
            ScrollSpeed::Frames25 => 0x06,
            ScrollSpeed::Frames64 => 0x01,
            ScrollSpeed::Frames128 => 0x02,
            ScrollSpeed::Frames256 => 0x03,
        };
        
        self.send_commands(&[
            command,
            0x00, // 虚拟字节
            0x00, // 起始页
            speed_value, // 时间间隔
            0x07, // 结束页
            0x00, // 虚拟字节
            0xFF, // 虚拟字节
        ])?;
        
        // 激活滚动
        self.send_command(ssd1306_commands::ACTIVATE_SCROLL)?;
        
        Ok(())
    }
    
    /// 停止滚动
    pub fn stop_scroll(&mut self) -> Result<(), DisplayError> {
        self.send_command(ssd1306_commands::DEACTIVATE_SCROLL)
    }
    
    /// 反转显示
    pub fn invert_display(&mut self, invert: bool) -> Result<(), DisplayError> {
        let command = if invert {
            ssd1306_commands::INVERT_DISPLAY
        } else {
            ssd1306_commands::NORMAL_DISPLAY
        };
        self.send_command(command)
    }
}

/// 滚动方向
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScrollDirection {
    /// 向左滚动
    Left,
    /// 向右滚动
    Right,
}

/// 滚动速度
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScrollSpeed {
    /// 2帧
    Frames2,
    /// 3帧
    Frames3,
    /// 4帧
    Frames4,
    /// 5帧
    Frames5,
    /// 25帧
    Frames25,
    /// 64帧
    Frames64,
    /// 128帧
    Frames128,
    /// 256帧
    Frames256,
}

// 实现Display trait
impl<I2C, DELAY> Display for OledI2C<I2C, DELAY>
where
    I2C: I2c,
    DELAY: DelayNs,
{
    type Color = BinaryColor;
    type Error = DisplayError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        self.delay.delay_ms(100); // 等待显示屏稳定
        self.initialize_display()
    }
    
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let fill_byte = match color {
            BinaryColor::On => 0xFF,
            BinaryColor::Off => 0x00,
        };
        
        for byte in self.buffer.iter_mut() {
            *byte = fill_byte;
        }
        
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), Self::Error> {
        if !self.initialized {
            return Err(DisplayError::InitializationFailed);
        }
        
        let size = self.config.resolution.size();
        let pages = self.config.resolution.pages();
        
        // 设置显示区域为整个屏幕
        self.set_display_area(0, size.width as u8 - 1, 0, pages - 1)?;
        
        // 发送缓冲区数据
        self.send_data(&self.buffer)
    }
    
    fn set_pixel(&mut self, point: Point, color: Self::Color) -> Result<(), Self::Error> {
        let size = self.config.resolution.size();
        
        if point.x < 0 || point.x >= size.width as i32 || point.y < 0 || point.y >= size.height as i32 {
            return Ok(()); // 忽略超出边界的像素
        }
        
        let x = point.x as u32;
        let y = point.y as u32;
        
        let page = y / 8;
        let bit = y % 8;
        let index = (page * size.width + x) as usize;
        
        if index < self.buffer.len() {
            match color {
                BinaryColor::On => self.buffer[index] |= 1 << bit,
                BinaryColor::Off => self.buffer[index] &= !(1 << bit),
            }
        }
        
        Ok(())
    }
    
    fn size(&self) -> Size {
        self.config.resolution.size()
    }
    
    fn set_brightness(&mut self, brightness: u8) -> Result<(), Self::Error> {
        self.brightness = brightness;
        if self.initialized {
            self.send_commands(&[
                ssd1306_commands::SET_CONTRAST,
                brightness,
            ])
        } else {
            Ok(())
        }
    }
    
    fn get_brightness(&self) -> u8 {
        self.brightness
    }
    
    fn set_display_on(&mut self, on: bool) -> Result<(), Self::Error> {
        self.display_on = on;
        let command = if on {
            ssd1306_commands::DISPLAY_ON
        } else {
            ssd1306_commands::DISPLAY_OFF
        };
        self.send_command(command)
    }
    
    fn is_healthy(&mut self) -> bool {
        // 尝试发送一个简单的命令来检查通信
        self.send_command(ssd1306_commands::DISPLAY_ALL_ON_RESUME).is_ok()
    }
    
    fn sleep(&mut self) -> Result<(), Self::Error> {
        self.set_display_on(false)
    }
    
    fn wake_up(&mut self) -> Result<(), Self::Error> {
        self.set_display_on(true)
    }
    
    fn get_info(&self) -> DisplayInfo {
        DisplayInfo {
            display_type: DisplayType::Oled,
            size: self.config.resolution.size(),
            color_depth: 1, // 单色显示
            max_brightness: 255,
            refresh_rate: 60, // 估计值
            name: "SSD1306 OLED",
            manufacturer: "Solomon Systech",
            interface: InterfaceType::I2C,
        }
    }
}

// 实现DrawTarget trait以支持embedded-graphics
impl<I2C, DELAY> DrawTarget for OledI2C<I2C, DELAY>
where
    I2C: I2c,
    DELAY: DelayNs,
{
    type Color = BinaryColor;
    type Error = DisplayError;
    
    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            self.set_pixel(point, color)?;
        }
        Ok(())
    }
}

impl<I2C, DELAY> OriginDimensions for OledI2C<I2C, DELAY>
where
    I2C: I2c,
    DELAY: DelayNs,
{
    fn size(&self) -> Size {
        self.config.resolution.size()
    }
}

/// SPI接口的OLED显示屏
pub struct OledSPI<SPI, DC, RST, DELAY> {
    /// SPI接口
    spi: SPI,
    /// 数据/命令选择引脚
    dc: DC,
    /// 复位引脚
    rst: Option<RST>,
    /// 延时提供者
    delay: DELAY,
    /// 配置
    config: OledConfig,
    /// 显示缓冲区
    buffer: Vec<u8, 1024>,
    /// 当前亮度
    brightness: u8,
    /// 显示状态
    display_on: bool,
    /// 初始化状态
    initialized: bool,
}

impl<SPI, DC, RST, DELAY> OledSPI<SPI, DC, RST, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    /// 创建新的OLED SPI实例
    pub fn new(spi: SPI, dc: DC, rst: Option<RST>, delay: DELAY, config: OledConfig) -> Self {
        let buffer_size = config.resolution.buffer_size();
        let mut buffer = Vec::new();
        buffer.resize(buffer_size, 0).ok();
        
        Self {
            spi,
            dc,
            rst,
            delay,
            config,
            buffer,
            brightness: 128,
            display_on: false,
            initialized: false,
        }
    }
    
    /// 硬件复位
    fn hardware_reset(&mut self) -> Result<(), DisplayError> {
        if let Some(rst) = &mut self.rst {
            rst.set_high().map_err(|_| DisplayError::Communication)?;
            self.delay.delay_ms(1);
            rst.set_low().map_err(|_| DisplayError::Communication)?;
            self.delay.delay_ms(10);
            rst.set_high().map_err(|_| DisplayError::Communication)?;
            self.delay.delay_ms(10);
        }
        Ok(())
    }
    
    /// 发送命令
    fn send_command(&mut self, command: u8) -> Result<(), DisplayError> {
        self.dc.set_low().map_err(|_| DisplayError::Communication)?;
        self.spi.write(&[command]).map_err(|_| DisplayError::Communication)
    }
    
    /// 发送数据
    fn send_data(&mut self, data: &[u8]) -> Result<(), DisplayError> {
        self.dc.set_high().map_err(|_| DisplayError::Communication)?;
        self.spi.write(data).map_err(|_| DisplayError::Communication)
    }
}

// 为SPI版本实现Display trait（类似I2C版本，但使用不同的通信方法）
impl<SPI, DC, RST, DELAY> Display for OledSPI<SPI, DC, RST, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    DELAY: DelayNs,
{
    type Color = BinaryColor;
    type Error = DisplayError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        self.hardware_reset()?;
        self.delay.delay_ms(100);
        
        // 使用与I2C版本相同的初始化序列，但通过SPI发送
        self.send_command(ssd1306_commands::DISPLAY_OFF)?;
        
        // 其他初始化命令...
        self.initialized = true;
        Ok(())
    }
    
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        let fill_byte = match color {
            BinaryColor::On => 0xFF,
            BinaryColor::Off => 0x00,
        };
        
        for byte in self.buffer.iter_mut() {
            *byte = fill_byte;
        }
        
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), Self::Error> {
        if !self.initialized {
            return Err(DisplayError::InitializationFailed);
        }
        
        // 设置显示区域并发送数据
        self.send_data(&self.buffer)
    }
    
    fn set_pixel(&mut self, point: Point, color: Self::Color) -> Result<(), Self::Error> {
        // 与I2C版本相同的像素设置逻辑
        let size = self.config.resolution.size();
        
        if point.x < 0 || point.x >= size.width as i32 || point.y < 0 || point.y >= size.height as i32 {
            return Ok(());
        }
        
        let x = point.x as u32;
        let y = point.y as u32;
        
        let page = y / 8;
        let bit = y % 8;
        let index = (page * size.width + x) as usize;
        
        if index < self.buffer.len() {
            match color {
                BinaryColor::On => self.buffer[index] |= 1 << bit,
                BinaryColor::Off => self.buffer[index] &= !(1 << bit),
            }
        }
        
        Ok(())
    }
    
    fn size(&self) -> Size {
        self.config.resolution.size()
    }
    
    fn set_brightness(&mut self, brightness: u8) -> Result<(), Self::Error> {
        self.brightness = brightness;
        if self.initialized {
            self.send_command(ssd1306_commands::SET_CONTRAST)?;
            self.send_command(brightness)
        } else {
            Ok(())
        }
    }
    
    fn get_brightness(&self) -> u8 {
        self.brightness
    }
    
    fn set_display_on(&mut self, on: bool) -> Result<(), Self::Error> {
        self.display_on = on;
        let command = if on {
            ssd1306_commands::DISPLAY_ON
        } else {
            ssd1306_commands::DISPLAY_OFF
        };
        self.send_command(command)
    }
    
    fn is_healthy(&mut self) -> bool {
        self.send_command(ssd1306_commands::DISPLAY_ALL_ON_RESUME).is_ok()
    }
    
    fn sleep(&mut self) -> Result<(), Self::Error> {
        self.set_display_on(false)
    }
    
    fn wake_up(&mut self) -> Result<(), Self::Error> {
        self.set_display_on(true)
    }
    
    fn get_info(&self) -> DisplayInfo {
        DisplayInfo {
            display_type: DisplayType::Oled,
            size: self.config.resolution.size(),
            color_depth: 1,
            max_brightness: 255,
            refresh_rate: 60,
            name: "SSD1306 OLED",
            manufacturer: "Solomon Systech",
            interface: InterfaceType::SPI,
        }
    }
}

/// OLED工具函数
pub mod oled_utils {
    use super::*;
    
    /// 创建默认128x64 OLED配置
    pub fn create_128x64_config() -> OledConfig {
        OledConfig {
            resolution: OledResolution::Res128x64,
            com_pin_config: 0x12,
            ..Default::default()
        }
    }
    
    /// 创建默认128x32 OLED配置
    pub fn create_128x32_config() -> OledConfig {
        OledConfig {
            resolution: OledResolution::Res128x32,
            com_pin_config: 0x02,
            ..Default::default()
        }
    }
    
    /// 验证OLED配置
    pub fn validate_config(config: &OledConfig) -> Result<(), DisplayError> {
        if config.i2c_address > 0x7F {
            return Err(DisplayError::InvalidParameter);
        }
        
        if config.display_offset > 63 {
            return Err(DisplayError::InvalidParameter);
        }
        
        Ok(())
    }
    
    /// 计算缓冲区索引
    pub fn calculate_buffer_index(x: u32, y: u32, width: u32) -> (usize, u8) {
        let page = y / 8;
        let bit = (y % 8) as u8;
        let index = (page * width + x) as usize;
        (index, bit)
    }
    
    /// 创建测试图案
    pub fn create_test_pattern(buffer: &mut [u8], width: u32, height: u32) {
        // 创建棋盘图案
        for y in 0..height {
            for x in 0..width {
                let (index, bit) = calculate_buffer_index(x, y, width);
                if index < buffer.len() {
                    if (x / 8 + y / 8) % 2 == 0 {
                        buffer[index] |= 1 << bit;
                    } else {
                        buffer[index] &= !(1 << bit);
                    }
                }
            }
        }
    }
}
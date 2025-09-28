//! LCD显示屏驱动模块
//!
//! 本模块提供对各种LCD显示屏的支持，包括字符LCD（如HD44780）和图形LCD。
//! 支持多种接口类型（并行、I2C、SPI）和不同的显示模式。
//!
//! # 特性
//! - 支持HD44780兼容的字符LCD
//! - 支持ILI9341等图形LCD
//! - 多种接口支持（4位/8位并行、I2C、SPI）
//! - 背光控制
//! - 自定义字符支持
//! - 滚动和光标控制
//! - 颜色显示支持（图形LCD）

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::{Rgb565, BinaryColor},
    prelude::*,
    text::Text,
};
use embedded_hal::{
    delay::DelayNs,
    i2c::I2c,
    spi::SpiDevice,
    digital::OutputPin,
};
use heapless::{Vec, String};

use super::{Display, DisplayError, DisplayInfo, DisplayType, InterfaceType};

/// LCD显示屏类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LcdType {
    /// HD44780兼容字符LCD
    Character,
    /// ILI9341图形LCD
    GraphicIli9341,
    /// ST7735图形LCD
    GraphicSt7735,
    /// 其他图形LCD
    GraphicOther,
}

/// 字符LCD尺寸
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CharacterLcdSize {
    /// 16x2字符
    Size16x2,
    /// 20x4字符
    Size20x4,
    /// 16x4字符
    Size16x4,
    /// 20x2字符
    Size20x2,
}

impl CharacterLcdSize {
    /// 获取列数
    pub fn columns(&self) -> u8 {
        match self {
            CharacterLcdSize::Size16x2 | CharacterLcdSize::Size16x4 => 16,
            CharacterLcdSize::Size20x4 | CharacterLcdSize::Size20x2 => 20,
        }
    }
    
    /// 获取行数
    pub fn rows(&self) -> u8 {
        match self {
            CharacterLcdSize::Size16x2 | CharacterLcdSize::Size20x2 => 2,
            CharacterLcdSize::Size20x4 | CharacterLcdSize::Size16x4 => 4,
        }
    }
    
    /// 获取总字符数
    pub fn total_chars(&self) -> u16 {
        self.columns() as u16 * self.rows() as u16
    }
}

/// HD44780命令
#[allow(dead_code)]
mod hd44780_commands {
    // 基本命令
    pub const CLEAR_DISPLAY: u8 = 0x01;
    pub const RETURN_HOME: u8 = 0x02;
    
    // 输入模式设置
    pub const ENTRY_MODE_SET: u8 = 0x04;
    pub const ENTRY_INCREMENT: u8 = 0x02;
    pub const ENTRY_SHIFT: u8 = 0x01;
    
    // 显示控制
    pub const DISPLAY_CONTROL: u8 = 0x08;
    pub const DISPLAY_ON: u8 = 0x04;
    pub const CURSOR_ON: u8 = 0x02;
    pub const BLINK_ON: u8 = 0x01;
    
    // 光标/显示移动
    pub const CURSOR_SHIFT: u8 = 0x10;
    pub const DISPLAY_MOVE: u8 = 0x08;
    pub const MOVE_RIGHT: u8 = 0x04;
    
    // 功能设置
    pub const FUNCTION_SET: u8 = 0x20;
    pub const DATA_LENGTH_8BIT: u8 = 0x10;
    pub const TWO_LINES: u8 = 0x08;
    pub const FONT_5X10: u8 = 0x04;
    
    // CGRAM地址设置
    pub const SET_CGRAM_ADDR: u8 = 0x40;
    
    // DDRAM地址设置
    pub const SET_DDRAM_ADDR: u8 = 0x80;
}

/// ILI9341命令
#[allow(dead_code)]
mod ili9341_commands {
    pub const SOFTWARE_RESET: u8 = 0x01;
    pub const READ_display_id: u8 = 0x04;
    pub const READ_display_status: u8 = 0x09;
    pub const READ_display_power_mode: u8 = 0x0A;
    pub const READ_display_madctl: u8 = 0x0B;
    pub const READ_display_pixel_format: u8 = 0x0C;
    pub const READ_display_image_format: u8 = 0x0D;
    pub const READ_display_signal_mode: u8 = 0x0E;
    pub const READ_display_self_diagnostic: u8 = 0x0F;
    pub const SLEEP_IN: u8 = 0x10;
    pub const SLEEP_OUT: u8 = 0x11;
    pub const PARTIAL_MODE_ON: u8 = 0x12;
    pub const NORMAL_DISPLAY_MODE_ON: u8 = 0x13;
    pub const DISPLAY_INVERSION_OFF: u8 = 0x20;
    pub const DISPLAY_INVERSION_ON: u8 = 0x21;
    pub const GAMMA_SET: u8 = 0x26;
    pub const DISPLAY_OFF: u8 = 0x28;
    pub const DISPLAY_ON: u8 = 0x29;
    pub const COLUMN_ADDRESS_SET: u8 = 0x2A;
    pub const PAGE_ADDRESS_SET: u8 = 0x2B;
    pub const MEMORY_WRITE: u8 = 0x2C;
    pub const COLOR_SET: u8 = 0x2D;
    pub const MEMORY_READ: u8 = 0x2E;
    pub const PARTIAL_AREA: u8 = 0x30;
    pub const VERTICAL_SCROLLING_DEFINITION: u8 = 0x33;
    pub const TEARING_EFFECT_LINE_OFF: u8 = 0x34;
    pub const TEARING_EFFECT_LINE_ON: u8 = 0x35;
    pub const MEMORY_ACCESS_CONTROL: u8 = 0x36;
    pub const VERTICAL_SCROLLING_START_ADDRESS: u8 = 0x37;
    pub const IDLE_MODE_OFF: u8 = 0x38;
    pub const IDLE_MODE_ON: u8 = 0x39;
    pub const PIXEL_FORMAT_SET: u8 = 0x3A;
    pub const WRITE_MEMORY_CONTINUE: u8 = 0x3C;
    pub const READ_MEMORY_CONTINUE: u8 = 0x3E;
    pub const SET_TEAR_SCANLINE: u8 = 0x44;
    pub const GET_SCANLINE: u8 = 0x45;
    pub const WRITE_DISPLAY_BRIGHTNESS: u8 = 0x51;
    pub const READ_DISPLAY_BRIGHTNESS: u8 = 0x52;
    pub const WRITE_CTRL_DISPLAY: u8 = 0x53;
    pub const READ_CTRL_DISPLAY: u8 = 0x54;
    pub const WRITE_CONTENT_ADAPTIVE_BRIGHTNESS_CONTROL: u8 = 0x55;
    pub const READ_CONTENT_ADAPTIVE_BRIGHTNESS_CONTROL: u8 = 0x56;
    pub const WRITE_CABC_MINIMUM_BRIGHTNESS: u8 = 0x5E;
    pub const READ_CABC_MINIMUM_BRIGHTNESS: u8 = 0x5F;
    pub const READ_ID1: u8 = 0xDA;
    pub const READ_ID2: u8 = 0xDB;
    pub const READ_ID3: u8 = 0xDC;
    pub const RGB_INTERFACE_SIGNAL_CONTROL: u8 = 0xB0;
    pub const FRAME_CONTROL_NORMAL: u8 = 0xB1;
    pub const FRAME_CONTROL_IDLE: u8 = 0xB2;
    pub const FRAME_CONTROL_PARTIAL: u8 = 0xB3;
    pub const DISPLAY_INVERSION_CONTROL: u8 = 0xB4;
    pub const BLANKING_PORCH_CONTROL: u8 = 0xB5;
    pub const DISPLAY_FUNCTION_CONTROL: u8 = 0xB6;
    pub const ENTRY_MODE_SET: u8 = 0xB7;
    pub const BACKLIGHT_CONTROL_1: u8 = 0xB8;
    pub const BACKLIGHT_CONTROL_2: u8 = 0xB9;
    pub const BACKLIGHT_CONTROL_3: u8 = 0xBA;
    pub const BACKLIGHT_CONTROL_4: u8 = 0xBB;
    pub const BACKLIGHT_CONTROL_5: u8 = 0xBC;
    pub const BACKLIGHT_CONTROL_7: u8 = 0xBE;
    pub const BACKLIGHT_CONTROL_8: u8 = 0xBF;
    pub const POWER_CONTROL_1: u8 = 0xC0;
    pub const POWER_CONTROL_2: u8 = 0xC1;
    pub const VCOM_CONTROL_1: u8 = 0xC5;
    pub const VCOM_CONTROL_2: u8 = 0xC7;
    pub const NV_MEMORY_WRITE: u8 = 0xD0;
    pub const NV_MEMORY_PROTECTION_KEY: u8 = 0xD1;
    pub const NV_MEMORY_STATUS_READ: u8 = 0xD2;
    pub const READ_ID4: u8 = 0xD3;
    pub const POSITIVE_GAMMA_CORRECTION: u8 = 0xE0;
    pub const NEGATIVE_GAMMA_CORRECTION: u8 = 0xE1;
    pub const DIGITAL_GAMMA_CONTROL_1: u8 = 0xE2;
    pub const DIGITAL_GAMMA_CONTROL_2: u8 = 0xE3;
    pub const INTERFACE_CONTROL: u8 = 0xF6;
}

/// LCD配置
#[derive(Debug, Clone)]
pub struct LcdConfig {
    /// LCD类型
    pub lcd_type: LcdType,
    /// 字符LCD尺寸（仅字符LCD使用）
    pub char_size: Option<CharacterLcdSize>,
    /// 图形LCD分辨率（仅图形LCD使用）
    pub resolution: Option<Size>,
    /// I2C地址（仅I2C接口使用）
    pub i2c_address: u8,
    /// 背光控制
    pub backlight_control: bool,
    /// 数据位宽（4位或8位，仅并行接口）
    pub data_width: u8,
    /// 字体大小（5x8或5x10）
    pub font_5x10: bool,
    /// 显示行数（1行或2行）
    pub two_lines: bool,
}

impl Default for LcdConfig {
    fn default() -> Self {
        Self {
            lcd_type: LcdType::Character,
            char_size: Some(CharacterLcdSize::Size16x2),
            resolution: None,
            i2c_address: 0x27, // PCF8574常用地址
            backlight_control: true,
            data_width: 4,
            font_5x10: false,
            two_lines: true,
        }
    }
}

/// 字符LCD（HD44780兼容）- 并行接口
pub struct CharacterLcdParallel<RS, EN, D4, D5, D6, D7, BL, DELAY> {
    /// 寄存器选择引脚
    rs: RS,
    /// 使能引脚
    en: EN,
    /// 数据引脚D4
    d4: D4,
    /// 数据引脚D5
    d5: D5,
    /// 数据引脚D6
    d6: D6,
    /// 数据引脚D7
    d7: D7,
    /// 背光控制引脚
    backlight: Option<BL>,
    /// 延时提供者
    delay: DELAY,
    /// 配置
    config: LcdConfig,
    /// 当前光标位置
    cursor_pos: (u8, u8),
    /// 显示状态
    display_on: bool,
    /// 光标状态
    cursor_on: bool,
    /// 闪烁状态
    blink_on: bool,
    /// 初始化状态
    initialized: bool,
}

impl<RS, EN, D4, D5, D6, D7, BL, DELAY> CharacterLcdParallel<RS, EN, D4, D5, D6, D7, BL, DELAY>
where
    RS: OutputPin,
    EN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
    BL: OutputPin,
    DELAY: DelayNs,
{
    /// 创建新的字符LCD实例
    pub fn new(
        rs: RS,
        en: EN,
        d4: D4,
        d5: D5,
        d6: D6,
        d7: D7,
        backlight: Option<BL>,
        delay: DELAY,
        config: LcdConfig,
    ) -> Self {
        Self {
            rs,
            en,
            d4,
            d5,
            d6,
            d7,
            backlight,
            delay,
            config,
            cursor_pos: (0, 0),
            display_on: true,
            cursor_on: false,
            blink_on: false,
            initialized: false,
        }
    }
    
    /// 写入4位数据
    fn write_4bits(&mut self, data: u8) -> Result<(), DisplayError> {
        // 设置数据引脚
        if data & 0x01 != 0 {
            self.d4.set_high().map_err(|_| DisplayError::Communication)?;
        } else {
            self.d4.set_low().map_err(|_| DisplayError::Communication)?;
        }
        
        if data & 0x02 != 0 {
            self.d5.set_high().map_err(|_| DisplayError::Communication)?;
        } else {
            self.d5.set_low().map_err(|_| DisplayError::Communication)?;
        }
        
        if data & 0x04 != 0 {
            self.d6.set_high().map_err(|_| DisplayError::Communication)?;
        } else {
            self.d6.set_low().map_err(|_| DisplayError::Communication)?;
        }
        
        if data & 0x08 != 0 {
            self.d7.set_high().map_err(|_| DisplayError::Communication)?;
        } else {
            self.d7.set_low().map_err(|_| DisplayError::Communication)?;
        }
        
        // 脉冲使能引脚
        self.en.set_high().map_err(|_| DisplayError::Communication)?;
        self.delay.delay_us(1);
        self.en.set_low().map_err(|_| DisplayError::Communication)?;
        self.delay.delay_us(100);
        
        Ok(())
    }
    
    /// 写入8位数据（通过两次4位写入）
    fn write_8bits(&mut self, data: u8) -> Result<(), DisplayError> {
        self.write_4bits(data >> 4)?; // 高4位
        self.write_4bits(data & 0x0F) // 低4位
    }
    
    /// 发送命令
    fn send_command(&mut self, command: u8) -> Result<(), DisplayError> {
        self.rs.set_low().map_err(|_| DisplayError::Communication)?;
        self.write_8bits(command)?;
        
        // 某些命令需要更长的延时
        match command {
            hd44780_commands::CLEAR_DISPLAY | hd44780_commands::RETURN_HOME => {
                self.delay.delay_ms(2);
            }
            _ => {
                self.delay.delay_us(50);
            }
        }
        
        Ok(())
    }
    
    /// 发送数据
    fn send_data(&mut self, data: u8) -> Result<(), DisplayError> {
        self.rs.set_high().map_err(|_| DisplayError::Communication)?;
        self.write_8bits(data)?;
        self.delay.delay_us(50);
        Ok(())
    }
    
    /// 初始化LCD
    fn initialize_lcd(&mut self) -> Result<(), DisplayError> {
        // 等待LCD上电稳定
        self.delay.delay_ms(50);
        
        // 初始化序列（4位模式）
        self.rs.set_low().map_err(|_| DisplayError::Communication)?;
        
        // 发送3次0x03来确保进入8位模式
        for _ in 0..3 {
            self.write_4bits(0x03)?;
            self.delay.delay_ms(5);
        }
        
        // 切换到4位模式
        self.write_4bits(0x02)?;
        self.delay.delay_ms(1);
        
        // 功能设置：4位模式，2行，5x8字体
        let mut function_set = hd44780_commands::FUNCTION_SET;
        if self.config.two_lines {
            function_set |= hd44780_commands::TWO_LINES;
        }
        if self.config.font_5x10 {
            function_set |= hd44780_commands::FONT_5X10;
        }
        self.send_command(function_set)?;
        
        // 显示控制：显示开，光标关，闪烁关
        let mut display_control = hd44780_commands::DISPLAY_CONTROL | hd44780_commands::DISPLAY_ON;
        if self.cursor_on {
            display_control |= hd44780_commands::CURSOR_ON;
        }
        if self.blink_on {
            display_control |= hd44780_commands::BLINK_ON;
        }
        self.send_command(display_control)?;
        
        // 清除显示
        self.send_command(hd44780_commands::CLEAR_DISPLAY)?;
        
        // 输入模式设置：光标右移，不移动显示
        self.send_command(hd44780_commands::ENTRY_MODE_SET | hd44780_commands::ENTRY_INCREMENT)?;
        
        self.initialized = true;
        Ok(())
    }
    
    /// 设置光标位置
    pub fn set_cursor(&mut self, col: u8, row: u8) -> Result<(), DisplayError> {
        if let Some(char_size) = self.config.char_size {
            if col >= char_size.columns() || row >= char_size.rows() {
                return Err(DisplayError::InvalidParameter);
            }
        }
        
        let address = match row {
            0 => 0x00 + col,
            1 => 0x40 + col,
            2 => 0x14 + col, // 对于20x4显示
            3 => 0x54 + col, // 对于20x4显示
            _ => return Err(DisplayError::InvalidParameter),
        };
        
        self.send_command(hd44780_commands::SET_DDRAM_ADDR | address)?;
        self.cursor_pos = (col, row);
        Ok(())
    }
    
    /// 写入字符串
    pub fn write_str(&mut self, s: &str) -> Result<(), DisplayError> {
        for ch in s.chars() {
            if ch.is_ascii() {
                self.send_data(ch as u8)?;
                self.cursor_pos.0 += 1;
                
                // 处理换行
                if let Some(char_size) = self.config.char_size {
                    if self.cursor_pos.0 >= char_size.columns() {
                        self.cursor_pos.0 = 0;
                        self.cursor_pos.1 += 1;
                        if self.cursor_pos.1 >= char_size.rows() {
                            self.cursor_pos.1 = 0;
                        }
                        self.set_cursor(self.cursor_pos.0, self.cursor_pos.1)?;
                    }
                }
            }
        }
        Ok(())
    }
    
    /// 创建自定义字符
    pub fn create_char(&mut self, location: u8, charmap: &[u8; 8]) -> Result<(), DisplayError> {
        if location > 7 {
            return Err(DisplayError::InvalidParameter);
        }
        
        self.send_command(hd44780_commands::SET_CGRAM_ADDR | (location << 3))?;
        
        for &byte in charmap {
            self.send_data(byte)?;
        }
        
        // 返回到DDRAM
        self.set_cursor(self.cursor_pos.0, self.cursor_pos.1)?;
        Ok(())
    }
    
    /// 设置背光
    pub fn set_backlight(&mut self, on: bool) -> Result<(), DisplayError> {
        if let Some(backlight) = &mut self.backlight {
            if on {
                backlight.set_high().map_err(|_| DisplayError::Communication)?;
            } else {
                backlight.set_low().map_err(|_| DisplayError::Communication)?;
            }
        }
        Ok(())
    }
    
    /// 设置光标可见性
    pub fn set_cursor_visible(&mut self, visible: bool) -> Result<(), DisplayError> {
        self.cursor_on = visible;
        let mut display_control = hd44780_commands::DISPLAY_CONTROL;
        
        if self.display_on {
            display_control |= hd44780_commands::DISPLAY_ON;
        }
        if self.cursor_on {
            display_control |= hd44780_commands::CURSOR_ON;
        }
        if self.blink_on {
            display_control |= hd44780_commands::BLINK_ON;
        }
        
        self.send_command(display_control)
    }
    
    /// 设置光标闪烁
    pub fn set_cursor_blink(&mut self, blink: bool) -> Result<(), DisplayError> {
        self.blink_on = blink;
        let mut display_control = hd44780_commands::DISPLAY_CONTROL;
        
        if self.display_on {
            display_control |= hd44780_commands::DISPLAY_ON;
        }
        if self.cursor_on {
            display_control |= hd44780_commands::CURSOR_ON;
        }
        if self.blink_on {
            display_control |= hd44780_commands::BLINK_ON;
        }
        
        self.send_command(display_control)
    }
    
    /// 滚动显示
    pub fn scroll_display(&mut self, right: bool) -> Result<(), DisplayError> {
        let command = hd44780_commands::CURSOR_SHIFT | hd44780_commands::DISPLAY_MOVE;
        let command = if right {
            command | hd44780_commands::MOVE_RIGHT
        } else {
            command
        };
        self.send_command(command)
    }
}

// 实现Display trait
impl<RS, EN, D4, D5, D6, D7, BL, DELAY> Display for CharacterLcdParallel<RS, EN, D4, D5, D6, D7, BL, DELAY>
where
    RS: OutputPin,
    EN: OutputPin,
    D4: OutputPin,
    D5: OutputPin,
    D6: OutputPin,
    D7: OutputPin,
    BL: OutputPin,
    DELAY: DelayNs,
{
    type Color = BinaryColor;
    type Error = DisplayError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        self.initialize_lcd()
    }
    
    fn clear(&mut self, _color: Self::Color) -> Result<(), Self::Error> {
        self.send_command(hd44780_commands::CLEAR_DISPLAY)?;
        self.cursor_pos = (0, 0);
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), Self::Error> {
        // 字符LCD不需要刷新缓冲区
        Ok(())
    }
    
    fn set_pixel(&mut self, _point: Point, _color: Self::Color) -> Result<(), Self::Error> {
        // 字符LCD不支持像素级操作
        Err(DisplayError::NotSupported)
    }
    
    fn size(&self) -> Size {
        if let Some(char_size) = self.config.char_size {
            // 每个字符5x8像素，加上间距
            Size::new(
                char_size.columns() as u32 * 6,
                char_size.rows() as u32 * 9,
            )
        } else {
            Size::new(96, 18) // 默认16x2
        }
    }
    
    fn set_brightness(&mut self, _brightness: u8) -> Result<(), Self::Error> {
        // 字符LCD通常通过背光控制亮度
        Ok(())
    }
    
    fn get_brightness(&self) -> u8 {
        255 // 字符LCD没有亮度概念
    }
    
    fn set_display_on(&mut self, on: bool) -> Result<(), Self::Error> {
        self.display_on = on;
        let mut display_control = hd44780_commands::DISPLAY_CONTROL;
        
        if self.display_on {
            display_control |= hd44780_commands::DISPLAY_ON;
        }
        if self.cursor_on {
            display_control |= hd44780_commands::CURSOR_ON;
        }
        if self.blink_on {
            display_control |= hd44780_commands::BLINK_ON;
        }
        
        self.send_command(display_control)
    }
    
    fn is_healthy(&mut self) -> bool {
        // 简单的健康检查：尝试设置光标位置
        self.set_cursor(0, 0).is_ok()
    }
    
    fn sleep(&mut self) -> Result<(), Self::Error> {
        self.set_display_on(false)?;
        self.set_backlight(false)
    }
    
    fn wake_up(&mut self) -> Result<(), Self::Error> {
        self.set_backlight(true)?;
        self.set_display_on(true)
    }
    
    fn get_info(&self) -> DisplayInfo {
        let size = if let Some(char_size) = self.config.char_size {
            Size::new(
                char_size.columns() as u32 * 6,
                char_size.rows() as u32 * 9,
            )
        } else {
            Size::new(96, 18)
        };
        
        DisplayInfo {
            display_type: DisplayType::Lcd,
            size,
            color_depth: 1,
            max_brightness: 1,
            refresh_rate: 0, // 字符LCD没有刷新率概念
            name: "HD44780 Character LCD",
            manufacturer: "Hitachi",
            interface: InterfaceType::Parallel,
        }
    }
}

/// 图形LCD（ILI9341）- SPI接口
pub struct GraphicLcdSpi<SPI, DC, RST, CS, DELAY> {
    /// SPI接口
    spi: SPI,
    /// 数据/命令选择引脚
    dc: DC,
    /// 复位引脚
    rst: Option<RST>,
    /// 片选引脚
    cs: Option<CS>,
    /// 延时提供者
    delay: DELAY,
    /// 配置
    config: LcdConfig,
    /// 显示缓冲区（可选，用于部分更新）
    buffer: Option<Vec<u16, 76800>>, // 320x240的RGB565缓冲区
    /// 当前亮度
    brightness: u8,
    /// 显示状态
    display_on: bool,
    /// 初始化状态
    initialized: bool,
}

impl<SPI, DC, RST, CS, DELAY> GraphicLcdSpi<SPI, DC, RST, CS, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    CS: OutputPin,
    DELAY: DelayNs,
{
    /// 创建新的图形LCD实例
    pub fn new(
        spi: SPI,
        dc: DC,
        rst: Option<RST>,
        cs: Option<CS>,
        delay: DELAY,
        config: LcdConfig,
    ) -> Self {
        Self {
            spi,
            dc,
            rst,
            cs,
            delay,
            config,
            buffer: None,
            brightness: 128,
            display_on: false,
            initialized: false,
        }
    }
    
    /// 硬件复位
    fn hardware_reset(&mut self) -> Result<(), DisplayError> {
        if let Some(rst) = &mut self.rst {
            rst.set_high().map_err(|_| DisplayError::Communication)?;
            self.delay.delay_ms(5);
            rst.set_low().map_err(|_| DisplayError::Communication)?;
            self.delay.delay_ms(20);
            rst.set_high().map_err(|_| DisplayError::Communication)?;
            self.delay.delay_ms(150);
        }
        Ok(())
    }
    
    /// 选择芯片
    fn select(&mut self) -> Result<(), DisplayError> {
        if let Some(cs) = &mut self.cs {
            cs.set_low().map_err(|_| DisplayError::Communication)?;
        }
        Ok(())
    }
    
    /// 取消选择芯片
    fn deselect(&mut self) -> Result<(), DisplayError> {
        if let Some(cs) = &mut self.cs {
            cs.set_high().map_err(|_| DisplayError::Communication)?;
        }
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
    
    /// 发送命令和数据
    fn send_command_data(&mut self, command: u8, data: &[u8]) -> Result<(), DisplayError> {
        self.send_command(command)?;
        if !data.is_empty() {
            self.send_data(data)?;
        }
        Ok(())
    }
    
    /// 初始化ILI9341
    fn initialize_ili9341(&mut self) -> Result<(), DisplayError> {
        self.hardware_reset()?;
        
        // 软件复位
        self.send_command(ili9341_commands::SOFTWARE_RESET)?;
        self.delay.delay_ms(120);
        
        // 退出睡眠模式
        self.send_command(ili9341_commands::SLEEP_OUT)?;
        self.delay.delay_ms(120);
        
        // 电源控制A
        self.send_command_data(0xCB, &[0x39, 0x2C, 0x00, 0x34, 0x02])?;
        
        // 电源控制B
        self.send_command_data(0xCF, &[0x00, 0xC1, 0x30])?;
        
        // 驱动时序控制A
        self.send_command_data(0xE8, &[0x85, 0x00, 0x78])?;
        
        // 驱动时序控制B
        self.send_command_data(0xEA, &[0x00, 0x00])?;
        
        // 电源序列控制
        self.send_command_data(0xED, &[0x64, 0x03, 0x12, 0x81])?;
        
        // 泵比控制
        self.send_command_data(0xF7, &[0x20])?;
        
        // 电源控制1
        self.send_command_data(ili9341_commands::POWER_CONTROL_1, &[0x23])?;
        
        // 电源控制2
        self.send_command_data(ili9341_commands::POWER_CONTROL_2, &[0x10])?;
        
        // VCOM控制1
        self.send_command_data(ili9341_commands::VCOM_CONTROL_1, &[0x3E, 0x28])?;
        
        // VCOM控制2
        self.send_command_data(ili9341_commands::VCOM_CONTROL_2, &[0x86])?;
        
        // 内存访问控制
        self.send_command_data(ili9341_commands::MEMORY_ACCESS_CONTROL, &[0x48])?;
        
        // 像素格式设置（16位RGB565）
        self.send_command_data(ili9341_commands::PIXEL_FORMAT_SET, &[0x55])?;
        
        // 帧率控制
        self.send_command_data(ili9341_commands::FRAME_CONTROL_NORMAL, &[0x00, 0x18])?;
        
        // 显示功能控制
        self.send_command_data(ili9341_commands::DISPLAY_FUNCTION_CONTROL, &[0x08, 0x82, 0x27])?;
        
        // 3Gamma功能禁用
        self.send_command_data(0xF2, &[0x00])?;
        
        // Gamma曲线选择
        self.send_command_data(ili9341_commands::GAMMA_SET, &[0x01])?;
        
        // 正Gamma校正
        self.send_command_data(
            ili9341_commands::POSITIVE_GAMMA_CORRECTION,
            &[0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00],
        )?;
        
        // 负Gamma校正
        self.send_command_data(
            ili9341_commands::NEGATIVE_GAMMA_CORRECTION,
            &[0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F],
        )?;
        
        // 显示开启
        self.send_command(ili9341_commands::DISPLAY_ON)?;
        
        self.display_on = true;
        self.initialized = true;
        
        Ok(())
    }
    
    /// 设置显示窗口
    fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) -> Result<(), DisplayError> {
        // 设置列地址
        self.send_command_data(
            ili9341_commands::COLUMN_ADDRESS_SET,
            &[
                (x0 >> 8) as u8,
                (x0 & 0xFF) as u8,
                (x1 >> 8) as u8,
                (x1 & 0xFF) as u8,
            ],
        )?;
        
        // 设置页地址
        self.send_command_data(
            ili9341_commands::PAGE_ADDRESS_SET,
            &[
                (y0 >> 8) as u8,
                (y0 & 0xFF) as u8,
                (y1 >> 8) as u8,
                (y1 & 0xFF) as u8,
            ],
        )?;
        
        // 开始内存写入
        self.send_command(ili9341_commands::MEMORY_WRITE)?;
        
        Ok(())
    }
    
    /// 绘制像素
    pub fn draw_pixel_rgb565(&mut self, x: u16, y: u16, color: u16) -> Result<(), DisplayError> {
        if let Some(resolution) = self.config.resolution {
            if x >= resolution.width as u16 || y >= resolution.height as u16 {
                return Ok(()); // 忽略超出边界的像素
            }
        }
        
        self.set_window(x, y, x, y)?;
        self.send_data(&[(color >> 8) as u8, (color & 0xFF) as u8])
    }
    
    /// 填充矩形区域
    pub fn fill_rect_rgb565(&mut self, x: u16, y: u16, w: u16, h: u16, color: u16) -> Result<(), DisplayError> {
        if let Some(resolution) = self.config.resolution {
            if x >= resolution.width as u16 || y >= resolution.height as u16 {
                return Ok();
            }
        }
        
        let x1 = x + w - 1;
        let y1 = y + h - 1;
        
        self.set_window(x, y, x1, y1)?;
        
        let color_bytes = [(color >> 8) as u8, (color & 0xFF) as u8];
        let pixel_count = w as u32 * h as u32;
        
        // 分块发送以避免SPI缓冲区溢出
        const CHUNK_SIZE: u32 = 64; // 每次发送64个像素
        
        for _ in 0..(pixel_count / CHUNK_SIZE) {
            let mut chunk = Vec::<u8, 128>::new(); // 64像素 * 2字节
            for _ in 0..CHUNK_SIZE {
                chunk.push(color_bytes[0]).ok();
                chunk.push(color_bytes[1]).ok();
            }
            self.send_data(&chunk)?;
        }
        
        // 发送剩余像素
        let remaining = pixel_count % CHUNK_SIZE;
        if remaining > 0 {
            let mut chunk = Vec::<u8, 128>::new();
            for _ in 0..remaining {
                chunk.push(color_bytes[0]).ok();
                chunk.push(color_bytes[1]).ok();
            }
            self.send_data(&chunk)?;
        }
        
        Ok(())
    }
    
    /// 绘制文本
    pub fn draw_text_rgb565(&mut self, text: &str, x: u16, y: u16, color: u16, bg_color: Option<u16>) -> Result<(), DisplayError> {
        // 简单的文本绘制实现
        // 这里需要字体数据，简化实现
        let char_width = 6;
        let char_height = 8;
        
        for (i, ch) in text.chars().enumerate() {
            let char_x = x + (i as u16 * char_width);
            
            // 绘制字符背景（如果指定）
            if let Some(bg) = bg_color {
                self.fill_rect_rgb565(char_x, y, char_width, char_height, bg)?;
            }
            
            // 这里应该根据字体数据绘制字符
            // 简化实现：绘制一个矩形代表字符
            if ch != ' ' {
                self.fill_rect_rgb565(char_x + 1, y + 1, char_width - 2, char_height - 2, color)?;
            }
        }
        
        Ok(())
    }
}

// 实现Display trait
impl<SPI, DC, RST, CS, DELAY> Display for GraphicLcdSpi<SPI, DC, RST, CS, DELAY>
where
    SPI: SpiDevice,
    DC: OutputPin,
    RST: OutputPin,
    CS: OutputPin,
    DELAY: DelayNs,
{
    type Color = Rgb565;
    type Error = DisplayError;
    
    fn init(&mut self) -> Result<(), Self::Error> {
        match self.config.lcd_type {
            LcdType::GraphicIli9341 => self.initialize_ili9341(),
            _ => Err(DisplayError::NotSupported),
        }
    }
    
    fn clear(&mut self, color: Self::Color) -> Result<(), Self::Error> {
        if let Some(resolution) = self.config.resolution {
            let color_u16 = ((color.r() as u16) << 11) | ((color.g() as u16) << 5) | (color.b() as u16);
            self.fill_rect_rgb565(0, 0, resolution.width as u16, resolution.height as u16, color_u16)
        } else {
            Err(DisplayError::InvalidParameter)
        }
    }
    
    fn flush(&mut self) -> Result<(), Self::Error> {
        // 图形LCD通常直接写入显示，不需要刷新
        Ok(())
    }
    
    fn set_pixel(&mut self, point: Point, color: Self::Color) -> Result<(), Self::Error> {
        if point.x >= 0 && point.y >= 0 {
            let color_u16 = ((color.r() as u16) << 11) | ((color.g() as u16) << 5) | (color.b() as u16);
            self.draw_pixel_rgb565(point.x as u16, point.y as u16, color_u16)
        } else {
            Ok(())
        }
    }
    
    fn size(&self) -> Size {
        self.config.resolution.unwrap_or(Size::new(320, 240))
    }
    
    fn set_brightness(&mut self, brightness: u8) -> Result<(), Self::Error> {
        self.brightness = brightness;
        // ILI9341支持亮度控制
        self.send_command_data(ili9341_commands::WRITE_DISPLAY_BRIGHTNESS, &[brightness])
    }
    
    fn get_brightness(&self) -> u8 {
        self.brightness
    }
    
    fn set_display_on(&mut self, on: bool) -> Result<(), Self::Error> {
        self.display_on = on;
        let command = if on {
            ili9341_commands::DISPLAY_ON
        } else {
            ili9341_commands::DISPLAY_OFF
        };
        self.send_command(command)
    }
    
    fn is_healthy(&mut self) -> bool {
        // 尝试读取显示ID
        self.send_command(ili9341_commands::READ_ID1).is_ok()
    }
    
    fn sleep(&mut self) -> Result<(), Self::Error> {
        self.send_command(ili9341_commands::SLEEP_IN)
    }
    
    fn wake_up(&mut self) -> Result<(), Self::Error> {
        self.send_command(ili9341_commands::SLEEP_OUT)?;
        self.delay.delay_ms(120);
        Ok(())
    }
    
    fn get_info(&self) -> DisplayInfo {
        DisplayInfo {
            display_type: DisplayType::Lcd,
            size: self.config.resolution.unwrap_or(Size::new(320, 240)),
            color_depth: 16, // RGB565
            max_brightness: 255,
            refresh_rate: 60,
            name: "ILI9341 TFT LCD",
            manufacturer: "Ilitek",
            interface: InterfaceType::SPI,
        }
    }
}

/// LCD工具函数
pub mod lcd_utils {
    use super::*;
    
    /// 创建默认字符LCD配置
    pub fn create_character_lcd_config(size: CharacterLcdSize) -> LcdConfig {
        LcdConfig {
            lcd_type: LcdType::Character,
            char_size: Some(size),
            resolution: None,
            i2c_address: 0x27,
            backlight_control: true,
            data_width: 4,
            font_5x10: false,
            two_lines: size.rows() > 1,
        }
    }
    
    /// 创建默认ILI9341配置
    pub fn create_ili9341_config() -> LcdConfig {
        LcdConfig {
            lcd_type: LcdType::GraphicIli9341,
            char_size: None,
            resolution: Some(Size::new(320, 240)),
            i2c_address: 0x00, // SPI不使用
            backlight_control: false, // 通过PWM控制
            data_width: 8,
            font_5x10: false,
            two_lines: false,
        }
    }
    
    /// RGB888转RGB565
    pub fn rgb888_to_rgb565(r: u8, g: u8, b: u8) -> u16 {
        ((r as u16 & 0xF8) << 8) | ((g as u16 & 0xFC) << 3) | ((b as u16 & 0xF8) >> 3)
    }
    
    /// RGB565转RGB888
    pub fn rgb565_to_rgb888(color: u16) -> (u8, u8, u8) {
        let r = ((color >> 11) & 0x1F) as u8;
        let g = ((color >> 5) & 0x3F) as u8;
        let b = (color & 0x1F) as u8;
        
        // 扩展到8位
        let r = (r << 3) | (r >> 2);
        let g = (g << 2) | (g >> 4);
        let b = (b << 3) | (b >> 2);
        
        (r, g, b)
    }
    
    /// 验证LCD配置
    pub fn validate_lcd_config(config: &LcdConfig) -> Result<(), DisplayError> {
        match config.lcd_type {
            LcdType::Character => {
                if config.char_size.is_none() {
                    return Err(DisplayError::InvalidParameter);
                }
                if config.data_width != 4 && config.data_width != 8 {
                    return Err(DisplayError::InvalidParameter);
                }
            }
            LcdType::GraphicIli9341 | LcdType::GraphicSt7735 | LcdType::GraphicOther => {
                if config.resolution.is_none() {
                    return Err(DisplayError::InvalidParameter);
                }
            }
        }
        
        Ok(())
    }
    
    /// 计算字符LCD的DDRAM地址
    pub fn calculate_ddram_address(col: u8, row: u8, lcd_size: CharacterLcdSize) -> Result<u8, DisplayError> {
        if col >= lcd_size.columns() || row >= lcd_size.rows() {
            return Err(DisplayError::InvalidParameter);
        }
        
        let address = match row {
            0 => 0x00 + col,
            1 => 0x40 + col,
            2 => match lcd_size {
                CharacterLcdSize::Size20x4 => 0x14 + col,
                CharacterLcdSize::Size16x4 => 0x10 + col,
                _ => return Err(DisplayError::InvalidParameter),
            },
            3 => match lcd_size {
                CharacterLcdSize::Size20x4 => 0x54 + col,
                CharacterLcdSize::Size16x4 => 0x50 + col,
                _ => return Err(DisplayError::InvalidParameter),
            },
            _ => return Err(DisplayError::InvalidParameter),
        };
        
        Ok(address)
    }
    
    /// 创建常用自定义字符
    pub mod custom_chars {
        /// 心形字符
        pub const HEART: [u8; 8] = [
            0b00000,
            0b01010,
            0b11111,
            0b11111,
            0b11111,
            0b01110,
            0b00100,
            0b00000,
        ];
        
        /// 笑脸字符
        pub const SMILEY: [u8; 8] = [
            0b00000,
            0b01010,
            0b00000,
            0b00000,
            0b10001,
            0b01110,
            0b00000,
            0b00000,
        ];
        
        /// 箭头向上
        pub const ARROW_UP: [u8; 8] = [
            0b00100,
            0b01110,
            0b11111,
            0b00100,
            0b00100,
            0b00100,
            0b00100,
            0b00000,
        ];
        
        /// 箭头向下
        pub const ARROW_DOWN: [u8; 8] = [
            0b00100,
            0b00100,
            0b00100,
            0b00100,
            0b11111,
            0b01110,
            0b00100,
            0b00000,
        ];
        
        /// 温度计
        pub const THERMOMETER: [u8; 8] = [
            0b00100,
            0b01010,
            0b01010,
            0b01110,
            0b01110,
            0b11111,
            0b11111,
            0b01110,
        ];
        
        /// 湿度计
        pub const HUMIDITY: [u8; 8] = [
            0b00100,
            0b00100,
            0b01010,
            0b01010,
            0b10001,
            0b10001,
            0b10001,
            0b01110,
        ];
    }
}
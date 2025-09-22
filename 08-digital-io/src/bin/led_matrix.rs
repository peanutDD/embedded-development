#![no_std]
#![no_main]

//! # LED矩阵显示程序
//! 
//! 演示LED矩阵的完整功能：
//! - 8x8 LED矩阵控制
//! - 字符和图形显示
//! - 滚动文本和动画效果
//! - 亮度控制和刷新率优化

use panic_halt as _;
use cortex_m_rt::entry;
use heapless::{Vec, String};
use critical_section::Mutex;
use core::cell::RefCell;

use digital_io::{
    ShiftRegisterController, OutputConfig, DriveStrength,
    OutputMode, OutputState,
};

/// LED矩阵尺寸
const MATRIX_WIDTH: usize = 8;
const MATRIX_HEIGHT: usize = 8;
const MATRIX_SIZE: usize = MATRIX_WIDTH * MATRIX_HEIGHT;

/// 像素状态
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PixelState {
    Off,
    On,
    Dim,
    Bright,
}

/// 显示模式
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DisplayMode {
    /// 静态显示
    Static,
    /// 滚动文本
    ScrollText,
    /// 动画
    Animation,
    /// 呼吸灯效果
    Breathing,
    /// 雨滴效果
    RainDrop,
    /// 波纹效果
    Ripple,
}

/// 动画类型
#[derive(Debug, Clone, Copy)]
pub enum AnimationType {
    /// 闪烁
    Blink,
    /// 淡入淡出
    FadeInOut,
    /// 左右滚动
    ScrollHorizontal,
    /// 上下滚动
    ScrollVertical,
    /// 旋转
    Rotate,
    /// 缩放
    Scale,
}

/// 字符字体（8x8点阵）
pub struct Font8x8 {
    /// 字符数据
    char_data: &'static [[u8; 8]],
}

/// LED矩阵控制器
pub struct LedMatrixController {
    /// 移位寄存器控制器（用于行选择）
    row_controller: ShiftRegisterController,
    /// 移位寄存器控制器（用于列数据）
    col_controller: ShiftRegisterController,
    /// 显示缓冲区
    display_buffer: [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT],
    /// 亮度缓冲区
    brightness_buffer: [[u8; MATRIX_WIDTH]; MATRIX_HEIGHT],
    /// 当前扫描行
    current_row: usize,
    /// 显示配置
    display_config: DisplayConfig,
    /// 动画控制器
    animation_controller: AnimationController,
    /// 文本滚动器
    text_scroller: TextScroller,
    /// 统计信息
    stats: MatrixStats,
}

/// 显示配置
#[derive(Debug, Clone)]
pub struct DisplayConfig {
    /// 刷新率（Hz）
    refresh_rate: u16,
    /// 全局亮度（0-255）
    global_brightness: u8,
    /// 显示模式
    display_mode: DisplayMode,
    /// 是否启用PWM调光
    enable_pwm: bool,
    /// PWM频率
    pwm_frequency: u32,
}

/// 动画控制器
pub struct AnimationController {
    /// 当前动画类型
    current_animation: AnimationType,
    /// 动画帧索引
    frame_index: u16,
    /// 动画速度（ms/帧）
    frame_duration: u32,
    /// 上次更新时间
    last_update: u32,
    /// 动画循环次数
    loop_count: u16,
    /// 动画状态
    animation_state: AnimationState,
}

/// 动画状态
#[derive(Debug, Clone, Copy)]
pub struct AnimationState {
    pub position_x: i16,
    pub position_y: i16,
    pub scale: u16,
    pub rotation: u16,
    pub alpha: u8,
}

/// 文本滚动器
pub struct TextScroller {
    /// 滚动文本
    text: String<128>,
    /// 当前位置
    position: i16,
    /// 滚动速度（像素/秒）
    scroll_speed: u16,
    /// 滚动方向
    scroll_direction: ScrollDirection,
    /// 字体
    font: Font8x8,
    /// 上次更新时间
    last_update: u32,
}

/// 滚动方向
#[derive(Debug, Clone, Copy)]
pub enum ScrollDirection {
    Left,
    Right,
    Up,
    Down,
}

/// 矩阵统计
#[derive(Debug, Default)]
pub struct MatrixStats {
    pub refresh_count: u32,
    pub frame_updates: u32,
    pub animation_frames: u32,
    pub text_scrolls: u32,
    pub brightness_changes: u32,
}

/// 预定义字符字体数据
static FONT_8X8_DATA: &[[u8; 8]] = &[
    // 空格 (0x20)
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    // ! (0x21)
    [0x18, 0x3C, 0x3C, 0x18, 0x18, 0x00, 0x18, 0x00],
    // " (0x22)
    [0x36, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    // # (0x23)
    [0x36, 0x36, 0x7F, 0x36, 0x7F, 0x36, 0x36, 0x00],
    // $ (0x24)
    [0x0C, 0x3E, 0x03, 0x1E, 0x30, 0x1F, 0x0C, 0x00],
    // % (0x25)
    [0x00, 0x63, 0x33, 0x18, 0x0C, 0x66, 0x63, 0x00],
    // & (0x26)
    [0x1C, 0x36, 0x1C, 0x6E, 0x3B, 0x33, 0x6E, 0x00],
    // ' (0x27)
    [0x06, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00],
    // ( (0x28)
    [0x18, 0x0C, 0x06, 0x06, 0x06, 0x0C, 0x18, 0x00],
    // ) (0x29)
    [0x06, 0x0C, 0x18, 0x18, 0x18, 0x0C, 0x06, 0x00],
    // * (0x2A)
    [0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00],
    // + (0x2B)
    [0x00, 0x0C, 0x0C, 0x3F, 0x0C, 0x0C, 0x00, 0x00],
    // , (0x2C)
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x06, 0x00],
    // - (0x2D)
    [0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00],
    // . (0x2E)
    [0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C, 0x00],
    // / (0x2F)
    [0x60, 0x30, 0x18, 0x0C, 0x06, 0x03, 0x01, 0x00],
    // 0 (0x30)
    [0x3E, 0x63, 0x73, 0x7B, 0x6F, 0x67, 0x3E, 0x00],
    // 1 (0x31)
    [0x0C, 0x0E, 0x0C, 0x0C, 0x0C, 0x0C, 0x3F, 0x00],
    // 2 (0x32)
    [0x1E, 0x33, 0x30, 0x1C, 0x06, 0x33, 0x3F, 0x00],
    // 3 (0x33)
    [0x1E, 0x33, 0x30, 0x1C, 0x30, 0x33, 0x1E, 0x00],
    // 4 (0x34)
    [0x38, 0x3C, 0x36, 0x33, 0x7F, 0x30, 0x78, 0x00],
    // 5 (0x35)
    [0x3F, 0x03, 0x1F, 0x30, 0x30, 0x33, 0x1E, 0x00],
    // 6 (0x36)
    [0x1C, 0x06, 0x03, 0x1F, 0x33, 0x33, 0x1E, 0x00],
    // 7 (0x37)
    [0x3F, 0x33, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x00],
    // 8 (0x38)
    [0x1E, 0x33, 0x33, 0x1E, 0x33, 0x33, 0x1E, 0x00],
    // 9 (0x39)
    [0x1E, 0x33, 0x33, 0x3E, 0x30, 0x18, 0x0E, 0x00],
    // A (0x41)
    [0x0C, 0x1E, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x00],
    // B (0x42)
    [0x3F, 0x66, 0x66, 0x3E, 0x66, 0x66, 0x3F, 0x00],
    // C (0x43)
    [0x3C, 0x66, 0x03, 0x03, 0x03, 0x66, 0x3C, 0x00],
    // H (0x48)
    [0x33, 0x33, 0x33, 0x3F, 0x33, 0x33, 0x33, 0x00],
    // E (0x45)
    [0x7F, 0x46, 0x16, 0x1E, 0x16, 0x46, 0x7F, 0x00],
    // L (0x4C)
    [0x0F, 0x06, 0x06, 0x06, 0x46, 0x66, 0x7F, 0x00],
    // O (0x4F)
    [0x1C, 0x36, 0x63, 0x63, 0x63, 0x36, 0x1C, 0x00],
];

impl Font8x8 {
    /// 创建新的字体
    pub fn new() -> Self {
        Self {
            char_data: FONT_8X8_DATA,
        }
    }
    
    /// 获取字符数据
    pub fn get_char_data(&self, ch: char) -> Option<&[u8; 8]> {
        let index = match ch {
            ' ' => 0,
            '!' => 1,
            '"' => 2,
            '#' => 3,
            '$' => 4,
            '%' => 5,
            '&' => 6,
            '\'' => 7,
            '(' => 8,
            ')' => 9,
            '*' => 10,
            '+' => 11,
            ',' => 12,
            '-' => 13,
            '.' => 14,
            '/' => 15,
            '0' => 16,
            '1' => 17,
            '2' => 18,
            '3' => 19,
            '4' => 20,
            '5' => 21,
            '6' => 22,
            '7' => 23,
            '8' => 24,
            '9' => 25,
            'A' => 26,
            'B' => 27,
            'C' => 28,
            'H' => 29,
            'E' => 30,
            'L' => 31,
            'O' => 32,
            _ => return None,
        };
        
        self.char_data.get(index)
    }
}

impl Default for DisplayConfig {
    fn default() -> Self {
        Self {
            refresh_rate: 100,
            global_brightness: 128,
            display_mode: DisplayMode::Static,
            enable_pwm: true,
            pwm_frequency: 1000,
        }
    }
}

impl AnimationController {
    /// 创建新的动画控制器
    pub fn new() -> Self {
        Self {
            current_animation: AnimationType::Blink,
            frame_index: 0,
            frame_duration: 100,
            last_update: 0,
            loop_count: 0,
            animation_state: AnimationState {
                position_x: 0,
                position_y: 0,
                scale: 100,
                rotation: 0,
                alpha: 255,
            },
        }
    }
    
    /// 更新动画
    pub fn update(&mut self, timestamp: u32, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) -> bool {
        if timestamp.saturating_sub(self.last_update) < self.frame_duration {
            return false;
        }
        
        self.last_update = timestamp;
        self.frame_index = self.frame_index.wrapping_add(1);
        
        match self.current_animation {
            AnimationType::Blink => {
                self.animate_blink(buffer);
            },
            AnimationType::FadeInOut => {
                self.animate_fade_in_out(buffer);
            },
            AnimationType::ScrollHorizontal => {
                self.animate_scroll_horizontal(buffer);
            },
            AnimationType::ScrollVertical => {
                self.animate_scroll_vertical(buffer);
            },
            AnimationType::Rotate => {
                self.animate_rotate(buffer);
            },
            AnimationType::Scale => {
                self.animate_scale(buffer);
            },
        }
        
        true
    }
    
    /// 闪烁动画
    fn animate_blink(&mut self, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) {
        let on = (self.frame_index / 10) % 2 == 0;
        let state = if on { PixelState::On } else { PixelState::Off };
        
        for row in buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = state;
            }
        }
    }
    
    /// 淡入淡出动画
    fn animate_fade_in_out(&mut self, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) {
        let cycle = self.frame_index % 100;
        let brightness = if cycle < 50 {
            cycle * 5
        } else {
            (100 - cycle) * 5
        };
        
        let state = if brightness > 200 {
            PixelState::Bright
        } else if brightness > 100 {
            PixelState::On
        } else if brightness > 50 {
            PixelState::Dim
        } else {
            PixelState::Off
        };
        
        for row in buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = state;
            }
        }
    }
    
    /// 水平滚动动画
    fn animate_scroll_horizontal(&mut self, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) {
        let offset = (self.frame_index / 5) % (MATRIX_WIDTH as u16 * 2);
        
        // 清空缓冲区
        for row in buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = PixelState::Off;
            }
        }
        
        // 绘制移动的图案
        for y in 0..MATRIX_HEIGHT {
            let x = (offset as usize + y) % MATRIX_WIDTH;
            if x < MATRIX_WIDTH {
                buffer[y][x] = PixelState::On;
            }
        }
    }
    
    /// 垂直滚动动画
    fn animate_scroll_vertical(&mut self, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) {
        let offset = (self.frame_index / 5) % (MATRIX_HEIGHT as u16 * 2);
        
        // 清空缓冲区
        for row in buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = PixelState::Off;
            }
        }
        
        // 绘制移动的图案
        for x in 0..MATRIX_WIDTH {
            let y = (offset as usize + x) % MATRIX_HEIGHT;
            if y < MATRIX_HEIGHT {
                buffer[y][x] = PixelState::On;
            }
        }
    }
    
    /// 旋转动画
    fn animate_rotate(&mut self, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) {
        let angle = (self.frame_index * 5) % 360;
        
        // 清空缓冲区
        for row in buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = PixelState::Off;
            }
        }
        
        // 绘制旋转的图案（简化的旋转效果）
        let center_x = MATRIX_WIDTH / 2;
        let center_y = MATRIX_HEIGHT / 2;
        
        for i in 0..4 {
            let radius = 2;
            let x = center_x as i32 + (radius as f32 * ((angle + i * 90) as f32 * 3.14159 / 180.0).cos()) as i32;
            let y = center_y as i32 + (radius as f32 * ((angle + i * 90) as f32 * 3.14159 / 180.0).sin()) as i32;
            
            if x >= 0 && x < MATRIX_WIDTH as i32 && y >= 0 && y < MATRIX_HEIGHT as i32 {
                buffer[y as usize][x as usize] = PixelState::On;
            }
        }
    }
    
    /// 缩放动画
    fn animate_scale(&mut self, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) {
        let scale = ((self.frame_index / 5) % 20) as i32;
        let size = if scale < 10 { scale } else { 20 - scale };
        
        // 清空缓冲区
        for row in buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = PixelState::Off;
            }
        }
        
        // 绘制缩放的矩形
        let center_x = MATRIX_WIDTH / 2;
        let center_y = MATRIX_HEIGHT / 2;
        
        for y in 0..MATRIX_HEIGHT {
            for x in 0..MATRIX_WIDTH {
                let dx = (x as i32 - center_x as i32).abs();
                let dy = (y as i32 - center_y as i32).abs();
                
                if dx <= size && dy <= size {
                    buffer[y][x] = PixelState::On;
                }
            }
        }
    }
    
    /// 设置动画类型
    pub fn set_animation(&mut self, animation: AnimationType) {
        self.current_animation = animation;
        self.frame_index = 0;
    }
    
    /// 设置动画速度
    pub fn set_speed(&mut self, frame_duration: u32) {
        self.frame_duration = frame_duration;
    }
}

impl TextScroller {
    /// 创建新的文本滚动器
    pub fn new() -> Self {
        Self {
            text: String::new(),
            position: 0,
            scroll_speed: 50,
            scroll_direction: ScrollDirection::Left,
            font: Font8x8::new(),
            last_update: 0,
        }
    }
    
    /// 设置滚动文本
    pub fn set_text(&mut self, text: &str) -> Result<(), ()> {
        self.text.clear();
        self.text.push_str(text).map_err(|_| ())?;
        self.position = MATRIX_WIDTH as i16;
        Ok(())
    }
    
    /// 更新滚动
    pub fn update(&mut self, timestamp: u32, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) -> bool {
        let update_interval = 1000 / self.scroll_speed as u32;
        if timestamp.saturating_sub(self.last_update) < update_interval {
            return false;
        }
        
        self.last_update = timestamp;
        
        match self.scroll_direction {
            ScrollDirection::Left => {
                self.position -= 1;
                if self.position < -(self.text.len() as i16 * 8) {
                    self.position = MATRIX_WIDTH as i16;
                }
            },
            ScrollDirection::Right => {
                self.position += 1;
                if self.position > MATRIX_WIDTH as i16 {
                    self.position = -(self.text.len() as i16 * 8);
                }
            },
            _ => {}
        }
        
        self.render_text(buffer);
        true
    }
    
    /// 渲染文本
    fn render_text(&self, buffer: &mut [[PixelState; MATRIX_WIDTH]; MATRIX_HEIGHT]) {
        // 清空缓冲区
        for row in buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = PixelState::Off;
            }
        }
        
        // 渲染每个字符
        for (char_index, ch) in self.text.chars().enumerate() {
            let char_x = self.position + (char_index as i16 * 8);
            
            if let Some(char_data) = self.font.get_char_data(ch) {
                for (row, &row_data) in char_data.iter().enumerate() {
                    if row < MATRIX_HEIGHT {
                        for col in 0..8 {
                            let pixel_x = char_x + col as i16;
                            if pixel_x >= 0 && pixel_x < MATRIX_WIDTH as i16 {
                                if (row_data >> (7 - col)) & 1 != 0 {
                                    buffer[row][pixel_x as usize] = PixelState::On;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    
    /// 设置滚动速度
    pub fn set_speed(&mut self, speed: u16) {
        self.scroll_speed = speed;
    }
    
    /// 设置滚动方向
    pub fn set_direction(&mut self, direction: ScrollDirection) {
        self.scroll_direction = direction;
    }
}

impl LedMatrixController {
    /// 创建新的LED矩阵控制器
    pub fn new() -> Self {
        let row_config = OutputConfig {
            mode: OutputMode::PushPull,
            drive_strength: DriveStrength::High,
            slew_rate_limit: false,
            open_drain: false,
        };
        
        let col_config = OutputConfig {
            mode: OutputMode::PushPull,
            drive_strength: DriveStrength::Medium,
            slew_rate_limit: true,
            open_drain: false,
        };
        
        Self {
            row_controller: ShiftRegisterController::new(8, row_config),
            col_controller: ShiftRegisterController::new(8, col_config),
            display_buffer: [[PixelState::Off; MATRIX_WIDTH]; MATRIX_HEIGHT],
            brightness_buffer: [[128; MATRIX_WIDTH]; MATRIX_HEIGHT],
            current_row: 0,
            display_config: DisplayConfig::default(),
            animation_controller: AnimationController::new(),
            text_scroller: TextScroller::new(),
            stats: MatrixStats::default(),
        }
    }
    
    /// 更新显示
    pub fn update(&mut self, timestamp: u32) -> Result<(), ()> {
        match self.display_config.display_mode {
            DisplayMode::Static => {
                // 静态显示不需要更新
            },
            DisplayMode::ScrollText => {
                if self.text_scroller.update(timestamp, &mut self.display_buffer) {
                    self.stats.text_scrolls += 1;
                }
            },
            DisplayMode::Animation => {
                if self.animation_controller.update(timestamp, &mut self.display_buffer) {
                    self.stats.animation_frames += 1;
                }
            },
            DisplayMode::Breathing => {
                self.update_breathing_effect(timestamp);
            },
            DisplayMode::RainDrop => {
                self.update_raindrop_effect(timestamp);
            },
            DisplayMode::Ripple => {
                self.update_ripple_effect(timestamp);
            },
        }
        
        self.stats.frame_updates += 1;
        Ok(())
    }
    
    /// 刷新显示（扫描一行）
    pub fn refresh(&mut self) -> Result<(), ()> {
        // 选择当前行
        let mut row_data = [false; 8];
        row_data[self.current_row] = true;
        self.row_controller.shift_out(&row_data)?;
        
        // 输出列数据
        let mut col_data = [false; 8];
        for (col, &pixel) in self.display_buffer[self.current_row].iter().enumerate() {
            col_data[col] = match pixel {
                PixelState::Off => false,
                PixelState::On => true,
                PixelState::Dim => {
                    // 简单的PWM模拟
                    (self.stats.refresh_count % 4) < 1
                },
                PixelState::Bright => true,
            };
        }
        self.col_controller.shift_out(&col_data)?;
        
        // 移动到下一行
        self.current_row = (self.current_row + 1) % MATRIX_HEIGHT;
        self.stats.refresh_count += 1;
        
        Ok(())
    }
    
    /// 设置像素
    pub fn set_pixel(&mut self, x: usize, y: usize, state: PixelState) -> Result<(), ()> {
        if x >= MATRIX_WIDTH || y >= MATRIX_HEIGHT {
            return Err(());
        }
        
        self.display_buffer[y][x] = state;
        Ok(())
    }
    
    /// 获取像素
    pub fn get_pixel(&self, x: usize, y: usize) -> Option<PixelState> {
        if x >= MATRIX_WIDTH || y >= MATRIX_HEIGHT {
            return None;
        }
        
        Some(self.display_buffer[y][x])
    }
    
    /// 清空显示
    pub fn clear(&mut self) {
        for row in self.display_buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = PixelState::Off;
            }
        }
    }
    
    /// 填充显示
    pub fn fill(&mut self, state: PixelState) {
        for row in self.display_buffer.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = state;
            }
        }
    }
    
    /// 绘制字符
    pub fn draw_char(&mut self, x: usize, y: usize, ch: char) -> Result<(), ()> {
        let font = Font8x8::new();
        if let Some(char_data) = font.get_char_data(ch) {
            for (row, &row_data) in char_data.iter().enumerate() {
                if y + row < MATRIX_HEIGHT {
                    for col in 0..8 {
                        if x + col < MATRIX_WIDTH {
                            let pixel_state = if (row_data >> (7 - col)) & 1 != 0 {
                                PixelState::On
                            } else {
                                PixelState::Off
                            };
                            self.display_buffer[y + row][x + col] = pixel_state;
                        }
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// 绘制文本
    pub fn draw_text(&mut self, x: usize, y: usize, text: &str) -> Result<(), ()> {
        for (i, ch) in text.chars().enumerate() {
            let char_x = x + i * 8;
            if char_x >= MATRIX_WIDTH {
                break;
            }
            self.draw_char(char_x, y, ch)?;
        }
        
        Ok(())
    }
    
    /// 设置显示模式
    pub fn set_display_mode(&mut self, mode: DisplayMode) {
        self.display_config.display_mode = mode;
    }
    
    /// 设置全局亮度
    pub fn set_brightness(&mut self, brightness: u8) {
        self.display_config.global_brightness = brightness;
        self.stats.brightness_changes += 1;
    }
    
    /// 设置动画
    pub fn set_animation(&mut self, animation: AnimationType) {
        self.animation_controller.set_animation(animation);
        self.display_config.display_mode = DisplayMode::Animation;
    }
    
    /// 设置滚动文本
    pub fn set_scroll_text(&mut self, text: &str) -> Result<(), ()> {
        self.text_scroller.set_text(text)?;
        self.display_config.display_mode = DisplayMode::ScrollText;
        Ok(())
    }
    
    /// 更新呼吸灯效果
    fn update_breathing_effect(&mut self, timestamp: u32) {
        let cycle = (timestamp / 50) % 100;
        let brightness = if cycle < 50 {
            cycle * 5
        } else {
            (100 - cycle) * 5
        };
        
        let state = if brightness > 200 {
            PixelState::Bright
        } else if brightness > 100 {
            PixelState::On
        } else if brightness > 50 {
            PixelState::Dim
        } else {
            PixelState::Off
        };
        
        self.fill(state);
    }
    
    /// 更新雨滴效果
    fn update_raindrop_effect(&mut self, timestamp: u32) {
        // 简单的雨滴效果
        if timestamp % 100 == 0 {
            // 向下移动所有像素
            for y in (1..MATRIX_HEIGHT).rev() {
                for x in 0..MATRIX_WIDTH {
                    self.display_buffer[y][x] = self.display_buffer[y - 1][x];
                }
            }
            
            // 在顶部随机生成新的雨滴
            for x in 0..MATRIX_WIDTH {
                self.display_buffer[0][x] = if (timestamp + x as u32) % 5 == 0 {
                    PixelState::On
                } else {
                    PixelState::Off
                };
            }
        }
    }
    
    /// 更新波纹效果
    fn update_ripple_effect(&mut self, timestamp: u32) {
        let center_x = MATRIX_WIDTH / 2;
        let center_y = MATRIX_HEIGHT / 2;
        let radius = ((timestamp / 100) % 10) as i32;
        
        self.clear();
        
        for y in 0..MATRIX_HEIGHT {
            for x in 0..MATRIX_WIDTH {
                let dx = x as i32 - center_x as i32;
                let dy = y as i32 - center_y as i32;
                let distance = ((dx * dx + dy * dy) as f32).sqrt() as i32;
                
                if (distance - radius).abs() <= 1 {
                    self.display_buffer[y][x] = PixelState::On;
                }
            }
        }
    }
    
    /// 获取统计信息
    pub fn get_stats(&self) -> &MatrixStats {
        &self.stats
    }
    
    /// 获取详细统计
    pub fn get_detailed_stats(&self) -> DetailedMatrixStats {
        DetailedMatrixStats {
            matrix: self.stats,
            current_row: self.current_row,
            display_mode: self.display_config.display_mode,
            global_brightness: self.display_config.global_brightness,
            animation_frame: self.animation_controller.frame_index,
            text_position: self.text_scroller.position,
        }
    }
}

/// 详细矩阵统计
#[derive(Debug)]
pub struct DetailedMatrixStats {
    pub matrix: MatrixStats,
    pub current_row: usize,
    pub display_mode: DisplayMode,
    pub global_brightness: u8,
    pub animation_frame: u16,
    pub text_position: i16,
}

/// 全局LED矩阵控制器
static LED_MATRIX_CONTROLLER: Mutex<RefCell<Option<LedMatrixController>>> = 
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 初始化LED矩阵控制器
    let mut controller = LedMatrixController::new();
    
    // 设置初始显示内容
    let _ = controller.draw_text(0, 0, "HELLO");
    let _ = controller.set_scroll_text("HELLO WORLD!");
    
    // 存储到全局变量
    critical_section::with(|cs| {
        LED_MATRIX_CONTROLLER.borrow(cs).replace(Some(controller));
    });
    
    let mut timestamp = 0u32;
    let mut mode_counter = 0u32;
    
    loop {
        timestamp = timestamp.wrapping_add(1);
        
        // 处理LED矩阵
        critical_section::with(|cs| {
            if let Some(ref mut controller) = LED_MATRIX_CONTROLLER.borrow(cs).borrow_mut().as_mut() {
                // 更新显示内容
                let _ = controller.update(timestamp);
                
                // 刷新显示（高频率）
                if timestamp % 10 == 0 {
                    let _ = controller.refresh();
                }
                
                // 切换显示模式
                if timestamp % 5000 == 0 {
                    mode_counter += 1;
                    match mode_counter % 6 {
                        0 => {
                            controller.set_display_mode(DisplayMode::Static);
                            controller.clear();
                            let _ = controller.draw_text(0, 0, "STATIC");
                        },
                        1 => {
                            let _ = controller.set_scroll_text("SCROLLING TEXT DEMO");
                        },
                        2 => {
                            controller.set_animation(AnimationType::Blink);
                        },
                        3 => {
                            controller.set_animation(AnimationType::FadeInOut);
                        },
                        4 => {
                            controller.set_display_mode(DisplayMode::Breathing);
                        },
                        5 => {
                            controller.set_display_mode(DisplayMode::RainDrop);
                        },
                        _ => {}
                    }
                }
                
                // 每2000次循环输出状态
                if timestamp % 2000 == 0 {
                    let stats = controller.get_detailed_stats();
                    
                    // 在实际应用中，这里可以通过串口输出状态
                    // 例如：显示刷新率、当前模式、动画帧数等
                }
            }
        });
        
        // 简单延时
        for _ in 0..100 {
            cortex_m::asm::nop();
        }
    }
}
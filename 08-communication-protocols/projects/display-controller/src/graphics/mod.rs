//! 图形处理模块
//!
//! 本模块提供图形渲染、字体管理和图像处理功能，支持多种显示设备。
//! 包含2D图形绘制、文本渲染、图像缩放和颜色转换等功能。
//!
//! # 模块结构
//! - `renderer` - 图形渲染器
//! - `fonts` - 字体管理
//!
//! # 特性
//! - 高效的2D图形渲染
//! - 多种字体支持
//! - 图像处理和转换
//! - 颜色空间转换
//! - 抗锯齿和滤波
//! - 内存优化的图形操作

pub mod renderer;
pub mod fonts;

use embedded_graphics::{
    pixelcolor::{BinaryColor, Rgb565, Rgb888},
    prelude::*,
    primitives::{Circle, Line, Rectangle, Triangle},
};
use heapless::{Vec, String};

/// 图形错误类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GraphicsError {
    /// 内存不足
    OutOfMemory,
    /// 无效参数
    InvalidParameter,
    /// 不支持的操作
    NotSupported,
    /// 字体错误
    FontError,
    /// 渲染错误
    RenderError,
    /// 颜色转换错误
    ColorConversionError,
}

/// 颜色格式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ColorFormat {
    /// 单色（1位）
    Monochrome,
    /// 灰度（8位）
    Grayscale8,
    /// RGB565（16位）
    Rgb565,
    /// RGB888（24位）
    Rgb888,
    /// ARGB8888（32位，带透明度）
    Argb8888,
}

impl ColorFormat {
    /// 获取每像素位数
    pub fn bits_per_pixel(&self) -> u8 {
        match self {
            ColorFormat::Monochrome => 1,
            ColorFormat::Grayscale8 => 8,
            ColorFormat::Rgb565 => 16,
            ColorFormat::Rgb888 => 24,
            ColorFormat::Argb8888 => 32,
        }
    }
    
    /// 获取每像素字节数
    pub fn bytes_per_pixel(&self) -> u8 {
        (self.bits_per_pixel() + 7) / 8
    }
}

/// 通用颜色类型
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
    pub a: u8, // 透明度
}

impl Color {
    /// 创建新颜色
    pub const fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self { r, g, b, a }
    }
    
    /// 创建RGB颜色（不透明）
    pub const fn rgb(r: u8, g: u8, b: u8) -> Self {
        Self::new(r, g, b, 255)
    }
    
    /// 创建灰度颜色
    pub const fn gray(value: u8) -> Self {
        Self::rgb(value, value, value)
    }
    
    /// 黑色
    pub const BLACK: Self = Self::rgb(0, 0, 0);
    /// 白色
    pub const WHITE: Self = Self::rgb(255, 255, 255);
    /// 红色
    pub const RED: Self = Self::rgb(255, 0, 0);
    /// 绿色
    pub const GREEN: Self = Self::rgb(0, 255, 0);
    /// 蓝色
    pub const BLUE: Self = Self::rgb(0, 0, 255);
    /// 黄色
    pub const YELLOW: Self = Self::rgb(255, 255, 0);
    /// 青色
    pub const CYAN: Self = Self::rgb(0, 255, 255);
    /// 洋红色
    pub const MAGENTA: Self = Self::rgb(255, 0, 255);
    /// 透明
    pub const TRANSPARENT: Self = Self::new(0, 0, 0, 0);
    
    /// 转换为RGB565
    pub fn to_rgb565(&self) -> u16 {
        let r = (self.r >> 3) as u16;
        let g = (self.g >> 2) as u16;
        let b = (self.b >> 3) as u16;
        (r << 11) | (g << 5) | b
    }
    
    /// 从RGB565创建
    pub fn from_rgb565(rgb565: u16) -> Self {
        let r = ((rgb565 >> 11) & 0x1F) as u8;
        let g = ((rgb565 >> 5) & 0x3F) as u8;
        let b = (rgb565 & 0x1F) as u8;
        
        Self::rgb(
            (r << 3) | (r >> 2),
            (g << 2) | (g >> 4),
            (b << 3) | (b >> 2),
        )
    }
    
    /// 转换为灰度
    pub fn to_grayscale(&self) -> u8 {
        // 使用标准灰度转换公式
        ((self.r as u16 * 299 + self.g as u16 * 587 + self.b as u16 * 114) / 1000) as u8
    }
    
    /// 转换为单色（黑白）
    pub fn to_monochrome(&self, threshold: u8) -> bool {
        self.to_grayscale() >= threshold
    }
    
    /// 混合两种颜色
    pub fn blend(&self, other: &Color, alpha: u8) -> Color {
        let inv_alpha = 255 - alpha;
        Color::new(
            ((self.r as u16 * inv_alpha as u16 + other.r as u16 * alpha as u16) / 255) as u8,
            ((self.g as u16 * inv_alpha as u16 + other.g as u16 * alpha as u16) / 255) as u8,
            ((self.b as u16 * inv_alpha as u16 + other.b as u16 * alpha as u16) / 255) as u8,
            self.a,
        )
    }
}

/// 从embedded_graphics颜色类型转换
impl From<BinaryColor> for Color {
    fn from(color: BinaryColor) -> Self {
        match color {
            BinaryColor::Off => Color::BLACK,
            BinaryColor::On => Color::WHITE,
        }
    }
}

impl From<Rgb565> for Color {
    fn from(color: Rgb565) -> Self {
        Color::from_rgb565(color.into_storage())
    }
}

impl From<Rgb888> for Color {
    fn from(color: Rgb888) -> Self {
        let rgb = color.into_storage();
        Color::rgb(
            ((rgb >> 16) & 0xFF) as u8,
            ((rgb >> 8) & 0xFF) as u8,
            (rgb & 0xFF) as u8,
        )
    }
}

/// 转换为embedded_graphics颜色类型
impl From<Color> for BinaryColor {
    fn from(color: Color) -> Self {
        if color.to_monochrome(128) {
            BinaryColor::On
        } else {
            BinaryColor::Off
        }
    }
}

impl From<Color> for Rgb565 {
    fn from(color: Color) -> Self {
        Rgb565::new(color.r >> 3, color.g >> 2, color.b >> 3)
    }
}

impl From<Color> for Rgb888 {
    fn from(color: Color) -> Self {
        Rgb888::new(color.r, color.g, color.b)
    }
}

/// 绘制风格
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrawStyle {
    /// 填充
    Fill,
    /// 描边
    Stroke,
    /// 填充+描边
    FillStroke,
}

/// 线条样式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LineStyle {
    /// 实线
    Solid,
    /// 虚线
    Dashed,
    /// 点线
    Dotted,
    /// 点划线
    DashDot,
}

/// 文本对齐方式
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TextAlign {
    /// 左对齐
    Left,
    /// 居中对齐
    Center,
    /// 右对齐
    Right,
    /// 顶部对齐
    Top,
    /// 中间对齐
    Middle,
    /// 底部对齐
    Bottom,
}

/// 图形绘制参数
#[derive(Debug, Clone)]
pub struct DrawParams {
    /// 填充颜色
    pub fill_color: Option<Color>,
    /// 描边颜色
    pub stroke_color: Option<Color>,
    /// 线条宽度
    pub stroke_width: u8,
    /// 线条样式
    pub line_style: LineStyle,
    /// 绘制风格
    pub draw_style: DrawStyle,
    /// 抗锯齿
    pub anti_alias: bool,
}

impl Default for DrawParams {
    fn default() -> Self {
        Self {
            fill_color: Some(Color::WHITE),
            stroke_color: Some(Color::BLACK),
            stroke_width: 1,
            line_style: LineStyle::Solid,
            draw_style: DrawStyle::Fill,
            anti_alias: false,
        }
    }
}

/// 文本绘制参数
#[derive(Debug, Clone)]
pub struct TextParams {
    /// 字体大小
    pub font_size: u8,
    /// 文本颜色
    pub color: Color,
    /// 背景颜色（可选）
    pub background_color: Option<Color>,
    /// 水平对齐
    pub h_align: TextAlign,
    /// 垂直对齐
    pub v_align: TextAlign,
    /// 行间距
    pub line_spacing: u8,
    /// 字符间距
    pub char_spacing: u8,
    /// 是否加粗
    pub bold: bool,
    /// 是否斜体
    pub italic: bool,
}

impl Default for TextParams {
    fn default() -> Self {
        Self {
            font_size: 12,
            color: Color::BLACK,
            background_color: None,
            h_align: TextAlign::Left,
            v_align: TextAlign::Top,
            line_spacing: 2,
            char_spacing: 1,
            bold: false,
            italic: false,
        }
    }
}

/// 图像信息
#[derive(Debug, Clone)]
pub struct ImageInfo {
    /// 宽度
    pub width: u16,
    /// 高度
    pub height: u16,
    /// 颜色格式
    pub format: ColorFormat,
    /// 数据大小（字节）
    pub data_size: usize,
}

impl ImageInfo {
    /// 创建新的图像信息
    pub fn new(width: u16, height: u16, format: ColorFormat) -> Self {
        let data_size = (width as usize * height as usize * format.bytes_per_pixel() as usize);
        Self {
            width,
            height,
            format,
            data_size,
        }
    }
    
    /// 计算像素索引
    pub fn pixel_index(&self, x: u16, y: u16) -> Option<usize> {
        if x < self.width && y < self.height {
            Some((y as usize * self.width as usize + x as usize) * self.format.bytes_per_pixel() as usize)
        } else {
            None
        }
    }
}

/// 图像数据
pub struct ImageData {
    /// 图像信息
    pub info: ImageInfo,
    /// 像素数据
    pub data: Vec<u8, 65536>, // 最大64KB图像数据
}

impl ImageData {
    /// 创建新的图像数据
    pub fn new(info: ImageInfo) -> Result<Self, GraphicsError> {
        let mut data = Vec::new();
        data.resize(info.data_size, 0)
            .map_err(|_| GraphicsError::OutOfMemory)?;
        
        Ok(Self { info, data })
    }
    
    /// 获取像素颜色
    pub fn get_pixel(&self, x: u16, y: u16) -> Result<Color, GraphicsError> {
        let index = self.info.pixel_index(x, y)
            .ok_or(GraphicsError::InvalidParameter)?;
        
        match self.info.format {
            ColorFormat::Monochrome => {
                let byte_index = index / 8;
                let bit_index = index % 8;
                if byte_index < self.data.len() {
                    let bit = (self.data[byte_index] >> (7 - bit_index)) & 1;
                    Ok(if bit == 1 { Color::WHITE } else { Color::BLACK })
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Grayscale8 => {
                if index < self.data.len() {
                    Ok(Color::gray(self.data[index]))
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Rgb565 => {
                if index + 1 < self.data.len() {
                    let rgb565 = (self.data[index] as u16) << 8 | self.data[index + 1] as u16;
                    Ok(Color::from_rgb565(rgb565))
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Rgb888 => {
                if index + 2 < self.data.len() {
                    Ok(Color::rgb(
                        self.data[index],
                        self.data[index + 1],
                        self.data[index + 2],
                    ))
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Argb8888 => {
                if index + 3 < self.data.len() {
                    Ok(Color::new(
                        self.data[index + 1],
                        self.data[index + 2],
                        self.data[index + 3],
                        self.data[index],
                    ))
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
        }
    }
    
    /// 设置像素颜色
    pub fn set_pixel(&mut self, x: u16, y: u16, color: Color) -> Result<(), GraphicsError> {
        let index = self.info.pixel_index(x, y)
            .ok_or(GraphicsError::InvalidParameter)?;
        
        match self.info.format {
            ColorFormat::Monochrome => {
                let byte_index = index / 8;
                let bit_index = index % 8;
                if byte_index < self.data.len() {
                    if color.to_monochrome(128) {
                        self.data[byte_index] |= 1 << (7 - bit_index);
                    } else {
                        self.data[byte_index] &= !(1 << (7 - bit_index));
                    }
                    Ok(())
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Grayscale8 => {
                if index < self.data.len() {
                    self.data[index] = color.to_grayscale();
                    Ok(())
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Rgb565 => {
                if index + 1 < self.data.len() {
                    let rgb565 = color.to_rgb565();
                    self.data[index] = (rgb565 >> 8) as u8;
                    self.data[index + 1] = rgb565 as u8;
                    Ok(())
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Rgb888 => {
                if index + 2 < self.data.len() {
                    self.data[index] = color.r;
                    self.data[index + 1] = color.g;
                    self.data[index + 2] = color.b;
                    Ok(())
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
            ColorFormat::Argb8888 => {
                if index + 3 < self.data.len() {
                    self.data[index] = color.a;
                    self.data[index + 1] = color.r;
                    self.data[index + 2] = color.g;
                    self.data[index + 3] = color.b;
                    Ok(())
                } else {
                    Err(GraphicsError::InvalidParameter)
                }
            }
        }
    }
    
    /// 清除图像（填充指定颜色）
    pub fn clear(&mut self, color: Color) {
        for y in 0..self.info.height {
            for x in 0..self.info.width {
                let _ = self.set_pixel(x, y, color);
            }
        }
    }
}

/// 图形工具函数
pub mod graphics_utils {
    use super::*;
    
    /// 计算两点之间的距离
    pub fn distance(p1: Point, p2: Point) -> f32 {
        let dx = (p2.x - p1.x) as f32;
        let dy = (p2.y - p1.y) as f32;
        (dx * dx + dy * dy).sqrt()
    }
    
    /// 计算点是否在矩形内
    pub fn point_in_rect(point: Point, rect: Rectangle) -> bool {
        point.x >= rect.top_left.x
            && point.x < rect.top_left.x + rect.size.width as i32
            && point.y >= rect.top_left.y
            && point.y < rect.top_left.y + rect.size.height as i32
    }
    
    /// 计算点是否在圆内
    pub fn point_in_circle(point: Point, circle: Circle) -> bool {
        let center = circle.center();
        let radius = circle.diameter() as f32 / 2.0;
        distance(point, center) <= radius
    }
    
    /// 线性插值
    pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }
    
    /// 颜色线性插值
    pub fn lerp_color(a: Color, b: Color, t: f32) -> Color {
        let t = t.clamp(0.0, 1.0);
        Color::new(
            (lerp(a.r as f32, b.r as f32, t)) as u8,
            (lerp(a.g as f32, b.g as f32, t)) as u8,
            (lerp(a.b as f32, b.b as f32, t)) as u8,
            (lerp(a.a as f32, b.a as f32, t)) as u8,
        )
    }
    
    /// 计算矩形的交集
    pub fn rect_intersection(rect1: Rectangle, rect2: Rectangle) -> Option<Rectangle> {
        let left = rect1.top_left.x.max(rect2.top_left.x);
        let top = rect1.top_left.y.max(rect2.top_left.y);
        let right = (rect1.top_left.x + rect1.size.width as i32)
            .min(rect2.top_left.x + rect2.size.width as i32);
        let bottom = (rect1.top_left.y + rect1.size.height as i32)
            .min(rect2.top_left.y + rect2.size.height as i32);
        
        if left < right && top < bottom {
            Some(Rectangle::new(
                Point::new(left, top),
                Size::new((right - left) as u32, (bottom - top) as u32),
            ))
        } else {
            None
        }
    }
    
    /// 计算矩形的并集
    pub fn rect_union(rect1: Rectangle, rect2: Rectangle) -> Rectangle {
        let left = rect1.top_left.x.min(rect2.top_left.x);
        let top = rect1.top_left.y.min(rect2.top_left.y);
        let right = (rect1.top_left.x + rect1.size.width as i32)
            .max(rect2.top_left.x + rect2.size.width as i32);
        let bottom = (rect1.top_left.y + rect1.size.height as i32)
            .max(rect2.top_left.y + rect2.size.height as i32);
        
        Rectangle::new(
            Point::new(left, top),
            Size::new((right - left) as u32, (bottom - top) as u32),
        )
    }
    
    /// 缩放尺寸（保持宽高比）
    pub fn scale_size_keep_aspect(original: Size, target: Size) -> Size {
        let scale_x = target.width as f32 / original.width as f32;
        let scale_y = target.height as f32 / original.height as f32;
        let scale = scale_x.min(scale_y);
        
        Size::new(
            (original.width as f32 * scale) as u32,
            (original.height as f32 * scale) as u32,
        )
    }
    
    /// 计算文本边界框
    pub fn calculate_text_bounds(text: &str, font_size: u8, char_spacing: u8) -> Size {
        let char_width = font_size as u32 * 6 / 10; // 假设字符宽度为字体大小的0.6倍
        let char_height = font_size as u32;
        
        let lines: Vec<&str, 32> = text.split('\n').collect();
        let max_line_length = lines.iter()
            .map(|line| line.len())
            .max()
            .unwrap_or(0);
        
        let width = max_line_length as u32 * (char_width + char_spacing as u32);
        let height = lines.len() as u32 * char_height;
        
        Size::new(width, height)
    }
    
    /// 颜色格式转换
    pub fn convert_color_format(
        src_data: &[u8],
        src_format: ColorFormat,
        dst_format: ColorFormat,
        width: u16,
        height: u16,
    ) -> Result<Vec<u8, 65536>, GraphicsError> {
        let mut dst_data = Vec::new();
        let dst_size = width as usize * height as usize * dst_format.bytes_per_pixel() as usize;
        dst_data.resize(dst_size, 0)
            .map_err(|_| GraphicsError::OutOfMemory)?;
        
        for y in 0..height {
            for x in 0..width {
                // 从源格式读取颜色
                let src_color = match src_format {
                    ColorFormat::Monochrome => {
                        let pixel_index = y as usize * width as usize + x as usize;
                        let byte_index = pixel_index / 8;
                        let bit_index = pixel_index % 8;
                        if byte_index < src_data.len() {
                            let bit = (src_data[byte_index] >> (7 - bit_index)) & 1;
                            if bit == 1 { Color::WHITE } else { Color::BLACK }
                        } else {
                            Color::BLACK
                        }
                    }
                    ColorFormat::Grayscale8 => {
                        let index = (y as usize * width as usize + x as usize);
                        if index < src_data.len() {
                            Color::gray(src_data[index])
                        } else {
                            Color::BLACK
                        }
                    }
                    ColorFormat::Rgb565 => {
                        let index = (y as usize * width as usize + x as usize) * 2;
                        if index + 1 < src_data.len() {
                            let rgb565 = (src_data[index] as u16) << 8 | src_data[index + 1] as u16;
                            Color::from_rgb565(rgb565)
                        } else {
                            Color::BLACK
                        }
                    }
                    ColorFormat::Rgb888 => {
                        let index = (y as usize * width as usize + x as usize) * 3;
                        if index + 2 < src_data.len() {
                            Color::rgb(src_data[index], src_data[index + 1], src_data[index + 2])
                        } else {
                            Color::BLACK
                        }
                    }
                    ColorFormat::Argb8888 => {
                        let index = (y as usize * width as usize + x as usize) * 4;
                        if index + 3 < src_data.len() {
                            Color::new(
                                src_data[index + 1],
                                src_data[index + 2],
                                src_data[index + 3],
                                src_data[index],
                            )
                        } else {
                            Color::BLACK
                        }
                    }
                };
                
                // 写入目标格式
                match dst_format {
                    ColorFormat::Monochrome => {
                        let pixel_index = y as usize * width as usize + x as usize;
                        let byte_index = pixel_index / 8;
                        let bit_index = pixel_index % 8;
                        if byte_index < dst_data.len() {
                            if src_color.to_monochrome(128) {
                                dst_data[byte_index] |= 1 << (7 - bit_index);
                            }
                        }
                    }
                    ColorFormat::Grayscale8 => {
                        let index = y as usize * width as usize + x as usize;
                        if index < dst_data.len() {
                            dst_data[index] = src_color.to_grayscale();
                        }
                    }
                    ColorFormat::Rgb565 => {
                        let index = (y as usize * width as usize + x as usize) * 2;
                        if index + 1 < dst_data.len() {
                            let rgb565 = src_color.to_rgb565();
                            dst_data[index] = (rgb565 >> 8) as u8;
                            dst_data[index + 1] = rgb565 as u8;
                        }
                    }
                    ColorFormat::Rgb888 => {
                        let index = (y as usize * width as usize + x as usize) * 3;
                        if index + 2 < dst_data.len() {
                            dst_data[index] = src_color.r;
                            dst_data[index + 1] = src_color.g;
                            dst_data[index + 2] = src_color.b;
                        }
                    }
                    ColorFormat::Argb8888 => {
                        let index = (y as usize * width as usize + x as usize) * 4;
                        if index + 3 < dst_data.len() {
                            dst_data[index] = src_color.a;
                            dst_data[index + 1] = src_color.r;
                            dst_data[index + 2] = src_color.g;
                            dst_data[index + 3] = src_color.b;
                        }
                    }
                }
            }
        }
        
        Ok(dst_data)
    }
    
    /// 简单的图像缩放（最近邻插值）
    pub fn scale_image_nearest(
        src_data: &[u8],
        src_width: u16,
        src_height: u16,
        dst_width: u16,
        dst_height: u16,
        format: ColorFormat,
    ) -> Result<Vec<u8, 65536>, GraphicsError> {
        let bytes_per_pixel = format.bytes_per_pixel() as usize;
        let dst_size = dst_width as usize * dst_height as usize * bytes_per_pixel;
        let mut dst_data = Vec::new();
        dst_data.resize(dst_size, 0)
            .map_err(|_| GraphicsError::OutOfMemory)?;
        
        let x_ratio = src_width as f32 / dst_width as f32;
        let y_ratio = src_height as f32 / dst_height as f32;
        
        for dst_y in 0..dst_height {
            for dst_x in 0..dst_width {
                let src_x = (dst_x as f32 * x_ratio) as u16;
                let src_y = (dst_y as f32 * y_ratio) as u16;
                
                if src_x < src_width && src_y < src_height {
                    let src_index = (src_y as usize * src_width as usize + src_x as usize) * bytes_per_pixel;
                    let dst_index = (dst_y as usize * dst_width as usize + dst_x as usize) * bytes_per_pixel;
                    
                    for i in 0..bytes_per_pixel {
                        if src_index + i < src_data.len() && dst_index + i < dst_data.len() {
                            dst_data[dst_index + i] = src_data[src_index + i];
                        }
                    }
                }
            }
        }
        
        Ok(dst_data)
    }
}

/// 重新导出常用类型
pub use renderer::{GraphicsRenderer, RenderTarget};
pub use fonts::{FontManager, FontInfo, FontStyle};
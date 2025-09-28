//! 字体管理模块
//! 
//! 提供字体加载、渲染和管理功能，支持位图字体和矢量字体

use crate::graphics::{Color, ColorFormat, GraphicsError};
use heapless::{FnvIndexMap, Vec, String};
use core::fmt;

/// 字体错误类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FontError {
    /// 字体未找到
    FontNotFound,
    /// 字符未找到
    CharacterNotFound,
    /// 字体大小无效
    InvalidSize,
    /// 字体格式不支持
    UnsupportedFormat,
    /// 内存不足
    OutOfMemory,
    /// 字体数据损坏
    CorruptedData,
}

impl fmt::Display for FontError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FontError::FontNotFound => write!(f, "Font not found"),
            FontError::CharacterNotFound => write!(f, "Character not found"),
            FontError::InvalidSize => write!(f, "Invalid font size"),
            FontError::UnsupportedFormat => write!(f, "Unsupported font format"),
            FontError::OutOfMemory => write!(f, "Out of memory"),
            FontError::CorruptedData => write!(f, "Corrupted font data"),
        }
    }
}

/// 字体类型
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FontType {
    /// 位图字体
    Bitmap,
    /// 矢量字体
    Vector,
    /// 系统字体
    System,
}

/// 字体样式
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FontStyle {
    /// 是否粗体
    pub bold: bool,
    /// 是否斜体
    pub italic: bool,
    /// 是否下划线
    pub underline: bool,
    /// 是否删除线
    pub strikethrough: bool,
}

impl Default for FontStyle {
    fn default() -> Self {
        Self {
            bold: false,
            italic: false,
            underline: false,
            strikethrough: false,
        }
    }
}

/// 字符信息
#[derive(Debug, Clone)]
pub struct CharacterInfo {
    /// 字符编码
    pub code: u32,
    /// 字符宽度
    pub width: u16,
    /// 字符高度
    pub height: u16,
    /// X偏移
    pub x_offset: i16,
    /// Y偏移
    pub y_offset: i16,
    /// 字符前进宽度
    pub advance: u16,
    /// 位图数据
    pub bitmap: Vec<u8, 256>,
}

impl CharacterInfo {
    /// 创建新的字符信息
    pub fn new(code: u32, width: u16, height: u16) -> Self {
        Self {
            code,
            width,
            height,
            x_offset: 0,
            y_offset: 0,
            advance: width,
            bitmap: Vec::new(),
        }
    }

    /// 设置位图数据
    pub fn set_bitmap(&mut self, data: &[u8]) -> Result<(), FontError> {
        self.bitmap.clear();
        for &byte in data {
            self.bitmap.push(byte).map_err(|_| FontError::OutOfMemory)?;
        }
        Ok(())
    }

    /// 获取像素值
    pub fn get_pixel(&self, x: u16, y: u16) -> bool {
        if x >= self.width || y >= self.height {
            return false;
        }

        let byte_index = (y * ((self.width + 7) / 8) + x / 8) as usize;
        let bit_index = 7 - (x % 8);

        if byte_index < self.bitmap.len() {
            (self.bitmap[byte_index] >> bit_index) & 1 != 0
        } else {
            false
        }
    }
}

/// 字体信息
#[derive(Debug, Clone)]
pub struct FontInfo {
    /// 字体名称
    pub name: String<32>,
    /// 字体类型
    pub font_type: FontType,
    /// 字体大小
    pub size: u16,
    /// 字体样式
    pub style: FontStyle,
    /// 行高
    pub line_height: u16,
    /// 基线偏移
    pub baseline: u16,
    /// 字符映射
    pub characters: FnvIndexMap<u32, CharacterInfo, 128>,
}

impl FontInfo {
    /// 创建新的字体信息
    pub fn new(name: &str, font_type: FontType, size: u16) -> Result<Self, FontError> {
        let mut font_name = String::new();
        font_name.push_str(name).map_err(|_| FontError::OutOfMemory)?;

        Ok(Self {
            name: font_name,
            font_type,
            size,
            style: FontStyle::default(),
            line_height: size + size / 4, // 默认行高为字体大小的1.25倍
            baseline: (size * 3) / 4,     // 默认基线为字体大小的75%
            characters: FnvIndexMap::new(),
        })
    }

    /// 添加字符
    pub fn add_character(&mut self, character: CharacterInfo) -> Result<(), FontError> {
        self.characters
            .insert(character.code, character)
            .map_err(|_| FontError::OutOfMemory)?;
        Ok(())
    }

    /// 获取字符信息
    pub fn get_character(&self, code: u32) -> Option<&CharacterInfo> {
        self.characters.get(&code)
    }

    /// 计算文本宽度
    pub fn calculate_text_width(&self, text: &str) -> u16 {
        let mut width = 0;
        for ch in text.chars() {
            if let Some(char_info) = self.get_character(ch as u32) {
                width += char_info.advance;
            }
        }
        width
    }

    /// 计算文本边界框
    pub fn calculate_text_bounds(&self, text: &str) -> (u16, u16) {
        let width = self.calculate_text_width(text);
        let height = self.line_height;
        (width, height)
    }
}

/// 字体管理器
#[derive(Debug)]
pub struct FontManager {
    /// 已加载的字体
    fonts: FnvIndexMap<String<32>, FontInfo, 8>,
    /// 默认字体名称
    default_font: Option<String<32>>,
    /// 字体缓存统计
    cache_hits: u32,
    cache_misses: u32,
}

impl FontManager {
    /// 创建新的字体管理器
    pub fn new() -> Self {
        Self {
            fonts: FnvIndexMap::new(),
            default_font: None,
            cache_hits: 0,
            cache_misses: 0,
        }
    }

    /// 加载字体
    pub fn load_font(&mut self, font: FontInfo) -> Result<(), FontError> {
        let name = font.name.clone();
        self.fonts.insert(name.clone(), font).map_err(|_| FontError::OutOfMemory)?;
        
        // 如果没有默认字体，设置为默认字体
        if self.default_font.is_none() {
            self.default_font = Some(name);
        }
        
        Ok(())
    }

    /// 获取字体
    pub fn get_font(&mut self, name: &str) -> Option<&FontInfo> {
        if let Some(font) = self.fonts.get(name) {
            self.cache_hits += 1;
            Some(font)
        } else {
            self.cache_misses += 1;
            None
        }
    }

    /// 获取默认字体
    pub fn get_default_font(&mut self) -> Option<&FontInfo> {
        if let Some(ref name) = self.default_font.clone() {
            self.get_font(name)
        } else {
            None
        }
    }

    /// 设置默认字体
    pub fn set_default_font(&mut self, name: &str) -> Result<(), FontError> {
        if self.fonts.contains_key(name) {
            let mut font_name = String::new();
            font_name.push_str(name).map_err(|_| FontError::OutOfMemory)?;
            self.default_font = Some(font_name);
            Ok(())
        } else {
            Err(FontError::FontNotFound)
        }
    }

    /// 卸载字体
    pub fn unload_font(&mut self, name: &str) -> Result<(), FontError> {
        if self.fonts.remove(name).is_some() {
            // 如果卸载的是默认字体，清除默认字体设置
            if let Some(ref default_name) = self.default_font {
                if default_name == name {
                    self.default_font = None;
                }
            }
            Ok(())
        } else {
            Err(FontError::FontNotFound)
        }
    }

    /// 获取已加载字体列表
    pub fn list_fonts(&self) -> Vec<&str, 8> {
        let mut font_list = Vec::new();
        for name in self.fonts.keys() {
            if font_list.push(name.as_str()).is_err() {
                break;
            }
        }
        font_list
    }

    /// 清除所有字体
    pub fn clear(&mut self) {
        self.fonts.clear();
        self.default_font = None;
        self.cache_hits = 0;
        self.cache_misses = 0;
    }

    /// 获取缓存统计
    pub fn get_cache_stats(&self) -> (u32, u32) {
        (self.cache_hits, self.cache_misses)
    }

    /// 获取缓存命中率
    pub fn get_cache_hit_rate(&self) -> f32 {
        let total = self.cache_hits + self.cache_misses;
        if total > 0 {
            self.cache_hits as f32 / total as f32
        } else {
            0.0
        }
    }
}

impl Default for FontManager {
    fn default() -> Self {
        Self::new()
    }
}

/// 文本渲染器
#[derive(Debug)]
pub struct TextRenderer {
    /// 字体管理器
    font_manager: FontManager,
    /// 当前字体
    current_font: Option<String<32>>,
    /// 文本颜色
    text_color: Color,
    /// 背景颜色
    background_color: Option<Color>,
}

impl TextRenderer {
    /// 创建新的文本渲染器
    pub fn new() -> Self {
        Self {
            font_manager: FontManager::new(),
            current_font: None,
            text_color: Color::new(255, 255, 255, 255), // 默认白色
            background_color: None,
        }
    }

    /// 设置当前字体
    pub fn set_font(&mut self, name: &str) -> Result<(), FontError> {
        if self.font_manager.fonts.contains_key(name) {
            let mut font_name = String::new();
            font_name.push_str(name).map_err(|_| FontError::OutOfMemory)?;
            self.current_font = Some(font_name);
            Ok(())
        } else {
            Err(FontError::FontNotFound)
        }
    }

    /// 设置文本颜色
    pub fn set_text_color(&mut self, color: Color) {
        self.text_color = color;
    }

    /// 设置背景颜色
    pub fn set_background_color(&mut self, color: Option<Color>) {
        self.background_color = color;
    }

    /// 获取字体管理器
    pub fn font_manager(&mut self) -> &mut FontManager {
        &mut self.font_manager
    }

    /// 渲染文本到缓冲区
    pub fn render_text(
        &mut self,
        text: &str,
        x: i32,
        y: i32,
        buffer: &mut [u8],
        width: u16,
        height: u16,
        format: ColorFormat,
    ) -> Result<(), FontError> {
        let font = if let Some(ref font_name) = self.current_font.clone() {
            self.font_manager.get_font(font_name)
        } else {
            self.font_manager.get_default_font()
        };

        let font = font.ok_or(FontError::FontNotFound)?;

        let mut cursor_x = x;
        let cursor_y = y;

        for ch in text.chars() {
            if let Some(char_info) = font.get_character(ch as u32) {
                self.render_character(
                    char_info,
                    cursor_x + char_info.x_offset as i32,
                    cursor_y + char_info.y_offset as i32,
                    buffer,
                    width,
                    height,
                    format,
                )?;
                cursor_x += char_info.advance as i32;
            }
        }

        Ok(())
    }

    /// 渲染单个字符
    fn render_character(
        &self,
        char_info: &CharacterInfo,
        x: i32,
        y: i32,
        buffer: &mut [u8],
        width: u16,
        height: u16,
        format: ColorFormat,
    ) -> Result<(), FontError> {
        for py in 0..char_info.height {
            for px in 0..char_info.width {
                let pixel_x = x + px as i32;
                let pixel_y = y + py as i32;

                // 检查边界
                if pixel_x < 0 || pixel_y < 0 || 
                   pixel_x >= width as i32 || pixel_y >= height as i32 {
                    continue;
                }

                if char_info.get_pixel(px, py) {
                    self.set_pixel(
                        buffer,
                        pixel_x as u16,
                        pixel_y as u16,
                        width,
                        self.text_color,
                        format,
                    );
                } else if let Some(bg_color) = self.background_color {
                    self.set_pixel(
                        buffer,
                        pixel_x as u16,
                        pixel_y as u16,
                        width,
                        bg_color,
                        format,
                    );
                }
            }
        }

        Ok(())
    }

    /// 设置像素
    fn set_pixel(
        &self,
        buffer: &mut [u8],
        x: u16,
        y: u16,
        width: u16,
        color: Color,
        format: ColorFormat,
    ) {
        let bytes_per_pixel = format.bytes_per_pixel();
        let index = (y as usize * width as usize + x as usize) * bytes_per_pixel;

        if index + bytes_per_pixel <= buffer.len() {
            match format {
                ColorFormat::RGB565 => {
                    let rgb565 = ((color.r as u16 & 0xF8) << 8) |
                                 ((color.g as u16 & 0xFC) << 3) |
                                 (color.b as u16 >> 3);
                    buffer[index] = (rgb565 >> 8) as u8;
                    buffer[index + 1] = rgb565 as u8;
                }
                ColorFormat::RGB888 => {
                    buffer[index] = color.r;
                    buffer[index + 1] = color.g;
                    buffer[index + 2] = color.b;
                }
                ColorFormat::RGBA8888 => {
                    buffer[index] = color.r;
                    buffer[index + 1] = color.g;
                    buffer[index + 2] = color.b;
                    buffer[index + 3] = color.a;
                }
                ColorFormat::Monochrome => {
                    let gray = (color.r as u16 + color.g as u16 + color.b as u16) / 3;
                    let byte_index = index / 8;
                    let bit_index = 7 - (index % 8);
                    if gray > 127 {
                        buffer[byte_index] |= 1 << bit_index;
                    } else {
                        buffer[byte_index] &= !(1 << bit_index);
                    }
                }
            }
        }
    }

    /// 计算文本尺寸
    pub fn measure_text(&mut self, text: &str) -> Result<(u16, u16), FontError> {
        let font = if let Some(ref font_name) = self.current_font.clone() {
            self.font_manager.get_font(font_name)
        } else {
            self.font_manager.get_default_font()
        };

        let font = font.ok_or(FontError::FontNotFound)?;
        Ok(font.calculate_text_bounds(text))
    }
}

impl Default for TextRenderer {
    fn default() -> Self {
        Self::new()
    }
}

/// 字体工具函数
pub mod font_utils {
    use super::*;

    /// 创建简单的8x8位图字体
    pub fn create_simple_8x8_font() -> Result<FontInfo, FontError> {
        let mut font = FontInfo::new("Simple8x8", FontType::Bitmap, 8)?;

        // 添加一些基本字符 (简化的位图数据)
        let char_data = [
            // 'A' (0x41)
            (0x41, [
                0b00111000,
                0b01000100,
                0b10000010,
                0b10000010,
                0b11111110,
                0b10000010,
                0b10000010,
                0b00000000,
            ]),
            // 'B' (0x42)
            (0x42, [
                0b11111100,
                0b10000010,
                0b10000010,
                0b11111100,
                0b10000010,
                0b10000010,
                0b11111100,
                0b00000000,
            ]),
            // 空格 (0x20)
            (0x20, [0; 8]),
        ];

        for (code, bitmap) in char_data.iter() {
            let mut char_info = CharacterInfo::new(*code, 8, 8);
            char_info.set_bitmap(bitmap)?;
            font.add_character(char_info)?;
        }

        Ok(font)
    }

    /// 创建6x8位图字体
    pub fn create_6x8_font() -> Result<FontInfo, FontError> {
        let mut font = FontInfo::new("Font6x8", FontType::Bitmap, 8)?;
        font.line_height = 8;
        font.baseline = 6;

        // 添加数字字符
        let digit_data = [
            // '0' (0x30)
            (0x30, [0b01111100, 0b10000010, 0b10000010, 0b10000010, 0b10000010, 0b01111100]),
            // '1' (0x31)
            (0x31, [0b00001000, 0b00011000, 0b00001000, 0b00001000, 0b00001000, 0b00011100]),
        ];

        for (code, bitmap) in digit_data.iter() {
            let mut char_info = CharacterInfo::new(*code, 6, 8);
            let mut full_bitmap = [0u8; 8];
            full_bitmap[..6].copy_from_slice(bitmap);
            char_info.set_bitmap(&full_bitmap)?;
            font.add_character(char_info)?;
        }

        Ok(font)
    }

    /// 验证字体数据
    pub fn validate_font(font: &FontInfo) -> Result<(), FontError> {
        if font.size == 0 {
            return Err(FontError::InvalidSize);
        }

        if font.characters.is_empty() {
            return Err(FontError::CorruptedData);
        }

        // 验证字符数据
        for char_info in font.characters.values() {
            if char_info.width == 0 || char_info.height == 0 {
                return Err(FontError::CorruptedData);
            }

            let expected_bytes = ((char_info.width + 7) / 8) * char_info.height;
            if char_info.bitmap.len() < expected_bytes as usize {
                return Err(FontError::CorruptedData);
            }
        }

        Ok(())
    }

    /// 计算字体内存使用
    pub fn calculate_font_memory_usage(font: &FontInfo) -> usize {
        let mut total = core::mem::size_of::<FontInfo>();
        
        for char_info in font.characters.values() {
            total += core::mem::size_of::<CharacterInfo>();
            total += char_info.bitmap.len();
        }
        
        total
    }

    /// 获取字体统计信息
    pub fn get_font_stats(font: &FontInfo) -> (usize, u16, u16, u16) {
        let char_count = font.characters.len();
        let avg_width = if char_count > 0 {
            font.characters.values()
                .map(|c| c.width)
                .sum::<u16>() / char_count as u16
        } else {
            0
        };
        let max_width = font.characters.values()
            .map(|c| c.width)
            .max()
            .unwrap_or(0);
        let min_width = font.characters.values()
            .map(|c| c.width)
            .min()
            .unwrap_or(0);

        (char_count, avg_width, max_width, min_width)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_character_info() {
        let mut char_info = CharacterInfo::new(0x41, 8, 8);
        let bitmap = [0xFF, 0x81, 0x81, 0x81, 0x81, 0x81, 0xFF, 0x00];
        
        assert!(char_info.set_bitmap(&bitmap).is_ok());
        assert_eq!(char_info.bitmap.len(), 8);
        assert!(char_info.get_pixel(0, 0));
        assert!(!char_info.get_pixel(1, 1));
    }

    #[test]
    fn test_font_manager() {
        let mut manager = FontManager::new();
        let font = font_utils::create_simple_8x8_font().unwrap();
        
        assert!(manager.load_font(font).is_ok());
        assert!(manager.get_font("Simple8x8").is_some());
        assert!(manager.get_default_font().is_some());
        
        let fonts = manager.list_fonts();
        assert_eq!(fonts.len(), 1);
        assert_eq!(fonts[0], "Simple8x8");
    }

    #[test]
    fn test_text_renderer() {
        let mut renderer = TextRenderer::new();
        let font = font_utils::create_simple_8x8_font().unwrap();
        
        assert!(renderer.font_manager().load_font(font).is_ok());
        assert!(renderer.set_font("Simple8x8").is_ok());
        
        let (width, height) = renderer.measure_text("AB").unwrap();
        assert_eq!(width, 16); // 2 characters * 8 pixels each
        assert_eq!(height, 10); // line_height
    }
}
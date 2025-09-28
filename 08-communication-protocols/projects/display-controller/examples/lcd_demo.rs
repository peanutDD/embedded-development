//! LCD 显示器演示程序
//! 
//! 演示如何使用 display-controller 库控制 LCD 显示器，
//! 包括字符 LCD 和图形 LCD 的基本操作。

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{Write, Transfer};
use embedded_hal::digital::v2::OutputPin;

use display_controller::{
    prelude::*,
    displays::lcd::{CharacterLcdParallel, GraphicLcdSpi, LcdConfig, LcdType, CharacterLcdSize},
    graphics::{Color, DrawParams, TextParams, ColorFormat},
};

// 模拟的硬件接口实现
struct MockSpi;
struct MockPin;
struct MockDelay;

impl Write<u8> for MockSpi {
    type Error = ();

    fn write(&mut self, _words: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl Transfer<u8> for MockSpi {
    type Error = ();

    fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
        Ok(words)
    }
}

impl OutputPin for MockPin {
    type Error = ();

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl DelayMs<u32> for MockDelay {
    fn delay_ms(&mut self, _ms: u32) {}
}

impl DelayMs<u16> for MockDelay {
    fn delay_ms(&mut self, _ms: u16) {}
}

impl DelayMs<u8> for MockDelay {
    fn delay_ms(&mut self, _ms: u8) {}
}

#[entry]
fn main() -> ! {
    // 初始化硬件
    let mut delay = MockDelay;

    // 运行字符 LCD 演示
    run_character_lcd_demo(&mut delay);
    
    // 运行图形 LCD 演示
    run_graphic_lcd_demo(&mut delay);

    loop {}
}

fn run_character_lcd_demo<D: DelayMs<u32>>(delay: &mut D) {
    // 创建模拟的并行接口引脚
    let rs = MockPin;
    let enable = MockPin;
    let d4 = MockPin;
    let d5 = MockPin;
    let d6 = MockPin;
    let d7 = MockPin;

    // 创建字符 LCD 配置
    let lcd_config = LcdConfig {
        lcd_type: LcdType::Character,
        character_size: Some(CharacterLcdSize::_16x2),
        backlight_control: true,
        cursor_blink: false,
        ..Default::default()
    };

    // 创建字符 LCD 显示器
    let mut char_lcd = CharacterLcdParallel::new(
        rs, enable, d4, d5, d6, d7, lcd_config
    );

    // 初始化显示器
    if let Err(_) = char_lcd.init(delay) {
        // 处理初始化错误
        loop {}
    }

    // 创建显示控制器
    let mut controller = DisplayController::with_default_config();
    
    // 初始化控制器
    if let Err(_) = controller.init() {
        loop {}
    }

    // 注册字符 LCD
    if let Err(_) = controller.display_manager().register_display("char_lcd", Box::new(char_lcd)) {
        loop {}
    }

    // 运行字符 LCD 演示
    demo_character_lcd_basic(&mut controller, delay);
    demo_character_lcd_scrolling(&mut controller, delay);
    demo_character_lcd_custom_chars(&mut controller, delay);
}

fn run_graphic_lcd_demo<D: DelayMs<u32>>(delay: &mut D) {
    // 创建模拟的 SPI 接口
    let spi = MockSpi;
    let dc = MockPin;
    let cs = MockPin;
    let rst = MockPin;

    // 创建图形 LCD 配置
    let lcd_config = LcdConfig {
        lcd_type: LcdType::Graphic,
        width: Some(240),
        height: Some(320),
        color_depth: Some(16),
        backlight_control: true,
        ..Default::default()
    };

    // 创建图形 LCD 显示器
    let mut graphic_lcd = GraphicLcdSpi::new(
        spi, dc, cs, Some(rst), lcd_config
    );

    // 初始化显示器
    if let Err(_) = graphic_lcd.init(delay) {
        loop {}
    }

    // 创建显示控制器
    let mut controller = DisplayController::with_default_config();
    
    if let Err(_) = controller.init() {
        loop {}
    }

    // 注册图形 LCD
    if let Err(_) = controller.display_manager().register_display("graphic_lcd", Box::new(graphic_lcd)) {
        loop {}
    }

    // 运行图形 LCD 演示
    demo_graphic_lcd_basic(&mut controller, delay);
    demo_graphic_lcd_colors(&mut controller, delay);
    demo_graphic_lcd_shapes(&mut controller, delay);
}

fn demo_character_lcd_basic<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    // 创建文本缓冲区
    let mut buffer = [0u8; 16 * 2]; // 16x2 字符显示
    let text_renderer = controller.text_renderer();
    
    // 显示欢迎信息
    let _ = text_renderer.render_text(
        "Hello LCD!",
        0,
        0,
        &mut buffer,
        16,
        2,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Display Demo",
        0,
        1,
        &mut buffer,
        16,
        2,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
    
    // 显示时间信息（模拟）
    let _ = controller.clear_all();
    let _ = text_renderer.render_text(
        "Time: 12:34:56",
        0,
        0,
        &mut buffer,
        16,
        2,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Date: 2024/01/01",
        0,
        1,
        &mut buffer,
        16,
        2,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
}

fn demo_character_lcd_scrolling<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 16 * 2];
    
    // 滚动文本演示
    let long_text = "This is a very long text that will scroll across the display";
    let text_len = long_text.len();
    
    for offset in 0..text_len.saturating_sub(16) {
        let _ = controller.clear_all();
        
        // 获取当前显示的文本片段
        let display_text = if offset + 16 <= text_len {
            &long_text[offset..offset + 16]
        } else {
            &long_text[offset..]
        };
        
        let _ = text_renderer.render_text(
            display_text,
            0,
            0,
            &mut buffer,
            16,
            2,
            ColorFormat::Monochrome,
        );
        
        let _ = text_renderer.render_text(
            "Scrolling Demo",
            0,
            1,
            &mut buffer,
            16,
            2,
            ColorFormat::Monochrome,
        );
        
        let _ = controller.refresh_all();
        delay.delay_ms(200);
    }
}

fn demo_character_lcd_custom_chars<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 16 * 2];
    
    // 显示自定义字符演示
    let _ = controller.clear_all();
    
    let _ = text_renderer.render_text(
        "Custom Chars:",
        0,
        0,
        &mut buffer,
        16,
        2,
        ColorFormat::Monochrome,
    );
    
    // 模拟显示自定义字符（实际实现中需要定义自定义字符）
    let _ = text_renderer.render_text(
        "[*] [>] [<] [^]",
        0,
        1,
        &mut buffer,
        16,
        2,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(3000);
}

fn demo_graphic_lcd_basic<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    let renderer = controller.renderer();
    
    // 绘制基本图形
    let white = Color::new(255, 255, 255, 255);
    let red = Color::new(255, 0, 0, 255);
    let green = Color::new(0, 255, 0, 255);
    let blue = Color::new(0, 0, 255, 255);
    
    // 绘制彩色矩形
    let red_params = DrawParams {
        color: red,
        fill: true,
        ..Default::default()
    };
    let _ = renderer.draw_rectangle(10, 10, 60, 40, &red_params);
    
    let green_params = DrawParams {
        color: green,
        fill: true,
        ..Default::default()
    };
    let _ = renderer.draw_rectangle(80, 10, 60, 40, &green_params);
    
    let blue_params = DrawParams {
        color: blue,
        fill: true,
        ..Default::default()
    };
    let _ = renderer.draw_rectangle(150, 10, 60, 40, &blue_params);
    
    // 绘制白色边框
    let border_params = DrawParams {
        color: white,
        fill: false,
        ..Default::default()
    };
    let _ = renderer.draw_rectangle(5, 5, 230, 50, &border_params);
    
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
}

fn demo_graphic_lcd_colors<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let renderer = controller.renderer();
    
    // 颜色渐变演示
    for i in 0..32 {
        let _ = controller.clear_all();
        
        // 创建渐变色
        let r = (i * 8) as u8;
        let g = ((31 - i) * 8) as u8;
        let b = 128u8;
        
        let gradient_color = Color::new(r, g, b, 255);
        let gradient_params = DrawParams {
            color: gradient_color,
            fill: true,
            ..Default::default()
        };
        
        // 绘制渐变矩形
        let _ = renderer.draw_rectangle(
            50 + i * 5,
            100,
            20,
            100,
            &gradient_params
        );
        
        let _ = controller.refresh_all();
        delay.delay_ms(100);
    }
}

fn demo_graphic_lcd_shapes<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let _ = controller.clear_all();
    let renderer = controller.renderer();
    
    // 绘制各种形状
    let colors = [
        Color::new(255, 0, 0, 255),   // 红色
        Color::new(0, 255, 0, 255),   // 绿色
        Color::new(0, 0, 255, 255),   // 蓝色
        Color::new(255, 255, 0, 255), // 黄色
        Color::new(255, 0, 255, 255), // 洋红
        Color::new(0, 255, 255, 255), // 青色
    ];
    
    // 绘制圆形
    for (i, &color) in colors.iter().enumerate() {
        let params = DrawParams {
            color,
            fill: true,
            ..Default::default()
        };
        
        let x = 40 + (i % 3) * 80;
        let y = 60 + (i / 3) * 80;
        let _ = renderer.draw_circle(x as u16, y as u16, 25, &params);
    }
    
    // 绘制连接线
    let line_params = DrawParams {
        color: Color::new(255, 255, 255, 255),
        fill: false,
        ..Default::default()
    };
    
    let _ = renderer.draw_line(40, 60, 120, 60, &line_params);
    let _ = renderer.draw_line(120, 60, 200, 60, &line_params);
    let _ = renderer.draw_line(40, 140, 120, 140, &line_params);
    let _ = renderer.draw_line(120, 140, 200, 140, &line_params);
    let _ = renderer.draw_line(40, 60, 40, 140, &line_params);
    let _ = renderer.draw_line(120, 60, 120, 140, &line_params);
    let _ = renderer.draw_line(200, 60, 200, 140, &line_params);
    
    let _ = controller.refresh_all();
    delay.delay_ms(3000);
}

/// 图形 LCD 文本显示演示
fn demo_graphic_lcd_text<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let _ = controller.clear_all();
    
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 240 * 320 * 2]; // RGB565 格式
    
    // 设置文本颜色
    text_renderer.set_text_color(Color::new(255, 255, 255, 255));
    
    // 显示标题
    let _ = text_renderer.render_text(
        "Graphic LCD Demo",
        20,
        20,
        &mut buffer,
        240,
        320,
        ColorFormat::RGB565,
    );
    
    // 显示多行文本
    let lines = [
        "Line 1: Hello World!",
        "Line 2: Graphics Test",
        "Line 3: Color Display",
        "Line 4: Text Rendering",
    ];
    
    for (i, line) in lines.iter().enumerate() {
        let _ = text_renderer.render_text(
            line,
            20,
            60 + i * 25,
            &mut buffer,
            240,
            320,
            ColorFormat::RGB565,
        );
    }
    
    let _ = controller.refresh_all();
    delay.delay_ms(3000);
}

/// 性能测试演示
fn demo_lcd_performance<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let renderer = controller.renderer();
    
    // 测试填充性能
    let colors = [
        Color::new(255, 0, 0, 255),
        Color::new(0, 255, 0, 255),
        Color::new(0, 0, 255, 255),
        Color::new(255, 255, 0, 255),
    ];
    
    for &color in colors.iter() {
        let fill_params = DrawParams {
            color,
            fill: true,
            ..Default::default()
        };
        
        // 全屏填充
        let _ = renderer.draw_rectangle(0, 0, 240, 320, &fill_params);
        let _ = controller.refresh_all();
        delay.delay_ms(500);
    }
    
    // 测试绘制性能
    let _ = controller.clear_all();
    let draw_params = DrawParams {
        color: Color::new(255, 255, 255, 255),
        fill: false,
        ..Default::default()
    };
    
    // 绘制网格
    for i in (0..240).step_by(20) {
        let _ = renderer.draw_line(i, 0, i, 320, &draw_params);
    }
    for i in (0..320).step_by(20) {
        let _ = renderer.draw_line(0, i, 240, i, &draw_params);
    }
    
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
}
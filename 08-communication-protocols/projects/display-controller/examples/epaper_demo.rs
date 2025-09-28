//! 电子纸显示器演示程序
//! 
//! 演示如何使用 display-controller 库控制电子纸显示器，
//! 包括全刷新、快速刷新、部分刷新和低功耗操作。

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::spi::{Write, Transfer};
use embedded_hal::digital::v2::{OutputPin, InputPin};

use display_controller::{
    prelude::*,
    displays::epaper::{EPaperDisplay, EPaperConfig, EPaperType, EPaperColor, RefreshMode, Rotation},
    graphics::{Color, DrawParams, TextParams, ColorFormat},
};

// 模拟的硬件接口实现
struct MockSpi;
struct MockOutputPin;
struct MockInputPin;
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

impl OutputPin for MockOutputPin {
    type Error = ();

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl InputPin for MockInputPin {
    type Error = ();

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(true)
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(false)
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

    // 运行电子纸演示
    run_epaper_demo(&mut delay);

    loop {}
}

fn run_epaper_demo<D: DelayMs<u32>>(delay: &mut D) {
    // 创建模拟的 SPI 接口和控制引脚
    let spi = MockSpi;
    let cs = MockOutputPin;
    let dc = MockOutputPin;
    let rst = MockOutputPin;
    let busy = MockInputPin;

    // 创建电子纸配置 (2.13" 黑白电子纸)
    let epaper_config = EPaperConfig {
        epaper_type: EPaperType::EPD2in13,
        color_mode: EPaperColor::BlackWhite,
        rotation: Rotation::Rotate0,
        refresh_mode: RefreshMode::Full,
        lut_full_update: None,
        lut_partial_update: None,
    };

    // 创建电子纸显示器
    let mut epaper = EPaperDisplay::new(
        spi, cs, dc, rst, busy, epaper_config
    );

    // 初始化显示器
    if let Err(_) = epaper.init(delay) {
        // 处理初始化错误
        loop {}
    }

    // 创建显示控制器
    let mut controller = DisplayController::with_default_config();
    
    // 初始化控制器
    if let Err(_) = controller.init() {
        loop {}
    }

    // 注册电子纸显示器
    if let Err(_) = controller.display_manager().register_display("epaper", Box::new(epaper)) {
        loop {}
    }

    // 运行各种演示
    demo_epaper_basic_text(&mut controller, delay);
    demo_epaper_graphics(&mut controller, delay);
    demo_epaper_partial_update(&mut controller, delay);
    demo_epaper_power_management(&mut controller, delay);
    demo_epaper_weather_display(&mut controller, delay);
}

fn demo_epaper_basic_text<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 250 * 122 / 8]; // 2.13" 电子纸缓冲区
    
    // 设置文本颜色（黑色）
    text_renderer.set_text_color(Color::new(0, 0, 0, 255));
    
    // 显示标题
    let _ = text_renderer.render_text(
        "E-Paper Display",
        10,
        10,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    // 显示基本信息
    let _ = text_renderer.render_text(
        "2.13 inch Display",
        10,
        30,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Black & White",
        10,
        50,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Low Power Demo",
        10,
        70,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Full Refresh Mode",
        10,
        90,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    // 全刷新显示
    let _ = controller.refresh_all();
    
    // 电子纸刷新需要较长时间
    delay.delay_ms(2000);
}

fn demo_epaper_graphics<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    let renderer = controller.renderer();
    
    // 绘制图形元素
    let black = Color::new(0, 0, 0, 255);
    let draw_params = DrawParams {
        color: black,
        fill: false,
        ..Default::default()
    };
    
    // 绘制边框
    let _ = renderer.draw_rectangle(5, 5, 240, 112, &draw_params);
    
    // 绘制网格
    for i in (20..240).step_by(40) {
        let _ = renderer.draw_line(i, 10, i, 112, &draw_params);
    }
    for i in (20..112).step_by(20) {
        let _ = renderer.draw_line(10, i, 240, i, &draw_params);
    }
    
    // 绘制圆形
    let fill_params = DrawParams {
        color: black,
        fill: true,
        ..Default::default()
    };
    let _ = renderer.draw_circle(60, 40, 15, &fill_params);
    let _ = renderer.draw_circle(140, 40, 15, &draw_params); // 空心圆
    let _ = renderer.draw_circle(220, 40, 15, &fill_params);
    
    // 绘制矩形
    let _ = renderer.draw_rectangle(40, 70, 30, 20, &fill_params);
    let _ = renderer.draw_rectangle(120, 70, 30, 20, &draw_params); // 空心矩形
    let _ = renderer.draw_rectangle(200, 70, 30, 20, &fill_params);
    
    // 全刷新显示
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
}

fn demo_epaper_partial_update<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 注意：部分刷新功能需要电子纸硬件支持
    // 这里演示概念，实际实现需要根据具体电子纸型号调整
    
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 250 * 122 / 8];
    
    // 显示静态内容
    let _ = controller.clear_all();
    
    let _ = text_renderer.render_text(
        "Partial Update Demo",
        10,
        10,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Static Content:",
        10,
        30,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Time:",
        10,
        70,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Counter:",
        10,
        90,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    // 全刷新显示静态内容
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
    
    // 模拟部分更新动态内容
    for i in 0..10 {
        // 清除动态区域（实际实现中只清除特定区域）
        
        // 更新时间显示
        let time_text = match i % 4 {
            0 => "12:34:56",
            1 => "12:34:57",
            2 => "12:34:58",
            _ => "12:34:59",
        };
        
        let _ = text_renderer.render_text(
            time_text,
            60,
            70,
            &mut buffer,
            250,
            122,
            ColorFormat::Monochrome,
        );
        
        // 更新计数器
        let counter_text = match i {
            0 => "001",
            1 => "002",
            2 => "003",
            3 => "004",
            4 => "005",
            5 => "006",
            6 => "007",
            7 => "008",
            8 => "009",
            _ => "010",
        };
        
        let _ = text_renderer.render_text(
            counter_text,
            80,
            90,
            &mut buffer,
            250,
            122,
            ColorFormat::Monochrome,
        );
        
        // 部分刷新（快速刷新）
        let _ = controller.refresh_all();
        delay.delay_ms(500);
    }
}

fn demo_epaper_power_management<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 250 * 122 / 8];
    
    // 显示电源管理信息
    let _ = controller.clear_all();
    
    let _ = text_renderer.render_text(
        "Power Management",
        10,
        10,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Low Power Mode",
        10,
        30,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Sleep in 5 sec...",
        10,
        50,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
    
    // 倒计时
    for i in (1..=5).rev() {
        let countdown_text = match i {
            5 => "Sleep in 5 sec...",
            4 => "Sleep in 4 sec...",
            3 => "Sleep in 3 sec...",
            2 => "Sleep in 2 sec...",
            _ => "Sleep in 1 sec...",
        };
        
        // 清除倒计时区域并更新
        let _ = text_renderer.render_text(
            countdown_text,
            10,
            50,
            &mut buffer,
            250,
            122,
            ColorFormat::Monochrome,
        );
        
        let _ = controller.refresh_all();
        delay.delay_ms(1000);
    }
    
    // 进入睡眠模式
    let _ = text_renderer.render_text(
        "Entering Sleep...",
        10,
        70,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(1000);
    
    // 执行睡眠
    let _ = controller.sleep();
    delay.delay_ms(3000); // 模拟睡眠时间
    
    // 唤醒
    let _ = controller.wake_up();
    
    // 显示唤醒信息
    let _ = controller.clear_all();
    let _ = text_renderer.render_text(
        "Wake Up!",
        10,
        10,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "System Ready",
        10,
        30,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
}

fn demo_epaper_weather_display<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 模拟天气显示应用
    let text_renderer = controller.text_renderer();
    let renderer = controller.renderer();
    let mut buffer = [0u8; 250 * 122 / 8];
    
    let _ = controller.clear_all();
    
    // 绘制天气界面框架
    let black = Color::new(0, 0, 0, 255);
    let border_params = DrawParams {
        color: black,
        fill: false,
        ..Default::default()
    };
    
    // 主边框
    let _ = renderer.draw_rectangle(5, 5, 240, 112, &border_params);
    
    // 分割线
    let _ = renderer.draw_line(5, 35, 245, 35, &border_params);
    let _ = renderer.draw_line(5, 75, 245, 75, &border_params);
    
    // 标题
    let _ = text_renderer.render_text(
        "Weather Station",
        15,
        15,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    // 当前时间
    let _ = text_renderer.render_text(
        "2024-01-01 12:34",
        130,
        15,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    // 温度信息
    let _ = text_renderer.render_text(
        "Temperature: 22C",
        15,
        45,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Humidity: 65%",
        15,
        60,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    // 天气状况
    let _ = text_renderer.render_text(
        "Condition: Sunny",
        15,
        85,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Wind: 5 km/h NE",
        15,
        100,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    // 绘制简单的天气图标（太阳）
    let fill_params = DrawParams {
        color: black,
        fill: true,
        ..Default::default()
    };
    
    // 太阳中心
    let _ = renderer.draw_circle(200, 55, 8, &fill_params);
    
    // 太阳光线
    let line_params = DrawParams {
        color: black,
        fill: false,
        ..Default::default()
    };
    
    let _ = renderer.draw_line(200, 40, 200, 45, &line_params); // 上
    let _ = renderer.draw_line(200, 65, 200, 70, &line_params); // 下
    let _ = renderer.draw_line(185, 55, 190, 55, &line_params); // 左
    let _ = renderer.draw_line(210, 55, 215, 55, &line_params); // 右
    
    // 对角线光线
    let _ = renderer.draw_line(188, 43, 192, 47, &line_params); // 左上
    let _ = renderer.draw_line(208, 47, 212, 43, &line_params); // 右上
    let _ = renderer.draw_line(188, 67, 192, 63, &line_params); // 左下
    let _ = renderer.draw_line(208, 63, 212, 67, &line_params); // 右下
    
    let _ = controller.refresh_all();
    delay.delay_ms(3000);
}

/// 电子纸信息显示演示
fn demo_epaper_info_display<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 250 * 122 / 8];
    
    // 显示电子纸技术信息
    let _ = controller.clear_all();
    
    let _ = text_renderer.render_text(
        "E-Paper Technology",
        10,
        5,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "* Bistable Display",
        10,
        25,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "* Ultra Low Power",
        10,
        40,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "* Sunlight Readable",
        10,
        55,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "* Wide Viewing Angle",
        10,
        70,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "* Paper-like Display",
        10,
        85,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Perfect for IoT!",
        10,
        105,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(3000);
}

/// 电子纸性能测试
fn demo_epaper_performance<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let renderer = controller.renderer();
    let text_renderer = controller.text_renderer();
    let mut buffer = [0u8; 250 * 122 / 8];
    
    // 测试不同刷新模式的性能
    let _ = controller.clear_all();
    
    let _ = text_renderer.render_text(
        "Performance Test",
        10,
        10,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Full Refresh Test",
        10,
        30,
        &mut buffer,
        250,
        122,
        ColorFormat::Monochrome,
    );
    
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
    
    // 测试绘制性能
    let black = Color::new(0, 0, 0, 255);
    let draw_params = DrawParams {
        color: black,
        fill: true,
        ..Default::default()
    };
    
    // 绘制多个矩形
    for i in 0..5 {
        let _ = controller.clear_all();
        
        for j in 0..=i {
            let _ = renderer.draw_rectangle(
                20 + j * 40,
                50,
                30,
                20,
                &draw_params
            );
        }
        
        let progress_text = match i {
            0 => "Progress: 20%",
            1 => "Progress: 40%",
            2 => "Progress: 60%",
            3 => "Progress: 80%",
            _ => "Progress: 100%",
        };
        
        let _ = text_renderer.render_text(
            progress_text,
            10,
            80,
            &mut buffer,
            250,
            122,
            ColorFormat::Monochrome,
        );
        
        let _ = controller.refresh_all();
        delay.delay_ms(1000);
    }
}
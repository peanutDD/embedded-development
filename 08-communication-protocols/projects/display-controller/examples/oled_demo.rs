//! OLED 显示器演示程序
//! 
//! 演示如何使用 display-controller 库控制 OLED 显示器，
//! 包括基本的图形绘制、文本显示和动画效果。

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::{Write, WriteRead};

use display_controller::{
    prelude::*,
    displays::oled::{OledI2C, OledConfig, OledResolution},
    graphics::{Color, DrawParams, TextParams},
};

// 模拟的 I2C 和 Delay 实现（实际使用时需要替换为具体平台的实现）
struct MockI2C;
struct MockDelay;

impl Write for MockI2C {
    type Error = ();

    fn write(&mut self, _addr: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl WriteRead for MockI2C {
    type Error = ();

    fn write_read(&mut self, _addr: u8, _bytes: &[u8], _buffer: &mut [u8]) -> Result<(), Self::Error> {
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
    let i2c = MockI2C;
    let mut delay = MockDelay;

    // 创建 OLED 配置
    let oled_config = OledConfig {
        resolution: OledResolution::_128x64,
        i2c_address: 0x3C,
        external_vcc: false,
        flip_horizontal: false,
        flip_vertical: false,
    };

    // 创建 OLED 显示器
    let mut oled = OledI2C::new(i2c, oled_config);

    // 初始化显示器
    if let Err(_) = oled.init(&mut delay) {
        // 处理初始化错误
        loop {}
    }

    // 创建显示控制器
    let mut controller = DisplayController::with_default_config();

    // 初始化控制器
    if let Err(_) = controller.init() {
        loop {}
    }

    // 注册 OLED 显示器
    if let Err(_) = controller.display_manager().register_display("oled", Box::new(oled)) {
        loop {}
    }

    // 运行演示
    run_oled_demo(&mut controller, &mut delay);

    loop {}
}

fn run_oled_demo<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 演示 1: 基本图形绘制
    demo_basic_graphics(controller, delay);
    
    // 演示 2: 文本显示
    demo_text_display(controller, delay);
    
    // 演示 3: 动画效果
    demo_animation(controller, delay);
    
    // 演示 4: 复合图形
    demo_complex_graphics(controller, delay);
}

fn demo_basic_graphics<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    // 获取渲染器
    let renderer = controller.renderer();
    
    // 设置绘制参数
    let draw_params = DrawParams {
        color: Color::new(255, 255, 255, 255), // 白色
        fill: true,
        ..Default::default()
    };

    // 绘制矩形
    let _ = renderer.draw_rectangle(10, 10, 40, 20, &draw_params);
    
    // 绘制圆形
    let _ = renderer.draw_circle(80, 20, 15, &draw_params);
    
    // 绘制线条
    let line_params = DrawParams {
        color: Color::new(255, 255, 255, 255),
        fill: false,
        ..Default::default()
    };
    let _ = renderer.draw_line(0, 40, 127, 40, &line_params);
    
    // 刷新显示
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
}

fn demo_text_display<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    // 获取文本渲染器
    let text_renderer = controller.text_renderer();
    
    // 设置文本颜色
    text_renderer.set_text_color(Color::new(255, 255, 255, 255));
    
    // 创建文本缓冲区（模拟）
    let mut buffer = [0u8; 128 * 64 / 8]; // 单色位图缓冲区
    
    // 渲染文本
    let _ = text_renderer.render_text(
        "Hello OLED!",
        10,
        10,
        &mut buffer,
        128,
        64,
        ColorFormat::Monochrome,
    );
    
    let _ = text_renderer.render_text(
        "Display Demo",
        10,
        30,
        &mut buffer,
        128,
        64,
        ColorFormat::Monochrome,
    );
    
    // 刷新显示
    let _ = controller.refresh_all();
    delay.delay_ms(2000);
}

fn demo_animation<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 弹跳球动画
    let mut ball_x = 10i32;
    let mut ball_y = 10i32;
    let mut vel_x = 2i32;
    let mut vel_y = 1i32;
    let ball_radius = 5u16;
    
    let draw_params = DrawParams {
        color: Color::new(255, 255, 255, 255),
        fill: true,
        ..Default::default()
    };

    for _ in 0..100 {
        // 清屏
        let _ = controller.clear_all();
        
        // 更新球的位置
        ball_x += vel_x;
        ball_y += vel_y;
        
        // 边界检测
        if ball_x <= ball_radius as i32 || ball_x >= (128 - ball_radius as i32) {
            vel_x = -vel_x;
        }
        if ball_y <= ball_radius as i32 || ball_y >= (64 - ball_radius as i32) {
            vel_y = -vel_y;
        }
        
        // 绘制球
        let renderer = controller.renderer();
        let _ = renderer.draw_circle(ball_x as u16, ball_y as u16, ball_radius, &draw_params);
        
        // 刷新显示
        let _ = controller.refresh_all();
        delay.delay_ms(50);
    }
}

fn demo_complex_graphics<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    let renderer = controller.renderer();
    
    // 绘制复杂图形组合
    let white = Color::new(255, 255, 255, 255);
    
    // 绘制边框
    let border_params = DrawParams {
        color: white,
        fill: false,
        ..Default::default()
    };
    let _ = renderer.draw_rectangle(0, 0, 127, 63, &border_params);
    
    // 绘制网格
    for i in (10..120).step_by(20) {
        let _ = renderer.draw_line(i, 5, i, 58, &border_params);
    }
    for i in (10..60).step_by(10) {
        let _ = renderer.draw_line(5, i, 122, i, &border_params);
    }
    
    // 绘制中心圆
    let fill_params = DrawParams {
        color: white,
        fill: true,
        ..Default::default()
    };
    let _ = renderer.draw_circle(64, 32, 8, &fill_params);
    
    // 绘制角落的小矩形
    let _ = renderer.draw_rectangle(110, 5, 15, 10, &fill_params);
    let _ = renderer.draw_rectangle(110, 48, 15, 10, &fill_params);
    let _ = renderer.draw_rectangle(5, 5, 15, 10, &fill_params);
    let _ = renderer.draw_rectangle(5, 48, 15, 10, &fill_params);
    
    // 刷新显示
    let _ = controller.refresh_all();
    delay.delay_ms(3000);
}

/// 系统信息显示演示
fn demo_system_info<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    // 清屏
    let _ = controller.clear_all();
    
    // 获取系统信息
    let system_info = controller.get_system_info();
    
    // 创建文本缓冲区
    let mut buffer = [0u8; 128 * 64 / 8];
    let text_renderer = controller.text_renderer();
    
    // 显示系统信息
    let _ = text_renderer.render_text(
        "System Info:",
        5,
        5,
        &mut buffer,
        128,
        64,
        ColorFormat::Monochrome,
    );
    
    // 显示显示器数量
    let display_count_text = if system_info.display_count == 1 {
        "Displays: 1"
    } else {
        "Displays: N"
    };
    let _ = text_renderer.render_text(
        display_count_text,
        5,
        20,
        &mut buffer,
        128,
        64,
        ColorFormat::Monochrome,
    );
    
    // 显示字体数量
    let font_count_text = if system_info.font_count == 0 {
        "Fonts: 0"
    } else {
        "Fonts: N"
    };
    let _ = text_renderer.render_text(
        font_count_text,
        5,
        35,
        &mut buffer,
        128,
        64,
        ColorFormat::Monochrome,
    );
    
    // 显示初始化状态
    let init_status = if system_info.initialized {
        "Status: OK"
    } else {
        "Status: ERR"
    };
    let _ = text_renderer.render_text(
        init_status,
        5,
        50,
        &mut buffer,
        128,
        64,
        ColorFormat::Monochrome,
    );
    
    // 刷新显示
    let _ = controller.refresh_all();
    delay.delay_ms(3000);
}

/// 性能测试演示
fn demo_performance_test<D: DelayMs<u32>>(controller: &mut DisplayController, delay: &mut D) {
    let renderer = controller.renderer();
    let draw_params = DrawParams {
        color: Color::new(255, 255, 255, 255),
        fill: true,
        ..Default::default()
    };
    
    // 测试绘制性能
    for frame in 0..50 {
        let _ = controller.clear_all();
        
        // 绘制多个移动的矩形
        for i in 0..5 {
            let x = (frame * 2 + i * 20) % 100;
            let y = (frame + i * 10) % 50;
            let _ = renderer.draw_rectangle(x as u16, y as u16, 10, 8, &draw_params);
        }
        
        let _ = controller.refresh_all();
        delay.delay_ms(20);
    }
}
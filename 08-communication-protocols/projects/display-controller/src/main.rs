//! Display Controller 主程序
//!
//! 这是一个演示程序，展示如何使用显示控制器库来控制不同类型的显示设备。
//! 程序会依次演示OLED、LCD和电子纸显示的基本功能。

#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::delay::DelayNs;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::{BinaryColor, Rgb565},
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

use display_controller::{
    displays::{DisplayManager, DisplayType},
    graphics::{renderer::Renderer, fonts::FontManager},
};

/// 主程序入口
#[entry]
fn main() -> ! {
    // 初始化系统
    let mut system = init_system();
    
    // 运行显示演示
    run_display_demo(&mut system);
    
    // 主循环
    loop {
        system.update();
        system.delay.delay_ms(100);
    }
}

/// 系统结构
struct System {
    display_manager: DisplayManager,
    renderer: Renderer,
    font_manager: FontManager,
    delay: cortex_m::delay::Delay,
}

impl System {
    /// 更新系统状态
    fn update(&mut self) {
        // 更新显示管理器
        if let Err(e) = self.display_manager.update() {
            // 处理错误
            self.handle_error(e);
        }
        
        // 检查显示设备状态
        self.check_display_health();
    }
    
    /// 处理错误
    fn handle_error(&mut self, error: display_controller::DisplayError) {
        match error {
            display_controller::DisplayError::Communication(_) => {
                // 尝试重新初始化通信
                let _ = self.display_manager.reinitialize_communication();
            },
            display_controller::DisplayError::Display(_) => {
                // 尝试重新初始化显示设备
                let _ = self.display_manager.reinitialize_displays();
            },
            _ => {
                // 其他错误，记录日志
                log::error!("System error: {:?}", error);
            }
        }
    }
    
    /// 检查显示设备健康状态
    fn check_display_health(&mut self) {
        let health = self.display_manager.get_health_status();
        
        for (display_type, is_healthy) in health {
            if !is_healthy {
                log::warn!("Display {:?} is not healthy", display_type);
                // 尝试恢复
                let _ = self.display_manager.recover_display(display_type);
            }
        }
    }
}

/// 初始化系统
fn init_system() -> System {
    // 初始化时钟和外设
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = /* 获取设备外设 */;
    
    // 初始化延时
    let delay = cortex_m::delay::Delay::new(cp.SYST, 72_000_000);
    
    // 初始化I2C和SPI
    let i2c = init_i2c(&dp);
    let spi = init_spi(&dp);
    
    // 创建显示管理器
    let mut display_manager = DisplayManager::new();
    
    // 注册显示设备
    if let Ok(oled) = display_controller::displays::oled::OledDisplay::new(i2c) {
        display_manager.register_display(DisplayType::Oled, Box::new(oled));
    }
    
    if let Ok(lcd) = display_controller::displays::lcd::LcdDisplay::new(
        spi,
        /* dc_pin */,
        /* rst_pin */,
    ) {
        display_manager.register_display(DisplayType::Lcd, Box::new(lcd));
    }
    
    // 初始化渲染器和字体管理器
    let renderer = Renderer::new();
    let font_manager = FontManager::new();
    
    System {
        display_manager,
        renderer,
        font_manager,
        delay,
    }
}

/// 运行显示演示
fn run_display_demo(system: &mut System) {
    // OLED演示
    if system.display_manager.is_available(DisplayType::Oled) {
        demo_oled(system);
        system.delay.delay_ms(2000);
    }
    
    // LCD演示
    if system.display_manager.is_available(DisplayType::Lcd) {
        demo_lcd(system);
        system.delay.delay_ms(2000);
    }
    
    // 电子纸演示
    if system.display_manager.is_available(DisplayType::EPaper) {
        demo_epaper(system);
        system.delay.delay_ms(5000); // 电子纸刷新较慢
    }
    
    // 综合演示
    demo_multi_display(system);
}

/// OLED演示
fn demo_oled(system: &mut System) {
    if let Ok(display) = system.display_manager.get_display_mut(DisplayType::Oled) {
        // 清屏
        display.clear(BinaryColor::Off).ok();
        
        // 绘制文本
        let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        Text::with_baseline("OLED Demo", Point::new(10, 10), text_style, Baseline::Top)
            .draw(display).ok();
        
        // 绘制矩形
        Rectangle::new(Point::new(10, 25), Size::new(50, 20))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
            .draw(display).ok();
        
        // 绘制圆形
        Circle::new(Point::new(80, 25), 15)
            .into_styled(PrimitiveStyle::with_fill(BinaryColor::On))
            .draw(display).ok();
        
        // 刷新显示
        display.flush().ok();
    }
}

/// LCD演示
fn demo_lcd(system: &mut System) {
    if let Ok(display) = system.display_manager.get_display_mut(DisplayType::Lcd) {
        // 清屏
        display.clear(Rgb565::BLACK).ok();
        
        // 绘制彩色文本
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        Text::with_baseline("LCD Demo", Point::new(10, 10), text_style, Baseline::Top)
            .draw(display).ok();
        
        // 绘制彩色矩形
        Rectangle::new(Point::new(10, 30), Size::new(60, 40))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
            .draw(display).ok();
        
        Rectangle::new(Point::new(80, 30), Size::new(60, 40))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
            .draw(display).ok();
        
        Rectangle::new(Point::new(45, 80), Size::new(60, 40))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::BLUE))
            .draw(display).ok();
        
        // 刷新显示
        display.flush().ok();
    }
}

/// 电子纸演示
fn demo_epaper(system: &mut System) {
    if let Ok(display) = system.display_manager.get_display_mut(DisplayType::EPaper) {
        // 清屏
        display.clear(BinaryColor::Off).ok();
        
        // 绘制标题
        let title_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
        Text::with_baseline("E-Paper Demo", Point::new(10, 10), title_style, Baseline::Top)
            .draw(display).ok();
        
        // 绘制信息文本
        Text::with_baseline("Low Power Display", Point::new(10, 30), title_style, Baseline::Top)
            .draw(display).ok();
        
        // 绘制边框
        Rectangle::new(Point::new(5, 5), Size::new(190, 90))
            .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 2))
            .draw(display).ok();
        
        // 刷新显示（电子纸需要完整刷新）
        display.flush().ok();
    }
}

/// 多显示设备演示
fn demo_multi_display(system: &mut System) {
    let available_displays = system.display_manager.get_available_displays();
    
    for display_type in available_displays {
        if let Ok(display) = system.display_manager.get_display_mut(display_type) {
            // 显示设备信息
            let info_text = match display_type {
                DisplayType::Oled => "OLED Active",
                DisplayType::Lcd => "LCD Active",
                DisplayType::EPaper => "E-Paper Active",
            };
            
            // 根据显示类型选择合适的颜色和样式
            match display_type {
                DisplayType::Oled | DisplayType::EPaper => {
                    display.clear(BinaryColor::Off).ok();
                    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
                    Text::with_baseline(info_text, Point::new(10, 30), style, Baseline::Top)
                        .draw(display).ok();
                },
                DisplayType::Lcd => {
                    display.clear(Rgb565::BLACK).ok();
                    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
                    Text::with_baseline(info_text, Point::new(10, 30), style, Baseline::Top)
                        .draw(display).ok();
                }
            }
            
            display.flush().ok();
        }
        
        system.delay.delay_ms(1000);
    }
}

/// 初始化I2C
fn init_i2c(dp: &/* DevicePeripherals */) -> /* I2C实现 */ {
    // 这里应该根据具体的MCU初始化I2C
    // 示例代码，需要根据实际硬件调整
    todo!("Initialize I2C based on your MCU")
}

/// 初始化SPI
fn init_spi(dp: &/* DevicePeripherals */) -> /* SPI实现 */ {
    // 这里应该根据具体的MCU初始化SPI
    // 示例代码，需要根据实际硬件调整
    todo!("Initialize SPI based on your MCU")
}
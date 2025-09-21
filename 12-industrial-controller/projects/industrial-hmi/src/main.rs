#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    pac,
    gpio::{Pin, Output, PushPull, Input, PullUp},
    spi::{Spi, NoMiso},
    timer::Timer,
    i2c::I2c,
    serial::{Serial, Config},
    otg_fs::{USB, UsbBus},
};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{Circle, PrimitiveStyle, Rectangle},
    text::{Baseline, Text},
};

use ili9341::{Ili9341, DisplaySize240x320};
use xpt2046::Xpt2046;

use heapless::{String, Vec};
use nb::block;

// HMI系统配置
#[derive(Debug, Clone)]
pub struct HmiConfig {
    pub screen_width: u16,
    pub screen_height: u16,
    pub touch_threshold: u16,
    pub update_interval_ms: u32,
    pub alarm_timeout_ms: u32,
    pub data_log_interval_ms: u32,
}

impl Default for HmiConfig {
    fn default() -> Self {
        Self {
            screen_width: 240,
            screen_height: 320,
            touch_threshold: 100,
            update_interval_ms: 100,
            alarm_timeout_ms: 5000,
            data_log_interval_ms: 1000,
        }
    }
}

// 屏幕页面枚举
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScreenPage {
    Main,
    Alarms,
    Trends,
    Settings,
    Diagnostics,
}

// 触摸事件
#[derive(Debug, Clone)]
pub struct TouchEvent {
    pub x: u16,
    pub y: u16,
    pub pressed: bool,
}

// 报警级别
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlarmLevel {
    Info,
    Warning,
    Critical,
    Emergency,
}

// 报警信息
#[derive(Debug, Clone)]
pub struct AlarmInfo {
    pub id: u16,
    pub level: AlarmLevel,
    pub message: String<64>,
    pub timestamp: u32,
    pub acknowledged: bool,
}

// 数据点
#[derive(Debug, Clone)]
pub struct DataPoint {
    pub tag: String<32>,
    pub value: f32,
    pub unit: String<8>,
    pub timestamp: u32,
    pub quality: bool,
}

// 趋势数据
#[derive(Debug, Clone)]
pub struct TrendData {
    pub tag: String<32>,
    pub values: Vec<f32, 100>,
    pub timestamps: Vec<u32, 100>,
    pub min_value: f32,
    pub max_value: f32,
}

// HMI系统主结构
pub struct IndustrialHmi<SPI, CS, DC, RST, TOUCH_CS, TOUCH_IRQ> {
    display: Ili9341<SPI, CS, DC, RST>,
    touch: Xpt2046<SPI, TOUCH_CS>,
    touch_irq: TOUCH_IRQ,
    config: HmiConfig,
    current_page: ScreenPage,
    data_points: Vec<DataPoint, 32>,
    alarms: Vec<AlarmInfo, 16>,
    trends: Vec<TrendData, 8>,
    last_update: u32,
    system_time: u32,
}

impl<SPI, CS, DC, RST, TOUCH_CS, TOUCH_IRQ> IndustrialHmi<SPI, CS, DC, RST, TOUCH_CS, TOUCH_IRQ>
where
    SPI: embedded_hal::spi::SpiDevice,
    CS: OutputPin,
    DC: OutputPin,
    RST: OutputPin,
    TOUCH_CS: OutputPin,
    TOUCH_IRQ: InputPin,
{
    pub fn new(
        spi: SPI,
        cs: CS,
        dc: DC,
        rst: RST,
        touch_cs: TOUCH_CS,
        touch_irq: TOUCH_IRQ,
        config: HmiConfig,
    ) -> Result<Self, &'static str> {
        // 初始化显示屏
        let display = Ili9341::new(spi.clone(), cs, dc, rst, &mut delay, &mut delay)
            .map_err(|_| "Failed to initialize display")?;
        
        // 初始化触摸屏
        let touch = Xpt2046::new(spi, touch_cs);
        
        Ok(Self {
            display,
            touch,
            touch_irq,
            config,
            current_page: ScreenPage::Main,
            data_points: Vec::new(),
            alarms: Vec::new(),
            trends: Vec::new(),
            last_update: 0,
            system_time: 0,
        })
    }
    
    // 系统初始化
    pub fn initialize(&mut self) -> Result<(), &'static str> {
        // 清屏
        self.display.clear(Rgb565::BLACK).map_err(|_| "Failed to clear display")?;
        
        // 显示启动画面
        self.show_startup_screen()?;
        
        // 初始化数据点
        self.initialize_data_points()?;
        
        // 初始化趋势数据
        self.initialize_trends()?;
        
        Ok(())
    }
    
    // 显示启动画面
    fn show_startup_screen(&mut self) -> Result<(), &'static str> {
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        
        Text::with_baseline("Industrial HMI", Point::new(80, 100), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
            
        Text::with_baseline("Version 1.0", Point::new(90, 120), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
            
        Text::with_baseline("Initializing...", Point::new(80, 160), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
            
        Ok(())
    }
    
    // 初始化数据点
    fn initialize_data_points(&mut self) -> Result<(), &'static str> {
        // 添加一些示例数据点
        let data_points = [
            ("TEMP_01", 25.5, "°C"),
            ("PRESS_01", 1.2, "bar"),
            ("FLOW_01", 150.0, "L/min"),
            ("LEVEL_01", 75.0, "%"),
        ];
        
        for (tag, value, unit) in data_points.iter() {
            let mut tag_str = String::new();
            let mut unit_str = String::new();
            
            tag_str.push_str(tag).map_err(|_| "Tag too long")?;
            unit_str.push_str(unit).map_err(|_| "Unit too long")?;
            
            let data_point = DataPoint {
                tag: tag_str,
                value: *value,
                unit: unit_str,
                timestamp: self.system_time,
                quality: true,
            };
            
            self.data_points.push(data_point).map_err(|_| "Too many data points")?;
        }
        
        Ok(())
    }
    
    // 初始化趋势数据
    fn initialize_trends(&mut self) -> Result<(), &'static str> {
        for data_point in &self.data_points {
            let trend = TrendData {
                tag: data_point.tag.clone(),
                values: Vec::new(),
                timestamps: Vec::new(),
                min_value: data_point.value,
                max_value: data_point.value,
            };
            
            self.trends.push(trend).map_err(|_| "Too many trends")?;
        }
        
        Ok(())
    }
    
    // 主运行循环
    pub fn run(&mut self) -> Result<(), &'static str> {
        loop {
            // 更新系统时间
            self.system_time += 1;
            
            // 处理触摸事件
            if let Some(touch_event) = self.read_touch()? {
                self.handle_touch_event(touch_event)?;
            }
            
            // 更新数据
            if self.system_time - self.last_update >= self.config.update_interval_ms {
                self.update_data()?;
                self.update_display()?;
                self.last_update = self.system_time;
            }
            
            // 检查报警
            self.check_alarms()?;
            
            // 延时
            cortex_m::asm::delay(1000);
        }
    }
    
    // 读取触摸事件
    fn read_touch(&mut self) -> Result<Option<TouchEvent>, &'static str> {
        // 检查触摸中断引脚
        if self.touch_irq.is_low().map_err(|_| "Failed to read touch IRQ")? {
            // 读取触摸坐标
            if let Ok(point) = self.touch.read_touch() {
                return Ok(Some(TouchEvent {
                    x: point.x,
                    y: point.y,
                    pressed: true,
                }));
            }
        }
        
        Ok(None)
    }
    
    // 处理触摸事件
    fn handle_touch_event(&mut self, event: TouchEvent) -> Result<(), &'static str> {
        match self.current_page {
            ScreenPage::Main => self.handle_main_page_touch(event)?,
            ScreenPage::Alarms => self.handle_alarms_page_touch(event)?,
            ScreenPage::Trends => self.handle_trends_page_touch(event)?,
            ScreenPage::Settings => self.handle_settings_page_touch(event)?,
            ScreenPage::Diagnostics => self.handle_diagnostics_page_touch(event)?,
        }
        
        Ok(())
    }
    
    // 处理主页面触摸
    fn handle_main_page_touch(&mut self, event: TouchEvent) -> Result<(), &'static str> {
        // 导航按钮区域
        if event.y < 50 {
            if event.x < 60 {
                self.current_page = ScreenPage::Alarms;
            } else if event.x < 120 {
                self.current_page = ScreenPage::Trends;
            } else if event.x < 180 {
                self.current_page = ScreenPage::Settings;
            } else {
                self.current_page = ScreenPage::Diagnostics;
            }
        }
        
        Ok(())
    }
    
    // 处理报警页面触摸
    fn handle_alarms_page_touch(&mut self, event: TouchEvent) -> Result<(), &'static str> {
        // 返回主页按钮
        if event.y < 30 && event.x > 200 {
            self.current_page = ScreenPage::Main;
        }
        
        // 确认报警按钮
        if event.y > 280 && event.x < 120 {
            self.acknowledge_alarms()?;
        }
        
        Ok(())
    }
    
    // 处理趋势页面触摸
    fn handle_trends_page_touch(&mut self, event: TouchEvent) -> Result<(), &'static str> {
        // 返回主页按钮
        if event.y < 30 && event.x > 200 {
            self.current_page = ScreenPage::Main;
        }
        
        Ok(())
    }
    
    // 处理设置页面触摸
    fn handle_settings_page_touch(&mut self, event: TouchEvent) -> Result<(), &'static str> {
        // 返回主页按钮
        if event.y < 30 && event.x > 200 {
            self.current_page = ScreenPage::Main;
        }
        
        Ok(())
    }
    
    // 处理诊断页面触摸
    fn handle_diagnostics_page_touch(&mut self, event: TouchEvent) -> Result<(), &'static str> {
        // 返回主页按钮
        if event.y < 30 && event.x > 200 {
            self.current_page = ScreenPage::Main;
        }
        
        Ok(())
    }
    
    // 更新数据
    fn update_data(&mut self) -> Result<(), &'static str> {
        // 模拟数据更新
        for data_point in &mut self.data_points {
            match data_point.tag.as_str() {
                "TEMP_01" => {
                    data_point.value += (self.system_time as f32 * 0.01).sin() * 2.0;
                },
                "PRESS_01" => {
                    data_point.value += (self.system_time as f32 * 0.02).cos() * 0.1;
                },
                "FLOW_01" => {
                    data_point.value += (self.system_time as f32 * 0.015).sin() * 10.0;
                },
                "LEVEL_01" => {
                    data_point.value += (self.system_time as f32 * 0.005).cos() * 5.0;
                },
                _ => {}
            }
            
            data_point.timestamp = self.system_time;
            
            // 更新趋势数据
            self.update_trend_data(&data_point.tag, data_point.value)?;
        }
        
        Ok(())
    }
    
    // 更新趋势数据
    fn update_trend_data(&mut self, tag: &str, value: f32) -> Result<(), &'static str> {
        for trend in &mut self.trends {
            if trend.tag.as_str() == tag {
                // 添加新值
                if trend.values.len() >= 100 {
                    trend.values.remove(0);
                    trend.timestamps.remove(0);
                }
                
                trend.values.push(value).map_err(|_| "Trend values full")?;
                trend.timestamps.push(self.system_time).map_err(|_| "Trend timestamps full")?;
                
                // 更新最小最大值
                if value < trend.min_value {
                    trend.min_value = value;
                }
                if value > trend.max_value {
                    trend.max_value = value;
                }
                
                break;
            }
        }
        
        Ok(())
    }
    
    // 更新显示
    fn update_display(&mut self) -> Result<(), &'static str> {
        match self.current_page {
            ScreenPage::Main => self.draw_main_page()?,
            ScreenPage::Alarms => self.draw_alarms_page()?,
            ScreenPage::Trends => self.draw_trends_page()?,
            ScreenPage::Settings => self.draw_settings_page()?,
            ScreenPage::Diagnostics => self.draw_diagnostics_page()?,
        }
        
        Ok(())
    }
    
    // 绘制主页面
    fn draw_main_page(&mut self) -> Result<(), &'static str> {
        // 清屏
        self.display.clear(Rgb565::BLACK).map_err(|_| "Failed to clear display")?;
        
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        let header_style = MonoTextStyle::new(&FONT_6X10, Rgb565::YELLOW);
        
        // 绘制标题
        Text::with_baseline("Industrial HMI - Main", Point::new(10, 10), header_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 绘制导航按钮
        self.draw_navigation_buttons()?;
        
        // 绘制数据点
        let mut y_pos = 60;
        for data_point in &self.data_points {
            let mut display_text = String::<64>::new();
            display_text.push_str(&data_point.tag).map_err(|_| "Text too long")?;
            display_text.push_str(": ").map_err(|_| "Text too long")?;
            
            // 格式化数值
            let value_str = format_float(data_point.value);
            display_text.push_str(&value_str).map_err(|_| "Text too long")?;
            display_text.push(' ').map_err(|_| "Text too long")?;
            display_text.push_str(&data_point.unit).map_err(|_| "Text too long")?;
            
            let color = if data_point.quality { Rgb565::GREEN } else { Rgb565::RED };
            let style = MonoTextStyle::new(&FONT_6X10, color);
            
            Text::with_baseline(&display_text, Point::new(10, y_pos), style, Baseline::Top)
                .draw(&mut self.display)
                .map_err(|_| "Failed to draw text")?;
            
            y_pos += 20;
        }
        
        // 绘制报警状态
        self.draw_alarm_status()?;
        
        Ok(())
    }
    
    // 绘制导航按钮
    fn draw_navigation_buttons(&mut self) -> Result<(), &'static str> {
        let button_style = PrimitiveStyle::with_stroke(Rgb565::WHITE, 1);
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        
        // 报警按钮
        Rectangle::new(Point::new(5, 30), Size::new(50, 20))
            .into_styled(button_style)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Alarms", Point::new(10, 35), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 趋势按钮
        Rectangle::new(Point::new(65, 30), Size::new(50, 20))
            .into_styled(button_style)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Trends", Point::new(70, 35), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 设置按钮
        Rectangle::new(Point::new(125, 30), Size::new(50, 20))
            .into_styled(button_style)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Settings", Point::new(130, 35), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 诊断按钮
        Rectangle::new(Point::new(185, 30), Size::new(50, 20))
            .into_styled(button_style)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Diag", Point::new(190, 35), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        Ok(())
    }
    
    // 绘制报警页面
    fn draw_alarms_page(&mut self) -> Result<(), &'static str> {
        // 清屏
        self.display.clear(Rgb565::BLACK).map_err(|_| "Failed to clear display")?;
        
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        let header_style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);
        
        // 绘制标题
        Text::with_baseline("Alarms", Point::new(10, 10), header_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 返回按钮
        Rectangle::new(Point::new(200, 5), Size::new(35, 20))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Back", Point::new(205, 10), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 绘制报警列表
        let mut y_pos = 40;
        for alarm in &self.alarms {
            let color = match alarm.level {
                AlarmLevel::Info => Rgb565::BLUE,
                AlarmLevel::Warning => Rgb565::YELLOW,
                AlarmLevel::Critical => Rgb565::RED,
                AlarmLevel::Emergency => Rgb565::MAGENTA,
            };
            
            let style = MonoTextStyle::new(&FONT_6X10, color);
            
            Text::with_baseline(&alarm.message, Point::new(10, y_pos), style, Baseline::Top)
                .draw(&mut self.display)
                .map_err(|_| "Failed to draw text")?;
            
            y_pos += 15;
            
            if y_pos > 270 {
                break;
            }
        }
        
        // 确认按钮
        Rectangle::new(Point::new(10, 285), Size::new(100, 25))
            .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Acknowledge", Point::new(15, 290), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        Ok(())
    }
    
    // 绘制趋势页面
    fn draw_trends_page(&mut self) -> Result<(), &'static str> {
        // 清屏
        self.display.clear(Rgb565::BLACK).map_err(|_| "Failed to clear display")?;
        
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        let header_style = MonoTextStyle::new(&FONT_6X10, Rgb565::CYAN);
        
        // 绘制标题
        Text::with_baseline("Trends", Point::new(10, 10), header_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 返回按钮
        Rectangle::new(Point::new(200, 5), Size::new(35, 20))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Back", Point::new(205, 10), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 绘制趋势图（简化版）
        if let Some(trend) = self.trends.first() {
            self.draw_trend_chart(trend, Point::new(10, 40), Size::new(220, 100))?;
        }
        
        Ok(())
    }
    
    // 绘制设置页面
    fn draw_settings_page(&mut self) -> Result<(), &'static str> {
        // 清屏
        self.display.clear(Rgb565::BLACK).map_err(|_| "Failed to clear display")?;
        
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        let header_style = MonoTextStyle::new(&FONT_6X10, Rgb565::GREEN);
        
        // 绘制标题
        Text::with_baseline("Settings", Point::new(10, 10), header_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 返回按钮
        Rectangle::new(Point::new(200, 5), Size::new(35, 20))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Back", Point::new(205, 10), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 显示设置项
        let settings = [
            "Update Interval: 100ms",
            "Alarm Timeout: 5000ms",
            "Data Log Interval: 1000ms",
            "Touch Threshold: 100",
        ];
        
        let mut y_pos = 40;
        for setting in &settings {
            Text::with_baseline(setting, Point::new(10, y_pos), text_style, Baseline::Top)
                .draw(&mut self.display)
                .map_err(|_| "Failed to draw text")?;
            y_pos += 20;
        }
        
        Ok(())
    }
    
    // 绘制诊断页面
    fn draw_diagnostics_page(&mut self) -> Result<(), &'static str> {
        // 清屏
        self.display.clear(Rgb565::BLACK).map_err(|_| "Failed to clear display")?;
        
        let text_style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
        let header_style = MonoTextStyle::new(&FONT_6X10, Rgb565::MAGENTA);
        
        // 绘制标题
        Text::with_baseline("Diagnostics", Point::new(10, 10), header_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 返回按钮
        Rectangle::new(Point::new(200, 5), Size::new(35, 20))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        Text::with_baseline("Back", Point::new(205, 10), text_style, Baseline::Top)
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw text")?;
        
        // 显示系统信息
        let diagnostics = [
            "System Time: Running",
            "Display: OK",
            "Touch: OK",
            "Memory: 75% Used",
            "CPU Load: 45%",
        ];
        
        let mut y_pos = 40;
        for diag in &diagnostics {
            Text::with_baseline(diag, Point::new(10, y_pos), text_style, Baseline::Top)
                .draw(&mut self.display)
                .map_err(|_| "Failed to draw text")?;
            y_pos += 20;
        }
        
        Ok(())
    }
    
    // 绘制趋势图
    fn draw_trend_chart(&mut self, trend: &TrendData, position: Point, size: Size) -> Result<(), &'static str> {
        // 绘制图表边框
        Rectangle::new(position, size)
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(&mut self.display)
            .map_err(|_| "Failed to draw rectangle")?;
        
        // 绘制数据点
        if trend.values.len() > 1 {
            let x_scale = size.width as f32 / (trend.values.len() - 1) as f32;
            let y_scale = size.height as f32 / (trend.max_value - trend.min_value);
            
            for i in 1..trend.values.len() {
                let x1 = position.x + ((i - 1) as f32 * x_scale) as i32;
                let y1 = position.y + size.height as i32 - 
                    ((trend.values[i - 1] - trend.min_value) * y_scale) as i32;
                
                let x2 = position.x + (i as f32 * x_scale) as i32;
                let y2 = position.y + size.height as i32 - 
                    ((trend.values[i] - trend.min_value) * y_scale) as i32;
                
                // 简化的线条绘制（使用点）
                Circle::new(Point::new(x2, y2), 1)
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::CYAN))
                    .draw(&mut self.display)
                    .map_err(|_| "Failed to draw circle")?;
            }
        }
        
        Ok(())
    }
    
    // 绘制报警状态
    fn draw_alarm_status(&mut self) -> Result<(), &'static str> {
        let active_alarms = self.alarms.iter().filter(|a| !a.acknowledged).count();
        
        if active_alarms > 0 {
            let alarm_style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);
            let mut alarm_text = String::<32>::new();
            alarm_text.push_str("Active Alarms: ").map_err(|_| "Text too long")?;
            
            // 简化的数字转换
            if active_alarms < 10 {
                alarm_text.push((b'0' + active_alarms as u8) as char).map_err(|_| "Text too long")?;
            } else {
                alarm_text.push_str("9+").map_err(|_| "Text too long")?;
            }
            
            Text::with_baseline(&alarm_text, Point::new(10, 300), alarm_style, Baseline::Top)
                .draw(&mut self.display)
                .map_err(|_| "Failed to draw text")?;
        }
        
        Ok(())
    }
    
    // 检查报警
    fn check_alarms(&mut self) -> Result<(), &'static str> {
        for data_point in &self.data_points {
            // 检查温度报警
            if data_point.tag.as_str() == "TEMP_01" && data_point.value > 30.0 {
                self.add_alarm(
                    1,
                    AlarmLevel::Warning,
                    "High Temperature",
                )?;
            }
            
            // 检查压力报警
            if data_point.tag.as_str() == "PRESS_01" && data_point.value > 1.5 {
                self.add_alarm(
                    2,
                    AlarmLevel::Critical,
                    "High Pressure",
                )?;
            }
        }
        
        Ok(())
    }
    
    // 添加报警
    fn add_alarm(&mut self, id: u16, level: AlarmLevel, message: &str) -> Result<(), &'static str> {
        // 检查是否已存在相同报警
        for alarm in &self.alarms {
            if alarm.id == id && !alarm.acknowledged {
                return Ok(()); // 报警已存在
            }
        }
        
        let mut msg = String::new();
        msg.push_str(message).map_err(|_| "Message too long")?;
        
        let alarm = AlarmInfo {
            id,
            level,
            message: msg,
            timestamp: self.system_time,
            acknowledged: false,
        };
        
        self.alarms.push(alarm).map_err(|_| "Too many alarms")?;
        
        Ok(())
    }
    
    // 确认报警
    fn acknowledge_alarms(&mut self) -> Result<(), &'static str> {
        for alarm in &mut self.alarms {
            alarm.acknowledged = true;
        }
        
        Ok(())
    }
    
    // 获取系统状态
    pub fn get_system_status(&self) -> SystemStatus {
        SystemStatus {
            current_page: self.current_page,
            active_alarms: self.alarms.iter().filter(|a| !a.acknowledged).count() as u16,
            data_points_count: self.data_points.len() as u16,
            system_time: self.system_time,
            memory_usage: 75, // 模拟值
        }
    }
}

// 系统状态
#[derive(Debug, Clone)]
pub struct SystemStatus {
    pub current_page: ScreenPage,
    pub active_alarms: u16,
    pub data_points_count: u16,
    pub system_time: u32,
    pub memory_usage: u8,
}

// 浮点数格式化函数（简化版）
fn format_float(value: f32) -> String<16> {
    let mut result = String::new();
    
    // 简化的浮点数转换
    let integer_part = value as i32;
    let fractional_part = ((value - integer_part as f32) * 10.0) as i32;
    
    // 转换整数部分
    if integer_part == 0 {
        result.push('0').ok();
    } else {
        let mut temp = integer_part.abs();
        let mut digits = String::<8>::new();
        
        while temp > 0 {
            digits.push((b'0' + (temp % 10) as u8) as char).ok();
            temp /= 10;
        }
        
        if integer_part < 0 {
            result.push('-').ok();
        }
        
        // 反转数字
        for ch in digits.chars().rev() {
            result.push(ch).ok();
        }
    }
    
    // 添加小数点和小数部分
    result.push('.').ok();
    result.push((b'0' + fractional_part.abs() as u8) as char).ok();
    
    result
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();
    
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    // 配置SPI1用于显示屏和触摸屏
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    
    let spi = Spi::new(
        dp.SPI1,
        (sck, miso, mosi),
        embedded_hal::spi::MODE_0,
        1.MHz(),
        &clocks,
    );
    
    // 显示屏控制引脚
    let display_cs = gpiob.pb0.into_push_pull_output();
    let display_dc = gpiob.pb1.into_push_pull_output();
    let display_rst = gpiob.pb2.into_push_pull_output();
    
    // 触摸屏控制引脚
    let touch_cs = gpioc.pc0.into_push_pull_output();
    let touch_irq = gpioc.pc1.into_pull_up_input();
    
    // 创建HMI系统
    let config = HmiConfig::default();
    let mut hmi = IndustrialHmi::new(
        spi,
        display_cs,
        display_dc,
        display_rst,
        touch_cs,
        touch_irq,
        config,
    ).unwrap();
    
    // 初始化系统
    hmi.initialize().unwrap();
    
    // 运行主循环
    hmi.run().unwrap();
    
    loop {}
}
#![no_std]
#![no_main]

use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m::delay::Delay;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    gpio::{Output, Pin, PushPull, Input, PullUp},
    pac,
    prelude::*,
    timer::{CounterUs, Event},
    spi::{Spi, NoMiso},
};

use heapless::{Vec, String};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
    primitives::{Circle, Line, Rectangle, PrimitiveStyle},
};
use micromath::F32Ext;

// 系统配置常量
const SYSTEM_CLOCK_HZ: u32 = 168_000_000;
const MATRIX_WIDTH: usize = 32;
const MATRIX_HEIGHT: usize = 16;
const REFRESH_RATE_HZ: u32 = 100;
const BRIGHTNESS_LEVELS: u8 = 16;

// SPI配置
const SPI_FREQUENCY: u32 = 10_000_000; // 10MHz

// 动画配置
const SCROLL_SPEED_MS: u32 = 100;
const ANIMATION_FRAME_MS: u32 = 50;

// 类型别名
type LedPin = Pin<'A', 0, Output<PushPull>>;
type ButtonPin = Pin<'C', 13, Input<PullUp>>;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("LED矩阵显示系统启动");

    // 初始化外设
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置系统时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())
        .sysclk(168.MHz())
        .require_pll48clk()
        .freeze();

    rprintln!("系统时钟配置完成: SYSCLK={}MHz", clocks.sysclk().raw() / 1_000_000);

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // LED矩阵控制引脚
    // 行选择引脚 (PA0-PA3, 4位地址，支持16行)
    let row_pins = [
        gpioa.pa0.into_push_pull_output().erase(),
        gpioa.pa1.into_push_pull_output().erase(),
        gpioa.pa2.into_push_pull_output().erase(),
        gpioa.pa3.into_push_pull_output().erase(),
    ];

    // 列数据引脚 (PB0-PB7, 8位数据，支持8列)
    let col_pins = [
        gpiob.pb0.into_push_pull_output().erase(),
        gpiob.pb1.into_push_pull_output().erase(),
        gpiob.pb2.into_push_pull_output().erase(),
        gpiob.pb3.into_push_pull_output().erase(),
        gpiob.pb4.into_push_pull_output().erase(),
        gpiob.pb5.into_push_pull_output().erase(),
        gpiob.pb6.into_push_pull_output().erase(),
        gpiob.pb7.into_push_pull_output().erase(),
    ];

    // 控制引脚
    let latch_pin = gpioa.pa4.into_push_pull_output(); // 锁存
    let clock_pin = gpioa.pa5.into_push_pull_output(); // 时钟
    let enable_pin = gpioa.pa6.into_push_pull_output(); // 使能
    let reset_pin = gpioa.pa7.into_push_pull_output();  // 复位

    // 按钮输入
    let mode_button = gpioc.pc13.into_pull_up_input();
    let brightness_button = gpioc.pc14.into_pull_up_input();
    let speed_button = gpioc.pc15.into_pull_up_input();

    // 配置SPI（可选，用于高速数据传输）
    let sck = gpioa.pa5.into_alternate();
    let mosi = gpioa.pa7.into_alternate();
    let spi = Spi::new(
        dp.SPI1,
        (sck, NoMiso::new(), mosi),
        embedded_hal::spi::MODE_0,
        SPI_FREQUENCY.Hz(),
        &clocks,
    );

    // 配置定时器
    let mut timer = dp.TIM2.counter_us(&clocks);
    timer.start((1000000 / REFRESH_RATE_HZ).micros()).unwrap();

    // 创建延时对象
    let mut delay = Delay::new(cp.SYST, clocks.sysclk().raw());

    // 创建LED矩阵控制器
    let mut matrix_controller = LedMatrixController::new(
        row_pins,
        col_pins,
        latch_pin,
        clock_pin,
        enable_pin,
        reset_pin,
    );

    // 创建显示缓冲区
    let mut display_buffer = DisplayBuffer::new();

    // 创建按钮管理器
    let mut button_manager = ButtonManager::new(
        mode_button,
        brightness_button,
        speed_button,
    );

    // 创建动画管理器
    let mut animation_manager = AnimationManager::new();

    // 创建文本渲染器
    let mut text_renderer = TextRenderer::new();

    rprintln!("LED矩阵系统初始化完成");
    rprintln!("矩阵尺寸: {}x{}", MATRIX_WIDTH, MATRIX_HEIGHT);

    let mut last_time = 0u32;
    let mut frame_count = 0u32;
    let mut current_mode = DisplayMode::Text;
    let mut brightness = 8u8;
    let mut animation_speed = 1u8;

    // 初始化显示内容
    text_renderer.set_text("Hello LED Matrix!");
    display_buffer.clear();

    loop {
        // 检查定时器
        if timer.wait().is_ok() {
            last_time = last_time.wrapping_add(1000 / REFRESH_RATE_HZ);
        }

        // 更新按钮状态
        button_manager.update();

        // 处理按钮事件
        if button_manager.is_mode_pressed() {
            current_mode = match current_mode {
                DisplayMode::Text => DisplayMode::Graphics,
                DisplayMode::Graphics => DisplayMode::Animation,
                DisplayMode::Animation => DisplayMode::Game,
                DisplayMode::Game => DisplayMode::Clock,
                DisplayMode::Clock => DisplayMode::Text,
            };
            rprintln!("切换显示模式: {:?}", current_mode);
            display_buffer.clear();
        }

        if button_manager.is_brightness_pressed() {
            brightness = if brightness >= BRIGHTNESS_LEVELS { 1 } else { brightness + 1 };
            matrix_controller.set_brightness(brightness);
            rprintln!("亮度调节: {}/{}", brightness, BRIGHTNESS_LEVELS);
        }

        if button_manager.is_speed_pressed() {
            animation_speed = if animation_speed >= 5 { 1 } else { animation_speed + 1 };
            animation_manager.set_speed(animation_speed);
            rprintln!("动画速度: {}", animation_speed);
        }

        // 根据当前模式更新显示内容
        match current_mode {
            DisplayMode::Text => {
                text_renderer.update(last_time);
                text_renderer.render(&mut display_buffer);
            }
            DisplayMode::Graphics => {
                render_graphics_demo(&mut display_buffer, last_time);
            }
            DisplayMode::Animation => {
                animation_manager.update(last_time);
                animation_manager.render(&mut display_buffer);
            }
            DisplayMode::Game => {
                render_game_demo(&mut display_buffer, last_time);
            }
            DisplayMode::Clock => {
                render_clock(&mut display_buffer, last_time);
            }
        }

        // 刷新LED矩阵显示
        matrix_controller.refresh(&display_buffer);

        // 性能统计
        frame_count += 1;
        if frame_count % (REFRESH_RATE_HZ * 5) == 0 {
            rprintln!("显示统计: 帧数={}, 时间={}ms, 模式={:?}", 
                     frame_count, last_time, current_mode);
            rprintln!("系统状态: 亮度={}, 速度={}", brightness, animation_speed);
        }

        // 短暂延时
        delay.delay_us(100);
    }
}

// LED矩阵控制器
struct LedMatrixController {
    row_pins: [Pin<'A', 0, Output<PushPull>>; 4],
    col_pins: [Pin<'B', 0, Output<PushPull>>; 8],
    latch_pin: Pin<'A', 4, Output<PushPull>>,
    clock_pin: Pin<'A', 5, Output<PushPull>>,
    enable_pin: Pin<'A', 6, Output<PushPull>>,
    reset_pin: Pin<'A', 7, Output<PushPull>>,
    current_row: usize,
    brightness: u8,
    pwm_counter: u8,
}

impl LedMatrixController {
    fn new(
        row_pins: [Pin<'A', 0, Output<PushPull>>; 4],
        col_pins: [Pin<'B', 0, Output<PushPull>>; 8],
        latch_pin: Pin<'A', 4, Output<PushPull>>,
        clock_pin: Pin<'A', 5, Output<PushPull>>,
        enable_pin: Pin<'A', 6, Output<PushPull>>,
        reset_pin: Pin<'A', 7, Output<PushPull>>,
    ) -> Self {
        let mut controller = Self {
            row_pins,
            col_pins,
            latch_pin,
            clock_pin,
            enable_pin,
            reset_pin,
            current_row: 0,
            brightness: 8,
            pwm_counter: 0,
        };
        
        controller.initialize();
        controller
    }

    fn initialize(&mut self) {
        // 复位矩阵
        self.reset_pin.set_low();
        cortex_m::asm::delay(1000);
        self.reset_pin.set_high();
        
        // 禁用显示
        self.enable_pin.set_high();
        
        // 清空所有输出
        self.clear_all_pins();
        
        rprintln!("LED矩阵控制器初始化完成");
    }

    fn refresh(&mut self, buffer: &DisplayBuffer) {
        // PWM亮度控制
        self.pwm_counter = (self.pwm_counter + 1) % BRIGHTNESS_LEVELS;
        let should_display = self.pwm_counter < self.brightness;

        if should_display {
            self.display_row(buffer);
        } else {
            self.enable_pin.set_high(); // 禁用显示
        }

        // 切换到下一行
        self.current_row = (self.current_row + 1) % MATRIX_HEIGHT;
    }

    fn display_row(&mut self, buffer: &DisplayBuffer) {
        // 禁用显示
        self.enable_pin.set_high();

        // 设置行地址
        self.set_row_address(self.current_row);

        // 输出列数据
        self.output_column_data(buffer, self.current_row);

        // 锁存数据
        self.latch_data();

        // 启用显示
        self.enable_pin.set_low();
    }

    fn set_row_address(&mut self, row: usize) {
        for (i, pin) in self.row_pins.iter_mut().enumerate() {
            if (row >> i) & 1 == 1 {
                pin.set_high();
            } else {
                pin.set_low();
            }
        }
    }

    fn output_column_data(&mut self, buffer: &DisplayBuffer, row: usize) {
        // 输出32列数据，分4次输出（每次8列）
        for chunk in 0..4 {
            let start_col = chunk * 8;
            
            for bit in 0..8 {
                let col = start_col + bit;
                let pixel_on = if col < MATRIX_WIDTH && row < MATRIX_HEIGHT {
                    buffer.get_pixel(col, row)
                } else {
                    false
                };

                if pixel_on {
                    self.col_pins[bit].set_high();
                } else {
                    self.col_pins[bit].set_low();
                }
            }

            // 时钟脉冲
            self.clock_pulse();
        }
    }

    fn latch_data(&mut self) {
        self.latch_pin.set_high();
        cortex_m::asm::delay(10);
        self.latch_pin.set_low();
    }

    fn clock_pulse(&mut self) {
        self.clock_pin.set_high();
        cortex_m::asm::delay(5);
        self.clock_pin.set_low();
        cortex_m::asm::delay(5);
    }

    fn clear_all_pins(&mut self) {
        for pin in &mut self.row_pins {
            pin.set_low();
        }
        for pin in &mut self.col_pins {
            pin.set_low();
        }
        self.latch_pin.set_low();
        self.clock_pin.set_low();
    }

    fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness.min(BRIGHTNESS_LEVELS);
    }
}

// 显示缓冲区
struct DisplayBuffer {
    buffer: [[bool; MATRIX_WIDTH]; MATRIX_HEIGHT],
}

impl DisplayBuffer {
    fn new() -> Self {
        Self {
            buffer: [[false; MATRIX_WIDTH]; MATRIX_HEIGHT],
        }
    }

    fn clear(&mut self) {
        for row in &mut self.buffer {
            for pixel in row {
                *pixel = false;
            }
        }
    }

    fn set_pixel(&mut self, x: usize, y: usize, on: bool) {
        if x < MATRIX_WIDTH && y < MATRIX_HEIGHT {
            self.buffer[y][x] = on;
        }
    }

    fn get_pixel(&self, x: usize, y: usize) -> bool {
        if x < MATRIX_WIDTH && y < MATRIX_HEIGHT {
            self.buffer[y][x]
        } else {
            false
        }
    }

    fn draw_line(&mut self, x0: usize, y0: usize, x1: usize, y1: usize, on: bool) {
        let dx = if x1 > x0 { x1 - x0 } else { x0 - x1 };
        let dy = if y1 > y0 { y1 - y0 } else { y0 - y1 };
        let sx = if x0 < x1 { 1 } else { -1 };
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut err = dx as i32 - dy as i32;
        
        let mut x = x0 as i32;
        let mut y = y0 as i32;

        loop {
            self.set_pixel(x as usize, y as usize, on);
            
            if x == x1 as i32 && y == y1 as i32 {
                break;
            }
            
            let e2 = 2 * err;
            if e2 > -(dy as i32) {
                err -= dy as i32;
                x += sx;
            }
            if e2 < dx as i32 {
                err += dx as i32;
                y += sy;
            }
        }
    }

    fn draw_circle(&mut self, cx: usize, cy: usize, radius: usize, on: bool) {
        let mut x = 0i32;
        let mut y = radius as i32;
        let mut d = 3 - 2 * radius as i32;

        while y >= x {
            self.draw_circle_points(cx as i32, cy as i32, x, y, on);
            x += 1;
            if d > 0 {
                y -= 1;
                d = d + 4 * (x - y) + 10;
            } else {
                d = d + 4 * x + 6;
            }
        }
    }

    fn draw_circle_points(&mut self, cx: i32, cy: i32, x: i32, y: i32, on: bool) {
        let points = [
            (cx + x, cy + y), (cx - x, cy + y),
            (cx + x, cy - y), (cx - x, cy - y),
            (cx + y, cy + x), (cx - y, cy + x),
            (cx + y, cy - x), (cx - y, cy - x),
        ];

        for (px, py) in points {
            if px >= 0 && py >= 0 {
                self.set_pixel(px as usize, py as usize, on);
            }
        }
    }

    fn draw_rectangle(&mut self, x: usize, y: usize, width: usize, height: usize, filled: bool, on: bool) {
        if filled {
            for dy in 0..height {
                for dx in 0..width {
                    self.set_pixel(x + dx, y + dy, on);
                }
            }
        } else {
            // 绘制边框
            for dx in 0..width {
                self.set_pixel(x + dx, y, on);
                self.set_pixel(x + dx, y + height - 1, on);
            }
            for dy in 0..height {
                self.set_pixel(x, y + dy, on);
                self.set_pixel(x + width - 1, y + dy, on);
            }
        }
    }
}

// 显示模式枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum DisplayMode {
    Text,      // 文本显示
    Graphics,  // 图形演示
    Animation, // 动画效果
    Game,      // 游戏演示
    Clock,     // 时钟显示
}

// 文本渲染器
struct TextRenderer {
    text: String<64>,
    scroll_offset: i32,
    last_update: u32,
}

impl TextRenderer {
    fn new() -> Self {
        Self {
            text: String::new(),
            scroll_offset: 0,
            last_update: 0,
        }
    }

    fn set_text(&mut self, text: &str) {
        self.text.clear();
        self.text.push_str(text).ok();
        self.scroll_offset = MATRIX_WIDTH as i32;
    }

    fn update(&mut self, current_time: u32) {
        if current_time.wrapping_sub(self.last_update) >= SCROLL_SPEED_MS {
            self.scroll_offset -= 1;
            if self.scroll_offset < -(self.text.len() as i32 * 6) {
                self.scroll_offset = MATRIX_WIDTH as i32;
            }
            self.last_update = current_time;
        }
    }

    fn render(&self, buffer: &mut DisplayBuffer) {
        // 简单的5x7字体渲染
        for (i, ch) in self.text.chars().enumerate() {
            let x_pos = self.scroll_offset + (i as i32 * 6);
            if x_pos >= -(5i32) && x_pos < MATRIX_WIDTH as i32 {
                self.render_char(buffer, ch, x_pos, 4);
            }
        }
    }

    fn render_char(&self, buffer: &mut DisplayBuffer, ch: char, x: i32, y: i32) {
        let font_data = get_char_font_data(ch);
        
        for (row, &row_data) in font_data.iter().enumerate() {
            for col in 0..5 {
                if (row_data >> (4 - col)) & 1 == 1 {
                    let px = x + col as i32;
                    let py = y + row as i32;
                    if px >= 0 && px < MATRIX_WIDTH as i32 && py >= 0 && py < MATRIX_HEIGHT as i32 {
                        buffer.set_pixel(px as usize, py as usize, true);
                    }
                }
            }
        }
    }
}

// 动画管理器
struct AnimationManager {
    current_animation: AnimationType,
    frame_counter: u32,
    last_update: u32,
    speed: u8,
}

impl AnimationManager {
    fn new() -> Self {
        Self {
            current_animation: AnimationType::Bouncing,
            frame_counter: 0,
            last_update: 0,
            speed: 1,
        }
    }

    fn set_speed(&mut self, speed: u8) {
        self.speed = speed;
    }

    fn update(&mut self, current_time: u32) {
        let frame_interval = ANIMATION_FRAME_MS / self.speed as u32;
        if current_time.wrapping_sub(self.last_update) >= frame_interval {
            self.frame_counter += 1;
            self.last_update = current_time;
        }
    }

    fn render(&self, buffer: &mut DisplayBuffer) {
        match self.current_animation {
            AnimationType::Bouncing => self.render_bouncing_ball(buffer),
            AnimationType::Spiral => self.render_spiral(buffer),
            AnimationType::Wave => self.render_wave(buffer),
            AnimationType::Rain => self.render_rain(buffer),
        }
    }

    fn render_bouncing_ball(&self, buffer: &mut DisplayBuffer) {
        let t = (self.frame_counter % 120) as f32;
        let x = ((t * 0.1).sin() * 12.0 + 16.0) as usize;
        let y = ((t * 0.15).sin().abs() * 12.0 + 2.0) as usize;
        
        buffer.draw_circle(x, y, 2, true);
    }

    fn render_spiral(&self, buffer: &mut DisplayBuffer) {
        let center_x = MATRIX_WIDTH / 2;
        let center_y = MATRIX_HEIGHT / 2;
        
        for i in 0..50 {
            let t = (self.frame_counter as f32 * 0.1 + i as f32 * 0.3) % (2.0 * 3.14159);
            let radius = (i as f32 * 0.3) % 10.0;
            let x = (center_x as f32 + radius * t.cos()) as usize;
            let y = (center_y as f32 + radius * t.sin()) as usize;
            
            if x < MATRIX_WIDTH && y < MATRIX_HEIGHT {
                buffer.set_pixel(x, y, true);
            }
        }
    }

    fn render_wave(&self, buffer: &mut DisplayBuffer) {
        for x in 0..MATRIX_WIDTH {
            let t = self.frame_counter as f32 * 0.1;
            let y = ((x as f32 * 0.3 + t).sin() * 4.0 + 8.0) as usize;
            if y < MATRIX_HEIGHT {
                buffer.set_pixel(x, y, true);
            }
        }
    }

    fn render_rain(&self, buffer: &mut DisplayBuffer) {
        // 简单的雨滴效果
        for i in 0..10 {
            let x = (i * 3 + 2) % MATRIX_WIDTH;
            let y = ((self.frame_counter + i * 7) % (MATRIX_HEIGHT * 2)) as usize;
            if y < MATRIX_HEIGHT {
                buffer.set_pixel(x, y, true);
                if y > 0 {
                    buffer.set_pixel(x, y - 1, true);
                }
            }
        }
    }
}

// 动画类型枚举
#[derive(Debug, Clone, Copy, PartialEq)]
enum AnimationType {
    Bouncing,
    Spiral,
    Wave,
    Rain,
}

// 按钮管理器
struct ButtonManager {
    mode_button: Pin<'C', 13, Input<PullUp>>,
    brightness_button: Pin<'C', 14, Input<PullUp>>,
    speed_button: Pin<'C', 15, Input<PullUp>>,
    mode_pressed: bool,
    brightness_pressed: bool,
    speed_pressed: bool,
    mode_last_state: bool,
    brightness_last_state: bool,
    speed_last_state: bool,
}

impl ButtonManager {
    fn new(
        mode_button: Pin<'C', 13, Input<PullUp>>,
        brightness_button: Pin<'C', 14, Input<PullUp>>,
        speed_button: Pin<'C', 15, Input<PullUp>>,
    ) -> Self {
        Self {
            mode_button,
            brightness_button,
            speed_button,
            mode_pressed: false,
            brightness_pressed: false,
            speed_pressed: false,
            mode_last_state: true,
            brightness_last_state: true,
            speed_last_state: true,
        }
    }

    fn update(&mut self) {
        let mode_current = self.mode_button.is_high();
        let brightness_current = self.brightness_button.is_high();
        let speed_current = self.speed_button.is_high();

        self.mode_pressed = self.mode_last_state && !mode_current;
        self.brightness_pressed = self.brightness_last_state && !brightness_current;
        self.speed_pressed = self.speed_last_state && !speed_current;

        self.mode_last_state = mode_current;
        self.brightness_last_state = brightness_current;
        self.speed_last_state = speed_current;
    }

    fn is_mode_pressed(&mut self) -> bool {
        let pressed = self.mode_pressed;
        self.mode_pressed = false;
        pressed
    }

    fn is_brightness_pressed(&mut self) -> bool {
        let pressed = self.brightness_pressed;
        self.brightness_pressed = false;
        pressed
    }

    fn is_speed_pressed(&mut self) -> bool {
        let pressed = self.speed_pressed;
        self.speed_pressed = false;
        pressed
    }
}

// 图形演示函数
fn render_graphics_demo(buffer: &mut DisplayBuffer, time: u32) {
    buffer.clear();
    
    let phase = (time / 1000) % 4;
    
    match phase {
        0 => {
            // 绘制矩形
            buffer.draw_rectangle(2, 2, 8, 6, false, true);
            buffer.draw_rectangle(12, 4, 6, 4, true, true);
            buffer.draw_rectangle(22, 1, 4, 8, false, true);
        }
        1 => {
            // 绘制圆形
            buffer.draw_circle(8, 8, 6, true);
            buffer.draw_circle(24, 8, 4, true);
        }
        2 => {
            // 绘制线条
            buffer.draw_line(0, 0, 31, 15, true);
            buffer.draw_line(0, 15, 31, 0, true);
            buffer.draw_line(16, 0, 16, 15, true);
            buffer.draw_line(0, 8, 31, 8, true);
        }
        3 => {
            // 组合图形
            buffer.draw_circle(16, 8, 7, true);
            buffer.draw_rectangle(12, 4, 8, 8, false, true);
        }
        _ => {}
    }
}

// 游戏演示函数
fn render_game_demo(buffer: &mut DisplayBuffer, time: u32) {
    buffer.clear();
    
    // 简单的贪吃蛇游戏演示
    let snake_length = 5;
    let head_x = ((time / 200) % (MATRIX_WIDTH - snake_length)) as usize;
    let head_y = 8;
    
    // 绘制蛇身
    for i in 0..snake_length {
        buffer.set_pixel(head_x + i, head_y, true);
    }
    
    // 绘制食物
    let food_x = (head_x + 10) % MATRIX_WIDTH;
    let food_y = 4;
    buffer.set_pixel(food_x, food_y, true);
    buffer.set_pixel(food_x, food_y + 1, true);
    buffer.set_pixel(food_x + 1, food_y, true);
    buffer.set_pixel(food_x + 1, food_y + 1, true);
}

// 时钟显示函数
fn render_clock(buffer: &mut DisplayBuffer, time: u32) {
    buffer.clear();
    
    // 简单的数字时钟显示
    let seconds = (time / 1000) % 60;
    let minutes = (time / 60000) % 60;
    
    // 显示分钟（十位）
    render_digit(buffer, (minutes / 10) as u8, 2, 4);
    // 显示分钟（个位）
    render_digit(buffer, (minutes % 10) as u8, 8, 4);
    
    // 显示冒号
    buffer.set_pixel(14, 6, true);
    buffer.set_pixel(14, 9, true);
    
    // 显示秒钟（十位）
    render_digit(buffer, (seconds / 10) as u8, 16, 4);
    // 显示秒钟（个位）
    render_digit(buffer, (seconds % 10) as u8, 22, 4);
}

// 数字渲染函数
fn render_digit(buffer: &mut DisplayBuffer, digit: u8, x: usize, y: usize) {
    let digit_patterns = [
        // 0
        [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        // 1
        [0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        // 2
        [0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111],
        // 3
        [0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110],
        // 4
        [0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010],
        // 5
        [0b11111, 0b10000, 0b11110, 0b00001, 0b00001, 0b10001, 0b01110],
        // 6
        [0b01110, 0b10001, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110],
        // 7
        [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000],
        // 8
        [0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110],
        // 9
        [0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b10001, 0b01110],
    ];
    
    if digit < 10 {
        let pattern = digit_patterns[digit as usize];
        for (row, &row_data) in pattern.iter().enumerate() {
            for col in 0..5 {
                if (row_data >> (4 - col)) & 1 == 1 {
                    buffer.set_pixel(x + col, y + row, true);
                }
            }
        }
    }
}

// 字符字体数据
fn get_char_font_data(ch: char) -> [u8; 7] {
    match ch {
        'A' => [0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'B' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110],
        'C' => [0b01110, 0b10001, 0b10000, 0b10000, 0b10000, 0b10001, 0b01110],
        'D' => [0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110],
        'E' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111],
        'F' => [0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000],
        'G' => [0b01110, 0b10001, 0b10000, 0b10111, 0b10001, 0b10001, 0b01110],
        'H' => [0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001],
        'I' => [0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110],
        'J' => [0b00111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100],
        'K' => [0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001],
        'L' => [0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111],
        'M' => [0b10001, 0b11011, 0b10101, 0b10101, 0b10001, 0b10001, 0b10001],
        'N' => [0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001],
        'O' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'P' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000],
        'Q' => [0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101],
        'R' => [0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001],
        'S' => [0b01110, 0b10001, 0b10000, 0b01110, 0b00001, 0b10001, 0b01110],
        'T' => [0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100],
        'U' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110],
        'V' => [0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100],
        'W' => [0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001],
        'X' => [0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001],
        'Y' => [0b10001, 0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100],
        'Z' => [0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111],
        ' ' => [0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000],
        '!' => [0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00000, 0b00100],
        '?' => [0b01110, 0b10001, 0b00001, 0b00110, 0b00100, 0b00000, 0b00100],
        _ => [0b11111, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11111], // 默认字符
    }
}

// 测试模块
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_display_buffer() {
        let mut buffer = DisplayBuffer::new();
        buffer.set_pixel(0, 0, true);
        assert_eq!(buffer.get_pixel(0, 0), true);
        assert_eq!(buffer.get_pixel(1, 1), false);
    }

    #[test]
    fn test_graphics_functions() {
        let mut buffer = DisplayBuffer::new();
        buffer.draw_line(0, 0, 5, 5, true);
        buffer.draw_circle(10, 8, 3, true);
        buffer.draw_rectangle(20, 5, 4, 6, false, true);
        
        // 验证一些关键点
        assert_eq!(buffer.get_pixel(0, 0), true);
        assert_eq!(buffer.get_pixel(5, 5), true);
    }
}
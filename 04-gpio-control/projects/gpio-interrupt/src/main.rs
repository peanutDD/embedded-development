#![no_std]
#![no_main]

use panic_halt as _;
use rtic::app;
use defmt_rtt as _;

use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Pin, Output, PushPull, Input, PullUp, PullDown, Edge},
    timer::Timer,
    rcc::Clocks,
    interrupt,
};

use heapless::{Vec, FnvIndexMap};
use cortex_m::peripheral::NVIC;
use rtic_monotonics::systick::*;

// 中断事件类型
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum InterruptEvent {
    ButtonPress,
    ButtonRelease,
    ButtonLongPress,
    MotionDetected,
    MotionStopped,
    EncoderClockwise,
    EncoderCounterClockwise,
    TimerExpired,
}

// 中断统计信息
#[derive(Clone, Copy, Debug)]
pub struct InterruptStats {
    pub count: u32,
    pub last_timestamp: u64,
    pub min_interval: u32,
    pub max_interval: u32,
    pub avg_interval: u32,
}

impl InterruptStats {
    pub fn new() -> Self {
        Self {
            count: 0,
            last_timestamp: 0,
            min_interval: u32::MAX,
            max_interval: 0,
            avg_interval: 0,
        }
    }

    pub fn update(&mut self, timestamp: u64) {
        if self.count > 0 {
            let interval = (timestamp - self.last_timestamp) as u32;
            if interval < self.min_interval {
                self.min_interval = interval;
            }
            if interval > self.max_interval {
                self.max_interval = interval;
            }
            // 简单移动平均
            self.avg_interval = (self.avg_interval * 7 + interval) / 8;
        }
        
        self.count += 1;
        self.last_timestamp = timestamp;
    }
}

// 按钮去抖动器
pub struct ButtonDebouncer {
    state: bool,
    last_state: bool,
    debounce_time: u32,
    last_change_time: u64,
    press_time: u64,
    long_press_threshold: u32,
    long_press_triggered: bool,
}

impl ButtonDebouncer {
    pub fn new(debounce_time_ms: u32, long_press_threshold_ms: u32) -> Self {
        Self {
            state: false,
            last_state: false,
            debounce_time: debounce_time_ms,
            last_change_time: 0,
            press_time: 0,
            long_press_threshold: long_press_threshold_ms,
            long_press_triggered: false,
        }
    }

    pub fn update(&mut self, raw_state: bool, timestamp: u64) -> Option<InterruptEvent> {
        if raw_state != self.last_state {
            self.last_change_time = timestamp;
            self.last_state = raw_state;
        }

        if timestamp - self.last_change_time >= self.debounce_time as u64 {
            if raw_state != self.state {
                self.state = raw_state;
                
                if self.state {
                    // 按钮按下
                    self.press_time = timestamp;
                    self.long_press_triggered = false;
                    return Some(InterruptEvent::ButtonPress);
                } else {
                    // 按钮释放
                    if !self.long_press_triggered {
                        return Some(InterruptEvent::ButtonRelease);
                    }
                }
            }
        }

        // 检查长按
        if self.state && !self.long_press_triggered {
            if timestamp - self.press_time >= self.long_press_threshold as u64 {
                self.long_press_triggered = true;
                return Some(InterruptEvent::ButtonLongPress);
            }
        }

        None
    }

    pub fn is_pressed(&self) -> bool {
        self.state
    }
}

// 旋转编码器
pub struct RotaryEncoder {
    last_a: bool,
    last_b: bool,
    position: i32,
    direction: i8,
}

impl RotaryEncoder {
    pub fn new() -> Self {
        Self {
            last_a: false,
            last_b: false,
            position: 0,
            direction: 0,
        }
    }

    pub fn update(&mut self, a_state: bool, b_state: bool) -> Option<InterruptEvent> {
        if a_state != self.last_a {
            if a_state {
                // A信号上升沿
                if b_state {
                    self.position -= 1;
                    self.direction = -1;
                    return Some(InterruptEvent::EncoderCounterClockwise);
                } else {
                    self.position += 1;
                    self.direction = 1;
                    return Some(InterruptEvent::EncoderClockwise);
                }
            }
        }
        
        self.last_a = a_state;
        self.last_b = b_state;
        None
    }

    pub fn get_position(&self) -> i32 {
        self.position
    }

    pub fn reset_position(&mut self) {
        self.position = 0;
    }
}

// 运动检测器
pub struct MotionDetector {
    is_active: bool,
    last_trigger_time: u64,
    timeout_ms: u32,
}

impl MotionDetector {
    pub fn new(timeout_ms: u32) -> Self {
        Self {
            is_active: false,
            last_trigger_time: 0,
            timeout_ms,
        }
    }

    pub fn trigger(&mut self, timestamp: u64) -> Option<InterruptEvent> {
        self.last_trigger_time = timestamp;
        
        if !self.is_active {
            self.is_active = true;
            return Some(InterruptEvent::MotionDetected);
        }
        None
    }

    pub fn update(&mut self, timestamp: u64) -> Option<InterruptEvent> {
        if self.is_active && timestamp - self.last_trigger_time >= self.timeout_ms as u64 {
            self.is_active = false;
            return Some(InterruptEvent::MotionStopped);
        }
        None
    }

    pub fn is_active(&self) -> bool {
        self.is_active
    }
}

// 事件处理器
pub struct EventHandler {
    led_states: [bool; 4],
    encoder_value: i32,
    motion_count: u32,
    button_count: u32,
}

impl EventHandler {
    pub fn new() -> Self {
        Self {
            led_states: [false; 4],
            encoder_value: 0,
            motion_count: 0,
            button_count: 0,
        }
    }

    pub fn handle_event(&mut self, event: InterruptEvent) {
        match event {
            InterruptEvent::ButtonPress => {
                self.button_count += 1;
                self.led_states[0] = !self.led_states[0];
                defmt::info!("按钮按下 - 计数: {}, LED0: {}", self.button_count, self.led_states[0]);
            }
            InterruptEvent::ButtonLongPress => {
                // 长按重置所有状态
                self.led_states = [false; 4];
                self.encoder_value = 0;
                self.motion_count = 0;
                self.button_count = 0;
                defmt::info!("长按检测 - 系统重置");
            }
            InterruptEvent::EncoderClockwise => {
                self.encoder_value += 1;
                self.led_states[1] = true;
                defmt::info!("编码器顺时针 - 值: {}", self.encoder_value);
            }
            InterruptEvent::EncoderCounterClockwise => {
                self.encoder_value -= 1;
                self.led_states[1] = true;
                defmt::info!("编码器逆时针 - 值: {}", self.encoder_value);
            }
            InterruptEvent::MotionDetected => {
                self.motion_count += 1;
                self.led_states[2] = true;
                defmt::info!("运动检测 - 计数: {}", self.motion_count);
            }
            InterruptEvent::MotionStopped => {
                self.led_states[2] = false;
                defmt::info!("运动停止");
            }
            InterruptEvent::TimerExpired => {
                self.led_states[3] = !self.led_states[3];
                defmt::info!("定时器中断 - LED3: {}", self.led_states[3]);
            }
            _ => {}
        }
    }

    pub fn get_led_states(&self) -> [bool; 4] {
        self.led_states
    }

    pub fn get_status(&self) -> (i32, u32, u32) {
        (self.encoder_value, self.motion_count, self.button_count)
    }
}

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        event_handler: EventHandler,
        interrupt_stats: FnvIndexMap<InterruptEvent, InterruptStats, 8>,
    }

    #[local]
    struct Local {
        // GPIO引脚
        led0: Pin<'C', 13, Output<PushPull>>,
        led1: Pin<'C', 14, Output<PushPull>>,
        led2: Pin<'C', 15, Output<PushPull>>,
        led3: Pin<'A', 8, Output<PushPull>>,
        
        // 输入引脚
        button: Pin<'A', 0, Input<PullUp>>,
        encoder_a: Pin<'A', 1, Input<PullUp>>,
        encoder_b: Pin<'A', 2, Input<PullUp>>,
        motion_sensor: Pin<'A', 3, Input<PullDown>>,
        
        // 处理器
        button_debouncer: ButtonDebouncer,
        rotary_encoder: RotaryEncoder,
        motion_detector: MotionDetector,
        
        // 定时器
        timer: Timer<pac::TIM2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        defmt::info!("GPIO中断系统初始化");

        let dp = ctx.device;
        let cp = ctx.core;

        // 配置时钟
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

        // 初始化系统定时器
        let systick_token = rtic_monotonics::create_systick_token!();
        Systick::start(cp.SYST, 84_000_000, systick_token);

        // 配置GPIO
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();

        // LED输出引脚
        let mut led0 = gpioc.pc13.into_push_pull_output();
        let mut led1 = gpioc.pc14.into_push_pull_output();
        let mut led2 = gpioc.pc15.into_push_pull_output();
        let mut led3 = gpioa.pa8.into_push_pull_output();

        // 初始化LED状态
        led0.set_low();
        led1.set_low();
        led2.set_low();
        led3.set_low();

        // 输入引脚配置
        let mut button = gpioa.pa0.into_pull_up_input();
        let mut encoder_a = gpioa.pa1.into_pull_up_input();
        let mut encoder_b = gpioa.pa2.into_pull_up_input();
        let mut motion_sensor = gpioa.pa3.into_pull_down_input();

        // 配置外部中断
        button.make_interrupt_source(&mut dp.SYSCFG.constrain());
        button.enable_interrupt(&mut dp.EXTI);
        button.trigger_on_edge(&mut dp.EXTI, Edge::RisingFalling);

        encoder_a.make_interrupt_source(&mut dp.SYSCFG.constrain());
        encoder_a.enable_interrupt(&mut dp.EXTI);
        encoder_a.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        motion_sensor.make_interrupt_source(&mut dp.SYSCFG.constrain());
        motion_sensor.enable_interrupt(&mut dp.EXTI);
        motion_sensor.trigger_on_edge(&mut dp.EXTI, Edge::Rising);

        // 配置定时器
        let mut timer = Timer::new(dp.TIM2, &clocks);
        timer.start(1.secs());
        timer.listen();

        // 启用中断
        unsafe {
            NVIC::unmask(pac::Interrupt::EXTI0);
            NVIC::unmask(pac::Interrupt::EXTI1);
            NVIC::unmask(pac::Interrupt::EXTI3);
            NVIC::unmask(pac::Interrupt::TIM2);
        }

        // 创建处理器
        let button_debouncer = ButtonDebouncer::new(50, 1000); // 50ms去抖，1s长按
        let rotary_encoder = RotaryEncoder::new();
        let motion_detector = MotionDetector::new(5000); // 5s超时

        // 启动周期性任务
        periodic_task::spawn().ok();
        status_report::spawn().ok();

        defmt::info!("GPIO中断系统初始化完成");
        defmt::info!("按钮: PA0 (上拉输入)");
        defmt::info!("编码器A: PA1 (上拉输入)");
        defmt::info!("编码器B: PA2 (上拉输入)");
        defmt::info!("运动传感器: PA3 (下拉输入)");
        defmt::info!("LED0-3: PC13-15, PA8");

        (
            Shared {
                event_handler: EventHandler::new(),
                interrupt_stats: FnvIndexMap::new(),
            },
            Local {
                led0,
                led1,
                led2,
                led3,
                button,
                encoder_a,
                encoder_b,
                motion_sensor,
                button_debouncer,
                rotary_encoder,
                motion_detector,
                timer,
            },
        )
    }

    // 按钮中断处理
    #[task(binds = EXTI0, local = [button, button_debouncer], shared = [event_handler, interrupt_stats])]
    fn button_interrupt(mut ctx: button_interrupt::Context) {
        let timestamp = Systick::now().ticks();
        let button_state = ctx.local.button.is_low();
        
        if let Some(event) = ctx.local.button_debouncer.update(button_state, timestamp) {
            ctx.shared.event_handler.lock(|handler| {
                handler.handle_event(event);
            });
            
            ctx.shared.interrupt_stats.lock(|stats| {
                stats.entry(event).or_insert_with(InterruptStats::new).update(timestamp);
            });
        }
        
        ctx.local.button.clear_interrupt_pending_bit();
    }

    // 编码器中断处理
    #[task(binds = EXTI1, local = [encoder_a, encoder_b, rotary_encoder], shared = [event_handler, interrupt_stats])]
    fn encoder_interrupt(mut ctx: encoder_interrupt::Context) {
        let timestamp = Systick::now().ticks();
        let a_state = ctx.local.encoder_a.is_high();
        let b_state = ctx.local.encoder_b.is_high();
        
        if let Some(event) = ctx.local.rotary_encoder.update(a_state, b_state) {
            ctx.shared.event_handler.lock(|handler| {
                handler.handle_event(event);
            });
            
            ctx.shared.interrupt_stats.lock(|stats| {
                stats.entry(event).or_insert_with(InterruptStats::new).update(timestamp);
            });
        }
        
        ctx.local.encoder_a.clear_interrupt_pending_bit();
    }

    // 运动传感器中断处理
    #[task(binds = EXTI3, local = [motion_sensor, motion_detector], shared = [event_handler, interrupt_stats])]
    fn motion_interrupt(mut ctx: motion_interrupt::Context) {
        let timestamp = Systick::now().ticks();
        
        if let Some(event) = ctx.local.motion_detector.trigger(timestamp) {
            ctx.shared.event_handler.lock(|handler| {
                handler.handle_event(event);
            });
            
            ctx.shared.interrupt_stats.lock(|stats| {
                stats.entry(event).or_insert_with(InterruptStats::new).update(timestamp);
            });
        }
        
        ctx.local.motion_sensor.clear_interrupt_pending_bit();
    }

    // 定时器中断处理
    #[task(binds = TIM2, local = [timer], shared = [event_handler, interrupt_stats])]
    fn timer_interrupt(mut ctx: timer_interrupt::Context) {
        let timestamp = Systick::now().ticks();
        let event = InterruptEvent::TimerExpired;
        
        ctx.shared.event_handler.lock(|handler| {
            handler.handle_event(event);
        });
        
        ctx.shared.interrupt_stats.lock(|stats| {
            stats.entry(event).or_insert_with(InterruptStats::new).update(timestamp);
        });
        
        ctx.local.timer.clear_interrupt_flag();
    }

    // 周期性任务 - 更新LED状态和运动检测
    #[task(local = [led0, led1, led2, led3, motion_detector], shared = [event_handler])]
    async fn periodic_task(mut ctx: periodic_task::Context) {
        loop {
            let timestamp = Systick::now().ticks();
            
            // 检查运动检测超时
            if let Some(event) = ctx.local.motion_detector.update(timestamp) {
                ctx.shared.event_handler.lock(|handler| {
                    handler.handle_event(event);
                });
            }
            
            // 更新LED状态
            let led_states = ctx.shared.event_handler.lock(|handler| {
                handler.get_led_states()
            });
            
            if led_states[0] { ctx.local.led0.set_high(); } else { ctx.local.led0.set_low(); }
            if led_states[1] { ctx.local.led1.set_high(); } else { ctx.local.led1.set_low(); }
            if led_states[2] { ctx.local.led2.set_high(); } else { ctx.local.led2.set_low(); }
            if led_states[3] { ctx.local.led3.set_high(); } else { ctx.local.led3.set_low(); }
            
            // 编码器LED自动熄灭
            if led_states[1] {
                Systick::delay(100.millis()).await;
                ctx.local.led1.set_low();
            }
            
            Systick::delay(50.millis()).await;
        }
    }

    // 状态报告任务
    #[task(shared = [event_handler, interrupt_stats])]
    async fn status_report(mut ctx: status_report::Context) {
        loop {
            Systick::delay(5.secs()).await;
            
            let (encoder_value, motion_count, button_count) = ctx.shared.event_handler.lock(|handler| {
                handler.get_status()
            });
            
            defmt::info!("=== 系统状态报告 ===");
            defmt::info!("编码器值: {}", encoder_value);
            defmt::info!("运动检测次数: {}", motion_count);
            defmt::info!("按钮按下次数: {}", button_count);
            
            ctx.shared.interrupt_stats.lock(|stats| {
                defmt::info!("=== 中断统计 ===");
                for (event, stat) in stats.iter() {
                    defmt::info!(
                        "{:?}: 次数={}, 最小间隔={}ms, 最大间隔={}ms, 平均间隔={}ms",
                        event, stat.count, stat.min_interval, stat.max_interval, stat.avg_interval
                    );
                }
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_button_debouncer() {
        let mut debouncer = ButtonDebouncer::new(50, 1000);
        
        // 测试按钮按下
        let event = debouncer.update(true, 0);
        assert_eq!(event, None); // 还在去抖时间内
        
        let event = debouncer.update(true, 60);
        assert_eq!(event, Some(InterruptEvent::ButtonPress));
        
        // 测试长按
        let event = debouncer.update(true, 1100);
        assert_eq!(event, Some(InterruptEvent::ButtonLongPress));
    }

    #[test]
    fn test_rotary_encoder() {
        let mut encoder = RotaryEncoder::new();
        
        // 测试顺时针旋转
        let event = encoder.update(true, false);
        assert_eq!(event, Some(InterruptEvent::EncoderClockwise));
        assert_eq!(encoder.get_position(), 1);
        
        // 测试逆时针旋转
        encoder.update(false, false);
        let event = encoder.update(true, true);
        assert_eq!(event, Some(InterruptEvent::EncoderCounterClockwise));
        assert_eq!(encoder.get_position(), 0);
    }

    #[test]
    fn test_motion_detector() {
        let mut detector = MotionDetector::new(1000);
        
        // 测试运动检测
        let event = detector.trigger(0);
        assert_eq!(event, Some(InterruptEvent::MotionDetected));
        assert!(detector.is_active());
        
        // 测试超时
        let event = detector.update(1100);
        assert_eq!(event, Some(InterruptEvent::MotionStopped));
        assert!(!detector.is_active());
    }

    #[test]
    fn test_interrupt_stats() {
        let mut stats = InterruptStats::new();
        
        stats.update(0);
        assert_eq!(stats.count, 1);
        
        stats.update(100);
        assert_eq!(stats.count, 2);
        assert_eq!(stats.min_interval, 100);
        assert_eq!(stats.max_interval, 100);
    }
}
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Counter},
    gpio::{gpiob::{PB0, PB1, PB2, PB3}, Output, PushPull, Input, PullUp},
    gpio::gpioc::PC13,
};
use cortex_m::peripheral::DWT;
use micromath::F32Ext;

type StatusLed = PB0<Output<PushPull>>;
type TimingLed = PB1<Output<PushPull>>;
type MeasureLed = PB2<Output<PushPull>>;
type ErrorLed = PB3<Output<PushPull>>;
type TriggerButton = PC13<Input<PullUp>>;

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 启用DWT计数器用于精确计时
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();

    // 配置GPIO
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    
    let mut status_led = gpiob.pb0.into_push_pull_output();
    let mut timing_led = gpiob.pb1.into_push_pull_output();
    let mut measure_led = gpiob.pb2.into_push_pull_output();
    let mut error_led = gpiob.pb3.into_push_pull_output();
    let trigger_button = gpioc.pc13.into_pull_up_input();

    // 配置高精度定时器
    let mut precision_timer = Timer::tim2(dp.TIM2, &clocks);
    precision_timer.start(1.mhz()); // 1MHz = 1μs分辨率

    // 配置基准定时器
    let mut reference_timer = Timer::tim3(dp.TIM3, &clocks);
    reference_timer.start(10.khz()); // 10kHz = 100μs分辨率

    // 创建延时定时器
    let mut delay = Timer::tim4(dp.TIM4, &clocks).delay_us();

    // 系统启动指示
    status_led.set_high();
    delay.delay_ms(1000u32);

    // 精确计时器
    let mut precise_timer = PreciseTimer::new();
    let mut timing_measurements = TimingMeasurements::new();
    let mut button_debouncer = ButtonDebouncer::new();

    // 演示模式状态
    let mut demo_mode = DemoMode::Microsecond;
    let mut demo_counter = 0u32;

    loop {
        // 按钮防抖处理
        let button_pressed = button_debouncer.update(trigger_button.is_low());

        if button_pressed {
            // 切换演示模式
            demo_mode = match demo_mode {
                DemoMode::Microsecond => DemoMode::Millisecond,
                DemoMode::Millisecond => DemoMode::Measurement,
                DemoMode::Measurement => DemoMode::Comparison,
                DemoMode::Comparison => DemoMode::Microsecond,
            };

            // 模式切换指示
            for _ in 0..3 {
                status_led.set_low();
                delay.delay_ms(100u32);
                status_led.set_high();
                delay.delay_ms(100u32);
            }
        }

        match demo_mode {
            DemoMode::Microsecond => {
                // 微秒级精确计时演示
                microsecond_timing_demo(
                    &mut precise_timer,
                    &mut timing_led,
                    &mut measure_led,
                    &mut delay,
                );
            }

            DemoMode::Millisecond => {
                // 毫秒级精确计时演示
                millisecond_timing_demo(
                    &mut precise_timer,
                    &mut timing_led,
                    &mut measure_led,
                    &mut delay,
                );
            }

            DemoMode::Measurement => {
                // 计时测量演示
                timing_measurement_demo(
                    &mut precise_timer,
                    &mut timing_measurements,
                    &mut timing_led,
                    &mut measure_led,
                    &mut error_led,
                    &mut delay,
                );
            }

            DemoMode::Comparison => {
                // 计时方法比较演示
                timing_comparison_demo(
                    &mut precise_timer,
                    &mut timing_measurements,
                    &mut timing_led,
                    &mut measure_led,
                    &mut delay,
                );
            }
        }

        demo_counter += 1;

        // 状态LED心跳
        if demo_counter % 100 == 0 {
            status_led.toggle();
        }

        delay.delay_ms(10u32);
    }
}

/// 微秒级精确计时演示
fn microsecond_timing_demo(
    timer: &mut PreciseTimer,
    timing_led: &mut impl embedded_hal::digital::v2::OutputPin,
    measure_led: &mut impl embedded_hal::digital::v2::OutputPin,
    delay: &mut impl embedded_hal::blocking::delay::DelayUs<u32>,
) {
    // 精确延时测试：10μs, 50μs, 100μs, 500μs
    let delays = [10, 50, 100, 500];
    
    for &delay_us in &delays {
        timing_led.set_high().ok();
        
        timer.start();
        delay.delay_us(delay_us);
        let measured_time = timer.elapsed_us();
        
        timing_led.set_low().ok();
        
        // 测量结果指示
        let error_percent = ((measured_time as f32 - delay_us as f32).abs() / delay_us as f32) * 100.0;
        
        if error_percent < 5.0 {
            // 误差小于5%，测量LED短闪
            measure_led.set_high().ok();
            delay.delay_ms(100u32);
            measure_led.set_low().ok();
        } else {
            // 误差较大，测量LED长闪
            measure_led.set_high().ok();
            delay.delay_ms(500u32);
            measure_led.set_low().ok();
        }
        
        delay.delay_ms(200u32);
    }
}

/// 毫秒级精确计时演示
fn millisecond_timing_demo(
    timer: &mut PreciseTimer,
    timing_led: &mut impl embedded_hal::digital::v2::OutputPin,
    measure_led: &mut impl embedded_hal::digital::v2::OutputPin,
    delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32>,
) {
    // 精确延时测试：1ms, 5ms, 10ms, 50ms
    let delays = [1, 5, 10, 50];
    
    for &delay_ms in &delays {
        timing_led.set_high().ok();
        
        timer.start();
        delay.delay_ms(delay_ms);
        let measured_time = timer.elapsed_ms();
        
        timing_led.set_low().ok();
        
        // 测量结果指示
        let error_percent = ((measured_time as f32 - delay_ms as f32).abs() / delay_ms as f32) * 100.0;
        
        // 根据误差闪烁不同次数
        let blink_count = if error_percent < 1.0 { 1 } else if error_percent < 5.0 { 2 } else { 3 };
        
        for _ in 0..blink_count {
            measure_led.set_high().ok();
            delay.delay_ms(100u32);
            measure_led.set_low().ok();
            delay.delay_ms(100u32);
        }
        
        delay.delay_ms(300u32);
    }
}

/// 计时测量演示
fn timing_measurement_demo(
    timer: &mut PreciseTimer,
    measurements: &mut TimingMeasurements,
    timing_led: &mut impl embedded_hal::digital::v2::OutputPin,
    measure_led: &mut impl embedded_hal::digital::v2::OutputPin,
    error_led: &mut impl embedded_hal::digital::v2::OutputPin,
    delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32>,
) {
    // 执行一系列计时测量
    for i in 0..10 {
        timing_led.set_high().ok();
        
        timer.start();
        
        // 模拟不同的工作负载
        let workload_time = 1 + (i * 2); // 1, 3, 5, 7, 9, 11, 13, 15, 17, 19 ms
        delay.delay_ms(workload_time);
        
        let measured_time = timer.elapsed_ms();
        timing_led.set_low().ok();
        
        // 记录测量结果
        measurements.add_measurement(measured_time);
        
        // 测量指示
        measure_led.set_high().ok();
        delay.delay_ms(50u32);
        measure_led.set_low().ok();
        
        delay.delay_ms(100u32);
    }
    
    // 分析测量结果
    let stats = measurements.get_statistics();
    
    // 根据统计结果显示LED
    if stats.max_time - stats.min_time > 5 {
        // 时间变化较大，错误LED闪烁
        for _ in 0..3 {
            error_led.set_high().ok();
            delay.delay_ms(200u32);
            error_led.set_low().ok();
            delay.delay_ms(200u32);
        }
    }
    
    measurements.clear();
}

/// 计时方法比较演示
fn timing_comparison_demo(
    timer: &mut PreciseTimer,
    measurements: &mut TimingMeasurements,
    timing_led: &mut impl embedded_hal::digital::v2::OutputPin,
    measure_led: &mut impl embedded_hal::digital::v2::OutputPin,
    delay: &mut impl embedded_hal::blocking::delay::DelayMs<u32>,
) {
    // 比较不同计时方法的精度
    
    // 方法1：DWT计数器
    timing_led.set_high().ok();
    let dwt_start = DWT::cycle_count();
    delay.delay_ms(10u32);
    let dwt_cycles = DWT::cycle_count().wrapping_sub(dwt_start);
    let dwt_time_us = dwt_cycles / 84; // 84MHz时钟
    timing_led.set_low().ok();
    
    delay.delay_ms(100u32);
    
    // 方法2：精确定时器
    timing_led.set_high().ok();
    timer.start();
    delay.delay_ms(10u32);
    let timer_time_us = timer.elapsed_us();
    timing_led.set_low().ok();
    
    // 比较结果指示
    let difference = (dwt_time_us as i32 - timer_time_us as i32).abs();
    
    if difference < 100 {
        // 差异小于100μs，测量LED单次闪烁
        measure_led.set_high().ok();
        delay.delay_ms(200u32);
        measure_led.set_low().ok();
    } else {
        // 差异较大，测量LED多次闪烁
        for _ in 0..3 {
            measure_led.set_high().ok();
            delay.delay_ms(100u32);
            measure_led.set_low().ok();
            delay.delay_ms(100u32);
        }
    }
    
    delay.delay_ms(500u32);
}

/// 精确计时器
struct PreciseTimer {
    start_cycles: u32,
}

impl PreciseTimer {
    fn new() -> Self {
        Self { start_cycles: 0 }
    }
    
    fn start(&mut self) {
        self.start_cycles = DWT::cycle_count();
    }
    
    fn elapsed_cycles(&self) -> u32 {
        DWT::cycle_count().wrapping_sub(self.start_cycles)
    }
    
    fn elapsed_us(&self) -> u32 {
        self.elapsed_cycles() / 84 // 84MHz时钟
    }
    
    fn elapsed_ms(&self) -> u32 {
        self.elapsed_us() / 1000
    }
}

/// 计时测量统计
struct TimingMeasurements {
    measurements: heapless::Vec<u32, 32>,
}

impl TimingMeasurements {
    fn new() -> Self {
        Self {
            measurements: heapless::Vec::new(),
        }
    }
    
    fn add_measurement(&mut self, time_ms: u32) {
        self.measurements.push(time_ms).ok();
    }
    
    fn get_statistics(&self) -> TimingStatistics {
        if self.measurements.is_empty() {
            return TimingStatistics::default();
        }
        
        let mut min_time = u32::MAX;
        let mut max_time = 0;
        let mut sum = 0;
        
        for &time in &self.measurements {
            min_time = min_time.min(time);
            max_time = max_time.max(time);
            sum += time;
        }
        
        let avg_time = sum / self.measurements.len() as u32;
        
        TimingStatistics {
            min_time,
            max_time,
            avg_time,
            count: self.measurements.len() as u32,
        }
    }
    
    fn clear(&mut self) {
        self.measurements.clear();
    }
}

/// 计时统计结果
#[derive(Default)]
struct TimingStatistics {
    min_time: u32,
    max_time: u32,
    avg_time: u32,
    count: u32,
}

/// 按钮防抖器
struct ButtonDebouncer {
    last_state: bool,
    stable_state: bool,
    counter: u8,
}

impl ButtonDebouncer {
    fn new() -> Self {
        Self {
            last_state: false,
            stable_state: false,
            counter: 0,
        }
    }
    
    fn update(&mut self, current_state: bool) -> bool {
        if current_state != self.last_state {
            self.counter = 0;
            self.last_state = current_state;
        } else {
            if self.counter < 5 {
                self.counter += 1;
            }
        }
        
        if self.counter >= 5 && current_state != self.stable_state {
            self.stable_state = current_state;
            return current_state; // 返回按下事件
        }
        
        false
    }
}

/// 演示模式
#[derive(Clone, Copy)]
enum DemoMode {
    Microsecond,
    Millisecond,
    Measurement,
    Comparison,
}
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin, Input, PullUp},
    timer::{Timer, Event, CounterUs},
    rcc::Clocks,
    interrupt,
};
use heapless::spsc::{Queue, Producer, Consumer};
use heapless::{Vec, Deque};
use rtt_target::{rprintln, rtt_init_print};
use input_capture::{CaptureEvent, EdgeType, CaptureMode};
use libm::{fabsf, sqrtf, fminf, fmaxf};
use cortex_m::peripheral::NVIC;

// 脉宽测量事件类型
#[derive(Debug, Clone, Copy)]
enum PulseWidthEvent {
    PulseStart(u32),         // 脉冲开始
    PulseEnd(u32),           // 脉冲结束
    MeasurementComplete(PulseWidthResult), // 测量完成
    CalibrationRequest,      // 校准请求
    ModeChange(PulseMode),   // 模式切换
    ThresholdUpdate(f32),    // 阈值更新
}

#[derive(Debug, Clone, Copy)]
enum PulseMode {
    SinglePulse,     // 单脉冲测量
    ContinuousPulse, // 连续脉冲测量
    DutyCycle,       // 占空比测量
    PulseWidth,      // 脉宽测量
    Period,          // 周期测量
    Statistical,     // 统计分析
}

#[derive(Debug, Clone, Copy)]
struct PulseWidthResult {
    pulse_width_us: f32,
    period_us: f32,
    duty_cycle: f32,
    frequency_hz: f32,
    rise_time_us: f32,
    fall_time_us: f32,
}

// 脉宽测量器
struct PulseWidthMeter {
    pulse_start_time: Option<u32>,
    pulse_end_time: Option<u32>,
    period_start_time: Option<u32>,
    last_pulse_width: f32,
    last_period: f32,
    last_duty_cycle: f32,
    pulse_buffer: Deque<f32, 100>,
    period_buffer: Deque<f32, 100>,
    timer_frequency: u32,
    measurement_active: bool,
    edge_count: u32,
}

impl PulseWidthMeter {
    fn new(timer_frequency: u32) -> Self {
        Self {
            pulse_start_time: None,
            pulse_end_time: None,
            period_start_time: None,
            last_pulse_width: 0.0,
            last_period: 0.0,
            last_duty_cycle: 0.0,
            pulse_buffer: Deque::new(),
            period_buffer: Deque::new(),
            timer_frequency,
            measurement_active: false,
            edge_count: 0,
        }
    }

    fn start_measurement(&mut self) {
        self.measurement_active = true;
        self.pulse_start_time = None;
        self.pulse_end_time = None;
        self.period_start_time = None;
        self.edge_count = 0;
    }

    fn stop_measurement(&mut self) {
        self.measurement_active = false;
    }

    fn process_edge(&mut self, timestamp: u32, edge_type: EdgeType) -> Option<PulseWidthResult> {
        if !self.measurement_active {
            return None;
        }

        self.edge_count += 1;

        match edge_type {
            EdgeType::Rising => {
                // 上升沿：脉冲开始，周期开始
                if let Some(period_start) = self.period_start_time {
                    // 计算周期
                    let period_ticks = timestamp.wrapping_sub(period_start);
                    let period_us = (period_ticks as f32) / (self.timer_frequency as f32) * 1_000_000.0;
                    self.last_period = period_us;
                    
                    if self.period_buffer.len() >= 100 {
                        self.period_buffer.pop_front();
                    }
                    let _ = self.period_buffer.push_back(period_us);
                }
                
                self.pulse_start_time = Some(timestamp);
                self.period_start_time = Some(timestamp);
            }
            EdgeType::Falling => {
                // 下降沿：脉冲结束
                if let Some(start_time) = self.pulse_start_time {
                    let pulse_ticks = timestamp.wrapping_sub(start_time);
                    let pulse_width_us = (pulse_ticks as f32) / (self.timer_frequency as f32) * 1_000_000.0;
                    self.last_pulse_width = pulse_width_us;
                    
                    if self.pulse_buffer.len() >= 100 {
                        self.pulse_buffer.pop_front();
                    }
                    let _ = self.pulse_buffer.push_back(pulse_width_us);
                    
                    // 计算占空比
                    if self.last_period > 0.0 {
                        self.last_duty_cycle = pulse_width_us / self.last_period;
                    }
                    
                    self.pulse_end_time = Some(timestamp);
                    
                    // 返回测量结果
                    return Some(PulseWidthResult {
                        pulse_width_us,
                        period_us: self.last_period,
                        duty_cycle: self.last_duty_cycle,
                        frequency_hz: if self.last_period > 0.0 { 1_000_000.0 / self.last_period } else { 0.0 },
                        rise_time_us: 0.0, // 简化，实际需要更精确的边沿检测
                        fall_time_us: 0.0,
                    });
                }
            }
        }

        None
    }

    fn get_statistics(&self) -> PulseStatistics {
        let mut stats = PulseStatistics::default();
        
        if !self.pulse_buffer.is_empty() {
            let mut sum = 0.0;
            let mut min_val = f32::MAX;
            let mut max_val = f32::MIN;
            
            for &pulse_width in self.pulse_buffer.iter() {
                sum += pulse_width;
                min_val = fminf(min_val, pulse_width);
                max_val = fmaxf(max_val, pulse_width);
            }
            
            stats.avg_pulse_width = sum / (self.pulse_buffer.len() as f32);
            stats.min_pulse_width = min_val;
            stats.max_pulse_width = max_val;
            
            // 计算标准差
            let mut variance_sum = 0.0;
            for &pulse_width in self.pulse_buffer.iter() {
                let diff = pulse_width - stats.avg_pulse_width;
                variance_sum += diff * diff;
            }
            stats.pulse_width_jitter = sqrtf(variance_sum / (self.pulse_buffer.len() as f32));
        }
        
        if !self.period_buffer.is_empty() {
            let mut sum = 0.0;
            for &period in self.period_buffer.iter() {
                sum += period;
            }
            stats.avg_period = sum / (self.period_buffer.len() as f32);
            stats.avg_frequency = if stats.avg_period > 0.0 { 1_000_000.0 / stats.avg_period } else { 0.0 };
        }
        
        stats.sample_count = self.pulse_buffer.len() as u32;
        stats.edge_count = self.edge_count;
        
        stats
    }

    fn reset(&mut self) {
        self.pulse_buffer.clear();
        self.period_buffer.clear();
        self.pulse_start_time = None;
        self.pulse_end_time = None;
        self.period_start_time = None;
        self.edge_count = 0;
    }
}

#[derive(Debug, Default)]
struct PulseStatistics {
    avg_pulse_width: f32,
    min_pulse_width: f32,
    max_pulse_width: f32,
    pulse_width_jitter: f32,
    avg_period: f32,
    avg_frequency: f32,
    sample_count: u32,
    edge_count: u32,
}

// 脉宽测量管理器
struct PulseWidthManager {
    pulse_meter1: PulseWidthMeter,
    pulse_meter2: PulseWidthMeter,
    current_mode: PulseMode,
    measurement_active: bool,
    calibration_active: bool,
    threshold_voltage: f32,
    statistics: PulseWidthStatistics,
    event_producer: Producer<'static, PulseWidthEvent, 64>,
}

#[derive(Debug, Default)]
struct PulseWidthStatistics {
    total_measurements: u32,
    successful_measurements: u32,
    failed_measurements: u32,
    total_pulses: u32,
    measurement_time_ms: u32,
    calibration_count: u32,
    mode_changes: u32,
    threshold_changes: u32,
}

impl PulseWidthManager {
    fn new(
        timer_frequency: u32,
        event_producer: Producer<'static, PulseWidthEvent, 64>,
    ) -> Self {
        Self {
            pulse_meter1: PulseWidthMeter::new(timer_frequency),
            pulse_meter2: PulseWidthMeter::new(timer_frequency),
            current_mode: PulseMode::SinglePulse,
            measurement_active: false,
            calibration_active: false,
            threshold_voltage: 2.5, // 2.5V默认阈值
            statistics: PulseWidthStatistics::default(),
            event_producer,
        }
    }

    fn set_mode(&mut self, mode: PulseMode) {
        self.current_mode = mode;
        self.statistics.mode_changes += 1;
        rprintln!("脉宽测量模式切换到: {:?}", mode);
    }

    fn start_measurement(&mut self) {
        self.measurement_active = true;
        self.pulse_meter1.start_measurement();
        self.pulse_meter2.start_measurement();
        rprintln!("开始脉宽测量");
    }

    fn stop_measurement(&mut self) {
        self.measurement_active = false;
        self.pulse_meter1.stop_measurement();
        self.pulse_meter2.stop_measurement();
        rprintln!("停止脉宽测量");
    }

    fn process_capture(&mut self, channel: u8, timestamp: u32, edge_type: EdgeType) -> Result<(), &'static str> {
        if !self.measurement_active {
            return Ok(());
        }

        let result = match channel {
            1 => self.pulse_meter1.process_edge(timestamp, edge_type),
            2 => self.pulse_meter2.process_edge(timestamp, edge_type),
            _ => return Err("无效的通道ID"),
        };

        if let Some(pulse_result) = result {
            self.statistics.successful_measurements += 1;
            self.statistics.total_pulses += 1;
            
            // 根据模式处理结果
            match self.current_mode {
                PulseMode::SinglePulse => {
                    rprintln!("单脉冲测量 - 脉宽: {:.2} us", pulse_result.pulse_width_us);
                    self.send_event(PulseWidthEvent::MeasurementComplete(pulse_result));
                }
                PulseMode::ContinuousPulse => {
                    // 连续测量，定期报告
                    if self.statistics.total_pulses % 10 == 0 {
                        rprintln!("连续脉冲测量 - 脉宽: {:.2} us, 频率: {:.2} Hz", 
                                 pulse_result.pulse_width_us, pulse_result.frequency_hz);
                    }
                }
                PulseMode::DutyCycle => {
                    rprintln!("占空比测量 - 占空比: {:.1}%, 周期: {:.2} us", 
                             pulse_result.duty_cycle * 100.0, pulse_result.period_us);
                }
                PulseMode::PulseWidth => {
                    rprintln!("脉宽测量 - 脉宽: {:.2} us", pulse_result.pulse_width_us);
                }
                PulseMode::Period => {
                    rprintln!("周期测量 - 周期: {:.2} us, 频率: {:.2} Hz", 
                             pulse_result.period_us, pulse_result.frequency_hz);
                }
                PulseMode::Statistical => {
                    // 统计模式，收集数据后分析
                    if self.statistics.total_pulses % 50 == 0 {
                        let stats = self.pulse_meter1.get_statistics();
                        rprintln!("统计分析 - 平均脉宽: {:.2} us, 抖动: {:.3} us", 
                                 stats.avg_pulse_width, stats.pulse_width_jitter);
                    }
                }
            }
        }

        self.statistics.total_measurements += 1;
        Ok(())
    }

    fn update_measurements(&mut self, dt_ms: u32) -> Result<(), &'static str> {
        self.statistics.measurement_time_ms += dt_ms;
        Ok(())
    }

    fn set_threshold(&mut self, threshold: f32) {
        self.threshold_voltage = threshold;
        self.statistics.threshold_changes += 1;
        rprintln!("阈值电压设置为: {:.2} V", threshold);
    }

    fn start_calibration(&mut self) {
        self.calibration_active = true;
        self.statistics.calibration_count += 1;
        rprintln!("开始脉宽测量校准");
    }

    fn end_calibration(&mut self) {
        self.calibration_active = false;
        rprintln!("脉宽测量校准完成");
    }

    fn get_current_result(&self) -> PulseWidthResult {
        PulseWidthResult {
            pulse_width_us: self.pulse_meter1.last_pulse_width,
            period_us: self.pulse_meter1.last_period,
            duty_cycle: self.pulse_meter1.last_duty_cycle,
            frequency_hz: if self.pulse_meter1.last_period > 0.0 { 
                1_000_000.0 / self.pulse_meter1.last_period 
            } else { 
                0.0 
            },
            rise_time_us: 0.0,
            fall_time_us: 0.0,
        }
    }

    fn send_event(&mut self, event: PulseWidthEvent) {
        if self.event_producer.enqueue(event).is_err() {
            rprintln!("警告: 事件队列已满");
        }
    }
}

static mut PULSE_EVENT_QUEUE: Queue<PulseWidthEvent, 64> = Queue::new();
static mut CAPTURE_TIMESTAMP: u32 = 0;

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("脉宽测量系统启动");

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    // 配置按钮
    let button1 = gpioc.pc13.into_pull_up_input(); // 模式切换
    let button2 = gpioc.pc14.into_pull_up_input(); // 测量控制
    let button3 = gpioc.pc15.into_pull_up_input(); // 校准/阈值调整

    // 配置LED指示灯
    let mut status_led = gpiob.pb0.into_push_pull_output();      // 系统状态
    let mut measurement_led = gpiob.pb1.into_push_pull_output(); // 测量活动
    let mut pulse_led = gpiob.pb2.into_push_pull_output();       // 脉冲检测
    let mut mode_led = gpiob.pb3.into_push_pull_output();        // 模式指示
    let mut precision_led = gpiob.pb4.into_push_pull_output();   // 精度指示
    let mut threshold_led = gpiob.pb5.into_push_pull_output();   // 阈值指示
    let mut calibration_led = gpiob.pb6.into_push_pull_output(); // 校准指示
    let mut error_led = gpiob.pb7.into_push_pull_output();       // 错误指示

    // 配置输入捕获引脚
    let capture_pin1 = gpioa.pa0.into_alternate(); // TIM2_CH1
    let capture_pin2 = gpioa.pa1.into_alternate(); // TIM2_CH2

    // 配置定时器用于输入捕获
    let mut timer2 = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer2.start(10.MHz()).unwrap(); // 10MHz计数频率，0.1us分辨率
    timer2.listen(Event::Update);

    // 配置系统定时器
    let mut delay = cp.SYST.delay(&clocks);

    // 创建事件队列
    let (event_producer, mut event_consumer) = unsafe {
        PULSE_EVENT_QUEUE.split()
    };

    // 创建脉宽测量管理器
    let mut pulse_manager = PulseWidthManager::new(
        10_000_000, // 10MHz定时器频率
        event_producer
    );

    // 启用中断
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }

    // 系统变量
    let mut last_button1_state = button1.is_high();
    let mut last_button2_state = button2.is_high();
    let mut last_button3_state = button3.is_high();
    let mut button1_debounce = 0u32;
    let mut button2_debounce = 0u32;
    let mut button3_debounce = 0u32;
    let mut system_tick = 0u32;
    let mut pulse_detected = false;

    rprintln!("脉宽测量系统就绪");

    loop {
        let current_time = system_tick;
        system_tick = system_tick.wrapping_add(1);

        // 按钮1处理（模式切换）
        let button1_state = button1.is_high();
        if button1_state != last_button1_state {
            button1_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button1_debounce) > 50 && button1_state && !last_button1_state {
            let new_mode = match pulse_manager.current_mode {
                PulseMode::SinglePulse => PulseMode::ContinuousPulse,
                PulseMode::ContinuousPulse => PulseMode::DutyCycle,
                PulseMode::DutyCycle => PulseMode::PulseWidth,
                PulseMode::PulseWidth => PulseMode::Period,
                PulseMode::Period => PulseMode::Statistical,
                PulseMode::Statistical => PulseMode::SinglePulse,
            };
            pulse_manager.send_event(PulseWidthEvent::ModeChange(new_mode));
        }
        last_button1_state = button1_state;

        // 按钮2处理（测量控制）
        let button2_state = button2.is_high();
        if button2_state != last_button2_state {
            button2_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button2_debounce) > 50 && button2_state && !last_button2_state {
            if pulse_manager.measurement_active {
                pulse_manager.stop_measurement();
            } else {
                pulse_manager.start_measurement();
            }
        }
        last_button2_state = button2_state;

        // 按钮3处理（校准/阈值调整）
        let button3_state = button3.is_high();
        if button3_state != last_button3_state {
            button3_debounce = current_time;
        }
        
        if current_time.wrapping_sub(button3_debounce) > 50 && button3_state && !last_button3_state {
            if pulse_manager.calibration_active {
                pulse_manager.end_calibration();
            } else {
                pulse_manager.start_calibration();
            }
        }
        last_button3_state = button3_state;

        // 处理事件队列
        while let Some(event) = event_consumer.dequeue() {
            match event {
                PulseWidthEvent::PulseStart(timestamp) => {
                    if let Err(e) = pulse_manager.process_capture(1, timestamp, EdgeType::Rising) {
                        rprintln!("处理脉冲开始错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                        pulse_detected = true;
                        pulse_led.set_high();
                    }
                }
                PulseWidthEvent::PulseEnd(timestamp) => {
                    if let Err(e) = pulse_manager.process_capture(1, timestamp, EdgeType::Falling) {
                        rprintln!("处理脉冲结束错误: {}", e);
                        error_led.set_high();
                    } else {
                        error_led.set_low();
                        pulse_detected = false;
                        pulse_led.set_low();
                    }
                }
                PulseWidthEvent::MeasurementComplete(result) => {
                    rprintln!("测量完成 - 脉宽: {:.2} us, 占空比: {:.1}%, 频率: {:.2} Hz", 
                             result.pulse_width_us, result.duty_cycle * 100.0, result.frequency_hz);
                }
                PulseWidthEvent::CalibrationRequest => {
                    pulse_manager.start_calibration();
                }
                PulseWidthEvent::ModeChange(mode) => {
                    pulse_manager.set_mode(mode);
                }
                PulseWidthEvent::ThresholdUpdate(threshold) => {
                    pulse_manager.set_threshold(threshold);
                }
            }
        }

        // 更新测量
        if let Err(e) = pulse_manager.update_measurements(10) {
            rprintln!("更新测量错误: {}", e);
            error_led.set_high();
        }

        // LED指示更新
        // 系统状态LED（心跳）
        if system_tick % 100 == 0 {
            status_led.toggle();
        }

        // 测量活动指示
        if pulse_manager.measurement_active {
            measurement_led.set_high();
        } else {
            measurement_led.set_low();
        }

        // 模式指示LED（不同模式不同闪烁模式）
        let mode_pattern = match pulse_manager.current_mode {
            PulseMode::SinglePulse => (2000, 1), // 慢闪1次
            PulseMode::ContinuousPulse => (1000, 2), // 中速闪2次
            PulseMode::DutyCycle => (500, 3), // 快闪3次
            PulseMode::PulseWidth => (300, 4), // 更快闪4次
            PulseMode::Period => (200, 5), // 很快闪5次
            PulseMode::Statistical => (100, 1), // 极快闪1次
        };
        
        let cycle_time = system_tick % mode_pattern.0;
        let flash_duration = mode_pattern.0 / (mode_pattern.1 * 2);
        let flash_index = cycle_time / (flash_duration * 2);
        
        if flash_index < mode_pattern.1 && (cycle_time % (flash_duration * 2)) < flash_duration {
            mode_led.set_high();
        } else {
            mode_led.set_low();
        }

        // 精度指示（根据测量统计）
        let stats = pulse_manager.pulse_meter1.get_statistics();
        if stats.sample_count > 10 {
            let precision_ratio = if stats.avg_pulse_width > 0.0 {
                stats.pulse_width_jitter / stats.avg_pulse_width
            } else {
                1.0
            };
            
            if precision_ratio < 0.01 { // <1% 抖动
                precision_led.set_high();
            } else if precision_ratio < 0.05 { // <5% 抖动
                if system_tick % 500 < 250 {
                    precision_led.set_high();
                } else {
                    precision_led.set_low();
                }
            } else {
                precision_led.set_low();
            }
        } else {
            precision_led.set_low();
        }

        // 阈值指示（根据阈值电压）
        let threshold_intensity = (pulse_manager.threshold_voltage / 5.0 * 100.0) as u32;
        if system_tick % 100 < threshold_intensity {
            threshold_led.set_high();
        } else {
            threshold_led.set_low();
        }

        // 校准指示
        if pulse_manager.calibration_active {
            calibration_led.set_high();
        } else {
            calibration_led.set_low();
        }

        // 定期输出统计信息
        if system_tick % 5000 == 0 {
            rprintln!("=== 脉宽测量统计信息 ===");
            rprintln!("总测量次数: {}", pulse_manager.statistics.total_measurements);
            rprintln!("成功测量: {}", pulse_manager.statistics.successful_measurements);
            rprintln!("失败测量: {}", pulse_manager.statistics.failed_measurements);
            rprintln!("总脉冲数: {}", pulse_manager.statistics.total_pulses);
            rprintln!("测量时间: {} ms", pulse_manager.statistics.measurement_time_ms);
            rprintln!("校准次数: {}", pulse_manager.statistics.calibration_count);
            rprintln!("模式切换: {}", pulse_manager.statistics.mode_changes);
            rprintln!("阈值调整: {}", pulse_manager.statistics.threshold_changes);
            rprintln!("当前模式: {:?}", pulse_manager.current_mode);
            rprintln!("阈值电压: {:.2} V", pulse_manager.threshold_voltage);
            
            let result = pulse_manager.get_current_result();
            rprintln!("当前测量 - 脉宽: {:.2} us, 周期: {:.2} us, 占空比: {:.1}%", 
                     result.pulse_width_us, result.period_us, result.duty_cycle * 100.0);
            
            let stats = pulse_manager.pulse_meter1.get_statistics();
            if stats.sample_count > 0 {
                rprintln!("统计信息 - 样本数: {}, 平均脉宽: {:.2} us, 抖动: {:.3} us", 
                         stats.sample_count, stats.avg_pulse_width, stats.pulse_width_jitter);
                rprintln!("脉宽范围: {:.2} - {:.2} us, 平均频率: {:.2} Hz", 
                         stats.min_pulse_width, stats.max_pulse_width, stats.avg_frequency);
            }
        }

        delay.delay_ms(10u32);
    }
}

// 定时器中断处理
#[interrupt]
fn TIM2() {
    // 这里应该处理输入捕获中断
    // 实际实现需要根据具体的HAL库接口
    unsafe {
        CAPTURE_TIMESTAMP = CAPTURE_TIMESTAMP.wrapping_add(1);
        // 发送捕获事件到队列
        // 这里简化处理，实际需要读取捕获寄存器和判断边沿类型
    }
}
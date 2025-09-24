#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel, Event},
    gpio::{gpioa::PA8, Alternate, AF1},
    interrupt,
};
use cortex_m::peripheral::NVIC;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

type CapturePin = PA8<Alternate<AF1>>;

// 全局变量
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM1>>>> = Mutex::new(RefCell::new(None));
static PWM_ANALYZER: Mutex<RefCell<Option<PwmAnalyzer>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 获取外设
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let capture_pin = gpioa.pa8.into_alternate_af1();

    // 配置定时器1用于PWM输入模式
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    
    // 配置PWM输入模式 (通道1和通道2)
    timer.configure_pwm_input(capture_pin);
    timer.listen(Event::Capture(Channel::C1));
    timer.listen(Event::Capture(Channel::C2));
    
    // 创建PWM分析器
    let pwm_analyzer = PwmAnalyzer::new(clocks.sysclk().0);
    
    // 将对象移动到全局变量
    cortex_m::interrupt::free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
        PWM_ANALYZER.borrow(cs).replace(Some(pwm_analyzer));
    });
    
    // 启用中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM1_CC);
    }
    
    // 配置系统定时器用于延时
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);

    // 主循环
    loop {
        delay.delay_ms(1000u32);
        
        // 读取PWM分析结果
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut analyzer) = PWM_ANALYZER.borrow(cs).borrow_mut().as_mut() {
                let measurements = analyzer.get_measurements();
                
                if measurements.is_valid() {
                    // 这里可以通过串口输出结果
                    // 或者控制LED显示PWM状态
                }
                
                // 重置分析器
                analyzer.reset();
            }
        });
    }
}

/// PWM测量结果
#[derive(Clone, Copy)]
pub struct PwmMeasurements {
    pub frequency: f32,
    pub duty_cycle: f32,
    pub period_us: f32,
    pub pulse_width_us: f32,
    pub valid: bool,
}

impl PwmMeasurements {
    pub fn new() -> Self {
        Self {
            frequency: 0.0,
            duty_cycle: 0.0,
            period_us: 0.0,
            pulse_width_us: 0.0,
            valid: false,
        }
    }
    
    pub fn is_valid(&self) -> bool {
        self.valid
    }
}

/// PWM分析器
pub struct PwmAnalyzer {
    timer_clock: u32,
    period_capture: u32,
    pulse_width_capture: u32,
    measurements: PwmMeasurements,
    capture_ready: bool,
}

impl PwmAnalyzer {
    /// 创建新的PWM分析器
    pub fn new(timer_clock: u32) -> Self {
        Self {
            timer_clock,
            period_capture: 0,
            pulse_width_capture: 0,
            measurements: PwmMeasurements::new(),
            capture_ready: false,
        }
    }
    
    /// 处理周期捕获 (通道1 - 上升沿到上升沿)
    pub fn handle_period_capture(&mut self, capture_value: u32) {
        self.period_capture = capture_value;
        self.update_measurements();
    }
    
    /// 处理脉宽捕获 (通道2 - 上升沿到下降沿)
    pub fn handle_pulse_width_capture(&mut self, capture_value: u32) {
        self.pulse_width_capture = capture_value;
        self.capture_ready = true;
        self.update_measurements();
    }
    
    /// 更新测量结果
    fn update_measurements(&mut self) {
        if self.capture_ready && self.period_capture > 0 {
            // 计算周期 (微秒)
            self.measurements.period_us = 
                (self.period_capture as f32 * 1_000_000.0) / self.timer_clock as f32;
            
            // 计算脉宽 (微秒)
            self.measurements.pulse_width_us = 
                (self.pulse_width_capture as f32 * 1_000_000.0) / self.timer_clock as f32;
            
            // 计算频率 (Hz)
            self.measurements.frequency = 1_000_000.0 / self.measurements.period_us;
            
            // 计算占空比 (%)
            self.measurements.duty_cycle = 
                (self.measurements.pulse_width_us / self.measurements.period_us) * 100.0;
            
            self.measurements.valid = true;
        }
    }
    
    /// 获取测量结果
    pub fn get_measurements(&self) -> PwmMeasurements {
        self.measurements
    }
    
    /// 重置分析器
    pub fn reset(&mut self) {
        self.capture_ready = false;
        self.measurements = PwmMeasurements::new();
    }
}

/// 脉冲宽度测量器 (单边沿触发)
pub struct PulseWidthMeter {
    timer_clock: u32,
    rising_edge_time: u32,
    falling_edge_time: u32,
    pulse_width_us: f32,
    measuring: bool,
    valid_measurement: bool,
}

impl PulseWidthMeter {
    /// 创建新的脉宽测量器
    pub fn new(timer_clock: u32) -> Self {
        Self {
            timer_clock,
            rising_edge_time: 0,
            falling_edge_time: 0,
            pulse_width_us: 0.0,
            measuring: false,
            valid_measurement: false,
        }
    }
    
    /// 处理上升沿
    pub fn handle_rising_edge(&mut self, timestamp: u32) {
        self.rising_edge_time = timestamp;
        self.measuring = true;
        self.valid_measurement = false;
    }
    
    /// 处理下降沿
    pub fn handle_falling_edge(&mut self, timestamp: u32) {
        if self.measuring {
            self.falling_edge_time = timestamp;
            
            // 计算脉宽
            let pulse_ticks = if self.falling_edge_time >= self.rising_edge_time {
                self.falling_edge_time - self.rising_edge_time
            } else {
                // 处理定时器溢出
                (0xFFFF - self.rising_edge_time) + self.falling_edge_time + 1
            };
            
            self.pulse_width_us = (pulse_ticks as f32 * 1_000_000.0) / self.timer_clock as f32;
            self.measuring = false;
            self.valid_measurement = true;
        }
    }
    
    /// 获取脉宽 (微秒)
    pub fn get_pulse_width_us(&self) -> f32 {
        if self.valid_measurement {
            self.pulse_width_us
        } else {
            0.0
        }
    }
    
    /// 检查测量是否有效
    pub fn is_valid(&self) -> bool {
        self.valid_measurement
    }
    
    /// 重置测量器
    pub fn reset(&mut self) {
        self.measuring = false;
        self.valid_measurement = false;
        self.pulse_width_us = 0.0;
    }
}

/// 多脉冲分析器
pub struct MultiPulseAnalyzer {
    pulse_widths: [f32; 16],
    pulse_count: usize,
    min_pulse_width: f32,
    max_pulse_width: f32,
    average_pulse_width: f32,
    pulse_meter: PulseWidthMeter,
}

impl MultiPulseAnalyzer {
    /// 创建多脉冲分析器
    pub fn new(timer_clock: u32) -> Self {
        Self {
            pulse_widths: [0.0; 16],
            pulse_count: 0,
            min_pulse_width: f32::MAX,
            max_pulse_width: 0.0,
            average_pulse_width: 0.0,
            pulse_meter: PulseWidthMeter::new(timer_clock),
        }
    }
    
    /// 处理上升沿
    pub fn handle_rising_edge(&mut self, timestamp: u32) {
        self.pulse_meter.handle_rising_edge(timestamp);
    }
    
    /// 处理下降沿
    pub fn handle_falling_edge(&mut self, timestamp: u32) {
        self.pulse_meter.handle_falling_edge(timestamp);
        
        if self.pulse_meter.is_valid() {
            let pulse_width = self.pulse_meter.get_pulse_width_us();
            self.add_pulse_width(pulse_width);
            self.pulse_meter.reset();
        }
    }
    
    /// 添加脉宽数据
    fn add_pulse_width(&mut self, pulse_width: f32) {
        if self.pulse_count < 16 {
            self.pulse_widths[self.pulse_count] = pulse_width;
            self.pulse_count += 1;
            
            // 更新统计信息
            self.update_statistics();
        }
    }
    
    /// 更新统计信息
    fn update_statistics(&mut self) {
        if self.pulse_count == 0 {
            return;
        }
        
        self.min_pulse_width = f32::MAX;
        self.max_pulse_width = 0.0;
        let mut sum = 0.0;
        
        for i in 0..self.pulse_count {
            let width = self.pulse_widths[i];
            
            if width < self.min_pulse_width {
                self.min_pulse_width = width;
            }
            
            if width > self.max_pulse_width {
                self.max_pulse_width = width;
            }
            
            sum += width;
        }
        
        self.average_pulse_width = sum / self.pulse_count as f32;
    }
    
    /// 获取最小脉宽
    pub fn get_min_pulse_width(&self) -> f32 {
        if self.pulse_count > 0 {
            self.min_pulse_width
        } else {
            0.0
        }
    }
    
    /// 获取最大脉宽
    pub fn get_max_pulse_width(&self) -> f32 {
        self.max_pulse_width
    }
    
    /// 获取平均脉宽
    pub fn get_average_pulse_width(&self) -> f32 {
        self.average_pulse_width
    }
    
    /// 获取脉冲数量
    pub fn get_pulse_count(&self) -> usize {
        self.pulse_count
    }
    
    /// 重置分析器
    pub fn reset(&mut self) {
        self.pulse_count = 0;
        self.min_pulse_width = f32::MAX;
        self.max_pulse_width = 0.0;
        self.average_pulse_width = 0.0;
        self.pulse_meter.reset();
    }
    
    /// 获取脉宽变化率 (标准差)
    pub fn get_pulse_width_variation(&self) -> f32 {
        if self.pulse_count < 2 {
            return 0.0;
        }
        
        let mean = self.average_pulse_width;
        let mut variance_sum = 0.0;
        
        for i in 0..self.pulse_count {
            let diff = self.pulse_widths[i] - mean;
            variance_sum += diff * diff;
        }
        
        let variance = variance_sum / (self.pulse_count - 1) as f32;
        variance.sqrt()
    }
}

/// 定时器1捕获中断处理程序
#[interrupt]
fn TIM1_CC() {
    cortex_m::interrupt::free(|cs| {
        if let (Some(ref mut timer), Some(ref mut analyzer)) = (
            TIMER.borrow(cs).borrow_mut().as_mut(),
            PWM_ANALYZER.borrow(cs).borrow_mut().as_mut(),
        ) {
            // 检查通道1捕获中断 (周期)
            if timer.is_capture_interrupt_pending(Channel::C1) {
                let capture_value = timer.get_capture_value(Channel::C1);
                analyzer.handle_period_capture(capture_value);
                timer.clear_capture_interrupt(Channel::C1);
            }
            
            // 检查通道2捕获中断 (脉宽)
            if timer.is_capture_interrupt_pending(Channel::C2) {
                let capture_value = timer.get_capture_value(Channel::C2);
                analyzer.handle_pulse_width_capture(capture_value);
                timer.clear_capture_interrupt(Channel::C2);
            }
        }
    });
}
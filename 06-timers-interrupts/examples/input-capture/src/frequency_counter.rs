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

type CapturePin = PA8<Alternate<AF1>>;

// 全局变量用于中断处理
static mut TIMER: Option<Timer<stm32::TIM1>> = None;
static mut FREQUENCY_COUNTER: Option<FrequencyCounter> = None;

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

    // 配置定时器1用于输入捕获
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    
    // 配置输入捕获
    timer.configure_input_capture(Channel::C1, capture_pin);
    timer.set_capture_mode(Channel::C1, CaptureMode::RisingEdge);
    
    // 启用捕获中断
    timer.listen(Event::Capture(Channel::C1));
    
    // 创建频率计数器
    let frequency_counter = FrequencyCounter::new(clocks.sysclk().0);
    
    // 将对象移动到全局变量
    unsafe {
        TIMER = Some(timer);
        FREQUENCY_COUNTER = Some(frequency_counter);
    }
    
    // 启用中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM1_CC);
    }
    
    // 配置系统定时器用于延时
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);

    // 主循环
    loop {
        delay.delay_ms(1000u32);
        
        // 读取频率测量结果
        unsafe {
            if let Some(ref mut counter) = FREQUENCY_COUNTER {
                let frequency = counter.get_frequency();
                let period = counter.get_period_us();
                
                // 这里可以通过串口输出结果
                // 或者控制LED显示频率范围
                
                // 重置计数器准备下次测量
                counter.reset();
            }
        }
    }
}

/// 捕获模式枚举
#[derive(Clone, Copy)]
pub enum CaptureMode {
    RisingEdge,
    FallingEdge,
    BothEdges,
}

/// 频率计数器结构体
pub struct FrequencyCounter {
    timer_clock: u32,
    last_capture: u32,
    current_capture: u32,
    capture_count: u32,
    frequency: f32,
    period_us: f32,
    valid_measurement: bool,
}

impl FrequencyCounter {
    /// 创建新的频率计数器
    pub fn new(timer_clock: u32) -> Self {
        Self {
            timer_clock,
            last_capture: 0,
            current_capture: 0,
            capture_count: 0,
            frequency: 0.0,
            period_us: 0.0,
            valid_measurement: false,
        }
    }
    
    /// 处理捕获事件
    pub fn handle_capture(&mut self, capture_value: u32) {
        self.current_capture = capture_value;
        
        if self.capture_count > 0 {
            // 计算周期
            let period_ticks = if self.current_capture >= self.last_capture {
                self.current_capture - self.last_capture
            } else {
                // 处理定时器溢出
                (0xFFFF - self.last_capture) + self.current_capture + 1
            };
            
            // 计算频率和周期
            if period_ticks > 0 {
                self.frequency = self.timer_clock as f32 / period_ticks as f32;
                self.period_us = (period_ticks as f32 * 1_000_000.0) / self.timer_clock as f32;
                self.valid_measurement = true;
            }
        }
        
        self.last_capture = self.current_capture;
        self.capture_count += 1;
    }
    
    /// 获取测量的频率 (Hz)
    pub fn get_frequency(&self) -> f32 {
        if self.valid_measurement {
            self.frequency
        } else {
            0.0
        }
    }
    
    /// 获取测量的周期 (微秒)
    pub fn get_period_us(&self) -> f32 {
        if self.valid_measurement {
            self.period_us
        } else {
            0.0
        }
    }
    
    /// 检查测量是否有效
    pub fn is_valid(&self) -> bool {
        self.valid_measurement
    }
    
    /// 重置计数器
    pub fn reset(&mut self) {
        self.capture_count = 0;
        self.valid_measurement = false;
        self.frequency = 0.0;
        self.period_us = 0.0;
    }
    
    /// 获取捕获次数
    pub fn get_capture_count(&self) -> u32 {
        self.capture_count
    }
}

/// 多通道频率计数器
pub struct MultiChannelFrequencyCounter {
    counters: [FrequencyCounter; 4],
    active_channels: u8,
}

impl MultiChannelFrequencyCounter {
    /// 创建多通道频率计数器
    pub fn new(timer_clock: u32) -> Self {
        Self {
            counters: [
                FrequencyCounter::new(timer_clock),
                FrequencyCounter::new(timer_clock),
                FrequencyCounter::new(timer_clock),
                FrequencyCounter::new(timer_clock),
            ],
            active_channels: 0,
        }
    }
    
    /// 启用通道
    pub fn enable_channel(&mut self, channel: usize) -> Result<(), ()> {
        if channel >= 4 {
            return Err(());
        }
        
        self.active_channels |= 1 << channel;
        Ok(())
    }
    
    /// 禁用通道
    pub fn disable_channel(&mut self, channel: usize) -> Result<(), ()> {
        if channel >= 4 {
            return Err(());
        }
        
        self.active_channels &= !(1 << channel);
        self.counters[channel].reset();
        Ok(())
    }
    
    /// 处理指定通道的捕获事件
    pub fn handle_channel_capture(&mut self, channel: usize, capture_value: u32) -> Result<(), ()> {
        if channel >= 4 || (self.active_channels & (1 << channel)) == 0 {
            return Err(());
        }
        
        self.counters[channel].handle_capture(capture_value);
        Ok(())
    }
    
    /// 获取指定通道的频率
    pub fn get_channel_frequency(&self, channel: usize) -> Result<f32, ()> {
        if channel >= 4 {
            return Err(());
        }
        
        Ok(self.counters[channel].get_frequency())
    }
    
    /// 获取所有活动通道的频率
    pub fn get_all_frequencies(&self) -> [f32; 4] {
        [
            self.counters[0].get_frequency(),
            self.counters[1].get_frequency(),
            self.counters[2].get_frequency(),
            self.counters[3].get_frequency(),
        ]
    }
    
    /// 重置所有通道
    pub fn reset_all(&mut self) {
        for counter in &mut self.counters {
            counter.reset();
        }
    }
}

/// 频率范围检测器
pub struct FrequencyRangeDetector {
    counter: FrequencyCounter,
    min_frequency: f32,
    max_frequency: f32,
    in_range_callback: Option<fn()>,
    out_of_range_callback: Option<fn()>,
}

impl FrequencyRangeDetector {
    /// 创建频率范围检测器
    pub fn new(timer_clock: u32, min_freq: f32, max_freq: f32) -> Self {
        Self {
            counter: FrequencyCounter::new(timer_clock),
            min_frequency: min_freq,
            max_frequency: max_freq,
            in_range_callback: None,
            out_of_range_callback: None,
        }
    }
    
    /// 设置范围内回调
    pub fn set_in_range_callback(&mut self, callback: fn()) {
        self.in_range_callback = Some(callback);
    }
    
    /// 设置超出范围回调
    pub fn set_out_of_range_callback(&mut self, callback: fn()) {
        self.out_of_range_callback = Some(callback);
    }
    
    /// 处理捕获事件并检查范围
    pub fn handle_capture(&mut self, capture_value: u32) {
        self.counter.handle_capture(capture_value);
        
        if self.counter.is_valid() {
            let frequency = self.counter.get_frequency();
            
            if frequency >= self.min_frequency && frequency <= self.max_frequency {
                if let Some(callback) = self.in_range_callback {
                    callback();
                }
            } else {
                if let Some(callback) = self.out_of_range_callback {
                    callback();
                }
            }
        }
    }
    
    /// 获取当前频率
    pub fn get_frequency(&self) -> f32 {
        self.counter.get_frequency()
    }
    
    /// 检查是否在范围内
    pub fn is_in_range(&self) -> bool {
        if !self.counter.is_valid() {
            return false;
        }
        
        let frequency = self.counter.get_frequency();
        frequency >= self.min_frequency && frequency <= self.max_frequency
    }
}

/// 定时器1捕获中断处理程序
#[interrupt]
fn TIM1_CC() {
    unsafe {
        if let (Some(ref mut timer), Some(ref mut counter)) = (&mut TIMER, &mut FREQUENCY_COUNTER) {
            // 检查是否是通道1捕获中断
            if timer.is_capture_interrupt_pending(Channel::C1) {
                // 读取捕获值
                let capture_value = timer.get_capture_value(Channel::C1);
                
                // 处理捕获事件
                counter.handle_capture(capture_value);
                
                // 清除中断标志
                timer.clear_capture_interrupt(Channel::C1);
            }
        }
    }
}
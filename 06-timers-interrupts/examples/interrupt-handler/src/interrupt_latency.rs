#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Event},
    gpio::{gpioa::*, gpiob::*, Output, PushPull, Input, PullUp, Alternate, AF1},
    interrupt,
    dwt::{Dwt, DwtExt},
};
use cortex_m::peripheral::{NVIC, DWT, DCB};
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

type TestPin = PB0<Output<PushPull>>;
type TriggerPin = PA8<Alternate<AF1>>;

// 全局变量
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM1>>>> = Mutex::new(RefCell::new(None));
static LATENCY_TESTER: Mutex<RefCell<Option<InterruptLatencyTester>>> = Mutex::new(RefCell::new(None));
static TEST_PIN: Mutex<RefCell<Option<TestPin>>> = Mutex::new(RefCell::new(None));
static DWT_COUNTER: Mutex<RefCell<Option<Dwt>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    // 获取外设
    let dp = stm32::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();

    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(168.mhz()).freeze(); // 使用最高频率以获得最佳精度

    // 启用DWT计数器用于高精度时间测量
    cp.DCB.enable_trace();
    let dwt = cp.DWT.constrain(cp.DCB, &clocks);

    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    
    let test_pin = gpiob.pb0.into_push_pull_output(); // 用于测量中断延迟
    let trigger_pin = gpioa.pa8.into_alternate_af1();
    let status_led = gpiob.pb1.into_push_pull_output();

    // 配置定时器1用于触发中断
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    timer.start(1.khz()); // 1kHz测试频率
    timer.listen(Event::Update);
    
    // 创建延迟测试器
    let latency_tester = InterruptLatencyTester::new(clocks.sysclk().0);
    
    // 将对象移动到全局变量
    cortex_m::interrupt::free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
        LATENCY_TESTER.borrow(cs).replace(Some(latency_tester));
        TEST_PIN.borrow(cs).replace(Some(test_pin));
        DWT_COUNTER.borrow(cs).replace(Some(dwt));
    });
    
    // 启用中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM1_UP_TIM10);
    }
    
    // 配置系统定时器用于延时
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);
    let mut status_led = status_led;
    let mut test_counter = 0u32;

    // 主循环
    loop {
        delay.delay_ms(1000u32);
        status_led.toggle();
        test_counter += 1;
        
        // 每10秒输出一次统计信息
        if test_counter % 10 == 0 {
            cortex_m::interrupt::free(|cs| {
                if let Some(ref tester) = LATENCY_TESTER.borrow(cs).borrow().as_ref() {
                    let stats = tester.get_statistics();
                    // 这里可以通过串口输出统计信息
                    // 例如：平均延迟、最大延迟、最小延迟、抖动等
                }
            });
        }
        
        // 每60秒重置统计信息
        if test_counter % 60 == 0 {
            cortex_m::interrupt::free(|cs| {
                if let Some(ref mut tester) = LATENCY_TESTER.borrow(cs).borrow_mut().as_mut() {
                    tester.reset_statistics();
                }
            });
        }
    }
}

/// 延迟测量统计信息
#[derive(Clone, Copy)]
pub struct LatencyStatistics {
    pub sample_count: u32,
    pub total_latency_cycles: u64,
    pub min_latency_cycles: u32,
    pub max_latency_cycles: u32,
    pub average_latency_cycles: u32,
    pub jitter_cycles: u32,
    pub min_latency_ns: u32,
    pub max_latency_ns: u32,
    pub average_latency_ns: u32,
    pub jitter_ns: u32,
}

impl LatencyStatistics {
    pub fn new() -> Self {
        Self {
            sample_count: 0,
            total_latency_cycles: 0,
            min_latency_cycles: u32::MAX,
            max_latency_cycles: 0,
            average_latency_cycles: 0,
            jitter_cycles: 0,
            min_latency_ns: u32::MAX,
            max_latency_ns: 0,
            average_latency_ns: 0,
            jitter_ns: 0,
        }
    }
}

/// 中断延迟测试器
pub struct InterruptLatencyTester {
    cpu_frequency: u32,
    statistics: LatencyStatistics,
    latency_samples: [u32; 64], // 存储最近64个样本用于计算抖动
    sample_index: usize,
    trigger_timestamp: u32,
    measurement_active: bool,
}

impl InterruptLatencyTester {
    /// 创建新的延迟测试器
    pub fn new(cpu_frequency: u32) -> Self {
        Self {
            cpu_frequency,
            statistics: LatencyStatistics::new(),
            latency_samples: [0; 64],
            sample_index: 0,
            trigger_timestamp: 0,
            measurement_active: false,
        }
    }
    
    /// 开始测量 (在中断触发前调用)
    pub fn start_measurement(&mut self, timestamp: u32) {
        self.trigger_timestamp = timestamp;
        self.measurement_active = true;
    }
    
    /// 结束测量 (在中断服务程序开始时调用)
    pub fn end_measurement(&mut self, response_timestamp: u32) {
        if !self.measurement_active {
            return;
        }
        
        let latency_cycles = response_timestamp.wrapping_sub(self.trigger_timestamp);
        self.add_sample(latency_cycles);
        self.measurement_active = false;
    }
    
    /// 添加延迟样本
    fn add_sample(&mut self, latency_cycles: u32) {
        // 更新统计信息
        self.statistics.sample_count += 1;
        self.statistics.total_latency_cycles += latency_cycles as u64;
        
        if latency_cycles < self.statistics.min_latency_cycles {
            self.statistics.min_latency_cycles = latency_cycles;
        }
        
        if latency_cycles > self.statistics.max_latency_cycles {
            self.statistics.max_latency_cycles = latency_cycles;
        }
        
        // 计算平均值
        self.statistics.average_latency_cycles = 
            (self.statistics.total_latency_cycles / self.statistics.sample_count as u64) as u32;
        
        // 存储样本用于抖动计算
        self.latency_samples[self.sample_index] = latency_cycles;
        self.sample_index = (self.sample_index + 1) % 64;
        
        // 计算抖动 (标准差)
        self.calculate_jitter();
        
        // 转换为纳秒
        self.convert_to_nanoseconds();
    }
    
    /// 计算抖动
    fn calculate_jitter(&mut self) {
        if self.statistics.sample_count < 2 {
            return;
        }
        
        let sample_count = core::cmp::min(self.statistics.sample_count as usize, 64);
        let mean = self.statistics.average_latency_cycles;
        
        let mut variance_sum = 0u64;
        for i in 0..sample_count {
            let diff = if self.latency_samples[i] > mean {
                self.latency_samples[i] - mean
            } else {
                mean - self.latency_samples[i]
            };
            variance_sum += (diff as u64) * (diff as u64);
        }
        
        let variance = variance_sum / (sample_count - 1) as u64;
        self.statistics.jitter_cycles = (variance as f32).sqrt() as u32;
    }
    
    /// 转换为纳秒
    fn convert_to_nanoseconds(&mut self) {
        let cycles_to_ns = |cycles: u32| -> u32 {
            ((cycles as u64 * 1_000_000_000) / self.cpu_frequency as u64) as u32
        };
        
        self.statistics.min_latency_ns = cycles_to_ns(self.statistics.min_latency_cycles);
        self.statistics.max_latency_ns = cycles_to_ns(self.statistics.max_latency_cycles);
        self.statistics.average_latency_ns = cycles_to_ns(self.statistics.average_latency_cycles);
        self.statistics.jitter_ns = cycles_to_ns(self.statistics.jitter_cycles);
    }
    
    /// 获取统计信息
    pub fn get_statistics(&self) -> LatencyStatistics {
        self.statistics
    }
    
    /// 重置统计信息
    pub fn reset_statistics(&mut self) {
        self.statistics = LatencyStatistics::new();
        self.latency_samples = [0; 64];
        self.sample_index = 0;
    }
    
    /// 获取当前DWT计数器值
    pub fn get_dwt_cycles(&self) -> u32 {
        // 这里需要访问DWT计数器
        // 在实际实现中，应该通过全局变量或其他方式访问
        0
    }
}

/// 多中断延迟分析器
pub struct MultiInterruptLatencyAnalyzer {
    testers: [InterruptLatencyTester; 4],
    interrupt_sources: [InterruptSource; 4],
    current_test_source: usize,
    test_cycle_count: u32,
}

impl MultiInterruptLatencyAnalyzer {
    /// 创建多中断延迟分析器
    pub fn new(cpu_frequency: u32) -> Self {
        Self {
            testers: [
                InterruptLatencyTester::new(cpu_frequency),
                InterruptLatencyTester::new(cpu_frequency),
                InterruptLatencyTester::new(cpu_frequency),
                InterruptLatencyTester::new(cpu_frequency),
            ],
            interrupt_sources: [
                InterruptSource::Timer1,
                InterruptSource::Timer2,
                InterruptSource::ExternalInt,
                InterruptSource::Uart,
            ],
            current_test_source: 0,
            test_cycle_count: 0,
        }
    }
    
    /// 开始测量指定中断源
    pub fn start_measurement(&mut self, source: InterruptSource, timestamp: u32) {
        if let Some(index) = self.get_source_index(source) {
            self.testers[index].start_measurement(timestamp);
        }
    }
    
    /// 结束测量指定中断源
    pub fn end_measurement(&mut self, source: InterruptSource, timestamp: u32) {
        if let Some(index) = self.get_source_index(source) {
            self.testers[index].end_measurement(timestamp);
        }
    }
    
    /// 获取中断源索引
    fn get_source_index(&self, source: InterruptSource) -> Option<usize> {
        for (i, &src) in self.interrupt_sources.iter().enumerate() {
            if src == source {
                return Some(i);
            }
        }
        None
    }
    
    /// 获取指定中断源的统计信息
    pub fn get_statistics(&self, source: InterruptSource) -> Option<LatencyStatistics> {
        if let Some(index) = self.get_source_index(source) {
            Some(self.testers[index].get_statistics())
        } else {
            None
        }
    }
    
    /// 获取所有中断源的统计信息
    pub fn get_all_statistics(&self) -> [LatencyStatistics; 4] {
        [
            self.testers[0].get_statistics(),
            self.testers[1].get_statistics(),
            self.testers[2].get_statistics(),
            self.testers[3].get_statistics(),
        ]
    }
    
    /// 比较不同中断源的延迟
    pub fn compare_latencies(&self) -> LatencyComparison {
        let stats = self.get_all_statistics();
        
        let mut min_avg_latency = u32::MAX;
        let mut max_avg_latency = 0;
        let mut best_source = InterruptSource::Timer1;
        let mut worst_source = InterruptSource::Timer1;
        
        for (i, stat) in stats.iter().enumerate() {
            if stat.sample_count > 0 {
                if stat.average_latency_ns < min_avg_latency {
                    min_avg_latency = stat.average_latency_ns;
                    best_source = self.interrupt_sources[i];
                }
                
                if stat.average_latency_ns > max_avg_latency {
                    max_avg_latency = stat.average_latency_ns;
                    worst_source = self.interrupt_sources[i];
                }
            }
        }
        
        LatencyComparison {
            best_source,
            worst_source,
            best_latency_ns: min_avg_latency,
            worst_latency_ns: max_avg_latency,
            latency_difference_ns: max_avg_latency.saturating_sub(min_avg_latency),
        }
    }
}

/// 中断源类型
#[derive(Clone, Copy, PartialEq)]
pub enum InterruptSource {
    Timer1,
    Timer2,
    ExternalInt,
    Uart,
}

/// 延迟比较结果
pub struct LatencyComparison {
    pub best_source: InterruptSource,
    pub worst_source: InterruptSource,
    pub best_latency_ns: u32,
    pub worst_latency_ns: u32,
    pub latency_difference_ns: u32,
}

/// 实时延迟监控器
pub struct RealtimeLatencyMonitor {
    threshold_ns: u32,
    violation_count: u32,
    max_violations_allowed: u32,
    current_latency_ns: u32,
    alert_callback: Option<fn()>,
}

impl RealtimeLatencyMonitor {
    /// 创建实时延迟监控器
    pub fn new(threshold_ns: u32, max_violations: u32) -> Self {
        Self {
            threshold_ns,
            violation_count: 0,
            max_violations_allowed: max_violations,
            current_latency_ns: 0,
            alert_callback: None,
        }
    }
    
    /// 设置告警回调
    pub fn set_alert_callback(&mut self, callback: fn()) {
        self.alert_callback = Some(callback);
    }
    
    /// 检查延迟是否超过阈值
    pub fn check_latency(&mut self, latency_ns: u32) -> bool {
        self.current_latency_ns = latency_ns;
        
        if latency_ns > self.threshold_ns {
            self.violation_count += 1;
            
            if self.violation_count > self.max_violations_allowed {
                if let Some(callback) = self.alert_callback {
                    callback();
                }
                return false; // 延迟违规
            }
        } else {
            // 重置违规计数
            self.violation_count = 0;
        }
        
        true // 延迟正常
    }
    
    /// 获取当前延迟
    pub fn get_current_latency(&self) -> u32 {
        self.current_latency_ns
    }
    
    /// 获取违规次数
    pub fn get_violation_count(&self) -> u32 {
        self.violation_count
    }
    
    /// 重置监控器
    pub fn reset(&mut self) {
        self.violation_count = 0;
        self.current_latency_ns = 0;
    }
}

/// 延迟告警回调函数
fn latency_alert() {
    // 延迟超过阈值时的处理
    // 例如：点亮告警LED、发送告警消息等
}

/// TIM1更新中断处理程序
#[interrupt]
fn TIM1_UP_TIM10() {
    // 立即获取时间戳以测量中断延迟
    let response_timestamp = get_dwt_cycles();
    
    cortex_m::interrupt::free(|cs| {
        // 设置测试引脚以便外部测量
        if let Some(ref mut pin) = TEST_PIN.borrow(cs).borrow_mut().as_mut() {
            pin.set_high();
        }
        
        if let Some(ref mut timer) = TIMER.borrow(cs).borrow_mut().as_mut() {
            if timer.is_update_interrupt_pending() {
                timer.clear_update_interrupt();
                
                // 结束延迟测量
                if let Some(ref mut tester) = LATENCY_TESTER.borrow(cs).borrow_mut().as_mut() {
                    tester.end_measurement(response_timestamp);
                }
            }
        }
        
        // 重置测试引脚
        if let Some(ref mut pin) = TEST_PIN.borrow(cs).borrow_mut().as_mut() {
            pin.set_low();
        }
    });
}

/// 获取DWT计数器值
fn get_dwt_cycles() -> u32 {
    // 直接读取DWT计数器
    unsafe {
        let dwt = &*stm32::DWT::ptr();
        dwt.cyccnt.read()
    }
}
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
    stm32,
    timer::{Timer, Channel, Event},
    gpio::{gpioa::*, gpiob::*, Output, PushPull, Input, PullUp, Alternate, AF1},
    interrupt,
};
use cortex_m::peripheral::NVIC;
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;

type TriggerPin = PA9<Output<PushPull>>;
type EchoPin = PA8<Alternate<AF1>>;

// 全局变量
static TIMER: Mutex<RefCell<Option<Timer<stm32::TIM1>>>> = Mutex::new(RefCell::new(None));
static ULTRASONIC_SENSOR: Mutex<RefCell<Option<UltrasonicSensor>>> = Mutex::new(RefCell::new(None));
static TRIGGER_PIN: Mutex<RefCell<Option<TriggerPin>>> = Mutex::new(RefCell::new(None));

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
    let gpiob = dp.GPIOB.split();
    
    let trigger_pin = gpioa.pa9.into_push_pull_output();
    let echo_pin = gpioa.pa8.into_alternate_af1();
    let led = gpiob.pb0.into_push_pull_output();

    // 配置定时器1用于输入捕获
    let mut timer = Timer::tim1(dp.TIM1, &clocks);
    timer.configure_input_capture(echo_pin, Channel::C1);
    timer.listen(Event::Capture(Channel::C1));
    
    // 创建超声波传感器
    let ultrasonic_sensor = UltrasonicSensor::new(clocks.sysclk().0);
    
    // 将对象移动到全局变量
    cortex_m::interrupt::free(|cs| {
        TIMER.borrow(cs).replace(Some(timer));
        ULTRASONIC_SENSOR.borrow(cs).replace(Some(ultrasonic_sensor));
        TRIGGER_PIN.borrow(cs).replace(Some(trigger_pin));
    });
    
    // 启用中断
    unsafe {
        NVIC::unmask(stm32::Interrupt::TIM1_CC);
    }
    
    // 配置系统定时器用于延时
    let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.sysclk().0);
    let mut led = led;

    // 主循环
    loop {
        // 触发超声波测量
        cortex_m::interrupt::free(|cs| {
            if let (Some(ref mut trigger), Some(ref mut sensor)) = (
                TRIGGER_PIN.borrow(cs).borrow_mut().as_mut(),
                ULTRASONIC_SENSOR.borrow(cs).borrow_mut().as_mut(),
            ) {
                sensor.trigger_measurement(trigger);
            }
        });
        
        delay.delay_ms(100u32);
        
        // 读取距离测量结果
        cortex_m::interrupt::free(|cs| {
            if let Some(ref mut sensor) = ULTRASONIC_SENSOR.borrow(cs).borrow_mut().as_mut() {
                if let Some(distance) = sensor.get_distance_cm() {
                    // 根据距离控制LED
                    if distance < 10.0 {
                        led.set_high();
                    } else {
                        led.set_low();
                    }
                }
            }
        });
        
        delay.delay_ms(50u32);
    }
}

/// 超声波传感器状态
#[derive(Clone, Copy, PartialEq)]
pub enum UltrasonicState {
    Idle,
    WaitingForEcho,
    EchoReceived,
    Timeout,
}

/// 超声波传感器
pub struct UltrasonicSensor {
    timer_clock: u32,
    state: UltrasonicState,
    echo_start_time: u32,
    echo_end_time: u32,
    distance_cm: Option<f32>,
    timeout_counter: u32,
    max_timeout: u32,
}

impl UltrasonicSensor {
    /// 创建新的超声波传感器
    pub fn new(timer_clock: u32) -> Self {
        Self {
            timer_clock,
            state: UltrasonicState::Idle,
            echo_start_time: 0,
            echo_end_time: 0,
            distance_cm: None,
            timeout_counter: 0,
            max_timeout: timer_clock / 100, // 10ms 超时
        }
    }
    
    /// 触发测量
    pub fn trigger_measurement(&mut self, trigger_pin: &mut TriggerPin) {
        // 发送10us的触发脉冲
        trigger_pin.set_high();
        // 这里需要精确的10us延时，实际应用中可能需要使用定时器
        for _ in 0..840 { // 大约10us @ 84MHz
            cortex_m::asm::nop();
        }
        trigger_pin.set_low();
        
        self.state = UltrasonicState::WaitingForEcho;
        self.distance_cm = None;
        self.timeout_counter = 0;
    }
    
    /// 处理回声开始 (上升沿)
    pub fn handle_echo_start(&mut self, timestamp: u32) {
        if self.state == UltrasonicState::WaitingForEcho {
            self.echo_start_time = timestamp;
        }
    }
    
    /// 处理回声结束 (下降沿)
    pub fn handle_echo_end(&mut self, timestamp: u32) {
        if self.state == UltrasonicState::WaitingForEcho {
            self.echo_end_time = timestamp;
            self.calculate_distance();
            self.state = UltrasonicState::EchoReceived;
        }
    }
    
    /// 计算距离
    fn calculate_distance(&mut self) {
        let echo_duration = if self.echo_end_time >= self.echo_start_time {
            self.echo_end_time - self.echo_start_time
        } else {
            // 处理定时器溢出
            (0xFFFF - self.echo_start_time) + self.echo_end_time + 1
        };
        
        // 计算时间 (微秒)
        let time_us = (echo_duration as f32 * 1_000_000.0) / self.timer_clock as f32;
        
        // 计算距离 (厘米)
        // 声速 = 343 m/s = 0.0343 cm/us
        // 距离 = (时间 * 声速) / 2 (往返)
        self.distance_cm = Some((time_us * 0.0343) / 2.0);
    }
    
    /// 获取距离 (厘米)
    pub fn get_distance_cm(&self) -> Option<f32> {
        self.distance_cm
    }
    
    /// 获取状态
    pub fn get_state(&self) -> UltrasonicState {
        self.state
    }
    
    /// 更新超时计数器
    pub fn update_timeout(&mut self) {
        if self.state == UltrasonicState::WaitingForEcho {
            self.timeout_counter += 1;
            if self.timeout_counter >= self.max_timeout {
                self.state = UltrasonicState::Timeout;
                self.distance_cm = None;
            }
        }
    }
    
    /// 重置传感器
    pub fn reset(&mut self) {
        self.state = UltrasonicState::Idle;
        self.distance_cm = None;
        self.timeout_counter = 0;
    }
}

/// 多超声波传感器管理器
pub struct MultiUltrasonicManager {
    sensors: [UltrasonicSensor; 4],
    current_sensor: usize,
    measurement_cycle: u32,
}

impl MultiUltrasonicManager {
    /// 创建多传感器管理器
    pub fn new(timer_clock: u32) -> Self {
        Self {
            sensors: [
                UltrasonicSensor::new(timer_clock),
                UltrasonicSensor::new(timer_clock),
                UltrasonicSensor::new(timer_clock),
                UltrasonicSensor::new(timer_clock),
            ],
            current_sensor: 0,
            measurement_cycle: 0,
        }
    }
    
    /// 循环测量所有传感器
    pub fn cycle_measurement(&mut self, trigger_pins: &mut [&mut dyn embedded_hal::digital::v2::OutputPin<Error = ()>]) {
        if self.current_sensor < self.sensors.len() && self.current_sensor < trigger_pins.len() {
            // 触发当前传感器
            trigger_pins[self.current_sensor].set_high().ok();
            for _ in 0..840 { // 10us延时
                cortex_m::asm::nop();
            }
            trigger_pins[self.current_sensor].set_low().ok();
            
            self.sensors[self.current_sensor].state = UltrasonicState::WaitingForEcho;
            self.sensors[self.current_sensor].distance_cm = None;
            self.sensors[self.current_sensor].timeout_counter = 0;
        }
        
        // 切换到下一个传感器
        self.current_sensor = (self.current_sensor + 1) % self.sensors.len();
        self.measurement_cycle += 1;
    }
    
    /// 处理回声信号
    pub fn handle_echo(&mut self, sensor_index: usize, rising_edge: bool, timestamp: u32) {
        if sensor_index < self.sensors.len() {
            if rising_edge {
                self.sensors[sensor_index].handle_echo_start(timestamp);
            } else {
                self.sensors[sensor_index].handle_echo_end(timestamp);
            }
        }
    }
    
    /// 获取传感器距离
    pub fn get_distance(&self, sensor_index: usize) -> Option<f32> {
        if sensor_index < self.sensors.len() {
            self.sensors[sensor_index].get_distance_cm()
        } else {
            None
        }
    }
    
    /// 获取所有距离
    pub fn get_all_distances(&self) -> [Option<f32>; 4] {
        [
            self.sensors[0].get_distance_cm(),
            self.sensors[1].get_distance_cm(),
            self.sensors[2].get_distance_cm(),
            self.sensors[3].get_distance_cm(),
        ]
    }
}

/// 距离滤波器
pub struct DistanceFilter {
    samples: [f32; 8],
    sample_count: usize,
    index: usize,
}

impl DistanceFilter {
    /// 创建距离滤波器
    pub fn new() -> Self {
        Self {
            samples: [0.0; 8],
            sample_count: 0,
            index: 0,
        }
    }
    
    /// 添加样本
    pub fn add_sample(&mut self, distance: f32) {
        self.samples[self.index] = distance;
        self.index = (self.index + 1) % 8;
        
        if self.sample_count < 8 {
            self.sample_count += 1;
        }
    }
    
    /// 获取滤波后的距离 (中位数滤波)
    pub fn get_filtered_distance(&self) -> f32 {
        if self.sample_count == 0 {
            return 0.0;
        }
        
        let mut sorted_samples = [0.0f32; 8];
        for i in 0..self.sample_count {
            sorted_samples[i] = self.samples[i];
        }
        
        // 简单的冒泡排序
        for i in 0..self.sample_count {
            for j in 0..(self.sample_count - 1 - i) {
                if sorted_samples[j] > sorted_samples[j + 1] {
                    let temp = sorted_samples[j];
                    sorted_samples[j] = sorted_samples[j + 1];
                    sorted_samples[j + 1] = temp;
                }
            }
        }
        
        // 返回中位数
        sorted_samples[self.sample_count / 2]
    }
    
    /// 获取平均距离
    pub fn get_average_distance(&self) -> f32 {
        if self.sample_count == 0 {
            return 0.0;
        }
        
        let mut sum = 0.0;
        for i in 0..self.sample_count {
            sum += self.samples[i];
        }
        
        sum / self.sample_count as f32
    }
    
    /// 重置滤波器
    pub fn reset(&mut self) {
        self.sample_count = 0;
        self.index = 0;
    }
}

/// 障碍物检测器
pub struct ObstacleDetector {
    distance_threshold: f32,
    detection_count: u32,
    detection_threshold: u32,
    obstacle_detected: bool,
    filter: DistanceFilter,
}

impl ObstacleDetector {
    /// 创建障碍物检测器
    pub fn new(distance_threshold: f32, detection_threshold: u32) -> Self {
        Self {
            distance_threshold,
            detection_count: 0,
            detection_threshold,
            obstacle_detected: false,
            filter: DistanceFilter::new(),
        }
    }
    
    /// 更新距离数据
    pub fn update(&mut self, distance: f32) {
        self.filter.add_sample(distance);
        let filtered_distance = self.filter.get_filtered_distance();
        
        if filtered_distance < self.distance_threshold {
            self.detection_count += 1;
            if self.detection_count >= self.detection_threshold {
                self.obstacle_detected = true;
            }
        } else {
            self.detection_count = 0;
            self.obstacle_detected = false;
        }
    }
    
    /// 检查是否检测到障碍物
    pub fn is_obstacle_detected(&self) -> bool {
        self.obstacle_detected
    }
    
    /// 获取滤波后的距离
    pub fn get_filtered_distance(&self) -> f32 {
        self.filter.get_filtered_distance()
    }
    
    /// 设置检测阈值
    pub fn set_distance_threshold(&mut self, threshold: f32) {
        self.distance_threshold = threshold;
    }
}

/// 定时器1捕获中断处理程序
#[interrupt]
fn TIM1_CC() {
    static mut EDGE_STATE: bool = false; // true = 上升沿, false = 下降沿
    
    cortex_m::interrupt::free(|cs| {
        if let (Some(ref mut timer), Some(ref mut sensor)) = (
            TIMER.borrow(cs).borrow_mut().as_mut(),
            ULTRASONIC_SENSOR.borrow(cs).borrow_mut().as_mut(),
        ) {
            if timer.is_capture_interrupt_pending(Channel::C1) {
                let timestamp = timer.get_capture_value(Channel::C1);
                
                if *EDGE_STATE {
                    // 上升沿 - 回声开始
                    sensor.handle_echo_start(timestamp);
                    // 切换到下降沿检测
                    timer.set_capture_edge(Channel::C1, false);
                    *EDGE_STATE = false;
                } else {
                    // 下降沿 - 回声结束
                    sensor.handle_echo_end(timestamp);
                    // 切换到上升沿检测
                    timer.set_capture_edge(Channel::C1, true);
                    *EDGE_STATE = true;
                }
                
                timer.clear_capture_interrupt(Channel::C1);
            }
        }
    });
}
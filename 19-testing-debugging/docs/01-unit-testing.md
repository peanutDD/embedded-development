# 单元测试

单元测试是嵌入式软件开发中确保代码质量和可靠性的重要手段。本文档介绍如何在Rust嵌入式项目中实施有效的单元测试策略。

## 测试基础

### 测试框架概述

Rust内置的测试框架为嵌入式开发提供了强大的测试能力：

```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_basic_functionality() {
        assert_eq!(2 + 2, 4);
    }
    
    #[test]
    #[should_panic]
    fn test_panic_condition() {
        panic!("This test should panic");
    }
    
    #[test]
    #[ignore]
    fn expensive_test() {
        // 耗时测试，默认不运行
    }
}
```

### 嵌入式测试特点

嵌入式测试需要考虑的特殊因素：

```rust
// 条件编译用于测试
#[cfg(test)]
use std::vec::Vec;

#[cfg(not(test))]
use heapless::Vec;

// 模拟硬件抽象层
#[cfg(test)]
mod mock_hal {
    pub struct MockGpio {
        state: bool,
    }
    
    impl MockGpio {
        pub fn new() -> Self {
            Self { state: false }
        }
        
        pub fn set_high(&mut self) {
            self.state = true;
        }
        
        pub fn set_low(&mut self) {
            self.state = false;
        }
        
        pub fn is_high(&self) -> bool {
            self.state
        }
    }
}
```

## 测试策略

### 1. 纯函数测试

纯函数是最容易测试的组件：

```rust
// 数学运算函数
pub fn calculate_pid(error: f32, integral: f32, derivative: f32, 
                    kp: f32, ki: f32, kd: f32) -> f32 {
    kp * error + ki * integral + kd * derivative
}

// 数据处理函数
pub fn moving_average(samples: &[f32], window_size: usize) -> f32 {
    if samples.is_empty() || window_size == 0 {
        return 0.0;
    }
    
    let start = samples.len().saturating_sub(window_size);
    let sum: f32 = samples[start..].iter().sum();
    sum / (samples.len() - start) as f32
}

// 协议解析函数
pub fn parse_modbus_frame(data: &[u8]) -> Result<ModbusFrame, ModbusError> {
    if data.len() < 4 {
        return Err(ModbusError::InvalidLength);
    }
    
    let address = data[0];
    let function = data[1];
    let crc = u16::from_le_bytes([data[data.len()-2], data[data.len()-1]]);
    
    // CRC校验
    if calculate_crc(&data[..data.len()-2]) != crc {
        return Err(ModbusError::CrcError);
    }
    
    Ok(ModbusFrame {
        address,
        function,
        data: &data[2..data.len()-2],
    })
}

#[cfg(test)]
mod pure_function_tests {
    use super::*;
    
    #[test]
    fn test_pid_calculation() {
        let result = calculate_pid(1.0, 0.5, 0.2, 1.0, 0.1, 0.01);
        assert!((result - 1.052).abs() < 0.001);
    }
    
    #[test]
    fn test_moving_average() {
        let samples = [1.0, 2.0, 3.0, 4.0, 5.0];
        assert_eq!(moving_average(&samples, 3), 4.0);
        assert_eq!(moving_average(&samples, 10), 3.0);
        assert_eq!(moving_average(&[], 3), 0.0);
    }
    
    #[test]
    fn test_modbus_parsing() {
        let valid_frame = [0x01, 0x03, 0x00, 0x01, 0xD5, 0xCA];
        let result = parse_modbus_frame(&valid_frame);
        assert!(result.is_ok());
        
        let frame = result.unwrap();
        assert_eq!(frame.address, 0x01);
        assert_eq!(frame.function, 0x03);
    }
    
    #[test]
    fn test_modbus_invalid_crc() {
        let invalid_frame = [0x01, 0x03, 0x00, 0x01, 0x00, 0x00];
        let result = parse_modbus_frame(&invalid_frame);
        assert_eq!(result.unwrap_err(), ModbusError::CrcError);
    }
}
```

### 2. 状态机测试

状态机是嵌入式系统的核心组件：

```rust
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemState {
    Idle,
    Running,
    Error,
    Shutdown,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SystemEvent {
    Start,
    Stop,
    ErrorOccurred,
    ErrorCleared,
    ShutdownRequested,
}

pub struct StateMachine {
    current_state: SystemState,
    error_count: u32,
}

impl StateMachine {
    pub fn new() -> Self {
        Self {
            current_state: SystemState::Idle,
            error_count: 0,
        }
    }
    
    pub fn handle_event(&mut self, event: SystemEvent) -> SystemState {
        use SystemState::*;
        use SystemEvent::*;
        
        self.current_state = match (self.current_state, event) {
            (Idle, Start) => Running,
            (Running, Stop) => Idle,
            (Running, ErrorOccurred) => {
                self.error_count += 1;
                Error
            },
            (Error, ErrorCleared) => {
                if self.error_count < 3 {
                    Idle
                } else {
                    Shutdown
                }
            },
            (_, ShutdownRequested) => Shutdown,
            (state, _) => state, // 无效转换，保持当前状态
        };
        
        self.current_state
    }
    
    pub fn get_state(&self) -> SystemState {
        self.current_state
    }
    
    pub fn get_error_count(&self) -> u32 {
        self.error_count
    }
}

#[cfg(test)]
mod state_machine_tests {
    use super::*;
    
    #[test]
    fn test_initial_state() {
        let sm = StateMachine::new();
        assert_eq!(sm.get_state(), SystemState::Idle);
        assert_eq!(sm.get_error_count(), 0);
    }
    
    #[test]
    fn test_normal_operation() {
        let mut sm = StateMachine::new();
        
        // 启动系统
        assert_eq!(sm.handle_event(SystemEvent::Start), SystemState::Running);
        
        // 停止系统
        assert_eq!(sm.handle_event(SystemEvent::Stop), SystemState::Idle);
    }
    
    #[test]
    fn test_error_handling() {
        let mut sm = StateMachine::new();
        
        // 启动并发生错误
        sm.handle_event(SystemEvent::Start);
        assert_eq!(sm.handle_event(SystemEvent::ErrorOccurred), SystemState::Error);
        assert_eq!(sm.get_error_count(), 1);
        
        // 清除错误
        assert_eq!(sm.handle_event(SystemEvent::ErrorCleared), SystemState::Idle);
    }
    
    #[test]
    fn test_multiple_errors_lead_to_shutdown() {
        let mut sm = StateMachine::new();
        
        // 模拟多次错误
        for _ in 0..3 {
            sm.handle_event(SystemEvent::Start);
            sm.handle_event(SystemEvent::ErrorOccurred);
            sm.handle_event(SystemEvent::ErrorCleared);
        }
        
        // 第四次错误应该导致关机
        sm.handle_event(SystemEvent::Start);
        sm.handle_event(SystemEvent::ErrorOccurred);
        assert_eq!(sm.handle_event(SystemEvent::ErrorCleared), SystemState::Shutdown);
    }
    
    #[test]
    fn test_shutdown_is_final_state() {
        let mut sm = StateMachine::new();
        
        // 进入关机状态
        sm.handle_event(SystemEvent::ShutdownRequested);
        assert_eq!(sm.get_state(), SystemState::Shutdown);
        
        // 任何事件都不应该改变状态
        assert_eq!(sm.handle_event(SystemEvent::Start), SystemState::Shutdown);
        assert_eq!(sm.handle_event(SystemEvent::ErrorCleared), SystemState::Shutdown);
    }
}
```

### 3. 数据结构测试

测试自定义数据结构的正确性：

```rust
use heapless::{Vec, FnvIndexMap};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorReading {
    pub timestamp: u64,
    pub value: f32,
    pub sensor_id: u8,
}

pub struct SensorBuffer {
    readings: Vec<SensorReading, 32>,
    capacity: usize,
}

impl SensorBuffer {
    pub fn new(capacity: usize) -> Self {
        Self {
            readings: Vec::new(),
            capacity: capacity.min(32),
        }
    }
    
    pub fn add_reading(&mut self, reading: SensorReading) -> Result<(), SensorReading> {
        if self.readings.len() >= self.capacity {
            self.readings.remove(0); // 移除最老的读数
        }
        
        self.readings.push(reading)
    }
    
    pub fn get_latest(&self) -> Option<&SensorReading> {
        self.readings.last()
    }
    
    pub fn get_average(&self, sensor_id: u8) -> Option<f32> {
        let values: Vec<f32, 32> = self.readings
            .iter()
            .filter(|r| r.sensor_id == sensor_id)
            .map(|r| r.value)
            .collect();
        
        if values.is_empty() {
            None
        } else {
            let sum: f32 = values.iter().sum();
            Some(sum / values.len() as f32)
        }
    }
    
    pub fn get_readings_in_range(&self, start_time: u64, end_time: u64) -> Vec<&SensorReading, 32> {
        self.readings
            .iter()
            .filter(|r| r.timestamp >= start_time && r.timestamp <= end_time)
            .collect()
    }
    
    pub fn len(&self) -> usize {
        self.readings.len()
    }
    
    pub fn is_empty(&self) -> bool {
        self.readings.is_empty()
    }
}

#[cfg(test)]
mod sensor_buffer_tests {
    use super::*;
    
    #[test]
    fn test_buffer_creation() {
        let buffer = SensorBuffer::new(10);
        assert_eq!(buffer.len(), 0);
        assert!(buffer.is_empty());
        assert!(buffer.get_latest().is_none());
    }
    
    #[test]
    fn test_add_reading() {
        let mut buffer = SensorBuffer::new(3);
        
        let reading1 = SensorReading {
            timestamp: 1000,
            value: 25.5,
            sensor_id: 1,
        };
        
        assert!(buffer.add_reading(reading1).is_ok());
        assert_eq!(buffer.len(), 1);
        assert_eq!(buffer.get_latest(), Some(&reading1));
    }
    
    #[test]
    fn test_buffer_overflow() {
        let mut buffer = SensorBuffer::new(2);
        
        let readings = [
            SensorReading { timestamp: 1000, value: 1.0, sensor_id: 1 },
            SensorReading { timestamp: 2000, value: 2.0, sensor_id: 1 },
            SensorReading { timestamp: 3000, value: 3.0, sensor_id: 1 },
        ];
        
        for reading in readings.iter() {
            buffer.add_reading(*reading).ok();
        }
        
        assert_eq!(buffer.len(), 2);
        assert_eq!(buffer.get_latest().unwrap().timestamp, 3000);
    }
    
    #[test]
    fn test_average_calculation() {
        let mut buffer = SensorBuffer::new(10);
        
        let readings = [
            SensorReading { timestamp: 1000, value: 10.0, sensor_id: 1 },
            SensorReading { timestamp: 2000, value: 20.0, sensor_id: 1 },
            SensorReading { timestamp: 3000, value: 30.0, sensor_id: 2 },
        ];
        
        for reading in readings.iter() {
            buffer.add_reading(*reading).ok();
        }
        
        assert_eq!(buffer.get_average(1), Some(15.0));
        assert_eq!(buffer.get_average(2), Some(30.0));
        assert_eq!(buffer.get_average(3), None);
    }
    
    #[test]
    fn test_time_range_query() {
        let mut buffer = SensorBuffer::new(10);
        
        let readings = [
            SensorReading { timestamp: 1000, value: 1.0, sensor_id: 1 },
            SensorReading { timestamp: 2000, value: 2.0, sensor_id: 1 },
            SensorReading { timestamp: 3000, value: 3.0, sensor_id: 1 },
            SensorReading { timestamp: 4000, value: 4.0, sensor_id: 1 },
        ];
        
        for reading in readings.iter() {
            buffer.add_reading(*reading).ok();
        }
        
        let range_readings = buffer.get_readings_in_range(1500, 3500);
        assert_eq!(range_readings.len(), 2);
        assert_eq!(range_readings[0].timestamp, 2000);
        assert_eq!(range_readings[1].timestamp, 3000);
    }
}
```

## 模拟和存根

### 硬件抽象层模拟

```rust
// 真实的GPIO trait
pub trait GpioPin {
    fn set_high(&mut self);
    fn set_low(&mut self);
    fn is_high(&self) -> bool;
    fn toggle(&mut self);
}

// 生产环境实现
#[cfg(not(test))]
pub struct HardwareGpio {
    // 实际硬件寄存器
}

#[cfg(not(test))]
impl GpioPin for HardwareGpio {
    fn set_high(&mut self) {
        // 操作硬件寄存器
    }
    
    fn set_low(&mut self) {
        // 操作硬件寄存器
    }
    
    fn is_high(&self) -> bool {
        // 读取硬件寄存器
        false
    }
    
    fn toggle(&mut self) {
        if self.is_high() {
            self.set_low();
        } else {
            self.set_high();
        }
    }
}

// 测试环境模拟
#[cfg(test)]
pub struct MockGpio {
    state: bool,
    toggle_count: u32,
}

#[cfg(test)]
impl MockGpio {
    pub fn new() -> Self {
        Self {
            state: false,
            toggle_count: 0,
        }
    }
    
    pub fn get_toggle_count(&self) -> u32 {
        self.toggle_count
    }
}

#[cfg(test)]
impl GpioPin for MockGpio {
    fn set_high(&mut self) {
        self.state = true;
    }
    
    fn set_low(&mut self) {
        self.state = false;
    }
    
    fn is_high(&self) -> bool {
        self.state
    }
    
    fn toggle(&mut self) {
        self.state = !self.state;
        self.toggle_count += 1;
    }
}

// 使用GPIO的业务逻辑
pub struct LedController<T: GpioPin> {
    led: T,
    blink_pattern: Vec<u32, 8>, // 闪烁模式（毫秒）
    current_step: usize,
}

impl<T: GpioPin> LedController<T> {
    pub fn new(led: T) -> Self {
        Self {
            led,
            blink_pattern: Vec::new(),
            current_step: 0,
        }
    }
    
    pub fn set_blink_pattern(&mut self, pattern: &[u32]) -> Result<(), ()> {
        self.blink_pattern.clear();
        for &duration in pattern {
            self.blink_pattern.push(duration).map_err(|_| ())?;
        }
        self.current_step = 0;
        Ok(())
    }
    
    pub fn update(&mut self) -> Option<u32> {
        if self.blink_pattern.is_empty() {
            return None;
        }
        
        self.led.toggle();
        let next_delay = self.blink_pattern[self.current_step];
        self.current_step = (self.current_step + 1) % self.blink_pattern.len();
        
        Some(next_delay)
    }
    
    pub fn turn_on(&mut self) {
        self.led.set_high();
    }
    
    pub fn turn_off(&mut self) {
        self.led.set_low();
    }
    
    pub fn is_on(&self) -> bool {
        self.led.is_high()
    }
}

#[cfg(test)]
mod led_controller_tests {
    use super::*;
    
    #[test]
    fn test_led_basic_operations() {
        let gpio = MockGpio::new();
        let mut controller = LedController::new(gpio);
        
        assert!(!controller.is_on());
        
        controller.turn_on();
        assert!(controller.is_on());
        
        controller.turn_off();
        assert!(!controller.is_on());
    }
    
    #[test]
    fn test_blink_pattern() {
        let gpio = MockGpio::new();
        let mut controller = LedController::new(gpio);
        
        let pattern = [100, 200, 100, 500];
        assert!(controller.set_blink_pattern(&pattern).is_ok());
        
        // 测试闪烁序列
        assert_eq!(controller.update(), Some(100));
        assert_eq!(controller.update(), Some(200));
        assert_eq!(controller.update(), Some(100));
        assert_eq!(controller.update(), Some(500));
        assert_eq!(controller.update(), Some(100)); // 循环回到开始
    }
    
    #[test]
    fn test_toggle_count() {
        let mut gpio = MockGpio::new();
        let mut controller = LedController::new(gpio);
        
        let pattern = [100, 200];
        controller.set_blink_pattern(&pattern).ok();
        
        // 执行几次更新
        for _ in 0..4 {
            controller.update();
        }
        
        // 检查GPIO被切换的次数
        // 注意：这里需要访问内部的GPIO，实际实现可能需要不同的方法
    }
}
```

### 时间模拟

```rust
// 时间抽象
pub trait TimeProvider {
    fn now_ms(&self) -> u64;
    fn delay_ms(&self, ms: u32);
}

// 生产环境时间提供者
#[cfg(not(test))]
pub struct SystemTime;

#[cfg(not(test))]
impl TimeProvider for SystemTime {
    fn now_ms(&self) -> u64 {
        // 从系统时钟获取时间
        0
    }
    
    fn delay_ms(&self, ms: u32) {
        // 实际延时
    }
}

// 测试环境时间模拟
#[cfg(test)]
pub struct MockTime {
    current_time: std::cell::RefCell<u64>,
}

#[cfg(test)]
impl MockTime {
    pub fn new() -> Self {
        Self {
            current_time: std::cell::RefCell::new(0),
        }
    }
    
    pub fn advance_time(&self, ms: u64) {
        *self.current_time.borrow_mut() += ms;
    }
    
    pub fn set_time(&self, ms: u64) {
        *self.current_time.borrow_mut() = ms;
    }
}

#[cfg(test)]
impl TimeProvider for MockTime {
    fn now_ms(&self) -> u64 {
        *self.current_time.borrow()
    }
    
    fn delay_ms(&self, ms: u32) {
        self.advance_time(ms as u64);
    }
}

// 使用时间的组件
pub struct Timer<T: TimeProvider> {
    time_provider: T,
    start_time: u64,
    duration: u64,
}

impl<T: TimeProvider> Timer<T> {
    pub fn new(time_provider: T, duration_ms: u64) -> Self {
        let start_time = time_provider.now_ms();
        Self {
            time_provider,
            start_time,
            duration: duration_ms,
        }
    }
    
    pub fn is_expired(&self) -> bool {
        self.time_provider.now_ms() >= self.start_time + self.duration
    }
    
    pub fn remaining_ms(&self) -> u64 {
        let current = self.time_provider.now_ms();
        if current >= self.start_time + self.duration {
            0
        } else {
            self.start_time + self.duration - current
        }
    }
    
    pub fn reset(&mut self) {
        self.start_time = self.time_provider.now_ms();
    }
}

#[cfg(test)]
mod timer_tests {
    use super::*;
    
    #[test]
    fn test_timer_creation() {
        let time = MockTime::new();
        let timer = Timer::new(&time, 1000);
        
        assert!(!timer.is_expired());
        assert_eq!(timer.remaining_ms(), 1000);
    }
    
    #[test]
    fn test_timer_expiration() {
        let time = MockTime::new();
        let timer = Timer::new(&time, 1000);
        
        // 推进时间
        time.advance_time(500);
        assert!(!timer.is_expired());
        assert_eq!(timer.remaining_ms(), 500);
        
        time.advance_time(500);
        assert!(timer.is_expired());
        assert_eq!(timer.remaining_ms(), 0);
    }
    
    #[test]
    fn test_timer_reset() {
        let time = MockTime::new();
        let mut timer = Timer::new(&time, 1000);
        
        time.advance_time(800);
        assert_eq!(timer.remaining_ms(), 200);
        
        timer.reset();
        assert_eq!(timer.remaining_ms(), 1000);
    }
}
```

## 属性测试

使用属性测试验证函数的通用性质：

```rust
// 需要添加到Cargo.toml:
// [dev-dependencies]
// proptest = "1.0"

#[cfg(test)]
mod property_tests {
    use super::*;
    use proptest::prelude::*;
    
    // 测试数学函数的性质
    proptest! {
        #[test]
        fn test_moving_average_properties(
            samples in prop::collection::vec(any::<f32>(), 1..100),
            window_size in 1usize..50
        ) {
            let avg = moving_average(&samples, window_size);
            
            // 性质1：平均值应该在最小值和最大值之间
            if let (Some(&min), Some(&max)) = (samples.iter().min_by(|a, b| a.partial_cmp(b).unwrap()),
                                              samples.iter().max_by(|a, b| a.partial_cmp(b).unwrap())) {
                prop_assert!(avg >= min && avg <= max);
            }
            
            // 性质2：如果所有样本相同，平均值应该等于样本值
            if samples.iter().all(|&x| x == samples[0]) {
                prop_assert!((avg - samples[0]).abs() < f32::EPSILON);
            }
        }
        
        #[test]
        fn test_crc_properties(data in prop::collection::vec(any::<u8>(), 0..1000)) {
            let crc1 = calculate_crc(&data);
            let crc2 = calculate_crc(&data);
            
            // 性质：相同输入应该产生相同输出
            prop_assert_eq!(crc1, crc2);
            
            // 性质：空数据的CRC应该是固定值
            if data.is_empty() {
                prop_assert_eq!(crc1, 0xFFFF); // 假设的初始值
            }
        }
        
        #[test]
        fn test_state_machine_properties(
            events in prop::collection::vec(
                prop_oneof![
                    Just(SystemEvent::Start),
                    Just(SystemEvent::Stop),
                    Just(SystemEvent::ErrorOccurred),
                    Just(SystemEvent::ErrorCleared),
                    Just(SystemEvent::ShutdownRequested),
                ], 
                0..50
            )
        ) {
            let mut sm = StateMachine::new();
            
            for event in events {
                let old_state = sm.get_state();
                let new_state = sm.handle_event(event);
                
                // 性质：状态转换应该是确定性的
                prop_assert_eq!(new_state, sm.get_state());
                
                // 性质：一旦进入Shutdown状态就不能退出
                if old_state == SystemState::Shutdown {
                    prop_assert_eq!(new_state, SystemState::Shutdown);
                }
            }
        }
    }
}
```

## 基准测试

测试性能关键代码的执行时间：

```rust
// 需要添加到Cargo.toml:
// [dev-dependencies]
// criterion = "0.5"

#[cfg(test)]
mod benchmarks {
    use super::*;
    use criterion::{black_box, criterion_group, criterion_main, Criterion};
    
    fn benchmark_moving_average(c: &mut Criterion) {
        let samples: Vec<f32> = (0..1000).map(|i| i as f32).collect();
        
        c.bench_function("moving_average_small_window", |b| {
            b.iter(|| moving_average(black_box(&samples), black_box(10)))
        });
        
        c.bench_function("moving_average_large_window", |b| {
            b.iter(|| moving_average(black_box(&samples), black_box(100)))
        });
    }
    
    fn benchmark_crc_calculation(c: &mut Criterion) {
        let data: Vec<u8> = (0..1024).map(|i| (i % 256) as u8).collect();
        
        c.bench_function("crc_1kb", |b| {
            b.iter(|| calculate_crc(black_box(&data)))
        });
    }
    
    fn benchmark_sensor_buffer(c: &mut Criterion) {
        let mut buffer = SensorBuffer::new(32);
        let reading = SensorReading {
            timestamp: 1000,
            value: 25.5,
            sensor_id: 1,
        };
        
        c.bench_function("sensor_buffer_add", |b| {
            b.iter(|| {
                buffer.add_reading(black_box(reading)).ok();
            })
        });
        
        // 填充缓冲区用于其他测试
        for i in 0..32 {
            buffer.add_reading(SensorReading {
                timestamp: i as u64,
                value: i as f32,
                sensor_id: 1,
            }).ok();
        }
        
        c.bench_function("sensor_buffer_average", |b| {
            b.iter(|| buffer.get_average(black_box(1)))
        });
    }
    
    criterion_group!(benches, benchmark_moving_average, benchmark_crc_calculation, benchmark_sensor_buffer);
    criterion_main!(benches);
}
```

## 测试工具和辅助函数

```rust
#[cfg(test)]
mod test_utils {
    use super::*;
    
    // 浮点数比较辅助函数
    pub fn assert_float_eq(a: f32, b: f32, epsilon: f32) {
        assert!((a - b).abs() < epsilon, "Expected {}, got {}, difference: {}", b, a, (a - b).abs());
    }
    
    // 创建测试用的传感器读数
    pub fn create_test_readings(count: usize, sensor_id: u8) -> Vec<SensorReading> {
        (0..count)
            .map(|i| SensorReading {
                timestamp: (i * 1000) as u64,
                value: (i as f32) * 0.1,
                sensor_id,
            })
            .collect()
    }
    
    // 验证状态转换序列
    pub fn verify_state_transitions(
        initial_state: SystemState,
        events: &[SystemEvent],
        expected_states: &[SystemState],
    ) {
        let mut sm = StateMachine::new();
        // 设置初始状态（可能需要额外的方法）
        
        for (i, &event) in events.iter().enumerate() {
            let new_state = sm.handle_event(event);
            assert_eq!(
                new_state, 
                expected_states[i],
                "Event {} ({:?}) resulted in state {:?}, expected {:?}",
                i, event, new_state, expected_states[i]
            );
        }
    }
    
    // 模拟传感器数据生成器
    pub struct SensorDataGenerator {
        base_value: f32,
        noise_amplitude: f32,
        trend: f32,
        current_time: u64,
    }
    
    impl SensorDataGenerator {
        pub fn new(base_value: f32, noise_amplitude: f32, trend: f32) -> Self {
            Self {
                base_value,
                noise_amplitude,
                trend,
                current_time: 0,
            }
        }
        
        pub fn next_reading(&mut self, sensor_id: u8) -> SensorReading {
            use std::f32::consts::PI;
            
            // 生成带噪声和趋势的数据
            let noise = (self.current_time as f32 * 0.1).sin() * self.noise_amplitude;
            let trend_component = self.trend * self.current_time as f32 * 0.001;
            let value = self.base_value + noise + trend_component;
            
            let reading = SensorReading {
                timestamp: self.current_time,
                value,
                sensor_id,
            };
            
            self.current_time += 100; // 100ms间隔
            reading
        }
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;
    use test_utils::*;
    
    #[test]
    fn test_sensor_system_integration() {
        let mut buffer = SensorBuffer::new(10);
        let mut generator = SensorDataGenerator::new(25.0, 2.0, 0.1);
        
        // 生成一系列传感器读数
        for _ in 0..15 {
            let reading = generator.next_reading(1);
            buffer.add_reading(reading).ok();
        }
        
        // 验证缓冲区行为
        assert_eq!(buffer.len(), 10); // 应该限制在容量内
        
        // 验证平均值计算
        let average = buffer.get_average(1).unwrap();
        assert!(average > 20.0 && average < 30.0); // 应该在合理范围内
        
        // 验证时间范围查询
        let recent_readings = buffer.get_readings_in_range(1000, 1500);
        assert!(!recent_readings.is_empty());
    }
    
    #[test]
    fn test_led_controller_with_timer() {
        let time = MockTime::new();
        let gpio = MockGpio::new();
        let mut controller = LedController::new(gpio);
        let mut timer = Timer::new(&time, 100);
        
        // 设置闪烁模式
        controller.set_blink_pattern(&[100, 200]).ok();
        
        // 模拟时间循环
        for cycle in 0..5 {
            if timer.is_expired() {
                if let Some(next_delay) = controller.update() {
                    timer = Timer::new(&time, next_delay as u64);
                }
            }
            
            time.advance_time(50);
        }
        
        // 验证LED状态变化
        // 这里需要根据具体实现验证
    }
}
```

## 总结

有效的单元测试策略包括：

1. **全面覆盖**：测试正常情况、边界条件和错误情况
2. **模拟硬件**：使用trait和依赖注入实现硬件抽象
3. **属性测试**：验证函数的数学性质和不变量
4. **性能测试**：确保关键代码满足性能要求
5. **集成测试**：验证组件间的交互
6. **测试工具**：开发辅助函数简化测试编写

通过系统性的测试方法，可以显著提高嵌入式软件的质量和可靠性。
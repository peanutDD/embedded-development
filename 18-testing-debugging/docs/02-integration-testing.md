# 集成测试

集成测试验证多个组件协同工作的正确性，是确保嵌入式系统整体功能的关键测试方法。本文档介绍如何在Rust嵌入式项目中实施有效的集成测试。

## 集成测试概述

### 测试层次

嵌入式系统的集成测试通常包含以下层次：

```rust
// 测试架构层次
pub mod integration_test_framework {
    use std::collections::HashMap;
    use std::sync::{Arc, Mutex};
    
    // 测试环境配置
    #[derive(Debug, Clone)]
    pub struct TestEnvironment {
        pub hardware_simulation: bool,
        pub real_time_constraints: bool,
        pub network_simulation: bool,
        pub power_simulation: bool,
    }
    
    impl Default for TestEnvironment {
        fn default() -> Self {
            Self {
                hardware_simulation: true,
                real_time_constraints: false,
                network_simulation: true,
                power_simulation: false,
            }
        }
    }
    
    // 测试执行器
    pub struct TestExecutor {
        environment: TestEnvironment,
        test_results: Arc<Mutex<HashMap<String, TestResult>>>,
    }
    
    #[derive(Debug, Clone)]
    pub struct TestResult {
        pub passed: bool,
        pub duration_ms: u64,
        pub error_message: Option<String>,
        pub metrics: HashMap<String, f64>,
    }
    
    impl TestExecutor {
        pub fn new(environment: TestEnvironment) -> Self {
            Self {
                environment,
                test_results: Arc::new(Mutex::new(HashMap::new())),
            }
        }
        
        pub fn run_test<F>(&self, test_name: &str, test_fn: F) -> TestResult
        where
            F: FnOnce() -> Result<HashMap<String, f64>, String>,
        {
            let start_time = std::time::Instant::now();
            
            let result = match test_fn() {
                Ok(metrics) => TestResult {
                    passed: true,
                    duration_ms: start_time.elapsed().as_millis() as u64,
                    error_message: None,
                    metrics,
                },
                Err(error) => TestResult {
                    passed: false,
                    duration_ms: start_time.elapsed().as_millis() as u64,
                    error_message: Some(error),
                    metrics: HashMap::new(),
                },
            };
            
            self.test_results.lock().unwrap().insert(test_name.to_string(), result.clone());
            result
        }
        
        pub fn get_test_summary(&self) -> TestSummary {
            let results = self.test_results.lock().unwrap();
            let total_tests = results.len();
            let passed_tests = results.values().filter(|r| r.passed).count();
            let total_duration: u64 = results.values().map(|r| r.duration_ms).sum();
            
            TestSummary {
                total_tests,
                passed_tests,
                failed_tests: total_tests - passed_tests,
                total_duration_ms: total_duration,
                pass_rate: if total_tests > 0 { 
                    (passed_tests as f64 / total_tests as f64) * 100.0 
                } else { 
                    0.0 
                },
            }
        }
    }
    
    #[derive(Debug)]
    pub struct TestSummary {
        pub total_tests: usize,
        pub passed_tests: usize,
        pub failed_tests: usize,
        pub total_duration_ms: u64,
        pub pass_rate: f64,
    }
}
```

## 硬件抽象层集成测试

### 外设集成测试

```rust
use std::sync::{Arc, Mutex};
use std::collections::VecDeque;

// 模拟SPI总线
pub struct MockSpiBus {
    transactions: Arc<Mutex<VecDeque<SpiTransaction>>>,
    current_transaction: Arc<Mutex<Option<SpiTransaction>>>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SpiTransaction {
    pub write_data: Vec<u8>,
    pub read_data: Vec<u8>,
    pub expected_write: Vec<u8>,
}

impl MockSpiBus {
    pub fn new() -> Self {
        Self {
            transactions: Arc::new(Mutex::new(VecDeque::new())),
            current_transaction: Arc::new(Mutex::new(None)),
        }
    }
    
    pub fn expect_transaction(&self, transaction: SpiTransaction) {
        self.transactions.lock().unwrap().push_back(transaction);
    }
    
    pub fn transfer(&self, write_data: &[u8]) -> Result<Vec<u8>, SpiError> {
        let mut transactions = self.transactions.lock().unwrap();
        
        if let Some(expected) = transactions.pop_front() {
            if expected.expected_write != write_data {
                return Err(SpiError::UnexpectedData);
            }
            Ok(expected.read_data)
        } else {
            Err(SpiError::NoExpectedTransaction)
        }
    }
    
    pub fn verify_all_transactions_completed(&self) -> bool {
        self.transactions.lock().unwrap().is_empty()
    }
}

#[derive(Debug, PartialEq)]
pub enum SpiError {
    UnexpectedData,
    NoExpectedTransaction,
    TransferFailed,
}

// 传感器驱动程序
pub struct TemperatureSensor<T> {
    spi: T,
    cs_pin: MockGpio,
}

impl<T> TemperatureSensor<T>
where
    T: SpiTransfer,
{
    pub fn new(spi: T, cs_pin: MockGpio) -> Self {
        Self { spi, cs_pin }
    }
    
    pub fn read_temperature(&mut self) -> Result<f32, SensorError> {
        // 选择芯片
        self.cs_pin.set_low();
        
        // 发送读取命令
        let command = [0x01, 0x00]; // 假设的读取温度命令
        let response = self.spi.transfer(&command)
            .map_err(|_| SensorError::CommunicationError)?;
        
        // 取消选择芯片
        self.cs_pin.set_high();
        
        // 解析响应
        if response.len() >= 2 {
            let raw_value = u16::from_be_bytes([response[0], response[1]]);
            let temperature = (raw_value as f32) * 0.0625; // 假设的转换公式
            Ok(temperature)
        } else {
            Err(SensorError::InvalidResponse)
        }
    }
    
    pub fn configure(&mut self, config: SensorConfig) -> Result<(), SensorError> {
        self.cs_pin.set_low();
        
        let command = [0x02, config.resolution as u8, config.sample_rate as u8];
        self.spi.transfer(&command)
            .map_err(|_| SensorError::CommunicationError)?;
        
        self.cs_pin.set_high();
        Ok(())
    }
}

pub trait SpiTransfer {
    fn transfer(&mut self, data: &[u8]) -> Result<Vec<u8>, SpiError>;
}

impl SpiTransfer for MockSpiBus {
    fn transfer(&mut self, data: &[u8]) -> Result<Vec<u8>, SpiError> {
        MockSpiBus::transfer(self, data)
    }
}

#[derive(Debug, PartialEq)]
pub enum SensorError {
    CommunicationError,
    InvalidResponse,
    ConfigurationError,
}

#[derive(Debug, Clone)]
pub struct SensorConfig {
    pub resolution: u8,
    pub sample_rate: u8,
}

// GPIO模拟（从之前的单元测试中复用）
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

#[cfg(test)]
mod sensor_integration_tests {
    use super::*;
    use integration_test_framework::*;
    
    #[test]
    fn test_temperature_sensor_read() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("temperature_sensor_read", || {
            let mut spi = MockSpiBus::new();
            let cs_pin = MockGpio::new();
            
            // 设置期望的SPI事务
            spi.expect_transaction(SpiTransaction {
                write_data: vec![0x01, 0x00],
                read_data: vec![0x19, 0x20], // 25.125°C
                expected_write: vec![0x01, 0x00],
            });
            
            let mut sensor = TemperatureSensor::new(spi, cs_pin);
            let temperature = sensor.read_temperature().unwrap();
            
            // 验证温度读数
            assert!((temperature - 25.125).abs() < 0.001);
            
            // 验证所有SPI事务都已完成
            assert!(sensor.spi.verify_all_transactions_completed());
            
            let mut metrics = HashMap::new();
            metrics.insert("temperature".to_string(), temperature as f64);
            Ok(metrics)
        });
        
        assert!(result.passed);
        assert!(result.metrics.contains_key("temperature"));
    }
    
    #[test]
    fn test_sensor_configuration() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("sensor_configuration", || {
            let mut spi = MockSpiBus::new();
            let cs_pin = MockGpio::new();
            
            // 设置配置命令的期望事务
            spi.expect_transaction(SpiTransaction {
                write_data: vec![0x02, 0x0C, 0x04],
                read_data: vec![0x00, 0x00, 0x00], // 配置确认
                expected_write: vec![0x02, 0x0C, 0x04],
            });
            
            let mut sensor = TemperatureSensor::new(spi, cs_pin);
            let config = SensorConfig {
                resolution: 12,
                sample_rate: 4,
            };
            
            let result = sensor.configure(config);
            assert!(result.is_ok());
            
            Ok(HashMap::new())
        });
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_sensor_error_handling() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("sensor_error_handling", || {
            let spi = MockSpiBus::new(); // 没有设置期望事务
            let cs_pin = MockGpio::new();
            
            let mut sensor = TemperatureSensor::new(spi, cs_pin);
            let result = sensor.read_temperature();
            
            // 应该返回通信错误
            assert_eq!(result.unwrap_err(), SensorError::CommunicationError);
            
            Ok(HashMap::new())
        });
        
        assert!(result.passed);
    }
}
```

### 通信协议集成测试

```rust
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

// UART通信模拟
pub struct MockUart {
    tx_buffer: Arc<Mutex<Vec<u8>>>,
    rx_buffer: Arc<Mutex<Vec<u8>>>,
    baud_rate: u32,
}

impl MockUart {
    pub fn new(baud_rate: u32) -> Self {
        Self {
            tx_buffer: Arc::new(Mutex::new(Vec::new())),
            rx_buffer: Arc::new(Mutex::new(Vec::new())),
            baud_rate,
        }
    }
    
    pub fn write(&self, data: &[u8]) -> Result<usize, UartError> {
        let mut buffer = self.tx_buffer.lock().unwrap();
        buffer.extend_from_slice(data);
        Ok(data.len())
    }
    
    pub fn read(&self, buffer: &mut [u8]) -> Result<usize, UartError> {
        let mut rx_buffer = self.rx_buffer.lock().unwrap();
        let bytes_to_read = buffer.len().min(rx_buffer.len());
        
        for i in 0..bytes_to_read {
            buffer[i] = rx_buffer.remove(0);
        }
        
        Ok(bytes_to_read)
    }
    
    pub fn inject_rx_data(&self, data: &[u8]) {
        let mut buffer = self.rx_buffer.lock().unwrap();
        buffer.extend_from_slice(data);
    }
    
    pub fn get_tx_data(&self) -> Vec<u8> {
        let mut buffer = self.tx_buffer.lock().unwrap();
        let data = buffer.clone();
        buffer.clear();
        data
    }
    
    pub fn simulate_transmission_delay(&self, data_len: usize) -> Duration {
        // 计算传输延迟：(bits_per_byte * data_len * 1000) / baud_rate 毫秒
        let bits = data_len * 10; // 8数据位 + 1起始位 + 1停止位
        let delay_ms = (bits * 1000) / self.baud_rate as usize;
        Duration::from_millis(delay_ms as u64)
    }
}

#[derive(Debug, PartialEq)]
pub enum UartError {
    BufferFull,
    NoData,
    ParityError,
    FramingError,
}

// Modbus协议实现
pub struct ModbusClient<T> {
    uart: T,
    slave_address: u8,
    timeout_ms: u32,
}

impl<T> ModbusClient<T>
where
    T: UartInterface,
{
    pub fn new(uart: T, slave_address: u8, timeout_ms: u32) -> Self {
        Self {
            uart,
            slave_address,
            timeout_ms,
        }
    }
    
    pub fn read_holding_registers(&mut self, start_address: u16, count: u16) -> Result<Vec<u16>, ModbusError> {
        // 构建Modbus RTU请求
        let mut request = Vec::new();
        request.push(self.slave_address);
        request.push(0x03); // 功能码：读保持寄存器
        request.extend_from_slice(&start_address.to_be_bytes());
        request.extend_from_slice(&count.to_be_bytes());
        
        // 计算并添加CRC
        let crc = calculate_modbus_crc(&request);
        request.extend_from_slice(&crc.to_le_bytes());
        
        // 发送请求
        self.uart.write(&request)
            .map_err(|_| ModbusError::CommunicationError)?;
        
        // 等待响应
        thread::sleep(Duration::from_millis(self.timeout_ms as u64));
        
        // 读取响应
        let mut response = vec![0u8; 256];
        let bytes_read = self.uart.read(&mut response)
            .map_err(|_| ModbusError::CommunicationError)?;
        
        response.truncate(bytes_read);
        
        // 解析响应
        self.parse_read_response(&response, count)
    }
    
    pub fn write_single_register(&mut self, address: u16, value: u16) -> Result<(), ModbusError> {
        let mut request = Vec::new();
        request.push(self.slave_address);
        request.push(0x06); // 功能码：写单个寄存器
        request.extend_from_slice(&address.to_be_bytes());
        request.extend_from_slice(&value.to_be_bytes());
        
        let crc = calculate_modbus_crc(&request);
        request.extend_from_slice(&crc.to_le_bytes());
        
        self.uart.write(&request)
            .map_err(|_| ModbusError::CommunicationError)?;
        
        // 等待确认响应
        thread::sleep(Duration::from_millis(self.timeout_ms as u64));
        
        let mut response = vec![0u8; 8];
        let bytes_read = self.uart.read(&mut response)
            .map_err(|_| ModbusError::CommunicationError)?;
        
        if bytes_read >= 8 && response[0] == self.slave_address && response[1] == 0x06 {
            Ok(())
        } else {
            Err(ModbusError::InvalidResponse)
        }
    }
    
    fn parse_read_response(&self, response: &[u8], expected_count: u16) -> Result<Vec<u16>, ModbusError> {
        if response.len() < 5 {
            return Err(ModbusError::InvalidResponse);
        }
        
        if response[0] != self.slave_address {
            return Err(ModbusError::InvalidSlaveAddress);
        }
        
        if response[1] != 0x03 {
            return Err(ModbusError::InvalidFunctionCode);
        }
        
        let byte_count = response[2] as usize;
        let expected_bytes = (expected_count * 2) as usize;
        
        if byte_count != expected_bytes {
            return Err(ModbusError::InvalidDataLength);
        }
        
        // 验证CRC
        let data_end = 3 + byte_count;
        if response.len() < data_end + 2 {
            return Err(ModbusError::InvalidResponse);
        }
        
        let received_crc = u16::from_le_bytes([response[data_end], response[data_end + 1]]);
        let calculated_crc = calculate_modbus_crc(&response[..data_end]);
        
        if received_crc != calculated_crc {
            return Err(ModbusError::CrcError);
        }
        
        // 解析寄存器值
        let mut registers = Vec::new();
        for i in 0..expected_count {
            let offset = 3 + (i * 2) as usize;
            let value = u16::from_be_bytes([response[offset], response[offset + 1]]);
            registers.push(value);
        }
        
        Ok(registers)
    }
}

pub trait UartInterface {
    fn write(&mut self, data: &[u8]) -> Result<usize, UartError>;
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, UartError>;
}

impl UartInterface for MockUart {
    fn write(&mut self, data: &[u8]) -> Result<usize, UartError> {
        MockUart::write(self, data)
    }
    
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, UartError> {
        MockUart::read(self, buffer)
    }
}

#[derive(Debug, PartialEq)]
pub enum ModbusError {
    CommunicationError,
    InvalidResponse,
    InvalidSlaveAddress,
    InvalidFunctionCode,
    InvalidDataLength,
    CrcError,
    Timeout,
}

// CRC计算函数
fn calculate_modbus_crc(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    
    for byte in data {
        crc ^= *byte as u16;
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    crc
}

#[cfg(test)]
mod modbus_integration_tests {
    use super::*;
    use integration_test_framework::*;
    
    #[test]
    fn test_modbus_read_registers() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("modbus_read_registers", || {
            let mut uart = MockUart::new(9600);
            
            // 模拟从设备响应
            let response = vec![
                0x01, // 从设备地址
                0x03, // 功能码
                0x04, // 字节数
                0x00, 0x64, // 寄存器1值：100
                0x00, 0xC8, // 寄存器2值：200
                0xB8, 0x44, // CRC
            ];
            uart.inject_rx_data(&response);
            
            let mut client = ModbusClient::new(uart, 0x01, 100);
            let registers = client.read_holding_registers(0x0000, 2).unwrap();
            
            assert_eq!(registers.len(), 2);
            assert_eq!(registers[0], 100);
            assert_eq!(registers[1], 200);
            
            // 验证发送的请求
            let tx_data = client.uart.get_tx_data();
            assert_eq!(tx_data[0], 0x01); // 从设备地址
            assert_eq!(tx_data[1], 0x03); // 功能码
            
            let mut metrics = HashMap::new();
            metrics.insert("registers_read".to_string(), registers.len() as f64);
            metrics.insert("register_1_value".to_string(), registers[0] as f64);
            metrics.insert("register_2_value".to_string(), registers[1] as f64);
            
            Ok(metrics)
        });
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_modbus_write_register() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("modbus_write_register", || {
            let mut uart = MockUart::new(9600);
            
            // 模拟写寄存器的确认响应
            let response = vec![
                0x01, // 从设备地址
                0x06, // 功能码
                0x00, 0x01, // 寄存器地址
                0x01, 0x2C, // 写入的值：300
                0x18, 0x0A, // CRC
            ];
            uart.inject_rx_data(&response);
            
            let mut client = ModbusClient::new(uart, 0x01, 100);
            let result = client.write_single_register(0x0001, 300);
            
            assert!(result.is_ok());
            
            // 验证发送的请求
            let tx_data = client.uart.get_tx_data();
            assert_eq!(tx_data[0], 0x01); // 从设备地址
            assert_eq!(tx_data[1], 0x06); // 功能码
            assert_eq!(u16::from_be_bytes([tx_data[2], tx_data[3]]), 0x0001); // 地址
            assert_eq!(u16::from_be_bytes([tx_data[4], tx_data[5]]), 300); // 值
            
            Ok(HashMap::new())
        });
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_modbus_crc_error() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("modbus_crc_error", || {
            let mut uart = MockUart::new(9600);
            
            // 模拟CRC错误的响应
            let response = vec![
                0x01, // 从设备地址
                0x03, // 功能码
                0x02, // 字节数
                0x00, 0x64, // 寄存器值
                0x00, 0x00, // 错误的CRC
            ];
            uart.inject_rx_data(&response);
            
            let mut client = ModbusClient::new(uart, 0x01, 100);
            let result = client.read_holding_registers(0x0000, 1);
            
            assert_eq!(result.unwrap_err(), ModbusError::CrcError);
            
            Ok(HashMap::new())
        });
        
        assert!(result.passed);
    }
}
```

## 系统级集成测试

### 多任务系统测试

```rust
use std::sync::{Arc, Mutex, Condvar};
use std::thread;
use std::time::{Duration, Instant};

// 任务调度器模拟
pub struct TaskScheduler {
    tasks: Vec<Task>,
    current_task: usize,
    running: Arc<Mutex<bool>>,
    scheduler_thread: Option<thread::JoinHandle<()>>,
}

#[derive(Clone)]
pub struct Task {
    pub id: u32,
    pub priority: u8,
    pub period_ms: u64,
    pub last_run: Arc<Mutex<Instant>>,
    pub run_count: Arc<Mutex<u32>>,
    pub max_execution_time: Arc<Mutex<Duration>>,
    pub task_fn: Arc<dyn Fn() + Send + Sync>,
}

impl TaskScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_task: 0,
            running: Arc::new(Mutex::new(false)),
            scheduler_thread: None,
        }
    }
    
    pub fn add_task(&mut self, task: Task) {
        self.tasks.push(task);
        // 按优先级排序
        self.tasks.sort_by(|a, b| b.priority.cmp(&a.priority));
    }
    
    pub fn start(&mut self) {
        *self.running.lock().unwrap() = true;
        
        let tasks = self.tasks.clone();
        let running = Arc::clone(&self.running);
        
        self.scheduler_thread = Some(thread::spawn(move || {
            let mut last_schedule = Instant::now();
            
            while *running.lock().unwrap() {
                let now = Instant::now();
                
                for task in &tasks {
                    let mut last_run = task.last_run.lock().unwrap();
                    
                    if now.duration_since(*last_run).as_millis() >= task.period_ms as u128 {
                        let start_time = Instant::now();
                        
                        // 执行任务
                        (task.task_fn)();
                        
                        let execution_time = start_time.elapsed();
                        
                        // 更新统计信息
                        *last_run = now;
                        *task.run_count.lock().unwrap() += 1;
                        
                        let mut max_time = task.max_execution_time.lock().unwrap();
                        if execution_time > *max_time {
                            *max_time = execution_time;
                        }
                    }
                }
                
                thread::sleep(Duration::from_millis(1)); // 调度器时间片
            }
        }));
    }
    
    pub fn stop(&mut self) {
        *self.running.lock().unwrap() = false;
        
        if let Some(handle) = self.scheduler_thread.take() {
            handle.join().unwrap();
        }
    }
    
    pub fn get_task_statistics(&self, task_id: u32) -> Option<TaskStatistics> {
        self.tasks.iter()
            .find(|task| task.id == task_id)
            .map(|task| TaskStatistics {
                id: task.id,
                run_count: *task.run_count.lock().unwrap(),
                max_execution_time: *task.max_execution_time.lock().unwrap(),
                priority: task.priority,
                period_ms: task.period_ms,
            })
    }
}

#[derive(Debug, Clone)]
pub struct TaskStatistics {
    pub id: u32,
    pub run_count: u32,
    pub max_execution_time: Duration,
    pub priority: u8,
    pub period_ms: u64,
}

// 系统监控器
pub struct SystemMonitor {
    cpu_usage: Arc<Mutex<f32>>,
    memory_usage: Arc<Mutex<f32>>,
    task_scheduler: Arc<Mutex<TaskScheduler>>,
    event_log: Arc<Mutex<Vec<SystemEvent>>>,
}

#[derive(Debug, Clone)]
pub struct SystemEvent {
    pub timestamp: Instant,
    pub event_type: EventType,
    pub description: String,
}

#[derive(Debug, Clone)]
pub enum EventType {
    TaskStarted,
    TaskCompleted,
    TaskOverrun,
    SystemError,
    ResourceLimitReached,
}

impl SystemMonitor {
    pub fn new(scheduler: TaskScheduler) -> Self {
        Self {
            cpu_usage: Arc::new(Mutex::new(0.0)),
            memory_usage: Arc::new(Mutex::new(0.0)),
            task_scheduler: Arc::new(Mutex::new(scheduler)),
            event_log: Arc::new(Mutex::new(Vec::new())),
        }
    }
    
    pub fn log_event(&self, event_type: EventType, description: String) {
        let event = SystemEvent {
            timestamp: Instant::now(),
            event_type,
            description,
        };
        
        self.event_log.lock().unwrap().push(event);
    }
    
    pub fn update_cpu_usage(&self, usage: f32) {
        *self.cpu_usage.lock().unwrap() = usage;
        
        if usage > 90.0 {
            self.log_event(
                EventType::ResourceLimitReached,
                format!("High CPU usage: {:.1}%", usage)
            );
        }
    }
    
    pub fn update_memory_usage(&self, usage: f32) {
        *self.memory_usage.lock().unwrap() = usage;
        
        if usage > 85.0 {
            self.log_event(
                EventType::ResourceLimitReached,
                format!("High memory usage: {:.1}%", usage)
            );
        }
    }
    
    pub fn get_system_status(&self) -> SystemStatus {
        SystemStatus {
            cpu_usage: *self.cpu_usage.lock().unwrap(),
            memory_usage: *self.memory_usage.lock().unwrap(),
            event_count: self.event_log.lock().unwrap().len(),
            uptime: Instant::now(), // 简化实现
        }
    }
    
    pub fn get_events_since(&self, since: Instant) -> Vec<SystemEvent> {
        self.event_log.lock().unwrap()
            .iter()
            .filter(|event| event.timestamp >= since)
            .cloned()
            .collect()
    }
}

#[derive(Debug)]
pub struct SystemStatus {
    pub cpu_usage: f32,
    pub memory_usage: f32,
    pub event_count: usize,
    pub uptime: Instant,
}

#[cfg(test)]
mod system_integration_tests {
    use super::*;
    use integration_test_framework::*;
    
    #[test]
    fn test_task_scheduler_basic_operation() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("task_scheduler_basic", || {
            let mut scheduler = TaskScheduler::new();
            
            // 创建测试任务
            let counter1 = Arc::new(Mutex::new(0));
            let counter2 = Arc::new(Mutex::new(0));
            
            let counter1_clone = Arc::clone(&counter1);
            let task1 = Task {
                id: 1,
                priority: 10,
                period_ms: 100,
                last_run: Arc::new(Mutex::new(Instant::now())),
                run_count: Arc::new(Mutex::new(0)),
                max_execution_time: Arc::new(Mutex::new(Duration::from_nanos(0))),
                task_fn: Arc::new(move || {
                    *counter1_clone.lock().unwrap() += 1;
                }),
            };
            
            let counter2_clone = Arc::clone(&counter2);
            let task2 = Task {
                id: 2,
                priority: 5,
                period_ms: 200,
                last_run: Arc::new(Mutex::new(Instant::now())),
                run_count: Arc::new(Mutex::new(0)),
                max_execution_time: Arc::new(Mutex::new(Duration::from_nanos(0))),
                task_fn: Arc::new(move || {
                    *counter2_clone.lock().unwrap() += 1;
                }),
            };
            
            scheduler.add_task(task1);
            scheduler.add_task(task2);
            
            // 运行调度器
            scheduler.start();
            thread::sleep(Duration::from_millis(550)); // 运行550ms
            scheduler.stop();
            
            // 验证任务执行次数
            let count1 = *counter1.lock().unwrap();
            let count2 = *counter2.lock().unwrap();
            
            // 任务1应该执行约5次（每100ms一次）
            assert!(count1 >= 4 && count1 <= 6);
            // 任务2应该执行约2-3次（每200ms一次）
            assert!(count2 >= 2 && count2 <= 3);
            
            // 获取任务统计信息
            let stats1 = scheduler.get_task_statistics(1).unwrap();
            let stats2 = scheduler.get_task_statistics(2).unwrap();
            
            let mut metrics = HashMap::new();
            metrics.insert("task1_runs".to_string(), stats1.run_count as f64);
            metrics.insert("task2_runs".to_string(), stats2.run_count as f64);
            metrics.insert("task1_max_time_us".to_string(), stats1.max_execution_time.as_micros() as f64);
            metrics.insert("task2_max_time_us".to_string(), stats2.max_execution_time.as_micros() as f64);
            
            Ok(metrics)
        });
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_system_monitor_integration() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("system_monitor_integration", || {
            let scheduler = TaskScheduler::new();
            let monitor = SystemMonitor::new(scheduler);
            
            // 模拟系统负载
            monitor.update_cpu_usage(45.5);
            monitor.update_memory_usage(67.2);
            
            let status = monitor.get_system_status();
            assert_eq!(status.cpu_usage, 45.5);
            assert_eq!(status.memory_usage, 67.2);
            
            // 测试高负载警告
            monitor.update_cpu_usage(95.0);
            monitor.update_memory_usage(90.0);
            
            let events = monitor.get_events_since(Instant::now() - Duration::from_secs(1));
            assert!(events.len() >= 2); // 应该有CPU和内存警告事件
            
            let mut metrics = HashMap::new();
            metrics.insert("cpu_usage".to_string(), status.cpu_usage as f64);
            metrics.insert("memory_usage".to_string(), status.memory_usage as f64);
            metrics.insert("event_count".to_string(), events.len() as f64);
            
            Ok(metrics)
        });
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_real_time_constraints() {
        let executor = TestExecutor::new(TestEnvironment {
            real_time_constraints: true,
            ..Default::default()
        });
        
        let result = executor.run_test("real_time_constraints", || {
            let mut scheduler = TaskScheduler::new();
            
            // 创建高优先级实时任务
            let execution_times = Arc::new(Mutex::new(Vec::new()));
            let execution_times_clone = Arc::clone(&execution_times);
            
            let rt_task = Task {
                id: 100,
                priority: 255, // 最高优先级
                period_ms: 10,  // 10ms周期
                last_run: Arc::new(Mutex::new(Instant::now())),
                run_count: Arc::new(Mutex::new(0)),
                max_execution_time: Arc::new(Mutex::new(Duration::from_nanos(0))),
                task_fn: Arc::new(move || {
                    let start = Instant::now();
                    // 模拟1ms的工作负载
                    thread::sleep(Duration::from_millis(1));
                    let duration = start.elapsed();
                    execution_times_clone.lock().unwrap().push(duration);
                }),
            };
            
            scheduler.add_task(rt_task);
            scheduler.start();
            thread::sleep(Duration::from_millis(100)); // 运行100ms
            scheduler.stop();
            
            // 分析实时性能
            let times = execution_times.lock().unwrap();
            let max_jitter = times.iter()
                .map(|t| t.as_micros())
                .max().unwrap_or(0) as f64;
            
            let avg_execution_time: f64 = times.iter()
                .map(|t| t.as_micros() as f64)
                .sum::<f64>() / times.len() as f64;
            
            // 验证实时约束
            assert!(max_jitter < 5000.0); // 最大抖动应小于5ms
            assert!(avg_execution_time < 2000.0); // 平均执行时间应小于2ms
            
            let mut metrics = HashMap::new();
            metrics.insert("max_jitter_us".to_string(), max_jitter);
            metrics.insert("avg_execution_time_us".to_string(), avg_execution_time);
            metrics.insert("task_runs".to_string(), times.len() as f64);
            
            Ok(metrics)
        });
        
        assert!(result.passed);
    }
}
```

## 端到端集成测试

### 完整系统测试

```rust
// 完整的嵌入式系统模拟
pub struct EmbeddedSystem {
    sensors: Vec<Box<dyn SensorInterface>>,
    actuators: Vec<Box<dyn ActuatorInterface>>,
    communication: Box<dyn CommunicationInterface>,
    data_logger: DataLogger,
    control_loop: ControlLoop,
    system_state: SystemState,
}

pub trait SensorInterface {
    fn read(&mut self) -> Result<SensorData, SensorError>;
    fn get_id(&self) -> u32;
    fn get_type(&self) -> SensorType;
}

pub trait ActuatorInterface {
    fn set_output(&mut self, value: f32) -> Result<(), ActuatorError>;
    fn get_id(&self) -> u32;
    fn get_type(&self) -> ActuatorType;
}

pub trait CommunicationInterface {
    fn send_data(&mut self, data: &[u8]) -> Result<(), CommunicationError>;
    fn receive_data(&mut self) -> Result<Vec<u8>, CommunicationError>;
    fn is_connected(&self) -> bool;
}

#[derive(Debug, Clone)]
pub struct SensorData {
    pub sensor_id: u32,
    pub timestamp: u64,
    pub value: f32,
    pub unit: String,
}

#[derive(Debug, Clone, Copy)]
pub enum SensorType {
    Temperature,
    Pressure,
    Humidity,
    Voltage,
    Current,
}

#[derive(Debug, Clone, Copy)]
pub enum ActuatorType {
    Motor,
    Valve,
    Heater,
    Led,
    Relay,
}

#[derive(Debug)]
pub enum SensorError {
    ReadFailure,
    CalibrationError,
    OutOfRange,
}

#[derive(Debug)]
pub enum ActuatorError {
    SetFailure,
    OutOfRange,
    SafetyLimitExceeded,
}

#[derive(Debug)]
pub enum CommunicationError {
    ConnectionLost,
    TransmissionError,
    ProtocolError,
}

// 数据记录器
pub struct DataLogger {
    log_entries: Vec<LogEntry>,
    max_entries: usize,
}

#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: u64,
    pub level: LogLevel,
    pub message: String,
    pub data: Option<SensorData>,
}

#[derive(Debug, Clone)]
pub enum LogLevel {
    Debug,
    Info,
    Warning,
    Error,
    Critical,
}

impl DataLogger {
    pub fn new(max_entries: usize) -> Self {
        Self {
            log_entries: Vec::new(),
            max_entries,
        }
    }
    
    pub fn log(&mut self, level: LogLevel, message: String, data: Option<SensorData>) {
        let entry = LogEntry {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            level,
            message,
            data,
        };
        
        self.log_entries.push(entry);
        
        // 保持日志条目数量限制
        if self.log_entries.len() > self.max_entries {
            self.log_entries.remove(0);
        }
    }
    
    pub fn get_logs_by_level(&self, level: LogLevel) -> Vec<&LogEntry> {
        self.log_entries.iter()
            .filter(|entry| std::mem::discriminant(&entry.level) == std::mem::discriminant(&level))
            .collect()
    }
    
    pub fn get_recent_logs(&self, count: usize) -> Vec<&LogEntry> {
        let start = self.log_entries.len().saturating_sub(count);
        self.log_entries[start..].iter().collect()
    }
}

// 控制回路
pub struct ControlLoop {
    setpoint: f32,
    kp: f32,
    ki: f32,
    kd: f32,
    integral: f32,
    previous_error: f32,
    output_limits: (f32, f32),
}

impl ControlLoop {
    pub fn new(kp: f32, ki: f32, kd: f32, output_limits: (f32, f32)) -> Self {
        Self {
            setpoint: 0.0,
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
            output_limits,
        }
    }
    
    pub fn set_setpoint(&mut self, setpoint: f32) {
        self.setpoint = setpoint;
    }
    
    pub fn update(&mut self, process_variable: f32, dt: f32) -> f32 {
        let error = self.setpoint - process_variable;
        
        // 积分项
        self.integral += error * dt;
        
        // 微分项
        let derivative = (error - self.previous_error) / dt;
        
        // PID输出
        let output = self.kp * error + self.ki * self.integral + self.kd * derivative;
        
        // 限制输出范围
        let limited_output = output.max(self.output_limits.0).min(self.output_limits.1);
        
        // 积分饱和处理
        if limited_output != output {
            self.integral -= error * dt; // 回退积分项
        }
        
        self.previous_error = error;
        limited_output
    }
    
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
    }
}

#[derive(Debug, Clone)]
pub enum SystemState {
    Initializing,
    Running,
    Error,
    Maintenance,
    Shutdown,
}

impl EmbeddedSystem {
    pub fn new() -> Self {
        Self {
            sensors: Vec::new(),
            actuators: Vec::new(),
            communication: Box::new(MockCommunication::new()),
            data_logger: DataLogger::new(1000),
            control_loop: ControlLoop::new(1.0, 0.1, 0.01, (-100.0, 100.0)),
            system_state: SystemState::Initializing,
        }
    }
    
    pub fn add_sensor(&mut self, sensor: Box<dyn SensorInterface>) {
        self.sensors.push(sensor);
    }
    
    pub fn add_actuator(&mut self, actuator: Box<dyn ActuatorInterface>) {
        self.actuators.push(actuator);
    }
    
    pub fn initialize(&mut self) -> Result<(), SystemError> {
        self.data_logger.log(
            LogLevel::Info,
            "System initialization started".to_string(),
            None,
        );
        
        // 初始化传感器
        for sensor in &mut self.sensors {
            match sensor.read() {
                Ok(_) => {
                    self.data_logger.log(
                        LogLevel::Info,
                        format!("Sensor {} initialized successfully", sensor.get_id()),
                        None,
                    );
                },
                Err(e) => {
                    self.data_logger.log(
                        LogLevel::Error,
                        format!("Sensor {} initialization failed: {:?}", sensor.get_id(), e),
                        None,
                    );
                    return Err(SystemError::InitializationFailed);
                }
            }
        }
        
        // 初始化执行器
        for actuator in &mut self.actuators {
            match actuator.set_output(0.0) {
                Ok(_) => {
                    self.data_logger.log(
                        LogLevel::Info,
                        format!("Actuator {} initialized successfully", actuator.get_id()),
                        None,
                    );
                },
                Err(e) => {
                    self.data_logger.log(
                        LogLevel::Error,
                        format!("Actuator {} initialization failed: {:?}", actuator.get_id(), e),
                        None,
                    );
                    return Err(SystemError::InitializationFailed);
                }
            }
        }
        
        self.system_state = SystemState::Running;
        self.data_logger.log(
            LogLevel::Info,
            "System initialization completed".to_string(),
            None,
        );
        
        Ok(())
    }
    
    pub fn run_cycle(&mut self) -> Result<SystemMetrics, SystemError> {
        if !matches!(self.system_state, SystemState::Running) {
            return Err(SystemError::InvalidState);
        }
        
        let mut metrics = SystemMetrics::new();
        
        // 读取传感器数据
        for sensor in &mut self.sensors {
            match sensor.read() {
                Ok(data) => {
                    metrics.sensor_readings.push(data.clone());
                    
                    // 如果是温度传感器，用于控制回路
                    if matches!(sensor.get_type(), SensorType::Temperature) {
                        let control_output = self.control_loop.update(data.value, 0.1);
                        
                        // 将控制输出应用到执行器
                        for actuator in &mut self.actuators {
                            if matches!(actuator.get_type(), ActuatorType::Heater) {
                                actuator.set_output(control_output).ok();
                                metrics.actuator_outputs.push((actuator.get_id(), control_output));
                            }
                        }
                    }
                    
                    self.data_logger.log(
                        LogLevel::Debug,
                        format!("Sensor {} reading: {}", data.sensor_id, data.value),
                        Some(data),
                    );
                },
                Err(e) => {
                    self.data_logger.log(
                        LogLevel::Warning,
                        format!("Sensor {} read error: {:?}", sensor.get_id(), e),
                        None,
                    );
                    metrics.error_count += 1;
                }
            }
        }
        
        // 发送数据到外部系统
        if self.communication.is_connected() {
            let data_packet = self.create_data_packet(&metrics);
            if let Err(e) = self.communication.send_data(&data_packet) {
                self.data_logger.log(
                    LogLevel::Warning,
                    format!("Communication error: {:?}", e),
                    None,
                );
                metrics.communication_errors += 1;
            }
        }
        
        Ok(metrics)
    }
    
    fn create_data_packet(&self, metrics: &SystemMetrics) -> Vec<u8> {
        // 简化的数据包格式
        let mut packet = Vec::new();
        packet.push(0xAA); // 包头
        packet.push(metrics.sensor_readings.len() as u8);
        
        for reading in &metrics.sensor_readings {
            packet.extend_from_slice(&reading.sensor_id.to_le_bytes());
            packet.extend_from_slice(&reading.value.to_le_bytes());
        }
        
        packet.push(0x55); // 包尾
        packet
    }
    
    pub fn get_system_state(&self) -> &SystemState {
        &self.system_state
    }
    
    pub fn get_logs(&self, count: usize) -> Vec<&LogEntry> {
        self.data_logger.get_recent_logs(count)
    }
}

#[derive(Debug)]
pub enum SystemError {
    InitializationFailed,
    InvalidState,
    CriticalError,
}

#[derive(Debug)]
pub struct SystemMetrics {
    pub sensor_readings: Vec<SensorData>,
    pub actuator_outputs: Vec<(u32, f32)>,
    pub error_count: u32,
    pub communication_errors: u32,
    pub cycle_time_ms: f32,
}

impl SystemMetrics {
    pub fn new() -> Self {
        Self {
            sensor_readings: Vec::new(),
            actuator_outputs: Vec::new(),
            error_count: 0,
            communication_errors: 0,
            cycle_time_ms: 0.0,
        }
    }
}

// 模拟实现
pub struct MockTemperatureSensor {
    id: u32,
    base_temperature: f32,
    noise_amplitude: f32,
    reading_count: u32,
}

impl MockTemperatureSensor {
    pub fn new(id: u32, base_temperature: f32) -> Self {
        Self {
            id,
            base_temperature,
            noise_amplitude: 1.0,
            reading_count: 0,
        }
    }
}

impl SensorInterface for MockTemperatureSensor {
    fn read(&mut self) -> Result<SensorData, SensorError> {
        self.reading_count += 1;
        
        // 模拟传感器噪声
        let noise = (self.reading_count as f32 * 0.1).sin() * self.noise_amplitude;
        let temperature = self.base_temperature + noise;
        
        Ok(SensorData {
            sensor_id: self.id,
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            value: temperature,
            unit: "°C".to_string(),
        })
    }
    
    fn get_id(&self) -> u32 {
        self.id
    }
    
    fn get_type(&self) -> SensorType {
        SensorType::Temperature
    }
}

pub struct MockHeater {
    id: u32,
    current_output: f32,
    max_output: f32,
}

impl MockHeater {
    pub fn new(id: u32, max_output: f32) -> Self {
        Self {
            id,
            current_output: 0.0,
            max_output,
        }
    }
}

impl ActuatorInterface for MockHeater {
    fn set_output(&mut self, value: f32) -> Result<(), ActuatorError> {
        if value < 0.0 || value > self.max_output {
            return Err(ActuatorError::OutOfRange);
        }
        
        self.current_output = value;
        Ok(())
    }
    
    fn get_id(&self) -> u32 {
        self.id
    }
    
    fn get_type(&self) -> ActuatorType {
        ActuatorType::Heater
    }
}

pub struct MockCommunication {
    connected: bool,
    tx_buffer: Vec<u8>,
    rx_buffer: Vec<u8>,
}

impl MockCommunication {
    pub fn new() -> Self {
        Self {
            connected: true,
            tx_buffer: Vec::new(),
            rx_buffer: Vec::new(),
        }
    }
    
    pub fn set_connected(&mut self, connected: bool) {
        self.connected = connected;
    }
    
    pub fn get_transmitted_data(&self) -> &[u8] {
        &self.tx_buffer
    }
}

impl CommunicationInterface for MockCommunication {
    fn send_data(&mut self, data: &[u8]) -> Result<(), CommunicationError> {
        if !self.connected {
            return Err(CommunicationError::ConnectionLost);
        }
        
        self.tx_buffer.extend_from_slice(data);
        Ok(())
    }
    
    fn receive_data(&mut self) -> Result<Vec<u8>, CommunicationError> {
        if !self.connected {
            return Err(CommunicationError::ConnectionLost);
        }
        
        let data = self.rx_buffer.clone();
        self.rx_buffer.clear();
        Ok(data)
    }
    
    fn is_connected(&self) -> bool {
        self.connected
    }
}

#[cfg(test)]
mod end_to_end_tests {
    use super::*;
    use integration_test_framework::*;
    
    #[test]
    fn test_complete_system_operation() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("complete_system_operation", || {
            let mut system = EmbeddedSystem::new();
            
            // 添加传感器和执行器
            system.add_sensor(Box::new(MockTemperatureSensor::new(1, 25.0)));
            system.add_actuator(Box::new(MockHeater::new(1, 100.0)));
            
            // 初始化系统
            system.initialize().unwrap();
            assert!(matches!(system.get_system_state(), SystemState::Running));
            
            // 设置控制目标
            system.control_loop.set_setpoint(30.0);
            
            // 运行多个控制周期
            let mut total_errors = 0;
            let mut cycle_count = 0;
            
            for _ in 0..10 {
                match system.run_cycle() {
                    Ok(metrics) => {
                        total_errors += metrics.error_count;
                        cycle_count += 1;
                        
                        // 验证传感器读数
                        assert!(!metrics.sensor_readings.is_empty());
                        
                        // 验证执行器输出
                        if !metrics.actuator_outputs.is_empty() {
                            let (actuator_id, output) = metrics.actuator_outputs[0];
                            assert_eq!(actuator_id, 1);
                            assert!(output >= -100.0 && output <= 100.0);
                        }
                    },
                    Err(e) => {
                        panic!("System cycle failed: {:?}", e);
                    }
                }
            }
            
            // 验证日志记录
            let logs = system.get_logs(20);
            assert!(logs.len() > 0);
            
            // 验证通信
            let comm = system.communication.as_any().downcast_ref::<MockCommunication>().unwrap();
            assert!(!comm.get_transmitted_data().is_empty());
            
            let mut metrics = HashMap::new();
            metrics.insert("total_cycles".to_string(), cycle_count as f64);
            metrics.insert("total_errors".to_string(), total_errors as f64);
            metrics.insert("log_entries".to_string(), logs.len() as f64);
            metrics.insert("transmitted_bytes".to_string(), comm.get_transmitted_data().len() as f64);
            
            Ok(metrics)
        });
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_system_fault_tolerance() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("system_fault_tolerance", || {
            let mut system = EmbeddedSystem::new();
            
            // 添加多个传感器以测试冗余
            system.add_sensor(Box::new(MockTemperatureSensor::new(1, 25.0)));
            system.add_sensor(Box::new(MockTemperatureSensor::new(2, 25.5)));
            system.add_actuator(Box::new(MockHeater::new(1, 100.0)));
            
            system.initialize().unwrap();
            
            // 模拟通信故障
            let comm = system.communication.as_any_mut().downcast_mut::<MockCommunication>().unwrap();
            comm.set_connected(false);
            
            // 系统应该继续运行，即使通信失败
            let metrics = system.run_cycle().unwrap();
            assert!(metrics.communication_errors > 0);
            
            // 恢复通信
            comm.set_connected(true);
            let metrics = system.run_cycle().unwrap();
            assert_eq!(metrics.communication_errors, 0);
            
            let mut test_metrics = HashMap::new();
            test_metrics.insert("fault_recovery_success".to_string(), 1.0);
            
            Ok(test_metrics)
        });
        
        assert!(result.passed);
    }
}

// 为了支持downcast，需要添加这个trait
trait AsAny {
    fn as_any(&self) -> &dyn std::any::Any;
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any;
}

impl AsAny for MockCommunication {
    fn as_any(&self) -> &dyn std::any::Any {
        self
    }
    
    fn as_any_mut(&mut self) -> &mut dyn std::any::Any {
        self
    }
}

impl CommunicationInterface for Box<dyn CommunicationInterface> {
    fn send_data(&mut self, data: &[u8]) -> Result<(), CommunicationError> {
        (**self).send_data(data)
    }
    
    fn receive_data(&mut self) -> Result<Vec<u8>, CommunicationError> {
        (**self).receive_data()
    }
    
    fn is_connected(&self) -> bool {
        (**self).is_connected()
    }
}
```

## 性能集成测试

### 负载测试

```rust
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

pub struct LoadTestRunner {
    test_duration: Duration,
    concurrent_tasks: usize,
    metrics: Arc<LoadTestMetrics>,
}

pub struct LoadTestMetrics {
    pub total_operations: AtomicU64,
    pub successful_operations: AtomicU64,
    pub failed_operations: AtomicU64,
    pub min_response_time: AtomicU64,
    pub max_response_time: AtomicU64,
    pub total_response_time: AtomicU64,
}

impl LoadTestMetrics {
    pub fn new() -> Self {
        Self {
            total_operations: AtomicU64::new(0),
            successful_operations: AtomicU64::new(0),
            failed_operations: AtomicU64::new(0),
            min_response_time: AtomicU64::new(u64::MAX),
            max_response_time: AtomicU64::new(0),
            total_response_time: AtomicU64::new(0),
        }
    }
    
    pub fn record_operation(&self, success: bool, response_time: Duration) {
        self.total_operations.fetch_add(1, Ordering::Relaxed);
        
        if success {
            self.successful_operations.fetch_add(1, Ordering::Relaxed);
        } else {
            self.failed_operations.fetch_add(1, Ordering::Relaxed);
        }
        
        let response_time_us = response_time.as_micros() as u64;
        self.total_response_time.fetch_add(response_time_us, Ordering::Relaxed);
        
        // 更新最小响应时间
        let mut current_min = self.min_response_time.load(Ordering::Relaxed);
        while response_time_us < current_min {
            match self.min_response_time.compare_exchange_weak(
                current_min,
                response_time_us,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(x) => current_min = x,
            }
        }
        
        // 更新最大响应时间
        let mut current_max = self.max_response_time.load(Ordering::Relaxed);
        while response_time_us > current_max {
            match self.max_response_time.compare_exchange_weak(
                current_max,
                response_time_us,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => break,
                Err(x) => current_max = x,
            }
        }
    }
    
    pub fn get_summary(&self) -> LoadTestSummary {
        let total = self.total_operations.load(Ordering::Relaxed);
        let successful = self.successful_operations.load(Ordering::Relaxed);
        let failed = self.failed_operations.load(Ordering::Relaxed);
        let total_time = self.total_response_time.load(Ordering::Relaxed);
        
        LoadTestSummary {
            total_operations: total,
            successful_operations: successful,
            failed_operations: failed,
            success_rate: if total > 0 { (successful as f64 / total as f64) * 100.0 } else { 0.0 },
            average_response_time_us: if total > 0 { total_time / total } else { 0 },
            min_response_time_us: self.min_response_time.load(Ordering::Relaxed),
            max_response_time_us: self.max_response_time.load(Ordering::Relaxed),
        }
    }
}

#[derive(Debug)]
pub struct LoadTestSummary {
    pub total_operations: u64,
    pub successful_operations: u64,
    pub failed_operations: u64,
    pub success_rate: f64,
    pub average_response_time_us: u64,
    pub min_response_time_us: u64,
    pub max_response_time_us: u64,
}

impl LoadTestRunner {
    pub fn new(test_duration: Duration, concurrent_tasks: usize) -> Self {
        Self {
            test_duration,
            concurrent_tasks,
            metrics: Arc::new(LoadTestMetrics::new()),
        }
    }
    
    pub fn run_load_test<F>(&self, test_operation: F) -> LoadTestSummary
    where
        F: Fn() -> Result<(), Box<dyn std::error::Error + Send + Sync>> + Send + Sync + 'static,
    {
        let test_operation = Arc::new(test_operation);
        let start_time = Instant::now();
        let mut handles = Vec::new();
        
        for _ in 0..self.concurrent_tasks {
            let metrics = Arc::clone(&self.metrics);
            let operation = Arc::clone(&test_operation);
            let duration = self.test_duration;
            
            let handle = thread::spawn(move || {
                let thread_start = Instant::now();
                
                while thread_start.elapsed() < duration {
                    let op_start = Instant::now();
                    let success = operation().is_ok();
                    let op_duration = op_start.elapsed();
                    
                    metrics.record_operation(success, op_duration);
                    
                    // 短暂休眠以避免过度占用CPU
                    thread::sleep(Duration::from_micros(100));
                }
            });
            
            handles.push(handle);
        }
        
        // 等待所有线程完成
        for handle in handles {
            handle.join().unwrap();
        }
        
        self.metrics.get_summary()
    }
}

#[cfg(test)]
mod load_tests {
    use super::*;
    use integration_test_framework::*;
    
    #[test]
    fn test_sensor_reading_load() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("sensor_reading_load", || {
            let runner = LoadTestRunner::new(Duration::from_secs(5), 4);
            
            let summary = runner.run_load_test(|| {
                let mut sensor = MockTemperatureSensor::new(1, 25.0);
                sensor.read().map_err(|e| Box::new(e) as Box<dyn std::error::Error + Send + Sync>)?;
                Ok(())
            });
            
            // 验证负载测试结果
            assert!(summary.total_operations > 1000); // 应该执行大量操作
            assert!(summary.success_rate > 95.0); // 成功率应该很高
            assert!(summary.average_response_time_us < 1000); // 平均响应时间应该很快
            
            let mut metrics = HashMap::new();
            metrics.insert("total_operations".to_string(), summary.total_operations as f64);
            metrics.insert("success_rate".to_string(), summary.success_rate);
            metrics.insert("avg_response_time_us".to_string(), summary.average_response_time_us as f64);
            metrics.insert("max_response_time_us".to_string(), summary.max_response_time_us as f64);
            
            Ok(metrics)
        });
        
        assert!(result.passed);
    }
    
    #[test]
    fn test_communication_throughput() {
        let executor = TestExecutor::new(TestEnvironment::default());
        
        let result = executor.run_test("communication_throughput", || {
            let runner = LoadTestRunner::new(Duration::from_secs(3), 2);
            let comm = Arc::new(Mutex::new(MockCommunication::new()));
            
            let summary = runner.run_load_test(|| {
                let data = vec![0x01, 0x02, 0x03, 0x04]; // 4字节数据包
                let mut comm_guard = comm.lock().unwrap();
                comm_guard.send_data(&data)
                    .map_err(|e| Box::new(e) as Box<dyn std::error::Error + Send + Sync>)?;
                Ok(())
            });
            
            // 计算吞吐量
            let bytes_per_operation = 4;
            let total_bytes = summary.successful_operations * bytes_per_operation;
            let throughput_bps = (total_bytes as f64 / 3.0) as u64; // 字节/秒
            
            let mut metrics = HashMap::new();
            metrics.insert("throughput_bps".to_string(), throughput_bps as f64);
            metrics.insert("total_bytes".to_string(), total_bytes as f64);
            
            Ok(metrics)
        });
        
        assert!(result.passed);
    }
}
```

## 测试报告和分析

### 测试结果分析

```rust
use std::collections::HashMap;
use std::fs::File;
use std::io::Write;

pub struct TestReportGenerator {
    test_results: Vec<TestResult>,
    system_metrics: HashMap<String, f64>,
}

impl TestReportGenerator {
    pub fn new() -> Self {
        Self {
            test_results: Vec::new(),
            system_metrics: HashMap::new(),
        }
    }
    
    pub fn add_test_result(&mut self, result: TestResult) {
        self.test_results.push(result);
    }
    
    pub fn add_system_metric(&mut self, name: String, value: f64) {
        self.system_metrics.insert(name, value);
    }
    
    pub fn generate_report(&self) -> TestReport {
        let total_tests = self.test_results.len();
        let passed_tests = self.test_results.iter().filter(|r| r.passed).count();
        let failed_tests = total_tests - passed_tests;
        
        let total_duration: u64 = self.test_results.iter().map(|r| r.duration_ms).sum();
        let average_duration = if total_tests > 0 {
            total_duration as f64 / total_tests as f64
        } else {
            0.0
        };
        
        // 分析失败的测试
        let failed_test_analysis = self.analyze_failures();
        
        // 性能分析
        let performance_analysis = self.analyze_performance();
        
        TestReport {
            summary: TestSummary {
                total_tests,
                passed_tests,
                failed_tests,
                total_duration_ms: total_duration,
                pass_rate: if total_tests > 0 {
                    (passed_tests as f64 / total_tests as f64) * 100.0
                } else {
                    0.0
                },
            },
            test_results: self.test_results.clone(),
            system_metrics: self.system_metrics.clone(),
            failure_analysis: failed_test_analysis,
            performance_analysis,
            recommendations: self.generate_recommendations(),
        }
    }
    
    fn analyze_failures(&self) -> FailureAnalysis {
        let failed_tests: Vec<&TestResult> = self.test_results
            .iter()
            .filter(|r| !r.passed)
            .collect();
        
        let mut error_categories = HashMap::new();
        let mut common_patterns = Vec::new();
        
        for test in &failed_tests {
            if let Some(error) = &test.error_message {
                // 简单的错误分类
                let category = if error.contains("timeout") {
                    "Timeout"
                } else if error.contains("communication") {
                    "Communication"
                } else if error.contains("sensor") {
                    "Sensor"
                } else if error.contains("memory") {
                    "Memory"
                } else {
                    "Other"
                };
                
                *error_categories.entry(category.to_string()).or_insert(0) += 1;
            }
        }
        
        // 查找常见模式
        if error_categories.get("Timeout").unwrap_or(&0) > &2 {
            common_patterns.push("Multiple timeout errors detected - consider increasing timeout values or optimizing performance".to_string());
        }
        
        if error_categories.get("Communication").unwrap_or(&0) > &1 {
            common_patterns.push("Communication errors detected - check network stability and protocol implementation".to_string());
        }
        
        FailureAnalysis {
            total_failures: failed_tests.len(),
            error_categories,
            common_patterns,
            critical_failures: failed_tests.iter()
                .filter(|t| t.error_message.as_ref().map_or(false, |e| e.contains("critical")))
                .map(|t| t.clone())
                .collect(),
        }
    }
    
    fn analyze_performance(&self) -> PerformanceAnalysis {
        let durations: Vec<u64> = self.test_results.iter().map(|r| r.duration_ms).collect();
        
        let min_duration = durations.iter().min().copied().unwrap_or(0);
        let max_duration = durations.iter().max().copied().unwrap_or(0);
        let avg_duration = if !durations.is_empty() {
            durations.iter().sum::<u64>() as f64 / durations.len() as f64
        } else {
            0.0
        };
        
        // 计算95百分位数
        let mut sorted_durations = durations.clone();
        sorted_durations.sort();
        let p95_index = (sorted_durations.len() as f64 * 0.95) as usize;
        let p95_duration = sorted_durations.get(p95_index).copied().unwrap_or(0);
        
        // 识别性能异常值
        let outliers: Vec<String> = self.test_results.iter()
            .filter(|r| r.duration_ms > (avg_duration * 2.0) as u64)
            .map(|r| format!("Test '{}' took {}ms (avg: {:.1}ms)", 
                           r.error_message.as_ref().unwrap_or(&"unknown".to_string()), 
                           r.duration_ms, 
                           avg_duration))
            .collect();
        
        PerformanceAnalysis {
            min_duration_ms: min_duration,
            max_duration_ms: max_duration,
            avg_duration_ms: avg_duration,
            p95_duration_ms: p95_duration,
            performance_outliers: outliers,
            system_metrics: self.system_metrics.clone(),
        }
    }
    
    fn generate_recommendations(&self) -> Vec<String> {
        let mut recommendations = Vec::new();
        
        let pass_rate = if !self.test_results.is_empty() {
            (self.test_results.iter().filter(|r| r.passed).count() as f64 / 
             self.test_results.len() as f64) * 100.0
        } else {
            0.0
        };
        
        if pass_rate < 95.0 {
            recommendations.push("Test pass rate is below 95%. Review failed tests and improve system reliability.".to_string());
        }
        
        if pass_rate < 80.0 {
            recommendations.push("Critical: Test pass rate is below 80%. Immediate attention required.".to_string());
        }
        
        // 性能建议
        let avg_duration = if !self.test_results.is_empty() {
            self.test_results.iter().map(|r| r.duration_ms).sum::<u64>() as f64 / 
            self.test_results.len() as f64
        } else {
            0.0
        };
        
        if avg_duration > 1000.0 {
            recommendations.push("Average test duration is high. Consider optimizing test setup and execution.".to_string());
        }
        
        // 系统指标建议
        if let Some(&cpu_usage) = self.system_metrics.get("cpu_usage") {
            if cpu_usage > 80.0 {
                recommendations.push("High CPU usage detected during testing. Monitor system resources.".to_string());
            }
        }
        
        if let Some(&memory_usage) = self.system_metrics.get("memory_usage") {
            if memory_usage > 85.0 {
                recommendations.push("High memory usage detected. Check for memory leaks.".to_string());
            }
        }
        
        recommendations
    }
    
    pub fn export_to_json(&self, filename: &str) -> Result<(), std::io::Error> {
        let report = self.generate_report();
        let json = serde_json::to_string_pretty(&report)?;
        
        let mut file = File::create(filename)?;
        file.write_all(json.as_bytes())?;
        
        Ok(())
    }
    
    pub fn export_to_html(&self, filename: &str) -> Result<(), std::io::Error> {
        let report = self.generate_report();
        let html = self.generate_html_report(&report);
        
        let mut file = File::create(filename)?;
        file.write_all(html.as_bytes())?;
        
        Ok(())
    }
    
    fn generate_html_report(&self, report: &TestReport) -> String {
        format!(r#"
<!DOCTYPE html>
<html>
<head>
    <title>Integration Test Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .summary {{ background: #f0f0f0; padding: 15px; border-radius: 5px; }}
        .pass {{ color: green; }}
        .fail {{ color: red; }}
        table {{ border-collapse: collapse; width: 100%; margin: 20px 0; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
        .metric {{ margin: 10px 0; }}
    </style>
</head>
<body>
    <h1>Integration Test Report</h1>
    
    <div class="summary">
        <h2>Test Summary</h2>
        <div class="metric">Total Tests: {}</div>
        <div class="metric">Passed: <span class="pass">{}</span></div>
        <div class="metric">Failed: <span class="fail">{}</span></div>
        <div class="metric">Pass Rate: {:.1}%</div>
        <div class="metric">Total Duration: {}ms</div>
    </div>
    
    <h2>Performance Analysis</h2>
    <div class="metric">Average Duration: {:.1}ms</div>
    <div class="metric">95th Percentile: {}ms</div>
    <div class="metric">Min Duration: {}ms</div>
    <div class="metric">Max Duration: {}ms</div>
    
    <h2>Recommendations</h2>
    <ul>
        {}
    </ul>
    
    <h2>Detailed Results</h2>
    <table>
        <tr>
            <th>Test Name</th>
            <th>Status</th>
            <th>Duration (ms)</th>
            <th>Error Message</th>
        </tr>
        {}
    </table>
</body>
</html>
        "#,
        report.summary.total_tests,
        report.summary.passed_tests,
        report.summary.failed_tests,
        report.summary.pass_rate,
        report.summary.total_duration_ms,
        report.performance_analysis.avg_duration_ms,
        report.performance_analysis.p95_duration_ms,
        report.performance_analysis.min_duration_ms,
        report.performance_analysis.max_duration_ms,
        report.recommendations.iter()
            .map(|r| format!("<li>{}</li>", r))
            .collect::<Vec<_>>()
            .join(""),
        report.test_results.iter()
            .enumerate()
            .map(|(i, r)| format!(
                "<tr><td>Test {}</td><td class=\"{}\">{}</td><td>{}</td><td>{}</td></tr>",
                i + 1,
                if r.passed { "pass" } else { "fail" },
                if r.passed { "PASS" } else { "FAIL" },
                r.duration_ms,
                r.error_message.as_ref().unwrap_or(&"".to_string())
            ))
            .collect::<Vec<_>>()
            .join("")
        )
    }
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct TestReport {
    pub summary: TestSummary,
    pub test_results: Vec<TestResult>,
    pub system_metrics: HashMap<String, f64>,
    pub failure_analysis: FailureAnalysis,
    pub performance_analysis: PerformanceAnalysis,
    pub recommendations: Vec<String>,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct FailureAnalysis {
    pub total_failures: usize,
    pub error_categories: HashMap<String, u32>,
    pub common_patterns: Vec<String>,
    pub critical_failures: Vec<TestResult>,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct PerformanceAnalysis {
    pub min_duration_ms: u64,
    pub max_duration_ms: u64,
    pub avg_duration_ms: f64,
    pub p95_duration_ms: u64,
    pub performance_outliers: Vec<String>,
    pub system_metrics: HashMap<String, f64>,
}
```

## 总结

集成测试是确保嵌入式系统各组件协同工作的关键环节：

1. **分层测试**：从硬件抽象层到系统级的全面测试
2. **模拟环境**：使用模拟器和存根实现可控的测试环境
3. **性能验证**：通过负载测试验证系统性能指标
4. **故障注入**：测试系统的容错能力和恢复机制
5. **自动化报告**：生成详细的测试报告和分析建议

通过系统性的集成测试，可以及早发现组件间的兼容性问题，确保系统的稳定性和可靠性。
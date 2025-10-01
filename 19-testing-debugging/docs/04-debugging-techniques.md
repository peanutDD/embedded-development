# 调试技术

调试是嵌入式开发中的关键技能，本章介绍各种调试技术和工具，帮助开发者快速定位和解决问题。

## 调试器集成

### GDB调试框架

```rust
use std::collections::HashMap;
use std::process::{Command, Stdio};
use std::io::{BufRead, BufReader, Write};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

pub struct GdbDebugger {
    process: Option<std::process::Child>,
    breakpoints: HashMap<String, Breakpoint>,
    variables: HashMap<String, VariableInfo>,
    call_stack: Vec<StackFrame>,
    current_state: DebuggerState,
}

#[derive(Debug, Clone)]
pub struct Breakpoint {
    pub id: u32,
    pub file: String,
    pub line: u32,
    pub condition: Option<String>,
    pub hit_count: u32,
    pub enabled: bool,
}

#[derive(Debug, Clone)]
pub struct VariableInfo {
    pub name: String,
    pub value: String,
    pub type_name: String,
    pub address: Option<u64>,
    pub size: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct StackFrame {
    pub level: u32,
    pub function: String,
    pub file: String,
    pub line: u32,
    pub address: u64,
}

#[derive(Debug, Clone, PartialEq)]
pub enum DebuggerState {
    NotStarted,
    Running,
    Stopped,
    Breakpoint,
    Error,
    Finished,
}

impl GdbDebugger {
    pub fn new() -> Self {
        Self {
            process: None,
            breakpoints: HashMap::new(),
            variables: HashMap::new(),
            call_stack: Vec::new(),
            current_state: DebuggerState::NotStarted,
        }
    }
    
    pub fn start_session(&mut self, target_file: &str) -> Result<(), DebugError> {
        let mut cmd = Command::new("arm-none-eabi-gdb")
            .arg(target_file)
            .arg("--interpreter=mi")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .map_err(|e| DebugError::ProcessStartFailed(e.to_string()))?;
        
        // 发送初始化命令
        if let Some(stdin) = cmd.stdin.as_mut() {
            writeln!(stdin, "target extended-remote localhost:3333")?;
            writeln!(stdin, "monitor reset halt")?;
            writeln!(stdin, "load")?;
        }
        
        self.process = Some(cmd);
        self.current_state = DebuggerState::Running;
        
        Ok(())
    }
    
    pub fn set_breakpoint(&mut self, file: &str, line: u32) -> Result<u32, DebugError> {
        let bp_id = self.breakpoints.len() as u32 + 1;
        
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "break {}:{}", file, line)?;
            }
        }
        
        let breakpoint = Breakpoint {
            id: bp_id,
            file: file.to_string(),
            line,
            condition: None,
            hit_count: 0,
            enabled: true,
        };
        
        self.breakpoints.insert(format!("{}:{}", file, line), breakpoint);
        Ok(bp_id)
    }
    
    pub fn set_conditional_breakpoint(&mut self, file: &str, line: u32, condition: &str) -> Result<u32, DebugError> {
        let bp_id = self.set_breakpoint(file, line)?;
        
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "condition {} {}", bp_id, condition)?;
            }
        }
        
        if let Some(bp) = self.breakpoints.get_mut(&format!("{}:{}", file, line)) {
            bp.condition = Some(condition.to_string());
        }
        
        Ok(bp_id)
    }
    
    pub fn continue_execution(&mut self) -> Result<(), DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "continue")?;
            }
        }
        
        self.current_state = DebuggerState::Running;
        Ok(())
    }
    
    pub fn step_over(&mut self) -> Result<(), DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "next")?;
            }
        }
        
        Ok(())
    }
    
    pub fn step_into(&mut self) -> Result<(), DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "step")?;
            }
        }
        
        Ok(())
    }
    
    pub fn step_out(&mut self) -> Result<(), DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "finish")?;
            }
        }
        
        Ok(())
    }
    
    pub fn examine_variable(&mut self, var_name: &str) -> Result<VariableInfo, DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "print {}", var_name)?;
                writeln!(stdin, "ptype {}", var_name)?;
                writeln!(stdin, "info address {}", var_name)?;
            }
        }
        
        // 模拟解析GDB输出
        let var_info = VariableInfo {
            name: var_name.to_string(),
            value: "0x12345678".to_string(), // 实际需要解析GDB输出
            type_name: "u32".to_string(),
            address: Some(0x20000000),
            size: Some(4),
        };
        
        self.variables.insert(var_name.to_string(), var_info.clone());
        Ok(var_info)
    }
    
    pub fn examine_memory(&mut self, address: u64, size: usize) -> Result<Vec<u8>, DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "x/{}xb 0x{:x}", size, address)?;
            }
        }
        
        // 模拟内存数据
        let memory_data = (0..size).map(|i| (i % 256) as u8).collect();
        Ok(memory_data)
    }
    
    pub fn get_call_stack(&mut self) -> Result<Vec<StackFrame>, DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "backtrace")?;
            }
        }
        
        // 模拟调用栈
        let stack_frames = vec![
            StackFrame {
                level: 0,
                function: "main".to_string(),
                file: "main.rs".to_string(),
                line: 42,
                address: 0x08000100,
            },
            StackFrame {
                level: 1,
                function: "init".to_string(),
                file: "lib.rs".to_string(),
                line: 15,
                address: 0x08000080,
            },
        ];
        
        self.call_stack = stack_frames.clone();
        Ok(stack_frames)
    }
    
    pub fn get_registers(&mut self) -> Result<HashMap<String, u32>, DebugError> {
        if let Some(process) = &mut self.process {
            if let Some(stdin) = process.stdin.as_mut() {
                writeln!(stdin, "info registers")?;
            }
        }
        
        // 模拟寄存器状态
        let mut registers = HashMap::new();
        registers.insert("r0".to_string(), 0x12345678);
        registers.insert("r1".to_string(), 0x87654321);
        registers.insert("sp".to_string(), 0x20008000);
        registers.insert("pc".to_string(), 0x08000100);
        registers.insert("lr".to_string(), 0x08000080);
        
        Ok(registers)
    }
    
    pub fn terminate_session(&mut self) -> Result<(), DebugError> {
        if let Some(mut process) = self.process.take() {
            if let Some(stdin) = process.stdin.as_mut() {
                let _ = writeln!(stdin, "quit");
            }
            
            let _ = process.wait();
        }
        
        self.current_state = DebuggerState::Finished;
        Ok(())
    }
}

#[derive(Debug)]
pub enum DebugError {
    ProcessStartFailed(String),
    CommunicationError(String),
    InvalidCommand(String),
    TargetNotResponding,
    BreakpointError(String),
    IoError(std::io::Error),
}

impl From<std::io::Error> for DebugError {
    fn from(error: std::io::Error) -> Self {
        DebugError::IoError(error)
    }
}
```

### 日志系统

```rust
use std::fs::{File, OpenOptions};
use std::io::{Write, BufWriter};
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};
use std::collections::VecDeque;

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum LogLevel {
    Trace = 0,
    Debug = 1,
    Info = 2,
    Warn = 3,
    Error = 4,
    Fatal = 5,
}

impl LogLevel {
    pub fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Trace => "TRACE",
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO",
            LogLevel::Warn => "WARN",
            LogLevel::Error => "ERROR",
            LogLevel::Fatal => "FATAL",
        }
    }
}

#[derive(Debug, Clone)]
pub struct LogEntry {
    pub timestamp: u64,
    pub level: LogLevel,
    pub module: String,
    pub message: String,
    pub file: Option<String>,
    pub line: Option<u32>,
}

pub struct Logger {
    level: LogLevel,
    outputs: Vec<Box<dyn LogOutput>>,
    buffer: Arc<Mutex<VecDeque<LogEntry>>>,
    max_buffer_size: usize,
}

pub trait LogOutput: Send + Sync {
    fn write_log(&mut self, entry: &LogEntry) -> Result<(), LogError>;
    fn flush(&mut self) -> Result<(), LogError>;
}

pub struct FileOutput {
    writer: BufWriter<File>,
    format: LogFormat,
}

pub struct ConsoleOutput {
    format: LogFormat,
    colored: bool,
}

pub struct MemoryOutput {
    buffer: VecDeque<LogEntry>,
    max_entries: usize,
}

#[derive(Debug, Clone)]
pub enum LogFormat {
    Simple,
    Detailed,
    Json,
    Custom(String),
}

impl Logger {
    pub fn new(level: LogLevel) -> Self {
        Self {
            level,
            outputs: Vec::new(),
            buffer: Arc::new(Mutex::new(VecDeque::new())),
            max_buffer_size: 1000,
        }
    }
    
    pub fn add_file_output(&mut self, filename: &str, format: LogFormat) -> Result<(), LogError> {
        let file = OpenOptions::new()
            .create(true)
            .append(true)
            .open(filename)
            .map_err(|e| LogError::FileError(e.to_string()))?;
        
        let output = FileOutput {
            writer: BufWriter::new(file),
            format,
        };
        
        self.outputs.push(Box::new(output));
        Ok(())
    }
    
    pub fn add_console_output(&mut self, format: LogFormat, colored: bool) {
        let output = ConsoleOutput { format, colored };
        self.outputs.push(Box::new(output));
    }
    
    pub fn add_memory_output(&mut self, max_entries: usize) {
        let output = MemoryOutput {
            buffer: VecDeque::new(),
            max_entries,
        };
        self.outputs.push(Box::new(output));
    }
    
    pub fn log(&mut self, level: LogLevel, module: &str, message: &str, file: Option<&str>, line: Option<u32>) {
        if level < self.level {
            return;
        }
        
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        let entry = LogEntry {
            timestamp,
            level,
            module: module.to_string(),
            message: message.to_string(),
            file: file.map(|s| s.to_string()),
            line,
        };
        
        // 添加到缓冲区
        if let Ok(mut buffer) = self.buffer.lock() {
            buffer.push_back(entry.clone());
            if buffer.len() > self.max_buffer_size {
                buffer.pop_front();
            }
        }
        
        // 写入所有输出
        for output in &mut self.outputs {
            let _ = output.write_log(&entry);
        }
    }
    
    pub fn trace(&mut self, module: &str, message: &str) {
        self.log(LogLevel::Trace, module, message, None, None);
    }
    
    pub fn debug(&mut self, module: &str, message: &str) {
        self.log(LogLevel::Debug, module, message, None, None);
    }
    
    pub fn info(&mut self, module: &str, message: &str) {
        self.log(LogLevel::Info, module, message, None, None);
    }
    
    pub fn warn(&mut self, module: &str, message: &str) {
        self.log(LogLevel::Warn, module, message, None, None);
    }
    
    pub fn error(&mut self, module: &str, message: &str) {
        self.log(LogLevel::Error, module, message, None, None);
    }
    
    pub fn fatal(&mut self, module: &str, message: &str) {
        self.log(LogLevel::Fatal, module, message, None, None);
    }
    
    pub fn flush(&mut self) -> Result<(), LogError> {
        for output in &mut self.outputs {
            output.flush()?;
        }
        Ok(())
    }
    
    pub fn get_recent_logs(&self, count: usize) -> Vec<LogEntry> {
        if let Ok(buffer) = self.buffer.lock() {
            buffer.iter().rev().take(count).cloned().collect()
        } else {
            Vec::new()
        }
    }
    
    pub fn filter_logs(&self, level: LogLevel, module: Option<&str>) -> Vec<LogEntry> {
        if let Ok(buffer) = self.buffer.lock() {
            buffer.iter()
                .filter(|entry| {
                    entry.level >= level && 
                    module.map_or(true, |m| entry.module.contains(m))
                })
                .cloned()
                .collect()
        } else {
            Vec::new()
        }
    }
}

impl FileOutput {
    fn format_entry(&self, entry: &LogEntry) -> String {
        match &self.format {
            LogFormat::Simple => {
                format!("[{}] {}: {}", entry.level.as_str(), entry.module, entry.message)
            }
            LogFormat::Detailed => {
                let file_info = if let (Some(file), Some(line)) = (&entry.file, entry.line) {
                    format!(" ({}:{})", file, line)
                } else {
                    String::new()
                };
                
                format!("[{}] {} [{}] {}{}", 
                       entry.timestamp, 
                       entry.level.as_str(), 
                       entry.module, 
                       entry.message,
                       file_info)
            }
            LogFormat::Json => {
                format!(r#"{{"timestamp":{},"level":"{}","module":"{}","message":"{}"}}"#,
                       entry.timestamp,
                       entry.level.as_str(),
                       entry.module,
                       entry.message)
            }
            LogFormat::Custom(template) => {
                template
                    .replace("{timestamp}", &entry.timestamp.to_string())
                    .replace("{level}", entry.level.as_str())
                    .replace("{module}", &entry.module)
                    .replace("{message}", &entry.message)
            }
        }
    }
}

impl LogOutput for FileOutput {
    fn write_log(&mut self, entry: &LogEntry) -> Result<(), LogError> {
        let formatted = self.format_entry(entry);
        writeln!(self.writer, "{}", formatted)
            .map_err(|e| LogError::WriteError(e.to_string()))?;
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), LogError> {
        self.writer.flush()
            .map_err(|e| LogError::WriteError(e.to_string()))?;
        Ok(())
    }
}

impl LogOutput for ConsoleOutput {
    fn write_log(&mut self, entry: &LogEntry) -> Result<(), LogError> {
        let formatted = match &self.format {
            LogFormat::Simple => {
                format!("[{}] {}: {}", entry.level.as_str(), entry.module, entry.message)
            }
            LogFormat::Detailed => {
                format!("[{}] {} [{}] {}", 
                       entry.timestamp, 
                       entry.level.as_str(), 
                       entry.module, 
                       entry.message)
            }
            _ => format!("[{}] {}: {}", entry.level.as_str(), entry.module, entry.message),
        };
        
        if self.colored {
            let colored_output = match entry.level {
                LogLevel::Trace => format!("\x1b[37m{}\x1b[0m", formatted), // White
                LogLevel::Debug => format!("\x1b[36m{}\x1b[0m", formatted), // Cyan
                LogLevel::Info => format!("\x1b[32m{}\x1b[0m", formatted),  // Green
                LogLevel::Warn => format!("\x1b[33m{}\x1b[0m", formatted),  // Yellow
                LogLevel::Error => format!("\x1b[31m{}\x1b[0m", formatted), // Red
                LogLevel::Fatal => format!("\x1b[35m{}\x1b[0m", formatted), // Magenta
            };
            println!("{}", colored_output);
        } else {
            println!("{}", formatted);
        }
        
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), LogError> {
        use std::io::{self, Write};
        io::stdout().flush()
            .map_err(|e| LogError::WriteError(e.to_string()))?;
        Ok(())
    }
}

impl LogOutput for MemoryOutput {
    fn write_log(&mut self, entry: &LogEntry) -> Result<(), LogError> {
        self.buffer.push_back(entry.clone());
        if self.buffer.len() > self.max_entries {
            self.buffer.pop_front();
        }
        Ok(())
    }
    
    fn flush(&mut self) -> Result<(), LogError> {
        // Memory output doesn't need flushing
        Ok(())
    }
}

#[derive(Debug)]
pub enum LogError {
    FileError(String),
    WriteError(String),
    FormatError(String),
}

// 宏定义简化日志使用
#[macro_export]
macro_rules! log_trace {
    ($logger:expr, $module:expr, $($arg:tt)*) => {
        $logger.trace($module, &format!($($arg)*))
    };
}

#[macro_export]
macro_rules! log_debug {
    ($logger:expr, $module:expr, $($arg:tt)*) => {
        $logger.debug($module, &format!($($arg)*))
    };
}

#[macro_export]
macro_rules! log_info {
    ($logger:expr, $module:expr, $($arg:tt)*) => {
        $logger.info($module, &format!($($arg)*))
    };
}

#[macro_export]
macro_rules! log_warn {
    ($logger:expr, $module:expr, $($arg:tt)*) => {
        $logger.warn($module, &format!($($arg)*))
    };
}

#[macro_export]
macro_rules! log_error {
    ($logger:expr, $module:expr, $($arg:tt)*) => {
        $logger.error($module, &format!($($arg)*))
    };
}
```

## 性能分析

### 性能监控器

```rust
use std::collections::HashMap;
use std::time::{Duration, Instant};
use std::sync::{Arc, Mutex};

pub struct PerformanceMonitor {
    metrics: Arc<Mutex<HashMap<String, PerformanceMetric>>>,
    active_timers: Arc<Mutex<HashMap<String, Instant>>>,
    sampling_enabled: bool,
    sample_interval: Duration,
}

#[derive(Debug, Clone)]
pub struct PerformanceMetric {
    pub name: String,
    pub total_time: Duration,
    pub call_count: u64,
    pub min_time: Duration,
    pub max_time: Duration,
    pub avg_time: Duration,
    pub last_time: Duration,
}

impl PerformanceMetric {
    pub fn new(name: String) -> Self {
        Self {
            name,
            total_time: Duration::ZERO,
            call_count: 0,
            min_time: Duration::MAX,
            max_time: Duration::ZERO,
            avg_time: Duration::ZERO,
            last_time: Duration::ZERO,
        }
    }
    
    pub fn update(&mut self, duration: Duration) {
        self.total_time += duration;
        self.call_count += 1;
        self.last_time = duration;
        
        if duration < self.min_time {
            self.min_time = duration;
        }
        
        if duration > self.max_time {
            self.max_time = duration;
        }
        
        self.avg_time = self.total_time / self.call_count as u32;
    }
}

impl PerformanceMonitor {
    pub fn new() -> Self {
        Self {
            metrics: Arc::new(Mutex::new(HashMap::new())),
            active_timers: Arc::new(Mutex::new(HashMap::new())),
            sampling_enabled: true,
            sample_interval: Duration::from_millis(100),
        }
    }
    
    pub fn start_timer(&self, name: &str) {
        if !self.sampling_enabled {
            return;
        }
        
        if let Ok(mut timers) = self.active_timers.lock() {
            timers.insert(name.to_string(), Instant::now());
        }
    }
    
    pub fn end_timer(&self, name: &str) {
        if !self.sampling_enabled {
            return;
        }
        
        let end_time = Instant::now();
        
        if let Ok(mut timers) = self.active_timers.lock() {
            if let Some(start_time) = timers.remove(name) {
                let duration = end_time - start_time;
                
                if let Ok(mut metrics) = self.metrics.lock() {
                    let metric = metrics.entry(name.to_string())
                        .or_insert_with(|| PerformanceMetric::new(name.to_string()));
                    metric.update(duration);
                }
            }
        }
    }
    
    pub fn measure<F, R>(&self, name: &str, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        self.start_timer(name);
        let result = f();
        self.end_timer(name);
        result
    }
    
    pub fn get_metric(&self, name: &str) -> Option<PerformanceMetric> {
        if let Ok(metrics) = self.metrics.lock() {
            metrics.get(name).cloned()
        } else {
            None
        }
    }
    
    pub fn get_all_metrics(&self) -> HashMap<String, PerformanceMetric> {
        if let Ok(metrics) = self.metrics.lock() {
            metrics.clone()
        } else {
            HashMap::new()
        }
    }
    
    pub fn reset_metric(&self, name: &str) {
        if let Ok(mut metrics) = self.metrics.lock() {
            metrics.remove(name);
        }
    }
    
    pub fn reset_all_metrics(&self) {
        if let Ok(mut metrics) = self.metrics.lock() {
            metrics.clear();
        }
    }
    
    pub fn enable_sampling(&mut self) {
        self.sampling_enabled = true;
    }
    
    pub fn disable_sampling(&mut self) {
        self.sampling_enabled = false;
    }
    
    pub fn generate_report(&self) -> PerformanceReport {
        let metrics = self.get_all_metrics();
        let total_functions = metrics.len();
        let total_calls: u64 = metrics.values().map(|m| m.call_count).sum();
        let total_time: Duration = metrics.values().map(|m| m.total_time).sum();
        
        // 找出最慢的函数
        let slowest_function = metrics.values()
            .max_by_key(|m| m.max_time)
            .map(|m| m.name.clone());
        
        // 找出调用最频繁的函数
        let most_called_function = metrics.values()
            .max_by_key(|m| m.call_count)
            .map(|m| m.name.clone());
        
        // 找出总时间最长的函数
        let most_time_consuming = metrics.values()
            .max_by_key(|m| m.total_time)
            .map(|m| m.name.clone());
        
        PerformanceReport {
            total_functions,
            total_calls,
            total_time,
            metrics,
            slowest_function,
            most_called_function,
            most_time_consuming,
        }
    }
}

#[derive(Debug)]
pub struct PerformanceReport {
    pub total_functions: usize,
    pub total_calls: u64,
    pub total_time: Duration,
    pub metrics: HashMap<String, PerformanceMetric>,
    pub slowest_function: Option<String>,
    pub most_called_function: Option<String>,
    pub most_time_consuming: Option<String>,
}

impl PerformanceReport {
    pub fn print_summary(&self) {
        println!("=== Performance Report ===");
        println!("Total Functions Monitored: {}", self.total_functions);
        println!("Total Function Calls: {}", self.total_calls);
        println!("Total Execution Time: {:?}", self.total_time);
        
        if let Some(ref func) = self.slowest_function {
            if let Some(metric) = self.metrics.get(func) {
                println!("Slowest Function: {} ({:?})", func, metric.max_time);
            }
        }
        
        if let Some(ref func) = self.most_called_function {
            if let Some(metric) = self.metrics.get(func) {
                println!("Most Called Function: {} ({} calls)", func, metric.call_count);
            }
        }
        
        if let Some(ref func) = self.most_time_consuming {
            if let Some(metric) = self.metrics.get(func) {
                println!("Most Time Consuming: {} ({:?} total)", func, metric.total_time);
            }
        }
        
        println!("\n=== Detailed Metrics ===");
        let mut sorted_metrics: Vec<_> = self.metrics.iter().collect();
        sorted_metrics.sort_by_key(|(_, m)| std::cmp::Reverse(m.total_time));
        
        for (name, metric) in sorted_metrics.iter().take(10) {
            println!("{}: calls={}, total={:?}, avg={:?}, min={:?}, max={:?}",
                    name,
                    metric.call_count,
                    metric.total_time,
                    metric.avg_time,
                    metric.min_time,
                    metric.max_time);
        }
    }
    
    pub fn export_csv(&self, filename: &str) -> Result<(), std::io::Error> {
        use std::fs::File;
        use std::io::Write;
        
        let mut file = File::create(filename)?;
        writeln!(file, "Function,Calls,Total_Time_us,Avg_Time_us,Min_Time_us,Max_Time_us")?;
        
        for (name, metric) in &self.metrics {
            writeln!(file, "{},{},{},{},{},{}",
                    name,
                    metric.call_count,
                    metric.total_time.as_micros(),
                    metric.avg_time.as_micros(),
                    metric.min_time.as_micros(),
                    metric.max_time.as_micros())?;
        }
        
        Ok(())
    }
}

// 性能监控宏
#[macro_export]
macro_rules! perf_measure {
    ($monitor:expr, $name:expr, $block:block) => {
        $monitor.measure($name, || $block)
    };
}

#[macro_export]
macro_rules! perf_timer {
    ($monitor:expr, $name:expr) => {
        let _timer = PerfTimer::new($monitor, $name);
    };
}

pub struct PerfTimer<'a> {
    monitor: &'a PerformanceMonitor,
    name: String,
}

impl<'a> PerfTimer<'a> {
    pub fn new(monitor: &'a PerformanceMonitor, name: &str) -> Self {
        monitor.start_timer(name);
        Self {
            monitor,
            name: name.to_string(),
        }
    }
}

impl<'a> Drop for PerfTimer<'a> {
    fn drop(&mut self) {
        self.monitor.end_timer(&self.name);
    }
}
```

## 内存分析

### 内存泄漏检测

```rust
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::alloc::{GlobalAlloc, Layout, System};
use std::ptr::NonNull;

pub struct MemoryTracker {
    allocations: Arc<Mutex<HashMap<usize, AllocationInfo>>>,
    total_allocated: Arc<Mutex<usize>>,
    peak_usage: Arc<Mutex<usize>>,
    allocation_count: Arc<Mutex<u64>>,
    deallocation_count: Arc<Mutex<u64>>,
}

#[derive(Debug, Clone)]
pub struct AllocationInfo {
    pub size: usize,
    pub timestamp: u64,
    pub backtrace: Vec<String>, // 简化的调用栈
}

impl MemoryTracker {
    pub fn new() -> Self {
        Self {
            allocations: Arc::new(Mutex::new(HashMap::new())),
            total_allocated: Arc::new(Mutex::new(0)),
            peak_usage: Arc::new(Mutex::new(0)),
            allocation_count: Arc::new(Mutex::new(0)),
            deallocation_count: Arc::new(Mutex::new(0)),
        }
    }
    
    pub fn track_allocation(&self, ptr: usize, size: usize) {
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        let info = AllocationInfo {
            size,
            timestamp,
            backtrace: self.capture_backtrace(),
        };
        
        if let Ok(mut allocations) = self.allocations.lock() {
            allocations.insert(ptr, info);
        }
        
        if let Ok(mut total) = self.total_allocated.lock() {
            *total += size;
            
            if let Ok(mut peak) = self.peak_usage.lock() {
                if *total > *peak {
                    *peak = *total;
                }
            }
        }
        
        if let Ok(mut count) = self.allocation_count.lock() {
            *count += 1;
        }
    }
    
    pub fn track_deallocation(&self, ptr: usize) {
        if let Ok(mut allocations) = self.allocations.lock() {
            if let Some(info) = allocations.remove(&ptr) {
                if let Ok(mut total) = self.total_allocated.lock() {
                    *total = total.saturating_sub(info.size);
                }
            }
        }
        
        if let Ok(mut count) = self.deallocation_count.lock() {
            *count += 1;
        }
    }
    
    fn capture_backtrace(&self) -> Vec<String> {
        // 简化的调用栈捕获
        vec![
            "main".to_string(),
            "allocate_memory".to_string(),
            "Vec::new".to_string(),
        ]
    }
    
    pub fn get_current_usage(&self) -> usize {
        if let Ok(total) = self.total_allocated.lock() {
            *total
        } else {
            0
        }
    }
    
    pub fn get_peak_usage(&self) -> usize {
        if let Ok(peak) = self.peak_usage.lock() {
            *peak
        } else {
            0
        }
    }
    
    pub fn get_allocation_count(&self) -> u64 {
        if let Ok(count) = self.allocation_count.lock() {
            *count
        } else {
            0
        }
    }
    
    pub fn get_deallocation_count(&self) -> u64 {
        if let Ok(count) = self.deallocation_count.lock() {
            *count
        } else {
            0
        }
    }
    
    pub fn detect_leaks(&self) -> Vec<MemoryLeak> {
        let mut leaks = Vec::new();
        
        if let Ok(allocations) = self.allocations.lock() {
            let current_time = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;
            
            for (ptr, info) in allocations.iter() {
                let age = current_time - info.timestamp;
                
                // 认为超过5秒未释放的内存可能是泄漏
                if age > 5000 {
                    leaks.push(MemoryLeak {
                        address: *ptr,
                        size: info.size,
                        age_ms: age,
                        backtrace: info.backtrace.clone(),
                    });
                }
            }
        }
        
        leaks.sort_by_key(|leak| std::cmp::Reverse(leak.size));
        leaks
    }
    
    pub fn generate_report(&self) -> MemoryReport {
        let current_usage = self.get_current_usage();
        let peak_usage = self.get_peak_usage();
        let allocation_count = self.get_allocation_count();
        let deallocation_count = self.get_deallocation_count();
        let leaks = self.detect_leaks();
        
        let active_allocations = if let Ok(allocations) = self.allocations.lock() {
            allocations.len()
        } else {
            0
        };
        
        MemoryReport {
            current_usage,
            peak_usage,
            allocation_count,
            deallocation_count,
            active_allocations,
            potential_leaks: leaks,
        }
    }
}

#[derive(Debug, Clone)]
pub struct MemoryLeak {
    pub address: usize,
    pub size: usize,
    pub age_ms: u64,
    pub backtrace: Vec<String>,
}

#[derive(Debug)]
pub struct MemoryReport {
    pub current_usage: usize,
    pub peak_usage: usize,
    pub allocation_count: u64,
    pub deallocation_count: u64,
    pub active_allocations: usize,
    pub potential_leaks: Vec<MemoryLeak>,
}

impl MemoryReport {
    pub fn print_summary(&self) {
        println!("=== Memory Usage Report ===");
        println!("Current Usage: {} bytes", self.current_usage);
        println!("Peak Usage: {} bytes", self.peak_usage);
        println!("Total Allocations: {}", self.allocation_count);
        println!("Total Deallocations: {}", self.deallocation_count);
        println!("Active Allocations: {}", self.active_allocations);
        println!("Potential Leaks: {}", self.potential_leaks.len());
        
        if !self.potential_leaks.is_empty() {
            println!("\n=== Potential Memory Leaks ===");
            for (i, leak) in self.potential_leaks.iter().enumerate().take(10) {
                println!("Leak #{}: {} bytes at 0x{:x} (age: {}ms)",
                        i + 1,
                        leak.size,
                        leak.address,
                        leak.age_ms);
                
                if !leak.backtrace.is_empty() {
                    println!("  Backtrace:");
                    for frame in &leak.backtrace {
                        println!("    {}", frame);
                    }
                }
            }
        }
        
        let leak_ratio = if self.allocation_count > 0 {
            (self.potential_leaks.len() as f64 / self.allocation_count as f64) * 100.0
        } else {
            0.0
        };
        
        println!("\nLeak Ratio: {:.2}%", leak_ratio);
        
        if leak_ratio > 5.0 {
            println!("WARNING: High leak ratio detected!");
        }
    }
}

// 自定义分配器用于内存跟踪
pub struct TrackingAllocator {
    tracker: MemoryTracker,
}

impl TrackingAllocator {
    pub fn new() -> Self {
        Self {
            tracker: MemoryTracker::new(),
        }
    }
    
    pub fn get_tracker(&self) -> &MemoryTracker {
        &self.tracker
    }
}

unsafe impl GlobalAlloc for TrackingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = System.alloc(layout);
        if !ptr.is_null() {
            self.tracker.track_allocation(ptr as usize, layout.size());
        }
        ptr
    }
    
    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        self.tracker.track_deallocation(ptr as usize);
        System.dealloc(ptr, layout);
    }
}
```

## 总结

调试技术是嵌入式开发的核心技能：

1. **调试器集成**：使用GDB等工具进行源码级调试
2. **日志系统**：实现分级日志记录和分析
3. **性能分析**：监控函数执行时间和系统性能
4. **内存分析**：检测内存泄漏和优化内存使用
5. **自动化工具**：开发自动化调试和分析工具

通过系统性的调试技术，可以快速定位问题，提高开发效率，确保系统的稳定性和可靠性。
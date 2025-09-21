# 性能监控系统

这是一个基于STM32F4和RTIC框架的实时性能监控系统，能够全面监控嵌入式系统的各项性能指标。

## 项目特性

### 监控指标
- **CPU使用率监控**：实时监控CPU使用率，包括平均值、峰值和历史趋势
- **内存使用监控**：监控堆内存、栈内存使用情况和内存碎片率
- **中断延迟监控**：测量中断响应时间和处理延迟
- **任务执行时间**：监控各个任务的执行时间和频率
- **功耗监控**：实时监控系统功耗和能耗统计
- **系统健康评分**：基于各项指标计算系统健康状态

### 系统特性
- **实时监控**：基于RTIC框架的实时任务调度
- **低开销**：优化的监控算法，最小化对系统性能的影响
- **自适应采样**：根据系统负载自动调整采样频率
- **异常检测**：自动检测性能异常和潜在问题
- **历史数据**：保存性能历史数据用于趋势分析
- **可视化输出**：通过串口和LED指示器显示系统状态

## 硬件连接

### STM32F4开发板连接
```
PA5  -> LED (系统状态指示)
PA0  -> 按钮 (触发性能测试)
PA1  -> 电压测量 (ADC输入)
PA2  -> 电流测量 (ADC输入)
```

### 外设配置
- **LED指示器**：
  - 绿色（常亮）：系统健康 (>80%)
  - 黄色（闪烁）：性能警告 (60-80%)
  - 红色（常亮）：严重问题 (<60%)

- **ADC配置**：
  - 12位分辨率
  - 采样频率：10kHz
  - 参考电压：3.3V

## 软件架构

### 核心组件

#### 1. 性能监控器 (SystemPerformanceMonitor)
```rust
pub struct SystemPerformanceMonitor {
    cpu_monitor: CpuMonitor,
    memory_monitor: MemoryMonitor,
    interrupt_monitor: InterruptLatencyMonitor,
    task_monitor: TaskMonitor,
    power_monitor: PowerMonitor,
    // ...
}
```

#### 2. CPU监控器 (CpuMonitor)
- 基于DWT周期计数器测量CPU使用率
- 记录空闲时间和总运行时间
- 计算平均使用率和峰值使用率
- 维护使用率历史数据

#### 3. 内存监控器 (MemoryMonitor)
- 监控堆内存分配和释放
- 跟踪栈指针变化
- 计算内存碎片率
- 检测内存泄漏

#### 4. 中断延迟监控器 (InterruptLatencyMonitor)
- 测量中断响应时间
- 记录最大延迟和平均延迟
- 检测中断抖动

#### 5. 任务监控器 (TaskMonitor)
- 跟踪任务执行时间
- 计算任务频率
- 检测任务超时

#### 6. 功耗监控器 (PowerMonitor)
- 实时测量电压和电流
- 计算瞬时功耗和平均功耗
- 统计总能耗

### RTIC任务架构

```rust
#[app(device = stm32f4xx_hal::pac, peripherals = true)]
mod app {
    // 共享资源
    #[shared]
    struct Shared {
        performance_monitor: SystemPerformanceMonitor,
        led: Pin<'A', 5, Output<PushPull>>,
    }
    
    // 本地资源
    #[local]
    struct Local {
        adc: Adc<pac::ADC1>,
        timer: Timer<pac::TIM2>,
    }
    
    // 任务定义
    #[task] async fn monitor_system();      // 优先级1：系统监控
    #[task] async fn update_display();      // 优先级1：显示更新
    #[task] async fn high_priority_task();  // 优先级2：高优先级任务
    #[task] async fn power_measurement();   // 优先级3：功耗测量
    #[task] fn button_interrupt();          // 优先级4：中断处理
}
```

## 构建和烧录

### 环境要求
- Rust 1.70+
- probe-rs
- STM32F4开发板

### 构建步骤
```bash
# 克隆项目
git clone <repository-url>
cd embedded-development/17-performance-optimization/projects/performance-monitor

# 构建项目
cargo build --release

# 烧录到开发板
cargo run --release --bin basic_monitor
```

### 其他示例程序
```bash
# CPU监控示例
cargo run --release --bin cpu_monitor

# 内存监控示例
cargo run --release --bin memory_monitor

# 功耗监控示例
cargo run --release --bin power_monitor

# 实时监控示例
cargo run --release --bin realtime_monitor

# 系统分析器
cargo run --release --bin system_profiler
```

## 系统配置

### 监控参数配置
```toml
[package.metadata.performance-monitor]
sampling_rate = 1000  # Hz
buffer_size = 1024
metrics = [
  "cpu_usage",
  "memory_usage", 
  "stack_usage",
  "interrupt_latency",
  "task_execution_time",
  "power_consumption"
]
```

### 实时约束配置
```toml
[package.metadata.realtime]
max_interrupt_latency = "10us"
max_task_execution_time = "1ms"
watchdog_timeout = "5s"
critical_memory_threshold = 90  # percent
```

## 功能特性

### 1. 实时监控
- 1ms精度的时间戳
- 微秒级中断延迟测量
- 实时CPU使用率计算
- 动态内存使用跟踪

### 2. 性能分析
- 系统健康评分算法
- 性能瓶颈自动检测
- 趋势分析和预测
- 异常模式识别

### 3. 数据管理
- 循环缓冲区存储历史数据
- 统计信息计算（平均值、最大值、最小值）
- 数据压缩和采样
- 内存高效的数据结构

### 4. 报警系统
- 可配置的阈值检测
- 多级报警机制
- 自动恢复检测
- 事件日志记录

## 扩展功能

### 1. 网络监控
```rust
// 添加网络性能监控
struct NetworkMonitor {
    packet_count: u32,
    bytes_transmitted: u64,
    latency_samples: Vec<u32, 32>,
    error_count: u32,
}
```

### 2. 文件系统监控
```rust
// 添加存储性能监控
struct StorageMonitor {
    read_operations: u32,
    write_operations: u32,
    read_latency: Vec<u32, 16>,
    write_latency: Vec<u32, 16>,
}
```

### 3. 温度监控
```rust
// 添加温度监控
struct ThermalMonitor {
    temperature_samples: Vec<f32, 64>,
    thermal_throttling: bool,
    overheat_events: u32,
}
```

## 调试和测试

### 性能测试
```bash
# 运行性能基准测试
cargo test --release performance_benchmarks

# 压力测试
cargo test --release stress_tests

# 内存泄漏测试
cargo test --release memory_leak_tests
```

### 调试输出
```rust
// 启用详细日志
export RUST_LOG=debug
cargo run --features semihosting

// 使用RTT输出
cargo run --features rtt
```

### 性能分析
```bash
# 生成性能报告
cargo run --release --bin system_profiler > performance_report.txt

# 分析内存使用
cargo run --release --bin memory_monitor > memory_analysis.txt
```

## 性能优化

### 1. 监控开销优化
- 使用无锁数据结构
- 优化采样算法
- 减少内存分配
- 批量数据处理

### 2. 实时性优化
- 中断优先级配置
- 任务调度优化
- 关键路径优化
- 缓存友好的数据布局

### 3. 功耗优化
- 动态采样频率调整
- 空闲时进入低功耗模式
- 外设按需启用
- 时钟频率动态调整

## 故障排除

### 常见问题

1. **高CPU使用率**
   - 检查任务执行时间
   - 优化算法复杂度
   - 减少中断频率
   - 使用DMA传输

2. **内存不足**
   - 检查内存泄漏
   - 优化数据结构
   - 减少缓冲区大小
   - 使用静态分配

3. **中断延迟过高**
   - 优化中断处理程序
   - 调整中断优先级
   - 减少关中断时间
   - 使用中断嵌套

4. **功耗过高**
   - 检查外设配置
   - 优化时钟设置
   - 使用低功耗模式
   - 减少不必要的计算

### 调试技巧
```rust
// 添加性能测量点
let start = DWT::cycle_count();
// 执行代码
let cycles = DWT::cycle_count() - start;
cortex_m_log::println!("Execution time: {} cycles", cycles);

// 内存使用检查
let stack_ptr = cortex_m::register::msp::read();
cortex_m_log::println!("Stack pointer: 0x{:08x}", stack_ptr);

// 中断统计
cortex_m_log::println!("Interrupt count: {}", interrupt_count);
```

## 开发指南

### 添加新的监控指标
1. 定义新的MetricType
2. 实现相应的监控器
3. 在SystemPerformanceMonitor中集成
4. 添加到性能报告中

### 自定义报警规则
```rust
impl SystemPerformanceMonitor {
    fn check_custom_alerts(&self) -> Vec<Alert, 8> {
        let mut alerts = Vec::new();
        
        // 自定义报警逻辑
        if self.get_cpu_usage() > CUSTOM_CPU_THRESHOLD {
            alerts.push(Alert::HighCpuUsage).ok();
        }
        
        alerts
    }
}
```

### 扩展数据输出
```rust
// 添加新的输出格式
impl PerformanceReport {
    fn to_json(&self) -> heapless::String<512> {
        // JSON序列化
    }
    
    fn to_csv(&self) -> heapless::String<256> {
        // CSV格式输出
    }
}
```

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 参考资料

- [RTIC框架文档](https://rtic.rs/)
- [STM32F4参考手册](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [Cortex-M4技术参考手册](https://developer.arm.com/documentation/100166/0001)
- [嵌入式Rust编程指南](https://docs.rust-embedded.org/book/)
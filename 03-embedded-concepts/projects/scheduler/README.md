# 嵌入式实时调度器

本项目实现了多种适用于嵌入式系统的实时调度算法，包括速率单调调度(RM)、最早截止期优先(EDF)、优先级继承协议(PIP)等，为嵌入式实时系统提供可靠的任务调度解决方案。

## 项目概述

实时调度是嵌入式系统的核心技术之一，特别是在对时间有严格要求的应用中。本项目提供了：

- 多种经典实时调度算法的实现
- 优先级反转问题的解决方案
- 任务同步和通信机制
- 调度性能分析和监控工具
- 可配置的调度策略

## 功能特性

### 调度算法
- **速率单调调度(RM)**: 基于任务周期的静态优先级调度
- **截止期单调调度(DM)**: 基于相对截止期的静态优先级调度
- **最早截止期优先(EDF)**: 动态优先级调度算法
- **最少松弛时间优先(LST)**: 基于松弛时间的调度
- **混合调度**: 结合多种算法的调度策略

### 同步机制
- **优先级继承协议(PIP)**: 解决优先级反转问题
- **优先级天花板协议(PCP)**: 防止死锁和优先级反转
- **信号量**: 任务同步原语
- **互斥锁**: 资源保护机制
- **消息队列**: 任务间通信

### 性能监控
- **调度延迟分析**: 测量任务响应时间
- **CPU利用率监控**: 实时监控系统负载
- **截止期错失统计**: 跟踪实时性能
- **任务执行时间分析**: 性能剖析工具
- **调度开销测量**: 系统开销分析

## 项目结构

```
scheduler/
├── src/
│   ├── lib.rs                 # 库入口
│   ├── schedulers/            # 调度算法实现
│   │   ├── mod.rs
│   │   ├── rate_monotonic.rs  # 速率单调调度
│   │   ├── edf.rs             # 最早截止期优先
│   │   ├── deadline_monotonic.rs # 截止期单调
│   │   └── hybrid.rs          # 混合调度
│   ├── sync/                  # 同步机制
│   │   ├── mod.rs
│   │   ├── mutex.rs           # 互斥锁
│   │   ├── semaphore.rs       # 信号量
│   │   ├── priority_inheritance.rs # 优先级继承
│   │   └── priority_ceiling.rs # 优先级天花板
│   ├── tasks/                 # 任务管理
│   │   ├── mod.rs
│   │   ├── task.rs            # 任务定义
│   │   ├── task_manager.rs    # 任务管理器
│   │   └── context.rs         # 上下文切换
│   ├── analysis/              # 性能分析
│   │   ├── mod.rs
│   │   ├── schedulability.rs  # 可调度性分析
│   │   ├── response_time.rs   # 响应时间分析
│   │   └── utilization.rs     # 利用率分析
│   └── bin/                   # 示例程序
│       ├── rate_monotonic_demo.rs
│       ├── edf_demo.rs
│       ├── priority_inheritance_demo.rs
│       └── scheduler_benchmark.rs
├── tests/                     # 测试用例
├── benches/                   # 性能测试
└── README.md
```

## 快速开始

### 构建项目

```bash
# 构建所有组件
cargo build

# 构建发布版本
cargo build --release

# 构建特定调度算法
cargo build --features rate-monotonic
cargo build --features earliest-deadline-first
```

### 运行示例

```bash
# 速率单调调度演示
cargo run --bin rate_monotonic_demo

# EDF调度演示
cargo run --bin edf_demo

# 优先级继承演示
cargo run --bin priority_inheritance_demo

# 调度器性能基准测试
cargo run --bin scheduler_benchmark --release
```

### 运行测试

```bash
# 运行所有测试
cargo test

# 运行特定测试
cargo test rate_monotonic
cargo test schedulability_analysis

# 运行性能测试
cargo bench
```

## 调度算法详解

### 1. 速率单调调度(RM)

速率单调调度是最经典的实时调度算法之一，为周期性任务分配静态优先级。

**特点**:
- 周期越短，优先级越高
- 静态优先级分配
- 适用于周期性任务
- 有理论可调度性界限

**可调度性条件**:
```
∑(Ci/Ti) ≤ n(2^(1/n) - 1)
```

其中：
- Ci: 任务i的执行时间
- Ti: 任务i的周期
- n: 任务数量

**使用场景**:
- 周期性控制任务
- 传感器数据采集
- 定时数据处理

### 2. 最早截止期优先(EDF)

EDF是最优的动态调度算法，总是选择截止期最早的任务执行。

**特点**:
- 动态优先级分配
- 理论上最优的调度算法
- 可以达到100%的CPU利用率
- 适用于周期性和非周期性任务

**可调度性条件**:
```
∑(Ci/Ti) ≤ 1
```

**使用场景**:
- 高利用率系统
- 混合任务类型
- 软实时系统

### 3. 优先级继承协议(PIP)

PIP解决优先级反转问题，当低优先级任务持有高优先级任务需要的资源时，临时提升其优先级。

**特点**:
- 动态优先级调整
- 解决优先级反转
- 保持系统响应性
- 实现相对简单

**工作原理**:
1. 高优先级任务被低优先级任务阻塞
2. 低优先级任务继承高优先级任务的优先级
3. 资源释放后恢复原优先级

### 4. 优先级天花板协议(PCP)

PCP通过预先计算资源的优先级天花板来防止死锁和优先级反转。

**特点**:
- 防止死锁
- 限制优先级反转时间
- 减少上下文切换
- 实现较复杂

**优势**:
- 保证无死锁
- 优先级反转时间有界
- 减少阻塞时间

## 使用示例

### 基本调度器使用

```rust
use scheduler::{RateMonotonicScheduler, Task, TaskConfig};

// 创建调度器
let mut scheduler = RateMonotonicScheduler::new();

// 定义任务
let task1_config = TaskConfig {
    id: 1,
    period: 100,        // 100ms周期
    execution_time: 20, // 20ms执行时间
    deadline: 100,      // 100ms截止期
    priority: 0,        // 将由RM算法自动分配
};

let task2_config = TaskConfig {
    id: 2,
    period: 200,
    execution_time: 50,
    deadline: 200,
    priority: 0,
};

// 添加任务
scheduler.add_task(task1_config)?;
scheduler.add_task(task2_config)?;

// 检查可调度性
if scheduler.is_schedulable() {
    println!("任务集可调度");
    
    // 开始调度
    scheduler.start();
} else {
    println!("任务集不可调度");
}
```

### 优先级继承使用

```rust
use scheduler::{PriorityInheritanceMutex, Task};

// 创建支持优先级继承的互斥锁
let mut mutex = PriorityInheritanceMutex::new();

// 在任务中使用
fn high_priority_task(mutex: &mut PriorityInheritanceMutex) {
    // 尝试获取锁
    if let Ok(_guard) = mutex.lock() {
        // 临界区代码
        critical_section_code();
    }
    // 锁自动释放
}

fn low_priority_task(mutex: &mut PriorityInheritanceMutex) {
    if let Ok(_guard) = mutex.lock() {
        // 如果高优先级任务等待，此任务会继承其优先级
        long_running_critical_section();
    }
}
```

### 性能分析

```rust
use scheduler::{ResponseTimeAnalyzer, UtilizationAnalyzer};

// 响应时间分析
let mut analyzer = ResponseTimeAnalyzer::new();
analyzer.add_task_set(&task_configs);

for (i, task) in task_configs.iter().enumerate() {
    if let Some(response_time) = analyzer.calculate_response_time(i) {
        println!("任务 {} 响应时间: {}ms", task.id, response_time);
        
        if response_time <= task.deadline {
            println!("任务 {} 满足截止期要求", task.id);
        } else {
            println!("任务 {} 可能错过截止期", task.id);
        }
    }
}

// 利用率分析
let utilization_analyzer = UtilizationAnalyzer::new(&task_configs);
let total_utilization = utilization_analyzer.calculate_total_utilization();
let rm_bound = utilization_analyzer.rate_monotonic_bound();

println!("总利用率: {:.2}%", total_utilization * 100.0);
println!("RM可调度界限: {:.2}%", rm_bound * 100.0);

if total_utilization <= rm_bound {
    println!("根据RM界限，任务集可调度");
} else {
    println!("需要进一步的可调度性分析");
}
```

## 配置选项

### 编译时配置

```toml
[features]
# 启用特定调度算法
rate-monotonic = []
earliest-deadline-first = []
priority-inheritance = []

# 调试功能
scheduler-debug = []
task-profiling = []
deadlock-detection = []
```

### 运行时配置

```rust
use scheduler::SchedulerConfig;

let config = SchedulerConfig {
    max_tasks: 32,              // 最大任务数
    time_slice_us: 1000,        // 时间片长度
    enable_profiling: true,     // 启用性能剖析
    enable_deadlock_detection: false, // 禁用死锁检测（性能优化）
    scheduler_type: SchedulerType::RateMonotonic,
};
```

## 性能优化

### 1. 调度开销优化

```rust
// 使用静态分配避免动态内存分配
const MAX_TASKS: usize = 16;
let mut scheduler = StaticScheduler::<MAX_TASKS>::new();

// 预计算优先级避免运行时计算
scheduler.precompute_priorities();

// 使用位操作优化就绪队列
scheduler.enable_bitmap_ready_queue();
```

### 2. 上下文切换优化

```rust
// 最小化保存的寄存器数量
scheduler.set_context_switch_mode(ContextSwitchMode::Minimal);

// 使用硬件辅助的上下文切换
scheduler.enable_hardware_context_switch();
```

### 3. 内存优化

```rust
// 使用栈分配的任务控制块
scheduler.use_stack_allocated_tcb();

// 优化任务栈大小
scheduler.set_default_stack_size(512); // 512字节栈
```

## 调试和分析

### 调度跟踪

```bash
# 启用调度跟踪
cargo run --features scheduler-debug

# 生成调度时序图
cargo run --bin scheduler_trace_analyzer
```

### 性能分析

```bash
# 运行性能基准测试
cargo bench

# 生成性能报告
cargo run --bin scheduler_benchmark --release > performance_report.txt
```

### 死锁检测

```bash
# 启用死锁检测
cargo run --features deadlock-detection

# 分析潜在的死锁情况
cargo run --bin deadlock_analyzer
```

## 最佳实践

### 1. 任务设计原则

- **单一职责**: 每个任务只负责一个明确的功能
- **最小执行时间**: 尽量减少任务的执行时间
- **避免阻塞**: 在高优先级任务中避免长时间阻塞操作
- **合理周期**: 根据实际需求设置任务周期

### 2. 优先级分配

```rust
// RM调度：周期越短优先级越高
task1.period = 50;  // 高优先级
task2.period = 100; // 中优先级  
task3.period = 200; // 低优先级

// DM调度：截止期越短优先级越高
task1.deadline = 30;  // 高优先级
task2.deadline = 80;  // 中优先级
task3.deadline = 150; // 低优先级
```

### 3. 资源共享

```rust
// 使用优先级继承避免优先级反转
let shared_resource = PriorityInheritanceMutex::new(data);

// 最小化临界区长度
{
    let _guard = shared_resource.lock();
    // 尽可能短的临界区代码
    quick_operation();
} // 锁自动释放
```

## 注意事项

1. **实时性保证**: 确保任务的执行时间不超过预期
2. **优先级反转**: 使用适当的同步机制避免优先级反转
3. **死锁预防**: 合理设计资源获取顺序
4. **栈溢出**: 为每个任务分配足够的栈空间
5. **中断处理**: 合理设计中断优先级和处理时间

## 扩展阅读

- [实时系统基础](../../docs/04-realtime-systems.md)
- [中断系统原理](../../docs/03-interrupt-system.md)
- [RTOS集成](../../../09-rtos-integration/README.md)

## 许可证

本项目采用 MIT 或 Apache-2.0 双重许可证。
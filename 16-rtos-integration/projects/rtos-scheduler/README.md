# RTOS调度器项目

这是一个基于RTIC框架的高级RTOS调度器实现，展示了多种调度算法和实时系统特性。

## 项目特性

### 调度算法
- **优先级调度** - 基于任务优先级的抢占式调度
- **轮转调度** - 时间片轮转调度算法
- **最早截止时间优先(EDF)** - 动态优先级调度
- **速率单调(RM)** - 静态优先级调度
- **抢占式调度** - 支持任务抢占
- **协作式调度** - 任务主动让出CPU

### 系统特性
- 实时任务管理
- 性能监控和统计
- 死锁检测
- 优先级反转处理
- 负载均衡
- 响应时间分析

## 硬件连接

### STM32F407VGT6开发板
```
LED (PA5)     -> 板载LED或外接LED
Button (PC13) -> 用户按钮（切换调度算法）
```

### 外设配置
- **TIM2**: 系统时钟 (1ms)
- **TIM3**: 任务1定时器 (10ms)
- **TIM4**: 任务2定时器 (20ms)
- **TIM5**: 任务3定时器 (50ms)
- **SysTick**: 单调时钟

## 软件架构

### 任务层次结构
```
优先级5: 系统时钟中断
优先级4: 任务定时器中断
优先级3: 按钮中断、高优先级任务
优先级2: 调度器、中优先级任务
优先级1: 低优先级任务、监控任务
```

### 调度器组件
1. **任务控制块(TCB)** - 存储任务状态信息
2. **就绪队列** - 管理可运行任务
3. **调度算法** - 选择下一个执行任务
4. **性能监控** - 收集系统统计信息

## 构建和烧录

### 环境要求
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install probe-rs --features cli
```

### 编译项目
```bash
# 编译主程序
cargo build --release

# 编译特定示例
cargo build --bin scheduler_demo --release
cargo build --bin priority_inversion --release
cargo build --bin deadline_scheduling --release
```

### 烧录程序
```bash
# 使用probe-rs烧录
probe-rs run --chip STM32F407VGTx target/thumbv7em-none-eabihf/release/rtos-scheduler

# 或使用OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program target/thumbv7em-none-eabihf/release/rtos-scheduler.bin 0x08000000 verify reset exit"
```

## 系统配置

### 调度参数
```rust
// 时间片配置
const TIME_SLICE: u32 = 10; // 10ms

// 任务配置
Task1: Priority=3, Period=10ms, Execution=2ms
Task2: Priority=2, Period=20ms, Execution=5ms
Task3: Priority=1, Period=50ms, Execution=8ms
```

### 内存配置
- **Flash**: 1024KB
- **RAM**: 128KB
- **任务栈**: 每个任务2KB
- **系统栈**: 4KB

## 功能特性

### 1. 多种调度算法
```rust
// 切换调度算法
match current_algorithm {
    PriorityBased => "基于优先级的抢占式调度",
    RoundRobin => "时间片轮转调度",
    EarliestDeadlineFirst => "最早截止时间优先",
    RateMonotonic => "速率单调调度",
    Preemptive => "抢占式调度",
    Cooperative => "协作式调度",
}
```

### 2. 实时性能监控
- CPU利用率统计
- 上下文切换计数
- 错过截止时间统计
- 响应时间分析
- 系统负载监控

### 3. 任务管理
- 动态任务创建
- 任务状态跟踪
- 优先级继承
- 资源管理

### 4. 故障检测
- 死锁检测
- 优先级反转检测
- 任务超时检测
- 系统过载检测

## 扩展功能

### 1. 高级调度特性
```rust
// 优先级继承协议
fn priority_inheritance() {
    // 防止优先级反转
}

// 负载均衡
fn load_balancing() {
    // 多核负载分配
}

// 能耗管理
fn power_management() {
    // 动态电压频率调节
}
```

### 2. 实时分析
```rust
// 可调度性分析
fn schedulability_analysis() -> bool {
    // 检查任务集是否可调度
}

// 响应时间分析
fn response_time_analysis() -> u32 {
    // 计算最坏情况响应时间
}
```

### 3. 通信机制
- 消息队列
- 信号量
- 互斥锁
- 事件标志

## 调试和测试

### 1. 性能测试
```bash
# 运行性能监控
cargo run --bin performance_monitor

# 测试调度延迟
cargo run --bin scheduler_latency_test

# 负载测试
cargo run --bin load_test
```

### 2. 调试输出
```rust
// 启用调试日志
cortex_m_log::println!("Task {} executing", task_id);
cortex_m_log::println!("Context switch: {} -> {}", old_task, new_task);
```

### 3. 实时跟踪
- 任务执行时间跟踪
- 中断延迟测量
- 系统调用统计

## 性能优化

### 1. 调度器优化
- 快速任务切换
- 优化的就绪队列
- 缓存友好的数据结构

### 2. 内存优化
- 静态内存分配
- 零拷贝消息传递
- 内存池管理

### 3. 实时优化
- 中断延迟最小化
- 关键路径优化
- 预测性调度

## 故障排除

### 常见问题

1. **任务无法调度**
   - 检查任务优先级设置
   - 验证任务状态
   - 确认就绪队列状态

2. **系统响应慢**
   - 检查CPU利用率
   - 分析任务执行时间
   - 优化调度算法

3. **内存不足**
   - 减少任务栈大小
   - 优化数据结构
   - 使用内存池

### 调试技巧
```rust
// 任务状态监控
fn debug_task_state(task: &TaskControlBlock) {
    cortex_m_log::println!(
        "Task {}: State={:?}, Priority={}, Deadline={}",
        task.task_id, task.state, task.priority, task.deadline
    );
}

// 调度器统计
fn debug_scheduler_stats(stats: &SchedulerStats) {
    cortex_m_log::println!(
        "Scheduler: Switches={}, Load={}%, Missed={}",
        stats.context_switches, stats.system_load, stats.missed_deadlines
    );
}
```

## 开发指南

### 1. 添加新任务
```rust
// 创建新任务
let new_task = TaskControlBlock::new(
    task_id,     // 任务ID
    priority,    // 优先级
    period,      // 周期
    exec_time    // 执行时间
);

// 注册任务
task_queue.push(new_task).ok();
```

### 2. 实现新调度算法
```rust
fn custom_scheduler(ctx: &mut scheduler_tick::Context) -> Option<u8> {
    // 实现自定义调度逻辑
    // 返回选中的任务ID
}
```

### 3. 性能监控扩展
```rust
// 添加新的性能指标
struct CustomMetrics {
    pub jitter: u32,
    pub throughput: u32,
    pub latency: u32,
}
```

## 参考资料

- [RTIC框架文档](https://rtic.rs/)
- [STM32F4xx HAL文档](https://docs.rs/stm32f4xx-hal/)
- [实时系统理论](https://en.wikipedia.org/wiki/Real-time_computing)
- [调度算法分析](https://en.wikipedia.org/wiki/Scheduling_(computing))

## 许可证

本项目采用MIT或Apache-2.0双重许可证。
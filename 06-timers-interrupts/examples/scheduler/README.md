# 调度器示例

本目录包含了STM32F4xx微控制器上各种任务调度器的实现示例，展示了协作式调度、抢占式调度、优先级调度和实时调度等不同的调度策略。

## 示例列表

### 1. 协作式调度器 (cooperative_scheduler.rs)

演示基于协作的任务调度，任务主动让出CPU控制权。

**功能特性：**
- 基于时间片的任务轮转
- 任务主动让出CPU
- 任务统计和性能监控
- 延时任务调度
- 任务启用/禁用控制
- 简单易理解的调度逻辑

**调度特点：**
- 非抢占式调度
- 任务必须主动让出CPU
- 适合简单的嵌入式应用
- 低开销，高可预测性

### 2. 抢占式调度器 (preemptive_scheduler.rs)

展示基于优先级的抢占式任务调度，高优先级任务可以抢占低优先级任务。

**功能特性：**
- 基于优先级的任务抢占
- 二叉堆实现的就绪队列
- 上下文切换统计
- CPU利用率监控
- 任务挂起/恢复控制
- 截止时间错过检测

**调度特点：**
- 抢占式调度
- 优先级驱动
- 实时响应能力
- 适合复杂的多任务应用

### 3. 优先级调度器 (priority_scheduler.rs)

演示优先级调度和优先级继承机制，解决优先级反转问题。

**功能特性：**
- 多级优先级管理
- 优先级继承协议
- 互斥锁管理
- 优先级反转检测
- 互斥锁争用统计
- 死锁预防机制

**调度特点：**
- 优先级继承
- 互斥锁保护
- 优先级反转处理
- 适合资源共享的应用

### 4. 实时调度器 (real_time_scheduler.rs)

实现硬实时和软实时任务调度，支持EDF (Earliest Deadline First) 算法。

**功能特性：**
- EDF调度算法
- 硬实时/软实时任务支持
- 截止时间错过处理
- 可调度性测试
- WCET (最坏执行时间) 监控
- 响应时间和抖动分析

**调度特点：**
- 截止时间驱动
- 可调度性保证
- 实时性能分析
- 适合实时控制系统

## 编译和运行

### 编译单个示例

```bash
# 编译协作式调度器
cargo build --bin cooperative_scheduler --release

# 编译抢占式调度器
cargo build --bin preemptive_scheduler --release

# 编译优先级调度器
cargo build --bin priority_scheduler --release

# 编译实时调度器
cargo build --bin real_time_scheduler --release
```

### 烧录到开发板

```bash
# 使用probe-run烧录
cargo run --bin cooperative_scheduler --release

# 或使用OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program target/thumbv7em-none-eabihf/release/cooperative_scheduler verify reset exit"
```

## 调度器理论基础

### 调度算法分类

1. **协作式调度 (Cooperative Scheduling)**
   - 任务主动让出CPU
   - 简单实现，低开销
   - 适合简单应用

2. **抢占式调度 (Preemptive Scheduling)**
   - 系统强制切换任务
   - 更好的响应性
   - 适合复杂应用

3. **优先级调度 (Priority Scheduling)**
   - 基于任务优先级
   - 支持优先级继承
   - 解决优先级反转

4. **实时调度 (Real-Time Scheduling)**
   - 截止时间约束
   - 可调度性分析
   - 硬实时保证

### 调度性能指标

1. **响应时间 (Response Time)**
   - 从任务就绪到开始执行的时间
   - 影响系统实时性

2. **周转时间 (Turnaround Time)**
   - 任务完成的总时间
   - 影响系统吞吐量

3. **CPU利用率 (CPU Utilization)**
   - CPU忙碌时间的百分比
   - 系统效率指标

4. **上下文切换开销 (Context Switch Overhead)**
   - 任务切换的时间成本
   - 影响系统性能

## 硬件连接

### 通用连接

```
PB0 -> 状态LED (系统运行指示)
PB1 -> 任务LED1 (高优先级任务指示)
PB2 -> 任务LED2 (中优先级任务指示)
PB3 -> 任务LED3 (低优先级任务指示)
```

### 实时调度器特殊连接

```
PB0 -> 状态LED (系统运行指示)
PB1 -> 硬实时任务LED
PB2 -> 软实时任务LED
PB3 -> 截止时间错过LED
```

## 使用说明

### 1. 协作式调度器测试

1. 连接LED到PB0-PB3
2. 烧录程序
3. 观察LED按时间片轮转闪烁
4. 监控任务统计信息

### 2. 抢占式调度器测试

1. 连接LED到PB0-PB2
2. 烧录程序
3. 观察高优先级任务抢占低优先级任务
4. 分析上下文切换频率

### 3. 优先级调度器测试

1. 连接LED到PB0-PB3
2. 烧录程序
3. 观察优先级继承机制
4. 监控互斥锁争用情况

### 4. 实时调度器测试

1. 连接LED到PB0-PB3
2. 烧录程序
3. 观察EDF调度行为
4. 监控截止时间错过情况

## 调度器配置

### 时间精度配置

```rust
// 协作式调度器：1ms时基
timer.start(1.khz());

// 抢占式调度器：1ms时基
timer.start(1.khz());

// 优先级调度器：1ms时基
timer.start(1.khz());

// 实时调度器：100μs时基 (高精度)
timer.start(10.khz());
```

### 任务优先级配置

```rust
pub enum Priority {
    Idle = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    Critical = 4,
}
```

### 实时任务配置

```rust
// 硬实时任务
RealTimeTask::new(
    1,                    // 任务ID
    TaskType::HardRealTime, // 任务类型
    Priority::Critical,   // 优先级
    1000,                // 1ms周期
    800,                 // 0.8ms截止时间
    200,                 // 0.2ms WCET
    task_handler,        // 处理函数
)
```

## 性能分析

### 可调度性测试

1. **Liu & Layland测试**
   ```
   EDF: U ≤ 1
   RM: U ≤ n(2^(1/n) - 1)
   ```

2. **响应时间分析**
   ```
   R_i = C_i + Σ(⌈R_i/T_j⌉ × C_j)
   ```

### 性能监控

```rust
// 获取调度器统计信息
let stats = scheduler.get_statistics();
println!("Context switches: {}", stats.context_switches);
println!("CPU utilization: {}%", stats.cpu_utilization);
println!("Deadline misses: {}", stats.total_deadline_misses);
```

## 注意事项

### 调度器设计原则

1. **确定性**：调度行为应该可预测
2. **公平性**：任务应该得到公平的CPU时间
3. **效率**：调度开销应该尽可能小
4. **实时性**：满足时间约束要求

### 常见问题

1. **优先级反转**
   - 使用优先级继承协议
   - 限制临界区长度
   - 避免嵌套锁

2. **截止时间错过**
   - 检查可调度性
   - 优化任务执行时间
   - 调整任务参数

3. **上下文切换开销**
   - 减少不必要的切换
   - 优化切换代码
   - 使用合适的时间片

### 调试技巧

1. **使用LED指示**：可视化任务执行状态
2. **统计分析**：收集调度性能数据
3. **时序分析**：使用示波器观察时序
4. **压力测试**：测试极限负载情况

## 扩展功能

### 1. 动态优先级调整

```rust
pub fn adjust_priority(&mut self, task_id: u32, new_priority: Priority) {
    // 动态调整任务优先级
}
```

### 2. 负载均衡

```rust
pub fn balance_load(&mut self) {
    // 在多核系统中平衡任务负载
}
```

### 3. 能耗管理

```rust
pub fn power_management(&mut self) {
    // 根据任务负载调整CPU频率
}
```

### 4. 故障恢复

```rust
pub fn fault_recovery(&mut self, task_id: u32) {
    // 任务故障时的恢复机制
}
```

## 应用场景

### 协作式调度器
- 简单的传感器数据采集
- 状态机实现
- 低功耗应用

### 抢占式调度器
- 多任务控制系统
- 用户界面应用
- 通信协议栈

### 优先级调度器
- 资源共享系统
- 复杂的嵌入式应用
- 工业控制系统

### 实时调度器
- 硬实时控制系统
- 音视频处理
- 安全关键系统

## 参考资料

- 《实时系统设计与分析》
- 《嵌入式实时操作系统》
- ARM Cortex-M编程手册
- FreeRTOS设计文档
- Rate Monotonic Analysis论文
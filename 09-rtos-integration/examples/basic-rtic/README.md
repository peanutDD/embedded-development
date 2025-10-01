# 基础 RTIC 示例

这是一个基础的 RTIC (Real-Time Interrupt-driven Concurrency) 示例项目，演示了如何在 STM32F4 微控制器上使用 RTIC 框架。

## 功能特性

- **LED 闪烁**: 使用软件定时器实现 LED 周期性闪烁
- **按钮中断**: 外部中断处理按钮按下事件
- **任务调度**: 展示 RTIC 的任务调度机制
- **资源共享**: 安全的共享资源访问
- **单调时钟**: 使用 DWT+SysTick 作为系统时钟源

## 硬件要求

- STM32F401 开发板 (或兼容的 STM32F4 系列)
- LED 连接到 PC13 引脚
- 按钮连接到 PA0 引脚 (带上拉电阻)

## 项目结构

```
basic_rtic/
├── Cargo.toml          # 项目配置和依赖
├── src/
│   └── main.rs         # 主程序代码
└── README.md           # 项目说明
```

## 代码说明

### 主要组件

1. **共享资源 (Shared)**
   - `counter`: 全局计数器，记录 LED 闪烁次数

2. **本地资源 (Local)**
   - `led`: PC13 引脚控制的 LED
   - `button`: PA0 引脚连接的按钮

3. **任务 (Tasks)**
   - `blink_led`: 周期性 LED 闪烁任务
   - `button_pressed`: 按钮中断处理任务
   - `rapid_blink`: 快速闪烁任务

### 工作流程

1. **初始化阶段**:
   - 配置系统时钟 (84MHz)
   - 初始化 GPIO 引脚
   - 配置外部中断
   - 启动 LED 闪烁任务

2. **运行阶段**:
   - LED 每 500ms 闪烁一次
   - 按钮按下时重置计数器并快速闪烁 3 次
   - 系统在空闲时进入低功耗模式

## 编译和烧录

### 前提条件

```bash
# 安装 Rust 嵌入式工具链
rustup target add thumbv7em-none-eabihf

# 安装调试工具
cargo install probe-run
```

### 编译项目

```bash
cd examples/basic_rtic
cargo build --release
```

### 烧录和运行

```bash
# 使用 probe-run 烧录和调试
cargo run --release

# 或者使用 OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
```

## 预期行为

1. **正常运行**: LED 每 500ms 闪烁一次
2. **按钮按下**: 
   - LED 快速闪烁 3 次 (每次约 60ms)
   - 计数器重置为 0
   - 恢复正常闪烁模式

## 学习要点

### RTIC 核心概念

1. **任务优先级**: 中断任务优先级高于软件任务
2. **资源锁定**: 使用 `lock()` 方法安全访问共享资源
3. **任务调度**: `spawn()` 和 `spawn_after()` 方法
4. **单调时钟**: 提供精确的时间基准

### 最佳实践

1. **中断处理**: 保持中断服务程序简短
2. **资源共享**: 最小化锁定时间
3. **任务设计**: 避免阻塞操作
4. **错误处理**: 使用 `Result` 类型处理可能的错误

## 扩展建议

1. **添加更多传感器**: 温度、湿度等
2. **实现通信协议**: UART、SPI、I2C
3. **添加显示功能**: LCD、OLED 显示
4. **电源管理**: 实现低功耗模式
5. **数据记录**: 添加数据存储功能

## 故障排除

### 常见问题

1. **编译错误**: 检查依赖版本兼容性
2. **烧录失败**: 确认调试器连接和权限
3. **LED 不闪烁**: 检查引脚配置和硬件连接
4. **按钮无响应**: 验证中断配置和引脚状态

### 调试技巧

1. 使用 `defmt` 进行日志输出
2. 利用 `probe-run` 的实时调试功能
3. 检查任务执行统计信息
4. 监控系统资源使用情况

## 相关资源

- [RTIC 官方文档](https://rtic.rs/)
- [STM32F4xx HAL 文档](https://docs.rs/stm32f4xx-hal/)
- [Cortex-M 编程指南](https://docs.rust-embedded.org/book/)
- [嵌入式 Rust 最佳实践](https://docs.rust-embedded.org/embedonomicon/)
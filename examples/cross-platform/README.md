# 跨平台嵌入式开发示例

本目录包含了跨平台嵌入式开发的示例代码，展示了如何创建平台无关的抽象层，实现代码在不同微控制器平台间的复用。

## 项目概述

跨平台嵌入式开发的核心思想是通过抽象层将硬件相关的代码与应用逻辑分离，使得同一套应用代码可以在不同的硬件平台上运行。

### 设计原则

- **硬件抽象层 (HAL)**：提供统一的硬件接口
- **平台无关性**：应用代码不依赖特定硬件
- **可移植性**：最小化平台迁移成本
- **模块化设计**：清晰的模块边界和接口
- **错误处理**：统一的错误处理机制

## 项目结构

```
cross-platform/
├── Cargo.toml              # 项目配置文件
├── README.md               # 本文档
├── src/
│   ├── lib.rs              # 跨平台抽象层库
│   ├── error.rs            # 统一错误处理
│   └── bin/
│       ├── gpio_abstraction.rs        # GPIO抽象示例
│       ├── sensor_interface.rs        # 传感器接口示例
│       └── communication_protocol.rs  # 通信协议示例
└── examples/
    ├── basic_usage.rs      # 基础使用示例
    └── advanced_features.rs # 高级功能示例
```

## 核心模块

### 1. 平台抽象特征 (Platform Abstraction Traits)

定义了各种硬件功能的统一接口：

- `GpioAbstraction`: GPIO操作抽象
- `SensorAbstraction`: 传感器读取抽象
- `CommunicationAbstraction`: 通信接口抽象
- `TimerAbstraction`: 定时器功能抽象

### 2. GPIO模块

提供跨平台的GPIO管理功能：

```rust
use cross_platform_examples::gpio::GpioManager;

let mut gpio = GpioManager::new();
gpio.configure_pin(0, PinMode::Output)?;
gpio.set_pin_high(0)?;
```

### 3. 传感器模块

统一的传感器接口：

```rust
use cross_platform_examples::sensor::SensorManager;

let mut sensors = SensorManager::new();
let reading = sensors.read_sensor(SensorType::Temperature)?;
```

### 4. 通信模块

跨平台通信抽象：

```rust
use cross_platform_examples::communication::CommunicationManager;

let mut comm = CommunicationManager::new();
comm.send_data(&data)?;
```

### 5. 时间模块

时间和定时器抽象：

```rust
use cross_platform_examples::time::{Timer, Duration};

let mut timer = Timer::new(Duration::from_millis(1000));
timer.start();
```

## 示例程序

### 1. GPIO抽象示例 (`gpio_abstraction.rs`)

展示如何使用GPIO抽象层控制LED和读取按钮状态：

- 跨平台GPIO初始化
- LED控制和闪烁模式
- 按钮状态检测
- 中断处理抽象

**主要功能：**
- 多种LED闪烁模式（慢闪、快闪、呼吸灯）
- 按钮防抖处理
- 状态机管理
- 平台特定的初始化和延时

### 2. 传感器接口示例 (`sensor_interface.rs`)

演示跨平台传感器数据采集和处理：

- 多传感器管理
- 数据滤波和校准
- 质量评估
- 错误处理和恢复

**支持的传感器类型：**
- 温度传感器 (DS18B20)
- 湿度传感器 (DHT22)
- 压力传感器 (BMP280)
- 光照传感器 (BH1750)

**数据处理功能：**
- 移动平均滤波
- 传感器校准
- 数据质量评估
- 统计信息收集

### 3. 通信协议示例 (`communication_protocol.rs`)

实现完整的跨平台通信协议栈：

- 分层协议架构
- 可靠数据传输
- 错误检测和纠正
- 网络管理

**协议栈层次：**
- **物理层**: 支持UART、SPI、I2C、CAN、以太网、无线
- **数据链路层**: 帧格式、校验和、重传机制
- **网络层**: 路由、地址管理、包转发
- **传输层**: 连接管理、流控制、可靠传输
- **应用层**: 消息处理、会话管理、命令执行

## 编译和运行

### 基本编译

```bash
# 编译库
cargo build

# 编译特定示例
cargo build --bin gpio_abstraction
cargo build --bin sensor_interface
cargo build --bin communication_protocol
```

### 平台特定编译

```bash
# STM32平台
cargo build --target thumbv7em-none-eabihf --features stm32

# ESP32平台
cargo build --target xtensa-esp32-none-elf --features esp32

# RP2040平台
cargo build --target thumbv6m-none-eabi --features rp2040

# nRF52平台
cargo build --target thumbv7em-none-eabihf --features nrf52
```

### 功能特性

```bash
# 启用调试功能
cargo build --features debug

# 启用异步支持
cargo build --features async

# 启用所有功能
cargo build --all-features
```

## 平台适配

### 添加新平台支持

1. **实现平台特定的HAL**：

```rust
// 在lib.rs中添加平台特定实现
#[cfg(feature = "new_platform")]
mod new_platform_hal {
    use crate::traits::*;
    
    pub struct NewPlatformGpio;
    
    impl GpioAbstraction for NewPlatformGpio {
        // 实现GPIO抽象接口
    }
}
```

2. **更新Cargo.toml**：

```toml
[features]
new_platform = ["new-platform-hal"]

[dependencies]
new-platform-hal = { version = "0.1", optional = true }
```

3. **添加平台初始化代码**：

```rust
#[cfg(feature = "new_platform")]
fn platform_init() -> Result<()> {
    // 平台特定初始化
    Ok(())
}
```

### 平台差异处理

- **时钟频率差异**: 通过配置文件或编译时常量处理
- **外设差异**: 使用条件编译和特征抽象
- **内存限制**: 通过堆栈大小配置和内存池管理
- **功耗管理**: 平台特定的低功耗模式实现

## 最佳实践

### 1. 抽象层设计

- 保持接口简洁明了
- 避免过度抽象
- 考虑性能影响
- 提供合理的默认实现

### 2. 错误处理

- 使用统一的错误类型
- 提供详细的错误信息
- 实现错误恢复机制
- 记录错误统计信息

### 3. 资源管理

- 明确资源所有权
- 及时释放资源
- 避免资源泄漏
- 使用RAII模式

### 4. 测试策略

- 单元测试抽象接口
- 集成测试完整功能
- 模拟测试硬件行为
- 跨平台兼容性测试

## 性能考虑

### 1. 抽象开销

- 使用零成本抽象
- 编译时优化
- 内联关键函数
- 避免动态分发

### 2. 内存使用

- 静态内存分配
- 避免堆分配
- 使用内存池
- 优化数据结构

### 3. 实时性

- 中断优先级管理
- 任务调度优化
- 减少关键路径延迟
- 使用硬件加速

## 调试和诊断

### 1. 调试输出

```rust
// 使用条件编译的调试输出
#[cfg(feature = "debug")]
fn debug_print(msg: &str) {
    #[cfg(feature = "defmt")]
    defmt::info!("{}", msg);
    
    #[cfg(feature = "semihosting")]
    cortex_m_semihosting::hprintln!("{}", msg).ok();
}
```

### 2. 性能分析

- 使用DWT周期计数器
- 测量函数执行时间
- 监控内存使用
- 分析中断延迟

### 3. 错误诊断

- 错误码映射
- 调用栈跟踪
- 状态转储
- 日志记录

## 扩展功能

### 1. 异步支持

```rust
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
async fn sensor_task() {
    let mut sensor = SensorManager::new();
    
    loop {
        let reading = sensor.read_async().await;
        // 处理传感器数据
        Timer::after(Duration::from_millis(100)).await;
    }
}
```

### 2. 网络功能

- TCP/UDP协议栈
- HTTP客户端/服务器
- MQTT支持
- CoAP协议

### 3. 文件系统

- FAT32支持
- 日志文件管理
- 配置文件存储
- 固件更新

## 常见问题

### Q: 如何选择合适的抽象级别？

A: 抽象级别应该平衡易用性和性能。过度抽象会影响性能，抽象不足会降低可移植性。建议从具体需求出发，逐步抽象。

### Q: 如何处理平台特定的功能？

A: 使用条件编译和特征门控，为平台特定功能提供可选的抽象接口，同时保持核心功能的跨平台兼容性。

### Q: 如何优化跨平台代码的性能？

A: 使用零成本抽象、编译时优化、内联函数等技术。在关键路径上可以考虑平台特定的优化实现。

### Q: 如何测试跨平台代码？

A: 结合单元测试、集成测试和硬件在环测试。使用模拟器和仿真器进行跨平台兼容性验证。

## 学习资源

### 官方文档

- [Rust Embedded Book](https://docs.rust-embedded.org/book/)
- [Embassy Framework](https://embassy.dev/)
- [embedded-hal](https://docs.rs/embedded-hal/)

### 示例项目

- [Real-Time Interrupt-driven Concurrency](https://rtic.rs/)
- [Knurling-rs](https://knurling.ferrous-systems.com/)
- [Awesome Embedded Rust](https://github.com/rust-embedded/awesome-embedded-rust)

### 社区资源

- [Rust Embedded Working Group](https://github.com/rust-embedded/wg)
- [Embedded Rust Community](https://matrix.to/#/#rust-embedded:matrix.org)
- [Rust Embedded Blog](https://blog.rust-embedded.org/)

## 贡献指南

欢迎贡献代码和改进建议！请遵循以下步骤：

1. Fork本项目
2. 创建功能分支
3. 提交更改
4. 创建Pull Request

### 代码规范

- 遵循Rust官方代码风格
- 添加适当的文档注释
- 编写单元测试
- 更新相关文档

## 许可证

本项目采用MIT许可证，详见LICENSE文件。
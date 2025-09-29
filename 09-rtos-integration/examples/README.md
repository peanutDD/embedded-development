# RTOS Integration Examples - RTOS集成示例

本目录包含了各种RTOS框架的示例代码，展示了在嵌入式系统中如何使用不同的实时操作系统进行开发。

## 示例项目概览

### 1. RTIC框架示例

#### basic_rtic - 基础RTIC示例
- **功能**: LED闪烁和按钮中断处理
- **特性**: 
  - 基于RTIC 2.0框架
  - 硬件抽象层使用
  - 中断驱动编程
  - 资源共享管理
- **适用场景**: RTIC框架入门学习
- **硬件**: STM32F407VG开发板

#### rtic-basic - RTIC基础功能
- **功能**: RTIC核心功能演示
- **特性**:
  - 任务调度和优先级管理
  - 共享资源访问
  - 单调时钟使用
- **适用场景**: RTIC进阶学习

### 2. FreeRTOS示例

#### freertos_tasks - FreeRTOS任务管理
- **功能**: 多任务创建、优先级管理和任务间通信
- **特性**:
  - 4个不同优先级的任务
  - 队列通信机制
  - 内存管理和错误处理
  - 系统监控和统计
- **适用场景**: FreeRTOS多任务系统开发
- **硬件**: STM32F407VG开发板

#### freertos-basic - FreeRTOS基础功能
- **功能**: FreeRTOS基本概念演示
- **特性**:
  - 任务创建和删除
  - 信号量和互斥量
  - 定时器使用

### 3. Embassy异步框架示例

#### embassy-async - Embassy异步编程
- **功能**: 基于Embassy框架的异步编程示例
- **特性**:
  - async/await语法使用
  - 并发任务执行
  - 异步I/O操作
  - 零分配运行时
  - 事件驱动编程
- **适用场景**: 现代异步嵌入式开发
- **硬件**: STM32F407VG开发板

### 4. 系统功能示例

#### memory_management - 内存管理示例
- **功能**: 嵌入式系统内存管理演示
- **特性**:
  - 堆内存分配器
  - 内存池管理
  - 栈使用监控
  - 内存泄漏检测
  - 碎片化分析
- **适用场景**: 内存受限系统开发
- **硬件**: STM32F407VG开发板

#### network_stack - 网络协议栈示例
- **功能**: 嵌入式网络通信演示
- **特性**:
  - TCP/UDP协议支持
  - HTTP客户端功能
  - 以太网硬件驱动
  - 网络性能监控
- **适用场景**: IoT设备网络功能开发
- **硬件**: STM32F407VG + ENC28J60以太网模块

## 示例项目对比

| 示例项目 | 框架 | 复杂度 | 主要特性 | 学习重点 |
|---------|------|--------|----------|----------|
| basic_rtic | RTIC | 简单 | LED控制、中断处理 | RTIC基础概念 |
| rtic-basic | RTIC | 中等 | 资源共享、任务调度 | RTIC进阶功能 |
| freertos_tasks | FreeRTOS | 中等 | 多任务、队列通信 | FreeRTOS任务管理 |
| freertos-basic | FreeRTOS | 简单 | 基础任务、同步原语 | FreeRTOS基础 |
| embassy-async | Embassy | 高级 | 异步编程、并发执行 | 异步编程模式 |
| memory_management | 通用 | 高级 | 内存分配、监控 | 内存管理技术 |
| network_stack | 通用 | 高级 | 网络通信、协议栈 | 网络编程 |

## 快速开始

### 环境准备

```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install probe-run
cargo install cargo-embed

# 安装调试工具
cargo install rtt-target  # 用于RTT日志
cargo install defmt-print # 用于defmt日志
```

### 运行示例

```bash
# 进入任意示例目录
cd basic_rtic

# 编译项目
cargo build --release

# 烧录和运行
cargo run --release

# 或使用cargo-embed
cargo embed --release
```

### 调试模式

```bash
# 调试模式运行
cargo run

# 查看日志输出
# RTT或defmt日志会自动显示在终端
```

## 学习路径建议

### 初学者路径
1. **basic_rtic** - 学习RTIC基础概念
2. **freertos-basic** - 了解FreeRTOS基本功能
3. **freertos_tasks** - 掌握多任务编程
4. **rtic-basic** - 深入RTIC进阶功能

### 进阶路径
1. **embassy-async** - 学习现代异步编程
2. **memory_management** - 掌握内存管理技术
3. **network_stack** - 学习网络编程

### 专业路径
1. 结合多个示例，理解不同框架的优缺点
2. 根据项目需求选择合适的RTOS框架
3. 学习性能优化和调试技巧

## 硬件要求

### 通用要求
- **开发板**: STM32F407VG Discovery或类似
- **调试器**: ST-Link V2或兼容调试器
- **连接线**: USB数据线

### 特殊要求
- **网络示例**: 需要ENC28J60以太网模块
- **传感器示例**: 需要I2C/SPI传感器模块
- **显示示例**: 需要LCD显示屏模块

## 常见问题

### 编译问题
```bash
# 检查工具链安装
rustup show
rustup target list --installed

# 更新依赖
cargo update
```

### 烧录问题
```bash
# 检查调试器连接
probe-run --list-probes

# 检查目标芯片
probe-run --chip STM32F407VGTx --list-chips
```

### 运行时问题
- 检查硬件连接
- 验证时钟配置
- 查看日志输出

## 扩展学习

### 相关文档
- [RTIC官方文档](https://rtic.rs/)
- [FreeRTOS官方文档](https://www.freertos.org/)
- [Embassy官方文档](https://embassy.dev/)
- [STM32F4xx HAL文档](https://docs.rs/stm32f4xx-hal/)

### 进阶项目
- 查看 `../projects/` 目录下的实战项目
- 多任务调度器实现
- IoT网关开发
- 实时控制系统

### 社区资源
- [Rust嵌入式工作组](https://github.com/rust-embedded)
- [嵌入式Rust书籍](https://docs.rust-embedded.org/book/)
- [Discovery书籍](https://docs.rust-embedded.org/discovery/)

## 贡献指南

欢迎提交新的示例项目或改进现有示例：

1. Fork本仓库
2. 创建新的示例目录
3. 添加完整的文档和README
4. 确保代码可以正常编译和运行
5. 提交Pull Request

### 示例项目要求
- 完整的Cargo.toml配置
- 详细的README文档
- 清晰的代码注释
- 硬件连接说明
- 预期行为描述
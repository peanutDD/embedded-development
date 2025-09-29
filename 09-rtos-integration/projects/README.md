# RTOS Integration Projects - RTOS集成实战项目

本目录包含了基于RTOS的实战项目，展示了如何在实际应用中使用实时操作系统构建复杂的嵌入式系统。

## 项目概览

### 1. task-scheduler - 基础任务调度器
- **项目描述**: 基础的任务调度器实现，展示RTOS任务管理的核心概念
- **技术栈**: RTIC框架
- **主要功能**:
  - 多任务调度管理
  - 优先级分配
  - 资源共享
  - 基础性能监控
- **适用场景**: 学习RTOS任务调度原理
- **复杂度**: ⭐⭐

### 2. multi-task-scheduler - 多任务调度器
- **项目描述**: 高级多任务调度器，实现复杂的实时系统管理
- **技术栈**: RTIC框架 + 性能监控
- **主要功能**:
  - 4个不同优先级任务协调
  - 实时性能统计和分析
  - 任务间通信机制
  - 紧急处理和安全机制
  - 完整的系统监控
- **适用场景**: 工业控制、数据采集系统
- **复杂度**: ⭐⭐⭐⭐

### 3. iot-gateway - IoT网关
- **项目描述**: 完整的IoT网关实现，集成传感器数据采集和云端通信
- **技术栈**: RTIC + 网络协议栈 + 数据加密
- **主要功能**:
  - 多传感器数据采集 (I2C/SPI/ADC)
  - 网络通信和云端数据传输
  - 数据加密和安全传输
  - 实时系统监控
  - 设备管理和配置
- **适用场景**: IoT设备、智能家居、工业监控
- **复杂度**: ⭐⭐⭐⭐⭐

## 项目对比分析

| 项目 | 框架 | 任务数 | 网络功能 | 传感器支持 | 安全特性 | 适用场景 |
|------|------|--------|----------|------------|----------|----------|
| task-scheduler | RTIC | 3-4个 | ❌ | 基础 | 基础 | 学习和原型 |
| multi-task-scheduler | RTIC | 5-7个 | ❌ | 模拟 | 中等 | 工业控制 |
| iot-gateway | RTIC | 7-10个 | ✅ | 完整 | 高级 | 商业产品 |

## 技术特性对比

### 实时性能
- **task-scheduler**: 基础实时响应，适合学习
- **multi-task-scheduler**: 高精度实时调度，微秒级响应
- **iot-gateway**: 复杂实时系统，多任务并发处理

### 系统复杂度
- **task-scheduler**: 简单系统架构，易于理解
- **multi-task-scheduler**: 中等复杂度，完整的监控系统
- **iot-gateway**: 高复杂度，企业级系统架构

### 功能完整性
- **task-scheduler**: 基础功能演示
- **multi-task-scheduler**: 完整的调度和监控功能
- **iot-gateway**: 端到端的IoT解决方案

## 快速开始

### 环境准备

```bash
# 安装必要工具
rustup target add thumbv7em-none-eabihf
cargo install probe-run cargo-embed
cargo install rtt-target defmt-print

# 克隆项目
git clone <repository-url>
cd 09-rtos-integration/projects
```

### 运行项目

```bash
# 选择项目
cd multi-task-scheduler

# 编译和运行
cargo run --release

# 或使用embed
cargo embed --release
```

## 项目详细介绍

### Task Scheduler - 基础任务调度器

#### 架构设计
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Task   │    │  Control Task   │    │  Monitor Task   │
│   (Priority 3)  │    │   (Priority 2)  │    │   (Priority 1)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │ Shared Resources│
                    │  - System State │
                    │  - Statistics   │
                    └─────────────────┘
```

#### 核心特性
- 基于优先级的抢占式调度
- 共享资源安全访问
- 基础性能统计
- 简单的错误处理

### Multi-Task Scheduler - 多任务调度器

#### 系统架构
```
┌─────────────────────────────────────────────────────────────────┐
│                        System Monitor                          │
│                       (Priority 1)                             │
└─────────────────────────────────────────────────────────────────┘
         │
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│  Sensor Task    │  │  Control Task   │  │   Comm Task     │
│  (Priority 3)   │  │  (Priority 2)   │  │  (Priority 2)   │
│  - 100Hz        │  │  - 50Hz         │  │  - 20Hz         │
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                    │                    │
         └────────────────────┼────────────────────┘
                              │
                    ┌─────────────────┐
                    │ Emergency Task  │
                    │  (Priority 4)   │
                    └─────────────────┘
```

#### 高级特性
- 实时性能监控和统计
- 任务执行时间测量
- 紧急处理机制
- 完整的错误恢复
- 系统健康检查

### IoT Gateway - IoT网关

#### 系统架构
```
┌─────────────────────────────────────────────────────────────────┐
│                      IoT Gateway System                        │
└─────────────────────────────────────────────────────────────────┘
         │
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ Sensor Manager  │  │ Network Manager │  │ Security Module │
│ - I2C Sensors   │  │ - TCP/UDP       │  │ - Encryption    │
│ - SPI Sensors   │  │ - HTTP Client   │  │ - Authentication│
│ - ADC Channels  │  │ - MQTT Client   │  │ - Data Integrity│
└─────────────────┘  └─────────────────┘  └─────────────────┘
         │                    │                    │
         └────────────────────┼────────────────────┘
                              │
┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐
│ Data Processor  │  │ Storage Manager │  │ Device Manager  │
│ - Data Fusion   │  │ - Flash Storage │  │ - Configuration │
│ - Filtering     │  │ - Logging       │  │ - Status Report │
│ - Validation    │  │ - Buffering     │  │ - Remote Update │
└─────────────────┘  └─────────────────┘  └─────────────────┘
```

#### 企业级特性
- 多协议传感器支持
- 安全的数据传输
- 边缘计算能力
- 远程设备管理
- 故障自动恢复
- 性能监控和报警

## 硬件要求

### 基础硬件 (所有项目)
- **MCU**: STM32F407VG (Cortex-M4, 168MHz)
- **内存**: 1MB Flash + 192KB RAM
- **调试器**: ST-Link V2或兼容设备
- **开发板**: STM32F407 Discovery或类似

### 扩展硬件 (IoT Gateway)
```
网络模块:
- ENC28J60 以太网控制器
- ESP32 WiFi模块 (可选)

传感器模块:
- DHT22 温湿度传感器 (I2C)
- BMP280 气压传感器 (I2C)
- LDR 光敏电阻 (ADC)
- 电压分压器 (ADC)

显示和交互:
- OLED显示屏 (I2C)
- 按钮和LED指示灯
- 蜂鸣器 (可选)

存储扩展:
- SD卡模块 (SPI)
- 外部Flash (SPI)
```

## 性能指标

### Task Scheduler
- **内存使用**: Flash ~20KB, RAM ~4KB
- **任务切换**: <2μs
- **中断响应**: <5μs
- **CPU利用率**: ~30%

### Multi-Task Scheduler
- **内存使用**: Flash ~35KB, RAM ~8KB
- **任务切换**: <1μs
- **中断响应**: <3μs
- **实时性**: 微秒级精度
- **CPU利用率**: ~60%

### IoT Gateway
- **内存使用**: Flash ~80KB, RAM ~24KB
- **网络吞吐**: 1Mbps+
- **传感器采样**: 100Hz+
- **数据延迟**: <100ms
- **CPU利用率**: ~80%

## 开发指南

### 项目选择建议

#### 初学者
1. 从 `task-scheduler` 开始
2. 理解RTOS基本概念
3. 学习任务调度原理

#### 中级开发者
1. 学习 `multi-task-scheduler`
2. 掌握性能监控技术
3. 理解实时系统设计

#### 高级开发者
1. 研究 `iot-gateway`
2. 学习系统集成技术
3. 掌握企业级开发

### 自定义开发

#### 添加新任务
```rust
#[task(shared = [system_state], priority = 2)]
fn custom_task(ctx: custom_task::Context) {
    // 任务实现
}

// 在init函数中启动
custom_task::spawn().ok();
```

#### 扩展传感器支持
```rust
pub struct CustomSensor {
    // 传感器配置
}

impl CustomSensor {
    pub fn read(&mut self) -> Result<f32, SensorError> {
        // 传感器读取实现
    }
}
```

#### 添加网络协议
```rust
pub struct CustomProtocol {
    // 协议配置
}

impl CustomProtocol {
    pub async fn send_data(&mut self, data: &[u8]) -> Result<(), NetworkError> {
        // 协议实现
    }
}
```

## 测试和验证

### 单元测试
```bash
# 运行单元测试
cargo test

# 运行特定测试
cargo test sensor_reading
```

### 集成测试
```bash
# 硬件在环测试
cargo run --bin integration_test

# 性能基准测试
cargo bench
```

### 调试技巧

#### 使用RTT日志
```rust
use rtt_target::{rprintln, rtt_init_print};

rtt_init_print!();
rprintln!("Debug message: {}", value);
```

#### 性能分析
```rust
let start = monotonics::MyMono::now();
// ... 代码执行 ...
let duration = monotonics::MyMono::now() - start;
rprintln!("Execution time: {} μs", duration.ticks());
```

## 部署和维护

### 生产部署
1. 使用release模式编译
2. 启用硬件看门狗
3. 配置故障恢复机制
4. 实施远程监控

### 系统维护
1. 定期性能监控
2. 日志分析和故障诊断
3. 固件更新机制
4. 配置备份和恢复

## 扩展项目建议

### 基于现有项目的扩展
1. **智能家居控制器**: 基于IoT Gateway扩展
2. **工业数据采集系统**: 基于Multi-Task Scheduler扩展
3. **机器人控制系统**: 基于Task Scheduler扩展

### 新项目方向
1. **边缘AI计算节点**
2. **分布式传感器网络**
3. **实时视频处理系统**
4. **无线通信基站**

## 社区和支持

### 获取帮助
- GitHub Issues: 报告问题和请求功能
- 讨论区: 技术讨论和经验分享
- 文档: 详细的技术文档和教程

### 贡献代码
1. Fork项目仓库
2. 创建功能分支
3. 提交代码和测试
4. 创建Pull Request

### 许可证
本项目采用MIT或Apache-2.0双重许可证，详见LICENSE文件。
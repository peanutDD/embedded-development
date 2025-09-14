# 基础LED控制项目

## 项目概述

这是一个入门级的嵌入式Rust项目，演示如何使用GPIO控制LED的基本操作。项目基于STM32F407VG Discovery开发板，通过控制板载LED实现闪烁效果。

## 学习目标

- 掌握嵌入式Rust项目的基本结构
- 理解GPIO输出控制的原理和实现
- 学习使用HAL库进行硬件抽象
- 掌握系统定时器的使用方法
- 了解嵌入式调试和日志输出

## 硬件要求

### 主要硬件
- **开发板**: STM32F407VG Discovery
- **调试器**: 板载ST-LINK/V2
- **LED**: 板载LED (PC13引脚)

### 硬件规格
- **MCU**: STM32F407VGT6
- **架构**: ARM Cortex-M4F
- **主频**: 168MHz
- **Flash**: 1MB
- **SRAM**: 128KB
- **GPIO**: 114个I/O引脚

## 软件依赖

### 核心依赖
```toml
cortex-m = "0.7"           # Cortex-M处理器支持
cortex-m-rt = "0.7"        # 运行时支持
stm32f4xx-hal = "0.19"     # STM32F4 HAL库
embedded-hal = "1.0"       # 嵌入式HAL特征
```

### 调试工具
```toml
rtt-target = "0.4"         # RTT调试输出
defmt = "0.3"              # 高效日志格式
panic-halt = "0.2"         # 异常处理
```

## 项目结构

```
basic-led/
├── Cargo.toml              # 项目配置和依赖
├── memory.x                # 内存布局配置
├── .cargo/
│   └── config.toml         # 构建配置
├── src/
│   └── main.rs             # 主程序
└── README.md               # 项目文档
```

## 核心功能

### 1. GPIO配置
```rust
// 配置PC13引脚为推挽输出
let gpioc = dp.GPIOC.split();
let mut led = gpioc.pc13.into_push_pull_output();
```

### 2. 定时器配置
```rust
// 配置1Hz定时器
let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
timer.start(1.Hz()).unwrap();
```

### 3. LED控制
```rust
// LED状态切换
if led_state {
    led.set_low();   // 点亮LED
} else {
    led.set_high();  // 熄灭LED
}
```

## 快速开始

### 1. 环境准备
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf

# 安装调试工具
cargo install probe-rs --features cli
cargo install flip-link
```

### 2. 硬件连接
1. 使用USB线连接STM32F407VG Discovery到电脑
2. 确保ST-LINK驱动已正确安装
3. 板载LED位于PC13引脚，无需额外连线

### 3. 编译和运行
```bash
# 进入项目目录
cd 04-gpio-control/projects/basic-led

# 编译项目
cargo build

# 运行项目（需要连接硬件）
cargo run

# 编译发布版本
cargo build --release
```

### 4. 调试输出
项目使用RTT进行调试输出，运行时会显示：
```
🚀 基础LED控制示例启动
硬件平台: STM32F407VG Discovery
LED引脚: PC13 (板载LED)
⚡ 系统时钟配置完成: 168MHz
💡 GPIO配置完成，开始LED闪烁循环
📊 闪烁频率: 1Hz (每秒1次)
🔆 LED开启 - 计数: 0 - 时间戳: 0ms
🔅 LED关闭 - 计数: 1 - 时间戳: 1000ms
```

## 代码详解

### 主要组件

1. **系统初始化**
   - 外设访问权限获取
   - 时钟系统配置
   - GPIO端口初始化

2. **定时器配置**
   - 使用SysTick定时器
   - 配置1Hz频率
   - 非阻塞等待机制

3. **GPIO控制**
   - 推挽输出模式
   - 高低电平切换
   - 状态反馈

4. **调试支持**
   - RTT实时输出
   - 运行状态监控
   - 错误处理机制

### 关键概念

- **HAL抽象**: 使用硬件抽象层简化底层操作
- **类型安全**: 编译时确保GPIO配置正确
- **零成本抽象**: 高级API不增加运行时开销
- **内存安全**: 防止悬空指针和缓冲区溢出

## 扩展练习

### 初级练习
1. 修改LED闪烁频率（0.5Hz, 2Hz, 10Hz）
2. 添加第二个LED控制
3. 实现不同的闪烁模式（快闪、慢闪、呼吸灯）

### 中级练习
1. 使用外部中断控制LED
2. 实现PWM调光功能
3. 添加按键控制LED状态

### 高级练习
1. 实现多LED矩阵控制
2. 集成传感器数据显示
3. 添加无线通信控制

## 故障排除

### 常见问题

1. **编译错误**
   ```
   error: linker `rust-lld` not found
   ```
   **解决**: 安装完整的Rust工具链
   ```bash
   rustup component add llvm-tools-preview
   ```

2. **烧录失败**
   ```
   Error: Probe could not be opened
   ```
   **解决**: 检查硬件连接和驱动安装

3. **LED不闪烁**
   - 检查GPIO引脚配置
   - 确认定时器频率设置
   - 验证电路连接

### 调试技巧

1. **使用RTT输出**
   ```rust
   rprintln!("调试信息: {}", value);
   ```

2. **检查寄存器状态**
   ```rust
   rprintln!("GPIO状态: {:?}", gpioc.pc13);
   ```

3. **监控系统状态**
   ```rust
   rprintln!("时钟频率: {}Hz", clocks.sysclk().raw());
   ```

## 性能优化

### 编译优化
- 使用`opt-level = "s"`优化代码大小
- 启用LTO链接时优化
- 移除未使用的代码段

### 运行时优化
- 使用WFI指令降低功耗
- 合理配置时钟频率
- 优化中断处理程序

## 相关资源

### 官方文档
- [STM32F4xx HAL文档](https://docs.rs/stm32f4xx-hal/)
- [Embedded HAL特征](https://docs.rs/embedded-hal/)
- [Cortex-M文档](https://docs.rs/cortex-m/)

### 学习资源
- [Rust嵌入式书籍](https://doc.rust-lang.org/embedded-book/)
- [STM32F4参考手册](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [Discovery板用户手册](https://www.st.com/resource/en/user_manual/dm00039084.pdf)

### 社区支持
- [Rust嵌入式工作组](https://github.com/rust-embedded/wg)
- [STM32 Rust社区](https://github.com/stm32-rs)
- [嵌入式Rust论坛](https://users.rust-lang.org/c/embedded/)

## 下一步学习

完成本项目后，建议继续学习：
1. **GPIO输入控制** - 按键检测和防抖
2. **中断处理** - 外部中断和定时器中断
3. **串口通信** - UART数据传输
4. **ADC采集** - 模拟信号读取
5. **PWM输出** - 精确波形生成

---

*本项目是Rust嵌入式编程教程的一部分，更多内容请参考主教程目录。*
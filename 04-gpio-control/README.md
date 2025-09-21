# 第4章：GPIO控制与数字I/O

## 概述

GPIO（General Purpose Input/Output）是嵌入式系统的基础外设，是微控制器与外部世界交互的主要接口。本章深入探讨GPIO的工作原理、电气特性、编程模型，并通过丰富的实践项目掌握数字I/O控制技术。

GPIO不仅仅是简单的数字输入输出，它涉及电气工程、数字电路设计、实时系统编程等多个领域。我们将从硬件原理出发，结合Rust的类型安全特性，构建可靠、高效的GPIO控制系统。

## 学习目标

- **硬件理解**: 深入理解GPIO电路原理、电气特性和时序要求
- **软件抽象**: 掌握embedded-hal抽象层和平台特定HAL库的使用
- **实时控制**: 实现精确的时序控制和中断驱动编程
- **系统集成**: 构建复杂的GPIO应用系统和状态机
- **性能优化**: 掌握GPIO操作的性能优化技术
- **可靠性设计**: 实现防抖动、错误处理和故障恢复机制

## 章节内容

### 4.1 GPIO硬件原理 ⭐
- **电路结构**: 输出驱动器、输入缓冲器、保护电路
- **电气特性**: VOH/VOL、IIH/IIL、驱动能力、功耗分析
- **工作模式**: 推挽输出、开漏输出、输入模式、模拟模式
- **时序特性**: 建立时间、保持时间、传播延迟
- **实践项目**: [gpio_analysis](projects/gpio_analysis/)

### 4.2 HAL抽象层设计 ⭐
- **embedded-hal特征**: InputPin、OutputPin、StatefulOutputPin
- **类型安全设计**: 编译时状态检查、零成本抽象
- **平台适配**: STM32、ESP32、RP2040、nRF52 HAL库
- **错误处理**: Result类型、错误传播、故障恢复
- **实践项目**: [hal_abstraction](projects/hal_abstraction/)

### 4.3 数字输出控制
- **LED驱动**: 电流限制、亮度控制、多LED管理
- **继电器控制**: 驱动电路、反向保护、时序控制
- **数码管显示**: 段码驱动、动态扫描、亮度调节
- **状态指示**: 系统状态、错误指示、用户反馈
- **实践项目**: [digital_output](projects/digital_output/)

### 4.4 数字输入处理
- **按钮检测**: 电平检测、边沿检测、状态机设计
- **防抖动算法**: 软件防抖、硬件防抖、滤波算法
- **开关矩阵**: 键盘扫描、矩阵编码、优化算法
- **传感器接口**: 数字传感器、开关量输入、隔离设计
- **实践项目**: [digital_input](projects/digital_input/)

### 4.5 中断驱动编程
- **中断原理**: 中断向量、优先级、嵌套中断
- **外部中断**: EXTI配置、边沿触发、电平触发
- **中断处理**: ISR设计、数据共享、原子操作
- **实时响应**: 中断延迟、抖动分析、优化策略
- **实践项目**: [interrupt_driven](projects/interrupt_driven/)

### 4.6 PWM与模拟输出
- **PWM原理**: 占空比、频率、分辨率关系
- **硬件PWM**: 定时器配置、通道复用、死区控制
- **软件PWM**: 位带操作、DMA辅助、多通道同步
- **应用场景**: LED调光、电机控制、音频输出
- **实践项目**: [pwm_control](projects/pwm_control/)

### 4.7 高级GPIO技术
- **GPIO复用**: 功能复用、引脚映射、冲突解决
- **高速GPIO**: 时序优化、信号完整性、EMI控制
- **低功耗设计**: 功耗模式、唤醒源、功耗测量
- **可靠性设计**: ESD保护、过压保护、故障检测
- **实践项目**: [advanced_gpio](projects/advanced_gpio/)

### 4.8 系统集成与优化
- **状态机设计**: 有限状态机、层次状态机、状态转换
- **任务调度**: 协作式调度、抢占式调度、实时约束
- **性能分析**: 执行时间、内存使用、功耗分析
- **测试验证**: 单元测试、集成测试、硬件在环测试
- **实践项目**: [system_integration](projects/system_integration/)

## 支持的硬件平台

本章的示例代码支持以下硬件平台：

### STM32系列
- **STM32F4 Discovery**: 主控芯片STM32F407VGT6
- **STM32F411 Nucleo**: 主控芯片STM32F411RET6
- **STM32F103 Blue Pill**: 主控芯片STM32F103C8T6

### ESP32系列
- **ESP32 DevKit**: 主控芯片ESP32-WROOM-32
- **ESP32-S3**: 主控芯片ESP32-S3-WROOM-1
- **ESP32-C3**: 主控芯片ESP32-C3-WROOM-02

### Raspberry Pi Pico
- **Pico**: 主控芯片RP2040
- **Pico W**: 带WiFi的RP2040

### Nordic nRF系列
- **nRF52840 DK**: 主控芯片nRF52840
- **nRF52832**: 主控芯片nRF52832

## 项目结构

```
04-gpio-control/
├── projects/
│   ├── basic-led/              # 基础LED闪烁
│   ├── button-led/             # 按钮控制LED
│   ├── pwm-breathing/          # PWM呼吸灯
│   ├── traffic-light/          # 交通灯系统
│   ├── led-matrix/             # LED矩阵显示
│   ├── servo-control/          # 舵机控制
│   └── gpio-interrupt/         # GPIO中断示例
├── examples/
│   ├── stm32f4/               # STM32F4示例
│   ├── esp32/                 # ESP32示例
│   ├── rp2040/                # RP2040示例
│   └── nrf52/                 # nRF52示例
├── docs/
│   ├── schematics/            # 电路图
│   └── datasheets/            # 数据手册
└── README.md
```

## 硬件准备

### 基础组件
- 开发板（任选一种支持的平台）
- LED（红、绿、蓝各几个）
- 按钮开关（轻触开关）
- 电阻（220Ω、10kΩ）
- 面包板和跳线
- USB数据线

### 进阶组件（可选）
- RGB LED
- 舵机（SG90或类似）
- 蜂鸣器
- 光敏电阻
- 温度传感器
- LED矩阵模块

## 快速开始

### 1. 创建基础LED项目

```bash
# 进入GPIO控制目录
cd 04-gpio-control/projects

# 创建基础LED项目
cargo new --bin basic-led
cd basic-led
```

### 2. 配置项目依赖

编辑 `Cargo.toml`：

```toml
[package]
name = "basic-led"
version = "0.1.0"
edition = "2021"

[dependencies]
# 核心库
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"

# HAL库（根据你的硬件选择）
# STM32F4
stm32f4xx-hal = { version = "0.19", features = ["stm32f407"] }

# 或者 ESP32
# esp32-hal = "0.16"

# 或者 RP2040
# rp-pico = "0.8"

# 嵌入式HAL特征
embedded-hal = "1.0"
nb = "1.1"

# 调试输出
rtt-target = { version = "0.4", features = ["cortex-m"] }
```

### 3. 基础LED闪烁代码

创建 `src/main.rs`：

```rust
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    gpio::{Output, PushPull, Pin},
};
use rtt_target::{rprintln, rtt_init_print};

type LedPin = Pin<'C', 13, Output<PushPull>>;

#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("GPIO LED控制示例启动");
    
    // 获取外设访问权限
    let dp = pac::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    // 配置GPIO
    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();
    
    rprintln!("开始LED闪烁循环");
    
    let mut counter = 0u32;
    
    loop {
        // 点亮LED
        led.set_low();
        rprintln!("LED开启 - 计数: {}", counter);
        delay_ms(500);
        
        // 熄灭LED
        led.set_high();
        rprintln!("LED关闭 - 计数: {}", counter);
        delay_ms(500);
        
        counter = counter.wrapping_add(1);
    }
}

// 简单的延时函数（生产环境应使用定时器）
fn delay_ms(ms: u32) {
    for _ in 0..(ms * 1000) {
        cortex_m::asm::nop();
    }
}
```

### 4. 配置构建环境

创建 `.cargo/config.toml`：

```toml
[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32F407VGTx"
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[build]
target = "thumbv7em-none-eabihf"

[env]
DEFMT_LOG = "debug"
```

创建 `memory.x`：

```
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}
```

### 5. 编译和运行

```bash
# 编译项目
cargo build

# 烧录到硬件（需要连接开发板）
cargo run

# 查看RTT输出
probe-rs rtt --chip STM32F407VGTx
```

## 实战项目列表

### 初级项目
1. **基础LED闪烁** - 学习GPIO输出控制
2. **按钮控制LED** - 学习GPIO输入检测
3. **多LED流水灯** - 学习多GPIO协调控制
4. **呼吸灯效果** - 学习PWM输出

### 中级项目
5. **交通灯系统** - 状态机和定时控制
6. **按钮防抖动** - 输入信号处理
7. **RGB LED调色** - 多通道PWM控制
8. **舵机控制** - 精确PWM时序

### 高级项目
9. **LED矩阵显示** - 复杂GPIO驱动
10. **GPIO中断处理** - 异步事件响应
11. **低功耗GPIO** - 功耗优化技术
12. **GPIO复用配置** - 高级功能配置

## 电路连接指南

### 基础LED连接

```
MCU GPIO Pin ----[220Ω电阻]---- LED正极(长脚)
                                  |
                               LED负极(短脚)
                                  |
                                 GND
```

### 按钮连接（下拉配置）

```
3.3V ---- 按钮 ---- MCU GPIO Pin ---- [10kΩ电阻] ---- GND
```

### PWM LED连接

```
MCU PWM Pin ----[220Ω电阻]---- LED正极
                                 |
                              LED负极
                                 |
                                GND
```

## 调试技巧

### 1. 使用RTT调试输出

```rust
use rtt_target::{rprintln, rtt_init_print};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("程序启动");
    
    // 你的代码
    rprintln!("GPIO状态: {}", gpio_state);
}
```

### 2. 使用probe-rs调试

```bash
# 启动调试会话
probe-rs debug --chip STM32F407VGTx target/thumbv7em-none-eabihf/debug/basic-led

# 在另一个终端查看RTT输出
probe-rs rtt --chip STM32F407VGTx
```

### 3. 逻辑分析仪

对于复杂的GPIO时序，建议使用逻辑分析仪：
- 连接探头到GPIO引脚
- 设置合适的采样率
- 分析信号时序和状态变化

## 性能优化

### 1. GPIO操作优化

```rust
// 避免频繁的HAL调用
// 不好的做法
for _ in 0..1000 {
    led.set_high();
    led.set_low();
}

// 更好的做法
let gpio_reg = &dp.GPIOC;
for _ in 0..1000 {
    gpio_reg.bsrr.write(|w| w.bs13().set_bit());
    gpio_reg.bsrr.write(|w| w.br13().set_bit());
}
```

### 2. 批量GPIO操作

```rust
// 同时控制多个GPIO
gpio_reg.odr.modify(|_, w| {
    w.odr13().bit(true)
     .odr14().bit(false)
     .odr15().bit(true)
});
```

## 故障排除

### 常见问题

1. **LED不亮**
   - 检查电路连接
   - 确认GPIO配置正确
   - 检查电阻值
   - 验证电源供应

2. **按钮无响应**
   - 检查上拉/下拉配置
   - 确认按钮连接
   - 添加防抖动处理

3. **PWM无效果**
   - 确认PWM通道配置
   - 检查频率和占空比设置
   - 验证GPIO复用配置

### 调试检查清单

- [ ] 硬件连接正确
- [ ] GPIO引脚配置正确
- [ ] 时钟配置正确
- [ ] 电源供应稳定
- [ ] 代码逻辑正确
- [ ] 寄存器配置正确

## 扩展学习

完成基础GPIO控制后，可以继续学习：

1. **定时器和中断** - 精确的时序控制
2. **串口通信** - 与外部设备通信
3. **I2C/SPI** - 复杂外设控制
4. **ADC/DAC** - 模拟信号处理

## 参考资源

- [embedded-hal文档](https://docs.rs/embedded-hal/)
- [STM32F4xx HAL文档](https://docs.rs/stm32f4xx-hal/)
- [GPIO原理详解](https://en.wikipedia.org/wiki/General-purpose_input/output)
- [PWM原理介绍](https://en.wikipedia.org/wiki/Pulse-width_modulation)

---

**准备好开始你的GPIO控制之旅了吗？让我们从最简单的LED闪烁开始！** 💡
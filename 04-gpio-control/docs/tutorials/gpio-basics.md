# GPIO基础教程

## 目录
- [GPIO简介](#gpio简介)
- [GPIO工作模式](#gpio工作模式)
- [电气特性](#电气特性)
- [编程基础](#编程基础)
- [常见问题](#常见问题)
- [实践项目](#实践项目)

## GPIO简介

### 什么是GPIO？
GPIO (General Purpose Input/Output) 是通用输入输出端口，是微控制器与外部世界交互的基本接口。每个GPIO引脚都可以配置为输入或输出模式，用于读取外部信号或控制外部设备。

### GPIO的重要性
- **数字接口**: 提供数字信号的输入输出功能
- **灵活配置**: 可根据需要配置为不同工作模式
- **实时控制**: 支持实时的信号处理和控制
- **扩展能力**: 通过GPIO可以连接各种外设

## GPIO工作模式

### 1. 输出模式 (Output Mode)

#### 推挽输出 (Push-Pull Output)
```
VCC ----[PMOS]---- GPIO引脚
                     |
GND ----[NMOS]---- GPIO引脚
```

**特点:**
- 可以输出强高电平和强低电平
- 驱动能力强，适合驱动LED等负载
- 功耗相对较高

**应用场景:**
- LED控制
- 继电器驱动
- 数字信号输出

#### 开漏输出 (Open-Drain Output)
```
VCC ----[上拉电阻]---- GPIO引脚
                         |
GND ----[NMOS]-------- GPIO引脚
```

**特点:**
- 只能输出低电平，高电平需要外部上拉
- 可以实现线与逻辑
- 支持不同电压电平转换

**应用场景:**
- I2C总线
- 多设备共享信号线
- 电平转换

### 2. 输入模式 (Input Mode)

#### 浮空输入 (Floating Input)
```
外部信号 ---- GPIO引脚 ---- 输入缓冲器
```

**特点:**
- 引脚呈高阻态
- 容易受到干扰
- 功耗最低

**注意事项:**
- 必须有外部上拉或下拉电阻
- 避免引脚悬空

#### 上拉输入 (Pull-Up Input)
```
VCC ----[内部上拉]---- GPIO引脚 ---- 输入缓冲器
                         |
                    外部信号
```

**特点:**
- 内部上拉电阻约10kΩ-100kΩ
- 默认状态为高电平
- 适合按钮等开关输入

#### 下拉输入 (Pull-Down Input)
```
外部信号 ---- GPIO引脚 ---- 输入缓冲器
                |
GND ----[内部下拉]
```

**特点:**
- 内部下拉电阻约10kΩ-100kΩ
- 默认状态为低电平
- 较少使用

### 3. 复用功能模式 (Alternate Function)

GPIO引脚可以复用为其他外设功能：
- **UART**: TX/RX
- **SPI**: MOSI/MISO/SCK/CS
- **I2C**: SDA/SCL
- **PWM**: 定时器输出
- **ADC**: 模拟输入

## 电气特性

### STM32F4系列典型参数

| 参数 | 最小值 | 典型值 | 最大值 | 单位 |
|------|--------|--------|--------|------|
| 工作电压 | 2.0 | 3.3 | 3.6 | V |
| 输入高电平 | 2.0 | - | VDD+0.3 | V |
| 输入低电平 | -0.3 | - | 1.3 | V |
| 输出高电平 | VDD-0.4 | - | VDD | V |
| 输出低电平 | 0 | - | 0.4 | V |
| 最大输出电流 | - | - | 25 | mA |
| 最大灌电流 | - | - | 25 | mA |

### 电流限制
- **单个引脚**: 最大25mA
- **所有引脚总和**: 最大120mA
- **电源引脚**: 最大150mA

### 上拉/下拉电阻
- **内部上拉**: 30kΩ - 50kΩ
- **内部下拉**: 30kΩ - 50kΩ

## 编程基础

### 1. GPIO初始化

```rust
use stm32f4xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull, Input, PullUp},
    prelude::*,
};

// 输出模式初始化
let gpioc = dp.GPIOC.split();
let mut led = gpioc.pc13.into_push_pull_output();

// 输入模式初始化
let button = gpioc.pc14.into_pull_up_input();
```

### 2. GPIO操作

```rust
// 输出控制
led.set_high();  // 输出高电平
led.set_low();   // 输出低电平
led.toggle();    // 电平翻转

// 输入读取
let button_state = button.is_high();
if button.is_low() {
    // 按钮被按下
}
```

### 3. 中断配置

```rust
use stm32f4xx_hal::interrupt;

// 配置外部中断
let mut syscfg = dp.SYSCFG.constrain();
let mut exti = dp.EXTI;

button.make_interrupt_source(&mut syscfg);
button.trigger_on_edge(&mut exti, Edge::Falling);
button.enable_interrupt(&mut exti);

// 中断处理函数
#[interrupt]
fn EXTI15_10() {
    // 处理中断
    cortex_m::interrupt::free(|cs| {
        // 清除中断标志
        BUTTON.borrow(cs).borrow_mut().as_mut().unwrap().clear_interrupt_pending_bit();
        // 处理按钮事件
    });
}
```

## 常见问题

### 1. 引脚悬空问题
**问题**: 输入引脚没有连接或上拉/下拉，导致读取值不稳定。

**解决方案**:
```rust
// 使用内部上拉
let button = gpioc.pc14.into_pull_up_input();

// 或使用外部上拉电阻 (10kΩ)
```

### 2. 电流过载
**问题**: GPIO输出电流超过最大限制，导致电压下降或芯片损坏。

**解决方案**:
```rust
// 使用限流电阻
// LED电路: GPIO -> 330Ω电阻 -> LED -> GND

// 大功率负载使用MOSFET驱动
```

### 3. 电平不匹配
**问题**: 3.3V系统与5V设备接口。

**解决方案**:
```rust
// 使用电平转换器
// 或使用开漏输出 + 5V上拉
let pin = gpioc.pc13.into_open_drain_output();
```

### 4. 中断抖动
**问题**: 机械按钮产生多次中断。

**解决方案**:
```rust
// 软件去抖动
static mut LAST_PRESS: u32 = 0;

#[interrupt]
fn EXTI15_10() {
    let now = get_time_ms();
    if now - unsafe { LAST_PRESS } > 50 {  // 50ms去抖动
        unsafe { LAST_PRESS = now; }
        // 处理按钮事件
    }
}
```

## 实践项目

### 项目1: LED闪烁
**目标**: 控制LED以1Hz频率闪烁

**电路**:
```
GPIO -> 330Ω -> LED -> GND
```

**代码要点**:
```rust
loop {
    led.set_high();
    delay.delay_ms(500u32);
    led.set_low();
    delay.delay_ms(500u32);
}
```

### 项目2: 按钮控制LED
**目标**: 按钮按下时LED亮，松开时LED灭

**电路**:
```
3.3V -> 10kΩ -> GPIO (按钮输入)
              -> 按钮 -> GND

GPIO -> 330Ω -> LED -> GND
```

**代码要点**:
```rust
loop {
    if button.is_low() {
        led.set_high();
    } else {
        led.set_low();
    }
}
```

### 项目3: PWM呼吸灯
**目标**: LED亮度呈正弦波变化

**代码要点**:
```rust
let mut pwm = tim2.pwm_hz(channels, 1.kHz(), &clocks);
let max_duty = pwm.get_max_duty();

loop {
    for i in 0..=100 {
        let duty = (max_duty as f32 * (i as f32 / 100.0).sin()) as u16;
        pwm.set_duty(Channel::C1, duty);
        delay.delay_ms(20u32);
    }
}
```

### 项目4: 多路LED控制
**目标**: 控制8个LED实现流水灯效果

**代码要点**:
```rust
let mut leds = [
    gpioc.pc0.into_push_pull_output().erase(),
    gpioc.pc1.into_push_pull_output().erase(),
    // ... 更多LED
];

loop {
    for led in leds.iter_mut() {
        led.set_high();
        delay.delay_ms(100u32);
        led.set_low();
    }
}
```

## 进阶主题

### 1. GPIO速度配置
```rust
use stm32f4xx_hal::gpio::Speed;

// 不同速度配置
let pin = gpioc.pc13.into_push_pull_output_with_speed(Speed::VeryHigh);
```

**速度等级**:
- **Low**: 8MHz，低功耗
- **Medium**: 50MHz，平衡
- **High**: 100MHz，高性能
- **VeryHigh**: 180MHz，最高性能

### 2. GPIO驱动强度
```rust
// 不同驱动强度配置影响输出电流能力
// 需要根据负载选择合适的驱动强度
```

### 3. 模拟开关
```rust
// 使用GPIO实现模拟开关功能
// 通过控制MOSFET实现信号通路切换
```

## 总结

GPIO是嵌入式系统的基础接口，掌握GPIO的正确使用方法对于嵌入式开发至关重要。本教程涵盖了GPIO的基本概念、工作模式、电气特性和编程方法，通过实践项目可以加深理解。

**关键要点**:
1. 正确配置GPIO工作模式
2. 注意电气特性限制
3. 合理使用上拉/下拉电阻
4. 实现软件去抖动
5. 选择合适的驱动方案

继续学习更多高级主题，如中断处理、DMA传输、低功耗设计等，将帮助你成为更优秀的嵌入式开发者。
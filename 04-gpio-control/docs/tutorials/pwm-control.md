# PWM控制教程

## 目录
- [PWM基础概念](#pwm基础概念)
- [PWM工作原理](#pwm工作原理)
- [硬件实现](#硬件实现)
- [软件编程](#软件编程)
- [应用实例](#应用实例)
- [高级技巧](#高级技巧)
- [故障排除](#故障排除)

## PWM基础概念

### 什么是PWM？
PWM (Pulse Width Modulation) 脉冲宽度调制是一种通过改变脉冲宽度来控制平均功率的技术。通过快速开关信号，可以模拟模拟信号的效果。

### PWM的关键参数

#### 1. 频率 (Frequency)
```
频率 = 1 / 周期时间
```
- **定义**: 每秒钟脉冲的次数
- **单位**: Hz (赫兹)
- **影响**: 决定开关速度和滤波要求

#### 2. 占空比 (Duty Cycle)
```
占空比 = (高电平时间 / 周期时间) × 100%
```
- **定义**: 高电平时间占整个周期的百分比
- **范围**: 0% - 100%
- **作用**: 控制平均输出功率

#### 3. 分辨率 (Resolution)
```
分辨率 = log₂(最大计数值)
```
- **定义**: 占空比的最小调节步长
- **常见值**: 8位(256级)、10位(1024级)、16位(65536级)

### PWM波形分析

```
     ┌─────┐     ┌─────┐     ┌─────┐
     │     │     │     │     │     │
   ──┘     └─────┘     └─────┘     └──
     
     ├─ Ton ─┤
     ├────── T ──────┤
     
占空比 = Ton / T × 100%
平均电压 = VCC × 占空比
```

## PWM工作原理

### 1. 时间域分析

**50%占空比示例**:
```
时间:  0  1  2  3  4  5  6  7  8  ms
信号:  ┌──┐  ┌──┐  ┌──┐  ┌──┐
      ─┘  └──┘  └──┘  └──┘  └──
平均值: 1.65V (3.3V × 50%)
```

**25%占空比示例**:
```
时间:  0  1  2  3  4  5  6  7  8  ms
信号:  ┌┐    ┌┐    ┌┐    ┌┐
      ─┘└────┘└────┘└────┘└────
平均值: 0.825V (3.3V × 25%)
```

### 2. 频域分析

PWM信号包含：
- **直流分量**: 平均值
- **基波**: PWM频率
- **谐波**: 基波的整数倍

**滤波要求**:
```rust
// 低通滤波器设计
// 截止频率 fc << PWM频率
// RC = 1 / (2π × fc)

// 例如: PWM频率 = 10kHz, fc = 100Hz
// R = 1kΩ, C = 1.6μF
```

## 硬件实现

### 1. 定时器PWM模式

#### STM32定时器结构
```
时钟源 -> 预分频器 -> 计数器 -> 比较器 -> PWM输出
         (PSC)      (ARR)    (CCR)
```

**关键寄存器**:
- **PSC**: 预分频值，决定计数时钟
- **ARR**: 自动重载值，决定PWM周期
- **CCR**: 比较值，决定占空比

#### 频率计算
```
PWM频率 = 时钟频率 / ((PSC + 1) × (ARR + 1))

例如:
- 系统时钟: 84MHz
- PSC = 83 (84分频)
- ARR = 999 (1000计数)
- PWM频率 = 84MHz / (84 × 1000) = 1kHz
```

#### 占空比计算
```
占空比 = CCR / (ARR + 1) × 100%

例如:
- ARR = 999
- CCR = 500
- 占空比 = 500 / 1000 × 100% = 50%
```

### 2. PWM输出模式

#### 模式1 (PWM Mode 1)
```
CCR < CNT: 输出低电平
CCR ≥ CNT: 输出高电平
```

#### 模式2 (PWM Mode 2)
```
CCR < CNT: 输出高电平
CCR ≥ CNT: 输出低电平
```

### 3. 多通道PWM

```rust
// 4通道PWM配置
let channels = (
    gpioa.pa8.into_alternate(),  // TIM1_CH1
    gpioa.pa9.into_alternate(),  // TIM1_CH2
    gpioa.pa10.into_alternate(), // TIM1_CH3
    gpioa.pa11.into_alternate(), // TIM1_CH4
);

let mut pwm = tim1.pwm_hz(channels, 1.kHz(), &clocks);
```

## 软件编程

### 1. 基础PWM配置

```rust
use stm32f4xx_hal::{
    pac,
    prelude::*,
    timer::{Channel, Timer},
};

fn setup_pwm() -> PwmChannels<TIM2, (C1, C2, C3, C4)> {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    
    let gpioa = dp.GPIOA.split();
    
    // 配置PWM引脚
    let channels = (
        gpioa.pa0.into_alternate(),  // TIM2_CH1
        gpioa.pa1.into_alternate(),  // TIM2_CH2
        gpioa.pa2.into_alternate(),  // TIM2_CH3
        gpioa.pa3.into_alternate(),  // TIM2_CH4
    );
    
    // 创建PWM实例
    let mut pwm = Timer::new(dp.TIM2, &clocks)
        .pwm_hz(channels, 1.kHz());
    
    // 启用所有通道
    pwm.enable(Channel::C1);
    pwm.enable(Channel::C2);
    pwm.enable(Channel::C3);
    pwm.enable(Channel::C4);
    
    pwm
}
```

### 2. 动态占空比控制

```rust
fn breathing_led_effect(pwm: &mut PwmChannels<TIM2, C1>) {
    let max_duty = pwm.get_max_duty();
    
    // 正弦波呼吸效果
    for i in 0..360 {
        let angle = i as f32 * 3.14159 / 180.0;
        let duty = ((angle.sin() + 1.0) / 2.0 * max_duty as f32) as u16;
        
        pwm.set_duty(Channel::C1, duty);
        delay_ms(10);
    }
}

fn sawtooth_wave(pwm: &mut PwmChannels<TIM2, C1>) {
    let max_duty = pwm.get_max_duty();
    
    // 锯齿波效果
    for duty in 0..=max_duty {
        pwm.set_duty(Channel::C1, duty);
        delay_ms(1);
    }
}

fn triangle_wave(pwm: &mut PwmChannels<TIM2, C1>) {
    let max_duty = pwm.get_max_duty();
    
    // 三角波效果
    // 上升沿
    for duty in 0..=max_duty {
        pwm.set_duty(Channel::C1, duty);
        delay_ms(1);
    }
    
    // 下降沿
    for duty in (0..=max_duty).rev() {
        pwm.set_duty(Channel::C1, duty);
        delay_ms(1);
    }
}
```

### 3. 多通道协调控制

```rust
fn rgb_color_mixing(pwm: &mut PwmChannels<TIM2, (C1, C2, C3)>) {
    let max_duty = pwm.get_max_duty();
    
    // RGB颜色循环
    let colors = [
        (max_duty, 0, 0),           // 红色
        (max_duty, max_duty/2, 0),  // 橙色
        (max_duty, max_duty, 0),    // 黄色
        (0, max_duty, 0),           // 绿色
        (0, 0, max_duty),           // 蓝色
        (max_duty/2, 0, max_duty),  // 紫色
    ];
    
    for (r, g, b) in colors.iter() {
        pwm.set_duty(Channel::C1, *r);  // 红色通道
        pwm.set_duty(Channel::C2, *g);  // 绿色通道
        pwm.set_duty(Channel::C3, *b);  // 蓝色通道
        delay_ms(1000);
    }
}

fn phase_shifted_outputs(pwm: &mut PwmChannels<TIM2, (C1, C2, C3, C4)>) {
    let max_duty = pwm.get_max_duty();
    
    // 四相位输出 (相位差90度)
    for i in 0..360 {
        let angle = i as f32 * 3.14159 / 180.0;
        
        let duty1 = ((angle.sin() + 1.0) / 2.0 * max_duty as f32) as u16;
        let duty2 = (((angle + 3.14159/2.0).sin() + 1.0) / 2.0 * max_duty as f32) as u16;
        let duty3 = (((angle + 3.14159).sin() + 1.0) / 2.0 * max_duty as f32) as u16;
        let duty4 = (((angle + 3.14159*1.5).sin() + 1.0) / 2.0 * max_duty as f32) as u16;
        
        pwm.set_duty(Channel::C1, duty1);
        pwm.set_duty(Channel::C2, duty2);
        pwm.set_duty(Channel::C3, duty3);
        pwm.set_duty(Channel::C4, duty4);
        
        delay_ms(10);
    }
}
```

## 应用实例

### 1. LED亮度控制

```rust
struct LedController {
    pwm: PwmChannels<TIM2, C1>,
    current_brightness: u16,
    target_brightness: u16,
}

impl LedController {
    fn new(pwm: PwmChannels<TIM2, C1>) -> Self {
        Self {
            pwm,
            current_brightness: 0,
            target_brightness: 0,
        }
    }
    
    fn set_brightness(&mut self, brightness: u8) {
        let max_duty = self.pwm.get_max_duty();
        self.target_brightness = (brightness as u32 * max_duty as u32 / 255) as u16;
    }
    
    fn update(&mut self) {
        // 平滑过渡到目标亮度
        if self.current_brightness < self.target_brightness {
            self.current_brightness += 1;
        } else if self.current_brightness > self.target_brightness {
            self.current_brightness -= 1;
        }
        
        self.pwm.set_duty(Channel::C1, self.current_brightness);
    }
    
    fn fade_in(&mut self, duration_ms: u32) {
        let max_duty = self.pwm.get_max_duty();
        let steps = duration_ms / 10; // 10ms per step
        let step_size = max_duty / steps as u16;
        
        for i in 0..steps {
            let duty = (i as u16 * step_size).min(max_duty);
            self.pwm.set_duty(Channel::C1, duty);
            delay_ms(10);
        }
    }
    
    fn fade_out(&mut self, duration_ms: u32) {
        let max_duty = self.pwm.get_max_duty();
        let steps = duration_ms / 10;
        let step_size = max_duty / steps as u16;
        
        for i in 0..steps {
            let duty = max_duty - (i as u16 * step_size).min(max_duty);
            self.pwm.set_duty(Channel::C1, duty);
            delay_ms(10);
        }
    }
}
```

### 2. 舵机控制

```rust
struct ServoController {
    pwm: PwmChannels<TIM2, C1>,
    min_pulse: u16,
    max_pulse: u16,
    current_angle: f32,
}

impl ServoController {
    fn new(pwm: PwmChannels<TIM2, C1>) -> Self {
        // 标准舵机: 1ms-2ms脉宽，20ms周期
        // 假设PWM频率为50Hz (20ms周期)
        let max_duty = pwm.get_max_duty();
        
        Self {
            pwm,
            min_pulse: max_duty / 20,      // 1ms (5%)
            max_pulse: max_duty / 10,      // 2ms (10%)
            current_angle: 90.0,
        }
    }
    
    fn set_angle(&mut self, angle: f32) {
        // 角度范围: 0-180度
        let angle = angle.clamp(0.0, 180.0);
        
        // 计算脉宽
        let pulse_width = self.min_pulse + 
            ((angle / 180.0) * (self.max_pulse - self.min_pulse) as f32) as u16;
        
        self.pwm.set_duty(Channel::C1, pulse_width);
        self.current_angle = angle;
    }
    
    fn sweep(&mut self, start_angle: f32, end_angle: f32, duration_ms: u32) {
        let steps = duration_ms / 20; // 20ms per step
        let angle_step = (end_angle - start_angle) / steps as f32;
        
        for i in 0..=steps {
            let angle = start_angle + (i as f32 * angle_step);
            self.set_angle(angle);
            delay_ms(20);
        }
    }
}
```

### 3. 电机速度控制

```rust
struct MotorController {
    pwm: PwmChannels<TIM2, C1>,
    direction_pin: Pin<'A', 1, Output<PushPull>>,
    current_speed: i16, // -100 to 100
}

impl MotorController {
    fn new(
        pwm: PwmChannels<TIM2, C1>,
        direction_pin: Pin<'A', 1, Output<PushPull>>
    ) -> Self {
        Self {
            pwm,
            direction_pin,
            current_speed: 0,
        }
    }
    
    fn set_speed(&mut self, speed: i16) {
        // 速度范围: -100 to 100
        let speed = speed.clamp(-100, 100);
        
        // 设置方向
        if speed >= 0 {
            self.direction_pin.set_high();
        } else {
            self.direction_pin.set_low();
        }
        
        // 设置PWM占空比
        let max_duty = self.pwm.get_max_duty();
        let duty = (speed.abs() as u32 * max_duty as u32 / 100) as u16;
        
        self.pwm.set_duty(Channel::C1, duty);
        self.current_speed = speed;
    }
    
    fn brake(&mut self) {
        self.pwm.set_duty(Channel::C1, 0);
        self.current_speed = 0;
    }
    
    fn gradual_speed_change(&mut self, target_speed: i16, duration_ms: u32) {
        let steps = duration_ms / 10;
        let speed_step = (target_speed - self.current_speed) as f32 / steps as f32;
        
        for i in 0..=steps {
            let speed = self.current_speed as f32 + (i as f32 * speed_step);
            self.set_speed(speed as i16);
            delay_ms(10);
        }
    }
}
```

## 高级技巧

### 1. 死区时间控制

```rust
// 用于H桥电机驱动，防止上下管同时导通
fn setup_complementary_pwm() {
    // 配置互补PWM输出
    // TIM1_CH1 和 TIM1_CH1N
    // 自动插入死区时间
    
    let dead_time = 100; // 死区时间 (时钟周期)
    // 在寄存器级别配置死区时间
}
```

### 2. PWM输入捕获

```rust
// 测量外部PWM信号的频率和占空比
fn measure_pwm_input() {
    // 配置定时器为PWM输入模式
    // 同时捕获上升沿和下降沿
    // 计算周期和占空比
}
```

### 3. 中心对齐PWM

```rust
// 中心对齐模式，减少EMI干扰
fn setup_center_aligned_pwm() {
    // 计数器在0和ARR之间上下计数
    // PWM波形关于中心对称
}
```

### 4. 同步多定时器

```rust
// 多个定时器同步启动
fn synchronize_timers() {
    // 使用主从模式
    // 一个定时器作为主定时器
    // 其他定时器作为从定时器
}
```

## 故障排除

### 1. PWM频率不正确

**问题**: 测量的PWM频率与设置不符

**检查项目**:
```rust
// 1. 检查时钟配置
let clocks = rcc.cfgr
    .hclk(84.MHz())
    .pclk1(42.MHz())
    .pclk2(84.MHz())
    .freeze();

// 2. 验证预分频和重载值
let expected_freq = timer_clock / ((psc + 1) * (arr + 1));

// 3. 检查定时器时钟源
// APB1定时器: TIM2, TIM3, TIM4, TIM5
// APB2定时器: TIM1, TIM8, TIM9, TIM10, TIM11
```

### 2. 占空比不准确

**问题**: 设置50%占空比，但测量结果不是50%

**解决方案**:
```rust
// 1. 检查CCR值计算
let duty_50_percent = max_duty / 2;

// 2. 考虑定时器分辨率
// 8位: 256级 (0.39%步进)
// 10位: 1024级 (0.098%步进)
// 16位: 65536级 (0.0015%步进)

// 3. 验证输出模式
// PWM Mode 1: CCR < CNT时输出低电平
// PWM Mode 2: CCR < CNT时输出高电平
```

### 3. PWM输出无信号

**问题**: 配置完成但引脚无PWM输出

**检查清单**:
```rust
// 1. 引脚复用配置
let pin = gpioa.pa8.into_alternate::<1>(); // TIM1_CH1

// 2. 定时器使能
pwm.enable(Channel::C1);

// 3. 时钟使能
// 通常由HAL自动处理

// 4. 占空比设置
pwm.set_duty(Channel::C1, max_duty / 2); // 不能为0

// 5. 输出比较使能
// 通常由HAL自动处理
```

### 4. PWM抖动或不稳定

**问题**: PWM输出不稳定，有抖动

**可能原因**:
```rust
// 1. 中断干扰
// 关闭不必要的中断或提高PWM定时器优先级

// 2. 时钟不稳定
// 使用外部晶振而非内部RC振荡器

// 3. 电源噪声
// 改善电源滤波

// 4. 负载变化
// 使用缓冲器或驱动器
```

## 性能优化

### 1. 减少CPU占用

```rust
// 使用DMA更新PWM占空比
fn setup_pwm_dma() {
    // 配置DMA从内存传输到CCR寄存器
    // 实现无CPU干预的PWM更新
}
```

### 2. 提高精度

```rust
// 使用高分辨率定时器
fn high_resolution_pwm() {
    // 使用系统最高时钟
    // 选择合适的预分频值
    // 平衡频率和分辨率
}
```

### 3. 降低功耗

```rust
// 动态调整PWM频率
fn adaptive_pwm_frequency() {
    // 根据应用需求调整频率
    // 低速应用使用低频率
    // 减少开关损耗
}
```

## 总结

PWM是嵌入式系统中非常重要的技术，广泛应用于电机控制、LED调光、音频生成等领域。掌握PWM的原理和编程技巧，能够实现精确的模拟控制效果。

**关键要点**:
1. 理解PWM的基本参数：频率、占空比、分辨率
2. 正确配置定时器和引脚
3. 根据应用选择合适的PWM频率
4. 注意电气特性和负载匹配
5. 使用高级功能提升性能

通过本教程的学习和实践，你应该能够熟练使用PWM技术实现各种控制应用。
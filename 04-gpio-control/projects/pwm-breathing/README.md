# PWM呼吸灯效果

这个项目演示了如何使用PWM（脉宽调制）技术实现平滑的呼吸灯效果。通过控制PWM的占空比，可以实现LED亮度的连续变化，创造出自然的呼吸效果。

## 功能特性

### 🌟 基础功能
- **PWM控制**: 使用硬件PWM生成精确的脉宽调制信号
- **呼吸效果**: 实现平滑的亮度渐变，模拟呼吸节奏
- **多种波形**: 支持线性、正弦波、指数等不同的亮度变化曲线
- **可配置参数**: 可调节呼吸周期、步数、频率等参数

### 🎨 高级功能
- **多通道控制**: 支持RGB LED的独立通道控制
- **多种效果**: 单色呼吸、颜色循环、彩虹效果、脉冲效果
- **实时调试**: RTT输出详细的状态信息
- **性能优化**: 高效的数学计算和内存使用

## 硬件连接

### 基础连接（单色LED）
```
STM32F407 PA8 ----[220Ω电阻]---- LED正极(长脚)
                                    |
                                 LED负极(短脚)
                                    |
                                   GND
```

### RGB LED连接
```
STM32F407 PA8 ----[220Ω]---- RGB LED Red
STM32F407 PA9 ----[220Ω]---- RGB LED Green  
STM32F407 PA10----[220Ω]---- RGB LED Blue
                               |
                            RGB LED GND
```

## 技术原理

### PWM基础
PWM通过快速开关信号来控制平均功率输出：
- **频率**: 开关的速度（本项目使用1kHz）
- **占空比**: 高电平时间占总周期的比例
- **分辨率**: 占空比的精度（取决于定时器配置）

### 呼吸算法
项目实现了三种呼吸波形：

1. **线性波形**: 亮度线性变化
   ```
   brightness = step / max_steps
   ```

2. **正弦波形**: 平滑的正弦曲线变化
   ```
   brightness = (sin(2π * step / max_steps) + 1) / 2
   ```

3. **指数波形**: 指数曲线变化，更自然
   ```
   brightness = (step / max_steps)²
   ```

## 代码结构

### 主要组件

1. **BreathingController**: 核心呼吸控制器
   - 管理呼吸状态和方向
   - 计算当前亮度值
   - 支持多种波形算法

2. **AdvancedBreathingController**: 高级控制器
   - 支持多通道RGB控制
   - 实现多种视觉效果
   - 效果切换和管理

3. **PWM工具函数**: 辅助计算函数
   - 最优预分频器计算
   - 占空比和百分比转换
   - 参数验证

### 关键参数

```rust
const PWM_FREQUENCY: u32 = 1000;        // PWM频率 1kHz
const BREATHING_PERIOD_MS: u32 = 2000;  // 呼吸周期 2秒
const STEPS_PER_CYCLE: u32 = 100;       // 每周期步数
```

## 编译和运行

### 1. 编译项目
```bash
cd 04-gpio-control/projects/pwm-breathing
cargo build
```

### 2. 烧录到硬件
```bash
cargo run
```

### 3. 查看调试输出
```bash
probe-rs rtt --chip STM32F407VGTx
```

## 调试输出示例

```
PWM呼吸灯效果启动
系统时钟配置完成: SYSCLK=168MHz
PWM配置完成: 频率=1000Hz, 最大占空比=999
开始呼吸灯循环
周期: 0, 步骤: 0/100, 占空比: 0/999 (0%)
周期: 0, 步骤: 10/100, 占空比: 95/999 (9%)
周期: 0, 步骤: 20/100, 占空比: 345/999 (34%)
...
完成第1个呼吸周期
```

## 扩展应用

### 1. RGB呼吸灯
修改代码支持三通道PWM，实现彩色呼吸效果：

```rust
// 配置三个PWM通道
let pwm_r = gpioa.pa8.into_alternate();  // Red
let pwm_g = gpioa.pa9.into_alternate();  // Green  
let pwm_b = gpioa.pa10.into_alternate(); // Blue
```

### 2. 音乐律动
结合ADC采集音频信号，实现音乐律动灯效：

```rust
let audio_level = adc.read_audio_input();
let brightness = calculate_brightness_from_audio(audio_level);
```

### 3. 环境感应
结合光敏传感器，实现自适应亮度调节：

```rust
let ambient_light = adc.read_light_sensor();
let max_brightness = calculate_max_brightness(ambient_light);
```

## 性能优化

### 1. 数学计算优化
- 使用查找表替代实时三角函数计算
- 预计算常用数值
- 使用定点数学库

### 2. 内存优化
- 避免动态内存分配
- 使用栈上数组
- 优化数据结构大小

### 3. 实时性优化
- 使用DMA传输PWM数据
- 中断驱动的状态更新
- 优先级调度

## 故障排除

### 常见问题

1. **LED不亮**
   - 检查PWM引脚配置
   - 确认GPIO复用设置
   - 验证电路连接

2. **呼吸效果不平滑**
   - 增加步数（STEPS_PER_CYCLE）
   - 调整PWM频率
   - 检查延时精度

3. **频率不正确**
   - 验证时钟配置
   - 检查预分频器计算
   - 确认定时器设置

### 调试技巧

1. **使用示波器**
   - 测量PWM信号频率
   - 检查占空比变化
   - 分析信号质量

2. **RTT调试输出**
   - 监控内部状态
   - 跟踪参数变化
   - 性能分析

3. **逻辑分析仪**
   - 多通道信号分析
   - 时序关系验证
   - 协议解析

## 相关资源

- [STM32F4 PWM配置指南](https://www.st.com/resource/en/application_note/dm00042534.pdf)
- [PWM原理详解](https://en.wikipedia.org/wiki/Pulse-width_modulation)
- [embedded-hal PWM特征](https://docs.rs/embedded-hal/latest/embedded_hal/pwm/index.html)

---

**享受你的PWM呼吸灯之旅！** ✨
# PWM控制示例

本目录包含了使用STM32定时器生成PWM信号控制各种设备的示例代码。

## 示例列表

### 1. LED调光器 (led_dimmer.rs)
- **功能**: 使用PWM控制LED亮度
- **特性**:
  - 渐亮渐暗效果
  - 亮度百分比控制
  - 呼吸灯效果
  - 平滑渐变功能

**硬件连接**:
- PA8 -> LED正极 (通过限流电阻)
- GND -> LED负极

**运行方式**:
```bash
cargo run --bin led_dimmer
```

### 2. 舵机控制 (servo_control.rs)
- **功能**: 使用PWM控制舵机角度
- **特性**:
  - 0-180度角度控制
  - 平滑移动功能
  - 扫描动作
  - 多舵机控制支持

**硬件连接**:
- PA8 -> 舵机信号线 (橙色/黄色)
- 5V -> 舵机电源线 (红色)
- GND -> 舵机地线 (棕色/黑色)

**运行方式**:
```bash
cargo run --bin servo_control
```

### 3. 电机调速 (motor_speed.rs)
- **功能**: 使用PWM控制直流电机速度
- **特性**:
  - 速度百分比控制
  - 正反转控制
  - 软启动/软停止
  - 刹车功能
  - 双电机差速控制

**硬件连接**:
- PA8 -> 电机驱动器PWM输入
- PA9 -> 电机驱动器方向控制1
- PB0 -> 电机驱动器方向控制2

**运行方式**:
```bash
cargo run --bin motor_speed
```

### 4. RGB LED控制 (rgb_led.rs)
- **功能**: 使用多通道PWM控制RGB LED
- **特性**:
  - RGB颜色控制
  - HSV颜色空间支持
  - 彩虹效果
  - 呼吸灯效果
  - 颜色渐变
  - 闪烁和频闪效果

**硬件连接**:
- PA8 -> RGB LED红色通道 (通过限流电阻)
- PA9 -> RGB LED绿色通道 (通过限流电阻)
- PA10 -> RGB LED蓝色通道 (通过限流电阻)
- GND -> RGB LED公共负极

**运行方式**:
```bash
cargo run --bin rgb_led
```

## PWM基础知识

### PWM参数
- **频率**: PWM信号的重复频率
- **占空比**: 高电平时间占整个周期的百分比
- **分辨率**: 占空比的最小调节步长

### 应用场景
1. **LED调光**: 1-10kHz频率，避免闪烁
2. **舵机控制**: 50Hz频率，1-2ms脉宽
3. **电机调速**: 10-20kHz频率，减少噪音
4. **音频生成**: 音频频率范围的PWM

## 硬件要求

- STM32F407开发板
- 面包板和跳线
- LED和限流电阻
- 舵机 (SG90或类似)
- 直流电机和驱动器模块
- RGB LED模块

## 编译和运行

1. 确保已安装Rust和相关工具链
2. 连接ST-Link调试器
3. 选择要运行的示例:

```bash
# LED调光示例
cargo run --bin led_dimmer

# 舵机控制示例
cargo run --bin servo_control

# 电机调速示例
cargo run --bin motor_speed

# RGB LED控制示例
cargo run --bin rgb_led
```

## 注意事项

1. **电源管理**: 确保外设电源供应充足
2. **信号电平**: 注意3.3V和5V电平兼容性
3. **负载能力**: GPIO输出电流限制，大功率设备需要驱动电路
4. **频率选择**: 根据应用选择合适的PWM频率
5. **占空比范围**: 某些应用有特定的占空比要求

## 扩展功能

- 添加编码器反馈的闭环控制
- 实现PID控制算法
- 多通道同步PWM输出
- DMA驱动的高性能PWM
- 故障检测和保护机制

## 相关文档

- [05-pwm-generation.md](../../05-pwm-generation.md) - PWM生成详细说明
- [07-advanced-timer-features.md](../../07-advanced-timer-features.md) - 高级定时器功能
- STM32F4xx参考手册 - 定时器章节
# 舵机控制系统

这是一个基于STM32F4的智能舵机控制系统，支持多种控制模式和高级功能。

## 功能特性

### 基础功能
- **多舵机控制**: 支持同时控制4个舵机
- **PWM信号生成**: 50Hz频率，500-2500μs脉宽范围
- **角度控制**: 0-180度精确角度控制
- **速度控制**: 可调节舵机运动速度

### 高级功能
- **平滑运动**: 支持速度限制的平滑角度过渡
- **多种控制模式**: 手动、扫描、序列、校准模式
- **动作序列**: 预设复杂动作序列，支持循环播放
- **实时控制**: 按钮交互式控制
- **状态指示**: LED状态指示和调试输出

## 硬件连接

### 舵机连接
```
舵机1 (PB4) - TIM3_CH1 - PWM输出
舵机2 (PB5) - TIM3_CH2 - PWM输出  
舵机3 (PB0) - TIM3_CH3 - PWM输出
舵机4 (PB1) - TIM3_CH4 - PWM输出
```

### 控制按钮
```
模式按钮 (PA0) - 切换控制模式
角度按钮 (PA1) - 调整角度/参数
速度按钮 (PA2) - 调整速度等级
序列按钮 (PA3) - 启动/停止动作序列
```

### 状态指示
```
状态LED (PC13) - 系统状态指示
```

### 舵机电源
```
VCC: 5V (外部电源)
GND: 共地
信号线: 连接到对应PWM引脚
```

## 技术原理

### PWM控制原理
舵机通过PWM信号控制角度：
- **频率**: 50Hz (20ms周期)
- **脉宽范围**: 500-2500μs
- **角度映射**: 线性映射到0-180度

### 控制算法
```rust
// 角度到脉宽转换
pulse_us = min_pulse + (angle / max_angle) * (max_pulse - min_pulse)

// PWM占空比计算
duty_cycle = (pulse_us * 65535) / 20000
```

### 平滑运动算法
```rust
// 速度限制的角度更新
max_step = (speed * dt_ms) / 1000.0
if |target - current| <= max_step {
    current = target
} else {
    current += sign(target - current) * max_step
}
```

## 代码结构

### 主要组件
- `ServoController`: 单个舵机控制器
- `MultiServoController`: 多舵机协调控制
- `ServoSequence`: 动作序列管理
- `ButtonManager`: 按钮事件处理
- `SystemState`: 系统状态管理

### 关键参数
```rust
const MIN_PULSE_US: u16 = 500;   // 最小脉宽
const MAX_PULSE_US: u16 = 2500;  // 最大脉宽
const MAX_ANGLE: f32 = 180.0;    // 最大角度
const PWM_FREQUENCY: u32 = 50;   // PWM频率
const UPDATE_RATE: u32 = 50;     // 更新频率
```

## 编译和运行

### 环境要求
- Rust 1.70+
- probe-rs
- STM32F407开发板

### 编译项目
```bash
cd servo-control
cargo build --release
```

### 烧录运行
```bash
cargo run --release
```

### 调试输出
```bash
# 查看RTT调试输出
probe-rs attach --chip STM32F407VGTx
```

## 调试输出示例

```
INFO  舵机控制系统启动
INFO  舵机控制系统初始化完成
INFO  模式按钮: 切换控制模式
INFO  角度按钮: 调整角度/参数
INFO  速度按钮: 调整速度等级
INFO  序列按钮: 启动/停止动作序列
INFO  舵机角度设置为: 90.0°, 脉宽: 1500μs
INFO  切换到模式: Sweep
INFO  手动角度: 120.0°
INFO  速度等级: 3
INFO  开始执行舵机序列，共8步
INFO  模式: Sweep, 角度: 135.0°, 速度: 90.0°/s
INFO  性能统计 - 平均更新时间: 45μs, 最大: 78μs, 更新次数: 250
```

## 控制模式详解

### 1. 手动模式 (Manual)
- 通过角度按钮调整目标角度
- 所有舵机同步运动
- 状态LED常亮

### 2. 扫描模式 (Sweep)
- 舵机在设定范围内往复运动
- 可调节扫描幅度和频率
- 状态LED闪烁指示

### 3. 序列模式 (Sequence)
- 执行预设的复杂动作序列
- 支持多舵机协调运动
- 可循环播放
- 状态LED熄灭

### 4. 校准模式 (Calibration)
- 用于舵机校准和测试
- 缓慢全范围运动
- 状态LED快速闪烁

## 扩展功能

### 1. 机械臂控制
```rust
// 添加逆运动学计算
impl RobotArm {
    fn inverse_kinematics(&self, target_pos: Position) -> JointAngles {
        // 逆运动学算法实现
    }
}
```

### 2. 轨迹规划
```rust
// 添加轨迹插值
impl TrajectoryPlanner {
    fn plan_path(&self, waypoints: &[Position]) -> Trajectory {
        // 轨迹规划算法
    }
}
```

### 3. 力反馈控制
```rust
// 添加电流检测和力控制
impl ForceController {
    fn control_force(&mut self, target_force: f32) {
        // 力控制算法
    }
}
```

### 4. 网络控制
```rust
// 添加WiFi/蓝牙控制接口
impl NetworkController {
    fn handle_remote_command(&mut self, cmd: Command) {
        // 网络命令处理
    }
}
```

## 性能优化

### 1. 数学计算优化
```rust
// 使用查找表优化三角函数
const SIN_TABLE: [f32; 360] = [...];

fn fast_sin(angle: f32) -> f32 {
    SIN_TABLE[angle as usize % 360]
}
```

### 2. 内存优化
```rust
// 使用heapless容器避免动态分配
use heapless::Vec;
let mut servos: Vec<ServoInfo, 8> = Vec::new();
```

### 3. 实时性优化
```rust
// 使用中断驱动的定时更新
#[interrupt]
fn TIM2() {
    // 高优先级舵机更新
    servo_controller.update();
}
```

## 故障排除

### 1. 舵机不响应
- 检查PWM信号连接
- 验证电源供应(5V)
- 确认脉宽范围设置

### 2. 运动不平滑
- 调整更新频率
- 检查速度限制设置
- 优化控制算法参数

### 3. 角度不准确
- 进行舵机校准
- 调整脉宽范围参数
- 检查机械安装

### 4. 系统不稳定
- 检查电源纹波
- 添加滤波电容
- 优化代码执行时间

## 相关资源

### 文档资料
- [STM32F4 HAL文档](https://docs.rs/stm32f4xx-hal/)
- [嵌入式Rust书籍](https://doc.rust-lang.org/embedded-book/)
- [PWM原理详解](../docs/pwm-principles.md)

### 示例代码
- [基础PWM示例](../examples/basic-pwm/)
- [定时器配置](../examples/timer-config/)
- [中断处理](../examples/interrupt-handling/)

### 工具链
- [probe-rs调试工具](https://probe.rs/)
- [defmt日志框架](https://defmt.ferrous-systems.com/)
- [embedded-hal抽象层](https://docs.rs/embedded-hal/)

---

**注意**: 使用前请确保正确连接硬件，避免短路或过载。建议在面包板上先进行原型验证。
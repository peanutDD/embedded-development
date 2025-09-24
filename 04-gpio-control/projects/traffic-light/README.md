# 智能交通灯系统

这个项目实现了一个功能完整的智能交通灯控制系统，支持多种工作模式和智能控制功能。系统采用状态机设计，具有良好的扩展性和可维护性。

## 功能特性

### 🚦 核心功能
- **多路交通控制**: 支持主路和辅路的独立交通灯控制
- **行人过街系统**: 集成行人信号灯和按钮请求功能
- **智能时序控制**: 可配置的绿灯、黄灯、红灯持续时间
- **状态机驱动**: 基于状态机的可靠状态转换

### 🌙 工作模式
- **正常模式**: 标准的交通灯循环控制
- **夜间模式**: 主路闪烁黄灯，辅路红灯
- **紧急模式**: 所有方向闪烁黄灯
- **维护模式**: 系统维护时的安全状态

### 🎛️ 交互控制
- **行人按钮**: 请求行人过街信号
- **紧急按钮**: 切换紧急模式
- **夜间模式按钮**: 手动切换夜间模式
- **状态指示灯**: 显示当前系统工作状态

### 📊 系统监控
- **实时统计**: 记录系统运行统计信息
- **RTT调试输出**: 详细的状态和事件日志
- **性能监控**: 周期计数和事件统计

## 硬件连接

### 主路交通灯 (STM32F407)
```
PA0 ----[220Ω]---- 主路红灯LED ---- GND
PA1 ----[220Ω]---- 主路黄灯LED ---- GND  
PA2 ----[220Ω]---- 主路绿灯LED ---- GND
```

### 辅路交通灯
```
PA3 ----[220Ω]---- 辅路红灯LED ---- GND
PA4 ----[220Ω]---- 辅路黄灯LED ---- GND
PA5 ----[220Ω]---- 辅路绿灯LED ---- GND
```

### 行人信号灯
```
PB0 ----[220Ω]---- 行人红灯LED ---- GND
PB1 ----[220Ω]---- 行人绿灯LED ---- GND
```

### 状态指示灯
```
PB2 ----[220Ω]---- 正常状态LED ---- GND
PB3 ----[220Ω]---- 紧急状态LED ---- GND
PB4 ----[220Ω]---- 夜间状态LED ---- GND
```

### 控制按钮
```
PC13 ---- 行人按钮 ---- GND (内部上拉)
PC14 ---- 紧急按钮 ---- GND (内部上拉)
PC15 ---- 夜间模式按钮 ---- GND (内部上拉)
```

## 系统架构

### 状态机设计

#### 正常模式状态转换
```
MainGreen → MainYellow → SideGreen → SideYellow → MainGreen
     ↓
PedestrianWait → PedestrianCross → MainGreen
```

#### 时序配置
```rust
const GREEN_DURATION_MS: u32 = 5000;    // 绿灯5秒
const YELLOW_DURATION_MS: u32 = 2000;   // 黄灯2秒  
const RED_DURATION_MS: u32 = 3000;      // 红灯3秒
const PEDESTRIAN_CROSS_MS: u32 = 8000;  // 行人通行8秒
```

### 核心组件

1. **TrafficLightController**: 主控制器
   - 管理所有交通灯状态
   - 处理状态转换逻辑
   - 控制闪烁效果

2. **SystemState**: 系统状态管理
   - 跟踪当前工作模式
   - 处理行人请求
   - 维护统计信息

3. **ButtonManager**: 按钮输入管理
   - 检测按钮按下事件
   - 防抖处理
   - 边沿检测

4. **各类灯光控制结构体**:
   - `MainTrafficLights`: 主路交通灯
   - `SideTrafficLights`: 辅路交通灯
   - `PedestrianLights`: 行人信号灯
   - `StatusLights`: 状态指示灯

## 工作流程

### 正常模式循环
1. **主路绿灯阶段** (5秒)
   - 主路: 绿灯
   - 辅路: 红灯
   - 行人: 红灯

2. **主路黄灯阶段** (2秒)
   - 主路: 黄灯
   - 辅路: 红灯
   - 行人: 红灯

3. **辅路绿灯阶段** (5秒)
   - 主路: 红灯
   - 辅路: 绿灯
   - 行人: 红灯

4. **辅路黄灯阶段** (2秒)
   - 主路: 红灯
   - 辅路: 黄灯
   - 行人: 红灯

### 行人过街流程
1. **行人按钮按下**
   - 系统记录行人请求
   - 等待当前绿灯阶段结束

2. **行人等待阶段** (1秒)
   - 所有车辆: 红灯
   - 行人: 红灯

3. **行人通行阶段** (8秒)
   - 所有车辆: 红灯
   - 行人: 绿灯
   - 最后3秒绿灯闪烁提醒

### 特殊模式

#### 夜间模式
- 主路: 闪烁黄灯 (1秒间隔)
- 辅路: 红灯
- 行人: 红灯

#### 紧急模式
- 所有方向: 闪烁黄灯 (0.5秒间隔)
- 行人: 关闭

#### 维护模式
- 所有灯光: 2秒间隔闪烁红灯

## 编译和运行

### 1. 编译项目
```bash
cd 04-gpio-control/projects/traffic-light
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

### 系统启动
```
智能交通灯系统启动
系统时钟配置完成: SYSCLK=168MHz
交通灯系统初始化完成
开始交通灯控制循环
```

### 正常运行
```
状态转换: MainGreen -> MainYellow
状态转换: MainYellow -> SideGreen
状态转换: SideGreen -> SideYellow
状态转换: SideYellow -> MainGreen
运行统计: 周期=1, 时间=10000ms, 模式=Normal
统计信息: 正常周期=4, 行人请求=0, 紧急事件=0
```

### 行人请求
```
行人按钮按下
状态转换: MainGreen -> MainYellow
状态转换: MainYellow -> PedestrianWait
状态转换: PedestrianWait -> PedestrianCross
状态转换: PedestrianCross -> MainGreen
```

### 模式切换
```
紧急按钮按下
进入紧急模式
夜间模式按钮按下
进入夜间模式
```

## 配置参数

### 时序参数
```rust
// 基础时序
const GREEN_DURATION_MS: u32 = 5000;
const YELLOW_DURATION_MS: u32 = 2000;
const RED_DURATION_MS: u32 = 3000;

// 行人时序
const PEDESTRIAN_WAIT_MS: u32 = 1000;
const PEDESTRIAN_CROSS_MS: u32 = 8000;

// 特殊模式时序
const NIGHT_MODE_BLINK_MS: u32 = 1000;
const EMERGENCY_BLINK_MS: u32 = 500;
```

### 系统参数
```rust
const SYSTEM_CLOCK_HZ: u32 = 168_000_000;
const TIMER_FREQUENCY_HZ: u32 = 1000;
```

## 扩展功能

### 1. 自动时间控制
添加RTC模块，实现基于时间的自动模式切换：

```rust
// 自动夜间模式
if current_hour >= NIGHT_MODE_START_HOUR || current_hour < NIGHT_MODE_END_HOUR {
    system_state.set_night_mode();
}
```

### 2. 车流量检测
集成传感器检测车流量，动态调整绿灯时间：

```rust
let traffic_density = sensor.read_traffic_density();
let green_duration = calculate_green_duration(traffic_density);
```

### 3. 网络连接
添加WiFi/以太网模块，实现远程监控和控制：

```rust
// 发送状态到服务器
network.send_status(system_state.get_statistics());

// 接收远程控制命令
if let Some(command) = network.receive_command() {
    system_state.process_remote_command(command);
}
```

### 4. 声音提示
添加蜂鸣器，为视障人士提供声音提示：

```rust
// 行人通行声音提示
if pedestrian_crossing {
    buzzer.play_crossing_sound();
}
```

## 安全特性

### 1. 故障检测
- LED故障检测
- 按钮故障检测
- 系统健康监控

### 2. 安全状态
- 故障时自动进入安全模式
- 所有方向红灯或闪烁黄灯
- 故障状态指示

### 3. 看门狗
- 系统死锁检测
- 自动重启机制
- 状态恢复

## 性能优化

### 1. 内存优化
- 使用栈上数据结构
- 避免动态内存分配
- 优化数据结构大小

### 2. 实时性优化
- 中断驱动的按钮检测
- 高精度定时器
- 优先级调度

### 3. 功耗优化
- 低功耗模式支持
- LED亮度调节
- 待机模式

## 故障排除

### 常见问题

1. **LED不亮**
   - 检查GPIO配置
   - 验证电路连接
   - 确认电源供应

2. **按钮无响应**
   - 检查上拉电阻配置
   - 验证按钮连接
   - 检查防抖逻辑

3. **状态转换异常**
   - 检查定时器配置
   - 验证状态机逻辑
   - 查看RTT调试输出

4. **系统重启**
   - 检查看门狗配置
   - 验证栈大小
   - 检查内存使用

### 调试技巧

1. **RTT调试**
   - 实时状态监控
   - 事件日志记录
   - 性能分析

2. **逻辑分析仪**
   - 信号时序分析
   - 多通道监控
   - 协议解析

3. **示波器**
   - PWM信号测量
   - 时序验证
   - 信号质量分析

## 相关资源

- [交通信号控制系统标准](https://en.wikipedia.org/wiki/Traffic_light_control_and_coordination)
- [STM32F4 GPIO配置指南](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [嵌入式状态机设计](https://www.state-machine.com/)
- [实时系统设计原则](https://en.wikipedia.org/wiki/Real-time_computing)

---

**让交通更安全，让出行更顺畅！** 🚦✨
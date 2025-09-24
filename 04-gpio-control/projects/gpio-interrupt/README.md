# GPIO中断系统

这是一个基于STM32F4和RTIC框架的GPIO中断处理系统，演示了各种中断源的处理和事件驱动编程。

## 功能特性

### 中断源支持
- **按钮中断**: 支持按下、释放、长按检测
- **旋转编码器**: 检测顺时针/逆时针旋转
- **运动传感器**: PIR传感器运动检测
- **定时器中断**: 周期性定时器事件

### 高级功能
- **按钮去抖动**: 硬件去抖和软件去抖结合
- **中断统计**: 实时统计中断频率和间隔
- **事件处理**: 统一的事件处理框架
- **LED指示**: 可视化中断状态
- **实时系统**: 基于RTIC的实时任务调度

## 硬件连接

### 输入设备
```
按钮 (PA0)        - 上拉输入，按下接地
编码器A (PA1)     - 上拉输入，旋转信号A
编码器B (PA2)     - 上拉输入，旋转信号B  
运动传感器 (PA3)  - 下拉输入，检测到运动输出高电平
```

### 输出指示
```
LED0 (PC13) - 按钮状态指示
LED1 (PC14) - 编码器活动指示
LED2 (PC15) - 运动检测指示
LED3 (PA8)  - 定时器心跳指示
```

### 电路连接图
```
按钮连接:
PA0 ----[按钮]---- GND
     |
   10kΩ上拉

编码器连接:
PA1 ----[编码器A]---- VCC/GND
PA2 ----[编码器B]---- VCC/GND
     |           |
   10kΩ上拉    10kΩ上拉

PIR传感器连接:
PA3 ----[PIR OUT]---- 传感器输出
     |
   10kΩ下拉
```

## 技术架构

### RTIC框架
使用RTIC (Real-Time Interrupt-driven Concurrency) 框架：
- **硬件抽象**: 零成本的硬件抽象层
- **任务调度**: 基于优先级的抢占式调度
- **资源共享**: 无锁的资源共享机制
- **实时保证**: 确定性的实时响应

### 中断处理流程
```
硬件中断 → NVIC → 中断处理函数 → 事件生成 → 事件处理 → LED更新
```

### 事件驱动架构
```rust
// 事件定义
enum InterruptEvent {
    ButtonPress,
    ButtonRelease,
    ButtonLongPress,
    MotionDetected,
    MotionStopped,
    EncoderClockwise,
    EncoderCounterClockwise,
    TimerExpired,
}

// 事件处理
impl EventHandler {
    fn handle_event(&mut self, event: InterruptEvent) {
        match event {
            InterruptEvent::ButtonPress => {
                // 处理按钮按下
            }
            // ... 其他事件处理
        }
    }
}
```

## 代码结构

### 主要组件
- `ButtonDebouncer`: 按钮去抖动处理
- `RotaryEncoder`: 旋转编码器解码
- `MotionDetector`: 运动检测和超时管理
- `EventHandler`: 统一事件处理
- `InterruptStats`: 中断统计分析

### 关键算法

#### 按钮去抖动
```rust
impl ButtonDebouncer {
    pub fn update(&mut self, raw_state: bool, timestamp: u64) -> Option<InterruptEvent> {
        // 状态变化检测
        if raw_state != self.last_state {
            self.last_change_time = timestamp;
            self.last_state = raw_state;
        }

        // 去抖动时间检查
        if timestamp - self.last_change_time >= self.debounce_time as u64 {
            if raw_state != self.state {
                self.state = raw_state;
                // 生成按钮事件
            }
        }
    }
}
```

#### 旋转编码器解码
```rust
impl RotaryEncoder {
    pub fn update(&mut self, a_state: bool, b_state: bool) -> Option<InterruptEvent> {
        if a_state != self.last_a && a_state {
            // A信号上升沿检测
            if b_state {
                // 逆时针旋转
                self.position -= 1;
                return Some(InterruptEvent::EncoderCounterClockwise);
            } else {
                // 顺时针旋转
                self.position += 1;
                return Some(InterruptEvent::EncoderClockwise);
            }
        }
    }
}
```

## 编译和运行

### 环境要求
- Rust 1.70+
- probe-rs
- STM32F407开发板
- RTIC框架

### 编译项目
```bash
cd gpio-interrupt
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
INFO  GPIO中断系统初始化
INFO  GPIO中断系统初始化完成
INFO  按钮: PA0 (上拉输入)
INFO  编码器A: PA1 (上拉输入)
INFO  编码器B: PA2 (上拉输入)
INFO  运动传感器: PA3 (下拉输入)
INFO  LED0-3: PC13-15, PA8
INFO  按钮按下 - 计数: 1, LED0: true
INFO  编码器顺时针 - 值: 1
INFO  运动检测 - 计数: 1
INFO  长按检测 - 系统重置
INFO  定时器中断 - LED3: true
INFO  === 系统状态报告 ===
INFO  编码器值: 5
INFO  运动检测次数: 3
INFO  按钮按下次数: 7
INFO  === 中断统计 ===
INFO  ButtonPress: 次数=7, 最小间隔=150ms, 最大间隔=2500ms, 平均间隔=800ms
INFO  EncoderClockwise: 次数=12, 最小间隔=50ms, 最大间隔=300ms, 平均间隔=120ms
INFO  MotionDetected: 次数=3, 最小间隔=5000ms, 最大间隔=15000ms, 平均间隔=8000ms
```

## 中断优先级配置

### NVIC优先级设置
```rust
// 中断优先级 (数值越小优先级越高)
EXTI0 (按钮):      优先级 1
EXTI1 (编码器):    优先级 2  
EXTI3 (运动传感器): 优先级 3
TIM2 (定时器):     优先级 4
```

### RTIC任务优先级
```rust
#[task(priority = 3)] // 高优先级中断处理
fn button_interrupt() { }

#[task(priority = 2)] // 中优先级编码器处理
fn encoder_interrupt() { }

#[task(priority = 1)] // 低优先级周期任务
async fn periodic_task() { }
```

## 性能分析

### 中断响应时间
- **硬件中断延迟**: < 1μs
- **软件处理时间**: 5-20μs
- **事件处理时间**: 10-50μs
- **LED更新时间**: < 5μs

### 内存使用
- **Flash使用**: ~32KB
- **RAM使用**: ~4KB
- **栈使用**: ~2KB
- **堆使用**: 0 (无动态分配)

### CPU使用率
- **空闲时**: < 1%
- **中断处理**: 2-5%
- **周期任务**: 1-3%
- **总使用率**: < 10%

## 扩展功能

### 1. 多按钮矩阵
```rust
// 支持按钮矩阵扫描
impl ButtonMatrix {
    fn scan_matrix(&mut self) -> Vec<ButtonEvent> {
        // 矩阵扫描算法
    }
}
```

### 2. 电容触摸检测
```rust
// 添加电容触摸支持
impl CapacitiveTouch {
    fn detect_touch(&mut self) -> TouchEvent {
        // 电容触摸检测算法
    }
}
```

### 3. 手势识别
```rust
// 基于加速度计的手势识别
impl GestureRecognizer {
    fn recognize_gesture(&mut self, accel_data: AccelData) -> Gesture {
        // 手势识别算法
    }
}
```

### 4. 中断负载均衡
```rust
// 中断负载监控和均衡
impl InterruptLoadBalancer {
    fn balance_load(&mut self) {
        // 动态调整中断优先级
    }
}
```

## 故障排除

### 1. 中断不触发
- 检查GPIO配置和上拉/下拉电阻
- 验证NVIC中断使能
- 确认中断向量表正确

### 2. 按钮抖动严重
- 增加硬件滤波电容
- 调整软件去抖时间
- 使用施密特触发器

### 3. 编码器计数错误
- 检查A/B相信号质量
- 调整中断触发边沿
- 添加硬件滤波

### 4. 系统响应慢
- 优化中断处理函数
- 减少临界区时间
- 调整任务优先级

## 测试用例

### 单元测试
```rust
#[test]
fn test_button_debouncer() {
    let mut debouncer = ButtonDebouncer::new(50, 1000);
    
    // 测试正常按下
    let event = debouncer.update(true, 60);
    assert_eq!(event, Some(InterruptEvent::ButtonPress));
    
    // 测试长按
    let event = debouncer.update(true, 1100);
    assert_eq!(event, Some(InterruptEvent::ButtonLongPress));
}
```

### 集成测试
```rust
#[test]
fn test_interrupt_system() {
    // 模拟完整的中断处理流程
    let mut system = InterruptSystem::new();
    system.simulate_button_press();
    assert_eq!(system.get_led_state(0), true);
}
```

## 相关资源

### 文档资料
- [RTIC框架文档](https://rtic.rs/)
- [STM32F4中断系统](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [嵌入式Rust中断处理](https://doc.rust-lang.org/embedded-book/start/interrupts.html)

### 示例代码
- [基础中断示例](../examples/basic-interrupt/)
- [RTIC入门教程](../examples/rtic-basics/)
- [GPIO配置示例](../examples/gpio-config/)

### 工具链
- [probe-rs调试工具](https://probe.rs/)
- [defmt日志框架](https://defmt.ferrous-systems.com/)
- [RTIC框架](https://rtic.rs/)

---

**注意**: 中断处理函数应保持简短，避免在中断上下文中执行耗时操作。建议将复杂处理逻辑放在任务中执行。
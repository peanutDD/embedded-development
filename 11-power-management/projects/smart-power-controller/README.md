# 智能电源控制器项目

这是一个基于STM32F4的智能电源控制器项目，提供多通道电源管理、负载均衡、能耗优化和智能保护功能。

## 项目特性

### 核心功能
- **多通道电源控制**: 4路独立电源通道，支持不同负载类型
- **智能负载均衡**: 多种负载均衡算法，优化功率分配
- **能耗优化**: 自适应效率优化，降低系统功耗
- **实时监控**: 电压、电流、功率、温度实时监测
- **智能保护**: 过流、过压、过温、短路保护
- **数据记录**: 系统运行数据记录和分析

### 高级特性
- **功率调度**: 基于时间的功率调度管理
- **睡眠管理**: 智能睡眠和唤醒机制
- **故障恢复**: 自动故障检测和恢复
- **通信接口**: 串口状态报告和控制
- **用户界面**: LED状态指示和按键控制

## 硬件连接

### 主控制器 (STM32F4)
```
MCU: STM32F407VGT6
时钟: 84MHz (HSE 8MHz)
Flash: 1MB
RAM: 192KB
```

### 电源通道接口
```
通道0: 高功率通道 (最大5A/50W)
- 电压检测: PA1 (ADC1_IN1)
- 电流检测: PA2 (ADC1_IN2)
- 控制输出: PB12 (PWM/开关)
- 状态LED: PC13

通道1: LED驱动通道 (最大3A/30W)
- 电压检测: PA4 (ADC1_IN4)
- 电流检测: PA5 (ADC1_IN5)
- PWM控制: PB13 (TIM1_CH1N)
- 状态LED: PC14

通道2: 电机驱动通道 (最大2A/20W)
- 电压检测: PA6 (ADC1_IN6)
- 电流检测: PA7 (ADC1_IN7)
- 控制输出: PB14 (PWM)
- 状态LED: PC15

通道3: 关键负载通道 (最大1A/10W)
- 电压检测: PB0 (ADC1_IN8)
- 电流检测: PB1 (ADC1_IN9)
- 控制输出: PB15 (开关)
- 状态LED: PD2
```

### 系统接口
```
温度传感器: PA3 (ADC1_IN3)
系统报警LED: PB0
用户按键: PA0 (外部中断)
I2C接口: PB8(SCL), PB9(SDA)
串口调试: PA9(TX), PA10(RX)
```

## 传感器配置

### 电压传感器
- 分压比: 11:1 (最大检测电压: 36V)
- 精度: ±1%
- 采样频率: 1kHz

### 电流传感器
- 类型: 霍尔电流传感器
- 量程: 0-10A
- 精度: ±1%
- 响应时间: <1ms

### 温度传感器
- 类型: NTC热敏电阻
- 量程: -40°C ~ +125°C
- 精度: ±0.5°C
- 时间常数: 5s

## 数据格式

### 通道数据结构
```rust
pub struct PowerChannel {
    pub id: u8,                    // 通道ID
    pub enabled: bool,             // 使能状态
    pub voltage: u16,              // 电压 (mV)
    pub current: u16,              // 电流 (mA)
    pub power: u16,                // 功率 (mW)
    pub max_current: u16,          // 最大电流限制
    pub max_power: u16,            // 最大功率限制
    pub priority: ChannelPriority, // 通道优先级
    pub load_type: LoadType,       // 负载类型
    pub control_mode: ControlMode, // 控制模式
    pub pwm_duty: u8,              // PWM占空比
    pub protection_status: ProtectionStatus, // 保护状态
}
```

### 系统状态
```rust
pub enum SystemState {
    Initializing,  // 初始化中
    Normal,        // 正常运行
    PowerSaving,   // 节能模式
    Overload,      // 过载状态
    Fault,         // 故障状态
    Shutdown,      // 关机状态
}
```

## LED状态指示

### 通道状态LED
- **常亮**: 通道正常工作
- **熄灭**: 通道关闭
- **快速闪烁**: 通道故障

### 系统报警LED
- **熄灭**: 系统正常
- **常亮**: 系统故障或过载
- **慢速闪烁**: 系统警告

## 构建和烧录

### 环境要求
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install cargo-binutils
rustup component add llvm-tools-preview

# 安装调试工具
# macOS
brew install openocd
# 或使用ST-Link工具
```

### 编译项目
```bash
# 进入项目目录
cd 11-power-management/projects/smart-power-controller

# 编译项目
cargo build --release

# 生成二进制文件
cargo objcopy --release -- -O binary target/smart-power-controller.bin
```

### 烧录程序
```bash
# 使用OpenOCD烧录
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
        -c "program target/thumbv7em-none-eabihf/release/smart-power-controller verify reset exit"

# 或使用ST-Link Utility
st-flash write target/smart-power-controller.bin 0x8000000
```

## 系统配置

### 默认配置
```rust
SystemConfig {
    max_total_power: 100000,    // 最大总功率: 100W
    max_total_current: 10000,   // 最大总电流: 10A
    supply_voltage: 12000,      // 供电电压: 12V
    temperature_limit: 800,     // 温度限制: 80°C
    efficiency_target: 85,      // 效率目标: 85%
    load_balancing: true,       // 启用负载均衡
    energy_optimization: true,  // 启用能耗优化
    data_logging: true,         // 启用数据记录
}
```

### 通道配置
```rust
// 通道0: 高功率通道
PowerChannel {
    max_current: 5000,          // 5A
    max_power: 50000,           // 50W
    priority: ChannelPriority::High,
    load_type: LoadType::Resistive,
    control_mode: ControlMode::OnOff,
}

// 通道1: LED驱动通道
PowerChannel {
    max_current: 3000,          // 3A
    max_power: 30000,           // 30W
    priority: ChannelPriority::Medium,
    load_type: LoadType::LED,
    control_mode: ControlMode::PWM,
}
```

## 功能特性

### 负载均衡算法
1. **轮询算法**: 均匀分配负载
2. **加权轮询**: 基于通道容量分配
3. **最少负载**: 优先分配给负载最轻的通道
4. **功率优先**: 基于功率需求分配
5. **优先级优先**: 基于通道优先级分配

### 能耗优化模式
1. **最大效率**: 优化系统整体效率
2. **最小功耗**: 降低系统总功耗
3. **平衡模式**: 效率和性能平衡
4. **性能模式**: 优先保证性能

### 保护机制
1. **过流保护**: 电流超限自动断开
2. **过压保护**: 电压异常保护
3. **欠压保护**: 电压过低保护
4. **过温保护**: 温度过高降额或断开
5. **短路保护**: 快速短路检测和保护

## 扩展功能

### 通信协议
- **Modbus RTU**: 工业标准通信协议
- **CAN总线**: 车载和工业应用
- **以太网**: 网络监控和控制
- **无线通信**: WiFi/蓝牙远程控制

### 显示界面
- **OLED显示**: 实时状态显示
- **Web界面**: 浏览器监控和控制
- **移动应用**: 手机APP控制

### 数据存储
- **Flash存储**: 配置参数和日志
- **SD卡**: 大容量数据记录
- **云存储**: 远程数据备份

## 调试和测试

### 串口调试
```bash
# 连接串口 (115200 8N1)
screen /dev/tty.usbserial-* 115200

# 或使用minicom
minicom -D /dev/tty.usbserial-* -b 115200
```

### 状态报告格式
```
=== Power Controller Status ===
System State: Normal
Total Power: 45W
CH0: 12V 2500mA 30W ON
CH1: 12V 1250mA 15W ON
CH2: 12V 0mA 0W OFF
CH3: 12V 0mA 0W OFF
===============================
```

### 测试用例
1. **功能测试**: 各通道独立控制测试
2. **保护测试**: 过流、过压、过温保护测试
3. **负载测试**: 不同负载类型适应性测试
4. **效率测试**: 系统效率优化测试
5. **稳定性测试**: 长时间运行稳定性测试

## 注意事项

### 安全警告
- 确保电源电压在安全范围内 (5V-24V)
- 注意电流限制，避免过载
- 定期检查散热情况
- 使用合适的保险丝和断路器

### 使用限制
- 最大总功率: 100W
- 最大工作温度: 85°C
- 电源电压范围: 5V-24V
- 环境湿度: <85% RH

### 维护建议
- 定期清洁散热器
- 检查连接线缆
- 更新固件版本
- 备份配置参数

## 故障排除

### 常见问题

**问题1: 通道无输出**
- 检查通道使能状态
- 确认负载连接正确
- 检查保护状态

**问题2: 效率低下**
- 检查负载匹配
- 调整控制参数
- 确认散热良好

**问题3: 频繁保护**
- 检查负载规格
- 调整保护阈值
- 确认电源质量

**问题4: 通信异常**
- 检查串口连接
- 确认波特率设置
- 检查通信协议

### 错误代码
- `E001`: 初始化失败
- `E002`: ADC读取错误
- `E003`: 通信超时
- `E004`: 温度传感器故障
- `E005`: 电流传感器故障

## 技术支持

如有技术问题，请参考：
1. 项目文档和示例代码
2. STM32F4参考手册
3. Rust嵌入式开发指南
4. 相关传感器数据手册

## 许可证

本项目采用MIT许可证，详见LICENSE文件。
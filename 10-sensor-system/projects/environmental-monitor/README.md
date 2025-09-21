# 环境监测系统

这是一个基于STM32F4的高级环境监测系统，能够同时监测多种环境参数，包括温度、湿度、气压、光照强度、CO2浓度和PM2.5/PM10颗粒物浓度。

## 项目特性

### 硬件支持
- **主控芯片**: STM32F401CCU6
- **温湿度传感器**: SHT30 (I2C接口)
- **气压传感器**: BMP280 (I2C接口)
- **光照传感器**: TSL2561 (I2C接口)
- **CO2传感器**: MH-Z19 (UART接口)
- **颗粒物传感器**: PMS7003 (UART接口)
- **存储器**: W25Q64 Flash (SPI接口)
- **显示器**: SSD1306 OLED (I2C接口)

### 软件功能
- **多传感器数据采集**: 支持同时读取多种环境参数
- **数据融合处理**: 智能数据融合和异常检测
- **实时数据滤波**: 低通滤波器消除噪声
- **数据存储**: Flash存储历史数据
- **统计分析**: 实时计算平均值、最值等统计信息
- **空气质量评估**: 基于PM2.5计算AQI指数
- **串口通信**: 实时数据输出和系统状态监控

## 硬件连接

### I2C总线连接 (I2C1 - 400kHz)
```
STM32F401    SHT30/BMP280/TSL2561/SSD1306
PB8    ----> SCL (时钟线)
PB9    ----> SDA (数据线)
3.3V   ----> VCC
GND    ----> GND
```

### I2C总线连接 (I2C2 - 400kHz)
```
STM32F401    备用I2C设备
PB10   ----> SCL (时钟线)
PB11   ----> SDA (数据线)
3.3V   ----> VCC
GND    ----> GND
```

### SPI总线连接 (SPI1 - 1MHz)
```
STM32F401    W25Q64 Flash
PA5    ----> SCK  (时钟)
PA6    ----> MISO (主入从出)
PA7    ----> MOSI (主出从入)
PA4    ----> CS   (片选)
3.3V   ----> VCC
GND    ----> GND
```

### UART连接
```
STM32F401    MH-Z19 CO2传感器
PA9    ----> RX
PA10   ----> TX
5V     ----> VCC
GND    ----> GND

STM32F401    PMS7003 颗粒物传感器
PA2    ----> TX (调试输出)
PA3    ----> RX (调试输入)
5V     ----> VCC
GND    ----> GND
```

### 状态指示LED
```
STM32F401    LED
PC13   ----> 状态LED (绿色)
PA8    ----> 错误LED (红色)
```

## 传感器配置

### BMP280 气压传感器
- **I2C地址**: 0x76
- **测量范围**: 
  - 温度: -40°C ~ +85°C
  - 气压: 300hPa ~ 1100hPa
- **精度**: 
  - 温度: ±1.0°C
  - 气压: ±1.0hPa
- **采样配置**: 温度过采样x2，气压过采样x16

### SHT30 温湿度传感器
- **I2C地址**: 0x44
- **测量范围**:
  - 温度: -40°C ~ +125°C
  - 湿度: 0% ~ 100% RH
- **精度**:
  - 温度: ±0.3°C
  - 湿度: ±2% RH
- **测量模式**: 高重复性，时钟拉伸使能

### TSL2561 光照传感器
- **I2C地址**: 0x39
- **测量范围**: 0.1 ~ 40,000 lux
- **精度**: ±20%
- **积分时间**: 402ms

## 数据格式

### 环境数据结构
```rust
pub struct EnvironmentalData {
    pub timestamp: u32,        // 时间戳 (秒)
    pub temperature: f32,      // 温度 (°C)
    pub humidity: f32,         // 湿度 (%)
    pub pressure: f32,         // 气压 (hPa)
    pub light_intensity: f32,  // 光照强度 (lux)
    pub co2_level: u16,        // CO2浓度 (ppm)
    pub pm25: u16,             // PM2.5 (μg/m³)
    pub pm10: u16,             // PM10 (μg/m³)
    pub air_quality_index: u8, // 空气质量指数 (0-255)
}
```

### 串口输出格式
```
Data: T=23.5C H=65.2% P=1013.2hPa AQI=45
Stats: Avg=23.1C Min=22.8C Max=23.7C
```

## LED状态指示

### 状态LED (PC13 - 绿色)
- **常亮**: 系统初始化成功
- **闪烁**: 正常运行，数据采集中
- **熄灭**: 系统错误或传感器故障

### 错误LED (PA8 - 红色)
- **常亮**: 系统初始化失败或传感器通信错误
- **熄灭**: 系统正常运行

## 构建和烧录

### 环境要求
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install cargo-binutils
rustup component add llvm-tools-preview

# 安装调试工具
brew install arm-none-eabi-gcc
brew install openocd
```

### 编译项目
```bash
cd 10-sensor-system/projects/environmental-monitor

# 调试版本
cargo build

# 发布版本 (优化)
cargo build --release

# 检查代码
cargo check
```

### 烧录程序
```bash
# 使用OpenOCD烧录
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program target/thumbv7em-none-eabihf/release/environmental-monitor verify reset exit"

# 或使用st-flash
st-flash write target/thumbv7em-none-eabihf/release/environmental-monitor 0x8000000
```

### 调试
```bash
# 启动OpenOCD服务器
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# 在另一个终端启动GDB
arm-none-eabi-gdb target/thumbv7em-none-eabihf/debug/environmental-monitor
(gdb) target remote localhost:3333
(gdb) load
(gdb) continue
```

## 系统配置

### 采样参数
- **采样间隔**: 1秒 (可配置)
- **数据缓冲**: 100个历史数据点
- **滤波系数**: α = 0.1 (低通滤波器)
- **统计周期**: 每10次采样输出一次统计信息

### 存储管理
- **Flash容量**: 8MB (W25Q64)
- **扇区大小**: 4KB
- **数据记录大小**: 32字节/条
- **存储容量**: 约262,144条记录
- **数据保留**: 断电保持

### 通信配置
- **串口波特率**: 115200 bps
- **数据位**: 8
- **停止位**: 1
- **校验位**: 无
- **流控**: 无

## 功能特性

### 多传感器管理
- **并发采集**: 支持多个传感器同时工作
- **错误隔离**: 单个传感器故障不影响其他传感器
- **自动重试**: 通信失败时自动重试机制
- **状态监控**: 实时监控传感器工作状态

### 数据缓冲和存储
- **环形缓冲**: 内存中维护最近100个数据点
- **Flash存储**: 所有数据永久保存到Flash
- **数据完整性**: CRC校验确保数据完整性
- **磨损均衡**: 自动管理Flash写入位置

### 错误处理和恢复
- **异常检测**: 自动检测和过滤异常数据
- **通信重试**: I2C/SPI通信失败自动重试
- **系统重启**: 严重错误时自动重启系统
- **状态报告**: 通过LED和串口报告系统状态

### 电源管理
- **低功耗模式**: 支持待机和睡眠模式
- **动态调频**: 根据负载调整CPU频率
- **外设控制**: 不使用时关闭外设电源
- **电压监控**: 监控系统供电电压

## 扩展功能

### 无线通信 (可选)
```toml
[features]
wireless = ["esp-wifi", "smoltcp"]
```
- **WiFi连接**: ESP32-C3模块
- **数据上传**: HTTP/MQTT协议
- **远程监控**: Web界面和手机APP
- **OTA更新**: 无线固件更新

### 数据加密 (可选)
```toml
[features]
security = ["aes", "sha2"]
```
- **AES加密**: 数据存储加密
- **SHA2哈希**: 数据完整性验证
- **密钥管理**: 安全密钥存储
- **访问控制**: 用户认证机制

### 高级分析 (可选)
```toml
[features]
advanced-analytics = ["nalgebra"]
```
- **趋势分析**: 数据趋势预测
- **异常检测**: 机器学习异常检测
- **相关性分析**: 多参数相关性分析
- **预警系统**: 智能预警和报警

### Modbus支持 (可选)
```toml
[features]
modbus-support = ["modbus"]
```
- **Modbus RTU**: 串口Modbus通信
- **Modbus TCP**: 以太网Modbus通信
- **从站模式**: 作为Modbus从站设备
- **寄存器映射**: 标准寄存器地址映射

## 调试和测试

### 串口调试
```bash
# macOS/Linux
screen /dev/tty.usbserial-* 115200

# Windows
putty -serial COM* -sercfg 115200,8,n,1,N
```

### 测试用例
```bash
# 运行单元测试
cargo test

# 运行集成测试
cargo test --test integration

# 运行基准测试
cargo bench
```

### 性能监控
- **CPU使用率**: 通过DWT周期计数器监控
- **内存使用**: 堆栈使用情况监控
- **I2C总线**: 通信成功率和延迟监控
- **Flash写入**: 写入次数和磨损监控

## 注意事项

### 硬件注意事项
1. **电源要求**: 确保3.3V和5V电源稳定
2. **I2C上拉**: 确保I2C总线有适当的上拉电阻(4.7kΩ)
3. **信号完整性**: 保持连线尽可能短，避免干扰
4. **散热设计**: 确保传感器有良好的通风环境

### 软件注意事项
1. **中断优先级**: 合理设置中断优先级避免冲突
2. **栈大小**: 确保栈大小足够，避免栈溢出
3. **看门狗**: 启用看门狗定时器防止系统死锁
4. **时钟配置**: 确保时钟配置正确，影响通信时序

### 环境注意事项
1. **温度范围**: 工作温度-10°C ~ +60°C
2. **湿度范围**: 相对湿度10% ~ 90%，无凝露
3. **振动保护**: 避免强烈振动影响传感器精度
4. **电磁干扰**: 远离强电磁干扰源

## 故障排除

### 常见问题

#### 系统无法启动
- 检查电源连接和电压
- 验证晶振是否正常工作
- 检查复位电路

#### 传感器读取失败
- 检查I2C总线连接和上拉电阻
- 验证传感器地址是否正确
- 检查电源电压是否稳定

#### 数据异常
- 检查传感器校准是否正确
- 验证环境条件是否在测量范围内
- 检查是否有电磁干扰

#### Flash存储错误
- 检查SPI连接是否正确
- 验证Flash芯片是否损坏
- 检查写保护设置

### 调试技巧
1. **使用串口输出**: 添加调试信息输出
2. **LED指示**: 利用LED显示系统状态
3. **逻辑分析仪**: 分析I2C/SPI通信时序
4. **示波器**: 检查信号质量和时序

## 技术支持

如有问题，请参考：
1. **STM32F4参考手册**: 详细的外设配置信息
2. **传感器数据手册**: 各传感器的详细规格
3. **Rust嵌入式书籍**: Rust嵌入式开发指南
4. **社区论坛**: 嵌入式开发社区支持

## 许可证

本项目采用MIT许可证，详见LICENSE文件。
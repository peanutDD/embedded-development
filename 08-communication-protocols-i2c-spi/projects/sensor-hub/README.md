# 传感器集线器项目

这是一个基于 STM32F4 的多传感器数据采集系统，使用 I2C 和 SPI 协议与多种传感器通信，并将数据存储到 Flash 存储器中。

## 项目特性

### 硬件支持
- **微控制器**: STM32F401 系列
- **I2C 传感器**: BMP280 (温度/压力), MPU6050 (加速度计/陀螺仪)
- **SPI 存储**: W25Q64 Flash 存储器
- **状态指示**: LED 状态显示
- **电源管理**: 传感器和存储器电源控制

### 软件功能
- **多传感器支持**: 同时读取多种传感器数据
- **数据缓冲**: 内存中缓冲传感器数据
- **Flash 存储**: 定期将数据写入 Flash 存储器
- **错误处理**: 传感器故障检测和系统恢复
- **状态监控**: LED 指示系统运行状态
- **低功耗**: 采样间隔期间进入低功耗模式

## 硬件连接

### I2C 连接 (传感器)
```
STM32F401    BMP280/MPU6050
PB6    ----> SCL
PB7    ----> SDA
3.3V   ----> VCC
GND    ----> GND
PB8    ----> 传感器电源控制
```

### SPI 连接 (Flash 存储)
```
STM32F401    W25Q64
PA5    ----> SCK
PA6    ----> MISO
PA7    ----> MOSI
PB0    ----> CS
3.3V   ----> VCC
GND    ----> GND
PB9    ----> Flash 电源控制
```

### 状态指示
```
STM32F401    LED
PC13   ----> LED (低电平点亮)
```

## 传感器配置

### BMP280 (温度/压力传感器)
- **I2C 地址**: 0x76
- **测量模式**: 正常模式
- **采样率**: 1Hz
- **精度**: 温度 ±1°C, 压力 ±1hPa

### MPU6050 (6轴传感器)
- **I2C 地址**: 0x68
- **加速度计**: ±2g 量程
- **陀螺仪**: ±250°/s 量程
- **采样率**: 1Hz

### W25Q64 (Flash 存储)
- **容量**: 8MB
- **页大小**: 256 字节
- **扇区大小**: 4KB
- **数据格式**: 44字节/记录 (包含校验和)

## 数据格式

每条传感器记录包含以下数据 (44字节):
```rust
struct SensorData {
    timestamp: u32,      // 时间戳 (4字节)
    temperature: f32,    // 温度 (4字节)
    pressure: f32,       // 压力 (4字节)
    humidity: f32,       // 湿度 (4字节)
    accel_x: f32,        // X轴加速度 (4字节)
    accel_y: f32,        // Y轴加速度 (4字节)
    accel_z: f32,        // Z轴加速度 (4字节)
    gyro_x: f32,         // X轴角速度 (4字节)
    gyro_y: f32,         // Y轴角速度 (4字节)
    gyro_z: f32,         // Z轴角速度 (4字节)
    checksum: u32,       // 校验和 (4字节)
}
```

## LED 状态指示

### 启动序列
- **渐进闪烁**: 系统启动中
- **2次闪烁**: 传感器初始化成功
- **3次闪烁**: Flash 初始化成功
- **5次快闪**: 传感器初始化失败
- **6次快闪**: Flash 初始化失败

### 运行状态
- **短暂点亮**: 正在采样
- **长时间亮起**: 系统状态正常 (每分钟一次)
- **1次短闪**: 传感器读取失败
- **10次快闪**: 系统恢复中

### 错误指示
- **连续快闪**: 系统错误，正在恢复
- **间歇闪烁**: 根据错误数量显示

## 构建和烧录

### 环境要求
```bash
# 安装 Rust 和目标平台
rustup target add thumbv7em-none-eabihf

# 安装工具链
sudo apt-get install gcc-arm-none-eabi gdb-arm-none-eabi
```

### 构建项目
```bash
# 调试版本
cargo build

# 发布版本 (优化)
cargo build --release
```

### 烧录程序
```bash
# 使用 OpenOCD
openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program target/thumbv7em-none-eabihf/release/sensor-hub verify reset exit"

# 或使用 st-flash
st-flash write target/thumbv7em-none-eabihf/release/sensor-hub.bin 0x8000000
```

## 系统配置

### 采样参数
```rust
const SAMPLE_INTERVAL_MS: u32 = 1000;  // 采样间隔 1秒
const LOG_BUFFER_SIZE: usize = 100;    // 缓冲区大小
const FLASH_LOG_INTERVAL: u32 = 10;    // 每10次采样写入Flash
```

### 存储管理
- **起始地址**: 0x1000 (4KB)
- **记录大小**: 44字节
- **自动擦除**: 写入新扇区时自动擦除
- **校验和**: 每条记录包含校验和

## 功能特性

### 1. 多传感器管理
```rust
// 传感器管理器支持多种传感器
let mut sensor_manager = SensorManager::new(i2c);
sensor_manager.initialize()?;
let data = sensor_manager.read_all_sensors()?;
```

### 2. 数据缓冲和存储
```rust
// 数据记录器管理内存缓冲
let mut data_logger = DataLogger::new();
data_logger.log_data(sensor_data, sample_count);

// Flash 存储管理
let mut flash_storage = FlashStorage::new(spi, cs);
flash_storage.write_sensor_data(data_buffer)?;
```

### 3. 错误处理和恢复
```rust
// 自动错误检测和系统恢复
if error_count > 10 {
    system_recovery(&mut sensor_manager, &mut flash_storage, &mut led);
    error_count = 0;
}
```

### 4. 电源管理
```rust
// 传感器电源控制
sensor_power.set_high();  // 打开传感器电源
flash_power.set_high();   // 打开Flash电源

// 低功耗等待
cortex_m::asm::wfi();
```

## 扩展功能

### 添加新传感器
1. 在 `sensors` 模块中添加传感器驱动
2. 更新 `SensorData` 结构体
3. 在 `SensorManager` 中添加初始化和读取函数

### 数据通信
- 可添加 UART/USB 接口用于数据上传
- 支持实时数据监控
- 可实现远程配置功能

### 存储优化
- 实现数据压缩算法
- 添加磨损均衡机制
- 支持数据加密存储

## 调试和测试

### 调试输出
```rust
// 启用调试功能
cargo build --features debug-output
```

### 测试模式
```rust
// 单独测试传感器
cargo build --features bmp280-sensor
cargo build --features mpu6050-sensor

// 测试存储功能
cargo build --features flash-storage
```

### 性能监控
- 监控采样频率
- 检查存储写入速度
- 分析系统功耗

## 注意事项

1. **电源稳定性**: 确保传感器和Flash供电稳定
2. **I2C 上拉**: I2C 总线需要上拉电阻 (4.7kΩ)
3. **SPI 时序**: 确保 SPI 时序符合 Flash 规格
4. **存储管理**: 定期检查 Flash 存储空间
5. **错误处理**: 实现完善的错误恢复机制

## 故障排除

### 传感器无响应
- 检查 I2C 连接和上拉电阻
- 验证传感器电源和地线
- 确认 I2C 地址正确

### Flash 写入失败
- 检查 SPI 连接
- 验证 CS 信号时序
- 确认 Flash 电源稳定

### 系统频繁重启
- 检查电源供应能力
- 验证看门狗配置
- 分析错误日志

这个传感器集线器项目展示了如何在嵌入式系统中集成多种通信协议和传感器，实现可靠的数据采集和存储系统。
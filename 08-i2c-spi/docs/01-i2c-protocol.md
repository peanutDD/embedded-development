# I2C 协议详解

## 概述

I2C (Inter-Integrated Circuit) 是由飞利浦公司开发的一种串行通信协议，用于连接微控制器和各种外设。它是一种多主机、多从机的通信总线，只需要两根线就能实现数据传输。

## I2C 总线特性

### 物理特性
- **双线制**: 只需要 SDA (数据线) 和 SCL (时钟线) 两根线
- **开漏输出**: 需要上拉电阻 (通常 4.7kΩ)
- **多主机支持**: 支持多个主机设备
- **多从机支持**: 最多支持 127 个从机设备

### 电气特性
- **电压范围**: 1.8V - 5V
- **上拉电阻**: 1kΩ - 10kΩ (典型值 4.7kΩ)
- **电容负载**: 标准模式 ≤ 400pF，快速模式 ≤ 400pF

### 速度模式
- **标准模式**: 100 kbit/s
- **快速模式**: 400 kbit/s
- **快速模式+**: 1 Mbit/s
- **高速模式**: 3.4 Mbit/s
- **超快速模式**: 5 Mbit/s

## I2C 协议格式

### 基本信号
```
SDA: ----\___/---\___/---
SCL: ------\___/---\___/---
     START  BIT   BIT  STOP
```

### 起始和停止条件
- **起始条件 (START)**: SCL 为高时，SDA 从高变低
- **停止条件 (STOP)**: SCL 为高时，SDA 从低变高
- **重复起始条件 (RESTART)**: 不发送停止条件，直接发送新的起始条件

### 数据传输格式
```
START + 地址(7位) + R/W(1位) + ACK + 数据(8位) + ACK + ... + STOP
```

#### 地址帧
- **7位地址**: 0x00 - 0x7F (0x00-0x07 和 0x78-0x7F 为保留地址)
- **10位地址**: 使用特殊格式扩展地址空间
- **R/W位**: 0 = 写操作，1 = 读操作

#### 数据帧
- **8位数据**: MSB 先传输
- **应答位 (ACK)**: 0 = 应答，1 = 非应答 (NACK)

## I2C 通信时序

### 写操作时序
```rust
// 主机写入数据到从机
fn i2c_write_sequence() {
    // 1. 发送起始条件
    send_start();
    
    // 2. 发送从机地址 + 写位 (0)
    send_byte(slave_address << 1 | 0);
    wait_ack();
    
    // 3. 发送寄存器地址 (可选)
    send_byte(register_address);
    wait_ack();
    
    // 4. 发送数据
    for data in write_data {
        send_byte(data);
        wait_ack();
    }
    
    // 5. 发送停止条件
    send_stop();
}
```

### 读操作时序
```rust
// 主机从从机读取数据
fn i2c_read_sequence() {
    // 1. 发送起始条件
    send_start();
    
    // 2. 发送从机地址 + 写位 (指定寄存器)
    send_byte(slave_address << 1 | 0);
    wait_ack();
    
    // 3. 发送寄存器地址
    send_byte(register_address);
    wait_ack();
    
    // 4. 重复起始条件
    send_restart();
    
    // 5. 发送从机地址 + 读位 (1)
    send_byte(slave_address << 1 | 1);
    wait_ack();
    
    // 6. 读取数据
    for i in 0..read_length {
        let data = read_byte();
        if i == read_length - 1 {
            send_nack(); // 最后一个字节发送 NACK
        } else {
            send_ack();  // 其他字节发送 ACK
        }
    }
    
    // 7. 发送停止条件
    send_stop();
}
```

## I2C 仲裁机制

### 多主机仲裁
当多个主机同时尝试控制总线时，I2C 使用仲裁机制来决定哪个主机获得控制权：

1. **线与逻辑**: 任何设备都可以将总线拉低
2. **仲裁过程**: 主机在发送每一位时检查总线状态
3. **仲裁失败**: 如果主机发送高电平但检测到低电平，则失去仲裁
4. **仲裁获胜**: 成功发送完整地址的主机获得总线控制权

```rust
fn arbitration_example() {
    // 主机 A 发送地址 0x50 (0101 0000)
    // 主机 B 发送地址 0x48 (0100 1000)
    
    // 位 0: A=0, B=0 -> 总线=0 (继续)
    // 位 1: A=1, B=1 -> 总线=1 (继续)
    // 位 2: A=0, B=0 -> 总线=0 (继续)
    // 位 3: A=1, B=0 -> 总线=0 (A 失去仲裁)
    
    // 主机 B 获得总线控制权
}
```

### 时钟同步
- **时钟延展**: 从机可以将 SCL 拉低来延长时钟周期
- **同步机制**: 多个主机的时钟会自动同步到最慢的时钟

## I2C 错误处理

### 常见错误类型
1. **总线忙**: 总线被其他设备占用
2. **无应答**: 从机没有响应 ACK
3. **仲裁失败**: 在多主机环境中失去仲裁
4. **数据冲突**: 发送和接收的数据不匹配
5. **超时错误**: 操作超时

### 错误检测和恢复
```rust
enum I2cError {
    BusBusy,
    NoAck,
    ArbitrationLost,
    DataConflict,
    Timeout,
}

fn error_recovery(error: I2cError) {
    match error {
        I2cError::BusBusy => {
            // 等待总线空闲
            wait_bus_idle();
        }
        I2cError::NoAck => {
            // 发送停止条件，重置总线
            send_stop();
        }
        I2cError::ArbitrationLost => {
            // 等待当前传输完成，然后重试
            wait_bus_idle();
            retry_transmission();
        }
        I2cError::Timeout => {
            // 重置 I2C 外设
            reset_i2c_peripheral();
        }
        _ => {
            // 通用恢复：重置总线
            reset_bus();
        }
    }
}
```

## I2C 地址分配

### 7位地址空间
```
0x00-0x07: 保留地址
0x08-0x77: 一般用途地址
0x78-0x7F: 保留地址
```

### 保留地址
- **0x00**: 通用调用地址
- **0x01**: CBUS 地址
- **0x02**: 不同总线格式
- **0x03**: 保留
- **0x04-0x07**: 高速模式主机代码
- **0x78-0x7B**: 10位从机地址
- **0x7C-0x7F**: 保留

### 常见设备地址
```rust
// 常见 I2C 设备地址
const EEPROM_24C02: u8 = 0x50;      // EEPROM
const RTC_DS1307: u8 = 0x68;        // 实时时钟
const TEMP_LM75: u8 = 0x48;         // 温度传感器
const ACCEL_ADXL345: u8 = 0x53;     // 加速度计
const GYRO_MPU6050: u8 = 0x68;      // 陀螺仪
const DISPLAY_SSD1306: u8 = 0x3C;   // OLED 显示屏
```

## I2C 性能优化

### 时钟频率选择
```rust
fn optimize_clock_frequency() {
    // 根据应用需求选择合适的时钟频率
    
    // 低功耗应用: 使用较低频率
    let low_power_freq = 10_000; // 10 kHz
    
    // 标准应用: 使用标准频率
    let standard_freq = 100_000; // 100 kHz
    
    // 高速应用: 使用快速模式
    let fast_freq = 400_000; // 400 kHz
    
    // 考虑因素:
    // 1. 电容负载
    // 2. 上拉电阻值
    // 3. 传输距离
    // 4. 功耗要求
}
```

### 批量传输优化
```rust
fn batch_transfer_optimization() {
    // 使用连续传输减少开销
    
    // 低效方式: 每个字节单独传输
    for data in write_data {
        i2c_write_single(slave_addr, reg_addr, data);
    }
    
    // 高效方式: 批量传输
    i2c_write_multiple(slave_addr, reg_addr, &write_data);
}
```

### DMA 传输
```rust
fn dma_transfer_setup() {
    // 使用 DMA 减少 CPU 负载
    
    // 配置 DMA 通道
    setup_dma_channel();
    
    // 启动 DMA 传输
    start_dma_transfer(data_buffer, length);
    
    // 等待传输完成
    wait_dma_complete();
}
```

## 调试技巧

### 逻辑分析仪调试
1. **信号完整性**: 检查信号边沿和电平
2. **时序分析**: 验证建立时间和保持时间
3. **协议解码**: 使用协议解码功能分析数据
4. **错误检测**: 识别 ACK/NACK 和时序错误

### 软件调试
```rust
fn debug_i2c_communication() {
    // 1. 检查总线状态
    if !is_bus_idle() {
        println!("Bus is busy");
        return;
    }
    
    // 2. 扫描设备地址
    for addr in 0x08..=0x77 {
        if i2c_probe(addr).is_ok() {
            println!("Found device at address: 0x{:02X}", addr);
        }
    }
    
    // 3. 监控传输状态
    let result = i2c_write(slave_addr, reg_addr, data);
    match result {
        Ok(_) => println!("Write successful"),
        Err(e) => println!("Write failed: {:?}", e),
    }
}
```

### 常见问题排查
1. **上拉电阻**: 检查是否正确连接上拉电阻
2. **地址冲突**: 确认设备地址不冲突
3. **时钟频率**: 验证时钟频率在设备支持范围内
4. **电源电压**: 确认所有设备使用相同的电源电压
5. **线路长度**: 检查传输距离是否过长

## 总结

I2C 是一种简单而强大的通信协议，具有以下优点：
- 只需两根线即可连接多个设备
- 支持多主机和多从机
- 内置仲裁和错误检测机制
- 广泛的设备支持

在实际应用中，需要注意：
- 正确的硬件连接和上拉电阻
- 合适的时钟频率选择
- 有效的错误处理机制
- 性能优化和调试技巧

掌握 I2C 协议的原理和实现细节，对于嵌入式系统开发具有重要意义。
# Modbus RTU 协议规范

## 概述

Modbus RTU（Remote Terminal Unit）是一种广泛应用于工业自动化领域的串行通信协议。本文档详细描述了Modbus RTU协议的实现规范和在嵌入式Rust项目中的应用。

## 协议特性

### 基本特征
- **传输模式**: 异步串行传输
- **数据格式**: 8位数据位，1位停止位，偶校验或无校验
- **波特率**: 1200-115200 bps（常用9600, 19200, 38400）
- **最大设备数**: 247个从站设备
- **帧间隔**: 至少3.5个字符时间

### 数据编码
- **字节序**: 大端序（Big-Endian）
- **CRC校验**: CRC-16（多项式0xA001）
- **地址范围**: 1-247（0为广播地址，248-255保留）

## 帧格式

### ADU（Application Data Unit）结构
```
+----------+----------+----------+----------+
| 地址字段 | 功能码   | 数据字段 | CRC校验  |
| 1 byte   | 1 byte   | 0-252    | 2 bytes  |
+----------+----------+----------+----------+
```

### 详细字段说明

#### 1. 地址字段（Address Field）
- **长度**: 1字节
- **范围**: 1-247
- **特殊值**: 0（广播地址）

#### 2. 功能码（Function Code）
- **长度**: 1字节
- **范围**: 1-127（正常功能）, 128-255（异常响应）

#### 3. 数据字段（Data Field）
- **长度**: 0-252字节
- **内容**: 根据功能码确定

#### 4. CRC校验（CRC Check）
- **长度**: 2字节
- **算法**: CRC-16
- **多项式**: 0xA001
- **初始值**: 0xFFFF

## 支持的功能码

### 读取功能码

#### 0x01 - 读取线圈状态
```rust
// 请求帧格式
struct ReadCoilsRequest {
    address: u8,           // 设备地址
    function_code: u8,     // 0x01
    starting_address: u16, // 起始地址
    quantity: u16,         // 线圈数量
    crc: u16,             // CRC校验
}

// 响应帧格式
struct ReadCoilsResponse {
    address: u8,        // 设备地址
    function_code: u8,  // 0x01
    byte_count: u8,     // 数据字节数
    coil_status: Vec<u8>, // 线圈状态
    crc: u16,          // CRC校验
}
```

#### 0x02 - 读取离散输入状态
```rust
// 请求帧格式
struct ReadDiscreteInputsRequest {
    address: u8,           // 设备地址
    function_code: u8,     // 0x02
    starting_address: u16, // 起始地址
    quantity: u16,         // 输入数量
    crc: u16,             // CRC校验
}
```

#### 0x03 - 读取保持寄存器
```rust
// 请求帧格式
struct ReadHoldingRegistersRequest {
    address: u8,           // 设备地址
    function_code: u8,     // 0x03
    starting_address: u16, // 起始地址
    quantity: u16,         // 寄存器数量
    crc: u16,             // CRC校验
}

// 响应帧格式
struct ReadHoldingRegistersResponse {
    address: u8,           // 设备地址
    function_code: u8,     // 0x03
    byte_count: u8,        // 数据字节数
    register_values: Vec<u16>, // 寄存器值
    crc: u16,             // CRC校验
}
```

#### 0x04 - 读取输入寄存器
```rust
// 请求帧格式
struct ReadInputRegistersRequest {
    address: u8,           // 设备地址
    function_code: u8,     // 0x04
    starting_address: u16, // 起始地址
    quantity: u16,         // 寄存器数量
    crc: u16,             // CRC校验
}
```

### 写入功能码

#### 0x05 - 写入单个线圈
```rust
// 请求帧格式
struct WriteSingleCoilRequest {
    address: u8,      // 设备地址
    function_code: u8, // 0x05
    coil_address: u16, // 线圈地址
    coil_value: u16,   // 线圈值（0x0000或0xFF00）
    crc: u16,         // CRC校验
}
```

#### 0x06 - 写入单个寄存器
```rust
// 请求帧格式
struct WriteSingleRegisterRequest {
    address: u8,         // 设备地址
    function_code: u8,   // 0x06
    register_address: u16, // 寄存器地址
    register_value: u16,   // 寄存器值
    crc: u16,           // CRC校验
}
```

#### 0x0F - 写入多个线圈
```rust
// 请求帧格式
struct WriteMultipleCoilsRequest {
    address: u8,           // 设备地址
    function_code: u8,     // 0x0F
    starting_address: u16, // 起始地址
    quantity: u16,         // 线圈数量
    byte_count: u8,        // 数据字节数
    coil_values: Vec<u8>,  // 线圈值
    crc: u16,             // CRC校验
}
```

#### 0x10 - 写入多个寄存器
```rust
// 请求帧格式
struct WriteMultipleRegistersRequest {
    address: u8,              // 设备地址
    function_code: u8,        // 0x10
    starting_address: u16,    // 起始地址
    quantity: u16,            // 寄存器数量
    byte_count: u8,           // 数据字节数
    register_values: Vec<u16>, // 寄存器值
    crc: u16,                // CRC校验
}
```

## 异常响应

### 异常功能码
异常响应的功能码 = 原功能码 + 0x80

### 异常代码
```rust
#[repr(u8)]
pub enum ModbusException {
    IllegalFunction = 0x01,      // 非法功能码
    IllegalDataAddress = 0x02,   // 非法数据地址
    IllegalDataValue = 0x03,     // 非法数据值
    SlaveDeviceFailure = 0x04,   // 从站设备故障
    Acknowledge = 0x05,          // 确认
    SlaveDeviceBusy = 0x06,      // 从站设备忙
    MemoryParityError = 0x08,    // 存储器奇偶校验错误
    GatewayPathUnavailable = 0x0A, // 网关路径不可用
    GatewayTargetFailed = 0x0B,    // 网关目标设备响应失败
}
```

### 异常响应帧格式
```rust
struct ExceptionResponse {
    address: u8,        // 设备地址
    function_code: u8,  // 原功能码 + 0x80
    exception_code: u8, // 异常代码
    crc: u16,          // CRC校验
}
```

## CRC-16 计算

### 算法实现
```rust
pub fn calculate_crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    
    for byte in data {
        crc ^= *byte as u16;
        
        for _ in 0..8 {
            if crc & 0x0001 != 0 {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    crc
}
```

### CRC表查找法
```rust
const CRC_TABLE: [u16; 256] = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    // ... 完整的CRC表
];

pub fn calculate_crc16_table(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    
    for byte in data {
        let table_index = ((crc ^ (*byte as u16)) & 0xFF) as usize;
        crc = (crc >> 8) ^ CRC_TABLE[table_index];
    }
    
    crc
}
```

## 时序要求

### 字符间隔
- **T1.5**: 1.5个字符时间
- **T3.5**: 3.5个字符时间
- **计算公式**: T = (1 + 8 + 1 + 1) / 波特率 = 11 / 波特率

### 帧间隔
```rust
pub fn calculate_frame_delay(baud_rate: u32) -> u32 {
    if baud_rate <= 19200 {
        // 低波特率：3.5个字符时间
        (3500000 / baud_rate).max(1750) // 最小1.75ms
    } else {
        // 高波特率：固定1.75ms
        1750
    }
}
```

## 数据模型

### 数据区域
```rust
pub struct ModbusDataModel {
    // 离散输入（只读）
    pub discrete_inputs: [bool; 10000],    // 地址 10001-20000
    
    // 线圈（读写）
    pub coils: [bool; 10000],              // 地址 00001-10000
    
    // 输入寄存器（只读）
    pub input_registers: [u16; 10000],     // 地址 30001-40000
    
    // 保持寄存器（读写）
    pub holding_registers: [u16; 10000],   // 地址 40001-50000
}
```

### 地址映射
```rust
pub fn modbus_to_internal_address(modbus_addr: u16, data_type: DataType) -> Option<usize> {
    match data_type {
        DataType::Coil => {
            if modbus_addr >= 1 && modbus_addr <= 10000 {
                Some((modbus_addr - 1) as usize)
            } else {
                None
            }
        },
        DataType::DiscreteInput => {
            if modbus_addr >= 10001 && modbus_addr <= 20000 {
                Some((modbus_addr - 10001) as usize)
            } else {
                None
            }
        },
        DataType::HoldingRegister => {
            if modbus_addr >= 40001 && modbus_addr <= 50000 {
                Some((modbus_addr - 40001) as usize)
            } else {
                None
            }
        },
        DataType::InputRegister => {
            if modbus_addr >= 30001 && modbus_addr <= 40000 {
                Some((modbus_addr - 30001) as usize)
            } else {
                None
            }
        },
    }
}
```

## 实现注意事项

### 1. 字节序处理
```rust
// 大端序转换
pub fn u16_to_be_bytes(value: u16) -> [u8; 2] {
    value.to_be_bytes()
}

pub fn u16_from_be_bytes(bytes: [u8; 2]) -> u16 {
    u16::from_be_bytes(bytes)
}
```

### 2. 超时处理
```rust
pub const RESPONSE_TIMEOUT_MS: u32 = 1000;  // 响应超时
pub const TURNAROUND_DELAY_MS: u32 = 100;   // 转向延迟
```

### 3. 错误处理
```rust
#[derive(Debug)]
pub enum ModbusError {
    InvalidCrc,
    InvalidLength,
    InvalidAddress,
    InvalidFunctionCode,
    Timeout,
    CommunicationError,
}
```

## 参考资料

1. **Modbus Application Protocol Specification V1.1b3**
2. **Modbus over Serial Line Specification and Implementation Guide V1.02**
3. **RFC 3164 - The BSD Syslog Protocol**
4. **IEC 61158 - Industrial communication networks**

## 版本历史

- **v1.0** (2024-01): 初始版本
- **v1.1** (2024-02): 添加异常处理
- **v1.2** (2024-03): 完善CRC计算和时序要求
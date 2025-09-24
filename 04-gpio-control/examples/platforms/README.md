# 嵌入式平台GPIO控制示例

本目录包含了针对不同嵌入式平台的GPIO控制示例代码，展示了如何在各种微控制器上实现基础的GPIO操作。

## 支持的平台

### 1. STM32F4 系列
- **目录**: `stm32f4/`
- **芯片**: STM32F407VGT6
- **特性**:
  - ARM Cortex-M4 内核 (168MHz)
  - 1MB Flash, 128KB RAM
  - 丰富的外设接口
  - 高性能浮点运算单元

**功能特性**:
- LED闪烁控制 (PC13)
- 按钮输入检测 (PA0)
- PWM呼吸灯效果 (PA1)
- 定时器中断处理
- GPIO状态监控
- 性能统计

### 2. ESP32 系列
- **目录**: `esp32/`
- **芯片**: ESP32-WROOM-32
- **特性**:
  - 双核 Xtensa LX6 (240MHz)
  - 4MB Flash, 520KB RAM
  - 内置WiFi和蓝牙
  - 丰富的GPIO和外设

**功能特性**:
- 异步任务处理
- LED闪烁控制 (GPIO2)
- 按钮监控 (GPIO4)
- PWM呼吸灯 (GPIO5)
- WiFi状态管理
- 传感器数据读取
- 系统监控

### 3. RP2040 系列
- **目录**: `rp2040/`
- **芯片**: RP2040 (Raspberry Pi Pico)
- **特性**:
  - 双核 ARM Cortex-M0+ (133MHz)
  - 2MB Flash, 264KB RAM
  - 独特的PIO状态机
  - USB原生支持

**功能特性**:
- LED闪烁控制 (GPIO25)
- 按钮输入检测 (GPIO15)
- PWM呼吸灯 (GPIO16)
- PIO状态机控制
- USB串口通信
- 多核处理支持
- 性能监控

### 4. nRF52 系列
- **目录**: `nrf52/`
- **芯片**: nRF52840
- **特性**:
  - ARM Cortex-M4F (64MHz)
  - 1MB Flash, 256KB RAM
  - 内置蓝牙5.0
  - 超低功耗设计

**功能特性**:
- 多LED控制 (跑马灯效果)
- 多按钮监控
- UART串口通信
- 蓝牙BLE支持
- 低功耗模式
- 温度和电压监控
- 异步任务处理

## 编译和运行

### 前置要求

1. **Rust工具链**:
   ```bash
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   rustup target add thumbv7em-none-eabihf  # STM32F4, nRF52
   rustup target add thumbv6m-none-eabi     # RP2040
   rustup target add xtensa-esp32-none-elf  # ESP32
   ```

2. **调试工具**:
   ```bash
   # probe-rs (推荐)
   cargo install probe-rs --features cli
   
   # 或者使用OpenOCD
   # 各平台安装方法不同，请参考官方文档
   ```

3. **平台特定工具**:
   - **ESP32**: `espup`, `cargo-espflash`
   - **RP2040**: `elf2uf2-rs`, `picotool`
   - **STM32**: `stm32cubeprog` (可选)
   - **nRF52**: `nrfjprog` (可选)

### 编译示例

进入对应平台目录并编译：

```bash
# STM32F4
cd stm32f4
cargo build --release

# ESP32
cd esp32
cargo build --release

# RP2040
cd rp2040
cargo build --release

# nRF52
cd nrf52
cargo build --release
```

### 烧录和运行

#### 使用 probe-rs (推荐)
```bash
# 自动检测并烧录
cargo run --release

# 或者手动指定芯片
probe-rs run --chip STM32F407VGTx target/thumbv7em-none-eabihf/release/stm32f4-gpio-examples
```

#### 使用平台特定工具

**STM32F4**:
```bash
# 使用ST-Link
st-flash write target/thumbv7em-none-eabihf/release/stm32f4-gpio-examples.bin 0x8000000
```

**ESP32**:
```bash
# 使用espflash
cargo espflash flash --release --monitor
```

**RP2040**:
```bash
# 使用UF2格式
elf2uf2-rs target/thumbv6m-none-eabi/release/rp2040-gpio-examples rp2040-gpio-examples.uf2
# 将UF2文件拖拽到BOOTSEL模式下的Pico
```

**nRF52**:
```bash
# 使用nrfjprog
nrfjprog --program target/thumbv7em-none-eabihf/release/nrf52-gpio-examples.hex --sectorerase --verify --reset
```

## 硬件连接

### 通用连接
- **LED**: 各平台内置LED或外接LED + 限流电阻
- **按钮**: 上拉电阻 + 按钮开关
- **PWM输出**: LED + 限流电阻 (用于呼吸灯效果)

### 平台特定连接

**STM32F4 (以STM32F407-DISCO为例)**:
- LED: PC13 (内置)
- 按钮: PA0 (内置USER按钮)
- PWM: PA1 (TIM2_CH2)

**ESP32-DevKitC**:
- LED: GPIO2 (内置)
- 按钮: GPIO4 (外接)
- PWM: GPIO5 (外接LED)

**Raspberry Pi Pico**:
- LED: GPIO25 (内置)
- 按钮: GPIO15 (外接)
- PWM: GPIO16 (外接LED)

**nRF52840-DK**:
- LED1-4: P0.13-P0.16 (内置)
- 按钮1-2: P0.11-P0.12 (内置)
- UART: P0.06(TX), P0.08(RX)

## 调试和监控

### 日志输出
所有示例都支持defmt日志输出，可以通过以下方式查看：

```bash
# 使用probe-rs
probe-rs run --chip <CHIP_NAME> --log-format defmt

# 使用RTT Viewer
JLinkRTTViewer
```

### 性能监控
各平台示例都包含性能监控功能：
- 循环执行时间统计
- 内存使用情况
- GPIO切换频率
- 中断响应时间

### 故障排除

1. **编译错误**:
   - 检查Rust工具链和目标架构
   - 确认依赖版本兼容性
   - 查看编译器错误信息

2. **烧录失败**:
   - 检查硬件连接
   - 确认调试器驱动
   - 尝试不同的烧录工具

3. **运行异常**:
   - 检查时钟配置
   - 确认GPIO引脚配置
   - 查看调试日志输出

## 扩展功能

### 添加新的GPIO功能
1. 在对应平台的`main.rs`中添加新的GPIO配置
2. 实现相应的控制逻辑
3. 添加必要的测试代码

### 移植到新平台
1. 创建新的平台目录
2. 配置`Cargo.toml`依赖
3. 实现平台特定的HAL调用
4. 添加内存配置和构建脚本

### 性能优化
- 使用DMA减少CPU负载
- 优化中断处理程序
- 实现低功耗模式
- 使用硬件定时器提高精度

## 相关资源

### 官方文档
- [STM32F4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0090-stm32f405415-stm32f407417-stm32f427437-and-stm32f429439-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [ESP32 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
- [RP2040 Datasheet](https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf)
- [nRF52840 Product Specification](https://infocenter.nordicsemi.com/pdf/nRF52840_PS_v1.1.pdf)

### 开发工具
- [probe-rs](https://probe.rs/) - 现代化的嵌入式调试工具
- [defmt](https://defmt.ferrous-systems.com/) - 高效的嵌入式日志框架
- [Embassy](https://embassy.dev/) - 现代异步嵌入式框架

### 社区资源
- [Embedded Rust Book](https://doc.rust-lang.org/embedded-book/)
- [Awesome Embedded Rust](https://github.com/rust-embedded/awesome-embedded-rust)
- [Rust Embedded Working Group](https://github.com/rust-embedded)

## 许可证

本项目采用 MIT 或 Apache-2.0 双重许可证。详见各平台目录中的许可证文件。
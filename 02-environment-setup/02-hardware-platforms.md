# 硬件平台配置

## 概述

本章节将详细介绍如何为不同的嵌入式硬件平台配置Rust开发环境。我们将涵盖最流行的嵌入式平台，包括STM32、ESP32、Raspberry Pi Pico和Nordic nRF系列。

## 支持的硬件平台

### 平台概览

| 平台 | 架构 | 主要特性 | 适用场景 |
|------|------|----------|----------|
| STM32 | ARM Cortex-M | 丰富外设、低功耗 | 工业控制、IoT |
| ESP32 | Xtensa/RISC-V | WiFi/蓝牙、高性能 | 物联网、智能家居 |
| Raspberry Pi Pico | ARM Cortex-M0+ | 低成本、易用 | 教育、原型开发 |
| Nordic nRF | ARM Cortex-M | 低功耗蓝牙、Zigbee | 可穿戴设备、传感器网络 |

## STM32开发环境配置

### 支持的STM32系列

STM32系列微控制器是最受欢迎的ARM Cortex-M平台之一：

- **STM32F0**: Cortex-M0+，入门级
- **STM32F1**: Cortex-M3，经典系列
- **STM32F4**: Cortex-M4，高性能
- **STM32F7**: Cortex-M7，超高性能
- **STM32H7**: Cortex-M7，最新高端系列
- **STM32L**: 低功耗系列
- **STM32G**: 主流系列

### 目标架构配置

根据你的STM32型号选择合适的目标架构：

```bash
# STM32F0系列 (Cortex-M0+)
rustup target add thumbv6m-none-eabi

# STM32F1系列 (Cortex-M3)
rustup target add thumbv7m-none-eabi

# STM32F4系列 (Cortex-M4F，带FPU)
rustup target add thumbv7em-none-eabihf

# STM32F7/H7系列 (Cortex-M7F，带FPU)
rustup target add thumbv7em-none-eabihf

# STM32L系列 (低功耗，根据具体型号选择)
rustup target add thumbv7em-none-eabi  # 或 thumbv7em-none-eabihf
```

### STM32项目配置

#### 1. 创建STM32项目

```bash
# 使用cargo-generate创建项目
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart

# 或手动创建
cargo new --bin stm32-project
cd stm32-project
```

#### 2. Cargo.toml配置

```toml
[package]
name = "stm32-project"
version = "0.1.0"
edition = "2021"

[dependencies]
# STM32F4xx HAL (以STM32F411为例)
stm32f4xx-hal = { version = "0.16", features = ["stm32f411"] }
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"

# 可选：RTT用于调试输出
rtt-target = { version = "0.4", optional = true }

[features]
default = []
rtt = ["rtt-target"]

# 配置目标架构
[build]
target = "thumbv7em-none-eabihf"
```

#### 3. 内存配置 (memory.x)

```ld
/* STM32F411RE的内存配置 */
MEMORY
{
  /* Flash存储器 */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  
  /* RAM */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}
```

#### 4. 构建配置 (.cargo/config.toml)

```toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32F411RETx"
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[env]
DEFMT_LOG = "debug"
```

### STM32开发板推荐

#### 入门级开发板
- **STM32F411 Nucleo**: 性价比高，功能丰富
- **STM32F103 Blue Pill**: 便宜，社区支持好
- **STM32F0 Discovery**: 适合初学者

#### 高级开发板
- **STM32F429 Discovery**: 带LCD显示屏
- **STM32H743 Nucleo**: 高性能应用
- **STM32L476 Discovery**: 低功耗应用

## ESP32开发环境配置

### ESP32系列概览

ESP32系列提供了强大的WiFi和蓝牙功能：

- **ESP32**: 经典版本，Xtensa架构
- **ESP32-S2**: 单核，专注WiFi
- **ESP32-S3**: 双核，AI加速
- **ESP32-C3**: RISC-V架构，低成本
- **ESP32-C6**: 最新RISC-V，WiFi 6支持

### 安装ESP32工具链

#### 1. 安装espup工具

```bash
# 安装espup
cargo install espup

# 安装ESP32工具链
espup install
```

#### 2. 配置环境变量

```bash
# 添加到 ~/.bashrc 或 ~/.zshrc
source $HOME/export-esp.sh
```

#### 3. 安装目标架构

```bash
# ESP32 (Xtensa)
rustup target add xtensa-esp32-espidf

# ESP32-S2 (Xtensa)
rustup target add xtensa-esp32s2-espidf

# ESP32-S3 (Xtensa)
rustup target add xtensa-esp32s3-espidf

# ESP32-C3 (RISC-V)
rustup target add riscv32imc-esp-espidf

# ESP32-C6 (RISC-V)
rustup target add riscv32imac-esp-espidf
```

### ESP32项目配置

#### 1. 创建ESP32项目

```bash
# 使用esp-idf-template
cargo generate esp-rs/esp-idf-template cargo

# 或使用no_std模板
cargo generate esp-rs/esp-template
```

#### 2. Cargo.toml配置

```toml
[package]
name = "esp32-project"
version = "0.1.0"
edition = "2021"

[dependencies]
esp-idf-sys = { version = "0.33", default-features = false }
esp-idf-hal = "0.42"
esp-idf-svc = "0.47"
embedded-svc = "0.25"

# WiFi和网络功能
esp-wifi = "0.1"
smoltcp = "0.10"

[build-dependencies]
embuild = "0.31"
```

#### 3. 构建配置

```toml
# .cargo/config.toml
[build]
target = "xtensa-esp32-espidf"

[target.xtensa-esp32-espidf]
linker = "ldproxy"
runner = "espflash flash --monitor"

[env]
ESP_IDF_VERSION = "v5.1"
```

### ESP32开发板推荐

- **ESP32-DevKitC**: 官方开发板
- **ESP32-WROOM-32**: 模块化设计
- **ESP32-S3-DevKitC**: 最新S3系列
- **ESP32-C3-DevKitM**: RISC-V入门

## Raspberry Pi Pico配置

### Pico系列概览

Raspberry Pi Pico基于RP2040微控制器：

- **Raspberry Pi Pico**: 基础版本
- **Raspberry Pi Pico W**: 带WiFi功能
- **Raspberry Pi Pico H**: 预焊接排针

### 安装Pico工具链

```bash
# 添加目标架构
rustup target add thumbv6m-none-eabi

# 安装elf2uf2-rs工具
cargo install elf2uf2-rs --locked

# 安装probe-rs (用于调试)
cargo install probe-rs --features cli
```

### Pico项目配置

#### 1. 创建Pico项目

```bash
# 使用官方模板
cargo generate --git https://github.com/rp-rs/rp2040-project-template

# 项目名称
cd my-pico-project
```

#### 2. Cargo.toml配置

```toml
[package]
name = "pico-project"
version = "0.1.0"
edition = "2021"

[dependencies]
rp-pico = "0.8"
cortex-m = "0.7"
cortex-m-rt = "0.7"
embedded-hal = "0.2"
panic-halt = "0.2"

# 可选功能
rp2040-hal = "0.9"
embedded-time = "0.12"
```

#### 3. 构建配置

```toml
# .cargo/config.toml
[build]
target = "thumbv6m-none-eabi"

[target.thumbv6m-none-eabi]
runner = "elf2uf2-rs -d"
rustflags = [
  "-C", "link-arg=--nmagic",
  "-C", "link-arg=-Tlink.x",
]
```

#### 4. 内存配置

```ld
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}
```

### Pico烧录方法

#### 方法1: UF2模式烧录

```bash
# 1. 按住BOOTSEL按钮，连接USB
# 2. 编译并烧录
cargo run

# 或手动烧录
cargo build --release
elf2uf2-rs target/thumbv6m-none-eabi/release/pico-project /Volumes/RPI-RP2/
```

#### 方法2: 使用调试器

```bash
# 使用probe-rs
probe-rs run --chip RP2040 target/thumbv6m-none-eabi/debug/pico-project
```

## Nordic nRF配置

### nRF系列概览

Nordic nRF系列专注于低功耗无线通信：

- **nRF52832**: 蓝牙5.0，经典版本
- **nRF52840**: 蓝牙5.0，USB支持
- **nRF5340**: 双核，蓝牙5.2
- **nRF9160**: LTE-M/NB-IoT

### 安装nRF工具链

```bash
# 添加目标架构
rustup target add thumbv7em-none-eabihf  # nRF52系列

# 安装nrf-util
pip install nrfutil

# 安装probe-rs
cargo install probe-rs --features cli
```

### nRF项目配置

#### 1. 创建nRF项目

```bash
# 使用nrf-rs模板
cargo generate --git https://github.com/nrf-rs/nrf-project-template

# 选择芯片型号
cd nrf-project
```

#### 2. Cargo.toml配置

```toml
[package]
name = "nrf-project"
version = "0.1.0"
edition = "2021"

[dependencies]
# nRF52840 HAL
nrf52840-hal = "0.16"
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"

# 蓝牙协议栈
nrf-softdevice = "0.1"
nrf-softdevice-s140 = "0.1"

# 可选：RTT调试
rtt-target = "0.4"
```

#### 3. 构建配置

```toml
# .cargo/config.toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip nRF52840_xxAA"
rustflags = [
  "-C", "link-arg=-Tlink.x",
]
```

#### 4. 内存配置 (带SoftDevice)

```ld
MEMORY
{
  /* SoftDevice S140占用空间 */
  FLASH : ORIGIN = 0x00027000, LENGTH = 868K
  RAM : ORIGIN = 0x20020000, LENGTH = 128K
}
```

### nRF开发板推荐

- **nRF52840 DK**: 官方开发套件
- **nRF52840 Dongle**: USB适配器形式
- **Adafruit Feather nRF52840**: 第三方开发板

## 通用配置技巧

### 1. 多平台项目结构

```
my-embedded-project/
├── Cargo.toml
├── src/
│   ├── main.rs
│   └── lib.rs
├── boards/
│   ├── stm32f4/
│   │   ├── Cargo.toml
│   │   └── memory.x
│   ├── esp32/
│   │   └── Cargo.toml
│   └── pico/
│       ├── Cargo.toml
│       └── memory.x
└── .cargo/
    └── config.toml
```

### 2. 条件编译配置

```rust
// src/lib.rs
#[cfg(feature = "stm32f4")]
pub use stm32f4xx_hal as hal;

#[cfg(feature = "esp32")]
pub use esp_idf_hal as hal;

#[cfg(feature = "rp2040")]
pub use rp2040_hal as hal;

#[cfg(feature = "nrf52840")]
pub use nrf52840_hal as hal;
```

### 3. 工作空间配置

```toml
# Cargo.toml (根目录)
[workspace]
members = [
    "boards/stm32f4",
    "boards/esp32",
    "boards/pico",
    "boards/nrf52840",
]

[workspace.dependencies]
cortex-m = "0.7"
embedded-hal = "0.2"
```

## 验证配置

### 创建测试项目

```rust
// src/main.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    // 平台特定的初始化代码
    #[cfg(feature = "stm32f4")]
    {
        use stm32f4xx_hal::prelude::*;
        let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
        let rcc = dp.RCC.constrain();
        let _clocks = rcc.cfgr.freeze();
    }
    
    loop {
        // 主循环
    }
}
```

### 编译测试

```bash
# 测试不同平台
cargo build --target thumbv7em-none-eabihf --features stm32f4
cargo build --target xtensa-esp32-espidf --features esp32
cargo build --target thumbv6m-none-eabi --features rp2040
cargo build --target thumbv7em-none-eabihf --features nrf52840
```

## 常见问题解决

### 1. 链接器错误

```bash
# 确保安装了正确的链接器
# 对于ARM目标
sudo apt-get install gcc-arm-none-eabi  # Ubuntu/Debian
brew install arm-none-eabi-gcc          # macOS

# 对于ESP32
espup install
```

### 2. 目标架构不匹配

```bash
# 检查已安装的目标
rustup target list --installed

# 添加缺失的目标
rustup target add <target-name>
```

### 3. 内存配置错误

- 检查memory.x文件是否正确
- 确认Flash和RAM大小与芯片规格匹配
- 注意SoftDevice占用的空间

## 下一步

配置完硬件平台后，建议：

1. 学习[调试工具配置](./03-debugging-tools.md)
2. 配置[IDE开发环境](./04-ide-setup.md)
3. 创建第一个硬件项目

---

**现在你已经为主流嵌入式平台配置好了开发环境！** 🎯
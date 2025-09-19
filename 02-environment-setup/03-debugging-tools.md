# 调试工具配置

## 概述

调试是嵌入式开发中最重要的技能之一。本章节将详细介绍如何配置和使用各种调试工具，包括probe-rs、OpenOCD、GDB调试器和RTT (Real Time Transfer) 等现代调试技术。

## 调试工具概览

### 主要调试工具对比

| 工具 | 优势 | 劣势 | 适用场景 |
|------|------|------|----------|
| probe-rs | 现代、快速、易用 | 相对较新 | 推荐用于新项目 |
| OpenOCD | 成熟、支持广泛 | 配置复杂 | 传统项目、特殊硬件 |
| GDB | 功能强大、标准工具 | 学习曲线陡峭 | 深度调试 |
| RTT | 实时、高性能 | 需要特定支持 | 日志输出、性能分析 |

## probe-rs 配置和使用

### 什么是 probe-rs

probe-rs 是用Rust编写的现代嵌入式调试工具，提供了：
- 快速的烧录和调试
- 内置RTT支持
- 简单的命令行界面
- 与Rust工具链深度集成

### 安装 probe-rs

```bash
# 方法1: 使用cargo安装
cargo install probe-rs --features cli

# 方法2: 从GitHub下载预编译版本
# 访问 https://github.com/probe-rs/probe-rs/releases

# 方法3: 使用包管理器 (macOS)
brew install probe-rs
```

### 验证安装

```bash
# 检查版本
probe-rs --version

# 列出支持的芯片
probe-rs chip list

# 列出连接的调试器
probe-rs list
```

### 基本使用

#### 1. 烧录程序

```bash
# 基本烧录命令
probe-rs run --chip STM32F411RETx target/thumbv7em-none-eabihf/debug/my-project

# 烧录并重置
probe-rs run --chip STM32F411RETx --reset-after-flashing target/debug/my-project

# 烧录到指定地址
probe-rs run --chip STM32F411RETx --base-address 0x08000000 firmware.bin
```

#### 2. 调试会话

```bash
# 启动GDB调试会话
probe-rs debug --chip STM32F411RETx target/debug/my-project

# 启动并连接到特定调试器
probe-rs debug --probe VID:PID --chip STM32F411RETx target/debug/my-project
```

#### 3. RTT输出

```bash
# 查看RTT输出
probe-rs rtt --chip STM32F411RETx

# 指定RTT通道
probe-rs rtt --chip STM32F411RETx --channel 0

# 保存RTT输出到文件
probe-rs rtt --chip STM32F411RETx --log-file debug.log
```

### probe-rs 配置文件

#### 项目配置 (.cargo/config.toml)

```toml
[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32F411RETx"

[build]
target = "thumbv7em-none-eabihf"

[env]
# 启用RTT
DEFMT_LOG = "debug"
```

#### 高级配置 (Probe.toml)

```toml
[default.general]
chip = "STM32F411RETx"
protocol = "Swd"
speed = 4000

[default.flashing]
halt_afterwards = false
restore_unwritten_bytes = false

[default.reset]
halt_afterwards = true

[default.rtt]
enabled = true
channels = [
    { up = 0, down = 0, name = "Terminal", up_mode = "NoBlockSkip", format = "Defmt" },
]
```

## OpenOCD 配置和使用

### 安装 OpenOCD

```bash
# Ubuntu/Debian
sudo apt-get install openocd

# macOS
brew install openocd

# Windows (使用MSYS2)
pacman -S mingw-w64-x86_64-openocd
```

### OpenOCD 配置文件

#### STM32配置示例

```tcl
# openocd.cfg
source [find interface/stlink.cfg]
source [find target/stm32f4x.cfg]

# 设置工作区域
$_TARGETNAME configure -work-area-phys 0x20000000 -work-area-size 0x8000

# 配置Flash
flash bank $_FLASHNAME stm32f2x 0x08000000 0 0 0 $_TARGETNAME

# 初始化
init
reset halt
```

#### ESP32配置示例

```tcl
# esp32_openocd.cfg
source [find interface/ftdi/esp32_devkitj_v1.cfg]
source [find target/esp32.cfg]

# 设置适配器速度
adapter speed 20000

init
reset halt
```

### OpenOCD 使用

#### 1. 启动OpenOCD服务器

```bash
# 使用配置文件启动
openocd -f openocd.cfg

# 使用内置配置
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

# 指定端口
openocd -f openocd.cfg -c "gdb_port 3333" -c "telnet_port 4444"
```

#### 2. 连接和控制

```bash
# 通过telnet连接
telnet localhost 4444

# 常用命令
> reset halt          # 重置并停止
> flash write_image erase firmware.elf
> reset run           # 重置并运行
> shutdown            # 关闭OpenOCD
```

## GDB 调试器配置

### 安装 GDB

```bash
# ARM GDB
# Ubuntu/Debian
sudo apt-get install gdb-multiarch

# macOS
brew install arm-none-eabi-gdb

# 或使用通用版本
brew install gdb
```

### GDB 配置文件

#### .gdbinit 配置

```gdb
# .gdbinit
set confirm off
set history save on
set history size 10000
set history filename ~/.gdb_history

# 连接到OpenOCD
define connect
    target extended-remote localhost:3333
    monitor reset halt
    load
    monitor reset init
end

# 常用别名
define restart
    monitor reset halt
    continue
end

# 自动加载符号
set auto-load safe-path /
```

### GDB 使用技巧

#### 1. 基本调试命令

```gdb
# 连接到目标
(gdb) target extended-remote localhost:3333

# 加载程序
(gdb) load

# 设置断点
(gdb) break main
(gdb) break src/main.rs:42

# 运行程序
(gdb) continue
(gdb) step
(gdb) next

# 查看变量
(gdb) print variable_name
(gdb) info locals
(gdb) info registers

# 查看内存
(gdb) x/16x 0x20000000
(gdb) x/s string_ptr
```

#### 2. 高级调试技巧

```gdb
# 查看调用栈
(gdb) backtrace
(gdb) frame 1

# 监视变量
(gdb) watch variable_name
(gdb) rwatch read_only_var

# 条件断点
(gdb) break main if argc > 1

# 查看汇编代码
(gdb) disassemble main
(gdb) layout asm

# 查看寄存器
(gdb) info registers
(gdb) print $pc
(gdb) print $sp
```

## RTT (Real Time Transfer) 配置

### 什么是 RTT

RTT是SEGGER开发的实时数据传输技术，具有以下优势：
- 高性能，低延迟
- 不占用串口资源
- 支持双向通信
- 不影响实时性

### 在Rust中使用RTT

#### 1. 添加依赖

```toml
# Cargo.toml
[dependencies]
rtt-target = { version = "0.4", features = ["cortex-m"] }
panic-rtt-target = { version = "0.1", features = ["cortex-m"] }

# 或使用defmt
defmt = "0.3"
defmt-rtt = "0.4"
panic-probe = { version = "0.3", features = ["print-defmt"] }
```

#### 2. 代码示例

```rust
// 使用rtt-target
use rtt_target::{rprintln, rtt_init_print};

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("Hello, RTT!");
    
    let mut counter = 0;
    loop {
        rprintln!("Counter: {}", counter);
        counter += 1;
        
        // 延时
        cortex_m::asm::delay(1_000_000);
    }
}

// 使用defmt
use defmt_rtt as _;
use panic_probe as _;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Starting application");
    
    loop {
        defmt::debug!("Loop iteration");
        cortex_m::asm::delay(1_000_000);
    }
}
```

#### 3. 查看RTT输出

```bash
# 使用probe-rs
probe-rs rtt --chip STM32F411RETx

# 使用JLinkRTTViewer (如果有J-Link)
JLinkRTTViewer

# 使用OpenOCD + telnet
telnet localhost 4444
> rtt setup 0x20000000 0x1000 "SEGGER RTT"
> rtt start
> rtt channels
```

## 调试器硬件配置

### 支持的调试器

#### 1. ST-Link (STM32官方)

```bash
# 检查ST-Link连接
probe-rs list

# 使用ST-Link调试
probe-rs run --probe STLink --chip STM32F411RETx firmware.elf
```

#### 2. J-Link (SEGGER)

```bash
# J-Link配置
probe-rs run --probe JLink --chip STM32F411RETx firmware.elf

# 使用J-Link RTT
JLinkExe -device STM32F411RE -if SWD -speed 4000
```

#### 3. CMSIS-DAP

```bash
# 通用CMSIS-DAP调试器
probe-rs run --probe CMSIS-DAP --chip STM32F411RETx firmware.elf
```

### 调试器连接

#### SWD连接 (推荐)

| 信号 | 功能 | 必需 |
|------|------|------|
| SWDIO | 数据线 | 是 |
| SWCLK | 时钟线 | 是 |
| GND | 地线 | 是 |
| VCC | 电源 | 可选 |
| NRST | 复位 | 可选 |

#### JTAG连接

| 信号 | 功能 | 必需 |
|------|------|------|
| TMS | 模式选择 | 是 |
| TCK | 时钟 | 是 |
| TDI | 数据输入 | 是 |
| TDO | 数据输出 | 是 |
| TRST | 复位 | 可选 |

## 高级调试技巧

### 1. 多核调试

```bash
# OpenOCD多核配置
# 在openocd.cfg中
set _CORES 2
set _CORE0_NAME cpu0
set _CORE1_NAME cpu1

# GDB中切换核心
(gdb) info threads
(gdb) thread 2
```

### 2. 实时跟踪

```rust
// 使用ITM (Instrumentation Trace Macrocell)
use cortex_m::itm;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut itm = cortex_m::Peripherals::take().unwrap().ITM;
    
    loop {
        itm::write_str(&mut itm.stim[0], "Hello from ITM\n");
        cortex_m::asm::delay(1_000_000);
    }
}
```

### 3. 性能分析

```bash
# 使用probe-rs进行性能分析
probe-rs trace --chip STM32F411RETx --trace-file trace.bin

# 分析跟踪数据
probe-rs trace analyze trace.bin
```

### 4. 内存分析

```gdb
# GDB中分析内存使用
(gdb) info mem
(gdb) x/100x $sp
(gdb) monitor mdw 0x20000000 100

# 查看堆栈使用
(gdb) info stack
(gdb) print $sp - $stack_base
```

## 调试最佳实践

### 1. 调试符号优化

```toml
# Cargo.toml
[profile.dev]
debug = true
opt-level = 1  # 轻微优化，保持调试信息

[profile.release]
debug = true   # 发布版本也保留调试信息
lto = true
```

### 2. 断言和检查

```rust
// 使用断言进行调试
debug_assert!(condition, "Condition failed: {}", value);

// 使用defmt进行结构化日志
defmt::trace!("Function entry: param={}", param);
defmt::debug!("Variable state: x={}, y={}", x, y);
defmt::info!("Operation completed successfully");
defmt::warn!("Unusual condition detected");
defmt::error!("Critical error occurred");
```

### 3. 错误处理调试

```rust
use defmt::unwrap;

// 调试友好的错误处理
let result = risky_operation().map_err(|e| {
    defmt::error!("Operation failed: {:?}", e);
    e
})?;

// 使用unwrap!宏获得更好的错误信息
let value = unwrap!(some_option, "Expected value was None");
```

## 常见问题解决

### 1. 连接问题

```bash
# 检查调试器连接
probe-rs list

# 重置调试器
probe-rs reset --chip STM32F411RETx

# 检查权限 (Linux)
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 2. 烧录失败

```bash
# 解锁芯片 (如果被保护)
probe-rs erase --chip STM32F411RETx

# 强制烧录
probe-rs run --chip STM32F411RETx --force-unlock firmware.elf

# 检查电源和连接
probe-rs info --chip STM32F411RETx
```

### 3. 调试信息缺失

```toml
# 确保调试信息生成
[profile.dev]
debug = 2      # 完整调试信息
strip = false  # 不剥离符号

[profile.release]
debug = 1      # 行号信息
strip = false
```

## 调试工作流程

### 典型调试流程

```mermaid
graph TD
    A[编写代码] --> B[编译项目]
    B --> C[烧录到硬件]
    C --> D[启动调试会话]
    D --> E[设置断点]
    E --> F[运行程序]
    F --> G{遇到问题?}
    G -->|是| H[分析问题]
    G -->|否| I[测试完成]
    H --> J[修改代码]
    J --> A
```

### 调试检查清单

- [ ] 硬件连接正确
- [ ] 调试器驱动已安装
- [ ] 目标芯片配置正确
- [ ] 编译包含调试信息
- [ ] RTT/串口输出正常
- [ ] 断点设置合理
- [ ] 内存映射正确

## 下一步

掌握调试工具后，建议：

1. 学习[IDE配置](./04-ide-setup.md)集成调试环境
2. 实践[项目模板](./05-project-templates.md)中的调试配置
3. 深入了解[交叉编译](./06-cross-compilation.md)优化

---

**现在你已经掌握了强大的嵌入式调试技能！** 🔍
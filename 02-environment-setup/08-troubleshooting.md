# 故障排除指南

本章提供嵌入式Rust开发中常见问题的解决方案和调试技巧，帮助开发者快速定位和解决问题。

## 学习目标

- 掌握常见编译错误的解决方法
- 学会调试硬件连接问题
- 了解性能问题的诊断和优化
- 掌握工具链问题的排查技巧

## 编译和构建问题

### 工具链问题

#### 目标架构不匹配

**问题症状：**
```
error[E0463]: can't find crate for `std`
```

**解决方案：**
```bash
# 检查已安装的目标
rustup target list --installed

# 安装缺失的目标
rustup target add thumbv7em-none-eabihf

# 验证目标配置
cat .cargo/config.toml
```

#### 链接器错误

**问题症状：**
```
= note: rust-lld: error: unable to find library -lgcc
```

**解决方案：**
```bash
# 安装ARM工具链
# macOS
brew install arm-none-eabi-gcc

# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi

# 检查链接器配置
[target.thumbv7em-none-eabihf]
linker = "arm-none-eabi-gcc"
```

### 内存配置问题

#### 栈溢出

**问题症状：**
```
HardFault exception
```

**诊断方法：**
```rust
// 启用栈溢出检测
#[no_mangle]
pub unsafe extern "C" fn HardFault() -> ! {
    // 检查栈指针
    let sp: u32;
    asm!("mov {}, sp", out(reg) sp);
    
    // 记录错误信息
    panic!("HardFault at SP: 0x{:08x}", sp);
}
```

**解决方案：**
```rust
// memory.x 中增加栈大小
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}

_stack_size = 8K; /* 增加栈大小 */
```

#### 堆内存不足

**问题症状：**
```
memory allocation of 1024 bytes failed
```

**解决方案：**
```rust
// 使用heapless容器
use heapless::Vec;

// 替代std::vec::Vec
let mut buffer: Vec<u8, 1024> = Vec::new();

// 或者配置全局分配器
use linked_list_allocator::LockedHeap;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();
```

## 硬件连接问题

### 调试器连接失败

#### probe-rs连接问题

**问题症状：**
```
Error: Probe could not be opened
```

**诊断步骤：**
```bash
# 列出可用的探针
probe-rs list

# 检查USB权限 (Linux)
sudo usermod -a -G dialout $USER
sudo udevadm control --reload-rules

# 检查驱动程序 (Windows)
# 使用Zadig安装WinUSB驱动
```

#### OpenOCD连接问题

**问题症状：**
```
Error: unable to open ftdi device with vid 0403, pid 6014
```

**解决方案：**
```bash
# 检查设备
lsusb | grep -i ftdi

# 配置OpenOCD
# openocd.cfg
source [find interface/stlink.cfg]
source [find target/stm32f4x.cfg]

# 测试连接
openocd -f openocd.cfg
```

### 串口通信问题

#### 波特率不匹配

**问题症状：**
乱码输出或无输出

**解决方案：**
```rust
// 确保波特率匹配
let serial = Serial::new(
    dp.USART2,
    (tx_pin, rx_pin),
    Config::default().baudrate(115200.bps()),
    clocks,
);

// 检查时钟配置
let clocks = rcc.cfgr
    .use_hse(8.mhz())
    .sysclk(84.mhz())
    .freeze();
```

## 运行时问题

### 异常和错误处理

#### Panic处理

**问题症状：**
程序突然停止，无错误信息

**调试方法：**
```rust
// 自定义panic处理器
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // 通过RTT输出panic信息
    rprintln!("Panic: {}", info);
    
    // 或通过串口输出
    if let Some(mut serial) = SERIAL.try_lock() {
        writeln!(serial, "Panic: {}", info).ok();
    }
    
    loop {}
}
```

#### 中断处理问题

**问题症状：**
中断不触发或频繁触发

**解决方案：**
```rust
// 检查中断配置
#[interrupt]
fn TIM2() {
    // 清除中断标志
    unsafe {
        (*TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit());
    }
    
    // 处理中断逻辑
    handle_timer_interrupt();
}

// 确保中断使能
timer.listen(Event::Update);
unsafe {
    NVIC::unmask(Interrupt::TIM2);
}
```

### 性能问题

#### 高CPU使用率

**诊断方法：**
```rust
// 使用性能计数器
use cortex_m::peripheral::DWT;

fn measure_cycles<F: FnOnce()>(f: F) -> u32 {
    let start = DWT::cycle_count();
    f();
    DWT::cycle_count().wrapping_sub(start)
}

// 测量函数执行时间
let cycles = measure_cycles(|| {
    expensive_function();
});
```

**优化策略：**
```rust
// 使用编译器优化
#[inline(always)]
fn hot_path_function() {
    // 关键路径代码
}

// 避免不必要的分配
use heapless::pool::{Pool, Node};

// 使用静态内存池
static mut MEMORY: [Node<[u8; 64]>; 16] = [Node::new(); 16];
static POOL: Pool<[u8; 64]> = Pool::new();
```

## 调试技巧

### RTT调试

#### 配置RTT

```rust
// Cargo.toml
[dependencies]
rtt-target = { version = "0.3", features = ["cortex-m"] }

// main.rs
use rtt_target::{rprintln, rtt_init_print};

fn main() -> ! {
    rtt_init_print!();
    rprintln!("Hello, RTT!");
    
    loop {
        rprintln!("Debug value: {}", sensor_reading);
    }
}
```

#### RTT查看器

```bash
# 使用probe-rs
probe-rs rtt --chip STM32F401RETx

# 使用JLinkRTTViewer
JLinkRTTViewer
```

### GDB调试

#### 启动GDB会话

```bash
# 启动OpenOCD
openocd -f openocd.cfg

# 在另一个终端启动GDB
arm-none-eabi-gdb target/thumbv7em-none-eabihf/debug/your_app

# GDB命令
(gdb) target remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) break main
(gdb) continue
```

#### 有用的GDB命令

```gdb
# 查看寄存器
info registers

# 查看内存
x/16x 0x20000000

# 查看栈回溯
backtrace

# 单步执行
step
next

# 查看变量
print variable_name
```

## 工具问题

### Cargo问题

#### 依赖冲突

**问题症状：**
```
error: failed to select a version for the requirement
```

**解决方案：**
```bash
# 清理构建缓存
cargo clean

# 更新依赖
cargo update

# 检查依赖树
cargo tree

# 使用特定版本
[dependencies]
embedded-hal = "=0.2.7"
```

#### 构建脚本问题

**问题症状：**
```
error: failed to run custom build command
```

**解决方案：**
```rust
// build.rs
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // 检查环境变量
    if env::var("TARGET").unwrap().starts_with("thumbv") {
        println!("cargo:rustc-link-arg=-Tlink.x");
    }
    
    // 生成配置文件
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("config.rs"))
        .unwrap()
        .write_all(include_bytes!("config.rs.in"))
        .unwrap();
}
```

### IDE和编辑器问题

#### VS Code配置

**问题症状：**
代码补全不工作，错误高亮不准确

**解决方案：**
```json
// .vscode/settings.json
{
    "rust-analyzer.cargo.target": "thumbv7em-none-eabihf",
    "rust-analyzer.checkOnSave.allTargets": false,
    "rust-analyzer.cargo.features": ["rt"],
    "rust-analyzer.cargo.noDefaultFeatures": true
}
```

## 环境问题

### 路径和权限

#### Windows路径问题

**问题症状：**
```
error: linker `link.exe` not found
```

**解决方案：**
```bash
# 安装Visual Studio Build Tools
# 或设置环境变量
set PATH=%PATH%;C:\Program Files (x86)\Microsoft Visual Studio\2019\BuildTools\VC\Tools\MSVC\14.29.30133\bin\Hostx64\x64
```

#### Linux权限问题

**问题症状：**
```
Permission denied (os error 13)
```

**解决方案：**
```bash
# 添加用户到dialout组
sudo usermod -a -G dialout $USER

# 设置udev规则
sudo cp 99-probe-rs.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 诊断工具

### 系统信息收集

```bash
#!/bin/bash
# debug_info.sh - 收集调试信息

echo "=== Rust工具链信息 ==="
rustc --version
cargo --version
rustup show

echo "=== 目标架构 ==="
rustup target list --installed

echo "=== 硬件连接 ==="
probe-rs list
lsusb | grep -E "(STM|FTDI|Segger)"

echo "=== 项目配置 ==="
cat Cargo.toml
cat .cargo/config.toml

echo "=== 构建信息 ==="
cargo check --verbose
```

### 日志分析

```rust
// 结构化日志记录
use log::{info, warn, error};

fn main() -> ! {
    // 初始化日志
    init_logger();
    
    info!("System starting");
    
    match initialize_hardware() {
        Ok(_) => info!("Hardware initialized successfully"),
        Err(e) => error!("Hardware initialization failed: {:?}", e),
    }
    
    loop {
        // 主循环
    }
}
```

## 最佳实践

### 预防性措施

1. **版本锁定**：在生产环境中锁定依赖版本
2. **持续集成**：设置自动化测试和构建
3. **文档记录**：记录已知问题和解决方案
4. **环境隔离**：使用Docker或虚拟环境

### 调试策略

1. **分层调试**：从简单到复杂逐步排查
2. **二分法**：通过注释代码定位问题
3. **日志记录**：添加详细的调试信息
4. **版本回退**：回到已知工作的版本

## 获取帮助

### 社区资源

- [Rust Embedded Working Group](https://github.com/rust-embedded/wg)
- [embedded.rs Discord](https://discord.gg/rust-embedded)
- [Stack Overflow](https://stackoverflow.com/questions/tagged/rust+embedded)

### 报告问题

提交问题时请包含：
- 完整的错误信息
- 最小可重现示例
- 环境信息（操作系统、Rust版本等）
- 已尝试的解决方案

## 总结

本章介绍了嵌入式Rust开发中的常见问题和解决方案，包括编译错误、硬件连接问题、运行时错误和工具配置问题。通过系统性的故障排除方法和预防性措施，可以大大提高开发效率和代码质量。

## 下一步

- 实践不同的调试技巧
- 建立项目特定的故障排除文档
- 参与社区讨论和问题解答
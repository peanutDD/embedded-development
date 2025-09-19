# 仿真器和测试环境

## 概述

仿真器和测试环境是嵌入式开发中不可或缺的工具，它们允许开发者在没有实际硬件的情况下进行开发、调试和测试。本章节将详细介绍各种仿真工具的配置和使用，以及如何建立完整的测试环境。

## 仿真器类型

### 1. 指令集仿真器 (ISS)

指令集仿真器模拟处理器的指令执行，适合算法验证和基础功能测试。

#### QEMU

QEMU是最流行的开源仿真器，支持多种架构。

**安装QEMU:**

```bash
# macOS
brew install qemu

# Ubuntu/Debian
sudo apt-get install qemu-system-arm qemu-system-misc

# 验证安装
qemu-system-arm --version
```

**ARM Cortex-M仿真配置:**

```bash
# 创建QEMU配置文件
# qemu-config.toml
[target.thumbv7m-none-eabi]
runner = "qemu-system-arm -cpu cortex-m3 -machine lm3s6965evb -nographic -semihosting-config enable=on,target=native -kernel"

[target.thumbv7em-none-eabihf]
runner = "qemu-system-arm -cpu cortex-m4 -machine netduinoplus2 -nographic -semihosting-config enable=on,target=native -kernel"
```

**QEMU运行示例:**

```bash
# 编译并在QEMU中运行
cargo build --target thumbv7m-none-eabi
qemu-system-arm \
    -cpu cortex-m3 \
    -machine lm3s6965evb \
    -nographic \
    -semihosting-config enable=on,target=native \
    -kernel target/thumbv7m-none-eabi/debug/my-project
```

#### Renode

Renode是专为嵌入式系统设计的仿真平台，支持完整的系统仿真。

**安装Renode:**

```bash
# macOS
brew install --cask renode

# Ubuntu/Debian
wget https://github.com/renode/renode/releases/download/v1.14.0/renode_1.14.0_amd64.deb
sudo dpkg -i renode_1.14.0_amd64.deb
```

**Renode脚本示例:**

```python
# stm32f4.resc
using sysbus
mach create "stm32f4"

machine LoadPlatformDescription @platforms/cpus/stm32f4.repl

sysbus LoadELF @target/thumbv7em-none-eabihf/debug/my-project

# 设置UART重定向
showAnalyzer sysbus.usart2

# 启动仿真
start
```

**运行Renode仿真:**

```bash
# 启动Renode
renode stm32f4.resc

# 或在Renode控制台中
(monitor) include @stm32f4.resc
(monitor) start
```

### 2. 周期精确仿真器

周期精确仿真器模拟处理器的时钟周期，适合性能分析和时序验证。

#### gem5

gem5是学术界广泛使用的周期精确仿真器。

```bash
# 安装gem5
git clone https://github.com/gem5/gem5.git
cd gem5
scons build/ARM/gem5.opt -j$(nproc)
```

#### 商业仿真器

- **ARM Fast Models**: ARM官方仿真器
- **Cadence Palladium**: 硬件加速仿真
- **Synopsys ZeBu**: 企业级仿真平台

## 单元测试环境

### 1. 主机测试 (Host Testing)

在主机上运行测试，适合纯逻辑测试。

**配置主机测试:**

```toml
# Cargo.toml
[lib]
name = "embedded_lib"
test = true

[[bin]]
name = "main"
test = false

[dev-dependencies]
std = { package = "std", version = "1.0", optional = true }

[features]
default = []
std = ["dep:std"]
```

**测试代码示例:**

```rust
// src/lib.rs
#![cfg_attr(not(test), no_std)]

pub fn add(a: i32, b: i32) -> i32 {
    a + b
}

pub fn multiply(a: i32, b: i32) -> i32 {
    a * b
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_add() {
        assert_eq!(add(2, 3), 5);
        assert_eq!(add(-1, 1), 0);
    }

    #[test]
    fn test_multiply() {
        assert_eq!(multiply(3, 4), 12);
        assert_eq!(multiply(0, 5), 0);
    }
}
```

**运行主机测试:**

```bash
# 运行所有测试
cargo test

# 运行特定测试
cargo test test_add

# 详细输出
cargo test -- --nocapture
```

### 2. 目标测试 (Target Testing)

在目标硬件或仿真器上运行测试。

#### defmt-test

defmt-test允许在嵌入式目标上运行测试。

**安装和配置:**

```bash
cargo install defmt-test
```

```toml
# Cargo.toml
[dev-dependencies]
defmt = "0.3"
defmt-rtt = "0.4"
defmt-test = "0.3"
panic-probe = { version = "0.3", features = ["print-defmt"] }

[[test]]
name = "integration"
harness = false
```

**测试代码:**

```rust
// tests/integration.rs
#![no_std]
#![no_main]

use defmt_test as _;
use panic_probe as _;

#[defmt_test::tests]
mod tests {
    use defmt::assert_eq;
    
    #[test]
    fn test_basic_math() {
        assert_eq!(2 + 2, 4);
    }
    
    #[test]
    fn test_embedded_function() {
        let result = my_embedded_lib::process_data(&[1, 2, 3]);
        assert_eq!(result, 6);
    }
}
```

**运行目标测试:**

```bash
# 在硬件上运行测试
cargo test --target thumbv7em-none-eabihf

# 在QEMU中运行测试
QEMU_AUDIO_DRV=none cargo test --target thumbv7m-none-eabi
```

### 3. 模拟测试 (Mock Testing)

使用模拟对象测试硬件抽象层。

**embedded-hal-mock示例:**

```toml
# Cargo.toml
[dev-dependencies]
embedded-hal-mock = "0.10"
```

```rust
// tests/mock_test.rs
#[cfg(test)]
mod tests {
    use embedded_hal_mock::spi::{Mock as SpiMock, Transaction as SpiTransaction};
    use my_driver::SensorDriver;

    #[test]
    fn test_sensor_read() {
        // 设置SPI模拟
        let expectations = [
            SpiTransaction::write_vec(vec![0x80]), // 读命令
            SpiTransaction::read_vec(vec![0x42]), // 期望的响应
        ];
        let spi = SpiMock::new(&expectations);
        
        // 创建驱动实例
        let mut sensor = SensorDriver::new(spi);
        
        // 执行测试
        let result = sensor.read_temperature().unwrap();
        assert_eq!(result, 66); // 0x42 = 66
        
        // 验证所有期望都被满足
        sensor.spi.done();
    }
}
```

## 集成测试环境

### 1. 硬件在环测试 (HIL)

硬件在环测试结合真实硬件和仿真环境。

**HIL测试框架:**

```rust
// tests/hil_test.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{prelude::*, stm32};

struct TestHarness {
    // 测试硬件接口
    led: stm32f4xx_hal::gpio::gpioa::PA5<stm32f4xx_hal::gpio::Output<stm32f4xx_hal::gpio::PushPull>>,
    button: stm32f4xx_hal::gpio::gpioc::PC13<stm32f4xx_hal::gpio::Input<stm32f4xx_hal::gpio::PullUp>>,
}

impl TestHarness {
    fn new() -> Self {
        let dp = stm32::Peripherals::take().unwrap();
        let gpioa = dp.GPIOA.split();
        let gpioc = dp.GPIOC.split();
        
        Self {
            led: gpioa.pa5.into_push_pull_output(),
            button: gpioc.pc13.into_pull_up_input(),
        }
    }
    
    fn test_led_control(&mut self) -> bool {
        self.led.set_high();
        // 验证LED状态
        true
    }
    
    fn test_button_input(&self) -> bool {
        self.button.is_low()
    }
}

#[entry]
fn main() -> ! {
    let mut harness = TestHarness::new();
    
    // 运行测试
    let led_test = harness.test_led_control();
    let button_test = harness.test_button_input();
    
    // 报告结果
    if led_test && button_test {
        // 测试通过
        loop {
            harness.led.toggle();
            cortex_m::asm::delay(1_000_000);
        }
    } else {
        // 测试失败
        panic!("HIL test failed");
    }
}
```

### 2. 自动化测试流水线

**GitHub Actions配置:**

```yaml
# .github/workflows/test.yml
name: Embedded Tests

on: [push, pull_request]

jobs:
  host-tests:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    
    - name: Install Rust
      uses: actions-rs/toolchain@v1
      with:
        toolchain: stable
        override: true
    
    - name: Run host tests
      run: cargo test --lib
  
  target-tests:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        target: [thumbv7m-none-eabi, thumbv7em-none-eabihf]
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Install Rust and targets
      uses: actions-rs/toolchain@v1
      with:
        toolchain: stable
        target: ${{ matrix.target }}
        override: true
    
    - name: Install QEMU
      run: |
        sudo apt-get update
        sudo apt-get install qemu-system-arm
    
    - name: Install defmt-test
      run: cargo install defmt-test
    
    - name: Run target tests
      run: cargo test --target ${{ matrix.target }}
      env:
        QEMU_AUDIO_DRV: none

  integration-tests:
    runs-on: self-hosted  # 需要连接硬件的runner
    if: github.event_name == 'push' && github.ref == 'refs/heads/main'
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Flash and test on hardware
      run: |
        cargo build --release --target thumbv7em-none-eabihf
        probe-rs run --chip STM32F411RETx target/thumbv7em-none-eabihf/release/integration-test
```

## 性能测试和基准测试

### 1. 微基准测试

```rust
// benches/micro_bench.rs
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m::peripheral::DWT;

#[entry]
fn main() -> ! {
    let mut core = cortex_m::Peripherals::take().unwrap();
    
    // 启用DWT计数器
    core.DWT.enable_cycle_counter();
    
    // 基准测试
    benchmark_function(&mut core.DWT);
    
    loop {}
}

fn benchmark_function(dwt: &mut DWT) {
    const ITERATIONS: u32 = 1000;
    
    // 预热
    for _ in 0..100 {
        test_function();
    }
    
    // 测量
    let start = DWT::cycle_count();
    
    for _ in 0..ITERATIONS {
        test_function();
    }
    
    let end = DWT::cycle_count();
    let total_cycles = end.wrapping_sub(start);
    let avg_cycles = total_cycles / ITERATIONS;
    
    // 输出结果（通过RTT或其他方式）
    rtt_target::rprintln!("Average cycles per iteration: {}", avg_cycles);
}

#[inline(never)]
fn test_function() {
    // 被测试的函数
    let mut sum = 0u32;
    for i in 0..100 {
        sum = sum.wrapping_add(i);
    }
    // 防止编译器优化
    unsafe { core::ptr::write_volatile(&mut sum, sum) };
}
```

### 2. 内存使用分析

```rust
// src/memory_analysis.rs
#![no_std]

use linked_list_allocator::LockedHeap;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// 内存使用统计
pub struct MemoryStats {
    pub heap_used: usize,
    pub heap_free: usize,
    pub stack_used: usize,
}

impl MemoryStats {
    pub fn collect() -> Self {
        let heap_used = ALLOCATOR.lock().used();
        let heap_free = ALLOCATOR.lock().free();
        let stack_used = estimate_stack_usage();
        
        Self {
            heap_used,
            heap_free,
            stack_used,
        }
    }
}

fn estimate_stack_usage() -> usize {
    extern "C" {
        static mut _stack_start: u32;
        static mut _stack_end: u32;
    }
    
    unsafe {
        let stack_start = &_stack_start as *const u32 as usize;
        let stack_end = &_stack_end as *const u32 as usize;
        let current_sp = cortex_m::register::msp::read() as usize;
        
        stack_start - current_sp
    }
}

// 内存泄漏检测
pub fn check_memory_leaks() -> bool {
    let stats_before = MemoryStats::collect();
    
    // 执行一些操作
    {
        let _vec = heapless::Vec::<u32, 100>::new();
        // 操作...
    }
    
    let stats_after = MemoryStats::collect();
    
    // 检查内存是否正确释放
    stats_before.heap_used == stats_after.heap_used
}
```

### 3. 实时性能监控

```rust
// src/performance_monitor.rs
use cortex_m::peripheral::{DWT, ITM};
use rtt_target::{rprintln, rtt_init_print};

pub struct PerformanceMonitor {
    dwt: DWT,
    last_cycle_count: u32,
}

impl PerformanceMonitor {
    pub fn new(mut dwt: DWT) -> Self {
        dwt.enable_cycle_counter();
        
        Self {
            dwt,
            last_cycle_count: DWT::cycle_count(),
        }
    }
    
    pub fn start_measurement(&mut self, name: &str) {
        self.last_cycle_count = DWT::cycle_count();
        rprintln!("Starting measurement: {}", name);
    }
    
    pub fn end_measurement(&mut self, name: &str) {
        let current = DWT::cycle_count();
        let elapsed = current.wrapping_sub(self.last_cycle_count);
        rprintln!("Measurement {}: {} cycles", name, elapsed);
    }
    
    pub fn measure<F, R>(&mut self, name: &str, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        self.start_measurement(name);
        let result = f();
        self.end_measurement(name);
        result
    }
}

// 使用示例
pub fn performance_test() {
    let dwt = unsafe { cortex_m::Peripherals::steal().DWT };
    let mut monitor = PerformanceMonitor::new(dwt);
    
    monitor.measure("matrix_multiply", || {
        matrix_multiply_3x3();
    });
    
    monitor.measure("fft_calculation", || {
        calculate_fft();
    });
}
```

## 调试和分析工具

### 1. RTT (Real-Time Transfer)

RTT提供高速、低延迟的调试输出。

**配置RTT:**

```toml
# Cargo.toml
[dependencies]
rtt-target = { version = "0.4", features = ["cortex-m"] }
panic-rtt-target = { version = "0.1", features = ["cortex-m"] }
```

```rust
// src/main.rs
use rtt_target::{rprintln, rtt_init_print};
use panic_rtt_target as _;

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("RTT initialized");
    
    loop {
        rprintln!("Debug message: {}", 42);
        cortex_m::asm::delay(1_000_000);
    }
}
```

**RTT查看器:**

```bash
# 使用probe-rs
probe-rs rtt --chip STM32F411RETx

# 使用JLinkRTTViewer
JLinkRTTViewer
```

### 2. ITM (Instrumentation Trace Macrocell)

ITM提供硬件级别的跟踪功能。

```rust
// src/itm_debug.rs
use cortex_m::peripheral::ITM;

pub struct ItmLogger {
    itm: ITM,
}

impl ItmLogger {
    pub fn new(itm: ITM) -> Self {
        Self { itm }
    }
    
    pub fn log(&mut self, port: u8, data: &[u8]) {
        for &byte in data {
            while !self.itm.stim[port as usize].is_fifo_ready() {}
            self.itm.stim[port as usize].write_u8(byte);
        }
    }
    
    pub fn log_str(&mut self, port: u8, s: &str) {
        self.log(port, s.as_bytes());
    }
}
```

### 3. 静态分析工具

**Clippy配置:**

```toml
# clippy.toml
avoid-breaking-exported-api = false
msrv = "1.70"

# 嵌入式特定lint
allow = [
    "clippy::empty_loop",           # 嵌入式常见模式
    "clippy::needless_loop",        # 主循环
]

deny = [
    "clippy::unwrap_used",          # 避免panic
    "clippy::expect_used",          # 避免panic
    "clippy::panic",                # 避免panic
    "clippy::float_arithmetic",     # 浮点运算检查
]
```

**MIRI检查:**

```bash
# 安装MIRI
rustup component add miri

# 运行MIRI检查
cargo +nightly miri test
```

## 测试策略和最佳实践

### 1. 测试金字塔

```
    /\
   /  \     集成测试 (少量)
  /____\    
 /      \   功能测试 (适量)
/________\  单元测试 (大量)
```

### 2. 测试分类

**按范围分类:**
- **单元测试**: 测试单个函数或模块
- **集成测试**: 测试模块间交互
- **系统测试**: 测试完整系统功能

**按环境分类:**
- **主机测试**: 在开发机上运行
- **仿真测试**: 在仿真器中运行
- **硬件测试**: 在真实硬件上运行

### 3. 测试组织

```
tests/
├── unit/                    # 单元测试
│   ├── math_utils.rs
│   └── protocol_parser.rs
├── integration/             # 集成测试
│   ├── sensor_driver.rs
│   └── communication.rs
├── hardware/                # 硬件测试
│   ├── gpio_test.rs
│   └── spi_test.rs
└── common/                  # 测试工具
    ├── mock_hardware.rs
    └── test_utils.rs
```

### 4. 持续集成策略

```yaml
# 测试矩阵
strategy:
  matrix:
    test-type: [unit, integration, hardware]
    target: [host, thumbv7em-none-eabihf]
    exclude:
      - test-type: hardware
        target: host
```

## 故障排除

### 1. 常见仿真问题

**QEMU启动失败:**
```bash
# 检查QEMU版本
qemu-system-arm --version

# 使用详细输出
qemu-system-arm -cpu cortex-m3 -machine lm3s6965evb -d cpu,exec -nographic
```

**Renode连接问题:**
```python
# 检查平台描述文件
(monitor) showAvailablePlatforms
(monitor) help machine
```

### 2. 测试调试技巧

```rust
// 测试调试宏
#[cfg(test)]
macro_rules! debug_test {
    ($($arg:tt)*) => {
        #[cfg(feature = "test-debug")]
        println!($($arg)*);
    };
}

// 条件编译测试
#[cfg(all(test, feature = "hardware-test"))]
mod hardware_tests {
    // 只在硬件测试时编译
}
```

### 3. 性能测试问题

```rust
// 防止编译器优化
#[inline(never)]
fn benchmark_target() {
    // 测试代码
}

// 使用黑盒函数
fn black_box<T>(dummy: T) -> T {
    unsafe { core::ptr::read_volatile(&dummy) }
}
```

## 下一步

掌握仿真和测试后，建议：

1. 学习[故障排除指南](./08-troubleshooting.md)解决常见问题
2. 实践建立完整的测试流水线
3. 探索高级调试和分析技术

---

**现在你拥有了完整的嵌入式测试和仿真环境！** 🧪
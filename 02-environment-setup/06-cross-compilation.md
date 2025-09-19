# 交叉编译配置

## 概述

交叉编译是嵌入式开发的核心概念，指在一个平台（主机）上编译出能在另一个平台（目标设备）上运行的代码。本章节将详细介绍Rust嵌入式开发中的交叉编译配置、优化技巧和常见问题解决方案。

## 交叉编译基础

### 1. 目标三元组 (Target Triple)

目标三元组定义了编译目标的架构、供应商和操作系统：

```
架构-供应商-系统-ABI
```

#### 常见嵌入式目标

| 目标三元组 | 架构 | 描述 | 典型芯片 |
|------------|------|------|----------|
| `thumbv6m-none-eabi` | ARM Cortex-M0/M0+ | 无操作系统，软浮点 | STM32F0, nRF51 |
| `thumbv7m-none-eabi` | ARM Cortex-M3 | 无操作系统，软浮点 | STM32F1, LPC1768 |
| `thumbv7em-none-eabi` | ARM Cortex-M4/M7 | 无操作系统，软浮点 | STM32F4, STM32F7 |
| `thumbv7em-none-eabihf` | ARM Cortex-M4F/M7F | 无操作系统，硬浮点 | STM32F4, STM32H7 |
| `thumbv8m.base-none-eabi` | ARM Cortex-M23 | ARMv8-M基础，软浮点 | STM32L5 |
| `thumbv8m.main-none-eabi` | ARM Cortex-M33 | ARMv8-M主线，软浮点 | STM32L5, nRF9160 |
| `riscv32imc-unknown-none-elf` | RISC-V 32位 | 无操作系统 | ESP32-C3, GD32VF103 |
| `riscv64gc-unknown-none-elf` | RISC-V 64位 | 无操作系统 | SiFive FU740 |

### 2. 目标安装

```bash
# 查看已安装的目标
rustup target list --installed

# 安装常用目标
rustup target add thumbv7em-none-eabihf
rustup target add thumbv6m-none-eabi
rustup target add riscv32imc-unknown-none-elf

# 批量安装
rustup target add thumbv7m-none-eabi thumbv7em-none-eabi thumbv7em-none-eabihf
```

## Cargo配置

### 1. 项目级配置 (.cargo/config.toml)

```toml
# .cargo/config.toml

[build]
# 默认目标架构
target = "thumbv7em-none-eabihf"

# 交叉编译工具链
[target.thumbv7em-none-eabihf]
# 使用特定的链接器
linker = "flip-link"
# 或使用arm-none-eabi-gcc
# linker = "arm-none-eabi-gcc"

# 运行器配置（用于cargo run）
runner = "probe-rs run --chip STM32F411RETx"

# 构建标志
rustflags = [
  # 链接参数
  "-C", "link-arg=-Tlink.x",
  "-C", "link-arg=-Tdefmt.x",  # 如果使用defmt
  
  # 优化选项
  "-C", "target-cpu=cortex-m4",
  "-C", "target-feature=+thumb2,+dsp",
  
  # 调试信息
  "-C", "debuginfo=2",
  
  # 安全选项
  "-D", "warnings",
  "-D", "unsafe_op_in_unsafe_fn",
]

# 多目标配置
[target.thumbv6m-none-eabi]
runner = "probe-rs run --chip STM32F030R8Tx"
rustflags = [
  "-C", "link-arg=-Tlink.x",
  "-C", "target-cpu=cortex-m0",
]

[target.riscv32imc-unknown-none-elf]
runner = "espflash flash --monitor"
rustflags = [
  "-C", "link-arg=-Tlinkall.x",
  "-C", "target-cpu=generic-rv32",
]

# 环境变量
[env]
DEFMT_LOG = "debug"
PROBE_RUN_CHIP = "STM32F411RETx"

# 别名
[alias]
rb = "run --bin"
rr = "run --release"
flash = "run --release"
size = "size --bin"
```

### 2. 全局配置 (~/.cargo/config.toml)

```toml
# 全局配置，影响所有项目

[build]
# 全局默认目标（可被项目配置覆盖）
# target = "thumbv7em-none-eabihf"

# 全局工具配置
[target.thumbv7em-none-eabihf]
linker = "flip-link"

[target.thumbv7m-none-eabi]
linker = "flip-link"

[target.thumbv6m-none-eabi]
linker = "flip-link"

# 注册表配置
[registries.crates-io]
protocol = "sparse"

# 网络配置
[http]
check-revoke = false
multiplexing = false

# Git配置
[net]
retry = 2
git-fetch-with-cli = true
```

## 链接器配置

### 1. 内存布局文件 (memory.x)

```ld
/* memory.x - STM32F411RE */
MEMORY
{
  /* Flash memory */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  
  /* RAM */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

/* 栈顶位置 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* 堆配置（可选） */
_heap_size = 32K;

/* 中断向量表位置 */
PROVIDE(_stext = ORIGIN(FLASH));

/* 确保.vector_table段在Flash开始位置 */
ASSERT(ORIGIN(FLASH) % 4 == 0, "FLASH origin must be aligned to 4 bytes");
ASSERT(LENGTH(FLASH) % 4 == 0, "FLASH length must be aligned to 4 bytes");
ASSERT(ORIGIN(RAM) % 4 == 0, "RAM origin must be aligned to 4 bytes");
ASSERT(LENGTH(RAM) % 4 == 0, "RAM length must be aligned to 4 bytes");
```

### 2. 高级内存配置

```ld
/* memory.x - STM32H743 (多RAM区域) */
MEMORY
{
  /* Flash */
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  
  /* 不同的RAM区域 */
  DTCM  : ORIGIN = 0x20000000, LENGTH = 128K  /* 数据紧耦合内存 */
  RAM_D1: ORIGIN = 0x24000000, LENGTH = 512K  /* AXI SRAM */
  RAM_D2: ORIGIN = 0x30000000, LENGTH = 288K  /* AHB SRAM */
  RAM_D3: ORIGIN = 0x38000000, LENGTH = 64K   /* AHB SRAM */
  
  /* 备份SRAM */
  BKPSRAM : ORIGIN = 0x38800000, LENGTH = 4K
}

/* 栈放在DTCM（最快） */
_stack_start = ORIGIN(DTCM) + LENGTH(DTCM);

/* 堆放在RAM_D1 */
_heap_start = ORIGIN(RAM_D1);
_heap_size = 256K;

/* DMA缓冲区放在RAM_D2（DMA可访问） */
SECTIONS
{
  .dma_buffers (NOLOAD) : ALIGN(4)
  {
    *(.dma_buffers .dma_buffers.*);
  } > RAM_D2
  
  .backup_data (NOLOAD) : ALIGN(4)
  {
    *(.backup_data .backup_data.*);
  } > BKPSRAM
}
```

### 3. 链接脚本优化

```ld
/* link.x - 自定义链接脚本 */
INCLUDE memory.x

/* 入口点 */
ENTRY(Reset);

/* 异常处理程序 */
EXTERN(RESET_VECTOR);
EXTERN(EXCEPTIONS);
EXTERN(INTERRUPTS);

SECTIONS
{
  /* 向量表 */
  .vector_table ORIGIN(FLASH) : ALIGN(4)
  {
    LONG(ORIGIN(RAM) + LENGTH(RAM)); /* 初始栈指针 */
    KEEP(*(.vector_table.reset_vector));
    KEEP(*(.vector_table.exceptions));
    KEEP(*(.vector_table.interrupts));
  } > FLASH
  
  /* 程序代码 */
  .text : ALIGN(4)
  {
    *(.text .text.*);
    *(.rodata .rodata.*);
    
    /* 保持函数对齐 */
    . = ALIGN(4);
  } > FLASH
  
  /* ARM.exidx段（异常处理） */
  .ARM.exidx : ALIGN(4)
  {
    __exidx_start = .;
    *(.ARM.exidx* .gnu.linkonce.armexidx.*);
    __exidx_end = .;
  } > FLASH
  
  /* 初始化数据 */
  .data : ALIGN(4)
  {
    __sdata = .;
    *(.data .data.*);
    . = ALIGN(4);
    __edata = .;
  } > RAM AT > FLASH
  
  __sidata = LOADADDR(.data);
  
  /* 未初始化数据 */
  .bss (NOLOAD) : ALIGN(4)
  {
    __sbss = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
    __ebss = .;
  } > RAM
  
  /* 堆 */
  .heap (NOLOAD) : ALIGN(4)
  {
    __sheap = .;
    . += _heap_size;
    __eheap = .;
  } > RAM
  
  /* 栈检查 */
  .stack (NOLOAD) : ALIGN(8)
  {
    __sstack = .;
    . = ORIGIN(RAM) + LENGTH(RAM);
    __estack = .;
  } > RAM
  
  /* 调试信息 */
  .debug_info 0 : { *(.debug_info) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_line 0 : { *(.debug_line) }
  .debug_str 0 : { *(.debug_str) }
  .debug_ranges 0 : { *(.debug_ranges) }
  
  /* 丢弃不需要的段 */
  /DISCARD/ :
  {
    *(.ARM.attributes);
    *(.comment);
    *(.note*);
  }
}

/* 断言检查 */
ASSERT(__sdata >= ORIGIN(RAM), "Data section starts before RAM");
ASSERT(__estack == ORIGIN(RAM) + LENGTH(RAM), "Stack doesn't end at RAM boundary");
ASSERT(__sheap < __sstack, "Heap and stack overlap");
```

## 构建脚本配置

### 1. 基础构建脚本 (build.rs)

```rust
// build.rs
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // 输出目录
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    
    // 生成内存布局
    generate_memory_x(out);
    
    // 链接库
    println!("cargo:rustc-link-lib=static=m");  // 数学库
    
    // 重新构建条件
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    
    // 环境变量
    println!("cargo:rustc-env=TARGET={}", env::var("TARGET").unwrap());
}

fn generate_memory_x(out_dir: &PathBuf) {
    // 根据特性生成不同的内存布局
    let memory_x = if cfg!(feature = "stm32f411") {
        include_str!("memory_stm32f411.x")
    } else if cfg!(feature = "stm32h743") {
        include_str!("memory_stm32h743.x")
    } else {
        include_str!("memory_default.x")
    };
    
    File::create(out_dir.join("memory.x"))
        .unwrap()
        .write_all(memory_x.as_bytes())
        .unwrap();
    
    println!("cargo:rustc-link-search={}", out_dir.display());
}
```

### 2. 高级构建脚本

```rust
// build.rs - 高级版本
use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let target = env::var("TARGET").unwrap();
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    
    // 目标特定配置
    match target.as_str() {
        "thumbv7em-none-eabihf" => configure_cortex_m4f(),
        "thumbv6m-none-eabi" => configure_cortex_m0(),
        "riscv32imc-unknown-none-elf" => configure_riscv32(),
        _ => panic!("Unsupported target: {}", target),
    }
    
    // 生成配置头文件
    generate_config_h(&out_dir);
    
    // 处理汇编文件
    compile_assembly(&target);
    
    // 链接外部库
    link_external_libs(&target);
}

fn configure_cortex_m4f() {
    println!("cargo:rustc-cfg=cortex_m");
    println!("cargo:rustc-cfg=cortex_m4");
    println!("cargo:rustc-cfg=has_fpu");
    
    // 链接ARM数学库
    println!("cargo:rustc-link-lib=static=arm_cortexM4lf_math");
}

fn configure_cortex_m0() {
    println!("cargo:rustc-cfg=cortex_m");
    println!("cargo:rustc-cfg=cortex_m0");
    
    // Cortex-M0没有FPU
    println!("cargo:rustc-cfg=no_fpu");
}

fn configure_riscv32() {
    println!("cargo:rustc-cfg=riscv");
    println!("cargo:rustc-cfg=riscv32");
    
    // RISC-V特定配置
    println!("cargo:rustc-link-arg=-Tlinkall.x");
}

fn generate_config_h(out_dir: &PathBuf) {
    let config = format!(
        r#"
#ifndef CONFIG_H
#define CONFIG_H

#define SYSTEM_CLOCK_HZ {}
#define HEAP_SIZE {}
#define STACK_SIZE {}

#ifdef CORTEX_M4
#define HAS_FPU 1
#else
#define HAS_FPU 0
#endif

#endif
"#,
        env::var("SYSTEM_CLOCK").unwrap_or_else(|_| "84000000".to_string()),
        env::var("HEAP_SIZE").unwrap_or_else(|_| "32768".to_string()),
        env::var("STACK_SIZE").unwrap_or_else(|_| "8192".to_string()),
    );
    
    fs::write(out_dir.join("config.h"), config).unwrap();
}

fn compile_assembly(target: &str) {
    let mut build = cc::Build::new();
    
    match target {
        t if t.starts_with("thumb") => {
            build
                .compiler("arm-none-eabi-gcc")
                .flag("-mcpu=cortex-m4")
                .flag("-mthumb")
                .flag("-mfloat-abi=hard")
                .flag("-mfpu=fpv4-sp-d16");
        }
        t if t.starts_with("riscv") => {
            build
                .compiler("riscv32-unknown-elf-gcc")
                .flag("-march=rv32imc")
                .flag("-mabi=ilp32");
        }
        _ => {}
    }
    
    // 编译汇编文件
    if let Ok(entries) = fs::read_dir("src/asm") {
        for entry in entries {
            if let Ok(entry) = entry {
                let path = entry.path();
                if path.extension().map_or(false, |ext| ext == "s" || ext == "S") {
                    build.file(path);
                }
            }
        }
    }
    
    build.compile("asm");
}

fn link_external_libs(target: &str) {
    // 根据目标链接不同的库
    match target {
        t if t.contains("eabihf") => {
            // 硬浮点目标
            println!("cargo:rustc-link-lib=static=m");
            println!("cargo:rustc-link-lib=static=c");
        }
        t if t.contains("eabi") => {
            // 软浮点目标
            println!("cargo:rustc-link-lib=static=m");
        }
        _ => {}
    }
}
```

## 编译优化

### 1. 性能优化配置

```toml
# Cargo.toml
[profile.release]
# 代码大小优化
opt-level = "s"          # 或 "z" 为最小体积
lto = true               # 链接时优化
codegen-units = 1        # 单个代码生成单元
panic = "abort"          # 不使用unwinding
strip = true             # 去除符号信息
overflow-checks = false  # 禁用溢出检查

[profile.dev]
# 开发时的平衡配置
opt-level = 1            # 轻微优化
debug = true             # 保留调试信息
lto = false              # 加快编译速度
overflow-checks = true   # 保留溢出检查

# 依赖包优化
[profile.dev.package."*"]
opt-level = 2            # 优化依赖包

# 特定包配置
[profile.release.package.heapless]
opt-level = 3            # 最高优化级别

[profile.release.package.cortex-m]
opt-level = "s"          # 体积优化
```

### 2. 目标特定优化

```toml
# .cargo/config.toml
[target.thumbv7em-none-eabihf]
rustflags = [
  # CPU特定优化
  "-C", "target-cpu=cortex-m4",
  "-C", "target-feature=+thumb2,+dsp,+fp16",
  
  # 链接时优化
  "-C", "lto=fat",
  
  # 代码生成优化
  "-C", "codegen-units=1",
  "-C", "embed-bitcode=yes",
  
  # 安全优化
  "-C", "control-flow-guard=checks",
  
  # 调试优化
  "-C", "debuginfo=2",
  "-C", "split-debuginfo=packed",
]

[target.thumbv6m-none-eabi]
rustflags = [
  # Cortex-M0优化
  "-C", "target-cpu=cortex-m0",
  "-C", "target-feature=+thumb2",
  
  # 体积优化（M0资源有限）
  "-C", "opt-level=s",
  "-C", "lto=thin",
]
```

### 3. 链接器优化

```toml
# .cargo/config.toml
[target.thumbv7em-none-eabihf]
rustflags = [
  # 使用优化链接器
  "-C", "linker=rust-lld",
  
  # 链接器参数
  "-C", "link-arg=-Tlink.x",
  "-C", "link-arg=--nmagic",        # 禁用页面对齐
  "-C", "link-arg=--gc-sections",   # 垃圾回收未使用段
  "-C", "link-arg=-Map=output.map", # 生成映射文件
  
  # 栈保护
  "-C", "link-arg=-z,stack-size=8192",
  
  # 优化选项
  "-C", "link-arg=-O2",
  "-C", "link-arg=--strip-all",     # 去除所有符号
]
```

## 多目标构建

### 1. 工作空间配置

```toml
# Cargo.toml (工作空间根目录)
[workspace]
members = [
    "stm32f4-project",
    "esp32c3-project", 
    "rp2040-project",
    "common-lib",
]

[workspace.dependencies]
# 共享依赖
cortex-m = "0.7"
embedded-hal = "1.0"
nb = "1.0"

# 工作空间级别的配置
[workspace.package]
version = "0.1.0"
edition = "2021"
authors = ["Your Name <your.email@example.com>"]
license = "MIT OR Apache-2.0"

[profile.release]
opt-level = "s"
lto = true
codegen-units = 1
```

### 2. 条件编译

```rust
// src/lib.rs
#[cfg(target_arch = "arm")]
pub mod arm_specific {
    use cortex_m;
    
    pub fn delay_cycles(cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}

#[cfg(target_arch = "riscv32")]
pub mod riscv_specific {
    pub fn delay_cycles(cycles: u32) {
        // RISC-V特定实现
        for _ in 0..cycles {
            unsafe { core::arch::asm!("nop") };
        }
    }
}

// 统一接口
#[cfg(target_arch = "arm")]
pub use arm_specific::*;

#[cfg(target_arch = "riscv32")]
pub use riscv_specific::*;

// 编译时特性检查
#[cfg(all(feature = "stm32f4", not(target_arch = "arm")))]
compile_error!("STM32F4 feature requires ARM target");

// 目标特定配置
#[cfg(target_os = "none")]
pub const HEAP_SIZE: usize = if cfg!(feature = "large-heap") {
    64 * 1024
} else {
    32 * 1024
};
```

### 3. 构建脚本

```bash
#!/bin/bash
# build-all.sh

set -e

# 定义目标列表
TARGETS=(
    "thumbv6m-none-eabi"
    "thumbv7m-none-eabi" 
    "thumbv7em-none-eabi"
    "thumbv7em-none-eabihf"
    "riscv32imc-unknown-none-elf"
)

# 构建所有目标
for target in "${TARGETS[@]}"; do
    echo "Building for $target..."
    cargo build --target $target --release
    
    # 检查二进制大小
    size=$(cargo size --target $target --release --bin main | tail -n1 | awk '{print $4}')
    echo "Binary size for $target: $size bytes"
done

echo "All targets built successfully!"
```

## 性能分析和优化

### 1. 代码大小分析

```bash
# 安装cargo-size
cargo install cargo-binutils
rustup component add llvm-tools-preview

# 分析二进制大小
cargo size --target thumbv7em-none-eabihf --release

# 详细段分析
cargo size --target thumbv7em-none-eabihf --release -- -A

# 符号大小分析
cargo nm --target thumbv7em-none-eabihf --release | sort -n
```

### 2. 反汇编分析

```bash
# 生成反汇编
cargo objdump --target thumbv7em-none-eabihf --release -- -d > disasm.txt

# 查看特定函数
cargo objdump --target thumbv7em-none-eabihf --release -- -d --disassemble=main

# 生成映射文件
cargo rustc --target thumbv7em-none-eabihf --release -- -C link-arg=-Map=output.map
```

### 3. 性能基准测试

```rust
// benches/performance.rs
#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

#[entry]
fn main() -> ! {
    // 初始化DWT计数器
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.DWT.enable_cycle_counter();
    
    // 基准测试
    let start = cortex_m::peripheral::DWT::cycle_count();
    
    // 测试代码
    test_function();
    
    let end = cortex_m::peripheral::DWT::cycle_count();
    let cycles = end.wrapping_sub(start);
    
    // 输出结果（通过RTT或其他方式）
    rtt_target::rprintln!("Function took {} cycles", cycles);
    
    loop {}
}

fn test_function() {
    // 被测试的函数
    for i in 0..1000 {
        unsafe { core::ptr::write_volatile(&mut black_box(i), i * 2) };
    }
}

fn black_box<T>(dummy: T) -> T {
    unsafe { core::ptr::read_volatile(&dummy) }
}
```

## 调试和故障排除

### 1. 常见编译错误

**链接器错误:**
```
error: linking with `rust-lld` failed: exit status: 1
```

解决方案：
```toml
# .cargo/config.toml
[target.thumbv7em-none-eabihf]
linker = "flip-link"  # 或 "arm-none-eabi-gcc"
```

**内存溢出:**
```
error: the program has overflowed the available memory
```

解决方案：
```rust
// 检查memory.x配置
// 优化代码大小
#[no_mangle]
#[link_section = ".text.unlikely"]
pub fn rarely_used_function() {
    // 将很少使用的函数放到特殊段
}
```

### 2. 调试技巧

```bash
# 详细构建输出
cargo build --target thumbv7em-none-eabihf --verbose

# 检查依赖树
cargo tree --target thumbv7em-none-eabihf

# 检查特性
cargo build --target thumbv7em-none-eabihf --features debug --verbose

# 环境变量调试
RUST_LOG=debug cargo build --target thumbv7em-none-eabihf
```

### 3. 链接器映射分析

```bash
# 生成详细映射文件
cargo rustc --target thumbv7em-none-eabihf --release -- \
    -C link-arg=-Map=output.map \
    -C link-arg=--cref \
    -C link-arg=--print-memory-usage

# 分析映射文件
grep -E "^\.text|^\.rodata|^\.data|^\.bss" output.map
```

## 最佳实践

### 1. 项目结构

```
embedded-project/
├── .cargo/
│   └── config.toml          # 目标配置
├── src/
│   ├── main.rs             # 主程序
│   ├── lib.rs              # 库接口
│   └── target_specific/    # 目标特定代码
│       ├── cortex_m.rs
│       └── riscv.rs
├── memory/                  # 内存配置
│   ├── stm32f411.x
│   ├── stm32h743.x
│   └── esp32c3.x
├── build.rs                # 构建脚本
├── Cargo.toml              # 项目配置
└── scripts/
    ├── build-all.sh        # 多目标构建
    └── analyze-size.sh     # 大小分析
```

### 2. 版本管理

```toml
# Cargo.toml
[dependencies]
# 使用精确版本避免意外更新
cortex-m = "=0.7.7"
cortex-m-rt = "=0.7.3"

# 或使用兼容版本
embedded-hal = "~1.0.0"

# 锁定工具链版本
# rust-toolchain.toml
[toolchain]
channel = "1.70.0"
components = ["rust-src", "llvm-tools-preview"]
targets = ["thumbv7em-none-eabihf", "riscv32imc-unknown-none-elf"]
```

### 3. CI/CD配置

```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  build:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        target:
          - thumbv6m-none-eabi
          - thumbv7m-none-eabi
          - thumbv7em-none-eabi
          - thumbv7em-none-eabihf
          - riscv32imc-unknown-none-elf
    
    steps:
    - uses: actions/checkout@v3
    
    - name: Install Rust
      uses: actions-rs/toolchain@v1
      with:
        toolchain: stable
        target: ${{ matrix.target }}
        override: true
        components: rust-src, llvm-tools-preview
    
    - name: Build
      run: cargo build --target ${{ matrix.target }} --release
    
    - name: Test
      run: cargo test --target ${{ matrix.target }}
      if: matrix.target == 'x86_64-unknown-linux-gnu'
    
    - name: Size Analysis
      run: |
        cargo install cargo-binutils
        cargo size --target ${{ matrix.target }} --release
```

## 下一步

掌握交叉编译后，建议：

1. 学习[仿真和测试环境](./07-simulation-testing.md)搭建
2. 了解[故障排除](./08-troubleshooting.md)常见问题
3. 实践多目标项目开发

---

**现在你已经掌握了嵌入式Rust的交叉编译精髓！** 🎯
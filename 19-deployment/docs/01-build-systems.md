# 构建系统 (Build Systems)

嵌入式Rust项目的构建系统配置和优化，包括Cargo配置、交叉编译、链接脚本和构建优化。

## 1. Cargo配置基础

### 1.1 项目结构
```
project/
├── Cargo.toml          # 项目配置
├── Cargo.lock          # 依赖锁定
├── .cargo/
│   └── config.toml     # Cargo配置
├── memory.x            # 内存布局
├── build.rs            # 构建脚本
├── src/
│   ├── main.rs
│   └── lib.rs
└── target/             # 构建输出
```

### 1.2 基本Cargo.toml配置
```toml
[package]
name = "embedded-project"
version = "0.1.0"
edition = "2021"
authors = ["Developer <dev@example.com>"]
description = "Embedded Rust project"
license = "MIT OR Apache-2.0"
repository = "https://github.com/example/embedded-project"

[dependencies]
# 核心依赖
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"

# HAL依赖
stm32f4xx-hal = { version = "0.14", features = ["stm32f401", "rt"] }
embedded-hal = "0.2"

# 可选依赖
serde = { version = "1.0", default-features = false, optional = true }
defmt = { version = "0.3", optional = true }

[features]
default = []
serde-support = ["serde"]
logging = ["defmt"]

# 构建配置
[profile.dev]
debug = true
opt-level = 1
overflow-checks = true

[profile.release]
debug = true
opt-level = "s"          # 优化大小
lto = true               # 链接时优化
codegen-units = 1        # 单个代码生成单元
panic = "abort"          # panic时直接abort

[profile.release-with-debug]
inherits = "release"
debug = true
strip = false

# 目标配置
[target.thumbv7em-none-eabihf]
runner = "probe-run --chip STM32F401RETx"
rustflags = [
  "-C", "linker=flip-link",
  "-C", "link-arg=-Tlink.x",
  "-C", "link-arg=-Tdefmt.x",
]
```

### 1.3 Cargo配置文件 (.cargo/config.toml)
```toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-run --chip STM32F401RETx"
rustflags = [
  # 使用flip-link防止栈溢出
  "-C", "linker=flip-link",
  
  # 链接脚本
  "-C", "link-arg=-Tlink.x",
  
  # defmt链接脚本 (如果使用defmt)
  "-C", "link-arg=-Tdefmt.x",
  
  # 优化选项
  "-C", "link-arg=-Wl,--gc-sections",
  "-C", "link-arg=-Wl,--print-memory-usage",
]

[env]
DEFMT_LOG = "debug"
PROBE_RUN_CHIP = "STM32F401RETx"

# 别名
[alias]
rb = "run --bin"
rr = "run --release"
br = "build --release"
```

## 2. 内存布局配置

### 2.1 memory.x文件
```ld
/* STM32F401RE内存布局 */
MEMORY
{
  /* Flash存储器 */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  
  /* RAM存储器 */
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}

/* 栈大小配置 */
_stack_size = 8K;

/* 堆大小配置 (可选) */
_heap_size = 16K;

/* 向量表位置 */
_stext = ORIGIN(FLASH);
```

### 2.2 高级内存配置
```ld
MEMORY
{
  /* Flash区域 */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  
  /* RAM区域分割 */
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
  RAM2 : ORIGIN = 0x20010000, LENGTH = 32K
  
  /* 特殊用途区域 */
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K  /* CCM RAM (如果有) */
  BACKUP : ORIGIN = 0x40024000, LENGTH = 4K   /* 备份寄存器 */
}

/* 区域分配 */
SECTIONS
{
  /* 快速访问数据 */
  .ccmram (NOLOAD) : ALIGN(4)
  {
    *(.ccmram .ccmram.*);
    . = ALIGN(4);
  } > CCMRAM
  
  /* DMA缓冲区 */
  .dma_buffers (NOLOAD) : ALIGN(4)
  {
    *(.dma_buffers .dma_buffers.*);
    . = ALIGN(4);
  } > RAM2
}
```

## 3. 构建脚本 (build.rs)

### 3.1 基本构建脚本
```rust
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    // 输出目录
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    
    // 生成内存布局
    generate_memory_layout(out);
    
    // 生成版本信息
    generate_version_info(out);
    
    // 生成配置文件
    generate_config(out);
    
    // 重新构建条件
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rerun-if-env-changed=TARGET_MEMORY_SIZE");
}

fn generate_memory_layout(out_dir: &PathBuf) {
    // 从环境变量获取内存大小
    let flash_size = env::var("TARGET_FLASH_SIZE")
        .unwrap_or_else(|_| "512K".to_string());
    let ram_size = env::var("TARGET_RAM_SIZE")
        .unwrap_or_else(|_| "96K".to_string());
    
    let memory_x = format!(
        r#"
MEMORY
{{
  FLASH : ORIGIN = 0x08000000, LENGTH = {}
  RAM : ORIGIN = 0x20000000, LENGTH = {}
}}
"#,
        flash_size, ram_size
    );
    
    File::create(out_dir.join("memory.x"))
        .unwrap()
        .write_all(memory_x.as_bytes())
        .unwrap();
    
    println!("cargo:rustc-link-search={}", out_dir.display());
}

fn generate_version_info(out_dir: &PathBuf) {
    let version_info = format!(
        r#"
pub const VERSION: &str = "{}";
pub const GIT_HASH: &str = "{}";
pub const BUILD_TIME: &str = "{}";
pub const TARGET: &str = "{}";
"#,
        env::var("CARGO_PKG_VERSION").unwrap(),
        get_git_hash(),
        chrono::Utc::now().format("%Y-%m-%d %H:%M:%S UTC"),
        env::var("TARGET").unwrap()
    );
    
    File::create(out_dir.join("version.rs"))
        .unwrap()
        .write_all(version_info.as_bytes())
        .unwrap();
}

fn generate_config(out_dir: &PathBuf) {
    let config = format!(
        r#"
pub const FLASH_SIZE: usize = {};
pub const RAM_SIZE: usize = {};
pub const STACK_SIZE: usize = {};
pub const HEAP_SIZE: usize = {};
"#,
        parse_memory_size(&env::var("TARGET_FLASH_SIZE").unwrap_or_else(|_| "512K".to_string())),
        parse_memory_size(&env::var("TARGET_RAM_SIZE").unwrap_or_else(|_| "96K".to_string())),
        parse_memory_size(&env::var("STACK_SIZE").unwrap_or_else(|_| "8K".to_string())),
        parse_memory_size(&env::var("HEAP_SIZE").unwrap_or_else(|_| "16K".to_string())),
    );
    
    File::create(out_dir.join("config.rs"))
        .unwrap()
        .write_all(config.as_bytes())
        .unwrap();
}

fn get_git_hash() -> String {
    std::process::Command::new("git")
        .args(&["rev-parse", "--short", "HEAD"])
        .output()
        .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_string())
        .unwrap_or_else(|_| "unknown".to_string())
}

fn parse_memory_size(size_str: &str) -> usize {
    let size_str = size_str.trim().to_uppercase();
    if size_str.ends_with('K') {
        size_str[..size_str.len()-1].parse::<usize>().unwrap() * 1024
    } else if size_str.ends_with('M') {
        size_str[..size_str.len()-1].parse::<usize>().unwrap() * 1024 * 1024
    } else {
        size_str.parse().unwrap()
    }
}
```

### 3.2 在代码中使用生成的配置
```rust
// 包含生成的文件
include!(concat!(env!("OUT_DIR"), "/version.rs"));
include!(concat!(env!("OUT_DIR"), "/config.rs"));

fn main() {
    // 使用版本信息
    defmt::info!("Firmware version: {}", VERSION);
    defmt::info!("Git hash: {}", GIT_HASH);
    defmt::info!("Build time: {}", BUILD_TIME);
    defmt::info!("Target: {}", TARGET);
    
    // 使用配置信息
    defmt::info!("Flash size: {} bytes", FLASH_SIZE);
    defmt::info!("RAM size: {} bytes", RAM_SIZE);
    defmt::info!("Stack size: {} bytes", STACK_SIZE);
    defmt::info!("Heap size: {} bytes", HEAP_SIZE);
}
```

## 4. 交叉编译配置

### 4.1 目标架构配置
```bash
# 安装目标架构
rustup target add thumbv7em-none-eabihf    # Cortex-M4/M7 with FPU
rustup target add thumbv7m-none-eabi       # Cortex-M3
rustup target add thumbv6m-none-eabi       # Cortex-M0/M0+
rustup target add thumbv8m.main-none-eabi  # Cortex-M33/M35P
```

### 4.2 链接器配置
```toml
# .cargo/config.toml
[target.thumbv7em-none-eabihf]
linker = "flip-link"
runner = "probe-run --chip STM32F401RETx"

rustflags = [
  # 链接脚本
  "-C", "link-arg=-Tlink.x",
  
  # 垃圾回收未使用的代码
  "-C", "link-arg=-Wl,--gc-sections",
  
  # 显示内存使用情况
  "-C", "link-arg=-Wl,--print-memory-usage",
  
  # 优化选项
  "-C", "target-cpu=cortex-m4",
  "-C", "target-feature=+thumb-mode",
]
```

### 4.3 条件编译
```rust
// 根据目标架构条件编译
#[cfg(target_arch = "arm")]
fn arm_specific_code() {
    // ARM特定代码
}

#[cfg(target_os = "none")]
fn no_std_code() {
    // no_std环境代码
}

#[cfg(feature = "stm32f4")]
fn stm32f4_code() {
    // STM32F4特定代码
}

// 根据内存大小条件编译
#[cfg(feature = "large-memory")]
const BUFFER_SIZE: usize = 4096;

#[cfg(not(feature = "large-memory"))]
const BUFFER_SIZE: usize = 1024;
```

## 5. 构建优化

### 5.1 大小优化
```toml
[profile.release]
opt-level = "s"          # 优化大小而非速度
lto = true               # 链接时优化
codegen-units = 1        # 减少代码生成单元
panic = "abort"          # 减少panic处理代码
strip = true             # 去除调试符号

[profile.release-size]
inherits = "release"
opt-level = "z"          # 更激进的大小优化
```

### 5.2 速度优化
```toml
[profile.release-speed]
inherits = "release"
opt-level = 3            # 最高速度优化
lto = "fat"              # 完整LTO
codegen-units = 1
```

### 5.3 调试优化
```toml
[profile.dev]
opt-level = 1            # 轻微优化以提高调试体验
debug = true
overflow-checks = true
incremental = true       # 增量编译

[profile.release-debug]
inherits = "release"
debug = true             # 保留调试信息
strip = false
```

## 6. 多目标构建

### 6.1 工作空间配置
```toml
# Cargo.toml (工作空间根目录)
[workspace]
members = [
    "firmware",
    "bootloader", 
    "tests",
    "tools",
]

[workspace.dependencies]
cortex-m = "0.7"
cortex-m-rt = "0.7"
stm32f4xx-hal = "0.14"

[workspace.metadata.cross]
default-target = "thumbv7em-none-eabihf"
```

### 6.2 成员项目配置
```toml
# firmware/Cargo.toml
[package]
name = "firmware"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = { workspace = true }
cortex-m-rt = { workspace = true }
stm32f4xx-hal = { workspace = true, features = ["stm32f401"] }

# bootloader/Cargo.toml
[package]
name = "bootloader"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = { workspace = true }
cortex-m-rt = { workspace = true }
```

### 6.3 构建脚本
```bash
#!/bin/bash
# build.sh

set -e

echo "Building embedded project..."

# 构建固件
echo "Building firmware..."
cd firmware
cargo build --release
cd ..

# 构建引导程序
echo "Building bootloader..."
cd bootloader
cargo build --release
cd ..

# 生成二进制文件
echo "Generating binary files..."
arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/firmware \
    firmware.bin

arm-none-eabi-objcopy -O binary \
    target/thumbv7em-none-eabihf/release/bootloader \
    bootloader.bin

# 合并固件
echo "Merging firmware..."
cat bootloader.bin firmware.bin > combined.bin

echo "Build completed successfully!"
echo "Firmware size: $(wc -c < firmware.bin) bytes"
echo "Bootloader size: $(wc -c < bootloader.bin) bytes"
echo "Combined size: $(wc -c < combined.bin) bytes"
```

## 7. 持续集成配置

### 7.1 GitHub Actions配置
```yaml
# .github/workflows/ci.yml
name: CI

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

env:
  CARGO_TERM_COLOR: always

jobs:
  check:
    name: Check
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    
    - name: Install Rust
      uses: actions-rs/toolchain@v1
      with:
        profile: minimal
        toolchain: stable
        target: thumbv7em-none-eabihf
        override: true
        components: rustfmt, clippy
    
    - name: Check formatting
      run: cargo fmt --all -- --check
    
    - name: Clippy
      run: cargo clippy --all-targets --all-features -- -D warnings
    
    - name: Check
      run: cargo check --all-targets --all-features

  build:
    name: Build
    runs-on: ubuntu-latest
    strategy:
      matrix:
        target:
          - thumbv7em-none-eabihf
          - thumbv7m-none-eabi
          - thumbv6m-none-eabi
    steps:
    - uses: actions/checkout@v3
    
    - name: Install Rust
      uses: actions-rs/toolchain@v1
      with:
        profile: minimal
        toolchain: stable
        target: ${{ matrix.target }}
        override: true
    
    - name: Build
      run: cargo build --release --target ${{ matrix.target }}
    
    - name: Build size analysis
      run: |
        arm-none-eabi-size target/${{ matrix.target }}/release/firmware
        arm-none-eabi-objdump -h target/${{ matrix.target }}/release/firmware

  test:
    name: Test
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    
    - name: Install Rust
      uses: actions-rs/toolchain@v1
      with:
        profile: minimal
        toolchain: stable
        override: true
    
    - name: Run tests
      run: cargo test --all-features
```

### 7.2 构建缓存优化
```yaml
    - name: Cache cargo registry
      uses: actions/cache@v3
      with:
        path: ~/.cargo/registry
        key: ${{ runner.os }}-cargo-registry-${{ hashFiles('**/Cargo.lock') }}
    
    - name: Cache cargo index
      uses: actions/cache@v3
      with:
        path: ~/.cargo/git
        key: ${{ runner.os }}-cargo-index-${{ hashFiles('**/Cargo.lock') }}
    
    - name: Cache cargo build
      uses: actions/cache@v3
      with:
        path: target
        key: ${{ runner.os }}-cargo-build-target-${{ hashFiles('**/Cargo.lock') }}
```

## 8. 构建工具和脚本

### 8.1 Makefile
```makefile
# Makefile
.PHONY: all build release debug clean flash size objdump

# 默认目标
TARGET = thumbv7em-none-eabihf
CHIP = STM32F401RETx

# 构建目标
all: build

build:
	cargo build

release:
	cargo build --release

debug:
	cargo build

# 清理
clean:
	cargo clean

# 烧录
flash: release
	probe-run --chip $(CHIP) target/$(TARGET)/release/firmware

# 大小分析
size: release
	arm-none-eabi-size target/$(TARGET)/release/firmware

# 反汇编
objdump: release
	arm-none-eabi-objdump -d target/$(TARGET)/release/firmware > firmware.asm

# 生成二进制文件
bin: release
	arm-none-eabi-objcopy -O binary target/$(TARGET)/release/firmware firmware.bin

# 生成十六进制文件
hex: release
	arm-none-eabi-objcopy -O ihex target/$(TARGET)/release/firmware firmware.hex

# 内存映射
map: release
	arm-none-eabi-nm -n target/$(TARGET)/release/firmware > firmware.map

# 符号表
symbols: release
	arm-none-eabi-nm -C target/$(TARGET)/release/firmware | sort > firmware.symbols

# 完整分析
analyze: size objdump map symbols
	@echo "Analysis complete. Check firmware.asm, firmware.map, and firmware.symbols"
```

### 8.2 PowerShell脚本 (Windows)
```powershell
# build.ps1
param(
    [string]$Target = "release",
    [string]$Chip = "STM32F401RETx",
    [switch]$Flash,
    [switch]$Size,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

if ($Clean) {
    Write-Host "Cleaning build artifacts..." -ForegroundColor Yellow
    cargo clean
    exit 0
}

Write-Host "Building for target: $Target" -ForegroundColor Green

if ($Target -eq "release") {
    cargo build --release
    $BinaryPath = "target/thumbv7em-none-eabihf/release/firmware"
} else {
    cargo build
    $BinaryPath = "target/thumbv7em-none-eabihf/debug/firmware"
}

if ($Size) {
    Write-Host "Binary size analysis:" -ForegroundColor Cyan
    arm-none-eabi-size $BinaryPath
}

if ($Flash) {
    Write-Host "Flashing to device..." -ForegroundColor Magenta
    probe-run --chip $Chip $BinaryPath
}

Write-Host "Build completed successfully!" -ForegroundColor Green
```

## 总结

构建系统是嵌入式Rust开发的基础，正确配置构建系统可以：

1. **提高开发效率**: 自动化构建、测试和部署流程
2. **优化性能**: 通过编译器优化减少代码大小和提高执行速度
3. **确保质量**: 通过持续集成保证代码质量
4. **简化部署**: 自动生成适合目标硬件的二进制文件

关键要点：
- 合理配置Cargo.toml和内存布局
- 使用构建脚本生成配置和版本信息
- 针对目标优化编译选项
- 建立完整的CI/CD流程
- 使用工具脚本简化常用操作
# 基础环境安装

## 概述

本章将指导你安装Rust嵌入式开发所需的基础工具和环境。我们将从Rust工具链开始，逐步配置完整的开发环境。

## 1. Rust工具链安装

### 1.1 安装Rustup

Rustup是Rust的官方工具链管理器，用于安装和管理Rust版本。

#### macOS和Linux

```bash
# 下载并安装rustup
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 重新加载环境变量
source ~/.bashrc
# 或者
source ~/.zshrc
```

#### Windows

1. 访问 [rustup.rs](https://rustup.rs/)
2. 下载并运行 `rustup-init.exe`
3. 按照安装向导完成安装
4. 重启命令提示符或PowerShell

### 1.2 验证安装

```bash
# 检查Rust版本
rustc --version

# 检查Cargo版本
cargo --version

# 检查Rustup版本
rustup --version
```

预期输出类似：
```
rustc 1.70.0 (90c541806 2023-05-31)
cargo 1.70.0 (ec8a8a0ca 2023-04-25)
rustup 1.26.0 (5af9b9484 2023-04-05)
```

### 1.3 配置Rust工具链

```bash
# 设置默认工具链为稳定版
rustup default stable

# 更新工具链
rustup update

# 查看已安装的工具链
rustup show
```

## 2. 嵌入式目标架构

### 2.1 常用目标架构

嵌入式开发需要为特定的目标架构编译代码。以下是常用的目标架构：

| 架构 | 目标三元组 | 适用硬件 |
|------|------------|----------|
| ARM Cortex-M0/M0+ | thumbv6m-none-eabi | STM32F0, Nordic nRF51 |
| ARM Cortex-M3 | thumbv7m-none-eabi | STM32F1, STM32L1 |
| ARM Cortex-M4/M7 | thumbv7em-none-eabi | STM32F4, STM32F7 (无FPU) |
| ARM Cortex-M4F/M7F | thumbv7em-none-eabihf | STM32F4, STM32F7 (有FPU) |
| ARM Cortex-M23 | thumbv8m.base-none-eabi | STM32L5 |
| ARM Cortex-M33 | thumbv8m.main-none-eabi | STM32L5, Nordic nRF91 |
| RISC-V | riscv32imac-unknown-none-elf | ESP32-C3, GD32VF103 |

### 2.2 安装目标架构

```bash
# 安装ARM Cortex-M4F目标（最常用）
rustup target add thumbv7em-none-eabihf

# 安装其他常用目标
rustup target add thumbv7m-none-eabi
rustup target add thumbv6m-none-eabi
rustup target add riscv32imac-unknown-none-elf

# 查看已安装的目标
rustup target list --installed
```

### 2.3 验证目标架构

创建一个简单的测试项目来验证目标架构：

```bash
# 创建测试项目
cargo new --bin embedded-test
cd embedded-test
```

编辑 `Cargo.toml`：
```toml
[package]
name = "embedded-test"
version = "0.1.0"
edition = "2021"

[dependencies]
panic-halt = "0.2"

[[bin]]
name = "embedded-test"
test = false
bench = false

[profile.dev]
codegen-units = 1
debug = 2
debug-assertions = true
incremental = false
opt-level = "s"
overflow-checks = true

[profile.release]
codegen-units = 1
debug = 2
debug-assertions = false
incremental = false
lto = "fat"
opt-level = "s"
overflow-checks = false
panic = "abort"
strip = "symbols"
```

创建 `src/main.rs`：
```rust
#![no_std]
#![no_main]

use panic_halt as _;

#[no_mangle]
pub unsafe extern "C" fn Reset() -> ! {
    loop {}
}
```

创建 `memory.x`：
```
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  RAM : ORIGIN = 0x20000000, LENGTH = 64K
}
```

创建 `.cargo/config.toml`：
```toml
[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32F411RETx"
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[build]
target = "thumbv7em-none-eabihf"

[env]
DEFMT_LOG = "debug"
```

编译测试：
```bash
cargo build --target thumbv7em-none-eabihf
```

## 3. 必要的Cargo工具

### 3.1 安装cargo-binutils

```bash
# 安装cargo-binutils（提供objdump, nm, objcopy等工具）
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

### 3.2 安装其他有用工具

```bash
# 项目模板生成工具
cargo install cargo-generate

# 代码格式化工具
rustup component add rustfmt

# 代码检查工具
rustup component add clippy

# 文档生成工具（通常已包含）
# rustup component add rust-docs

# 交叉编译辅助工具
cargo install cross
```

### 3.3 验证工具安装

```bash
# 验证binutils
cargo objdump -- --version
cargo nm -- --version
cargo objcopy -- --version

# 验证其他工具
cargo generate --version
cargo fmt --version
cargo clippy --version
```

## 4. 调试和烧录工具

### 4.1 安装probe-rs

probe-rs是现代的Rust嵌入式调试和烧录工具：

```bash
# 安装probe-rs
cargo install probe-rs --features cli

# 验证安装
probe-rs --version
```

### 4.2 安装OpenOCD（可选）

OpenOCD是传统的调试工具，某些情况下仍然有用：

#### macOS
```bash
# 使用Homebrew安装
brew install openocd
```

#### Ubuntu/Debian
```bash
sudo apt update
sudo apt install openocd
```

#### Windows
1. 从 [OpenOCD官网](https://openocd.org/) 下载预编译版本
2. 解压到合适位置
3. 将bin目录添加到PATH环境变量

### 4.3 USB权限配置（Linux）

在Linux系统上，需要配置USB设备权限：

```bash
# 创建udev规则文件
sudo tee /etc/udev/rules.d/99-probe-rs.rules > /dev/null <<EOF
# ST-Link v2
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="3748", MODE="0666"
# ST-Link v2.1
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374b", MODE="0666"
# ST-Link v3
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="374f", MODE="0666"
# J-Link
SUBSYSTEM=="usb", ATTR{idVendor}=="1366", ATTR{idProduct}=="0101", MODE="0666"
# CMSIS-DAP
SUBSYSTEM=="usb", ATTR{idVendor}=="0d28", ATTR{idProduct}=="0204", MODE="0666"
EOF

# 重新加载udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## 5. 环境变量配置

### 5.1 必要的环境变量

将以下内容添加到你的shell配置文件（`~/.bashrc`, `~/.zshrc`等）：

```bash
# Rust环境
export PATH="$HOME/.cargo/bin:$PATH"

# 嵌入式开发相关
export DEFMT_LOG=debug

# 如果使用OpenOCD
# export OPENOCD_SCRIPTS=/usr/share/openocd/scripts
```

### 5.2 重新加载环境变量

```bash
# 重新加载配置文件
source ~/.bashrc
# 或者
source ~/.zshrc

# 验证环境变量
echo $PATH
echo $DEFMT_LOG
```

## 6. 创建第一个项目

### 6.1 使用模板创建项目

```bash
# 使用官方模板创建STM32项目
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart

# 输入项目名称
# Project Name: my-first-embedded-project

cd my-first-embedded-project
```

### 6.2 配置项目

编辑 `Cargo.toml` 添加必要的依赖：

```toml
[dependencies]
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"

# 如果需要RTT调试输出
rtt-target = { version = "0.4", features = ["cortex-m"] }
```

### 6.3 编译项目

```bash
# 编译项目
cargo build

# 检查代码
cargo check

# 运行clippy检查
cargo clippy

# 格式化代码
cargo fmt
```

## 7. 验证完整环境

### 7.1 环境检查脚本

创建一个脚本来验证环境配置：

```bash
#!/bin/bash
# check-environment.sh

echo "=== Rust嵌入式环境检查 ==="
echo

# 检查Rust工具链
echo "1. 检查Rust工具链..."
rustc --version || echo "❌ Rust未安装"
cargo --version || echo "❌ Cargo未安装"
rustup --version || echo "❌ Rustup未安装"
echo

# 检查目标架构
echo "2. 检查目标架构..."
rustup target list --installed | grep -E "thumbv|riscv" || echo "❌ 未安装嵌入式目标架构"
echo

# 检查工具
echo "3. 检查开发工具..."
cargo objdump -- --version > /dev/null 2>&1 && echo "✅ cargo-binutils已安装" || echo "❌ cargo-binutils未安装"
cargo generate --version > /dev/null 2>&1 && echo "✅ cargo-generate已安装" || echo "❌ cargo-generate未安装"
probe-rs --version > /dev/null 2>&1 && echo "✅ probe-rs已安装" || echo "❌ probe-rs未安装"
echo

# 检查环境变量
echo "4. 检查环境变量..."
echo "PATH包含cargo: $(echo $PATH | grep -q cargo && echo "✅" || echo "❌")"
echo "DEFMT_LOG设置: ${DEFMT_LOG:-未设置}"
echo

echo "=== 检查完成 ==="
```

运行检查：
```bash
chmod +x check-environment.sh
./check-environment.sh
```

### 7.2 测试编译

```bash
# 创建测试项目
cargo new --bin env-test
cd env-test

# 添加嵌入式依赖到Cargo.toml
echo '
[dependencies]
cortex-m = "0.7"
panic-halt = "0.2"' >> Cargo.toml

# 创建简单的嵌入式代码
cat > src/main.rs << 'EOF'
#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    loop {
        // 主循环
    }
}
EOF

# 编译测试
cargo build --target thumbv7em-none-eabihf
```

## 8. 常见问题解决

### 8.1 链接器错误

如果遇到链接器错误，可能需要安装额外的工具：

#### macOS
```bash
# 安装Xcode命令行工具
xcode-select --install
```

#### Ubuntu/Debian
```bash
sudo apt install build-essential
sudo apt install gcc-arm-none-eabi
```

#### Windows
安装Visual Studio Build Tools或完整的Visual Studio。

### 8.2 权限问题

如果在Linux上遇到USB设备权限问题：

```bash
# 将用户添加到dialout组
sudo usermod -a -G dialout $USER

# 重新登录或重启
```

### 8.3 网络问题

如果在中国大陆遇到网络问题，可以配置镜像：

```bash
# 配置Cargo镜像
mkdir -p ~/.cargo
cat > ~/.cargo/config.toml << 'EOF'
[source.crates-io]
replace-with = 'ustc'

[source.ustc]
registry = "https://mirrors.ustc.edu.cn/crates.io-index"
EOF

# 配置rustup镜像
export RUSTUP_DIST_SERVER=https://mirrors.ustc.edu.cn/rust-static
export RUSTUP_UPDATE_ROOT=https://mirrors.ustc.edu.cn/rust-static/rustup
```

## 9. 下一步

基础环境安装完成后，你可以：

1. 继续阅读 [硬件平台配置](./02-hardware-platforms.md)
2. 配置你的IDE或编辑器
3. 尝试创建第一个嵌入式项目
4. 学习调试工具的使用

## 小结

本章介绍了Rust嵌入式开发的基础环境配置：

- ✅ 安装Rust工具链和rustup
- ✅ 添加嵌入式目标架构
- ✅ 安装必要的开发工具
- ✅ 配置调试和烧录工具
- ✅ 设置环境变量
- ✅ 验证环境配置

完成这些步骤后，你就拥有了一个功能完整的Rust嵌入式开发环境！

---

**环境配置是成功的第一步，让我们继续配置特定的硬件平台！** 🚀
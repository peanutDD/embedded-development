# 项目模板和脚手架

## 概述

项目模板和脚手架工具可以大大提高开发效率，帮助快速创建标准化的嵌入式项目结构。本章节将介绍如何使用和创建各种项目模板，包括官方模板、社区模板和自定义模板。

## 官方模板工具

### 1. cargo-generate

`cargo-generate` 是Rust生态系统中最流行的项目模板工具。

#### 安装

```bash
cargo install cargo-generate
```

#### 使用方法

```bash
# 从GitHub模板创建项目
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart

# 指定项目名称
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart --name my-project

# 使用本地模板
cargo generate --path ./my-template

# 使用特定分支或标签
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart --branch main
```

### 2. 推荐的官方模板

#### Cortex-M 快速开始模板

```bash
# 通用Cortex-M模板
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart

# 项目结构
my-project/
├── .cargo/
│   └── config.toml
├── src/
│   └── main.rs
├── memory.x
├── Cargo.toml
├── build.rs
└── openocd.cfg
```

#### RTIC模板

```bash
# Real-Time Interrupt-driven Concurrency
cargo generate --git https://github.com/rtic-rs/cortex-m-rtic-quickstart
```

#### Embassy模板

```bash
# 异步嵌入式框架
cargo generate --git https://github.com/embassy-rs/embassy --subfolder examples/stm32f4
```

## 平台特定模板

### STM32 系列

#### 1. STM32F4 模板

```bash
# 创建STM32F4项目
cargo generate --git https://github.com/stm32-rs/stm32f4xx-hal --subfolder examples/template
```

**生成的项目结构：**

```
stm32f4-project/
├── .cargo/
│   └── config.toml          # 目标配置
├── src/
│   ├── main.rs              # 主程序
│   └── lib.rs               # 库文件
├── memory.x                 # 内存布局
├── Cargo.toml              # 依赖配置
├── build.rs                # 构建脚本
├── openocd.cfg             # OpenOCD配置
└── README.md               # 项目说明
```

**Cargo.toml 示例：**

```toml
[package]
name = "stm32f4-project"
version = "0.1.0"
edition = "2021"

[dependencies]
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"
stm32f4xx-hal = { version = "0.17", features = ["stm32f411", "rt"] }

[dependencies.nb]
version = "1.0"

# 可选的异步支持
embassy-executor = { version = "0.3", features = ["arch-cortex-m", "executor-thread", "integrated-timers"], optional = true }
embassy-time = { version = "0.1", features = ["tick-hz-32_768"], optional = true }
embassy-stm32 = { version = "0.1", features = ["stm32f411re", "time-driver-any", "exti"], optional = true }

[features]
default = []
embassy = ["embassy-executor", "embassy-time", "embassy-stm32"]

[[bin]]
name = "main"
path = "src/main.rs"

[profile.release]
debug = true
lto = true
opt-level = "s"

[profile.dev]
debug = true
opt-level = 1
```

#### 2. STM32H7 高性能模板

```bash
cargo generate --git https://github.com/stm32-rs/stm32h7xx-hal --subfolder examples/template
```

### ESP32 系列

#### 1. ESP32-C3 模板

```bash
# ESP32-C3 (RISC-V)
cargo generate --git https://github.com/esp-rs/esp-template

# 选择配置选项
# - MCU: esp32c3
# - Advanced template: true
# - Dev Containers: false
```

#### 2. ESP-IDF 模板

```bash
# 使用ESP-IDF框架
cargo generate --git https://github.com/esp-rs/esp-idf-template
```

**生成的项目特点：**

```toml
# Cargo.toml
[dependencies]
esp-idf-sys = { version = "0.33", default-features = false }
esp-idf-hal = "0.42"
esp-idf-svc = "0.47"
embedded-svc = "0.25"

[build-dependencies]
embuild = "0.31"
```

### Raspberry Pi Pico

#### 1. Pico模板

```bash
# Raspberry Pi Pico (RP2040)
cargo generate --git https://github.com/rp-rs/rp2040-project-template
```

#### 2. 高级Pico模板

```bash
# 包含更多功能的模板
cargo generate --git https://github.com/rp-rs/rp2040-hal --subfolder rp2040-hal/examples/template
```

### Nordic nRF 系列

#### 1. nRF52 模板

```bash
# Nordic nRF52系列
cargo generate --git https://github.com/nrf-rs/nrf-project-template
```

#### 2. nRF Connect SDK 模板

```bash
# 使用nRF Connect SDK
cargo generate --git https://github.com/nrf-rs/nrf-connect-template
```

## 自定义模板创建

### 1. 模板结构

创建自己的模板需要遵循特定的目录结构：

```
my-template/
├── cargo-generate.toml      # 模板配置
├── .cargo/
│   └── config.toml.liquid   # 动态配置文件
├── src/
│   └── main.rs.liquid       # 动态源文件
├── Cargo.toml.liquid        # 动态Cargo配置
├── memory.x.liquid          # 动态内存配置
└── README.md.liquid         # 动态说明文件
```

### 2. 模板配置文件

**cargo-generate.toml:**

```toml
[template]
cargo_generate_version = ">=0.10.0"

[hooks]
pre = ["pre-script.rhai"]
post = ["post-script.rhai"]

[placeholders.mcu]
type = "string"
prompt = "选择MCU型号"
choices = ["stm32f411", "stm32f407", "stm32h743", "esp32c3", "rp2040"]
default = "stm32f411"

[placeholders.use_rtic]
type = "bool"
prompt = "使用RTIC框架?"
default = false

[placeholders.use_embassy]
type = "bool"
prompt = "使用Embassy异步框架?"
default = false

[placeholders.use_defmt]
type = "bool"
prompt = "使用defmt日志框架?"
default = true

[placeholders.author_name]
type = "string"
prompt = "作者姓名"
default = "Your Name"

[placeholders.author_email]
type = "string"
prompt = "作者邮箱"
default = "your.email@example.com"
```

### 3. 动态Cargo.toml模板

**Cargo.toml.liquid:**

```toml
[package]
name = "{{project-name}}"
version = "0.1.0"
edition = "2021"
authors = ["{{author_name}} <{{author_email}}>"]
description = "Embedded project for {{mcu}}"

[dependencies]
cortex-m = "0.7"
cortex-m-rt = "0.7"
panic-halt = "0.2"

{% if mcu == "stm32f411" -%}
stm32f4xx-hal = { version = "0.17", features = ["stm32f411", "rt"] }
{% elsif mcu == "stm32f407" -%}
stm32f4xx-hal = { version = "0.17", features = ["stm32f407", "rt"] }
{% elsif mcu == "stm32h743" -%}
stm32h7xx-hal = { version = "0.15", features = ["stm32h743", "rt"] }
{% elsif mcu == "esp32c3" -%}
esp32c3-hal = "0.12"
esp-backtrace = { version = "0.8", features = ["esp32c3", "panic-handler", "exception-handler", "print-uart"] }
{% elsif mcu == "rp2040" -%}
rp2040-hal = { version = "0.9", features = ["rt"] }
rp2040-boot2 = "0.3"
{% endif -%}

{% if use_rtic -%}
rtic = { version = "2.0", features = ["thumbv7-backend"] }
rtic-monotonics = { version = "1.0", features = ["cortex-m-systick"] }
{% endif -%}

{% if use_embassy -%}
embassy-executor = { version = "0.3", features = ["arch-cortex-m", "executor-thread", "integrated-timers"] }
embassy-time = { version = "0.1", features = ["tick-hz-32_768"] }
{% if mcu contains "stm32" -%}
embassy-stm32 = { version = "0.1", features = ["{{mcu}}", "time-driver-any", "exti"] }
{% endif -%}
{% endif -%}

{% if use_defmt -%}
defmt = "0.3"
defmt-rtt = "0.4"
panic-probe = { version = "0.3", features = ["print-defmt"] }
{% endif -%}

[profile.release]
debug = true
lto = true
opt-level = "s"

[profile.dev]
debug = true
opt-level = 1
```

### 4. 动态源文件模板

**src/main.rs.liquid:**

```rust
#![no_std]
#![no_main]

{% if use_defmt -%}
use defmt_rtt as _;
use panic_probe as _;
use defmt::info;
{% else -%}
use panic_halt as _;
{% endif -%}

{% if use_rtic -%}
#[rtic::app(device = {{mcu}}_pac, dispatchers = [EXTI0])]
mod app {
    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        {% if use_defmt -%}
        info!("RTIC application started");
        {% endif -%}
        
        // 初始化代码
        
        (Shared {}, Local {}, init::Monotonics())
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
{% elsif use_embassy -%}
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    {% if use_defmt -%}
    info!("Embassy application started");
    {% endif -%}
    
    // 初始化代码
    
    loop {
        {% if use_defmt -%}
        info!("Hello from Embassy!");
        {% endif -%}
        Timer::after(Duration::from_secs(1)).await;
    }
}
{% else -%}
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    {% if use_defmt -%}
    info!("Application started");
    {% endif -%}
    
    // 初始化代码
    
    loop {
        // 主循环
    }
}
{% endif -%}
```

### 5. 钩子脚本

**pre-script.rhai:**

```rhai
// 预处理脚本
let mcu = variable::get("mcu");

if mcu == "esp32c3" {
    // ESP32特殊处理
    variable::set("target", "riscv32imc-unknown-none-elf");
} else {
    // ARM Cortex-M目标
    variable::set("target", "thumbv7em-none-eabihf");
}

print("正在为 " + mcu + " 创建项目...");
```

**post-script.rhai:**

```rhai
// 后处理脚本
let project_name = variable::get("project-name");
let mcu = variable::get("mcu");

print("项目 '" + project_name + "' 已创建完成!");
print("目标MCU: " + mcu);
print("");
print("下一步:");
print("1. cd " + project_name);
print("2. cargo build");
print("3. cargo run");
```

## 高级模板功能

### 1. 条件文件包含

```toml
# cargo-generate.toml
[conditional.'use_rtic == true']
ignore = ["src/embassy_main.rs"]

[conditional.'use_embassy == true']
ignore = ["src/rtic_main.rs", "src/main.rs"]

[conditional.'mcu == "esp32c3"']
ignore = ["memory.x", "openocd.cfg"]
```

### 2. 文件重命名

```toml
# cargo-generate.toml
[template]
include = ["**/*"]
exclude = ["target/**/*"]

[[template.file]]
path = "src/main_{{framework}}.rs"
to = "src/main.rs"
when = "framework != 'bare-metal'"
```

### 3. 多目标支持

**memory.x.liquid:**

```ld
{% if mcu == "stm32f411" -%}
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}
{% elsif mcu == "stm32h743" -%}
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM : ORIGIN = 0x20000000, LENGTH = 512K
  ITCM : ORIGIN = 0x00000000, LENGTH = 64K
  DTCM : ORIGIN = 0x20000000, LENGTH = 128K
}
{% elsif mcu == "rp2040" -%}
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}
{% endif -%}

/* 通用配置 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
```

## 企业级模板

### 1. 公司标准模板

```toml
# cargo-generate.toml
[template]
cargo_generate_version = ">=0.10.0"

[placeholders.company]
type = "string"
prompt = "公司名称"
default = "Your Company"

[placeholders.project_type]
type = "string"
prompt = "项目类型"
choices = ["sensor", "actuator", "gateway", "controller"]
default = "sensor"

[placeholders.compliance]
type = "string"
prompt = "合规标准"
choices = ["automotive", "medical", "industrial", "consumer"]
default = "industrial"
```

### 2. 合规性配置

**Cargo.toml.liquid (企业版):**

```toml
[package]
name = "{{project-name}}"
version = "0.1.0"
edition = "2021"
authors = ["{{company}} Development Team"]
license = "Proprietary"
description = "{{project_type}} device for {{compliance}} applications"

[dependencies]
# 基础依赖
cortex-m = "0.7"
cortex-m-rt = "0.7"

{% if compliance == "automotive" -%}
# 汽车级依赖
panic-abort = "0.3"  # 不使用unwinding
heapless = { version = "0.7", default-features = false }
{% elsif compliance == "medical" -%}
# 医疗级依赖
panic-halt = "0.2"
nb = "1.0"
{% endif -%}

# 安全相关
{% if compliance == "automotive" or compliance == "medical" -%}
typenum = { version = "1.16", default-features = false }
generic-array = { version = "0.14", default-features = false }
{% endif -%}

[profile.release]
{% if compliance == "automotive" or compliance == "medical" -%}
panic = "abort"
{% endif -%}
debug = false
lto = true
opt-level = "s"
codegen-units = 1

[profile.dev]
panic = "abort"
debug = true
opt-level = 1
```

## 模板管理

### 1. 本地模板库

```bash
# 创建本地模板库
mkdir -p ~/.cargo-generate/templates
cd ~/.cargo-generate/templates

# 克隆常用模板
git clone https://github.com/your-company/embedded-templates.git company-templates
git clone https://github.com/rust-embedded/cortex-m-quickstart.git cortex-m

# 使用本地模板
cargo generate --path ~/.cargo-generate/templates/company-templates/stm32-sensor
```

### 2. 模板版本管理

```bash
# 使用特定版本的模板
cargo generate --git https://github.com/your-org/template.git --tag v1.2.0

# 使用特定分支
cargo generate --git https://github.com/your-org/template.git --branch feature/new-mcu
```

### 3. 模板更新

```bash
# 更新本地模板缓存
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart.git --force-git-init

# 清理模板缓存
rm -rf ~/.cargo/registry/cache/github.com-*/
```

## 最佳实践

### 1. 模板设计原则

- **最小化原则**: 只包含必要的文件和依赖
- **可配置性**: 通过placeholders支持多种配置
- **文档完整**: 提供清晰的README和注释
- **版本兼容**: 明确支持的工具链版本

### 2. 项目结构标准化

```
embedded-project/
├── .cargo/
│   └── config.toml          # 构建配置
├── .github/
│   └── workflows/           # CI/CD配置
├── docs/                    # 项目文档
├── examples/                # 示例代码
├── src/
│   ├── main.rs             # 主程序
│   ├── lib.rs              # 库接口
│   └── drivers/            # 驱动模块
├── tests/                   # 测试代码
├── benches/                 # 性能测试
├── memory.x                 # 内存布局
├── Cargo.toml              # 项目配置
├── build.rs                # 构建脚本
├── README.md               # 项目说明
└── CHANGELOG.md            # 变更日志
```

### 3. 依赖管理

```toml
# 推荐的依赖版本策略
[dependencies]
# 核心依赖使用精确版本
cortex-m = "=0.7.7"
cortex-m-rt = "=0.7.3"

# HAL使用兼容版本
stm32f4xx-hal = "~0.17.1"

# 开发依赖可以更宽松
[dev-dependencies]
cortex-m-semihosting = "0.5"
```

## 故障排除

### 1. 常见问题

**模板生成失败:**
```bash
# 检查网络连接
curl -I https://github.com/rust-embedded/cortex-m-quickstart.git

# 使用详细输出
cargo generate --git https://github.com/rust-embedded/cortex-m-quickstart.git --verbose
```

**依赖解析错误:**
```bash
# 更新索引
cargo update

# 检查目标安装
rustup target list --installed
```

### 2. 调试技巧

```bash
# 生成到临时目录进行调试
cargo generate --git https://your-template.git --destination /tmp/debug-project

# 检查生成的文件
find /tmp/debug-project -type f -name "*.toml" -exec cat {} \;
```

## 下一步

掌握项目模板后，建议：

1. 学习[交叉编译配置](./06-cross-compilation.md)优化构建过程
2. 了解[仿真和测试](./07-simulation-testing.md)环境搭建
3. 实践创建自己的项目模板

---

**现在你可以快速创建标准化的嵌入式项目了！** 🚀
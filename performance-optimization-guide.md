# 嵌入式Rust开发性能优化指南

本指南旨在帮助解决嵌入式Rust开发中遇到的性能问题，特别是Rust Analyzer索引缓慢和编译时间过长的问题。

## 问题诊断

### 常见性能问题症状

1. **Rust Analyzer索引缓慢**
   - 日志显示 "overly long loop turn took XXXms"
   - 索引过程中IDE响应缓慢
   - 代码补全和错误检查延迟

2. **编译时间过长**
   - `cargo build` 执行时间超过预期
   - 依赖项重复编译
   - 内存使用过高

## 优化策略

### 1. 依赖项优化

#### 禁用不必要的默认特性
```toml
[dependencies]
# 禁用默认特性，只启用需要的功能
stm32f4xx-hal = { version = "0.19", features = ["stm32f407", "rt"], default-features = false }
embedded-hal = { version = "1.0.0-rc.2", default-features = false }
heapless = { version = "0.8", default-features = false }
```

#### 条件编译调试功能
```toml
[dependencies]
# 只在需要时启用RTT调试
rtt-target = { version = "0.4", optional = true }

[features]
default = ["rtt"]
rtt = ["dep:rtt-target"]
```

#### 移除未使用的依赖
```toml
# 注释或删除不需要的依赖
# defmt = "0.3"
# defmt-rtt = "0.4"
```

### 2. 编译配置优化

#### Cargo.toml 优化
```toml
[profile.dev]
codegen-units = 1      # 减少代码生成单元
debug = 2              # 保持调试信息
opt-level = "s"        # 优化代码大小
lto = false            # 开发时禁用LTO以加快编译

[profile.release]
codegen-units = 1
lto = true             # 发布时启用LTO
opt-level = "s"        # 优化代码大小
```

#### .cargo/config.toml 优化
```toml
[build]
target = "thumbv7em-none-eabihf"
jobs = 2               # 限制并行作业数以减少内存使用

[target.thumbv7em-none-eabihf]
rustflags = [
  "-C", "link-arg=-Tlink.x",
  "-C", "target-cpu=cortex-m4",
  "-C", "target-feature=+thumb2,+dsp",
  "-C", "opt-level=s",
  "-C", "codegen-units=1",
]

[env]
DEFMT_LOG = "info"     # 减少日志级别
RUST_LOG = "warn"
```

### 3. 工具链配置

#### rust-toolchain.toml
```toml
[toolchain]
channel = "stable"     # 使用稳定版本
targets = ["thumbv7em-none-eabihf"]
components = ["rust-src", "rustfmt", "clippy"]
profile = "minimal"    # 最小化安装
```

### 4. 代码优化

#### 内存使用优化
```rust
// 使用更小的数据类型
struct AppState {
    blink_counter: u16,        // 而不是u32
    breathing_counter: u16,    // 而不是u32
    mode_change_count: u16,    // 而不是u32
}

// 使用饱和运算避免溢出检查
counter = counter.saturating_add(1);
counter = counter.wrapping_add(1);
```

#### 函数内联优化
```rust
#[inline]
fn frequently_called_function() {
    // 频繁调用的小函数
}
```

#### 条件编译调试代码
```rust
#[cfg(feature = "rtt")]
use rtt_target::{rprintln, rtt_init_print};

#[cfg(not(feature = "rtt"))]
macro_rules! rprintln {
    ($($arg:tt)*) => {};
}

// 在代码中使用条件编译
#[cfg(feature = "rtt")]
rprintln!("Debug message");
```

### 5. 系统级优化

#### 降低时钟频率
```rust
// 开发时使用较低的系统时钟
let clocks = rcc
    .cfgr
    .use_hse(8.MHz())
    .sysclk(84.MHz())      // 而不是168MHz
    .freeze();

// 降低定时器频率
timer.start(25.Hz()).unwrap(); // 而不是50Hz
```

#### 减少中断频率
```rust
// 使用较低的定时器频率减少CPU负载
let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
timer.start(25.Hz()).unwrap(); // 25Hz而不是50Hz或更高
```

### 6. IDE配置优化

#### VS Code settings.json
```json
{
    "rust-analyzer.cargo.target": "thumbv7em-none-eabihf",
    "rust-analyzer.checkOnSave.allTargets": false,
    "rust-analyzer.cargo.features": ["rtt"],
    "rust-analyzer.cargo.noDefaultFeatures": true,
    "rust-analyzer.procMacro.enable": false,
    "rust-analyzer.imports.granularity.group": "module",
    "rust-analyzer.completion.autoimport.enable": false
}
```

## 性能监控

### 编译时间测量
```bash
# 测量编译时间
time cargo build

# 详细编译信息
cargo build --timings

# 分析依赖编译时间
cargo build -Z timings
```

### 内存使用监控
```bash
# 监控编译内存使用
/usr/bin/time -v cargo build

# 限制内存使用
ulimit -m 2097152  # 限制为2GB
```

### 代码大小分析
```bash
# 分析二进制大小
cargo bloat --release --crates

# 分析符号大小
cargo nm --release | head -20
```

## 最佳实践

### 1. 渐进式优化
- 先优化依赖项配置
- 再优化编译配置
- 最后优化代码实现

### 2. 性能测试
- 在优化前后测量编译时间
- 监控内存使用情况
- 验证功能正确性

### 3. 文档记录
- 记录优化前后的性能数据
- 文档化配置更改的原因
- 建立项目特定的优化指南

## 故障排除

### 编译错误
```bash
# 清理构建缓存
cargo clean

# 重新生成Cargo.lock
rm Cargo.lock
cargo build

# 检查依赖冲突
cargo tree
```

### Rust Analyzer问题
```bash
# 重启Rust Analyzer
# VS Code: Ctrl+Shift+P -> "Rust Analyzer: Restart Server"

# 清理Rust Analyzer缓存
rm -rf ~/.cache/rust-analyzer/

# 检查配置
rust-analyzer --version
```

## 总结

通过系统性的优化策略，可以显著改善嵌入式Rust开发的性能：

1. **依赖优化**：减少不必要的特性和依赖
2. **编译配置**：优化编译参数和并行度
3. **代码优化**：使用高效的数据结构和算法
4. **系统优化**：降低时钟频率和中断频率
5. **工具配置**：优化IDE和工具链设置

这些优化措施不仅能提高开发效率，还能减少资源消耗，特别适合在资源受限的开发环境中使用。
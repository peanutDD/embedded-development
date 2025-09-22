# 构建自动化示例

这个示例展示了嵌入式Rust项目的完整构建自动化流程，包括本地构建脚本、CI/CD配置和部署策略。

## 项目结构

```
build-automation/
├── src/
│   └── main.rs              # 主程序源码
├── scripts/
│   └── build.sh             # 构建脚本
├── .cargo/
│   └── config.toml          # Cargo配置
├── .github/
│   └── workflows/
│       └── ci.yml           # GitHub Actions CI/CD
├── build.rs                 # 构建脚本
├── memory.x                 # 内存布局
├── Cargo.toml              # 项目配置
└── README.md               # 项目文档
```

## 功能特性

### 1. 构建时信息生成
- 自动生成构建时间戳
- 获取Git提交哈希
- 记录构建配置和目标架构
- 生成版本信息

### 2. 多配置构建支持
- Debug构建：优化调试体验
- Release构建：性能优化
- Size-opt构建：大小优化
- Speed-opt构建：速度优化

### 3. 自动化构建脚本
- 代码格式化检查
- Clippy静态分析
- 单元测试执行
- 二进制文件生成
- 构建产物验证

### 4. CI/CD流水线
- 多目标架构构建
- 安全审计
- 文档生成
- 自动化部署
- 发布管理

## 快速开始

### 环境要求

1. **Rust工具链**
   ```bash
   # 安装Rust
   curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
   
   # 添加嵌入式目标
   rustup target add thumbv7em-none-eabihf
   rustup target add thumbv6m-none-eabi
   
   # 安装组件
   rustup component add rustfmt clippy
   ```

2. **ARM工具链**
   ```bash
   # macOS
   brew install armmbed/formulae/arm-none-eabi-gcc
   
   # Ubuntu/Debian
   sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi
   
   # Arch Linux
   sudo pacman -S arm-none-eabi-gcc arm-none-eabi-binutils
   ```

3. **调试工具（可选）**
   ```bash
   # OpenOCD
   brew install openocd  # macOS
   sudo apt-get install openocd  # Ubuntu
   
   # ST-Link工具
   brew install stlink  # macOS
   sudo apt-get install stlink-tools  # Ubuntu
   ```

### 本地构建

1. **基本构建**
   ```bash
   # Debug构建
   cargo build --target thumbv7em-none-eabihf
   
   # Release构建
   cargo build --target thumbv7em-none-eabihf --release
   
   # 大小优化构建
   cargo build --target thumbv7em-none-eabihf --profile size-opt
   ```

2. **使用构建脚本**
   ```bash
   # 完整构建流程
   ./scripts/build.sh --all
   
   # 仅构建
   ./scripts/build.sh --release
   
   # 清理并构建
   ./scripts/build.sh --clean --release
   
   # 生成文档
   ./scripts/build.sh --docs
   
   # 查看帮助
   ./scripts/build.sh --help
   ```

3. **检查构建结果**
   ```bash
   # 查看二进制大小
   arm-none-eabi-size target/thumbv7em-none-eabihf/release/build-automation
   
   # 查看生成的产物
   ls -la artifacts/
   ```

### 代码质量检查

```bash
# 格式化代码
cargo fmt --all

# 静态分析
cargo clippy --target thumbv7em-none-eabihf --all-targets --all-features -- -D warnings

# 运行测试（如果支持）
cargo test --lib

# 安全审计
cargo audit
```

## 配置说明

### Cargo.toml配置

项目支持多种构建配置：

```toml
[profile.dev]
opt-level = "s"          # 大小优化
debug = 2                # 完整调试信息

[profile.release]
opt-level = 3            # 性能优化
lto = true              # 链接时优化
codegen-units = 1       # 单个代码生成单元

[profile.size-opt]
inherits = "release"
opt-level = "s"         # 大小优化
lto = true

[profile.speed-opt]
inherits = "release"
opt-level = 3           # 速度优化
lto = "fat"
```

### 内存布局配置

`memory.x`文件定义了目标设备的内存布局：

```
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}
```

### Cargo配置

`.cargo/config.toml`文件配置了构建选项：

```toml
[target.thumbv7em-none-eabihf]
rustflags = [
  "-C", "link-arg=-Tlink.x",
]

[build]
target = "thumbv7em-none-eabihf"
```

## CI/CD流水线

### GitHub Actions工作流

项目包含完整的CI/CD配置：

1. **检查阶段**
   - 代码格式检查
   - Clippy静态分析
   - 基本编译检查

2. **测试阶段**
   - 单元测试执行
   - 集成测试（如果支持）

3. **构建阶段**
   - 多目标架构构建
   - 多配置构建
   - 二进制产物生成

4. **安全阶段**
   - 依赖安全审计
   - 许可证检查

5. **部署阶段**
   - 文档生成和部署
   - 发布管理
   - 环境部署

### 触发条件

- **推送到main/develop分支**：完整CI流程
- **Pull Request**：检查和测试
- **发布标签**：构建发布版本
- **手动触发**：按需执行

## 部署策略

### 本地部署

```bash
# 使用ST-Link烧录
st-flash write artifacts/build-automation.bin 0x8000000

# 使用OpenOCD烧录
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program artifacts/build-automation.bin 0x08000000 verify reset exit"

# 使用GDB调试
arm-none-eabi-gdb target/thumbv7em-none-eabihf/debug/build-automation
```

### 自动化部署

项目支持多种部署环境：

1. **开发环境**：每次推送自动部署
2. **测试环境**：develop分支自动部署
3. **生产环境**：发布标签手动部署

## 监控和维护

### 构建监控

- 构建时间跟踪
- 二进制大小监控
- 依赖更新检查
- 安全漏洞扫描

### 运行时监控

- 系统健康检查
- 性能指标收集
- 错误日志记录
- 远程诊断支持

## 故障排除

### 常见问题

1. **链接器错误**
   ```
   解决方案：检查memory.x文件和链接器配置
   ```

2. **目标架构不匹配**
   ```
   解决方案：确认rustup target已安装
   rustup target add thumbv7em-none-eabihf
   ```

3. **ARM工具链缺失**
   ```
   解决方案：安装ARM GCC工具链
   ```

4. **内存不足**
   ```
   解决方案：优化代码或调整内存布局
   ```

### 调试技巧

1. **使用详细输出**
   ```bash
   cargo build --target thumbv7em-none-eabihf --verbose
   ```

2. **检查生成的汇编**
   ```bash
   cargo rustc --target thumbv7em-none-eabihf --release -- --emit asm
   ```

3. **分析二进制大小**
   ```bash
   cargo bloat --target thumbv7em-none-eabihf --release
   ```

## 扩展功能

### 自定义构建步骤

可以在`build.rs`中添加自定义构建逻辑：

```rust
fn main() {
    // 自定义构建步骤
    generate_version_info();
    configure_hardware_features();
    optimize_for_target();
}
```

### 集成其他工具

- **cargo-embed**：简化烧录和调试
- **probe-run**：运行时日志输出
- **defmt**：高效日志框架
- **flip-link**：栈溢出保护

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 贡献指南

欢迎提交Issue和Pull Request！请确保：

1. 代码通过所有检查
2. 添加适当的测试
3. 更新相关文档
4. 遵循项目代码风格

## 相关资源

- [Rust嵌入式开发指南](https://docs.rust-embedded.org/)
- [cortex-m-rt文档](https://docs.rs/cortex-m-rt/)
- [STM32F4xx HAL文档](https://docs.rs/stm32f4xx-hal/)
- [RTIC框架文档](https://rtic.rs/)
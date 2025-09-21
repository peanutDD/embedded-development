# 🤝 贡献指南

感谢您对Rust嵌入式开发教程项目的关注！我们欢迎各种形式的贡献，无论是代码、文档、测试用例还是问题反馈。

## 📋 目录

- [贡献方式](#贡献方式)
- [开发环境设置](#开发环境设置)
- [贡献流程](#贡献流程)
- [代码规范](#代码规范)
- [文档规范](#文档规范)
- [测试要求](#测试要求)
- [提交规范](#提交规范)
- [问题报告](#问题报告)
- [功能请求](#功能请求)
- [社区准则](#社区准则)

## 🎯 贡献方式

### 🐛 问题修复
- 修复现有代码中的bug
- 改进错误处理和边界情况
- 优化性能问题
- 修正文档错误

### ✨ 功能增强
- 添加新的示例项目
- 实现新的硬件平台支持
- 增加新的通信协议
- 扩展现有功能

### 📚 文档改进
- 完善教程内容
- 添加更多注释
- 翻译文档
- 改进README和指南

### 🧪 测试贡献
- 编写单元测试
- 添加集成测试
- 创建基准测试
- 验证硬件兼容性

### 🎨 用户体验
- 改进代码结构
- 优化构建配置
- 简化安装流程
- 增强错误信息

## 🛠️ 开发环境设置

### 基础要求
```bash
# 1. 安装Rust (1.70+)
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

# 2. 添加嵌入式目标平台
rustup target add thumbv7em-none-eabihf
rustup target add thumbv6m-none-eabi
rustup target add thumbv7m-none-eabi

# 3. 安装开发工具
cargo install probe-run
cargo install cargo-embed
cargo install cargo-flash

# 4. 安装代码质量工具
rustup component add rustfmt
rustup component add clippy
```

### 推荐工具
```bash
# 代码覆盖率
cargo install cargo-tarpaulin

# 安全审计
cargo install cargo-audit

# 依赖检查
cargo install cargo-outdated

# 文档生成
cargo install mdbook
```

### 硬件设置
- **开发板**: STM32F4 Discovery, ESP32-C3, Raspberry Pi Pico
- **调试器**: ST-Link V2, J-Link, 或板载调试器
- **传感器**: DHT22, BMP280, MPU6050 (可选)

## 🔄 贡献流程

### 1. 准备工作
```bash
# Fork项目到你的GitHub账户
# 克隆你的fork
git clone https://github.com/YOUR_USERNAME/embedded-development.git
cd embedded-development

# 添加上游仓库
git remote add upstream https://github.com/original-repo/embedded-development.git

# 创建开发分支
git checkout -b feature/your-feature-name
```

### 2. 开发过程
```bash
# 保持代码同步
git fetch upstream
git rebase upstream/main

# 进行开发
# ... 编写代码 ...

# 运行测试
./scripts/verify_all.sh

# 检查代码质量
cargo fmt --all
cargo clippy --all-targets -- -D warnings
```

### 3. 提交更改
```bash
# 添加更改
git add .

# 提交更改 (遵循提交规范)
git commit -m "feat: add new sensor driver for BME280"

# 推送到你的fork
git push origin feature/your-feature-name
```

### 4. 创建Pull Request
1. 在GitHub上创建Pull Request
2. 填写详细的PR描述
3. 链接相关的Issue
4. 等待代码审查
5. 根据反馈进行修改

## 📝 代码规范

### Rust代码风格
```rust
// ✅ 好的示例
use embedded_hal::digital::v2::OutputPin;
use cortex_m::delay::Delay;

/// LED控制器结构体
/// 
/// 提供基本的LED开关控制功能
pub struct LedController<PIN> {
    pin: PIN,
    state: bool,
}

impl<PIN> LedController<PIN>
where
    PIN: OutputPin,
{
    /// 创建新的LED控制器
    pub fn new(pin: PIN) -> Self {
        Self {
            pin,
            state: false,
        }
    }
    
    /// 打开LED
    pub fn turn_on(&mut self) -> Result<(), PIN::Error> {
        self.pin.set_high()?;
        self.state = true;
        Ok(())
    }
}
```

### 命名规范
- **函数**: `snake_case` - `read_sensor_data()`
- **结构体**: `PascalCase` - `SensorManager`
- **常量**: `SCREAMING_SNAKE_CASE` - `MAX_BUFFER_SIZE`
- **模块**: `snake_case` - `sensor_drivers`

### 错误处理
```rust
// ✅ 使用Result类型
pub fn read_temperature() -> Result<f32, SensorError> {
    // 实现
}

// ✅ 自定义错误类型
#[derive(Debug, thiserror::Error)]
pub enum SensorError {
    #[error("I2C communication failed")]
    I2cError,
    #[error("Invalid sensor data: {0}")]
    InvalidData(String),
}
```

### 文档注释
```rust
/// 温度传感器驱动
/// 
/// 支持DHT22和DHT11传感器，提供温度和湿度读取功能。
/// 
/// # 示例
/// 
/// ```rust
/// let mut sensor = TemperatureSensor::new(pin);
/// let temp = sensor.read_temperature()?;
/// println!("Temperature: {:.1}°C", temp);
/// ```
/// 
/// # 错误
/// 
/// 当传感器通信失败时返回 [`SensorError`]
pub struct TemperatureSensor<PIN> {
    // 字段
}
```

## 📖 文档规范

### README结构
每个章节的README应包含：
1. **概述** - 章节目标和内容
2. **学习目标** - 具体的学习成果
3. **硬件要求** - 需要的硬件组件
4. **软件依赖** - Rust crates和工具
5. **快速开始** - 运行示例的步骤
6. **项目结构** - 文件组织说明
7. **详细教程** - 分步骤的教学内容
8. **练习题** - 巩固学习的练习
9. **参考资源** - 相关文档和链接

### 代码注释
```rust
// ✅ 好的注释
/// 配置PWM定时器
/// 
/// 设置定时器频率为1kHz，占空比为50%
/// 这将产生一个方波信号用于控制舵机
fn configure_pwm_timer(&mut self) -> Result<(), TimerError> {
    // 计算预分频值以获得1kHz频率
    let prescaler = self.clock_freq / 1000 - 1;
    
    // 设置自动重载值为50%占空比
    let auto_reload = 1000 / 2;
    
    // 应用配置
    self.timer.set_prescaler(prescaler);
    self.timer.set_auto_reload(auto_reload);
    
    Ok(())
}
```

### Markdown格式
- 使用清晰的标题层次
- 添加代码语法高亮
- 包含图表和示意图
- 提供可点击的链接
- 使用表格组织信息

## 🧪 测试要求

### 单元测试
```rust
#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_led_controller_creation() {
        let pin = MockPin::new();
        let controller = LedController::new(pin);
        assert!(!controller.is_on());
    }
    
    #[test]
    fn test_led_turn_on() {
        let mut pin = MockPin::new();
        let mut controller = LedController::new(pin);
        
        controller.turn_on().unwrap();
        assert!(controller.is_on());
    }
}
```

### 集成测试
```rust
// tests/integration_test.rs
use embedded_development::sensor_hub::SensorHub;

#[tokio::test]
async fn test_sensor_hub_integration() {
    let mut hub = SensorHub::new().await;
    
    // 添加传感器
    hub.add_sensor("temp", Box::new(MockTemperatureSensor::new()));
    
    // 读取数据
    let data = hub.read_all_sensors().await.unwrap();
    assert!(!data.is_empty());
}
```

### 硬件测试
```rust
// 仅在有硬件时运行
#[cfg(feature = "hardware-test")]
#[test]
fn test_real_hardware() {
    // 硬件测试代码
}
```

## 📋 提交规范

### 提交消息格式
```
<type>(<scope>): <description>

[optional body]

[optional footer]
```

### 提交类型
- `feat`: 新功能
- `fix`: 修复bug
- `docs`: 文档更新
- `style`: 代码格式化
- `refactor`: 代码重构
- `test`: 添加测试
- `chore`: 构建工具或辅助工具的变动

### 示例
```bash
feat(gpio): add support for GPIO interrupt handling

- Implement interrupt callback registration
- Add debouncing for button inputs
- Update documentation with examples

Closes #123
```

## 🐛 问题报告

### 报告Bug
使用以下模板报告问题：

```markdown
## Bug描述
简要描述遇到的问题

## 复现步骤
1. 执行命令 `cargo run --example led_blink`
2. 观察LED行为
3. 发现LED不闪烁

## 预期行为
LED应该每秒闪烁一次

## 实际行为
LED保持常亮状态

## 环境信息
- OS: macOS 13.0
- Rust版本: 1.70.0
- 硬件: STM32F4 Discovery
- 分支: main

## 附加信息
错误日志、截图等
```

### 功能请求
```markdown
## 功能描述
希望添加对ESP32-S3的支持

## 使用场景
开发WiFi连接的IoT设备

## 建议实现
1. 添加ESP32-S3 HAL支持
2. 创建WiFi连接示例
3. 更新文档

## 替代方案
使用ESP32-C3作为替代
```

## 🌟 社区准则

### 行为准则
- **尊重他人**: 保持友善和专业
- **建设性反馈**: 提供有用的建议
- **包容性**: 欢迎不同背景的贡献者
- **学习态度**: 保持开放和学习的心态

### 沟通方式
- **GitHub Issues**: 报告问题和功能请求
- **GitHub Discussions**: 技术讨论和问答
- **Pull Request**: 代码审查和讨论
- **邮件**: 私人或敏感问题

### 审查流程
1. **自动检查**: CI/CD流水线验证
2. **代码审查**: 维护者审查代码质量
3. **测试验证**: 确保功能正常工作
4. **文档检查**: 验证文档完整性
5. **合并**: 满足所有要求后合并

## 🎖️ 贡献者认可

### 贡献等级
- **🥉 贡献者**: 提交第一个PR
- **🥈 活跃贡献者**: 5个以上PR被合并
- **🥇 核心贡献者**: 重要功能或大量贡献
- **💎 维护者**: 长期维护和指导

### 认可方式
- README中的贡献者列表
- 发布说明中的感谢
- 特殊徽章和标签
- 社区推荐

## 📞 获取帮助

### 联系方式
- **GitHub Issues**: 技术问题和bug报告
- **GitHub Discussions**: 一般讨论和问答
- **邮件**: embedded-rust-tutorial@example.com
- **社区论坛**: [Rust嵌入式论坛](https://users.rust-lang.org/c/embedded/)

### 常见问题
查看 [FAQ文档](FAQ.md) 了解常见问题的解答。

### 学习资源
- [Rust官方文档](https://doc.rust-lang.org/)
- [嵌入式Rust之书](https://docs.rust-embedded.org/book/)
- [项目Wiki](https://github.com/your-username/embedded-development/wiki)

---

再次感谢您的贡献！每一个贡献都让这个项目变得更好，帮助更多人学习Rust嵌入式开发。

**🚀 让我们一起构建最好的Rust嵌入式教程！**
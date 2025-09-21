# 嵌入式测试框架 (Embedded Test Framework)

一个专为嵌入式Rust开发设计的综合测试框架，提供单元测试、集成测试、硬件测试和性能测试功能。

## 项目特性

### 测试类型
- **单元测试**: 纯函数和数据结构测试
- **集成测试**: 硬件抽象层和通信协议测试
- **硬件测试**: GPIO、UART、SPI、I2C等外设测试
- **性能测试**: 执行时间、内存使用和吞吐量测试
- **压力测试**: 长时间运行和极限条件测试

### 框架特性
- **实时测试**: 基于RTIC的实时测试执行
- **模拟硬件**: 支持硬件模拟和存根
- **性能分析**: 详细的性能指标收集
- **内存分析**: 内存泄漏检测和使用统计
- **测试报告**: 自动生成测试报告和统计信息
- **属性测试**: 基于属性的随机测试
- **基准测试**: 性能基准测试和比较

## 硬件要求

### 开发板
- STM32F401RE Nucleo板或兼容板
- ARM Cortex-M4处理器
- 最小512KB Flash，96KB RAM

### 外设连接
```
LED (测试指示器):
- PA5 -> 板载LED

调试接口:
- SWD接口用于程序下载和调试
- RTT用于实时日志输出

可选外设 (用于硬件测试):
- UART: PA2 (TX), PA3 (RX)
- SPI: PA5 (SCK), PA6 (MISO), PA7 (MOSI)
- I2C: PB8 (SCL), PB9 (SDA)
- ADC: PA0 (ADC1_IN0)
```

## 软件架构

### 核心组件

#### 1. 测试框架核心 (`test_framework`)
```rust
pub struct TestRunner {
    pub suites: Vec<TestSuite, 16>,
    pub start_time: u32,
}

pub struct TestSuite {
    pub name: String<64>,
    pub results: Vec<TestResult, 32>,
    pub setup_time_us: u32,
    pub teardown_time_us: u32,
}

pub struct TestResult {
    pub name: String<64>,
    pub passed: bool,
    pub duration_us: u32,
    pub message: String<128>,
}
```

#### 2. 硬件抽象层测试 (`hal_testing`)
```rust
pub struct TestHardware {
    pub led: PA5<Output<PushPull>>,
    pub delay: Delay,
    pub timer: Timer<stm32::TIM2>,
}
```

#### 3. 模拟硬件 (`mock_hardware`)
```rust
pub struct MockGpio {
    pub expectations: Vec<GpioTransaction, 32>,
    pub current_state: State,
}
```

#### 4. 性能测试 (`performance_testing`)
```rust
pub struct PerformanceTest {
    pub name: String<64>,
    pub iterations: u32,
    pub min_duration_us: u32,
    pub max_duration_us: u32,
    pub avg_duration_us: u32,
}
```

#### 5. 内存测试 (`memory_testing`)
```rust
pub struct MemoryTest {
    pub initial_free: usize,
    pub current_free: usize,
    pub peak_usage: usize,
    pub allocations: u32,
    pub deallocations: u32,
}
```

### 测试执行流程

1. **初始化阶段**
   - 硬件外设初始化
   - 测试环境设置
   - 时钟和定时器配置

2. **测试执行阶段**
   - 按测试套件顺序执行
   - 记录执行时间和结果
   - 实时输出测试状态

3. **结果分析阶段**
   - 生成测试报告
   - 统计成功/失败率
   - 性能指标分析

4. **清理阶段**
   - 资源释放
   - 内存泄漏检查
   - 最终报告输出

## 构建和运行

### 环境准备
```bash
# 安装Rust工具链
rustup target add thumbv7em-none-eabihf

# 安装调试工具
cargo install probe-run
cargo install flip-link

# 安装defmt工具
cargo install defmt-print
```

### 编译项目
```bash
# 编译所有测试
cargo build --release

# 编译特定测试运行器
cargo build --bin unit_test_runner --release
cargo build --bin integration_test_runner --release
cargo build --bin hardware_test_runner --release
cargo build --bin performance_test_runner --release
```

### 运行测试
```bash
# 运行完整测试套件
cargo run --release

# 运行特定类型的测试
cargo run --bin unit_test_runner --release
cargo run --bin integration_test_runner --release
cargo run --bin hardware_test_runner --release
cargo run --bin performance_test_runner --release
```

### 功能特性测试
```bash
# 启用特定测试功能
cargo run --features "unit-testing,performance-testing" --release
cargo run --features "hardware-testing,mock-hardware" --release
cargo run --features "property-testing,fuzz-testing" --release
```

## 测试配置

### 基本配置
```toml
[package.metadata.test-framework]
hardware_required = true
test_timeout = 60
max_test_duration = 30
setup_script = "scripts/setup_test_environment.sh"
teardown_script = "scripts/cleanup_test_environment.sh"
```

### 测试分类
```toml
unit_tests = ["math", "data_structures", "algorithms"]
integration_tests = ["hal", "communication", "protocols"]
hardware_tests = ["gpio", "uart", "spi", "i2c", "adc", "timers"]
performance_tests = ["throughput", "latency", "memory", "power"]
```

### 报告配置
```toml
report_format = "json"
report_output = "test_results"
coverage_enabled = true
benchmark_enabled = true
```

## 使用示例

### 1. 创建单元测试
```rust
fn test_basic_math() -> bool {
    assert_test!(2 + 2 == 4, "Basic addition failed");
    assert_test!(10 - 5 == 5, "Basic subtraction failed");
    true
}

// 添加到测试套件
unit_test_suite.add_result(test_case!("Basic Math", test_basic_math));
```

### 2. 性能测试
```rust
let mut perf_test = PerformanceTest::new("Algorithm Performance", 1000);
let result = perf_test.run(|| {
    // 测试算法代码
    algorithm_under_test();
    true
});
```

### 3. 内存测试
```rust
let mut mem_test = MemoryTest::new();

// 执行可能产生内存泄漏的代码
allocate_and_use_memory();

mem_test.checkpoint();
let no_leaks = mem_test.verify_no_leaks();
```

### 4. 硬件测试
```rust
let mut hw_test = TestHardware::new(led, delay, timer);
let blink_result = hw_test.blink_test();
let timer_result = hw_test.timer_test();
```

## 扩展功能

### 自定义测试宏
```rust
#[macro_export]
macro_rules! assert_test {
    ($condition:expr, $message:expr) => {
        if !$condition {
            defmt::error!("Assertion failed: {}", $message);
            return false;
        }
    };
}

#[macro_export]
macro_rules! test_case {
    ($name:expr, $test_fn:expr) => {{
        let start_time = get_timestamp();
        let passed = $test_fn();
        let duration = get_timestamp() - start_time;
        
        TestResult {
            name: String::from($name),
            passed,
            duration_us: duration,
            message: String::new(),
        }
    }};
}
```

### 测试数据生成器
```rust
let mut generator = TestDataGenerator::new(12345);
let test_data = generator.generate_test_vector(100);
let random_value = generator.next_u32();
```

### 模拟硬件设置
```rust
let mut mock_gpio = MockGpio::new();
mock_gpio.expect_set_high();
mock_gpio.expect_set_low();

// 执行测试
run_gpio_test(&mut mock_gpio);

// 验证期望
assert!(mock_gpio.verify());
```

## 调试和分析

### 实时日志
```rust
defmt::info!("Test started: {}", test_name);
defmt::error!("Test failed: {}", error_message);
defmt::debug!("Debug info: {}", debug_data);
```

### 性能分析
- 执行时间测量 (微秒精度)
- 内存使用统计
- CPU使用率监控
- 中断延迟测量

### 测试报告
```
=== Test Report Summary ===
Total tests: 15
Passed: 13
Failed: 2
Success rate: 86%
Total duration: 1250 μs
Average test time: 83 μs
```

## 故障排除

### 常见问题

1. **编译错误**
   - 检查目标架构配置
   - 验证依赖版本兼容性
   - 确认feature flags正确

2. **硬件连接问题**
   - 验证引脚连接
   - 检查电源供应
   - 确认调试器连接

3. **测试失败**
   - 查看详细错误信息
   - 检查硬件状态
   - 验证测试逻辑

4. **性能问题**
   - 优化编译设置
   - 减少测试数据量
   - 检查内存使用

### 调试技巧

1. **使用RTT日志**
   ```bash
   # 实时查看日志输出
   defmt-print -e target/thumbv7em-none-eabihf/release/test_runner
   ```

2. **GDB调试**
   ```bash
   # 启动GDB调试会话
   arm-none-eabi-gdb target/thumbv7em-none-eabihf/release/test_runner
   ```

3. **性能分析**
   ```bash
   # 生成性能报告
   cargo run --features "benchmark" --release
   ```

## 开发指南

### 添加新测试
1. 在相应模块中实现测试函数
2. 返回bool表示测试结果
3. 使用assert_test!宏进行断言
4. 添加到测试套件中

### 扩展硬件支持
1. 在hal_testing模块中添加新硬件抽象
2. 实现相应的测试方法
3. 创建对应的模拟硬件类
4. 更新测试配置

### 性能优化
1. 使用release模式编译
2. 启用LTO优化
3. 调整测试数据大小
4. 优化测试算法

### 贡献指南
1. Fork项目仓库
2. 创建功能分支
3. 添加测试用例
4. 提交Pull Request

## 许可证

本项目采用MIT或Apache-2.0双重许可证。详见LICENSE文件。

## 联系方式

- 项目仓库: https://github.com/example/embedded-development
- 问题报告: https://github.com/example/embedded-development/issues
- 邮箱: dev@example.com
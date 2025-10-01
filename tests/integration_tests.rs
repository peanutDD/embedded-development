use std::path::Path;
use std::process::Command;
use std::time::Duration;
use tokio::time::timeout;

/// 测试结果
#[derive(Debug, Clone)]
pub struct TestResult {
  pub name: String,
  pub passed: bool,
  pub message: String,
  pub duration: Duration,
}

/// 测试套件
pub struct TestSuite {
  pub name: String,
  pub tests: Vec<TestResult>,
}

impl TestSuite {
  pub fn new(name: &str) -> Self {
    Self {
      name: name.to_string(),
      tests: Vec::new(),
    }
  }

  pub fn add_test(&mut self, result: TestResult) {
    self.tests.push(result);
  }

  pub fn passed_count(&self) -> usize {
    self.tests.iter().filter(|t| t.passed).count()
  }

  pub fn failed_count(&self) -> usize {
    self.tests.iter().filter(|t| !t.passed).count()
  }

  pub fn success_rate(&self) -> f64 {
    if self.tests.is_empty() {
      0.0
    } else {
      self.passed_count() as f64 / self.tests.len() as f64
    }
  }
}

/// 集成测试管理器
pub struct IntegrationTestManager {
  pub base_path: String,
  pub test_suites: Vec<TestSuite>,
}

impl IntegrationTestManager {
  pub fn new(base_path: &str) -> Self {
    Self {
      base_path: base_path.to_string(),
      test_suites: Vec::new(),
    }
  }

  /// 运行所有测试
  pub async fn run_all_tests(&mut self) -> Result<(), Box<dyn std::error::Error>> {
    println!("🚀 开始运行嵌入式开发教程集成测试...\n");

    // 基础篇测试 (第1-7章)
    self.test_basic_chapters().await?;

    // 进阶篇测试 (第8-14章)
    self.test_advanced_chapters().await?;

    // 专业开发篇测试 (第15-19章)
    self.test_professional_chapters().await?;

    // 生成测试报告
    self.generate_report();

    Ok(())
  }

  /// 测试基础篇章节
  async fn test_basic_chapters(&mut self) -> Result<(), Box<dyn std::error::Error>> {
    let mut suite = TestSuite::new("基础篇 (第1-7章)");

    // 第1章：Rust基础
    suite.add_test(self.test_rust_basics().await);

    // 第2章：嵌入式环境搭建
    suite.add_test(self.test_embedded_setup().await);

    // 第3章：硬件抽象层
    suite.add_test(self.test_hal_layer().await);

    // 第4章：GPIO控制
    suite.add_test(self.test_gpio_control().await);

    // 第5章：定时器与PWM
    suite.add_test(self.test_timer_pwm().await);

    // 第6章：串口通信
    suite.add_test(self.test_uart_communication().await);

    // 第7章：ADC与DAC
    suite.add_test(self.test_adc_dac().await);

    self.test_suites.push(suite);
    Ok(())
  }

  /// 测试进阶篇章节
  async fn test_advanced_chapters(&mut self) -> Result<(), Box<dyn std::error::Error>> {
    let mut suite = TestSuite::new("进阶篇 (第8-14章)");

    // 第8章：I2C通信
    suite.add_test(self.test_i2c_communication().await);

    // 第9章：SPI通信
    suite.add_test(self.test_spi_communication().await);

    // 第10章：文件系统
    suite.add_test(self.test_file_system().await);

    // 第11章：网络通信
    suite.add_test(self.test_network_communication().await);

    // 第12章：多任务处理
    suite.add_test(self.test_multitasking().await);

    // 第13章：中断处理
    suite.add_test(self.test_interrupt_handling().await);

    // 第14章：电源管理
    suite.add_test(self.test_power_management().await);

    self.test_suites.push(suite);
    Ok(())
  }

  /// 测试专业开发篇章节
  async fn test_professional_chapters(&mut self) -> Result<(), Box<dyn std::error::Error>> {
    let mut suite = TestSuite::new("专业开发篇 (第15-19章)");

    // 第15章：嵌入式操作系统
    suite.add_test(self.test_embedded_os().await);

    // 第16章：工业物联网
    suite.add_test(self.test_industrial_iot().await);

    // 第17章：无线通信
    suite.add_test(self.test_wireless_communication().await);

    // 第18章：机器学习与AI
    suite.add_test(self.test_machine_learning_ai().await);

    // 第19章：系统集成与部署
    suite.add_test(self.test_system_integration().await);

    self.test_suites.push(suite);
    Ok(())
  }

  /// 测试Rust基础
  async fn test_rust_basics(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "Rust基础语法和概念";

    // 检查基础示例文件是否存在
    let examples_path = format!("{}/01-rust-basics/examples", self.base_path);
    if !Path::new(&examples_path).exists() {
      return TestResult {
        name: name.to_string(),
        passed: false,
        message: "示例目录不存在".to_string(),
        duration: start.elapsed(),
      };
    }

    // 尝试编译基础示例
    match self
      .compile_rust_project(&format!("{}/01-rust-basics", self.base_path))
      .await
    {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "Rust基础示例编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试嵌入式环境搭建
  async fn test_embedded_setup(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "嵌入式开发环境";

    // 检查必要的工具链
    let tools = vec!["rustc", "cargo", "probe-run"];
    for tool in tools {
      if let Err(_) = Command::new(tool).arg("--version").output() {
        return TestResult {
          name: name.to_string(),
          passed: false,
          message: format!("工具 {} 未安装或不可用", tool),
          duration: start.elapsed(),
        };
      }
    }

    TestResult {
      name: name.to_string(),
      passed: true,
      message: "嵌入式开发环境配置正确".to_string(),
      duration: start.elapsed(),
    }
  }

  /// 测试硬件抽象层
  async fn test_hal_layer(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "硬件抽象层(HAL)";

    let hal_path = format!(
      "{}/03-embedded-concepts/projects/custom_hal",
      self.base_path
    );
    match self.compile_rust_project(&hal_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "HAL层编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("HAL编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试GPIO控制
  async fn test_gpio_control(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "GPIO控制";

    let gpio_path = format!("{}/04-gpio-control/projects/basic-led", self.base_path);
    match self.compile_rust_project(&gpio_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "GPIO控制项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("GPIO项目编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试定时器与PWM
  async fn test_timer_pwm(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "定时器与PWM";

    let timer_path = format!(
      "{}/06-timers-interrupts/projects/servo_control",
      self.base_path
    );
    match self.compile_rust_project(&timer_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "定时器PWM项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("定时器PWM编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试串口通信
  async fn test_uart_communication(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "串口通信";

    let uart_path = format!(
      "{}/05-serial-communication/projects/data-logger",
      self.base_path
    );
    match self.compile_rust_project(&uart_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "串口通信项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("串口通信编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试ADC与DAC
  async fn test_adc_dac(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "ADC与DAC";

    let adc_path = format!("{}/07-adc-dac/projects/voltmeter", self.base_path);
    match self.compile_rust_project(&adc_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "ADC/DAC项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("ADC/DAC编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试I2C通信
  async fn test_i2c_communication(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "I2C通信";

    let i2c_path = format!(
      "{}/08-communication-protocols-i2c-spi/projects/sensor_hub",
      self.base_path
    );
    match self.compile_rust_project(&i2c_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "I2C通信项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("I2C通信编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试SPI通信
  async fn test_spi_communication(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "SPI通信";

    let spi_path = format!(
      "{}/08-communication-protocols-i2c-spi/projects/display_controller",
      self.base_path
    );
    match self.compile_rust_project(&spi_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "SPI通信项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("SPI通信编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试文件系统
  async fn test_file_system(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "文件系统";

    let fs_path = format!("{}/09-rtos/projects/data_storage", self.base_path);
    match self.compile_rust_project(&fs_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "文件系统项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("文件系统编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试网络通信
  async fn test_network_communication(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "网络通信";

    let net_path = format!("{}/10-sensor-system/projects/web_server", self.base_path);
    match self.compile_rust_project(&net_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "网络通信项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("网络通信编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试多任务处理
  async fn test_multitasking(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "多任务处理";

    let task_path = format!(
      "{}/11-power-management/projects/task_scheduler",
      self.base_path
    );
    match self.compile_rust_project(&task_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "多任务处理项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("多任务处理编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试中断处理
  async fn test_interrupt_handling(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "中断处理";

    let int_path = format!(
      "{}/12-industrial-projects/projects/interrupt-manager",
      self.base_path
    );
    match self.compile_rust_project(&int_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "中断处理项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("中断处理编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试电源管理
  async fn test_power_management(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "电源管理";

    let power_path = format!(
      "{}/13-security-encryption/projects/low_power_sensor",
      self.base_path
    );
    match self.compile_rust_project(&power_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "电源管理项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("电源管理编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试嵌入式操作系统
  async fn test_embedded_os(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "嵌入式操作系统";

    let os_path = format!("{}/15-rtos-integration/projects/mini_rtos", self.base_path);
    match self.compile_rust_project(&os_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "嵌入式OS项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("嵌入式OS编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试工业物联网
  async fn test_industrial_iot(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "工业物联网";

    let iot_path = format!(
      "{}/16-performance-optimization/projects/industrial_gateway",
      self.base_path
    );
    match self.compile_rust_project(&iot_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "工业物联网项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("工业物联网编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试无线通信
  async fn test_wireless_communication(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "无线通信";

    let wireless_path = format!(
      "{}/14-wireless-communication/projects/mesh_network",
      self.base_path
    );
    match self.compile_rust_project(&wireless_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "无线通信项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("无线通信编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试机器学习与AI
  async fn test_machine_learning_ai(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "机器学习与AI";

    let ai_path = format!(
      "{}/17-embedded-ai/projects/smart-camera",
      self.base_path
    );
    match self.compile_rust_project(&ai_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "机器学习AI项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("机器学习AI编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 测试系统集成与部署
  async fn test_system_integration(&self) -> TestResult {
    let start = std::time::Instant::now();
    let name = "系统集成与部署";

    let integration_path = format!(
      "{}/19-system-integration/projects/iot-platform",
      self.base_path
    );
    match self.compile_rust_project(&integration_path).await {
      Ok(_) => TestResult {
        name: name.to_string(),
        passed: true,
        message: "系统集成项目编译成功".to_string(),
        duration: start.elapsed(),
      },
      Err(e) => TestResult {
        name: name.to_string(),
        passed: false,
        message: format!("系统集成编译失败: {}", e),
        duration: start.elapsed(),
      },
    }
  }

  /// 编译Rust项目
  async fn compile_rust_project(&self, project_path: &str) -> Result<(), String> {
    if !Path::new(project_path).exists() {
      return Err(format!("项目路径不存在: {}", project_path));
    }

    let cargo_toml = format!("{}/Cargo.toml", project_path);
    if !Path::new(&cargo_toml).exists() {
      return Err("Cargo.toml文件不存在".to_string());
    }

    // 使用超时机制避免编译卡死
    let compile_future = async {
      let output = Command::new("cargo")
        .arg("check")
        .arg("--manifest-path")
        .arg(&cargo_toml)
        .output()
        .map_err(|e| format!("执行cargo命令失败: {}", e))?;

      if output.status.success() {
        Ok(())
      } else {
        let stderr = String::from_utf8_lossy(&output.stderr);
        Err(format!("编译错误: {}", stderr))
      }
    };

    match timeout(Duration::from_secs(60), compile_future).await {
      Ok(result) => result,
      Err(_) => Err("编译超时".to_string()),
    }
  }

  /// 生成测试报告
  fn generate_report(&self) {
    println!("\n📊 测试报告");
    println!("{}", "=".repeat(80));

    let mut total_tests = 0;
    let mut total_passed = 0;
    let mut total_failed = 0;

    for suite in &self.test_suites {
      println!("\n📁 {}", suite.name);
      println!("{}", "-".repeat(40));

      for test in &suite.tests {
        let status = if test.passed {
          "✅ 通过"
        } else {
          "❌ 失败"
        };
        println!(
          "  {} {} ({:.2}s)",
          status,
          test.name,
          test.duration.as_secs_f64()
        );
        if !test.passed {
          println!("    💬 {}", test.message);
        }
      }

      let passed = suite.passed_count();
      let failed = suite.failed_count();
      let success_rate = suite.success_rate() * 100.0;

      println!(
        "  📈 成功率: {:.1}% ({}/{} 通过)",
        success_rate,
        passed,
        passed + failed
      );

      total_tests += suite.tests.len();
      total_passed += passed;
      total_failed += failed;
    }

    println!("\n🎯 总体统计");
    println!("{}", "=".repeat(80));
    println!("  总测试数: {}", total_tests);
    println!("  通过数量: {} ✅", total_passed);
    println!("  失败数量: {} ❌", total_failed);
    println!(
      "  总成功率: {:.1}%",
      (total_passed as f64 / total_tests as f64) * 100.0
    );

    if total_failed == 0 {
      println!("\n🎉 所有测试都通过了！嵌入式开发教程质量良好。");
    } else {
      println!(
        "\n⚠️  有 {} 个测试失败，请检查相关章节的代码。",
        total_failed
      );
    }
  }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
  let base_path = std::env::args()
    .nth(1)
    .unwrap_or_else(|| "/Users/tyone/github/embedded-development".to_string());

  let mut test_manager = IntegrationTestManager::new(&base_path);
  test_manager.run_all_tests().await?;

  Ok(())
}

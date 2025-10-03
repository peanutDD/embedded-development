//! 测试支持模块
//!
//! 提供跨平台的测试工具和模拟器

use crate::error::CrossPlatformError;
use crate::Result;
use heapless::{String, Vec};

/// 模拟GPIO引脚
pub struct MockGpio {
  state: bool,
  direction: GpioDirection,
}

/// GPIO方向
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GpioDirection {
  Input,
  Output,
}

impl MockGpio {
  /// 创建新的模拟GPIO
  pub fn new(direction: GpioDirection) -> Self {
    Self {
      state: false,
      direction,
    }
  }

  /// 设置引脚状态
  pub fn set_state(&mut self, state: bool) -> Result<()> {
    if self.direction != GpioDirection::Output {
      return Err(CrossPlatformError::InvalidOperation);
    }
    self.state = state;
    Ok(())
  }

  /// 读取引脚状态
  pub fn get_state(&self) -> bool {
    self.state
  }

  /// 设置引脚方向
  pub fn set_direction(&mut self, direction: GpioDirection) {
    self.direction = direction;
  }
}

/// 模拟传感器
pub struct MockSensor<T> {
  data: Vec<T, 32>,
  current_index: usize,
}

impl<T: Clone> MockSensor<T> {
  /// 创建新的模拟传感器
  pub fn new() -> Self {
    Self {
      data: Vec::new(),
      current_index: 0,
    }
  }

  /// 添加测试数据
  pub fn add_data(&mut self, value: T) -> Result<()> {
    self
      .data
      .push(value)
      .map_err(|_| CrossPlatformError::ResourceExhausted)?;
    Ok(())
  }

  /// 读取下一个数据
  pub fn read_next(&mut self) -> Result<T> {
    if self.current_index >= self.data.len() {
      return Err(CrossPlatformError::NoData);
    }

    let data = self.data[self.current_index].clone();
    self.current_index += 1;
    Ok(data)
  }

  /// 重置读取位置
  pub fn reset(&mut self) {
    self.current_index = 0;
  }
}

impl<T: Clone> Default for MockSensor<T> {
  fn default() -> Self {
    Self::new()
  }
}

/// 测试断言宏
#[macro_export]
macro_rules! assert_gpio_state {
  ($gpio:expr, $expected:expr) => {
    assert_eq!($gpio.get_state(), $expected, "GPIO state mismatch");
  };
}

/// 测试用例结构
pub struct TestCase {
  name: String<64>,
  passed: bool,
  error_message: Option<String<128>>,
}

impl TestCase {
  /// 创建新的测试用例
  pub fn new(name: &str) -> Result<Self> {
    Ok(Self {
      name: String::from(name),
      passed: false,
      error_message: None,
    })
  }

  /// 标记测试通过
  pub fn pass(&mut self) {
    self.passed = true;
    self.error_message = None;
  }

  /// 标记测试失败
  pub fn fail(&mut self, message: &str) {
    self.passed = false;
    self.error_message = Some(String::from(message));
  }

  /// 检查测试是否通过
  pub fn is_passed(&self) -> bool {
    self.passed
  }

  /// 获取测试名称
  pub fn name(&self) -> &str {
    &self.name
  }

  /// 获取错误消息
  pub fn error_message(&self) -> Option<&str> {
    self.error_message.as_ref().map(|s| s.as_str())
  }
}

/// 测试套件
pub struct TestSuite {
  cases: Vec<TestCase, 16>,
}

impl TestSuite {
  /// 创建新的测试套件
  pub fn new() -> Self {
    Self { cases: Vec::new() }
  }

  /// 添加测试用例
  pub fn add_case(&mut self, case: TestCase) -> Result<()> {
    self
      .cases
      .push(case)
      .map_err(|_| CrossPlatformError::ResourceExhausted)?;
    Ok(())
  }

  /// 运行所有测试
  pub fn run_all(&self) -> TestResult {
    let total = self.cases.len();
    let passed = self.cases.iter().filter(|c| c.is_passed()).count();

    TestResult {
      total,
      passed,
      failed: total - passed,
    }
  }

  /// 获取失败的测试
  pub fn get_failed_tests(&self) -> Vec<&TestCase, 16> {
    let mut failed = Vec::new();
    for case in &self.cases {
      if !case.is_passed() {
        let _ = failed.push(case);
      }
    }
    failed
  }
}

impl Default for TestSuite {
  fn default() -> Self {
    Self::new()
  }
}

/// 测试结果
#[derive(Debug, Clone, Copy)]
pub struct TestResult {
  pub total: usize,
  pub passed: usize,
  pub failed: usize,
}

impl TestResult {
  /// 检查所有测试是否通过
  pub fn all_passed(&self) -> bool {
    self.failed == 0
  }

  /// 获取通过率
  pub fn pass_rate(&self) -> f32 {
    if self.total == 0 {
      0.0
    } else {
      self.passed as f32 / self.total as f32
    }
  }
}

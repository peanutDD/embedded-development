//! 第3章：控制流练习
//!
//! 本文件包含Rust控制流的专门练习，包括：
//! - if/else 条件语句
//! - loop 无限循环
//! - while 条件循环
//! - for 迭代循环
//! - match 模式匹配
//! - 循环控制 (break, continue)

#![allow(dead_code, unused_variables)]

fn main() {
  println!("=== 第3章：控制流练习 ===");
  println!("运行 cargo test exercise_03 来测试你的答案");
  println!();

  // 示例运行
  println!("数字5的分类: {}", classify_number(5));
  println!("1到10的和: {}", sum_range(1, 10));
}

// ============================================================================
// 练习1: 条件语句基础
// ============================================================================

/// 练习1: 数字分类
///
/// 要求：
/// 1. 如果数字为0，返回"zero"
/// 2. 如果数字为正数，返回"positive"
/// 3. 如果数字为负数，返回"negative"
fn classify_number(n: i32) -> &'static str {
  if n == 0 {
    "zero"
  } else if n > 0 {
    "positive"
  } else {
    "negative"
  }
}

/// 练习2: 复杂条件判断
///
/// 判断一个年份是否为闰年
/// 规则：
/// - 能被4整除但不能被100整除，或者
/// - 能被400整除
fn is_leap_year(year: u32) -> bool {
  (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

/// 练习3: 条件表达式
///
/// 返回两个数中的最大值
fn max_of_two(a: i32, b: i32) -> i32 {
  if a > b {
    a
  } else {
    b
  }
}

// ============================================================================
// 练习4: 循环基础
// ============================================================================

/// 练习4: 使用loop计算阶乘
///
/// 要求：使用loop循环计算n的阶乘
fn factorial_loop(n: u32) -> u64 {
  if n == 0 {
    return 1;
  }

  let mut result = 1;
  let mut i = 1;

  loop {
    result *= i;
    i += 1;
    if i > n as u64 {
      break;
    }
  }

  result
}

/// 练习5: 使用while计算斐波那契数列
///
/// 返回第n个斐波那契数
fn fibonacci_while(n: u32) -> u64 {
  if n <= 1 {
    return n as u64;
  }

  let mut a = 0;
  let mut b = 1;
  let mut count = 2;

  while count <= n {
    let temp = a + b;
    a = b;
    b = temp;
    count += 1;
  }

  b
}

/// 练习6: 使用for计算范围内数字的和
///
/// 计算从start到end（包含）的所有数字的和
fn sum_range(start: i32, end: i32) -> i32 {
  let mut sum = 0;
  for i in start..=end {
    sum += i;
  }
  sum
}

// ============================================================================
// 练习7: 模式匹配基础
// ============================================================================

/// 练习7: 使用match处理枚举
///
/// 定义一个表示方向的枚举
#[derive(Debug, PartialEq)]
enum Direction {
  North,
  South,
  East,
  West,
}

impl Direction {
  /// 返回相反的方向
  fn opposite(&self) -> Direction {
    match self {
      Direction::North => Direction::South,
      Direction::South => Direction::North,
      Direction::East => Direction::West,
      Direction::West => Direction::East,
    }
  }

  /// 返回方向的角度（以北为0度，顺时针）
  fn to_degrees(&self) -> u32 {
    match self {
      Direction::North => 0,
      Direction::East => 90,
      Direction::South => 180,
      Direction::West => 270,
    }
  }
}

/// 练习8: 复杂的match模式
///
/// 根据数字返回不同的描述
fn describe_number(n: i32) -> String {
  match n {
    0 => "零".to_string(),
    1..=10 => "小数".to_string(),
    11..=100 => "中数".to_string(),
    101..=1000 => "大数".to_string(),
    _ if n < 0 => "负数".to_string(),
    _ => "超大数".to_string(),
  }
}

// ============================================================================
// 练习9: 循环控制
// ============================================================================

/// 练习9: 查找第一个满足条件的数字
///
/// 在给定范围内查找第一个能被7整除的数字
fn find_first_divisible_by_7(start: i32, end: i32) -> Option<i32> {
  for i in start..=end {
    if i % 7 == 0 {
      return Some(i);
    }
  }
  None
}

/// 练习10: 跳过特定条件
///
/// 计算1到n之间所有奇数的和
fn sum_odd_numbers(n: i32) -> i32 {
  let mut sum = 0;
  for i in 1..=n {
    if i % 2 == 0 {
      continue; // 跳过偶数
    }
    sum += i;
  }
  sum
}

// ============================================================================
// 练习11: 嵌套循环
// ============================================================================

/// 练习11: 打印乘法表
///
/// 生成n×n的乘法表，返回为字符串向量
fn multiplication_table(n: u32) -> Vec<String> {
  let mut table = Vec::new();

  for i in 1..=n {
    let mut row = String::new();
    for j in 1..=n {
      if j > 1 {
        row.push('\t');
      }
      row.push_str(&format!("{}", i * j));
    }
    table.push(row);
  }

  table
}

/// 练习12: 查找矩阵中的最大值
///
/// 在二维数组中查找最大值及其位置
fn find_max_in_matrix(matrix: &[Vec<i32>]) -> Option<(i32, usize, usize)> {
  if matrix.is_empty() || matrix[0].is_empty() {
    return None;
  }

  let mut max_val = matrix[0][0];
  let mut max_row = 0;
  let mut max_col = 0;

  for (i, row) in matrix.iter().enumerate() {
    for (j, &val) in row.iter().enumerate() {
      if val > max_val {
        max_val = val;
        max_row = i;
        max_col = j;
      }
    }
  }

  Some((max_val, max_row, max_col))
}

// ============================================================================
// 练习13: 高级模式匹配
// ============================================================================

/// 练习13: 处理Option类型
///
/// 安全地除法运算，返回Option<f64>
fn safe_divide(dividend: f64, divisor: f64) -> Option<f64> {
  if divisor == 0.0 {
    None
  } else {
    Some(dividend / divisor)
  }
}

/// 处理除法结果
fn handle_division_result(result: Option<f64>) -> String {
  match result {
    Some(value) if value.is_infinite() => "结果为无穷大".to_string(),
    Some(value) if value.is_nan() => "结果为NaN".to_string(),
    Some(value) => format!("结果为: {:.2}", value),
    None => "除数不能为零".to_string(),
  }
}

/// 练习14: 处理Result类型
///
/// 解析字符串为数字
fn parse_and_double(s: &str) -> Result<i32, String> {
  match s.parse::<i32>() {
    Ok(num) => Ok(num * 2),
    Err(_) => Err(format!("无法解析 '{}' 为数字", s)),
  }
}

// ============================================================================
// 练习15: 综合应用
// ============================================================================

/// 练习15: 实现一个简单的计算器
///
/// 支持基本的四则运算
#[derive(Debug, PartialEq)]
enum Operation {
  Add,
  Subtract,
  Multiply,
  Divide,
}

fn calculate(a: f64, op: Operation, b: f64) -> Result<f64, String> {
  match op {
    Operation::Add => Ok(a + b),
    Operation::Subtract => Ok(a - b),
    Operation::Multiply => Ok(a * b),
    Operation::Divide => {
      if b == 0.0 {
        Err("除数不能为零".to_string())
      } else {
        Ok(a / b)
      }
    }
  }
}

// ============================================================================
// 测试用例
// ============================================================================

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_classify_number() {
    assert_eq!(classify_number(0), "zero");
    assert_eq!(classify_number(5), "positive");
    assert_eq!(classify_number(-3), "negative");
  }

  #[test]
  fn test_is_leap_year() {
    assert!(is_leap_year(2000)); // 能被400整除
    assert!(is_leap_year(2004)); // 能被4整除但不能被100整除
    assert!(!is_leap_year(1900)); // 能被100整除但不能被400整除
    assert!(!is_leap_year(2001)); // 不能被4整除
  }

  #[test]
  fn test_max_of_two() {
    assert_eq!(max_of_two(5, 3), 5);
    assert_eq!(max_of_two(2, 8), 8);
    assert_eq!(max_of_two(4, 4), 4);
  }

  #[test]
  fn test_factorial_loop() {
    assert_eq!(factorial_loop(0), 1);
    assert_eq!(factorial_loop(1), 1);
    assert_eq!(factorial_loop(5), 120);
    assert_eq!(factorial_loop(6), 720);
  }

  #[test]
  fn test_fibonacci_while() {
    assert_eq!(fibonacci_while(0), 0);
    assert_eq!(fibonacci_while(1), 1);
    assert_eq!(fibonacci_while(2), 1);
    assert_eq!(fibonacci_while(3), 2);
    assert_eq!(fibonacci_while(10), 55);
  }

  #[test]
  fn test_sum_range() {
    assert_eq!(sum_range(1, 5), 15); // 1+2+3+4+5
    assert_eq!(sum_range(1, 1), 1);
    assert_eq!(sum_range(5, 5), 5);
  }

  #[test]
  fn test_direction() {
    assert_eq!(Direction::North.opposite(), Direction::South);
    assert_eq!(Direction::East.opposite(), Direction::West);

    assert_eq!(Direction::North.to_degrees(), 0);
    assert_eq!(Direction::East.to_degrees(), 90);
    assert_eq!(Direction::South.to_degrees(), 180);
    assert_eq!(Direction::West.to_degrees(), 270);
  }

  #[test]
  fn test_describe_number() {
    assert_eq!(describe_number(0), "零");
    assert_eq!(describe_number(5), "小数");
    assert_eq!(describe_number(50), "中数");
    assert_eq!(describe_number(500), "大数");
    assert_eq!(describe_number(-5), "负数");
    assert_eq!(describe_number(2000), "超大数");
  }

  #[test]
  fn test_find_first_divisible_by_7() {
    assert_eq!(find_first_divisible_by_7(1, 10), Some(7));
    assert_eq!(find_first_divisible_by_7(8, 13), None);
    assert_eq!(find_first_divisible_by_7(14, 20), Some(14));
  }

  #[test]
  fn test_sum_odd_numbers() {
    assert_eq!(sum_odd_numbers(5), 9); // 1+3+5
    assert_eq!(sum_odd_numbers(1), 1);
    assert_eq!(sum_odd_numbers(10), 25); // 1+3+5+7+9
  }

  #[test]
  fn test_multiplication_table() {
    let table = multiplication_table(3);
    assert_eq!(table.len(), 3);
    assert_eq!(table[0], "1\t2\t3");
    assert_eq!(table[1], "2\t4\t6");
    assert_eq!(table[2], "3\t6\t9");
  }

  #[test]
  fn test_find_max_in_matrix() {
    let matrix = vec![vec![1, 2, 3], vec![4, 5, 6], vec![7, 8, 9]];
    assert_eq!(find_max_in_matrix(&matrix), Some((9, 2, 2)));

    let empty_matrix: Vec<Vec<i32>> = vec![];
    assert_eq!(find_max_in_matrix(&empty_matrix), None);
  }

  #[test]
  fn test_safe_divide() {
    assert_eq!(safe_divide(10.0, 2.0), Some(5.0));
    assert_eq!(safe_divide(10.0, 0.0), None);
  }

  #[test]
  fn test_handle_division_result() {
    assert_eq!(handle_division_result(Some(5.0)), "结果为: 5.00");
    assert_eq!(handle_division_result(None), "除数不能为零");
  }

  #[test]
  fn test_parse_and_double() {
    assert_eq!(parse_and_double("5"), Ok(10));
    assert_eq!(
      parse_and_double("abc"),
      Err("无法解析 'abc' 为数字".to_string())
    );
  }

  #[test]
  fn test_calculate() {
    assert_eq!(calculate(5.0, Operation::Add, 3.0), Ok(8.0));
    assert_eq!(calculate(5.0, Operation::Subtract, 3.0), Ok(2.0));
    assert_eq!(calculate(5.0, Operation::Multiply, 3.0), Ok(15.0));
    assert_eq!(calculate(6.0, Operation::Divide, 2.0), Ok(3.0));
    assert!(calculate(5.0, Operation::Divide, 0.0).is_err());
  }
}

// ============================================================================
// 挑战练习
// ============================================================================

/// 挑战1: 实现一个状态机
///
/// 模拟一个简单的交通灯状态机
#[derive(Debug, PartialEq, Clone)]
enum TrafficLight {
  Red,
  Yellow,
  Green,
}

impl TrafficLight {
  fn next(&self) -> TrafficLight {
    match self {
      TrafficLight::Red => TrafficLight::Green,
      TrafficLight::Green => TrafficLight::Yellow,
      TrafficLight::Yellow => TrafficLight::Red,
    }
  }

  fn duration_seconds(&self) -> u32 {
    match self {
      TrafficLight::Red => 30,
      TrafficLight::Yellow => 5,
      TrafficLight::Green => 25,
    }
  }
}

/// 挑战2: 实现一个简单的解释器
///
/// 解释简单的数学表达式（只支持加法和减法）
fn simple_calculator(expression: &str) -> Result<i32, String> {
  let expression = expression.replace(" ", "");

  if expression.is_empty() {
    return Err("表达式不能为空".to_string());
  }

  let mut result = 0;
  let mut current_number = String::new();
  let mut operation = '+';

  for ch in expression.chars() {
    match ch {
      '0'..='9' => {
        current_number.push(ch);
      }
      '+' | '-' => {
        if current_number.is_empty() {
          return Err("无效的表达式".to_string());
        }

        let num: i32 = current_number
          .parse()
          .map_err(|_| "无效的数字".to_string())?;

        match operation {
          '+' => result += num,
          '-' => result -= num,
          _ => return Err("不支持的操作".to_string()),
        }

        current_number.clear();
        operation = ch;
      }
      _ => return Err(format!("不支持的字符: {}", ch)),
    }
  }

  // 处理最后一个数字
  if !current_number.is_empty() {
    let num: i32 = current_number
      .parse()
      .map_err(|_| "无效的数字".to_string())?;

    match operation {
      '+' => result += num,
      '-' => result -= num,
      _ => return Err("不支持的操作".to_string()),
    }
  }

  Ok(result)
}

#[cfg(test)]
mod challenge_tests {
  use super::*;

  #[test]
  fn test_traffic_light() {
    let mut light = TrafficLight::Red;
    assert_eq!(light.duration_seconds(), 30);

    light = light.next();
    assert_eq!(light, TrafficLight::Green);
    assert_eq!(light.duration_seconds(), 25);

    light = light.next();
    assert_eq!(light, TrafficLight::Yellow);
    assert_eq!(light.duration_seconds(), 5);

    light = light.next();
    assert_eq!(light, TrafficLight::Red);
  }

  #[test]
  fn test_simple_calculator() {
    assert_eq!(simple_calculator("1+2+3"), Ok(6));
    assert_eq!(simple_calculator("10-5+3"), Ok(8));
    assert_eq!(simple_calculator("100-50-25"), Ok(25));
    assert_eq!(simple_calculator("5"), Ok(5));

    assert!(simple_calculator("").is_err());
    assert!(simple_calculator("1+").is_err());
    assert!(simple_calculator("1*2").is_err());
  }
}

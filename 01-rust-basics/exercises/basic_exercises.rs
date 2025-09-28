// Rust基础练习题
// 涵盖变量、数据类型、控制流、函数等基础概念
// 每个练习都有详细说明和测试用例

#![allow(dead_code, unused_variables)]

fn main() {
  println!("=== Rust 基础练习题 ===");
  println!("请完成以下练习，运行测试验证你的答案");
  println!("使用 cargo test 运行所有测试");
  println!();

  // 运行一些示例
  println!("示例运行:");
  println!("exercise_1_result: {:?}", exercise_1());
  println!("exercise_2_result: {:?}", exercise_2());
  println!("exercise_3_result: {}", exercise_3(5));
}

// ============================================================================
// 练习1: 变量和数据类型 (难度: 初级)
// ============================================================================

/// 练习1: 创建变量并进行基本运算
///
/// 要求:
/// 1. 创建两个整数变量 a = 10, b = 20
/// 2. 计算它们的和、差、积、商
/// 3. 返回所有结果的元组 (sum, diff, product, quotient)
///
/// 提示: 注意整数除法的结果类型
fn exercise_1() -> (i32, i32, i32, i32) {
  // TODO: 在这里实现你的代码
  let a = 10;
  let b = 20;

  let sum = a + b;
  let diff = a - b;
  let product = a * b;
  let quotient = b / a;

  (sum, diff, product, quotient)
}

/// 练习2: 字符串操作
///
/// 要求:
/// 1. 创建一个字符串变量包含你的名字
/// 2. 创建另一个字符串包含问候语 "Hello, "
/// 3. 将它们连接起来
/// 4. 返回连接后的字符串和原字符串的长度
fn exercise_2() -> (String, usize) {
  // TODO: 在这里实现你的代码
  let name = String::from("Rust学习者");
  let greeting = String::from("Hello, ");

  let full_greeting = format!("{}{}", greeting, name);
  let length = full_greeting.len();

  (full_greeting, length)
}

// ============================================================================
// 练习3: 控制流 (难度: 初级)
// ============================================================================

/// 练习3: 条件判断
///
/// 要求:
/// 实现一个函数，根据输入的数字返回不同的字符串:
/// - 如果数字是偶数且大于10，返回 "Large Even"
/// - 如果数字是偶数且小于等于10，返回 "Small Even"
/// - 如果数字是奇数且大于10，返回 "Large Odd"
/// - 如果数字是奇数且小于等于10，返回 "Small Odd"
fn exercise_3(num: i32) -> &'static str {
  // TODO: 在这里实现你的代码
  match (num % 2 == 0, num > 10) {
    (true, true) => "Large Even",
    (true, false) => "Small Even",
    (false, true) => "Large Odd",
    (false, false) => "Small Odd",
  }
}

/// 练习4: 循环
///
/// 要求:
/// 计算1到n的所有数字的和
/// 使用for循环实现
fn exercise_4(n: i32) -> i32 {
  // TODO: 在这里实现你的代码
  let mut sum = 0;
  for i in 1..=n {
    sum += i;
  }
  sum
}

/// 练习5: while循环
///
/// 要求:
/// 找到第一个大于给定数字的2的幂
/// 例如: 输入10，返回16 (因为16是第一个大于10的2的幂)
fn exercise_5(num: i32) -> i32 {
  // TODO: 在这里实现你的代码
  let mut power = 1;
  while power <= num {
    power *= 2;
  }
  power
}

// ============================================================================
// 练习6-10: 函数和所有权 (难度: 中级)
// ============================================================================

/// 练习6: 函数参数
///
/// 要求:
/// 实现一个函数，接受两个参数并返回较大的那个
/// 使用泛型，使其可以比较任何可比较的类型
fn exercise_6<T: PartialOrd>(a: T, b: T) -> T {
  // TODO: 在这里实现你的代码
  if a > b {
    a
  } else {
    b
  }
}

/// 练习7: 借用和引用
///
/// 要求:
/// 实现一个函数，计算字符串中单词的数量
/// 函数不应该获取字符串的所有权
fn exercise_7(s: &str) -> usize {
  // TODO: 在这里实现你的代码
  if s.trim().is_empty() {
    0
  } else {
    s.split_whitespace().count()
  }
}

/// 练习8: 可变引用
///
/// 要求:
/// 实现一个函数，将向量中的所有元素都乘以2
/// 函数应该修改原向量，而不是创建新向量
fn exercise_8(vec: &mut Vec<i32>) {
  // TODO: 在这里实现你的代码
  for item in vec.iter_mut() {
    *item *= 2;
  }
}

/// 练习9: 所有权转移
///
/// 要求:
/// 实现一个函数，接受一个字符串，在其前后添加括号，然后返回新字符串
/// 函数应该获取字符串的所有权
fn exercise_9(s: String) -> String {
  // TODO: 在这里实现你的代码
  format!("({})", s)
}

/// 练习10: 生命周期
///
/// 要求:
/// 实现一个函数，返回两个字符串切片中较长的那个
/// 注意生命周期参数
fn exercise_10<'a>(s1: &'a str, s2: &'a str) -> &'a str {
  // TODO: 在这里实现你的代码
  if s1.len() > s2.len() {
    s1
  } else {
    s2
  }
}

// ============================================================================
// 练习11-15: 结构体和枚举 (难度: 中级)
// ============================================================================

/// 练习11: 定义结构体
///
/// 要求:
/// 1. 定义一个Person结构体，包含name(String)和age(u32)字段
/// 2. 实现一个new函数创建Person实例
/// 3. 实现一个greet方法返回问候语
#[derive(Debug, PartialEq)]
struct Person {
  // TODO: 定义字段
  name: String,
  age: u32,
}

impl Person {
  // TODO: 实现new函数
  fn new(name: String, age: u32) -> Self {
    Person { name, age }
  }

  // TODO: 实现greet方法
  fn greet(&self) -> String {
    format!(
      "Hello, my name is {} and I am {} years old.",
      self.name, self.age
    )
  }
}

/// 练习12: 枚举定义
///
/// 要求:
/// 定义一个Color枚举，包含Red, Green, Blue三个变体
/// 实现一个方法返回颜色的RGB值
#[derive(Debug, PartialEq)]
enum Color {
  // TODO: 定义枚举变体
  Red,
  Green,
  Blue,
}

impl Color {
  // TODO: 实现rgb方法，返回(u8, u8, u8)
  fn rgb(&self) -> (u8, u8, u8) {
    match self {
      Color::Red => (255, 0, 0),
      Color::Green => (0, 255, 0),
      Color::Blue => (0, 0, 255),
    }
  }
}

/// 练习13: 复杂枚举
///
/// 要求:
/// 定义一个Shape枚举，包含:
/// - Circle { radius: f64 }
/// - Rectangle { width: f64, height: f64 }
/// - Triangle { base: f64, height: f64 }
/// 实现计算面积的方法
#[derive(Debug)]
enum Shape {
  // TODO: 定义枚举变体
  Circle { radius: f64 },
  Rectangle { width: f64, height: f64 },
  Triangle { base: f64, height: f64 },
}

impl Shape {
  // TODO: 实现area方法
  fn area(&self) -> f64 {
    match self {
      Shape::Circle { radius } => std::f64::consts::PI * radius * radius,
      Shape::Rectangle { width, height } => width * height,
      Shape::Triangle { base, height } => 0.5 * base * height,
    }
  }
}

/// 练习14: Option处理
///
/// 要求:
/// 实现一个函数，在向量中查找指定元素的索引
/// 如果找到返回Some(index)，否则返回None
fn exercise_14<T: PartialEq>(vec: &[T], target: &T) -> Option<usize> {
  // TODO: 在这里实现你的代码
  for (index, item) in vec.iter().enumerate() {
    if item == target {
      return Some(index);
    }
  }
  None
}

/// 练习15: Result处理
///
/// 要求:
/// 实现一个安全的除法函数
/// 如果除数为0，返回错误信息
/// 否则返回计算结果
fn exercise_15(dividend: f64, divisor: f64) -> Result<f64, String> {
  // TODO: 在这里实现你的代码
  if divisor == 0.0 {
    Err("Division by zero".to_string())
  } else {
    Ok(dividend / divisor)
  }
}

// ============================================================================
// 练习16-20: 集合和迭代器 (难度: 中高级)
// ============================================================================

/// 练习16: Vector操作
///
/// 要求:
/// 实现一个函数，接受一个整数向量，返回一个新向量，
/// 包含原向量中所有偶数的平方
fn exercise_16(numbers: Vec<i32>) -> Vec<i32> {
  // TODO: 在这里实现你的代码
  numbers
    .into_iter()
    .filter(|&x| x % 2 == 0)
    .map(|x| x * x)
    .collect()
}

/// 练习17: HashMap操作
///
/// 要求:
/// 实现一个函数，统计字符串中每个字符出现的次数
/// 返回HashMap<char, usize>
fn exercise_17(s: &str) -> std::collections::HashMap<char, usize> {
  // TODO: 在这里实现你的代码
  use std::collections::HashMap;
  let mut char_count = HashMap::new();

  for ch in s.chars() {
    *char_count.entry(ch).or_insert(0) += 1;
  }

  char_count
}

/// 练习18: 迭代器链式操作
///
/// 要求:
/// 给定一个字符串向量，返回所有长度大于3的字符串，
/// 转换为大写，并按字母顺序排序
fn exercise_18(strings: Vec<String>) -> Vec<String> {
  // TODO: 在这里实现你的代码
  let mut result: Vec<String> = strings
    .into_iter()
    .filter(|s| s.len() > 3)
    .map(|s| s.to_uppercase())
    .collect();

  result.sort();
  result
}

/// 练习19: 自定义迭代器
///
/// 要求:
/// 实现一个斐波那契数列迭代器
/// 可以生成前n个斐波那契数
struct Fibonacci {
  // TODO: 定义必要的字段
  current: u64,
  next: u64,
  count: usize,
  max_count: usize,
}

impl Fibonacci {
  fn new(max_count: usize) -> Self {
    // TODO: 实现构造函数
    Fibonacci {
      current: 0,
      next: 1,
      count: 0,
      max_count,
    }
  }
}

impl Iterator for Fibonacci {
  type Item = u64;

  fn next(&mut self) -> Option<Self::Item> {
    // TODO: 实现迭代器逻辑
    if self.count >= self.max_count {
      return None;
    }

    let current = self.current;
    self.current = self.next;
    self.next = current + self.next;
    self.count += 1;

    Some(current)
  }
}

/// 练习20: 复杂数据处理
///
/// 要求:
/// 给定一个学生成绩的向量 Vec<(String, u32)>，
/// 返回平均分大于等于80的学生名单，按成绩降序排列
fn exercise_20(grades: Vec<(String, u32)>) -> Vec<String> {
  // TODO: 在这里实现你的代码
  let mut high_achievers: Vec<(String, u32)> = grades
    .into_iter()
    .filter(|(_, grade)| *grade >= 80)
    .collect();

  high_achievers.sort_by(|a, b| b.1.cmp(&a.1));

  high_achievers.into_iter().map(|(name, _)| name).collect()
}

// ============================================================================
// 测试用例
// ============================================================================

#[cfg(test)]
mod tests {
  use super::*;
  use std::collections::HashMap;

  #[test]
  fn test_exercise_1() {
    assert_eq!(exercise_1(), (30, -10, 200, 2));
  }

  #[test]
  fn test_exercise_2() {
    let (greeting, length) = exercise_2();
    assert!(greeting.contains("Hello, "));
    assert!(greeting.contains("Rust学习者"));
    assert_eq!(length, greeting.len());
  }

  #[test]
  fn test_exercise_3() {
    assert_eq!(exercise_3(5), "Small Odd");
    assert_eq!(exercise_3(8), "Small Even");
    assert_eq!(exercise_3(15), "Large Odd");
    assert_eq!(exercise_3(20), "Large Even");
  }

  #[test]
  fn test_exercise_4() {
    assert_eq!(exercise_4(5), 15); // 1+2+3+4+5 = 15
    assert_eq!(exercise_4(10), 55); // 1+2+...+10 = 55
  }

  #[test]
  fn test_exercise_5() {
    assert_eq!(exercise_5(10), 16);
    assert_eq!(exercise_5(7), 8);
    assert_eq!(exercise_5(16), 32);
  }

  #[test]
  fn test_exercise_6() {
    assert_eq!(exercise_6(5, 10), 10);
    assert_eq!(exercise_6("hello", "world"), "world");
  }

  #[test]
  fn test_exercise_7() {
    assert_eq!(exercise_7("hello world"), 2);
    assert_eq!(exercise_7(""), 0);
    assert_eq!(exercise_7("   "), 0);
    assert_eq!(exercise_7("single"), 1);
  }

  #[test]
  fn test_exercise_8() {
    let mut vec = vec![1, 2, 3, 4];
    exercise_8(&mut vec);
    assert_eq!(vec, vec![2, 4, 6, 8]);
  }

  #[test]
  fn test_exercise_9() {
    let s = String::from("hello");
    assert_eq!(exercise_9(s), "(hello)");
  }

  #[test]
  fn test_exercise_10() {
    assert_eq!(exercise_10("short", "longer"), "longer");
    assert_eq!(exercise_10("same", "size"), "same"); // 相等时返回第一个
  }

  #[test]
  fn test_person() {
    let person = Person::new("Alice".to_string(), 25);
    assert_eq!(person.name, "Alice");
    assert_eq!(person.age, 25);
    assert!(person.greet().contains("Alice"));
    assert!(person.greet().contains("25"));
  }

  #[test]
  fn test_color() {
    assert_eq!(Color::Red.rgb(), (255, 0, 0));
    assert_eq!(Color::Green.rgb(), (0, 255, 0));
    assert_eq!(Color::Blue.rgb(), (0, 0, 255));
  }

  #[test]
  fn test_shape() {
    let circle = Shape::Circle { radius: 2.0 };
    let rectangle = Shape::Rectangle {
      width: 3.0,
      height: 4.0,
    };
    let triangle = Shape::Triangle {
      base: 6.0,
      height: 8.0,
    };

    assert!((circle.area() - (std::f64::consts::PI * 4.0)).abs() < 0.001);
    assert_eq!(rectangle.area(), 12.0);
    assert_eq!(triangle.area(), 24.0);
  }

  #[test]
  fn test_exercise_14() {
    let vec = vec![1, 2, 3, 4, 5];
    assert_eq!(exercise_14(&vec, &3), Some(2));
    assert_eq!(exercise_14(&vec, &10), None);
  }

  #[test]
  fn test_exercise_15() {
    assert_eq!(exercise_15(10.0, 2.0), Ok(5.0));
    assert!(exercise_15(10.0, 0.0).is_err());
  }

  #[test]
  fn test_exercise_16() {
    let numbers = vec![1, 2, 3, 4, 5, 6];
    let result = exercise_16(numbers);
    assert_eq!(result, vec![4, 16, 36]); // 2^2, 4^2, 6^2
  }

  #[test]
  fn test_exercise_17() {
    let result = exercise_17("hello");
    let mut expected = HashMap::new();
    expected.insert('h', 1);
    expected.insert('e', 1);
    expected.insert('l', 2);
    expected.insert('o', 1);
    assert_eq!(result, expected);
  }

  #[test]
  fn test_exercise_18() {
    let strings = vec![
      "hi".to_string(),
      "hello".to_string(),
      "world".to_string(),
      "rust".to_string(),
    ];
    let result = exercise_18(strings);
    assert_eq!(result, vec!["HELLO", "RUST", "WORLD"]);
  }

  #[test]
  fn test_fibonacci() {
    let fib: Vec<u64> = Fibonacci::new(6).collect();
    assert_eq!(fib, vec![0, 1, 1, 2, 3, 5]);
  }

  #[test]
  fn test_exercise_20() {
    let grades = vec![
      ("Alice".to_string(), 95),
      ("Bob".to_string(), 75),
      ("Charlie".to_string(), 85),
      ("David".to_string(), 90),
    ];
    let result = exercise_20(grades);
    assert_eq!(result, vec!["Alice", "David", "Charlie"]);
  }
}

// ============================================================================
// 挑战练习 (难度: 高级)
// ============================================================================

/// 挑战练习1: 实现一个简单的计算器
///
/// 要求:
/// 实现一个计算器，支持基本的四则运算
/// 输入格式: "数字 操作符 数字"
/// 例如: "10 + 5", "20 * 3"
fn challenge_calculator(expression: &str) -> Result<f64, String> {
  // TODO: 实现计算器逻辑
  let parts: Vec<&str> = expression.split_whitespace().collect();

  if parts.len() != 3 {
    return Err("Invalid expression format".to_string());
  }

  let left: f64 = parts[0]
    .parse()
    .map_err(|_| "Invalid left operand".to_string())?;
  let operator = parts[1];
  let right: f64 = parts[2]
    .parse()
    .map_err(|_| "Invalid right operand".to_string())?;

  match operator {
    "+" => Ok(left + right),
    "-" => Ok(left - right),
    "*" => Ok(left * right),
    "/" => {
      if right == 0.0 {
        Err("Division by zero".to_string())
      } else {
        Ok(left / right)
      }
    }
    _ => Err(format!("Unsupported operator: {}", operator)),
  }
}

/// 挑战练习2: 实现一个通用的排序函数
///
/// 要求:
/// 实现一个泛型排序函数，可以对任何可比较的类型进行排序
/// 支持升序和降序
fn challenge_sort<T: Ord + Clone>(mut vec: Vec<T>, ascending: bool) -> Vec<T> {
  // TODO: 实现排序逻辑
  if ascending {
    vec.sort();
  } else {
    vec.sort_by(|a, b| b.cmp(a));
  }
  vec
}

#[cfg(test)]
mod challenge_tests {
  use super::*;

  #[test]
  fn test_challenge_calculator() {
    assert_eq!(challenge_calculator("10 + 5"), Ok(15.0));
    assert_eq!(challenge_calculator("20 * 3"), Ok(60.0));
    assert_eq!(challenge_calculator("15 / 3"), Ok(5.0));
    assert!(challenge_calculator("10 / 0").is_err());
    assert!(challenge_calculator("invalid").is_err());
  }

  #[test]
  fn test_challenge_sort() {
    let numbers = vec![3, 1, 4, 1, 5, 9, 2, 6];
    assert_eq!(
      challenge_sort(numbers.clone(), true),
      vec![1, 1, 2, 3, 4, 5, 6, 9]
    );
    assert_eq!(challenge_sort(numbers, false), vec![9, 6, 5, 4, 3, 2, 1, 1]);
  }
}

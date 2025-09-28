// 错误处理演示程序
// 展示Rust的Result、Option、panic!、自定义错误等错误处理机制

use std::error::Error;
use std::fmt;
use std::fs::File;
use std::io::{self, Read, Write};
use std::num::ParseIntError;

fn main() {
  println!("=== Rust 错误处理演示 ===\n");

  // 1. Result基础
  result_basics_demo();

  // 2. Option处理
  option_handling_demo();

  // 3. 错误传播
  error_propagation_demo();

  // 4. 自定义错误
  custom_error_demo();

  // 5. 错误链和转换
  error_conversion_demo();

  // 6. 最佳实践
  best_practices_demo();
}

fn result_basics_demo() {
  println!("1. Result基础演示:");

  // 基本的Result使用
  match divide(10.0, 2.0) {
    Ok(result) => println!("  10 ÷ 2 = {}", result),
    Err(error) => println!("  错误: {}", error),
  }

  match divide(10.0, 0.0) {
    Ok(result) => println!("  10 ÷ 0 = {}", result),
    Err(error) => println!("  错误: {}", error),
  }

  // 使用unwrap和expect
  let safe_result = divide(8.0, 2.0).unwrap(); // 确定不会出错时使用
  println!("  安全除法结果: {}", safe_result);

  // expect提供更好的错误信息
  let expected_result = divide(15.0, 3.0).expect("这个除法应该是安全的");
  println!("  期望结果: {}", expected_result);

  // 使用unwrap_or提供默认值
  let default_result = divide(10.0, 0.0).unwrap_or(0.0);
  println!("  带默认值的结果: {}", default_result);

  // 使用unwrap_or_else提供计算默认值的闭包
  let computed_default = divide(10.0, 0.0).unwrap_or_else(|_| {
    println!("    计算默认值...");
    -1.0
  });
  println!("  计算的默认值: {}", computed_default);

  // 链式操作
  let chain_result = divide(20.0, 4.0)
    .and_then(|x| divide(x, 2.0))
    .and_then(|x| divide(x, 1.0));

  match chain_result {
    Ok(result) => println!("  链式计算结果: {}", result),
    Err(error) => println!("  链式计算错误: {}", error),
  }

  println!();
}

fn divide(a: f64, b: f64) -> Result<f64, String> {
  if b == 0.0 {
    Err("除数不能为零".to_string())
  } else {
    Ok(a / b)
  }
}

fn option_handling_demo() {
  println!("2. Option处理演示:");

  let numbers = vec![1, 2, 3, 4, 5];

  // 安全的索引访问
  match numbers.get(2) {
    Some(value) => println!("  索引2的值: {}", value),
    None => println!("  索引2不存在"),
  }

  match numbers.get(10) {
    Some(value) => println!("  索引10的值: {}", value),
    None => println!("  索引10不存在"),
  }

  // Option的方法
  let maybe_number = Some(42);

  // map: 转换Some中的值
  let doubled = maybe_number.map(|x| x * 2);
  println!("  42翻倍: {:?}", doubled);

  // filter: 过滤值
  let filtered = maybe_number.filter(|&x| x > 50);
  println!("  过滤大于50: {:?}", filtered);

  // and_then: 链式操作
  let chained = maybe_number
    .and_then(|x| if x > 40 { Some(x) } else { None })
    .and_then(|x| Some(x + 10));
  println!("  链式操作结果: {:?}", chained);

  // 字符串解析示例
  let input = "42";
  match parse_number(input) {
    Some(num) => println!("  解析数字: {}", num),
    None => println!("  解析失败"),
  }

  let invalid_input = "abc";
  match parse_number(invalid_input) {
    Some(num) => println!("  解析数字: {}", num),
    None => println!("  解析失败: {}", invalid_input),
  }

  // Option和Result的转换
  let option_to_result = maybe_number.ok_or("没有值");
  println!("  Option转Result: {:?}", option_to_result);

  let result_to_option = divide(10.0, 2.0).ok();
  println!("  Result转Option: {:?}", result_to_option);

  println!();
}

fn parse_number(s: &str) -> Option<i32> {
  s.parse().ok()
}

fn error_propagation_demo() {
  println!("3. 错误传播演示:");

  // 使用?操作符
  match read_and_parse_file("test.txt") {
    Ok(number) => println!("  文件中的数字: {}", number),
    Err(error) => println!("  读取文件错误: {}", error),
  }

  // 手动错误传播
  match read_and_parse_manual("test.txt") {
    Ok(number) => println!("  手动传播结果: {}", number),
    Err(error) => println!("  手动传播错误: {}", error),
  }

  // 多层错误传播
  match complex_operation() {
    Ok(result) => println!("  复杂操作结果: {}", result),
    Err(error) => println!("  复杂操作错误: {}", error),
  }

  println!();
}

// 使用?操作符的函数
fn read_and_parse_file(filename: &str) -> Result<i32, Box<dyn Error>> {
  let mut file = File::open(filename)?;
  let mut contents = String::new();
  file.read_to_string(&mut contents)?;
  let number: i32 = contents.trim().parse()?;
  Ok(number)
}

// 手动错误传播
fn read_and_parse_manual(filename: &str) -> Result<i32, Box<dyn Error>> {
  let mut file = match File::open(filename) {
    Ok(file) => file,
    Err(error) => return Err(Box::new(error)),
  };

  let mut contents = String::new();
  match file.read_to_string(&mut contents) {
    Ok(_) => {}
    Err(error) => return Err(Box::new(error)),
  }

  let number = match contents.trim().parse::<i32>() {
    Ok(number) => number,
    Err(error) => return Err(Box::new(error)),
  };

  Ok(number)
}

fn complex_operation() -> Result<String, Box<dyn Error>> {
  let result1 = step1()?;
  let result2 = step2(result1)?;
  let result3 = step3(result2)?;
  Ok(result3)
}

fn step1() -> Result<i32, &'static str> {
  Ok(42)
}

fn step2(input: i32) -> Result<f64, &'static str> {
  if input > 0 {
    Ok(input as f64 * 2.5)
  } else {
    Err("输入必须为正数")
  }
}

fn step3(input: f64) -> Result<String, &'static str> {
  Ok(format!("最终结果: {:.2}", input))
}

// 自定义错误类型
#[derive(Debug)]
enum MathError {
  DivisionByZero,
  NegativeSquareRoot,
  Overflow,
  InvalidInput(String),
}

impl fmt::Display for MathError {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    match self {
      MathError::DivisionByZero => write!(f, "除数不能为零"),
      MathError::NegativeSquareRoot => write!(f, "负数不能开平方根"),
      MathError::Overflow => write!(f, "数值溢出"),
      MathError::InvalidInput(msg) => write!(f, "无效输入: {}", msg),
    }
  }
}

impl Error for MathError {}

fn custom_error_demo() {
  println!("4. 自定义错误演示:");

  // 安全除法
  match safe_divide(10.0, 2.0) {
    Ok(result) => println!("  安全除法: 10 ÷ 2 = {}", result),
    Err(error) => println!("  错误: {}", error),
  }

  match safe_divide(10.0, 0.0) {
    Ok(result) => println!("  安全除法: 10 ÷ 0 = {}", result),
    Err(error) => println!("  错误: {}", error),
  }

  // 安全开方
  match safe_sqrt(16.0) {
    Ok(result) => println!("  安全开方: √16 = {}", result),
    Err(error) => println!("  错误: {}", error),
  }

  match safe_sqrt(-4.0) {
    Ok(result) => println!("  安全开方: √(-4) = {}", result),
    Err(error) => println!("  错误: {}", error),
  }

  // 计算器示例
  match calculator("10 + 5") {
    Ok(result) => println!("  计算器: 10 + 5 = {}", result),
    Err(error) => println!("  计算器错误: {}", error),
  }

  match calculator("10 / 0") {
    Ok(result) => println!("  计算器: 10 / 0 = {}", result),
    Err(error) => println!("  计算器错误: {}", error),
  }

  match calculator("invalid") {
    Ok(result) => println!("  计算器: invalid = {}", result),
    Err(error) => println!("  计算器错误: {}", error),
  }

  println!();
}

fn safe_divide(a: f64, b: f64) -> Result<f64, MathError> {
  if b == 0.0 {
    Err(MathError::DivisionByZero)
  } else {
    Ok(a / b)
  }
}

fn safe_sqrt(x: f64) -> Result<f64, MathError> {
  if x < 0.0 {
    Err(MathError::NegativeSquareRoot)
  } else {
    Ok(x.sqrt())
  }
}

fn calculator(expression: &str) -> Result<f64, MathError> {
  let parts: Vec<&str> = expression.split_whitespace().collect();

  if parts.len() != 3 {
    return Err(MathError::InvalidInput(
      "表达式格式应为: 数字 操作符 数字".to_string(),
    ));
  }

  let a: f64 = parts[0]
    .parse()
    .map_err(|_| MathError::InvalidInput(format!("无效数字: {}", parts[0])))?;

  let operator = parts[1];

  let b: f64 = parts[2]
    .parse()
    .map_err(|_| MathError::InvalidInput(format!("无效数字: {}", parts[2])))?;

  match operator {
    "+" => Ok(a + b),
    "-" => Ok(a - b),
    "*" => Ok(a * b),
    "/" => safe_divide(a, b),
    "sqrt" => safe_sqrt(a), // 忽略第二个操作数
    _ => Err(MathError::InvalidInput(format!(
      "不支持的操作符: {}",
      operator
    ))),
  }
}

// 错误转换和包装
#[derive(Debug)]
struct AppError {
  kind: AppErrorKind,
  message: String,
}

#[derive(Debug)]
enum AppErrorKind {
  Io,
  Parse,
  Math,
  Network,
}

impl fmt::Display for AppError {
  fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
    write!(f, "{:?}: {}", self.kind, self.message)
  }
}

impl Error for AppError {}

impl From<io::Error> for AppError {
  fn from(error: io::Error) -> Self {
    AppError {
      kind: AppErrorKind::Io,
      message: error.to_string(),
    }
  }
}

impl From<ParseIntError> for AppError {
  fn from(error: ParseIntError) -> Self {
    AppError {
      kind: AppErrorKind::Parse,
      message: error.to_string(),
    }
  }
}

impl From<MathError> for AppError {
  fn from(error: MathError) -> Self {
    AppError {
      kind: AppErrorKind::Math,
      message: error.to_string(),
    }
  }
}

fn error_conversion_demo() {
  println!("5. 错误转换演示:");

  match process_file_data("42") {
    Ok(result) => println!("  处理结果: {}", result),
    Err(error) => println!("  处理错误: {}", error),
  }

  match process_file_data("invalid") {
    Ok(result) => println!("  处理结果: {}", result),
    Err(error) => println!("  处理错误: {}", error),
  }

  // 错误链示例
  match complex_file_operation() {
    Ok(result) => println!("  复杂文件操作结果: {}", result),
    Err(error) => {
      println!("  复杂文件操作错误: {}", error);

      // 打印错误链
      let mut source = error.source();
      while let Some(err) = source {
        println!("    原因: {}", err);
        source = err.source();
      }
    }
  }

  println!();
}

fn process_file_data(data: &str) -> Result<f64, AppError> {
  let number: i32 = data.parse()?; // 自动转换ParseIntError
  let result = safe_divide(number as f64, 2.0)?; // 自动转换MathError
  Ok(result)
}

fn complex_file_operation() -> Result<String, AppError> {
  // 模拟文件操作失败
  Err(AppError {
    kind: AppErrorKind::Io,
    message: "文件不存在".to_string(),
  })
}

fn best_practices_demo() {
  println!("6. 最佳实践演示:");

  // 1. 使用类型别名简化Result
  type AppResult<T> = Result<T, AppError>;

  fn typed_operation() -> AppResult<i32> {
    Ok(42)
  }

  match typed_operation() {
    Ok(value) => println!("  类型别名结果: {}", value),
    Err(error) => println!("  类型别名错误: {}", error),
  }

  // 2. 早期返回模式
  fn early_return_example(input: i32) -> Result<i32, &'static str> {
    if input < 0 {
      return Err("输入不能为负数");
    }

    if input == 0 {
      return Err("输入不能为零");
    }

    Ok(input * 2)
  }

  match early_return_example(5) {
    Ok(result) => println!("  早期返回结果: {}", result),
    Err(error) => println!("  早期返回错误: {}", error),
  }

  match early_return_example(-1) {
    Ok(result) => println!("  早期返回结果: {}", result),
    Err(error) => println!("  早期返回错误: {}", error),
  }

  // 3. 使用map_err转换错误
  fn map_error_example(input: &str) -> Result<i32, String> {
    input.parse::<i32>().map_err(|e| format!("解析错误: {}", e))
  }

  match map_error_example("42") {
    Ok(result) => println!("  map_err结果: {}", result),
    Err(error) => println!("  map_err错误: {}", error),
  }

  match map_error_example("abc") {
    Ok(result) => println!("  map_err结果: {}", result),
    Err(error) => println!("  map_err错误: {}", error),
  }

  // 4. 组合多个可能失败的操作
  fn combine_operations() -> Result<String, Box<dyn Error>> {
    let a = "10".parse::<i32>()?;
    let b = "5".parse::<i32>()?;
    let result = safe_divide(a as f64, b as f64)?;
    Ok(format!("组合操作结果: {}", result))
  }

  match combine_operations() {
    Ok(result) => println!("  {}", result),
    Err(error) => println!("  组合操作错误: {}", error),
  }

  println!("  错误处理最佳实践:");
  println!("    1. 优先使用Result而不是panic!");
  println!("    2. 使用?操作符简化错误传播");
  println!("    3. 为应用定义统一的错误类型");
  println!("    4. 实现From trait进行错误转换");
  println!("    5. 提供有意义的错误信息");
  println!("    6. 在库代码中返回Result，在应用代码中处理错误");
  println!("    7. 使用类型别名简化复杂的Result类型");

  println!();
}

// 恐慌处理示例
fn panic_demo() {
  println!("恐慌处理演示:");

  // 设置恐慌钩子
  std::panic::set_hook(Box::new(|panic_info| {
    println!("自定义恐慌处理: {:?}", panic_info);
  }));

  // 捕获恐慌
  let result = std::panic::catch_unwind(|| {
    panic!("这是一个测试恐慌");
  });

  match result {
    Ok(_) => println!("  没有恐慌发生"),
    Err(_) => println!("  捕获到恐慌"),
  }

  // 恢复默认恐慌处理
  let _ = std::panic::take_hook();
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_safe_divide() {
    assert_eq!(safe_divide(10.0, 2.0), Ok(5.0));
    assert!(matches!(
      safe_divide(10.0, 0.0),
      Err(MathError::DivisionByZero)
    ));
  }

  #[test]
  fn test_safe_sqrt() {
    assert_eq!(safe_sqrt(16.0), Ok(4.0));
    assert!(matches!(
      safe_sqrt(-4.0),
      Err(MathError::NegativeSquareRoot)
    ));
  }

  #[test]
  fn test_calculator() {
    assert_eq!(calculator("10 + 5"), Ok(15.0));
    assert_eq!(calculator("10 - 3"), Ok(7.0));
    assert_eq!(calculator("4 * 3"), Ok(12.0));
    assert_eq!(calculator("15 / 3"), Ok(5.0));
    assert!(matches!(
      calculator("10 / 0"),
      Err(MathError::DivisionByZero)
    ));
    assert!(matches!(
      calculator("invalid"),
      Err(MathError::InvalidInput(_))
    ));
  }

  #[test]
  fn test_parse_number() {
    assert_eq!(parse_number("42"), Some(42));
    assert_eq!(parse_number("abc"), None);
  }

  #[test]
  fn test_error_conversion() {
    let parse_error: ParseIntError = "abc".parse::<i32>().unwrap_err();
    let app_error: AppError = parse_error.into();
    assert!(matches!(app_error.kind, AppErrorKind::Parse));
  }
}

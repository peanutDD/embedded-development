// Rust错误处理示例：展示Result、Option、自定义错误、错误传播等

use std::fmt;
use std::error::Error;
use std::fs::File;
use std::io::{self, Read};
use std::num::ParseIntError;

// 1. 自定义错误类型
#[derive(Debug)]
enum MathError {
    DivisionByZero,
    NegativeLogarithm,
    InvalidInput(String),
}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            MathError::DivisionByZero => write!(f, "除零错误"),
            MathError::NegativeLogarithm => write!(f, "负数对数错误"),
            MathError::InvalidInput(msg) => write!(f, "无效输入: {}", msg),
        }
    }
}

impl Error for MathError {}

// 2. 使用Result的数学运算
fn divide(a: f64, b: f64) -> Result<f64, MathError> {
    if b == 0.0 {
        Err(MathError::DivisionByZero)
    } else {
        Ok(a / b)
    }
}

fn logarithm(x: f64) -> Result<f64, MathError> {
    if x <= 0.0 {
        Err(MathError::NegativeLogarithm)
    } else {
        Ok(x.ln())
    }
}

fn square_root(x: f64) -> Result<f64, MathError> {
    if x < 0.0 {
        Err(MathError::InvalidInput("负数不能开平方根".to_string()))
    } else {
        Ok(x.sqrt())
    }
}

// 3. 错误传播和组合
fn complex_calculation(a: f64, b: f64, c: f64) -> Result<f64, MathError> {
    let division_result = divide(a, b)?;
    let log_result = logarithm(division_result)?;
    let sqrt_result = square_root(log_result + c)?;
    Ok(sqrt_result)
}

// 4. 多种错误类型的处理
#[derive(Debug)]
enum AppError {
    Io(io::Error),
    Parse(ParseIntError),
    Math(MathError),
    Custom(String),
}

impl fmt::Display for AppError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            AppError::Io(err) => write!(f, "IO错误: {}", err),
            AppError::Parse(err) => write!(f, "解析错误: {}", err),
            AppError::Math(err) => write!(f, "数学错误: {}", err),
            AppError::Custom(msg) => write!(f, "自定义错误: {}", msg),
        }
    }
}

impl Error for AppError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            AppError::Io(err) => Some(err),
            AppError::Parse(err) => Some(err),
            AppError::Math(err) => Some(err),
            AppError::Custom(_) => None,
        }
    }
}

impl From<io::Error> for AppError {
    fn from(err: io::Error) -> Self {
        AppError::Io(err)
    }
}

impl From<ParseIntError> for AppError {
    fn from(err: ParseIntError) -> Self {
        AppError::Parse(err)
    }
}

impl From<MathError> for AppError {
    fn from(err: MathError) -> Self {
        AppError::Math(err)
    }
}

// 5. 文件操作和错误处理
fn read_number_from_file(filename: &str) -> Result<i32, AppError> {
    let mut file = File::open(filename)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    let number: i32 = contents.trim().parse()?;
    Ok(number)
}

// 6. Option的使用示例
fn find_word(text: &str, word: &str) -> Option<usize> {
    text.find(word)
}

fn get_first_word(text: &str) -> Option<&str> {
    text.split_whitespace().next()
}

fn safe_division(a: i32, b: i32) -> Option<i32> {
    if b != 0 {
        Some(a / b)
    } else {
        None
    }
}

// 7. Option和Result的组合使用
fn parse_and_divide(s1: &str, s2: &str) -> Result<Option<i32>, ParseIntError> {
    let num1: i32 = s1.parse()?;
    let num2: i32 = s2.parse()?;
    Ok(safe_division(num1, num2))
}

// 8. 错误恢复策略
fn divide_with_default(a: f64, b: f64, default: f64) -> f64 {
    divide(a, b).unwrap_or(default)
}

fn divide_with_fallback(a: f64, b: f64) -> Result<f64, MathError> {
    divide(a, b).or_else(|_| {
        println!("除法失败，尝试使用默认值");
        Ok(0.0)
    })
}

// 9. 链式错误处理
fn process_numbers(numbers: Vec<&str>) -> Result<Vec<i32>, AppError> {
    numbers
        .iter()
        .map(|s| s.parse::<i32>().map_err(AppError::from))
        .collect()
}

// 10. 自定义Result类型
type MathResult<T> = Result<T, MathError>;
type AppResult<T> = Result<T, AppError>;

fn calculate_average(numbers: &[f64]) -> MathResult<f64> {
    if numbers.is_empty() {
        return Err(MathError::InvalidInput("空数组".to_string()));
    }
    
    let sum: f64 = numbers.iter().sum();
    Ok(sum / numbers.len() as f64)
}

// 11. 错误上下文添加
trait ErrorContext<T> {
    fn with_context<F>(self, f: F) -> Result<T, AppError>
    where
        F: FnOnce() -> String;
}

impl<T, E> ErrorContext<T> for Result<T, E>
where
    E: Error + 'static,
{
    fn with_context<F>(self, f: F) -> Result<T, AppError>
    where
        F: FnOnce() -> String,
    {
        self.map_err(|_| AppError::Custom(f()))
    }
}

// 12. 早期返回模式
fn validate_and_process(input: &str) -> AppResult<String> {
    if input.is_empty() {
        return Err(AppError::Custom("输入不能为空".to_string()));
    }
    
    if input.len() < 3 {
        return Err(AppError::Custom("输入长度不能少于3个字符".to_string()));
    }
    
    if !input.chars().all(|c| c.is_alphanumeric()) {
        return Err(AppError::Custom("输入只能包含字母和数字".to_string()));
    }
    
    Ok(input.to_uppercase())
}

// 13. 批量错误处理
fn process_batch(inputs: Vec<&str>) -> (Vec<String>, Vec<AppError>) {
    let mut successes = Vec::new();
    let mut errors = Vec::new();
    
    for input in inputs {
        match validate_and_process(input) {
            Ok(result) => successes.push(result),
            Err(error) => errors.push(error),
        }
    }
    
    (successes, errors)
}

// 14. 错误累积
fn accumulate_errors(inputs: Vec<&str>) -> Result<Vec<String>, Vec<AppError>> {
    let mut results = Vec::new();
    let mut errors = Vec::new();
    
    for input in inputs {
        match validate_and_process(input) {
            Ok(result) => results.push(result),
            Err(error) => errors.push(error),
        }
    }
    
    if errors.is_empty() {
        Ok(results)
    } else {
        Err(errors)
    }
}

// 15. 异步错误处理（模拟）
async fn async_operation(value: i32) -> Result<i32, AppError> {
    if value < 0 {
        Err(AppError::Custom("值不能为负数".to_string()))
    } else {
        Ok(value * 2)
    }
}

fn main() {
    println!("=== Rust错误处理示例 ===\n");
    
    // 1. 基本Result使用
    match divide(10.0, 2.0) {
        Ok(result) => println!("10 / 2 = {}", result),
        Err(e) => println!("错误: {}", e),
    }
    
    match divide(10.0, 0.0) {
        Ok(result) => println!("10 / 0 = {}", result),
        Err(e) => println!("错误: {}", e),
    }
    
    // 2. 错误传播
    match complex_calculation(10.0, 2.0, 1.0) {
        Ok(result) => println!("复杂计算结果: {}", result),
        Err(e) => println!("复杂计算错误: {}", e),
    }
    
    // 3. Option使用
    let text = "Hello world from Rust";
    match find_word(text, "world") {
        Some(index) => println!("找到'world'在位置: {}", index),
        None => println!("未找到'world'"),
    }
    
    if let Some(first) = get_first_word(text) {
        println!("第一个单词: {}", first);
    }
    
    // 4. Option和Result组合
    match parse_and_divide("10", "2") {
        Ok(Some(result)) => println!("解析并除法结果: {}", result),
        Ok(None) => println!("除法结果为None（除零）"),
        Err(e) => println!("解析错误: {}", e),
    }
    
    // 5. 错误恢复
    let result = divide_with_default(10.0, 0.0, -1.0);
    println!("带默认值的除法: {}", result);
    
    match divide_with_fallback(10.0, 0.0) {
        Ok(result) => println!("带回退的除法: {}", result),
        Err(e) => println!("回退也失败: {}", e),
    }
    
    // 6. 批量处理
    let numbers = vec!["1", "2", "abc", "4", "5"];
    match process_numbers(numbers) {
        Ok(nums) => println!("解析的数字: {:?}", nums),
        Err(e) => println!("批量处理错误: {}", e),
    }
    
    // 7. 平均值计算
    let values = vec![1.0, 2.0, 3.0, 4.0, 5.0];
    match calculate_average(&values) {
        Ok(avg) => println!("平均值: {}", avg),
        Err(e) => println!("平均值计算错误: {}", e),
    }
    
    // 8. 输入验证
    let inputs = vec!["hello", "hi", "world123", "test!", ""];
    let (successes, errors) = process_batch(inputs);
    println!("成功处理: {:?}", successes);
    println!("处理错误: {:?}", errors);
    
    // 9. 错误累积
    let test_inputs = vec!["valid123", "also456", ""];
    match accumulate_errors(test_inputs) {
        Ok(results) => println!("所有输入都有效: {:?}", results),
        Err(errors) => println!("累积的错误: {:?}", errors),
    }
    
    // 10. 链式操作
    let chain_result = "42"
        .parse::<i32>()
        .map(|n| n * 2)
        .and_then(|n| if n > 50 { Ok(n) } else { Err("42".parse::<i32>().unwrap_err()) })
        .map(|n| format!("结果: {}", n));
    
    match chain_result {
        Ok(s) => println!("链式操作: {}", s),
        Err(e) => println!("链式操作错误: {}", e),
    }
    
    println!("\n=== 错误处理最佳实践 ===");
    println!("1. 使用Result<T, E>处理可恢复错误");
    println!("2. 使用Option<T>处理可能为空的值");
    println!("3. 使用?操作符进行错误传播");
    println!("4. 创建自定义错误类型提供更好的错误信息");
    println!("5. 使用From trait进行错误类型转换");
    println!("6. 提供错误恢复机制");
    println!("7. 在适当的层级处理错误");
    println!("8. 使用unwrap()和expect()要谨慎");
}

// 错误处理工具函数
mod error_utils {
    use super::*;
    
    // 安全的unwrap替代
    pub fn safe_unwrap<T, E: fmt::Debug>(result: Result<T, E>, msg: &str) -> T {
        match result {
            Ok(value) => value,
            Err(e) => {
                eprintln!("错误 {}: {:?}", msg, e);
                panic!("程序终止");
            }
        }
    }
    
    // 错误日志记录
    pub fn log_error<E: fmt::Display>(error: &E, context: &str) {
        eprintln!("[ERROR] {}: {}", context, error);
    }
    
    // 错误重试机制
    pub fn retry<T, E, F>(mut f: F, max_attempts: usize) -> Result<T, E>
    where
        F: FnMut() -> Result<T, E>,
    {
        let mut attempts = 0;
        loop {
            match f() {
                Ok(result) => return Ok(result),
                Err(e) => {
                    attempts += 1;
                    if attempts >= max_attempts {
                        return Err(e);
                    }
                    println!("重试第{}次...", attempts);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_divide_success() {
        assert_eq!(divide(10.0, 2.0).unwrap(), 5.0);
    }
    
    #[test]
    fn test_divide_by_zero() {
        assert!(divide(10.0, 0.0).is_err());
    }
    
    #[test]
    fn test_logarithm_positive() {
        assert!(logarithm(2.718).is_ok());
    }
    
    #[test]
    fn test_logarithm_negative() {
        assert!(logarithm(-1.0).is_err());
    }
    
    #[test]
    fn test_safe_division_some() {
        assert_eq!(safe_division(10, 2), Some(5));
    }
    
    #[test]
    fn test_safe_division_none() {
        assert_eq!(safe_division(10, 0), None);
    }
    
    #[test]
    fn test_validate_and_process_valid() {
        assert_eq!(validate_and_process("abc123").unwrap(), "ABC123");
    }
    
    #[test]
    fn test_validate_and_process_empty() {
        assert!(validate_and_process("").is_err());
    }
    
    #[test]
    fn test_validate_and_process_too_short() {
        assert!(validate_and_process("ab").is_err());
    }
    
    #[test]
    fn test_calculate_average() {
        let numbers = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        assert_eq!(calculate_average(&numbers).unwrap(), 3.0);
    }
    
    #[test]
    fn test_calculate_average_empty() {
        let numbers = vec![];
        assert!(calculate_average(&numbers).is_err());
    }
}
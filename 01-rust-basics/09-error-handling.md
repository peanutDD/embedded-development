# 错误处理

## 概述

错误处理是编程中的重要主题，Rust通过类型系统提供了强大而安全的错误处理机制。与许多语言使用异常不同，Rust使用`Result<T, E>`类型来表示可能失败的操作，使错误处理成为类型系统的一部分，强制开发者显式处理错误情况。

## 学习目标

- 理解Rust错误处理的哲学和设计原则
- 掌握`Result<T, E>`和`Option<T>`类型的使用
- 学会错误传播和`?`操作符的使用
- 掌握自定义错误类型的创建和使用
- 理解可恢复错误和不可恢复错误的区别
- 学会错误处理的最佳实践和模式
- 掌握错误链和错误上下文的处理
- 了解第三方错误处理库的使用

## 1. Rust错误处理基础

### 1.1 错误处理哲学

```rust
// Rust的错误处理原则：
// 1. 错误是值，不是异常
// 2. 错误必须被显式处理
// 3. 编译时保证错误处理的完整性

// 传统语言的异常处理（伪代码）
// try {
//     let result = risky_operation();
//     process(result);
// } catch (Exception e) {
//     handle_error(e);
// }

// Rust的错误处理方式
fn rust_error_handling() {
    match risky_operation() {
        Ok(result) => process(result),
        Err(error) => handle_error(error),
    }
}

fn risky_operation() -> Result<i32, String> {
    // 模拟可能失败的操作
    if rand::random::<bool>() {
        Ok(42)
    } else {
        Err("Operation failed".to_string())
    }
}

fn process(value: i32) {
    println!("Processing value: {}", value);
}

fn handle_error(error: String) {
    println!("Error occurred: {}", error);
}

// 错误处理的强制性
fn must_handle_errors() {
    // 编译错误：未处理Result
    // let _result = risky_operation();  // 警告：未使用的Result
    
    // 正确的处理方式
    let _result = risky_operation().unwrap_or_else(|e| {
        println!("Handled error: {}", e);
        0  // 默认值
    });
}

fn main() {
    rust_error_handling();
    must_handle_errors();
}
```

### 1.2 Option类型回顾

```rust
// Option<T>用于表示可能为空的值
fn find_user(id: u32) -> Option<User> {
    if id == 1 {
        Some(User {
            id: 1,
            name: "Alice".to_string(),
            email: "alice@example.com".to_string(),
        })
    } else {
        None
    }
}

#[derive(Debug)]
struct User {
    id: u32,
    name: String,
    email: String,
}

// Option的常用方法
fn option_methods_demo() {
    let user = find_user(1);
    
    // 使用match处理Option
    match user {
        Some(u) => println!("Found user: {}", u.name),
        None => println!("User not found"),
    }
    
    // 使用if let简化处理
    if let Some(u) = find_user(1) {
        println!("User email: {}", u.email);
    }
    
    // 使用unwrap_or提供默认值
    let default_user = User {
        id: 0,
        name: "Guest".to_string(),
        email: "guest@example.com".to_string(),
    };
    let user = find_user(999).unwrap_or(default_user);
    println!("User: {:?}", user);
    
    // 使用map转换Option中的值
    let user_name = find_user(1).map(|u| u.name);
    println!("User name: {:?}", user_name);
    
    // 使用and_then链式操作
    let user_email_domain = find_user(1)
        .and_then(|u| u.email.split('@').nth(1).map(|s| s.to_string()));
    println!("Email domain: {:?}", user_email_domain);
}

fn main() {
    option_methods_demo();
}
```

## 2. Result类型详解

### 2.1 Result基础

```rust
use std::fs::File;
use std::io::ErrorKind;

fn open_file_example() {
    let file_result = File::open("hello.txt");
    
    match file_result {
        Ok(file) => println!("文件打开成功: {:?}", file),
        Err(error) => println!("文件打开失败: {:?}", error),
    }
}

// Result的类型定义
// enum Result<T, E> {
//     Ok(T),
//     Err(E),
// }

// 自定义返回Result的函数
fn divide(a: f64, b: f64) -> Result<f64, String> {
    if b == 0.0 {
        Err("Division by zero".to_string())
    } else {
        Ok(a / b)
    }
}

fn parse_number(s: &str) -> Result<i32, std::num::ParseIntError> {
    s.parse::<i32>()
}

// Result的常用方法
fn result_methods_demo() {
    // 使用unwrap（危险，会panic）
    // let result = divide(10.0, 0.0).unwrap();  // 会panic
    
    // 使用expect（提供错误信息）
    // let result = divide(10.0, 0.0).expect("Division failed");  // 会panic with message
    
    // 使用unwrap_or提供默认值
    let result = divide(10.0, 0.0).unwrap_or(0.0);
    println!("Result with default: {}", result);
    
    // 使用unwrap_or_else提供计算默认值的闭包
    let result = divide(10.0, 0.0).unwrap_or_else(|err| {
        println!("Error occurred: {}", err);
        -1.0
    });
    println!("Result with computed default: {}", result);
    
    // 使用map转换Ok值
    let result = divide(10.0, 2.0).map(|x| x * 2.0);
    println!("Mapped result: {:?}", result);
    
    // 使用map_err转换Err值
    let result = divide(10.0, 0.0).map_err(|e| format!("Math error: {}", e));
    println!("Mapped error: {:?}", result);
    
    // 使用and_then链式操作
    let result = parse_number("42")
        .and_then(|n| divide(100.0, n as f64))
        .map(|x| x.round() as i32);
    println!("Chained result: {:?}", result);
}

fn main() {
    open_file_example();
    result_methods_demo();
}
```

### 2.2 错误匹配和处理

```rust
use std::fs::File;
use std::io::{self, Read, ErrorKind};

fn handle_file_errors() {
    let file_result = File::open("hello.txt");
    
    let mut file = match file_result {
        Ok(file) => file,
        Err(error) => match error.kind() {
            ErrorKind::NotFound => {
                println!("File not found, creating new file...");
                match File::create("hello.txt") {
                    Ok(fc) => fc,
                    Err(e) => panic!("Problem creating the file: {:?}", e),
                }
            }
            ErrorKind::PermissionDenied => {
                panic!("Permission denied to access the file");
            }
            other_error => {
                panic!("Problem opening the file: {:?}", other_error);
            }
        }
    };
    
    // 读取文件内容
    let mut contents = String::new();
    match file.read_to_string(&mut contents) {
        Ok(bytes_read) => println!("Read {} bytes: {}", bytes_read, contents),
        Err(e) => println!("Error reading file: {}", e),
    }
}

// 使用if let简化错误处理
fn simplified_error_handling() {
    if let Err(e) = File::open("nonexistent.txt") {
        match e.kind() {
            ErrorKind::NotFound => println!("File not found"),
            ErrorKind::PermissionDenied => println!("Permission denied"),
            _ => println!("Other error: {}", e),
        }
    }
}

// 组合多个可能失败的操作
fn multiple_operations() -> Result<String, io::Error> {
    let mut file = File::open("config.txt")?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;
    
    // 进一步处理内容
    let processed = contents.trim().to_uppercase();
    Ok(processed)
}

fn main() {
    handle_file_errors();
    simplified_error_handling();
    
    match multiple_operations() {
        Ok(content) => println!("Processed content: {}", content),
        Err(e) => println!("Error in multiple operations: {}", e),
    }
}
```

## 3. 错误传播和?操作符

### 3.1 ?操作符基础

```rust
use std::fs::File;
use std::io::{self, Read};

// 传统的错误传播方式
fn read_username_from_file_verbose() -> Result<String, io::Error> {
    let f = File::open("hello.txt");
    
    let mut f = match f {
        Ok(file) => file,
        Err(e) => return Err(e),
    };
    
    let mut s = String::new();
    
    match f.read_to_string(&mut s) {
        Ok(_) => Ok(s),
        Err(e) => Err(e),
    }
}

// 使用?操作符简化
fn read_username_from_file() -> Result<String, io::Error> {
    let mut f = File::open("hello.txt")?;
    let mut s = String::new();
    f.read_to_string(&mut s)?;
    Ok(s)
}

// 进一步简化
fn read_username_from_file_short() -> Result<String, io::Error> {
    let mut s = String::new();
    File::open("hello.txt")?.read_to_string(&mut s)?;
    Ok(s)
}

// 最简化版本
fn read_username_from_file_shortest() -> Result<String, io::Error> {
    std::fs::read_to_string("hello.txt")
}

// ?操作符在Option中的使用
fn find_user_email(user_id: u32) -> Option<String> {
    let user = find_user(user_id)?;  // 如果None，直接返回None
    Some(user.email)
}

// 链式使用?操作符
fn process_user_data(user_id: u32) -> Option<String> {
    let user = find_user(user_id)?;
    let domain = user.email.split('@').nth(1)?;
    Some(domain.to_uppercase())
}

fn main() {
    // 测试不同版本的函数
    match read_username_from_file() {
        Ok(content) => println!("File content: {}", content),
        Err(e) => println!("Error reading file: {}", e),
    }
    
    if let Some(email) = find_user_email(1) {
        println!("User email: {}", email);
    }
    
    if let Some(domain) = process_user_data(1) {
        println!("Email domain: {}", domain);
    }
}
```

### 3.2 ?操作符的工作原理

```rust
// ?操作符的展开
fn example_with_question_mark() -> Result<i32, String> {
    let x = might_fail()?;
    Ok(x * 2)
}

// 等价于：
fn example_without_question_mark() -> Result<i32, String> {
    let x = match might_fail() {
        Ok(val) => val,
        Err(err) => return Err(err),
    };
    Ok(x * 2)
}

fn might_fail() -> Result<i32, String> {
    if rand::random::<bool>() {
        Ok(42)
    } else {
        Err("Something went wrong".to_string())
    }
}

// ?操作符与From trait
use std::num::ParseIntError;

fn parse_and_double(s: &str) -> Result<i32, ParseIntError> {
    let n: i32 = s.parse()?;  // ParseIntError自动转换
    Ok(n * 2)
}

// 自定义错误类型的From实现
#[derive(Debug)]
enum MyError {
    ParseError(ParseIntError),
    IoError(std::io::Error),
}

impl From<ParseIntError> for MyError {
    fn from(err: ParseIntError) -> MyError {
        MyError::ParseError(err)
    }
}

impl From<std::io::Error> for MyError {
    fn from(err: std::io::Error) -> MyError {
        MyError::IoError(err)
    }
}

fn complex_operation(s: &str) -> Result<i32, MyError> {
    let n: i32 = s.parse()?;  // ParseIntError -> MyError
    let content = std::fs::read_to_string("config.txt")?;  // io::Error -> MyError
    Ok(n + content.len() as i32)
}

fn main() {
    match example_with_question_mark() {
        Ok(result) => println!("Result: {}", result),
        Err(e) => println!("Error: {}", e),
    }
    
    match parse_and_double("42") {
        Ok(result) => println!("Doubled: {}", result),
        Err(e) => println!("Parse error: {}", e),
    }
    
    match complex_operation("10") {
        Ok(result) => println!("Complex result: {}", result),
        Err(e) => println!("Complex error: {:?}", e),
    }
}
```

## 4. 自定义错误类型

### 4.1 简单自定义错误

```rust
use std::fmt;

// 简单的自定义错误枚举
#[derive(Debug)]
enum MathError {
    DivisionByZero,
    NegativeSquareRoot,
    Overflow,
}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            MathError::DivisionByZero => write!(f, "Cannot divide by zero"),
            MathError::NegativeSquareRoot => write!(f, "Cannot take square root of negative number"),
            MathError::Overflow => write!(f, "Mathematical overflow occurred"),
        }
    }
}

impl std::error::Error for MathError {}

// 使用自定义错误的函数
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

fn safe_multiply(a: f64, b: f64) -> Result<f64, MathError> {
    let result = a * b;
    if result.is_infinite() {
        Err(MathError::Overflow)
    } else {
        Ok(result)
    }
}

// 组合使用自定义错误
fn complex_calculation(a: f64, b: f64, c: f64) -> Result<f64, MathError> {
    let division_result = safe_divide(a, b)?;
    let sqrt_result = safe_sqrt(division_result)?;
    let final_result = safe_multiply(sqrt_result, c)?;
    Ok(final_result)
}

fn main() {
    // 测试各种错误情况
    match safe_divide(10.0, 0.0) {
        Ok(result) => println!("Division result: {}", result),
        Err(e) => println!("Division error: {}", e),
    }
    
    match safe_sqrt(-4.0) {
        Ok(result) => println!("Square root result: {}", result),
        Err(e) => println!("Square root error: {}", e),
    }
    
    match complex_calculation(16.0, 4.0, 2.0) {
        Ok(result) => println!("Complex calculation result: {}", result),
        Err(e) => println!("Complex calculation error: {}", e),
    }
}
```

### 4.2 复杂自定义错误类型

```rust
use std::fmt;
use std::error::Error;
use std::num::ParseIntError;
use std::io;

// 复杂的自定义错误类型
#[derive(Debug)]
pub enum AppError {
    Io(io::Error),
    Parse(ParseIntError),
    Custom(String),
    Validation { field: String, message: String },
    Network { code: u16, message: String },
}

impl fmt::Display for AppError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AppError::Io(err) => write!(f, "IO error: {}", err),
            AppError::Parse(err) => write!(f, "Parse error: {}", err),
            AppError::Custom(msg) => write!(f, "Custom error: {}", msg),
            AppError::Validation { field, message } => {
                write!(f, "Validation error in field '{}': {}", field, message)
            }
            AppError::Network { code, message } => {
                write!(f, "Network error {}: {}", code, message)
            }
        }
    }
}

impl Error for AppError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            AppError::Io(err) => Some(err),
            AppError::Parse(err) => Some(err),
            _ => None,
        }
    }
}

// 实现From trait用于自动转换
impl From<io::Error> for AppError {
    fn from(err: io::Error) -> AppError {
        AppError::Io(err)
    }
}

impl From<ParseIntError> for AppError {
    fn from(err: ParseIntError) -> AppError {
        AppError::Parse(err)
    }
}

impl From<String> for AppError {
    fn from(err: String) -> AppError {
        AppError::Custom(err)
    }
}

impl From<&str> for AppError {
    fn from(err: &str) -> AppError {
        AppError::Custom(err.to_string())
    }
}

// 使用复杂错误类型的应用
struct User {
    id: u32,
    name: String,
    email: String,
    age: u32,
}

fn validate_user(name: &str, email: &str, age_str: &str) -> Result<User, AppError> {
    // 验证姓名
    if name.is_empty() {
        return Err(AppError::Validation {
            field: "name".to_string(),
            message: "Name cannot be empty".to_string(),
        });
    }
    
    // 验证邮箱
    if !email.contains('@') {
        return Err(AppError::Validation {
            field: "email".to_string(),
            message: "Invalid email format".to_string(),
        });
    }
    
    // 解析年龄（可能产生ParseIntError，自动转换为AppError）
    let age: u32 = age_str.parse()?;
    
    // 验证年龄范围
    if age > 150 {
        return Err(AppError::Validation {
            field: "age".to_string(),
            message: "Age must be less than 150".to_string(),
        });
    }
    
    Ok(User {
        id: rand::random(),
        name: name.to_string(),
        email: email.to_string(),
        age,
    })
}

fn save_user_to_file(user: &User) -> Result<(), AppError> {
    let user_data = format!("{},{},{},{}\n", user.id, user.name, user.email, user.age);
    std::fs::write("users.txt", user_data)?;  // io::Error自动转换为AppError
    Ok(())
}

fn create_and_save_user(name: &str, email: &str, age_str: &str) -> Result<User, AppError> {
    let user = validate_user(name, email, age_str)?;
    save_user_to_file(&user)?;
    Ok(user)
}

fn main() {
    // 测试各种错误情况
    let test_cases = vec![
        ("", "test@example.com", "25"),  // 空姓名
        ("Alice", "invalid-email", "25"),  // 无效邮箱
        ("Bob", "bob@example.com", "abc"),  // 无效年龄
        ("Charlie", "charlie@example.com", "200"),  // 年龄超出范围
        ("David", "david@example.com", "30"),  // 正常情况
    ];
    
    for (name, email, age) in test_cases {
        match create_and_save_user(name, email, age) {
            Ok(user) => println!("Created user: {} (ID: {})", user.name, user.id),
            Err(e) => {
                println!("Error creating user: {}", e);
                
                // 打印错误链
                let mut source = e.source();
                while let Some(err) = source {
                    println!("  Caused by: {}", err);
                    source = err.source();
                }
            }
        }
    }
## 5. 错误处理最佳实践

### 5.1 错误处理策略

```rust
use std::error::Error;
use std::fmt;

// 1. 使用类型系统表达可能的失败
fn get_user_by_id(id: u32) -> Option<User> {
    // 明确表示可能没有找到用户
    if id == 0 {
        None
    } else {
        Some(User {
            id,
            name: format!("User {}", id),
            email: format!("user{}@example.com", id),
            age: 25,
        })
    }
}

// 2. 区分可恢复和不可恢复的错误
fn process_config_file(path: &str) -> Result<Config, ConfigError> {
    // 可恢复错误：文件不存在，可以使用默认配置
    let content = match std::fs::read_to_string(path) {
        Ok(content) => content,
        Err(_) => {
            println!("Config file not found, using defaults");
            return Ok(Config::default());
        }
    };
    
    // 不可恢复错误：配置格式错误，应该返回错误
    parse_config(&content)
}

// 3. 提供有意义的错误信息
#[derive(Debug)]
struct ConfigError {
    message: String,
    line: Option<usize>,
    column: Option<usize>,
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match (self.line, self.column) {
            (Some(line), Some(col)) => {
                write!(f, "Config error at line {}, column {}: {}", line, col, self.message)
            }
            (Some(line), None) => {
                write!(f, "Config error at line {}: {}", line, self.message)
            }
            _ => write!(f, "Config error: {}", self.message),
        }
    }
}

impl Error for ConfigError {}

struct Config {
    database_url: String,
    port: u16,
    debug: bool,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            database_url: "sqlite://default.db".to_string(),
            port: 8080,
            debug: false,
        }
    }
}

fn parse_config(content: &str) -> Result<Config, ConfigError> {
    let mut config = Config::default();
    
    for (line_num, line) in content.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        
        let parts: Vec<&str> = line.split('=').collect();
        if parts.len() != 2 {
            return Err(ConfigError {
                message: "Invalid format, expected key=value".to_string(),
                line: Some(line_num + 1),
                column: None,
            });
        }
        
        let key = parts[0].trim();
        let value = parts[1].trim();
        
        match key {
            "database_url" => config.database_url = value.to_string(),
            "port" => {
                config.port = value.parse().map_err(|_| ConfigError {
                    message: format!("Invalid port number: {}", value),
                    line: Some(line_num + 1),
                    column: Some(parts[0].len() + 1),
                })?;
            }
            "debug" => {
                config.debug = value.parse().map_err(|_| ConfigError {
                    message: format!("Invalid boolean value: {}", value),
                    line: Some(line_num + 1),
                    column: Some(parts[0].len() + 1),
                })?;
            }
            _ => {
                return Err(ConfigError {
                    message: format!("Unknown configuration key: {}", key),
                    line: Some(line_num + 1),
                    column: None,
                });
            }
        }
    }
    
    Ok(config)
}

// 4. 错误转换和包装
fn load_and_validate_config(path: &str) -> Result<Config, Box<dyn Error>> {
    let config = process_config_file(path)?;
    
    // 验证配置
    if config.port < 1024 {
        return Err("Port number must be >= 1024".into());
    }
    
    if config.database_url.is_empty() {
        return Err("Database URL cannot be empty".into());
    }
    
    Ok(config)
}

fn main() {
    // 测试配置加载
    match load_and_validate_config("config.txt") {
        Ok(config) => {
            println!("Config loaded successfully:");
            println!("  Database: {}", config.database_url);
            println!("  Port: {}", config.port);
            println!("  Debug: {}", config.debug);
        }
        Err(e) => {
            eprintln!("Failed to load config: {}", e);
            
            // 打印错误链
            let mut source = e.source();
            while let Some(err) = source {
                eprintln!("  Caused by: {}", err);
                source = err.source();
            }
        }
    }
}
```

### 5.2 错误处理模式

```rust
use std::collections::HashMap;

// 1. 早期返回模式
fn validate_user_input(input: &HashMap<String, String>) -> Result<ValidatedInput, String> {
    // 检查必需字段
    let name = input.get("name").ok_or("Missing name field")?;
    if name.is_empty() {
        return Err("Name cannot be empty".to_string());
    }
    
    let email = input.get("email").ok_or("Missing email field")?;
    if !email.contains('@') {
        return Err("Invalid email format".to_string());
    }
    
    let age_str = input.get("age").ok_or("Missing age field")?;
    let age: u32 = age_str.parse().map_err(|_| "Invalid age format")?;
    if age > 150 {
        return Err("Age must be less than 150".to_string());
    }
    
    Ok(ValidatedInput {
        name: name.clone(),
        email: email.clone(),
        age,
    })
}

struct ValidatedInput {
    name: String,
    email: String,
    age: u32,
}

// 2. 错误累积模式
#[derive(Debug)]
struct ValidationErrors {
    errors: Vec<String>,
}

impl ValidationErrors {
    fn new() -> Self {
        ValidationErrors { errors: Vec::new() }
    }
    
    fn add_error(&mut self, error: String) {
        self.errors.push(error);
    }
    
    fn has_errors(&self) -> bool {
        !self.errors.is_empty()
    }
    
    fn into_result<T>(self, value: T) -> Result<T, ValidationErrors> {
        if self.has_errors() {
            Err(self)
        } else {
            Ok(value)
        }
    }
}

impl fmt::Display for ValidationErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Validation errors:\n")?;
        for error in &self.errors {
            write!(f, "  - {}\n", error)?;
        }
        Ok(())
    }
}

impl std::error::Error for ValidationErrors {}

fn validate_user_comprehensive(input: &HashMap<String, String>) -> Result<ValidatedInput, ValidationErrors> {
    let mut errors = ValidationErrors::new();
    
    // 验证姓名
    let name = match input.get("name") {
        Some(name) if !name.is_empty() => name.clone(),
        Some(_) => {
            errors.add_error("Name cannot be empty".to_string());
            String::new()
        }
        None => {
            errors.add_error("Missing name field".to_string());
            String::new()
        }
    };
    
    // 验证邮箱
    let email = match input.get("email") {
        Some(email) if email.contains('@') && email.contains('.') => email.clone(),
        Some(email) => {
            errors.add_error(format!("Invalid email format: {}", email));
            String::new()
        }
        None => {
            errors.add_error("Missing email field".to_string());
            String::new()
        }
    };
    
    // 验证年龄
    let age = match input.get("age") {
        Some(age_str) => match age_str.parse::<u32>() {
            Ok(age) if age <= 150 => age,
            Ok(age) => {
                errors.add_error(format!("Age {} is too high (max 150)", age));
                0
            }
            Err(_) => {
                errors.add_error(format!("Invalid age format: {}", age_str));
                0
            }
        },
        None => {
            errors.add_error("Missing age field".to_string());
            0
        }
    };
    
    errors.into_result(ValidatedInput { name, email, age })
}

// 3. 重试模式
use std::time::{Duration, Instant};
use std::thread;

fn retry_operation<F, T, E>(
    mut operation: F,
    max_attempts: usize,
    delay: Duration,
) -> Result<T, E>
where
    F: FnMut() -> Result<T, E>,
{
    let mut attempts = 0;
    
    loop {
        attempts += 1;
        
        match operation() {
            Ok(result) => return Ok(result),
            Err(e) if attempts >= max_attempts => return Err(e),
            Err(_) => {
                println!("Attempt {} failed, retrying in {:?}...", attempts, delay);
                thread::sleep(delay);
            }
        }
    }
}

// 模拟可能失败的网络操作
fn unreliable_network_call() -> Result<String, String> {
    use rand::Rng;
    let mut rng = rand::thread_rng();
    
    if rng.gen_bool(0.7) {  // 70% 失败率
        Err("Network timeout".to_string())
    } else {
        Ok("Success response".to_string())
    }
}

fn main() {
    // 测试早期返回模式
    let mut input = HashMap::new();
    input.insert("name".to_string(), "Alice".to_string());
    input.insert("email".to_string(), "alice@example.com".to_string());
    input.insert("age".to_string(), "25".to_string());
    
    match validate_user_input(&input) {
        Ok(validated) => println!("Validation successful: {:?}", validated),
        Err(e) => println!("Validation failed: {}", e),
    }
    
    // 测试错误累积模式
    let mut bad_input = HashMap::new();
    bad_input.insert("name".to_string(), "".to_string());
    bad_input.insert("email".to_string(), "invalid-email".to_string());
    bad_input.insert("age".to_string(), "200".to_string());
    
    match validate_user_comprehensive(&bad_input) {
        Ok(validated) => println!("Comprehensive validation successful: {:?}", validated),
        Err(e) => println!("Comprehensive validation failed:\n{}", e),
    }
    
    // 测试重试模式
    let start = Instant::now();
    match retry_operation(
        unreliable_network_call,
        5,
        Duration::from_millis(100),
    ) {
        Ok(response) => println!("Network call succeeded: {}", response),
        Err(e) => println!("Network call failed after retries: {}", e),
    }
    println!("Total time: {:?}", start.elapsed());
}
```

## 6. 实践示例

### 6.1 文件处理应用

```rust
use std::fs::File;
use std::io::{self, BufRead, BufReader, Write};
use std::path::Path;

#[derive(Debug)]
enum FileProcessError {
    Io(io::Error),
    InvalidFormat(String),
    ProcessingError(String),
}

impl fmt::Display for FileProcessError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FileProcessError::Io(err) => write!(f, "IO error: {}", err),
            FileProcessError::InvalidFormat(msg) => write!(f, "Invalid format: {}", msg),
            FileProcessError::ProcessingError(msg) => write!(f, "Processing error: {}", msg),
        }
    }
}

impl std::error::Error for FileProcessError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            FileProcessError::Io(err) => Some(err),
            _ => None,
        }
    }
}

impl From<io::Error> for FileProcessError {
    fn from(err: io::Error) -> Self {
        FileProcessError::Io(err)
    }
}

struct CsvProcessor {
    input_path: String,
    output_path: String,
}

impl CsvProcessor {
    fn new(input_path: String, output_path: String) -> Self {
        CsvProcessor { input_path, output_path }
    }
    
    fn process(&self) -> Result<ProcessingStats, FileProcessError> {
        let input_file = File::open(&self.input_path)?;
        let reader = BufReader::new(input_file);
        
        let mut output_file = File::create(&self.output_path)?;
        let mut stats = ProcessingStats::new();
        
        let mut lines = reader.lines();
        
        // 处理标题行
        if let Some(header_line) = lines.next() {
            let header = header_line?;
            self.validate_header(&header)?;
            writeln!(output_file, "{},processed_at", header)?;
            stats.lines_processed += 1;
        } else {
            return Err(FileProcessError::InvalidFormat("Empty file".to_string()));
        }
        
        // 处理数据行
        for (line_num, line_result) in lines.enumerate() {
            let line = line_result?;
            
            match self.process_line(&line, line_num + 2) {
                Ok(processed_line) => {
                    writeln!(output_file, "{},{}", processed_line, chrono::Utc::now().format("%Y-%m-%d %H:%M:%S"))?;
                    stats.lines_processed += 1;
                }
                Err(e) => {
                    eprintln!("Warning: Skipping line {}: {}", line_num + 2, e);
                    stats.lines_skipped += 1;
                }
            }
        }
        
        Ok(stats)
    }
    
    fn validate_header(&self, header: &str) -> Result<(), FileProcessError> {
        let expected_columns = vec!["id", "name", "email", "age"];
        let actual_columns: Vec<&str> = header.split(',').map(|s| s.trim()).collect();
        
        for expected in &expected_columns {
            if !actual_columns.contains(expected) {
                return Err(FileProcessError::InvalidFormat(
                    format!("Missing required column: {}", expected)
                ));
            }
        }
        
        Ok(())
    }
    
    fn process_line(&self, line: &str, line_num: usize) -> Result<String, FileProcessError> {
        let fields: Vec<&str> = line.split(',').map(|s| s.trim()).collect();
        
        if fields.len() < 4 {
            return Err(FileProcessError::InvalidFormat(
                format!("Line {} has insufficient fields", line_num)
            ));
        }
        
        // 验证ID
        let id: u32 = fields[0].parse().map_err(|_| {
            FileProcessError::InvalidFormat(format!("Invalid ID at line {}: {}", line_num, fields[0]))
        })?;
        
        // 验证姓名
        let name = fields[1];
        if name.is_empty() {
            return Err(FileProcessError::InvalidFormat(
                format!("Empty name at line {}", line_num)
            ));
        }
        
        // 验证邮箱
        let email = fields[2];
        if !email.contains('@') {
            return Err(FileProcessError::InvalidFormat(
                format!("Invalid email at line {}: {}", line_num, email)
            ));
        }
        
        // 验证年龄
        let age: u32 = fields[3].parse().map_err(|_| {
            FileProcessError::InvalidFormat(format!("Invalid age at line {}: {}", line_num, fields[3]))
        })?;
        
        if age > 150 {
            return Err(FileProcessError::ProcessingError(
                format!("Unrealistic age at line {}: {}", line_num, age)
            ));
        }
        
        // 处理数据（例如：标准化邮箱格式）
        let processed_email = email.to_lowercase();
        
        Ok(format!("{},{},{},{}", id, name, processed_email, age))
    }
}

#[derive(Debug)]
struct ProcessingStats {
    lines_processed: usize,
    lines_skipped: usize,
}

impl ProcessingStats {
    fn new() -> Self {
        ProcessingStats {
            lines_processed: 0,
            lines_skipped: 0,
        }
    }
    
    fn total_lines(&self) -> usize {
        self.lines_processed + self.lines_skipped
    }
    
    fn success_rate(&self) -> f64 {
        if self.total_lines() == 0 {
            0.0
        } else {
            self.lines_processed as f64 / self.total_lines() as f64 * 100.0
        }
    }
}

fn main() {
    let processor = CsvProcessor::new(
        "input.csv".to_string(),
        "output.csv".to_string(),
    );
    
    match processor.process() {
        Ok(stats) => {
            println!("Processing completed successfully!");
            println!("  Lines processed: {}", stats.lines_processed);
            println!("  Lines skipped: {}", stats.lines_skipped);
            println!("  Success rate: {:.1}%", stats.success_rate());
        }
        Err(e) => {
            eprintln!("Processing failed: {}", e);
            
            // 打印错误链
            let mut source = e.source();
            while let Some(err) = source {
                eprintln!("  Caused by: {}", err);
                source = err.source();
            }
            
            std::process::exit(1);
        }
    }
}
```

## 7. 练习题

### 练习1：基础错误处理
实现一个计算器，能够处理除零错误和无效输入：

```rust
#[derive(Debug)]
enum CalculatorError {
    DivisionByZero,
    InvalidOperation(String),
    ParseError(String),
}

impl fmt::Display for CalculatorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CalculatorError::DivisionByZero => write!(f, "Cannot divide by zero"),
            CalculatorError::InvalidOperation(op) => write!(f, "Invalid operation: {}", op),
            CalculatorError::ParseError(input) => write!(f, "Cannot parse input: {}", input),
        }
    }
}

impl std::error::Error for CalculatorError {}

struct Calculator;

impl Calculator {
    fn calculate(expression: &str) -> Result<f64, CalculatorError> {
        // 实现基本的四则运算解析和计算
        // 支持格式："10 + 5", "20 - 3", "6 * 7", "15 / 3"
        
        let parts: Vec<&str> = expression.split_whitespace().collect();
        if parts.len() != 3 {
            return Err(CalculatorError::InvalidOperation(expression.to_string()));
        }
        
        let left: f64 = parts[0].parse()
            .map_err(|_| CalculatorError::ParseError(parts[0].to_string()))?;
        let operator = parts[1];
        let right: f64 = parts[2].parse()
            .map_err(|_| CalculatorError::ParseError(parts[2].to_string()))?;
        
        match operator {
            "+" => Ok(left + right),
            "-" => Ok(left - right),
            "*" => Ok(left * right),
            "/" => {
                if right == 0.0 {
                    Err(CalculatorError::DivisionByZero)
                } else {
                    Ok(left / right)
                }
            }
            _ => Err(CalculatorError::InvalidOperation(operator.to_string())),
        }
    }
}

// 测试代码
fn test_calculator() {
    let test_cases = vec![
        "10 + 5",
        "20 - 3", 
        "6 * 7",
        "15 / 3",
        "10 / 0",  // 除零错误
        "abc + 5",  // 解析错误
        "10 % 3",  // 无效操作
        "10 +",  // 格式错误
    ];
    
    for expression in test_cases {
        match Calculator::calculate(expression) {
            Ok(result) => println!("{} = {}", expression, result),
            Err(e) => println!("{} -> Error: {}", expression, e),
        }
    }
}
```

### 练习2：文件操作错误处理
实现一个配置文件管理器：

```rust
use std::collections::HashMap;
use std::fs;
use std::path::Path;

#[derive(Debug)]
enum ConfigError {
    FileNotFound(String),
    ParseError { line: usize, message: String },
    ValidationError(String),
    IoError(std::io::Error),
}

impl fmt::Display for ConfigError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConfigError::FileNotFound(path) => write!(f, "Config file not found: {}", path),
            ConfigError::ParseError { line, message } => {
                write!(f, "Parse error at line {}: {}", line, message)
            }
            ConfigError::ValidationError(msg) => write!(f, "Validation error: {}", msg),
            ConfigError::IoError(err) => write!(f, "IO error: {}", err),
        }
    }
}

impl std::error::Error for ConfigError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            ConfigError::IoError(err) => Some(err),
            _ => None,
        }
    }
}

impl From<std::io::Error> for ConfigError {
    fn from(err: std::io::Error) -> Self {
        ConfigError::IoError(err)
    }
}

struct ConfigManager {
    config: HashMap<String, String>,
}

impl ConfigManager {
    fn new() -> Self {
        ConfigManager {
            config: HashMap::new(),
        }
    }
    
    fn load_from_file<P: AsRef<Path>>(path: P) -> Result<Self, ConfigError> {
        let path_str = path.as_ref().to_string_lossy().to_string();
        
        if !path.as_ref().exists() {
            return Err(ConfigError::FileNotFound(path_str));
        }
        
        let content = fs::read_to_string(path)?;
        let mut config = HashMap::new();
        
        for (line_num, line) in content.lines().enumerate() {
            let line = line.trim();
            
            // 跳过空行和注释
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            
            // 解析键值对
            if let Some(eq_pos) = line.find('=') {
                let key = line[..eq_pos].trim().to_string();
                let value = line[eq_pos + 1..].trim().to_string();
                
                if key.is_empty() {
                    return Err(ConfigError::ParseError {
                        line: line_num + 1,
                        message: "Empty key".to_string(),
                    });
                }
                
                config.insert(key, value);
            } else {
                return Err(ConfigError::ParseError {
                    line: line_num + 1,
                    message: "Missing '=' separator".to_string(),
                });
            }
        }
        
        let mut manager = ConfigManager { config };
        manager.validate()?;
        Ok(manager)
    }
    
    fn validate(&self) -> Result<(), ConfigError> {
        // 检查必需的配置项
        let required_keys = vec!["database_url", "port", "log_level"];
        
        for key in required_keys {
            if !self.config.contains_key(key) {
                return Err(ConfigError::ValidationError(
                    format!("Missing required configuration: {}", key)
                ));
            }
        }
        
        // 验证端口号
        if let Some(port_str) = self.config.get("port") {
            let port: u16 = port_str.parse().map_err(|_| {
                ConfigError::ValidationError(format!("Invalid port number: {}", port_str))
            })?;
            
            if port < 1024 {
                return Err(ConfigError::ValidationError(
                    "Port number must be >= 1024".to_string()
                ));
            }
        }
        
        // 验证日志级别
        if let Some(log_level) = self.config.get("log_level") {
            let valid_levels = vec!["debug", "info", "warn", "error"];
            if !valid_levels.contains(&log_level.as_str()) {
                return Err(ConfigError::ValidationError(
                    format!("Invalid log level: {}. Must be one of: {:?}", log_level, valid_levels)
                ));
            }
        }
        
        Ok(())
    }
    
    fn get(&self, key: &str) -> Option<&String> {
        self.config.get(key)
    }
    
    fn save_to_file<P: AsRef<Path>>(&self, path: P) -> Result<(), ConfigError> {
        let mut content = String::new();
        content.push_str("# Configuration file\n");
        content.push_str("# Generated automatically\n\n");
        
        for (key, value) in &self.config {
            content.push_str(&format!("{}={}\n", key, value));
        }
        
        fs::write(path, content)?;
        Ok(())
    }
}

// 测试代码
fn test_config_manager() {
    // 测试加载不存在的文件
    match ConfigManager::load_from_file("nonexistent.conf") {
        Ok(_) => println!("Unexpected success"),
        Err(e) => println!("Expected error: {}", e),
    }
    
    // 创建测试配置文件
    let test_config = r#"
# Test configuration
database_url=sqlite://test.db
port=8080
log_level=info
debug=true
"#;
    
    fs::write("test.conf", test_config).unwrap();
    
    // 测试加载有效配置
    match ConfigManager::load_from_file("test.conf") {
        Ok(config) => {
            println!("Config loaded successfully");
            println!("Database URL: {:?}", config.get("database_url"));
            println!("Port: {:?}", config.get("port"));
            println!("Log Level: {:?}", config.get("log_level"));
            
            // 测试保存配置
            if let Err(e) = config.save_to_file("output.conf") {
                println!("Failed to save config: {}", e);
            } else {
                println!("Config saved successfully");
            }
        }
        Err(e) => println!("Failed to load config: {}", e),
    }
    
    // 清理测试文件
    let _ = fs::remove_file("test.conf");
    let _ = fs::remove_file("output.conf");
}
```

### 练习3：网络请求错误处理
实现一个简单的HTTP客户端（模拟）：

```rust
use std::time::Duration;
use std::thread;

#[derive(Debug)]
enum HttpError {
    NetworkError(String),
    TimeoutError,
    InvalidUrl(String),
    HttpStatus { code: u16, message: String },
    ParseError(String),
}

impl fmt::Display for HttpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HttpError::NetworkError(msg) => write!(f, "Network error: {}", msg),
            HttpError::TimeoutError => write!(f, "Request timeout"),
            HttpError::InvalidUrl(url) => write!(f, "Invalid URL: {}", url),
            HttpError::HttpStatus { code, message } => {
                write!(f, "HTTP {} error: {}", code, message)
            }
            HttpError::ParseError(msg) => write!(f, "Parse error: {}", msg),
        }
    }
}

impl std::error::Error for HttpError {}

#[derive(Debug)]
struct HttpResponse {
    status_code: u16,
    headers: HashMap<String, String>,
    body: String,
}

struct HttpClient {
    timeout: Duration,
    max_retries: usize,
}

impl HttpClient {
    fn new() -> Self {
        HttpClient {
            timeout: Duration::from_secs(30),
            max_retries: 3,
        }
    }
    
    fn with_timeout(mut self, timeout: Duration) -> Self {
        self.timeout = timeout;
        self
    }
    
    fn with_max_retries(mut self, max_retries: usize) -> Self {
        self.max_retries = max_retries;
        self
    }
    
    fn get(&self, url: &str) -> Result<HttpResponse, HttpError> {
        self.validate_url(url)?;
        
        for attempt in 1..=self.max_retries {
            match self.make_request(url) {
                Ok(response) => return Ok(response),
                Err(HttpError::NetworkError(_)) if attempt < self.max_retries => {
                    println!("Attempt {} failed, retrying...", attempt);
                    thread::sleep(Duration::from_millis(1000 * attempt as u64));
                    continue;
                }
                Err(e) => return Err(e),
            }
        }
        
        Err(HttpError::NetworkError("Max retries exceeded".to_string()))
    }
    
    fn validate_url(&self, url: &str) -> Result<(), HttpError> {
        if url.is_empty() {
            return Err(HttpError::InvalidUrl("Empty URL".to_string()));
        }
        
        if !url.starts_with("http://") && !url.starts_with("https://") {
            return Err(HttpError::InvalidUrl(
                format!("URL must start with http:// or https://: {}", url)
            ));
        }
        
        Ok(())
    }
    
    fn make_request(&self, url: &str) -> Result<HttpResponse, HttpError> {
        // 模拟网络请求
        use rand::Rng;
        let mut rng = rand::thread_rng();
        
        // 模拟网络延迟
        let delay = Duration::from_millis(rng.gen_range(100..=2000));
        thread::sleep(delay);
        
        // 模拟超时
        if delay > self.timeout {
            return Err(HttpError::TimeoutError);
        }
        
        // 模拟网络错误
        if rng.gen_bool(0.2) {  // 20% 网络错误概率
            return Err(HttpError::NetworkError("Connection refused".to_string()));
        }
        
        // 模拟HTTP状态码
        let status_code = if rng.gen_bool(0.8) {  // 80% 成功概率
            200
        } else {
            match rng.gen_range(1..=4) {
                1 => 404,
                2 => 500,
                3 => 503,
                _ => 429,
            }
        };
        
        if status_code != 200 {
            let message = match status_code {
                404 => "Not Found",
                500 => "Internal Server Error",
                503 => "Service Unavailable",
                429 => "Too Many Requests",
                _ => "Unknown Error",
            };
            
            return Err(HttpError::HttpStatus {
                code: status_code,
                message: message.to_string(),
            });
        }
        
        // 构造成功响应
        let mut headers = HashMap::new();
        headers.insert("content-type".to_string(), "application/json".to_string());
        headers.insert("content-length".to_string(), "100".to_string());
        
        Ok(HttpResponse {
            status_code: 200,
            headers,
            body: format!(r#"{{"url": "{}", "timestamp": "2024-01-01T00:00:00Z"}}"#, url),
        })
    }
    
    fn parse_json_response(&self, response: &HttpResponse) -> Result<serde_json::Value, HttpError> {
        serde_json::from_str(&response.body)
            .map_err(|e| HttpError::ParseError(format!("JSON parse error: {}", e)))
    }
}

// 测试代码
fn test_http_client() {
    let client = HttpClient::new()
        .with_timeout(Duration::from_millis(1500))
        .with_max_retries(3);
    
    let test_urls = vec![
        "https://api.example.com/users",
        "http://invalid-domain.test/data",
        "https://slow-api.example.com/slow",
        "not-a-url",
        "",
    ];
    
    for url in test_urls {
        println!("\nTesting URL: {}", url);
        
        match client.get(url) {
            Ok(response) => {
                println!("Success! Status: {}", response.status_code);
                println!("Headers: {:?}", response.headers);
                println!("Body: {}", response.body);
                
                // 尝试解析JSON
                match client.parse_json_response(&response) {
                    Ok(json) => println!("Parsed JSON: {}", json),
                    Err(e) => println!("JSON parse failed: {}", e),
                }
            }
            Err(e) => {
                println!("Request failed: {}", e);
                
                // 根据错误类型提供不同的处理建议
                match e {
                    HttpError::NetworkError(_) => {
                        println!("  Suggestion: Check network connection and try again");
                    }
                    HttpError::TimeoutError => {
                        println!("  Suggestion: Increase timeout or try again later");
                    }
                    HttpError::InvalidUrl(_) => {
                        println!("  Suggestion: Check URL format");
                    }
                    HttpError::HttpStatus { code, .. } => {
                        match code {
                            404 => println!("  Suggestion: Check if the resource exists"),
                            500 => println!("  Suggestion: Server error, try again later"),
                            503 => println!("  Suggestion: Service unavailable, try again later"),
                            429 => println!("  Suggestion: Rate limited, wait before retrying"),
                            _ => println!("  Suggestion: Check server status"),
                        }
                    }
                    HttpError::ParseError(_) => {
                        println!("  Suggestion: Check response format");
                    }
                }
            }
        }
    }
}
```

## 总结

通过本章的学习，你应该掌握了：

### 核心概念
- `Option<T>`和`Result<T, E>`类型的使用
- `?`操作符的工作原理和应用场景
- 自定义错误类型的设计和实现
- 错误转换和传播机制

### 实用技能
- 如何选择合适的错误处理策略
- 错误信息的设计和用户体验考虑
- 复杂应用中的错误处理架构
- 测试和调试错误处理代码

### 最佳实践
- 使用类型系统表达可能的失败
- 提供有意义的错误信息
- 区分可恢复和不可恢复的错误
- 合理使用panic!和Result
- 错误处理的性能考虑

### 高级特性
- 错误链和错误源追踪
- 自定义错误类型的From实现
- 错误处理模式（早期返回、错误累积、重试等）
- 与外部库的错误类型集成

Rust的错误处理系统虽然在开始时可能显得复杂，但它提供了强大的工具来构建健壮和可靠的应用程序。通过强制处理错误情况，Rust帮助开发者编写更安全、更可预测的代码。
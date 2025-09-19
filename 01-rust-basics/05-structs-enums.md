# 结构体与枚举

## 概述

结构体（struct）和枚举（enum）是Rust中定义自定义数据类型的两种主要方式。结构体允许我们将相关的数据组合在一起，而枚举让我们可以定义一个类型，它可以是几种可能的变体之一。这两种类型是构建复杂数据结构和实现类型安全的基础。

## 学习目标

- 掌握结构体的定义和使用方法
- 理解方法和关联函数的概念
- 学会使用枚举类型进行数据建模
- 深入理解Option和Result枚举
- 掌握模式匹配在结构体和枚举中的应用
- 学会设计和实现复杂的数据结构

## 1. 结构体基础

### 1.1 结构体定义

```rust
// 基本结构体定义
struct User {
    username: String,
    email: String,
    sign_in_count: u64,
    active: bool,
}

// 元组结构体
struct Color(i32, i32, i32);
struct Point(i32, i32, i32);

// 单元结构体（没有字段）
struct AlwaysEqual;

fn main() {
    // 创建结构体实例
    let user1 = User {
        email: String::from("someone@example.com"),
        username: String::from("someusername123"),
        active: true,
        sign_in_count: 1,
    };
    
    // 元组结构体实例
    let black = Color(0, 0, 0);
    let origin = Point(0, 0, 0);
    
    // 单元结构体实例
    let subject = AlwaysEqual;
}
```

### 1.2 访问和修改结构体字段

```rust
struct Rectangle {
    width: u32,
    height: u32,
}

fn main() {
    // 不可变结构体
    let rect1 = Rectangle {
        width: 30,
        height: 50,
    };
    
    println!("Width: {}, Height: {}", rect1.width, rect1.height);
    
    // 可变结构体
    let mut rect2 = Rectangle {
        width: 20,
        height: 40,
    };
    
    rect2.width = 25;  // 修改字段值
    println!("New width: {}", rect2.width);
}
```

### 1.3 结构体更新语法

```rust
struct User {
    username: String,
    email: String,
    sign_in_count: u64,
    active: bool,
}

fn main() {
    let user1 = User {
        email: String::from("someone@example.com"),
        username: String::from("someusername123"),
        active: true,
        sign_in_count: 1,
    };
    
    // 使用结构体更新语法创建新实例
    let user2 = User {
        email: String::from("another@example.com"),
        ..user1  // 其余字段从user1复制
    };
    
    // 注意：user1的username被移动到user2，user1不再完全有效
    println!("User2 username: {}", user2.username);
    // println!("User1 username: {}", user1.username); // 错误！
    println!("User1 active: {}", user1.active); // 但基本类型字段仍可访问
}
```

### 1.4 结构体作为函数参数和返回值

```rust
struct Rectangle {
    width: u32,
    height: u32,
}

// 接受结构体引用作为参数
fn area(rectangle: &Rectangle) -> u32 {
    rectangle.width * rectangle.height
}

// 接受结构体所有权
fn take_ownership(rectangle: Rectangle) -> u32 {
    rectangle.width * rectangle.height
}

// 返回结构体
fn create_square(size: u32) -> Rectangle {
    Rectangle {
        width: size,
        height: size,
    }
}

fn main() {
    let rect = Rectangle {
        width: 30,
        height: 50,
    };
    
    println!("Area: {}", area(&rect));  // 借用
    println!("Still can use rect: {}", rect.width);
    
    let square = create_square(10);
    println!("Square area: {}", area(&square));
}
```

## 2. 方法和关联函数

### 2.1 方法定义

```rust
struct Rectangle {
    width: u32,
    height: u32,
}

impl Rectangle {
    // 方法：第一个参数是&self
    fn area(&self) -> u32 {
        self.width * self.height
    }
    
    // 可变方法：第一个参数是&mut self
    fn double_size(&mut self) {
        self.width *= 2;
        self.height *= 2;
    }
    
    // 消费方法：第一个参数是self
    fn into_square(self) -> Rectangle {
        let size = std::cmp::max(self.width, self.height);
        Rectangle {
            width: size,
            height: size,
        }
    }
    
    // 方法可以有多个参数
    fn can_hold(&self, other: &Rectangle) -> bool {
        self.width > other.width && self.height > other.height
    }
}

fn main() {
    let mut rect = Rectangle {
        width: 30,
        height: 50,
    };
    
    println!("Area: {}", rect.area());
    
    rect.double_size();
    println!("New dimensions: {}x{}", rect.width, rect.height);
    
    let rect2 = Rectangle {
        width: 10,
        height: 40,
    };
    
    println!("Can rect hold rect2? {}", rect.can_hold(&rect2));
    
    // 消费方法会获取所有权
    let square = rect.into_square();
    // println!("{}", rect.width); // 错误！rect已被移动
    println!("Square size: {}x{}", square.width, square.height);
}
```

### 2.2 关联函数

```rust
struct Rectangle {
    width: u32,
    height: u32,
}

impl Rectangle {
    // 关联函数：没有self参数，通常用作构造函数
    fn new(width: u32, height: u32) -> Rectangle {
        Rectangle { width, height }
    }
    
    fn square(size: u32) -> Rectangle {
        Rectangle {
            width: size,
            height: size,
        }
    }
    
    // 常量关联函数
    fn default() -> Rectangle {
        Rectangle {
            width: 1,
            height: 1,
        }
    }
}

fn main() {
    // 使用::语法调用关联函数
    let rect = Rectangle::new(30, 50);
    let square = Rectangle::square(25);
    let default_rect = Rectangle::default();
    
    println!("Rectangle: {}x{}", rect.width, rect.height);
    println!("Square: {}x{}", square.width, square.height);
    println!("Default: {}x{}", default_rect.width, default_rect.height);
}
```

### 2.3 多个impl块

```rust
struct Rectangle {
    width: u32,
    height: u32,
}

// 第一个impl块
impl Rectangle {
    fn area(&self) -> u32 {
        self.width * self.height
    }
}

// 第二个impl块
impl Rectangle {
    fn perimeter(&self) -> u32 {
        2 * (self.width + self.height)
    }
}

// 为不同的trait实现可以分开
impl std::fmt::Display for Rectangle {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "Rectangle({}x{})", self.width, self.height)
    }
}

fn main() {
    let rect = Rectangle {
        width: 30,
        height: 50,
    };
    
    println!("Area: {}", rect.area());
    println!("Perimeter: {}", rect.perimeter());
    println!("Rectangle: {}", rect);  // 使用Display trait
}
```

## 3. 枚举基础

### 3.1 枚举定义

```rust
// 简单枚举
enum Direction {
    North,
    South,
    East,
    West,
}

// 带数据的枚举
enum Message {
    Quit,                       // 无数据
    Move { x: i32, y: i32 },   // 命名字段
    Write(String),              // 单个值
    ChangeColor(i32, i32, i32), // 元组
}

// 枚举变体可以有不同类型的数据
enum IpAddr {
    V4(u8, u8, u8, u8),
    V6(String),
}

fn main() {
    let north = Direction::North;
    
    let msg1 = Message::Quit;
    let msg2 = Message::Move { x: 10, y: 20 };
    let msg3 = Message::Write(String::from("Hello"));
    let msg4 = Message::ChangeColor(255, 0, 0);
    
    let home = IpAddr::V4(127, 0, 0, 1);
    let loopback = IpAddr::V6(String::from("::1"));
}
```

### 3.2 枚举方法

```rust
enum Message {
    Quit,
    Move { x: i32, y: i32 },
    Write(String),
    ChangeColor(i32, i32, i32),
}

impl Message {
    fn process(&self) {
        match self {
            Message::Quit => println!("Quitting..."),
            Message::Move { x, y } => println!("Moving to ({}, {})", x, y),
            Message::Write(text) => println!("Writing: {}", text),
            Message::ChangeColor(r, g, b) => {
                println!("Changing color to RGB({}, {}, {})", r, g, b)
            }
        }
    }
    
    fn is_quit(&self) -> bool {
        matches!(self, Message::Quit)
    }
}

fn main() {
    let messages = vec![
        Message::Quit,
        Message::Move { x: 10, y: 20 },
        Message::Write(String::from("Hello, World!")),
        Message::ChangeColor(255, 0, 0),
    ];
    
    for message in &messages {
        message.process();
        if message.is_quit() {
            println!("Found quit message!");
        }
    }
}
```

### 3.3 C风格枚举

```rust
// 指定判别值
enum HttpStatus {
    Ok = 200,
    NotFound = 404,
    InternalServerError = 500,
}

// 自动递增
enum Priority {
    Low,      // 0
    Medium,   // 1
    High,     // 2
    Critical, // 3
}

fn main() {
    let status = HttpStatus::Ok;
    let priority = Priority::High;
    
    println!("Status code: {}", status as i32);
    println!("Priority level: {}", priority as i32);
}
```

## 4. Option枚举详解

### 4.1 Option基础

```rust
fn main() {
    // Option<T>是标准库中定义的枚举
    let some_number = Some(5);
    let some_string = Some("a string");
    let absent_number: Option<i32> = None;
    
    // 使用match处理Option
    match some_number {
        Some(value) => println!("Got a value: {}", value),
        None => println!("No value"),
    }
    
    // 使用if let简化
    if let Some(value) = some_string {
        println!("String value: {}", value);
    }
}
```

### 4.2 Option的常用方法

```rust
fn main() {
    let x = Some(5);
    let y: Option<i32> = None;
    
    // is_some() 和 is_none()
    println!("x is some: {}", x.is_some());
    println!("y is none: {}", y.is_none());
    
    // unwrap() - 危险！如果是None会panic
    println!("x unwrapped: {}", x.unwrap());
    // println!("y unwrapped: {}", y.unwrap()); // 会panic!
    
    // unwrap_or() - 提供默认值
    println!("y with default: {}", y.unwrap_or(0));
    
    // unwrap_or_else() - 使用闭包计算默认值
    println!("y with computed default: {}", y.unwrap_or_else(|| 42));
    
    // expect() - 带自定义错误消息的unwrap
    println!("x expected: {}", x.expect("x should have a value"));
    
    // map() - 转换Some中的值
    let doubled = x.map(|val| val * 2);
    println!("Doubled: {:?}", doubled);
    
    // and_then() - 链式操作
    let result = x.and_then(|val| {
        if val > 0 {
            Some(val * 2)
        } else {
            None
        }
    });
    println!("And then result: {:?}", result);
    
    // filter() - 条件过滤
    let filtered = x.filter(|&val| val > 3);
    println!("Filtered: {:?}", filtered);
}
```

### 4.3 Option在实际应用中的使用

```rust
struct Person {
    name: String,
    age: Option<u32>,  // 年龄可能未知
    email: Option<String>,  // 邮箱可能为空
}

impl Person {
    fn new(name: String) -> Person {
        Person {
            name,
            age: None,
            email: None,
        }
    }
    
    fn with_age(mut self, age: u32) -> Person {
        self.age = Some(age);
        self
    }
    
    fn with_email(mut self, email: String) -> Person {
        self.email = Some(email);
        self
    }
    
    fn display_info(&self) {
        println!("Name: {}", self.name);
        
        match self.age {
            Some(age) => println!("Age: {}", age),
            None => println!("Age: Unknown"),
        }
        
        if let Some(email) = &self.email {
            println!("Email: {}", email);
        } else {
            println!("Email: Not provided");
        }
    }
    
    fn is_adult(&self) -> Option<bool> {
        self.age.map(|age| age >= 18)
    }
}

fn main() {
    let person1 = Person::new("Alice".to_string())
        .with_age(25)
        .with_email("alice@example.com".to_string());
    
    let person2 = Person::new("Bob".to_string())
        .with_age(16);
    
    person1.display_info();
    println!("Is adult: {:?}", person1.is_adult());
    
    person2.display_info();
    println!("Is adult: {:?}", person2.is_adult());
}
```

## 5. Result枚举详解

### 5.1 Result基础

```rust
// Result<T, E>用于可能失败的操作
fn divide(dividend: f64, divisor: f64) -> Result<f64, String> {
    if divisor == 0.0 {
        Err("Division by zero".to_string())
    } else {
        Ok(dividend / divisor)
    }
}

fn main() {
    let result1 = divide(10.0, 2.0);
    let result2 = divide(10.0, 0.0);
    
    match result1 {
        Ok(value) => println!("Result: {}", value),
        Err(error) => println!("Error: {}", error),
    }
    
    match result2 {
        Ok(value) => println!("Result: {}", value),
        Err(error) => println!("Error: {}", error),
    }
}
```

### 5.2 Result的常用方法

```rust
fn parse_number(s: &str) -> Result<i32, std::num::ParseIntError> {
    s.parse::<i32>()
}

fn main() {
    let good_result = parse_number("42");
    let bad_result = parse_number("abc");
    
    // is_ok() 和 is_err()
    println!("Good result is ok: {}", good_result.is_ok());
    println!("Bad result is err: {}", bad_result.is_err());
    
    // unwrap() - 危险！如果是Err会panic
    println!("Good result: {}", good_result.unwrap());
    // println!("Bad result: {}", bad_result.unwrap()); // 会panic!
    
    // unwrap_or() - 提供默认值
    println!("Bad result with default: {}", bad_result.unwrap_or(0));
    
    // expect() - 带自定义错误消息
    println!("Good result expected: {}", good_result.expect("Should be a number"));
    
    // map() - 转换Ok中的值
    let doubled = good_result.map(|val| val * 2);
    println!("Doubled: {:?}", doubled);
    
    // map_err() - 转换Err中的值
    let mapped_err = bad_result.map_err(|e| format!("Parse error: {}", e));
    println!("Mapped error: {:?}", mapped_err);
    
    // and_then() - 链式操作
    let chained = good_result.and_then(|val| {
        if val > 0 {
            Ok(val * 2)
        } else {
            Err("Value must be positive".parse().unwrap())
        }
    });
    println!("Chained result: {:?}", chained);
}
```

### 5.3 错误传播

```rust
use std::fs::File;
use std::io::{self, Read};

// 使用?操作符进行错误传播
fn read_file_contents(filename: &str) -> Result<String, io::Error> {
    let mut file = File::open(filename)?;  // 如果失败，直接返回错误
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;   // 如果失败，直接返回错误
    Ok(contents)
}

// 不使用?操作符的版本（更冗长）
fn read_file_contents_verbose(filename: &str) -> Result<String, io::Error> {
    let file = File::open(filename);
    let mut file = match file {
        Ok(file) => file,
        Err(error) => return Err(error),
    };
    
    let mut contents = String::new();
    match file.read_to_string(&mut contents) {
        Ok(_) => Ok(contents),
        Err(error) => Err(error),
    }
}

fn main() {
    match read_file_contents("example.txt") {
        Ok(contents) => println!("File contents: {}", contents),
        Err(error) => println!("Error reading file: {}", error),
    }
}
```

### 5.4 自定义错误类型

```rust
use std::fmt;

#[derive(Debug)]
enum MathError {
    DivisionByZero,
    NegativeSquareRoot,
    InvalidInput(String),
}

impl fmt::Display for MathError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            MathError::DivisionByZero => write!(f, "Division by zero"),
            MathError::NegativeSquareRoot => write!(f, "Square root of negative number"),
            MathError::InvalidInput(msg) => write!(f, "Invalid input: {}", msg),
        }
    }
}

impl std::error::Error for MathError {}

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

fn complex_calculation(a: f64, b: f64) -> Result<f64, MathError> {
    let division_result = safe_divide(a, b)?;
    let sqrt_result = safe_sqrt(division_result)?;
    Ok(sqrt_result)
}

fn main() {
    let results = vec![
        complex_calculation(16.0, 4.0),  // 应该成功
        complex_calculation(16.0, 0.0),  // 除零错误
        complex_calculation(-16.0, 4.0), // 负数开方错误
    ];
    
    for (i, result) in results.iter().enumerate() {
        match result {
            Ok(value) => println!("Result {}: {}", i + 1, value),
            Err(error) => println!("Error {}: {}", i + 1, error),
        }
    }
}
```

## 6. 模式匹配

### 6.1 match表达式

```rust
enum Coin {
    Penny,
    Nickel,
    Dime,
    Quarter(String), // 带数据的变体
}

fn value_in_cents(coin: Coin) -> u8 {
    match coin {
        Coin::Penny => {
            println!("Lucky penny!");
            1
        }
        Coin::Nickel => 5,
        Coin::Dime => 10,
        Coin::Quarter(state) => {
            println!("State quarter from {}!", state);
            25
        }
    }
}

fn main() {
    let coin = Coin::Quarter("Alaska".to_string());
    println!("Value: {} cents", value_in_cents(coin));
}
```

### 6.2 if let表达式

```rust
fn main() {
    let some_value = Some(3);
    
    // 使用match
    match some_value {
        Some(3) => println!("three"),
        _ => (),
    }
    
    // 使用if let简化
    if let Some(3) = some_value {
        println!("three");
    }
    
    // if let with else
    let mut count = 0;
    let coin = Coin::Quarter("Alaska".to_string());
    
    if let Coin::Quarter(state) = coin {
        println!("State quarter from {}!", state);
    } else {
        count += 1;
    }
}
```

### 6.3 while let循环

```rust
fn main() {
    let mut stack = Vec::new();
    stack.push(1);
    stack.push(2);
    stack.push(3);
    
    // 使用while let处理Option
    while let Some(top) = stack.pop() {
        println!("Popped: {}", top);
    }
    
    println!("Stack is now empty");
}
```

### 6.4 复杂模式匹配

```rust
struct Point {
    x: i32,
    y: i32,
}

enum Message {
    Quit,
    Move { x: i32, y: i32 },
    Write(String),
    ChangeColor(i32, i32, i32),
}

fn main() {
    let point = Point { x: 0, y: 7 };
    
    // 解构结构体
    match point {
        Point { x, y: 0 } => println!("On the x axis at {}", x),
        Point { x: 0, y } => println!("On the y axis at {}", y),
        Point { x, y } => println!("On neither axis: ({}, {})", x, y),
    }
    
    // 解构枚举
    let msg = Message::ChangeColor(0, 160, 255);
    
    match msg {
        Message::Quit => println!("Quit"),
        Message::Move { x, y } => println!("Move to ({}, {})", x, y),
        Message::Write(text) => println!("Text message: {}", text),
        Message::ChangeColor(r, g, b) => {
            println!("Change color to red {}, green {}, blue {}", r, g, b)
        }
    }
    
    // 使用守卫
    let num = Some(4);
    
    match num {
        Some(x) if x < 5 => println!("less than five: {}", x),
        Some(x) => println!("{}", x),
        None => (),
    }
    
    // 绑定模式
    let msg = Message::Write("Hello".to_string());
    
    match msg {
        Message::Write(text) if text.len() > 10 => {
            println!("Long message: {}", text)
        }
        Message::Write(text) => println!("Short message: {}", text),
        _ => (),
    }
}
```

## 7. 实践示例

### 7.1 状态机实现

```rust
#[derive(Debug, PartialEq)]
enum State {
    Idle,
    Running,
    Paused,
    Stopped,
}

#[derive(Debug)]
enum Event {
    Start,
    Pause,
    Resume,
    Stop,
    Reset,
}

struct StateMachine {
    state: State,
}

impl StateMachine {
    fn new() -> Self {
        StateMachine { state: State::Idle }
    }
    
    fn handle_event(&mut self, event: Event) -> Result<(), String> {
        let new_state = match (&self.state, event) {
            (State::Idle, Event::Start) => State::Running,
            (State::Running, Event::Pause) => State::Paused,
            (State::Paused, Event::Resume) => State::Running,
            (State::Running, Event::Stop) => State::Stopped,
            (State::Paused, Event::Stop) => State::Stopped,
            (State::Stopped, Event::Reset) => State::Idle,
            (current_state, event) => {
                return Err(format!(
                    "Invalid transition: {:?} -> {:?}",
                    current_state, event
                ));
            }
        };
        
        println!("Transitioning from {:?} to {:?}", self.state, new_state);
        self.state = new_state;
        Ok(())
    }
    
    fn current_state(&self) -> &State {
        &self.state
    }
}

fn main() {
    let mut machine = StateMachine::new();
    
    let events = vec![
        Event::Start,
        Event::Pause,
        Event::Resume,
        Event::Stop,
        Event::Reset,
        Event::Start,  // 无效转换示例
    ];
    
    for event in events {
        match machine.handle_event(event) {
            Ok(()) => println!("Current state: {:?}", machine.current_state()),
            Err(error) => println!("Error: {}", error),
        }
    }
}
```

### 7.2 配置系统

```rust
use std::collections::HashMap;

#[derive(Debug, Clone)]
enum ConfigValue {
    String(String),
    Integer(i64),
    Float(f64),
    Boolean(bool),
    Array(Vec<ConfigValue>),
    Object(HashMap<String, ConfigValue>),
}

impl ConfigValue {
    fn as_string(&self) -> Option<&String> {
        match self {
            ConfigValue::String(s) => Some(s),
            _ => None,
        }
    }
    
    fn as_integer(&self) -> Option<i64> {
        match self {
            ConfigValue::Integer(i) => Some(*i),
            _ => None,
        }
    }
    
    fn as_boolean(&self) -> Option<bool> {
        match self {
            ConfigValue::Boolean(b) => Some(*b),
            _ => None,
        }
    }
    
    fn as_array(&self) -> Option<&Vec<ConfigValue>> {
        match self {
            ConfigValue::Array(arr) => Some(arr),
            _ => None,
        }
    }
}

struct Config {
    values: HashMap<String, ConfigValue>,
}

impl Config {
    fn new() -> Self {
        Config {
            values: HashMap::new(),
        }
    }
    
    fn set(&mut self, key: String, value: ConfigValue) {
        self.values.insert(key, value);
    }
    
    fn get(&self, key: &str) -> Option<&ConfigValue> {
        self.values.get(key)
    }
    
    fn get_string(&self, key: &str) -> Option<&String> {
        self.get(key)?.as_string()
    }
    
    fn get_integer(&self, key: &str) -> Option<i64> {
        self.get(key)?.as_integer()
    }
    
    fn get_boolean(&self, key: &str) -> Option<bool> {
        self.get(key)?.as_boolean()
    }
}

fn main() {
    let mut config = Config::new();
    
    config.set("app_name".to_string(), ConfigValue::String("MyApp".to_string()));
    config.set("port".to_string(), ConfigValue::Integer(8080));
    config.set("debug".to_string(), ConfigValue::Boolean(true));
    config.set("features".to_string(), ConfigValue::Array(vec![
        ConfigValue::String("auth".to_string()),
        ConfigValue::String("logging".to_string()),
    ]));
    
    // 读取配置
    if let Some(app_name) = config.get_string("app_name") {
        println!("App name: {}", app_name);
    }
    
    if let Some(port) = config.get_integer("port") {
        println!("Port: {}", port);
    }
    
    if let Some(debug) = config.get_boolean("debug") {
        println!("Debug mode: {}", debug);
    }
    
    if let Some(features) = config.get("features").and_then(|v| v.as_array()) {
        println!("Features: {:?}", features);
    }
}
```

## 8. 最佳实践

### 8.1 结构体设计原则

```rust
// 好的实践：使用构造函数
struct User {
    id: u64,
    username: String,
    email: String,
    created_at: std::time::SystemTime,
}

impl User {
    fn new(username: String, email: String) -> Result<Self, String> {
        if username.is_empty() {
            return Err("Username cannot be empty".to_string());
        }
        
        if !email.contains('@') {
            return Err("Invalid email format".to_string());
        }
        
        Ok(User {
            id: generate_id(),
            username,
            email,
            created_at: std::time::SystemTime::now(),
        })
    }
    
    // 提供安全的访问方法
    fn username(&self) -> &str {
        &self.username
    }
    
    fn email(&self) -> &str {
        &self.email
    }
}

fn generate_id() -> u64 {
    // 简化的ID生成
    42
}
```

### 8.2 枚举设计原则

```rust
// 好的实践：使用枚举表示状态和错误
#[derive(Debug, PartialEq)]
enum OrderStatus {
    Pending,
    Processing,
    Shipped { tracking_number: String },
    Delivered { delivered_at: std::time::SystemTime },
    Cancelled { reason: String },
}

#[derive(Debug)]
enum OrderError {
    InvalidItem,
    InsufficientStock,
    PaymentFailed,
    ShippingUnavailable,
}

struct Order {
    id: u64,
    items: Vec<String>,
    status: OrderStatus,
}

impl Order {
    fn new(id: u64, items: Vec<String>) -> Result<Self, OrderError> {
        if items.is_empty() {
            return Err(OrderError::InvalidItem);
        }
        
        Ok(Order {
            id,
            items,
            status: OrderStatus::Pending,
        })
    }
    
    fn ship(&mut self, tracking_number: String) -> Result<(), OrderError> {
        match self.status {
            OrderStatus::Processing => {
                self.status = OrderStatus::Shipped { tracking_number };
                Ok(())
            }
            _ => Err(OrderError::ShippingUnavailable),
        }
    }
    
    fn can_cancel(&self) -> bool {
        matches!(self.status, OrderStatus::Pending | OrderStatus::Processing)
    }
}
```

### 8.3 错误处理最佳实践

```rust
use std::fmt;

// 定义应用程序特定的错误类型
#[derive(Debug)]
enum AppError {
    Io(std::io::Error),
    Parse(std::num::ParseIntError),
    Custom(String),
}

impl fmt::Display for AppError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            AppError::Io(err) => write!(f, "IO error: {}", err),
            AppError::Parse(err) => write!(f, "Parse error: {}", err),
            AppError::Custom(msg) => write!(f, "Application error: {}", msg),
        }
    }
}

impl std::error::Error for AppError {}

// 实现From trait进行错误转换
impl From<std::io::Error> for AppError {
    fn from(error: std::io::Error) -> Self {
        AppError::Io(error)
    }
}

impl From<std::num::ParseIntError> for AppError {
    fn from(error: std::num::ParseIntError) -> Self {
        AppError::Parse(error)
    }
}

// 使用?操作符进行错误传播
fn process_file(filename: &str) -> Result<i32, AppError> {
    let contents = std::fs::read_to_string(filename)?;  // IO错误自动转换
    let number: i32 = contents.trim().parse()?;         // Parse错误自动转换
    
    if number < 0 {
        return Err(AppError::Custom("Number must be positive".to_string()));
    }
    
    Ok(number * 2)
}
```

## 9. 总结

结构体和枚举是Rust类型系统的核心：

1. **结构体**用于组合相关数据，支持方法和关联函数
2. **枚举**用于表示多种可能的值，特别适合状态建模
3. **Option**和**Result**是处理空值和错误的标准方式
4. **模式匹配**提供了强大而安全的数据解构能力
5. **方法系统**支持面向对象的编程模式
6. **错误处理**通过类型系统保证程序的健壮性

这些概念在嵌入式开发中特别重要，因为它们提供了零成本抽象和编译时安全保证。

## 练习题

1. 设计一个表示几何图形的枚举，包含圆形、矩形和三角形，并实现计算面积的方法
2. 创建一个配置管理系统，使用Result处理配置加载和验证错误
3. 实现一个简单的状态机，模拟交通信号灯的状态转换
4. 设计一个用户管理系统，使用Option处理可选字段，使用Result处理验证错误
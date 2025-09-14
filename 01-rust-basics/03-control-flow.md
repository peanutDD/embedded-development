# 控制流

## 概述

控制流是编程的基础，它决定了程序的执行路径。Rust提供了丰富的控制流结构，包括条件语句、循环和强大的模式匹配功能。

## 条件语句

### if表达式

在Rust中，`if`是一个表达式，这意味着它可以返回值：

```rust
fn main() {
    let number = 6;
    
    if number % 4 == 0 {
        println!("number能被4整除");
    } else if number % 3 == 0 {
        println!("number能被3整除");
    } else if number % 2 == 0 {
        println!("number能被2整除");
    } else {
        println!("number不能被4、3或2整除");
    }
}
```

### if作为表达式

```rust
fn main() {
    let condition = true;
    let number = if condition { 5 } else { 6 };
    
    println!("number的值是: {}", number);
    
    // 更复杂的例子
    let x = 5;
    let result = if x > 0 {
        "positive"
    } else if x < 0 {
        "negative"
    } else {
        "zero"
    };
    
    println!("x是: {}", result);
}
```

### 条件必须是bool类型

```rust
fn main() {
    let number = 3;
    
    // 正确：条件是bool类型
    if number != 0 {
        println!("number不为零");
    }
    
    // 错误：Rust不会自动将非bool类型转换为bool
    // if number {
    //     println!("number不为零");
    // }
}
```

## 循环

### loop循环

`loop`创建一个无限循环：

```rust
fn main() {
    let mut counter = 0;
    
    let result = loop {
        counter += 1;
        
        if counter == 10 {
            break counter * 2; // 从循环中返回值
        }
    };
    
    println!("结果是: {}", result);
}
```

### 循环标签

当有嵌套循环时，可以使用标签来指定break或continue作用于哪个循环：

```rust
fn main() {
    let mut count = 0;
    
    'counting_up: loop {
        println!("count = {}", count);
        let mut remaining = 10;
        
        loop {
            println!("remaining = {}", remaining);
            if remaining == 9 {
                break; // 跳出内层循环
            }
            if count == 2 {
                break 'counting_up; // 跳出外层循环
            }
            remaining -= 1;
        }
        
        count += 1;
    }
    
    println!("结束计数，count = {}", count);
}
```

### while循环

```rust
fn main() {
    let mut number = 3;
    
    while number != 0 {
        println!("{}!", number);
        number -= 1;
    }
    
    println!("发射！！！");
}
```

### for循环

`for`循环是最常用的循环类型：

```rust
fn main() {
    // 遍历数组
    let a = [10, 20, 30, 40, 50];
    
    for element in a {
        println!("值是: {}", element);
    }
    
    // 使用范围
    for number in 1..4 {
        println!("{}!", number);
    }
    
    // 包含结束值的范围
    for number in 1..=3 {
        println!("{}!", number);
    }
    
    // 反向遍历
    for number in (1..4).rev() {
        println!("{}!", number);
    }
}
```

### 遍历集合的不同方式

```rust
fn main() {
    let vec = vec![1, 2, 3, 4, 5];
    
    // 获取所有权（消费迭代器）
    for item in vec {
        println!("拥有: {}", item);
    }
    // vec在这里不再可用
    
    let vec2 = vec![1, 2, 3, 4, 5];
    
    // 借用（不可变引用）
    for item in &vec2 {
        println!("借用: {}", item);
    }
    // vec2仍然可用
    
    let mut vec3 = vec![1, 2, 3, 4, 5];
    
    // 可变借用
    for item in &mut vec3 {
        *item *= 2;
    }
    
    println!("修改后的vec3: {:?}", vec3);
}
```

## 模式匹配

### match表达式

`match`是Rust中强大的控制流结构：

```rust
#[derive(Debug)]
enum Coin {
    Penny,
    Nickel,
    Dime,
    Quarter,
}

fn value_in_cents(coin: Coin) -> u8 {
    match coin {
        Coin::Penny => {
            println!("幸运便士！");
            1
        }
        Coin::Nickel => 5,
        Coin::Dime => 10,
        Coin::Quarter => 25,
    }
}

fn main() {
    let coin = Coin::Penny;
    println!("硬币价值: {} 分", value_in_cents(coin));
}
```

### 绑定值的模式

```rust
#[derive(Debug)]
enum UsState {
    Alabama,
    Alaska,
    // ... 其他州
}

enum Coin {
    Penny,
    Nickel,
    Dime,
    Quarter(UsState),
}

fn value_in_cents(coin: Coin) -> u8 {
    match coin {
        Coin::Penny => 1,
        Coin::Nickel => 5,
        Coin::Dime => 10,
        Coin::Quarter(state) => {
            println!("来自{:?}州的25分硬币！", state);
            25
        }
    }
}

fn main() {
    let coin = Coin::Quarter(UsState::Alaska);
    println!("硬币价值: {} 分", value_in_cents(coin));
}
```

### 匹配Option<T>

```rust
fn plus_one(x: Option<i32>) -> Option<i32> {
    match x {
        None => None,
        Some(i) => Some(i + 1),
    }
}

fn main() {
    let five = Some(5);
    let six = plus_one(five);
    let none = plus_one(None);
    
    println!("five: {:?}", five);
    println!("six: {:?}", six);
    println!("none: {:?}", none);
}
```

### 通配模式和_占位符

```rust
fn main() {
    let dice_roll = 9;
    
    match dice_roll {
        3 => add_fancy_hat(),
        7 => remove_fancy_hat(),
        other => move_player(other), // 绑定其他值
    }
    
    // 或者使用_忽略值
    match dice_roll {
        3 => add_fancy_hat(),
        7 => remove_fancy_hat(),
        _ => reroll(), // 忽略其他所有值
    }
}

fn add_fancy_hat() {
    println!("戴上帽子");
}

fn remove_fancy_hat() {
    println!("脱下帽子");
}

fn move_player(num_spaces: u8) {
    println!("移动{}格", num_spaces);
}

fn reroll() {
    println!("重新掷骰子");
}
```

### if let简洁控制流

当只关心一个模式时，`if let`比`match`更简洁：

```rust
fn main() {
    let config_max = Some(3u8);
    
    // 使用match
    match config_max {
        Some(max) => println!("最大值是{}", max),
        _ => (),
    }
    
    // 使用if let更简洁
    if let Some(max) = config_max {
        println!("最大值是{}", max);
    }
    
    // if let也可以有else
    let mut count = 0;
    let coin = Coin::Quarter(UsState::Alaska);
    
    if let Coin::Quarter(state) = coin {
        println!("来自{:?}州的25分硬币！", state);
    } else {
        count += 1;
    }
}
```

### while let条件循环

```rust
fn main() {
    let mut stack = Vec::new();
    
    stack.push(1);
    stack.push(2);
    stack.push(3);
    
    // 当pop返回Some时继续循环
    while let Some(top) = stack.pop() {
        println!("{}", top);
    }
}
```

## 高级模式匹配

### 匹配字面值

```rust
fn main() {
    let x = 1;
    
    match x {
        1 => println!("一"),
        2 => println!("二"),
        3 => println!("三"),
        _ => println!("其他"),
    }
}
```

### 匹配命名变量

```rust
fn main() {
    let x = Some(5);
    let y = 10;
    
    match x {
        Some(50) => println!("得到50"),
        Some(y) => println!("匹配，y = {:?}", y), // 这里的y遮蔽了外部的y
        _ => println!("默认情况，x = {:?}", x),
    }
    
    println!("最后：x = {:?}, y = {:?}", x, y);
}
```

### 多个模式

```rust
fn main() {
    let x = 1;
    
    match x {
        1 | 2 => println!("一或二"),
        3 => println!("三"),
        _ => println!("其他"),
    }
}
```

### 通过..=匹配值的范围

```rust
fn main() {
    let x = 5;
    
    match x {
        1..=5 => println!("一到五"),
        _ => println!("其他"),
    }
    
    let x = 'c';
    
    match x {
        'a'..='j' => println!("前十个字母"),
        'k'..='z' => println!("后十六个字母"),
        _ => println!("其他"),
    }
}
```

### 解构结构体

```rust
struct Point {
    x: i32,
    y: i32,
}

fn main() {
    let p = Point { x: 0, y: 7 };
    
    match p {
        Point { x, y: 0 } => println!("在x轴上，x = {}", x),
        Point { x: 0, y } => println!("在y轴上，y = {}", y),
        Point { x, y } => println!("不在轴上：({}, {})", x, y),
    }
    
    // 使用..忽略剩余字段
    let origin = Point { x: 0, y: 0 };
    
    match origin {
        Point { x, .. } => println!("x是{}", x),
    }
}
```

### 解构枚举

```rust
enum Message {
    Quit,
    Move { x: i32, y: i32 },
    Write(String),
    ChangeColor(i32, i32, i32),
}

fn main() {
    let msg = Message::ChangeColor(0, 160, 255);
    
    match msg {
        Message::Quit => {
            println!("退出变体没有数据可解构。")
        }
        Message::Move { x, y } => {
            println!("移动到x: {}, y: {}", x, y);
        }
        Message::Write(text) => println!("文本消息: {}", text),
        Message::ChangeColor(r, g, b) => {
            println!("改变颜色为红: {}, 绿: {}, 蓝: {}", r, g, b)
        }
    }
}
```

### 匹配守卫

匹配守卫是一个指定于match分支模式之后的额外if条件：

```rust
fn main() {
    let num = Some(4);
    
    match num {
        Some(x) if x < 5 => println!("小于五: {}", x),
        Some(x) => println!("{}", x),
        None => (),
    }
    
    let x = Some(5);
    let y = 10;
    
    match x {
        Some(50) => println!("得到50"),
        Some(n) if n == y => println!("匹配，n = {}", n),
        _ => println!("默认情况，x = {:?}", x),
    }
    
    println!("最后：x = {:?}, y = {:?}", x, y);
}
```

## 嵌入式开发中的控制流

在嵌入式开发中，控制流结构经常用于状态机和事件处理：

```rust
// 模拟嵌入式系统中的状态机
#[derive(Debug, PartialEq)]
enum SystemState {
    Idle,
    Processing,
    Error(u8), // 错误码
    Shutdown,
}

#[derive(Debug)]
enum Event {
    Start,
    DataReceived,
    ErrorOccurred(u8),
    Reset,
    PowerOff,
}

struct EmbeddedSystem {
    state: SystemState,
    error_count: u32,
}

impl EmbeddedSystem {
    fn new() -> Self {
        EmbeddedSystem {
            state: SystemState::Idle,
            error_count: 0,
        }
    }
    
    fn handle_event(&mut self, event: Event) {
        println!("当前状态: {:?}, 事件: {:?}", self.state, event);
        
        self.state = match (&self.state, event) {
            (SystemState::Idle, Event::Start) => SystemState::Processing,
            (SystemState::Processing, Event::DataReceived) => {
                println!("处理数据...");
                SystemState::Processing
            }
            (_, Event::ErrorOccurred(code)) => {
                self.error_count += 1;
                SystemState::Error(code)
            }
            (SystemState::Error(_), Event::Reset) => {
                println!("重置系统");
                SystemState::Idle
            }
            (_, Event::PowerOff) => SystemState::Shutdown,
            _ => {
                println!("无效的状态转换");
                self.state.clone()
            }
        };
        
        println!("新状态: {:?}\n", self.state);
    }
    
    fn run(&mut self) {
        // 模拟主循环
        let events = vec![
            Event::Start,
            Event::DataReceived,
            Event::ErrorOccurred(404),
            Event::Reset,
            Event::Start,
            Event::PowerOff,
        ];
        
        for event in events {
            if self.state == SystemState::Shutdown {
                break;
            }
            self.handle_event(event);
        }
        
        println!("系统关闭，总错误数: {}", self.error_count);
    }
}

fn main() {
    let mut system = EmbeddedSystem::new();
    system.run();
}
```

### 中断处理模拟

```rust
// 模拟中断处理
#[derive(Debug)]
enum InterruptType {
    Timer,
    Gpio(u8), // GPIO引脚号
    Uart,
    Adc,
}

fn handle_interrupt(interrupt: InterruptType) {
    match interrupt {
        InterruptType::Timer => {
            println!("定时器中断：更新系统时钟");
            // 处理定时器中断
        }
        InterruptType::Gpio(pin) if pin < 16 => {
            println!("GPIO中断：引脚{}状态改变", pin);
            // 处理GPIO中断
        }
        InterruptType::Gpio(pin) => {
            println!("警告：无效的GPIO引脚{}", pin);
        }
        InterruptType::Uart => {
            println!("UART中断：数据接收");
            // 处理串口数据
        }
        InterruptType::Adc => {
            println!("ADC中断：转换完成");
            // 处理ADC数据
        }
    }
}

fn main() {
    // 模拟中断序列
    let interrupts = vec![
        InterruptType::Timer,
        InterruptType::Gpio(13),
        InterruptType::Uart,
        InterruptType::Adc,
        InterruptType::Gpio(20), // 无效引脚
    ];
    
    for interrupt in interrupts {
        handle_interrupt(interrupt);
    }
}
```

## 实践练习

### 练习1：简单计算器

```rust
#[derive(Debug)]
enum Operation {
    Add,
    Subtract,
    Multiply,
    Divide,
}

fn calculate(a: f64, b: f64, op: Operation) -> Result<f64, String> {
    match op {
        Operation::Add => Ok(a + b),
        Operation::Subtract => Ok(a - b),
        Operation::Multiply => Ok(a * b),
        Operation::Divide => {
            if b != 0.0 {
                Ok(a / b)
            } else {
                Err("除零错误".to_string())
            }
        }
    }
}

fn main() {
    let operations = vec![
        (10.0, 5.0, Operation::Add),
        (10.0, 3.0, Operation::Subtract),
        (4.0, 7.0, Operation::Multiply),
        (15.0, 3.0, Operation::Divide),
        (10.0, 0.0, Operation::Divide), // 错误情况
    ];
    
    for (a, b, op) in operations {
        match calculate(a, b, op) {
            Ok(result) => println!("{} {:?} {} = {}", a, op, b, result),
            Err(error) => println!("错误：{}", error),
        }
    }
}
```

### 练习2：猜数字游戏

```rust
use std::io;
use std::cmp::Ordering;

fn main() {
    println!("猜数字游戏！");
    
    let secret_number = 42; // 在实际程序中，这应该是随机数
    let mut attempts = 0;
    
    loop {
        println!("请输入你的猜测：");
        
        let mut guess = String::new();
        
        io::stdin()
            .read_line(&mut guess)
            .expect("读取输入失败");
        
        let guess: u32 = match guess.trim().parse() {
            Ok(num) => num,
            Err(_) => {
                println!("请输入一个有效的数字！");
                continue;
            }
        };
        
        attempts += 1;
        
        match guess.cmp(&secret_number) {
            Ordering::Less => println!("太小了！"),
            Ordering::Greater => println!("太大了！"),
            Ordering::Equal => {
                println!("你赢了！用了{}次尝试。", attempts);
                break;
            }
        }
        
        if attempts >= 5 {
            println!("游戏结束！答案是{}。", secret_number);
            break;
        }
    }
}
```

## 小结

本章介绍了Rust的控制流结构：

- **条件语句**：`if`表达式可以返回值，条件必须是bool类型
- **循环**：`loop`、`while`和`for`循环，支持标签和提前退出
- **模式匹配**：强大的`match`表达式，必须穷尽所有可能
- **简化语法**：`if let`和`while let`提供更简洁的模式匹配
- **高级模式**：支持范围、守卫、解构等复杂模式

控制流是编程的基础，掌握这些概念对于编写清晰、安全的Rust代码至关重要。

## 下一章

[函数与闭包](./04-functions-closures.md) - 学习函数定义、参数传递和闭包
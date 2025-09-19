# 函数与闭包

## 概述

函数是Rust程序的基本构建块。Rust的函数系统既强大又灵活，支持多种参数传递方式、返回值类型，以及高阶函数特性。闭包（closure）是Rust中的匿名函数，可以捕获其环境中的变量，为函数式编程提供了强大的支持。

## 学习目标

- 掌握函数的定义和调用语法
- 理解参数传递的不同方式
- 学会使用多种返回值类型
- 掌握闭包的语法和使用场景
- 理解函数指针和函数类型
- 学会高阶函数的编程模式

## 1. 函数基础

### 1.1 函数定义语法

```rust
// 基本函数定义
fn function_name(parameter: Type) -> ReturnType {
    // 函数体
    return_value
}

// 示例：简单的加法函数
fn add(a: i32, b: i32) -> i32 {
    a + b  // 最后一个表达式作为返回值
}

// 无返回值的函数（返回单元类型 ()）
fn print_message(msg: &str) {
    println!("{}", msg);
}

// 显式返回单元类型
fn do_nothing() -> () {
    // 空函数体
}
```

### 1.2 函数调用

```rust
fn main() {
    let result = add(5, 3);
    println!("5 + 3 = {}", result);
    
    print_message("Hello, Rust!");
    
    // 函数调用也是表达式
    let doubled = multiply_by_two(add(2, 3));
    println!("Doubled result: {}", doubled);
}

fn multiply_by_two(x: i32) -> i32 {
    x * 2
}
```

### 1.3 函数命名规范

```rust
// 使用snake_case命名
fn calculate_area(width: f64, height: f64) -> f64 {
    width * height
}

// 布尔返回值通常以is_或has_开头
fn is_even(number: i32) -> bool {
    number % 2 == 0
}

fn has_permission(user: &str) -> bool {
    user == "admin"
}
```

## 2. 参数传递

### 2.1 值传递（移动语义）

```rust
fn take_ownership(s: String) {
    println!("Taking ownership of: {}", s);
    // s在函数结束时被drop
}

fn main() {
    let my_string = String::from("Hello");
    take_ownership(my_string);
    // println!("{}", my_string); // 错误！my_string已被移动
}
```

### 2.2 引用传递（借用）

```rust
// 不可变引用
fn calculate_length(s: &String) -> usize {
    s.len()  // 不能修改s
}

// 可变引用
fn append_exclamation(s: &mut String) {
    s.push('!');
}

fn main() {
    let s1 = String::from("Hello");
    let len = calculate_length(&s1);
    println!("Length of '{}' is {}", s1, len);  // s1仍然有效
    
    let mut s2 = String::from("Hello");
    append_exclamation(&mut s2);
    println!("{}", s2);  // 输出: Hello!
}
```

### 2.3 复制类型的传递

```rust
fn square(x: i32) -> i32 {
    x * x  // i32实现了Copy trait，所以是复制而不是移动
}

fn main() {
    let num = 5;
    let result = square(num);
    println!("Original: {}, Squared: {}", num, result);  // num仍然有效
}
```

### 2.4 多参数和参数解构

```rust
// 多个参数
fn create_point(x: f64, y: f64, z: f64) -> (f64, f64, f64) {
    (x, y, z)
}

// 元组参数解构
fn distance_from_origin((x, y): (f64, f64)) -> f64 {
    (x * x + y * y).sqrt()
}

// 结构体参数
struct Point {
    x: f64,
    y: f64,
}

fn move_point(mut point: Point, dx: f64, dy: f64) -> Point {
    point.x += dx;
    point.y += dy;
    point
}
```

## 3. 返回值

### 3.1 单个返回值

```rust
// 表达式返回（推荐）
fn add(a: i32, b: i32) -> i32 {
    a + b  // 没有分号，作为返回值
}

// 显式return语句
fn subtract(a: i32, b: i32) -> i32 {
    return a - b;  // 有分号，需要return关键字
}

// 条件返回
fn absolute_value(x: i32) -> i32 {
    if x >= 0 {
        x
    } else {
        -x
    }
}
```

### 3.2 多个返回值（元组）

```rust
fn divide_with_remainder(dividend: i32, divisor: i32) -> (i32, i32) {
    let quotient = dividend / divisor;
    let remainder = dividend % divisor;
    (quotient, remainder)
}

fn main() {
    let (q, r) = divide_with_remainder(17, 5);
    println!("17 ÷ 5 = {} remainder {}", q, r);
}
```

### 3.3 Result类型返回

```rust
fn safe_divide(dividend: f64, divisor: f64) -> Result<f64, String> {
    if divisor == 0.0 {
        Err("Division by zero".to_string())
    } else {
        Ok(dividend / divisor)
    }
}

fn main() {
    match safe_divide(10.0, 2.0) {
        Ok(result) => println!("Result: {}", result),
        Err(error) => println!("Error: {}", error),
    }
}
```

### 3.4 Option类型返回

```rust
fn find_first_even(numbers: &[i32]) -> Option<i32> {
    for &num in numbers {
        if num % 2 == 0 {
            return Some(num);
        }
    }
    None
}

fn main() {
    let numbers = [1, 3, 4, 7, 8];
    match find_first_even(&numbers) {
        Some(even) => println!("First even number: {}", even),
        None => println!("No even numbers found"),
    }
}
```

## 4. 闭包（Closures）

### 4.1 闭包基础语法

```rust
fn main() {
    // 基本闭包语法
    let add_one = |x| x + 1;
    println!("5 + 1 = {}", add_one(5));
    
    // 带类型注解的闭包
    let multiply: fn(i32, i32) -> i32 = |x, y| x * y;
    println!("3 * 4 = {}", multiply(3, 4));
    
    // 多行闭包
    let complex_calculation = |x: i32| {
        let doubled = x * 2;
        let squared = doubled * doubled;
        squared + 1
    };
    
    println!("Complex result: {}", complex_calculation(3));
}
```

### 4.2 环境捕获

```rust
fn main() {
    let x = 10;
    let y = 20;
    
    // 捕获环境变量（不可变借用）
    let capture_immutable = || {
        println!("x = {}, y = {}", x, y);
        x + y
    };
    
    println!("Sum: {}", capture_immutable());
    println!("x and y are still accessible: {}, {}", x, y);
    
    // 捕获环境变量（可变借用）
    let mut z = 30;
    let mut capture_mutable = || {
        z += 10;
        println!("Modified z: {}", z);
    };
    
    capture_mutable();
    // println!("z: {}", z); // 错误！z被可变借用
}
```

### 4.3 移动闭包

```rust
fn main() {
    let data = vec![1, 2, 3, 4, 5];
    
    // move关键字强制移动捕获
    let take_ownership = move || {
        println!("Data: {:?}", data);
        data.len()
    };
    
    println!("Length: {}", take_ownership());
    // println!("{:?}", data); // 错误！data已被移动
}

// 在线程中使用移动闭包
use std::thread;

fn spawn_thread_example() {
    let data = vec![1, 2, 3];
    
    let handle = thread::spawn(move || {
        println!("Thread data: {:?}", data);
    });
    
    handle.join().unwrap();
}
```

### 4.4 闭包作为参数

```rust
// 接受闭包作为参数的函数
fn apply_operation<F>(x: i32, y: i32, op: F) -> i32
where
    F: Fn(i32, i32) -> i32,
{
    op(x, y)
}

fn main() {
    let add = |a, b| a + b;
    let multiply = |a, b| a * b;
    
    println!("Add: {}", apply_operation(5, 3, add));
    println!("Multiply: {}", apply_operation(5, 3, multiply));
    
    // 直接传递闭包
    println!("Subtract: {}", apply_operation(5, 3, |a, b| a - b));
}
```

### 4.5 闭包trait：Fn、FnMut、FnOnce

```rust
// Fn: 可以多次调用，不可变借用捕获的变量
fn call_multiple_times<F>(f: F) 
where
    F: Fn() -> i32,
{
    println!("First call: {}", f());
    println!("Second call: {}", f());
}

// FnMut: 可以多次调用，可变借用捕获的变量
fn call_with_mutation<F>(mut f: F)
where
    F: FnMut() -> i32,
{
    println!("First call: {}", f());
    println!("Second call: {}", f());
}

// FnOnce: 只能调用一次，获取捕获变量的所有权
fn call_once<F>(f: F) -> i32
where
    F: FnOnce() -> i32,
{
    f()
}

fn main() {
    let x = 10;
    
    // Fn示例
    let immutable_closure = || x * 2;
    call_multiple_times(immutable_closure);
    
    // FnMut示例
    let mut counter = 0;
    let mut mutable_closure = || {
        counter += 1;
        counter
    };
    call_with_mutation(&mut mutable_closure);
    
    // FnOnce示例
    let data = vec![1, 2, 3];
    let once_closure = move || data.len();
    println!("Once call: {}", call_once(once_closure));
}
```

## 5. 函数指针

### 5.1 函数指针类型

```rust
// 函数指针类型：fn(参数类型) -> 返回类型
fn add(a: i32, b: i32) -> i32 {
    a + b
}

fn multiply(a: i32, b: i32) -> i32 {
    a * b
}

fn main() {
    // 函数指针变量
    let operation: fn(i32, i32) -> i32 = add;
    println!("Result: {}", operation(5, 3));
    
    // 函数指针数组
    let operations: [fn(i32, i32) -> i32; 2] = [add, multiply];
    for op in operations.iter() {
        println!("Operation result: {}", op(4, 6));
    }
}
```

### 5.2 函数指针作为参数

```rust
fn execute_operation(x: i32, y: i32, op: fn(i32, i32) -> i32) -> i32 {
    op(x, y)
}

fn add(a: i32, b: i32) -> i32 { a + b }
fn subtract(a: i32, b: i32) -> i32 { a - b }

fn main() {
    println!("Add: {}", execute_operation(10, 5, add));
    println!("Subtract: {}", execute_operation(10, 5, subtract));
}
```

### 5.3 函数指针与闭包的区别

```rust
fn main() {
    // 函数指针：只能指向具名函数
    let fn_ptr: fn(i32) -> i32 = |x| x * 2;  // 简单闭包可以强制转换
    
    // 闭包：可以捕获环境
    let multiplier = 3;
    let closure = |x| x * multiplier;  // 这个不能转换为函数指针
    
    // 使用泛型同时接受函数指针和闭包
    fn apply_twice<F>(x: i32, f: F) -> i32 
    where
        F: Fn(i32) -> i32,
    {
        f(f(x))
    }
    
    println!("Function pointer: {}", apply_twice(5, fn_ptr));
    println!("Closure: {}", apply_twice(5, closure));
}
```

## 6. 高阶函数

### 6.1 返回闭包的函数

```rust
// 返回闭包需要使用Box<dyn Fn>
fn create_adder(x: i32) -> Box<dyn Fn(i32) -> i32> {
    Box::new(move |y| x + y)
}

fn create_multiplier(factor: i32) -> impl Fn(i32) -> i32 {
    move |x| x * factor
}

fn main() {
    let add_5 = create_adder(5);
    println!("10 + 5 = {}", add_5(10));
    
    let double = create_multiplier(2);
    println!("7 * 2 = {}", double(7));
}
```

### 6.2 函数组合

```rust
fn compose<F, G, A, B, C>(f: F, g: G) -> impl Fn(A) -> C
where
    F: Fn(B) -> C,
    G: Fn(A) -> B,
{
    move |x| f(g(x))
}

fn main() {
    let add_one = |x: i32| x + 1;
    let double = |x: i32| x * 2;
    
    // 组合函数：先加1，再乘2
    let add_then_double = compose(double, add_one);
    println!("(5 + 1) * 2 = {}", add_then_double(5));
    
    // 组合函数：先乘2，再加1
    let double_then_add = compose(add_one, double);
    println!("5 * 2 + 1 = {}", double_then_add(5));
}
```

### 6.3 迭代器与高阶函数

```rust
fn main() {
    let numbers = vec![1, 2, 3, 4, 5];
    
    // map: 转换每个元素
    let doubled: Vec<i32> = numbers.iter()
        .map(|x| x * 2)
        .collect();
    println!("Doubled: {:?}", doubled);
    
    // filter: 过滤元素
    let evens: Vec<i32> = numbers.iter()
        .filter(|&&x| x % 2 == 0)
        .cloned()
        .collect();
    println!("Evens: {:?}", evens);
    
    // fold: 累积操作
    let sum = numbers.iter().fold(0, |acc, x| acc + x);
    println!("Sum: {}", sum);
    
    // 链式操作
    let result: Vec<i32> = numbers.iter()
        .filter(|&&x| x > 2)
        .map(|x| x * x)
        .collect();
    println!("Filtered and squared: {:?}", result);
}
```

## 7. 递归函数

### 7.1 基本递归

```rust
fn factorial(n: u64) -> u64 {
    if n <= 1 {
        1
    } else {
        n * factorial(n - 1)
    }
}

fn fibonacci(n: u32) -> u64 {
    match n {
        0 => 0,
        1 => 1,
        _ => fibonacci(n - 1) + fibonacci(n - 2),
    }
}

fn main() {
    println!("5! = {}", factorial(5));
    println!("Fibonacci(10) = {}", fibonacci(10));
}
```

### 7.2 尾递归优化

```rust
fn factorial_tail_recursive(n: u64) -> u64 {
    fn factorial_helper(n: u64, acc: u64) -> u64 {
        if n <= 1 {
            acc
        } else {
            factorial_helper(n - 1, acc * n)
        }
    }
    factorial_helper(n, 1)
}

fn fibonacci_tail_recursive(n: u32) -> u64 {
    fn fib_helper(n: u32, a: u64, b: u64) -> u64 {
        if n == 0 {
            a
        } else {
            fib_helper(n - 1, b, a + b)
        }
    }
    fib_helper(n, 0, 1)
}
```

## 8. 实践示例

### 8.1 计算器函数

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
            if b == 0.0 {
                Err("Division by zero".to_string())
            } else {
                Ok(a / b)
            }
        }
    }
}

// 使用函数指针的版本
fn calculate_with_fn(a: f64, b: f64, op: fn(f64, f64) -> f64) -> f64 {
    op(a, b)
}

fn main() {
    println!("{:?}", calculate(10.0, 5.0, Operation::Add));
    println!("{:?}", calculate(10.0, 0.0, Operation::Divide));
    
    println!("{}", calculate_with_fn(10.0, 5.0, |a, b| a + b));
}
```

### 8.2 函数式编程风格的数据处理

```rust
#[derive(Debug)]
struct Person {
    name: String,
    age: u32,
    salary: u32,
}

fn main() {
    let people = vec![
        Person { name: "Alice".to_string(), age: 30, salary: 50000 },
        Person { name: "Bob".to_string(), age: 25, salary: 45000 },
        Person { name: "Charlie".to_string(), age: 35, salary: 60000 },
        Person { name: "Diana".to_string(), age: 28, salary: 55000 },
    ];
    
    // 函数式处理：找出年龄大于25且薪水大于50000的人的平均薪水
    let average_salary = people.iter()
        .filter(|person| person.age > 25 && person.salary > 50000)
        .map(|person| person.salary)
        .fold((0, 0), |(sum, count), salary| (sum + salary, count + 1))
        .0 as f64 / people.iter()
            .filter(|person| person.age > 25 && person.salary > 50000)
            .count() as f64;
    
    println!("Average salary: {:.2}", average_salary);
    
    // 使用闭包进行复杂过滤
    let filter_criteria = |min_age: u32, min_salary: u32| {
        move |person: &&Person| person.age >= min_age && person.salary >= min_salary
    };
    
    let qualified_people: Vec<&Person> = people.iter()
        .filter(filter_criteria(30, 50000))
        .collect();
    
    println!("Qualified people: {:?}", qualified_people);
}
```

## 9. 最佳实践

### 9.1 函数设计原则

```rust
// 好的实践：单一职责，清晰的输入输出
fn validate_email(email: &str) -> bool {
    email.contains('@') && email.contains('.')
}

fn format_user_display(name: &str, email: &str) -> String {
    if validate_email(email) {
        format!("{} <{}>", name, email)
    } else {
        name.to_string()
    }
}

// 好的实践：使用Result处理错误
fn parse_age(input: &str) -> Result<u32, String> {
    input.parse::<u32>()
        .map_err(|_| format!("Invalid age: {}", input))
}
```

### 9.2 性能考虑

```rust
// 避免不必要的克隆
fn process_strings_bad(strings: Vec<String>) -> Vec<String> {
    strings.into_iter()
        .map(|s| s.to_uppercase())  // 直接消费，避免克隆
        .collect()
}

// 使用引用减少内存分配
fn count_long_strings(strings: &[String], min_length: usize) -> usize {
    strings.iter()
        .filter(|s| s.len() >= min_length)
        .count()
}

// 内联简单函数
#[inline]
fn square(x: i32) -> i32 {
    x * x
}
```

## 10. 总结

函数和闭包是Rust编程的核心概念：

1. **函数**提供了代码组织和重用的基本机制
2. **参数传递**遵循所有权规则，支持值传递、引用传递
3. **返回值**可以是单个值、元组、Result或Option类型
4. **闭包**提供了强大的函数式编程能力，可以捕获环境变量
5. **函数指针**允许将函数作为一等公民传递
6. **高阶函数**支持函数组合和抽象

掌握这些概念对于编写高质量的Rust代码至关重要，特别是在嵌入式开发中，良好的函数设计可以提高代码的可维护性和性能。

## 练习题

1. 实现一个通用的排序函数，接受比较函数作为参数
2. 创建一个闭包工厂，根据不同参数生成不同的验证函数
3. 使用递归实现二叉树的遍历算法
4. 实现一个简单的函数式编程库，包含map、filter、reduce等操作
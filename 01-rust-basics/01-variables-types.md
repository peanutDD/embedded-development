# 变量与数据类型

## 概述

Rust是一门静态类型语言，这意味着所有变量的类型在编译时都必须确定。本章将介绍Rust的变量系统和类型系统，这是理解Rust语言的基础。

## 变量声明

### 不可变变量（默认）

在Rust中，变量默认是不可变的（immutable）：

```rust
fn main() {
    let x = 5;
    println!("x的值是: {}", x);
    
    // 下面这行会编译错误
    // x = 6; // 错误：cannot assign twice to immutable variable
}
```

### 可变变量

使用`mut`关键字声明可变变量：

```rust
fn main() {
    let mut x = 5;
    println!("x的值是: {}", x);
    
    x = 6; // 正确：可变变量可以重新赋值
    println!("x的新值是: {}", x);
}
```

### 变量遮蔽（Shadowing）

可以声明与之前变量同名的新变量：

```rust
fn main() {
    let x = 5;
    let x = x + 1; // 遮蔽之前的x
    let x = x * 2;  // 再次遮蔽
    
    println!("x的值是: {}", x); // 输出: 12
    
    // 遮蔽还可以改变类型
    let spaces = "   ";
    let spaces = spaces.len(); // 从字符串变为数字
}
```

## 基本数据类型

### 整数类型

Rust提供了多种整数类型：

| 长度 | 有符号 | 无符号 |
|------|--------|--------|
| 8位  | i8     | u8     |
| 16位 | i16    | u16    |
| 32位 | i32    | u32    |
| 64位 | i64    | u64    |
| 128位| i128   | u128   |
| arch | isize  | usize  |

```rust
fn main() {
    // 不同的整数类型
    let a: i8 = -128;
    let b: u8 = 255;
    let c: i32 = -2_147_483_648;
    let d: u32 = 4_294_967_295;
    
    // 数字字面量
    let decimal = 98_222;        // 十进制
    let hex = 0xff;              // 十六进制
    let octal = 0o77;            // 八进制
    let binary = 0b1111_0000;    // 二进制
    let byte = b'A';             // 字节（仅限u8）
    
    println!("各种整数: {}, {}, {}, {}", a, b, c, d);
    println!("字面量: {}, {}, {}, {}, {}", decimal, hex, octal, binary, byte);
}
```

### 浮点类型

Rust有两种浮点类型：

```rust
fn main() {
    let x = 2.0;      // f64（默认）
    let y: f32 = 3.0; // f32
    
    println!("浮点数: {}, {}", x, y);
}
```

### 布尔类型

```rust
fn main() {
    let t = true;
    let f: bool = false;
    
    println!("布尔值: {}, {}", t, f);
}
```

### 字符类型

```rust
fn main() {
    let c = 'z';
    let z = 'ℤ';
    let heart_eyed_cat = '😻';
    
    println!("字符: {}, {}, {}", c, z, heart_eyed_cat);
}
```

## 复合数据类型

### 元组（Tuple）

元组可以包含不同类型的值：

```rust
fn main() {
    let tup: (i32, f64, u8) = (500, 6.4, 1);
    
    // 解构元组
    let (x, y, z) = tup;
    println!("解构后的值: {}, {}, {}", x, y, z);
    
    // 通过索引访问
    let five_hundred = tup.0;
    let six_point_four = tup.1;
    let one = tup.2;
    
    println!("索引访问: {}, {}, {}", five_hundred, six_point_four, one);
}
```

### 数组（Array）

数组中的所有元素必须是相同类型：

```rust
fn main() {
    let a = [1, 2, 3, 4, 5];
    let months = ["January", "February", "March", "April", "May", "June",
                  "July", "August", "September", "October", "November", "December"];
    
    // 指定类型和长度
    let a: [i32; 5] = [1, 2, 3, 4, 5];
    
    // 初始化相同值的数组
    let a = [3; 5]; // 等同于 [3, 3, 3, 3, 3]
    
    // 访问数组元素
    let first = a[0];
    let second = a[1];
    
    println!("数组元素: {}, {}", first, second);
    println!("数组长度: {}", a.len());
}
```

## 类型推断和类型注解

### 类型推断

Rust编译器通常可以推断变量的类型：

```rust
fn main() {
    let x = 42;        // 推断为i32
    let y = 3.14;      // 推断为f64
    let z = "hello";   // 推断为&str
    
    println!("{}, {}, {}", x, y, z);
}
```

### 显式类型注解

有时需要显式指定类型：

```rust
fn main() {
    let guess: u32 = "42".parse().expect("不是一个数字！");
    
    // 或者使用turbofish语法
    let guess = "42".parse::<u32>().expect("不是一个数字！");
    
    println!("猜测的数字: {}", guess);
}
```

## 常量

常量使用`const`关键字声明，必须指定类型：

```rust
const THREE_HOURS_IN_SECONDS: u32 = 60 * 60 * 3;

fn main() {
    println!("三小时的秒数: {}", THREE_HOURS_IN_SECONDS);
}
```

## 嵌入式开发中的类型选择

在嵌入式开发中，选择合适的数据类型非常重要：

```rust
// 嵌入式开发中的类型选择示例
fn embedded_types_example() {
    // 寄存器地址通常使用usize或特定的指针类型
    let register_addr: usize = 0x4000_0000;
    
    // GPIO引脚编号通常使用u8就足够
    let pin_number: u8 = 13;
    
    // 传感器读数可能需要特定精度
    let temperature: f32 = 25.6;
    
    // 状态标志使用bool
    let is_sensor_ready: bool = true;
    
    // 时间戳可能需要u64
    let timestamp: u64 = 1234567890;
    
    println!("寄存器地址: 0x{:X}", register_addr);
    println!("引脚号: {}", pin_number);
    println!("温度: {}°C", temperature);
    println!("传感器就绪: {}", is_sensor_ready);
    println!("时间戳: {}", timestamp);
}
```

## 类型转换

### 显式转换（as关键字）

```rust
fn main() {
    let a = 13u8;
    let b = 7u32;
    let c = a as u32 + b;
    
    println!("转换结果: {}", c);
    
    // 浮点数转整数（会截断）
    let f = 3.99;
    let i = f as i32;
    println!("浮点转整数: {} -> {}", f, i);
}
```

### From和Into特征

```rust
fn main() {
    // From特征
    let s = String::from("hello");
    
    // Into特征（通常由From自动实现）
    let s: String = "hello".into();
    
    println!("字符串: {}", s);
}
```

## 实践练习

### 练习1：温度转换器

创建一个程序，将摄氏度转换为华氏度：

```rust
fn celsius_to_fahrenheit(celsius: f64) -> f64 {
    celsius * 9.0 / 5.0 + 32.0
}

fn main() {
    let temp_c = 25.0;
    let temp_f = celsius_to_fahrenheit(temp_c);
    
    println!("{}°C = {}°F", temp_c, temp_f);
}
```

### 练习2：数组操作

```rust
fn main() {
    let numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    
    // 计算数组元素的和
    let mut sum = 0;
    for number in numbers.iter() {
        sum += number;
    }
    
    println!("数组元素的和: {}", sum);
    
    // 找出最大值
    let mut max = numbers[0];
    for &number in numbers.iter() {
        if number > max {
            max = number;
        }
    }
    
    println!("最大值: {}", max);
}
```

## 小结

本章介绍了Rust的变量系统和基本数据类型：

- 变量默认不可变，使用`mut`关键字创建可变变量
- 变量遮蔽允许重新定义同名变量
- Rust提供丰富的基本数据类型：整数、浮点数、布尔值、字符
- 复合类型包括元组和数组
- 类型推断减少了显式类型注解的需要
- 在嵌入式开发中，选择合适的数据类型对性能和内存使用很重要

掌握这些基础知识后，你就可以开始学习Rust的所有权系统了。

## 下一章

[所有权系统](./02-ownership.md) - 学习Rust最独特和重要的特性
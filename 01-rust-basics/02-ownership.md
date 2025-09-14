# 所有权系统

## 概述

所有权（Ownership）是Rust最独特和重要的特性，它使Rust能够在不使用垃圾回收器的情况下保证内存安全。理解所有权系统是掌握Rust的关键。

## 所有权规则

Rust的所有权系统基于三个基本规则：

1. **Rust中的每一个值都有一个被称为其所有者（owner）的变量**
2. **值在任一时刻有且只有一个所有者**
3. **当所有者（变量）离开作用域，这个值将被丢弃**

## 变量作用域

作用域是一个项（变量）在程序中有效的范围：

```rust
fn main() {
    {                      // s在这里无效，它尚未声明
        let s = "hello";   // 从此处起，s是有效的
        
        // 使用s
        println!("{}", s);
    }                      // 此作用域已结束，s不再有效
    
    // println!("{}", s); // 错误：s已经不在作用域内
}
```

## String类型和内存管理

### 字符串字面量 vs String

```rust
fn main() {
    // 字符串字面量：存储在程序的二进制文件中，不可变
    let s1 = "hello";
    
    // String类型：可变，存储在堆上
    let mut s2 = String::from("hello");
    s2.push_str(", world!");
    
    println!("{}", s1);
    println!("{}", s2);
}
```

### 内存分配和释放

```rust
fn main() {
    {
        let s = String::from("hello"); // s进入作用域
        
        // 使用s
        println!("{}", s);
    } // s离开作用域，调用drop函数，内存被释放
}
```

## 移动语义

### 简单类型的复制

```rust
fn main() {
    let x = 5;
    let y = x; // 复制x的值给y
    
    println!("x = {}, y = {}", x, y); // 两个变量都有效
}
```

### 复杂类型的移动

```rust
fn main() {
    let s1 = String::from("hello");
    let s2 = s1; // s1的所有权移动到s2
    
    println!("{}", s2); // 正确
    // println!("{}", s1); // 错误：s1的值已经被移动
}
```

### 移动的内部机制

```rust
// String的内部结构类似于：
// struct String {
//     ptr: *mut u8,    // 指向堆内存的指针
//     len: usize,      // 当前长度
//     capacity: usize, // 容量
// }

fn demonstrate_move() {
    let s1 = String::from("hello");
    
    // 移动发生时，只复制栈上的数据（指针、长度、容量）
    // 堆上的实际数据不会被复制
    let s2 = s1;
    
    // s1不再有效，避免了双重释放（double free）错误
    println!("{}", s2);
}
```

## 克隆

如果确实需要深度复制堆上的数据，可以使用`clone`方法：

```rust
fn main() {
    let s1 = String::from("hello");
    let s2 = s1.clone(); // 深度复制
    
    println!("s1 = {}, s2 = {}", s1, s2); // 两个变量都有效
}
```

## Copy特征

实现了`Copy`特征的类型在赋值时会被复制而不是移动：

```rust
fn main() {
    // 这些类型实现了Copy特征
    let x = 5;        // i32
    let y = true;     // bool
    let z = 'a';      // char
    let tuple = (1, 2); // 元组（如果所有元素都实现Copy）
    
    let a = x; // 复制，不是移动
    println!("x = {}, a = {}", x, a); // 都有效
}
```

## 所有权与函数

### 传递值给函数

```rust
fn main() {
    let s = String::from("hello");  // s进入作用域
    
    takes_ownership(s);             // s的值移动到函数里
                                    // s不再有效
    
    let x = 5;                      // x进入作用域
    
    makes_copy(x);                  // x被复制到函数里
                                    // x仍然有效
    
    println!("x = {}", x);          // 正确
    // println!("{}", s);           // 错误：s已被移动
}

fn takes_ownership(some_string: String) { // some_string进入作用域
    println!("{}", some_string);
} // some_string离开作用域并被丢弃

fn makes_copy(some_integer: i32) { // some_integer进入作用域
    println!("{}", some_integer);
} // some_integer离开作用域，没有特殊操作
```

### 返回值和作用域

```rust
fn main() {
    let s1 = gives_ownership();         // gives_ownership将返回值移动给s1
    
    let s2 = String::from("hello");     // s2进入作用域
    
    let s3 = takes_and_gives_back(s2);  // s2被移动到函数中，
                                        // 函数返回值移动给s3
    
    println!("s1 = {}, s3 = {}", s1, s3);
    // s2不再有效
}

fn gives_ownership() -> String {             // gives_ownership将返回值移动给调用它的函数
    let some_string = String::from("yours"); // some_string进入作用域
    some_string                              // 返回some_string并移出给调用的函数
}

fn takes_and_gives_back(a_string: String) -> String { // a_string进入作用域
    a_string  // 返回a_string并移出给调用的函数
}
```

## 引用和借用

### 不可变引用

引用允许你使用值但不获取其所有权：

```rust
fn main() {
    let s1 = String::from("hello");
    
    let len = calculate_length(&s1); // 传递引用
    
    println!("'{}' 的长度是 {}。", s1, len); // s1仍然有效
}

fn calculate_length(s: &String) -> usize { // s是String的引用
    s.len()
} // s离开作用域，但因为它不拥有引用的值，所以什么也不会发生
```

### 可变引用

```rust
fn main() {
    let mut s = String::from("hello");
    
    change(&mut s); // 传递可变引用
    
    println!("{}", s);
}

fn change(some_string: &mut String) {
    some_string.push_str(", world");
}
```

### 引用的规则

1. **在任意给定时间，要么只能有一个可变引用，要么只能有多个不可变引用**
2. **引用必须总是有效的**

```rust
fn main() {
    let mut s = String::from("hello");
    
    // 多个不可变引用是允许的
    let r1 = &s;
    let r2 = &s;
    println!("{} and {}", r1, r2);
    // r1和r2在此之后不再使用
    
    // 可变引用
    let r3 = &mut s;
    println!("{}", r3);
    
    // 下面的代码会编译错误
    // let r4 = &s;     // 错误：不能在有可变引用时创建不可变引用
    // let r5 = &mut s; // 错误：不能有多个可变引用
}
```

### 悬垂引用

Rust编译器保证引用永远不会变成悬垂引用：

```rust
// 这段代码不会编译
// fn dangle() -> &String {
//     let s = String::from("hello");
//     &s // 错误：返回了对局部变量的引用
// }

// 正确的做法
fn no_dangle() -> String {
    let s = String::from("hello");
    s // 直接返回String，移动所有权
}
```

## 切片类型

切片让你引用集合中一段连续的元素序列，而不用引用整个集合：

### 字符串切片

```rust
fn main() {
    let s = String::from("hello world");
    
    let hello = &s[0..5];  // 或 &s[..5]
    let world = &s[6..11]; // 或 &s[6..]
    let whole = &s[..];    // 整个字符串的切片
    
    println!("{}, {}, {}", hello, world, whole);
}
```

### 字符串切片作为参数

```rust
fn first_word(s: &str) -> &str {
    let bytes = s.as_bytes();
    
    for (i, &item) in bytes.iter().enumerate() {
        if item == b' ' {
            return &s[0..i];
        }
    }
    
    &s[..]
}

fn main() {
    let my_string = String::from("hello world");
    
    // first_word适用于String的切片
    let word = first_word(&my_string[0..6]);
    let word = first_word(&my_string[..]);
    // first_word也适用于String的引用，等同于整个String的切片
    let word = first_word(&my_string);
    
    let my_string_literal = "hello world";
    
    // first_word适用于字符串字面量的切片
    let word = first_word(&my_string_literal[0..6]);
    let word = first_word(&my_string_literal[..]);
    
    // 因为字符串字面量本身就是切片，这样写也可以
    let word = first_word(my_string_literal);
    
    println!("第一个单词是: {}", word);
}
```

### 其他切片

```rust
fn main() {
    let a = [1, 2, 3, 4, 5];
    
    let slice = &a[1..3]; // 类型是&[i32]
    
    assert_eq!(slice, &[2, 3]);
    
    println!("数组切片: {:?}", slice);
}
```

## 嵌入式开发中的所有权

在嵌入式开发中，所有权系统特别重要：

```rust
// 模拟嵌入式环境中的资源管理
struct GpioPin {
    pin_number: u8,
}

impl GpioPin {
    fn new(pin: u8) -> Self {
        println!("初始化GPIO引脚 {}", pin);
        GpioPin { pin_number: pin }
    }
    
    fn set_high(&self) {
        println!("设置引脚 {} 为高电平", self.pin_number);
    }
    
    fn set_low(&self) {
        println!("设置引脚 {} 为低电平", self.pin_number);
    }
}

impl Drop for GpioPin {
    fn drop(&mut self) {
        println!("释放GPIO引脚 {}", self.pin_number);
    }
}

fn configure_pin(pin: GpioPin) {
    pin.set_high();
    // pin在函数结束时被自动释放
}

fn main() {
    let pin = GpioPin::new(13);
    
    // 移动所有权到函数
    configure_pin(pin);
    
    // pin不再有效
    // pin.set_low(); // 错误：pin已被移动
    
    // 使用引用避免移动
    let pin2 = GpioPin::new(14);
    configure_pin_ref(&pin2);
    pin2.set_low(); // 仍然有效
}

fn configure_pin_ref(pin: &GpioPin) {
    pin.set_high();
    // 只是借用，不获取所有权
}
```

## 实践练习

### 练习1：字符串操作

```rust
fn main() {
    let mut s = String::from("hello");
    
    // 练习：实现一个函数，接受字符串的可变引用，并在末尾添加感叹号
    add_exclamation(&mut s);
    println!("{}", s); // 应该输出 "hello!"
    
    // 练习：实现一个函数，返回字符串中第一个单词
    let sentence = String::from("hello world rust");
    let first = get_first_word(&sentence);
    println!("第一个单词: {}", first);
}

fn add_exclamation(s: &mut String) {
    s.push('!');
}

fn get_first_word(s: &String) -> &str {
    let bytes = s.as_bytes();
    
    for (i, &item) in bytes.iter().enumerate() {
        if item == b' ' {
            return &s[0..i];
        }
    }
    
    &s[..]
}
```

### 练习2：数组切片

```rust
fn main() {
    let numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    
    // 获取前5个元素
    let first_half = &numbers[..5];
    println!("前半部分: {:?}", first_half);
    
    // 获取后5个元素
    let second_half = &numbers[5..];
    println!("后半部分: {:?}", second_half);
    
    // 计算切片的和
    let sum = sum_slice(first_half);
    println!("前半部分的和: {}", sum);
}

fn sum_slice(slice: &[i32]) -> i32 {
    let mut total = 0;
    for &value in slice {
        total += value;
    }
    total
}
```

## 常见错误和解决方案

### 错误1：使用已移动的值

```rust
// 错误示例
// fn main() {
//     let s = String::from("hello");
//     let s2 = s;
//     println!("{}", s); // 错误：s已被移动
// }

// 解决方案1：使用引用
fn solution1() {
    let s = String::from("hello");
    let s2 = &s; // 借用而不是移动
    println!("{}", s); // 正确
    println!("{}", s2);
}

// 解决方案2：克隆
fn solution2() {
    let s = String::from("hello");
    let s2 = s.clone(); // 深度复制
    println!("{}", s); // 正确
    println!("{}", s2);
}
```

### 错误2：可变引用冲突

```rust
// 错误示例
// fn main() {
//     let mut s = String::from("hello");
//     let r1 = &mut s;
//     let r2 = &mut s; // 错误：不能有多个可变引用
//     println!("{}, {}", r1, r2);
// }

// 解决方案：使用作用域分离引用
fn solution() {
    let mut s = String::from("hello");
    
    {
        let r1 = &mut s;
        r1.push_str(", world");
    } // r1在这里离开作用域
    
    let r2 = &mut s; // 现在可以创建新的可变引用
    r2.push('!');
    
    println!("{}", s);
}
```

## 小结

所有权系统是Rust的核心特性，它确保了内存安全：

- **所有权规则**：每个值有唯一所有者，所有者离开作用域时值被释放
- **移动语义**：复杂类型的赋值会移动所有权，避免双重释放
- **借用**：通过引用可以使用值而不获取所有权
- **引用规则**：要么多个不可变引用，要么一个可变引用
- **切片**：引用集合中的连续元素序列

理解所有权系统需要时间和实践，但它是编写安全、高效Rust代码的基础。

## 下一章

[控制流](./03-control-flow.md) - 学习条件语句、循环和模式匹配
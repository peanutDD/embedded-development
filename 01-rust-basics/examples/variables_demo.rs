//! 变量和类型演示示例
//! 
//! 这个示例展示了Rust中的基本变量声明、类型系统和所有权概念

fn main() {
    println!("=== Rust 变量和类型演示 ===");
    
    // 1. 变量声明和可变性
    variables_and_mutability();
    
    // 2. 数据类型
    data_types_demo();
    
    // 3. 所有权基础
    ownership_basics();
    
    // 4. 引用和借用
    references_and_borrowing();
    
    // 5. 切片
    slices_demo();
}

/// 演示变量声明和可变性
fn variables_and_mutability() {
    println!("\n--- 变量和可变性 ---");
    
    // 不可变变量
    let x = 5;
    println!("不可变变量 x = {}", x);
    
    // 可变变量
    let mut y = 10;
    println!("可变变量 y = {}", y);
    y = 15;
    println!("修改后 y = {}", y);
    
    // 变量遮蔽 (shadowing)
    let z = 20;
    println!("第一个 z = {}", z);
    let z = z + 5;
    println!("遮蔽后 z = {}", z);
    let z = "现在是字符串";
    println!("再次遮蔽 z = {}", z);
    
    // 常量
    const MAX_POINTS: u32 = 100_000;
    println!("常量 MAX_POINTS = {}", MAX_POINTS);
}

/// 演示数据类型
fn data_types_demo() {
    println!("\n--- 数据类型 ---");
    
    // 整数类型
    let a: i8 = -128;
    let b: u8 = 255;
    let c: i32 = -2_147_483_648;
    let d: u32 = 4_294_967_295;
    println!("整数: i8={}, u8={}, i32={}, u32={}", a, b, c, d);
    
    // 浮点类型
    let e: f32 = 3.14;
    let f: f64 = 2.718281828;
    println!("浮点: f32={}, f64={}", e, f);
    
    // 布尔类型
    let g: bool = true;
    let h: bool = false;
    println!("布尔: g={}, h={}", g, h);
    
    // 字符类型
    let i: char = 'A';
    let j: char = '中';
    let k: char = '🦀';
    println!("字符: i={}, j={}, k={}", i, j, k);
    
    // 元组类型
    let tup: (i32, f64, u8) = (500, 6.4, 1);
    let (x, y, z) = tup; // 解构
    println!("元组解构: x={}, y={}, z={}", x, y, z);
    println!("元组访问: tup.0={}, tup.1={}, tup.2={}", tup.0, tup.1, tup.2);
    
    // 数组类型
    let arr: [i32; 5] = [1, 2, 3, 4, 5];
    let arr2 = [3; 5]; // [3, 3, 3, 3, 3]
    println!("数组: arr[0]={}, arr2[0]={}", arr[0], arr2[0]);
    println!("数组长度: arr.len()={}", arr.len());
}

/// 演示所有权基础
fn ownership_basics() {
    println!("\n--- 所有权基础 ---");
    
    // 字符串字面量 (存储在栈上)
    let s1 = "hello";
    println!("字符串字面量: {}", s1);
    
    // String 类型 (存储在堆上)
    let mut s2 = String::from("hello");
    s2.push_str(", world!");
    println!("String 类型: {}", s2);
    
    // 移动 (move)
    let s3 = s2;
    println!("移动后: s3 = {}", s3);
    // println!("s2 = {}", s2); // 这行会编译错误，因为 s2 已被移动
    
    // 克隆 (clone)
    let s4 = s3.clone();
    println!("克隆: s3 = {}, s4 = {}", s3, s4);
    
    // 函数调用中的所有权转移
    let s5 = String::from("ownership");
    takes_ownership(s5);
    // println!("s5 = {}", s5); // 这行会编译错误
    
    let x = 5;
    makes_copy(x);
    println!("x 仍然有效: {}", x); // 基本类型实现了 Copy trait
}

/// 接受所有权的函数
fn takes_ownership(some_string: String) {
    println!("函数接受所有权: {}", some_string);
} // some_string 在这里被丢弃

/// 复制值的函数
fn makes_copy(some_integer: i32) {
    println!("函数复制值: {}", some_integer);
} // some_integer 在这里超出作用域，但没有特殊处理

/// 演示引用和借用
fn references_and_borrowing() {
    println!("\n--- 引用和借用 ---");
    
    let s1 = String::from("hello");
    
    // 不可变引用
    let len = calculate_length(&s1);
    println!("字符串 '{}' 的长度是 {}", s1, len);
    
    // 可变引用
    let mut s2 = String::from("hello");
    change(&mut s2);
    println!("修改后的字符串: {}", s2);
    
    // 多个不可变引用
    let r1 = &s1;
    let r2 = &s1;
    println!("多个不可变引用: r1={}, r2={}", r1, r2);
    
    // 引用的作用域
    let mut s3 = String::from("hello");
    {
        let r3 = &mut s3;
        r3.push_str(", world");
    } // r3 在这里超出作用域
    println!("引用作用域结束后: {}", s3);
}

/// 计算字符串长度（借用）
fn calculate_length(s: &String) -> usize {
    s.len()
} // s 超出作用域，但因为它不拥有引用的值，所以不会丢弃任何东西

/// 修改字符串（可变借用）
fn change(some_string: &mut String) {
    some_string.push_str(", world");
}

/// 演示切片
fn slices_demo() {
    println!("\n--- 切片 ---");
    
    let s = String::from("hello world");
    
    // 字符串切片
    let hello = &s[0..5];
    let world = &s[6..11];
    println!("字符串切片: '{}' 和 '{}'", hello, world);
    
    // 简化的切片语法
    let hello2 = &s[..5];  // 等同于 &s[0..5]
    let world2 = &s[6..];  // 等同于 &s[6..11]
    let full = &s[..];     // 等同于 &s[0..11]
    println!("简化语法: '{}', '{}', '{}'", hello2, world2, full);
    
    // 字符串字面量就是切片
    let s_literal = "Hello, world!";
    println!("字符串字面量: {}", s_literal);
    
    // 数组切片
    let a = [1, 2, 3, 4, 5];
    let slice = &a[1..3];
    println!("数组切片: {:?}", slice);
    
    // 使用切片的函数
    let word = first_word(&s);
    println!("第一个单词: '{}'", word);
}

/// 返回字符串中第一个单词的切片
fn first_word(s: &str) -> &str {
    let bytes = s.as_bytes();
    
    for (i, &item) in bytes.iter().enumerate() {
        if item == b' ' {
            return &s[0..i];
        }
    }
    
    &s[..]
}
# 集合类型

## 概述

Rust标准库提供了多种集合类型，用于存储多个值。与数组和元组不同，集合类型的数据存储在堆上，这意味着数据的数量不需要在编译时就知道，并且可以随着程序的运行增长或缩小。本章将深入介绍最常用的集合类型：Vector、HashMap、字符串处理和迭代器。

## 学习目标

- 掌握Vector的创建、操作和使用场景
- 理解HashMap的工作原理和常用操作
- 深入学习Rust字符串处理的各种方法
- 掌握迭代器的使用和函数式编程技巧
- 学会选择合适的集合类型解决实际问题
- 理解集合类型的性能特征和内存管理

## 1. Vector详解

### 1.1 Vector基础

```rust
fn main() {
    // 创建空的Vector
    let mut v1: Vec<i32> = Vec::new();
    
    // 使用vec!宏创建Vector
    let mut v2 = vec![1, 2, 3, 4, 5];
    
    // 指定容量创建Vector
    let mut v3 = Vec::with_capacity(10);
    
    // 添加元素
    v1.push(1);
    v1.push(2);
    v1.push(3);
    
    println!("v1: {:?}", v1);
    println!("v2: {:?}", v2);
    println!("v3 capacity: {}", v3.capacity());
}
```

### 1.2 访问Vector元素

```rust
fn main() {
    let v = vec![1, 2, 3, 4, 5];
    
    // 使用索引访问（可能panic）
    let third = v[2];
    println!("The third element is {}", third);
    
    // 使用get方法安全访问
    match v.get(2) {
        Some(third) => println!("The third element is {}", third),
        None => println!("There is no third element."),
    }
    
    // 访问不存在的元素
    match v.get(100) {
        Some(element) => println!("Element: {}", element),
        None => println!("No element at index 100"),
    }
    
    // 获取第一个和最后一个元素
    if let Some(first) = v.first() {
        println!("First element: {}", first);
    }
    
    if let Some(last) = v.last() {
        println!("Last element: {}", last);
    }
}
```

### 1.3 遍历Vector

```rust
fn main() {
    let mut v = vec![100, 32, 57];
    
    // 不可变引用遍历
    for i in &v {
        println!("{}", i);
    }
    
    // 可变引用遍历并修改
    for i in &mut v {
        *i += 50;
    }
    
    println!("Modified vector: {:?}", v);
    
    // 获取索引和值
    for (index, value) in v.iter().enumerate() {
        println!("Index: {}, Value: {}", index, value);
    }
    
    // 消费Vector进行遍历
    for i in v {
        println!("Consuming: {}", i);
    }
    // println!("{:?}", v); // 错误！v已被消费
}
```

### 1.4 Vector的常用方法

```rust
fn main() {
    let mut v = vec![1, 2, 3, 4, 5];
    
    // 长度和容量
    println!("Length: {}", v.len());
    println!("Capacity: {}", v.capacity());
    println!("Is empty: {}", v.is_empty());
    
    // 添加和删除元素
    v.push(6);
    println!("After push: {:?}", v);
    
    if let Some(last) = v.pop() {
        println!("Popped: {}", last);
    }
    println!("After pop: {:?}", v);
    
    // 插入和删除指定位置的元素
    v.insert(2, 99);
    println!("After insert: {:?}", v);
    
    let removed = v.remove(2);
    println!("Removed: {}, Vector: {:?}", removed, v);
    
    // 清空Vector
    v.clear();
    println!("After clear: {:?}", v);
    
    // 重新填充
    v.extend([1, 2, 3, 4, 5]);
    println!("After extend: {:?}", v);
    
    // 截断Vector
    v.truncate(3);
    println!("After truncate: {:?}", v);
    
    // 保留满足条件的元素
    v.retain(|&x| x > 2);
    println!("After retain: {:?}", v);
}
```

### 1.5 Vector的切片操作

```rust
fn main() {
    let v = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    
    // 获取切片
    let slice1 = &v[2..5];      // [3, 4, 5]
    let slice2 = &v[..3];       // [1, 2, 3]
    let slice3 = &v[7..];       // [8, 9, 10]
    let slice4 = &v[..];        // 整个vector
    
    println!("slice1: {:?}", slice1);
    println!("slice2: {:?}", slice2);
    println!("slice3: {:?}", slice3);
    println!("slice4: {:?}", slice4);
    
    // 使用chunks分块
    for chunk in v.chunks(3) {
        println!("Chunk: {:?}", chunk);
    }
    
    // 使用windows滑动窗口
    for window in v.windows(3) {
        println!("Window: {:?}", window);
    }
    
    // 分割Vector
    let (left, right) = v.split_at(5);
    println!("Left: {:?}, Right: {:?}", left, right);
}
```

### 1.6 存储不同类型的Vector

```rust
// 使用枚举存储不同类型
#[derive(Debug)]
enum SpreadsheetCell {
    Int(i32),
    Float(f64),
    Text(String),
}

fn main() {
    let row = vec![
        SpreadsheetCell::Int(3),
        SpreadsheetCell::Text(String::from("blue")),
        SpreadsheetCell::Float(10.12),
    ];
    
    for cell in &row {
        match cell {
            SpreadsheetCell::Int(i) => println!("Integer: {}", i),
            SpreadsheetCell::Float(f) => println!("Float: {}", f),
            SpreadsheetCell::Text(s) => println!("Text: {}", s),
        }
    }
    
    // 使用trait对象存储不同类型
    trait Drawable {
        fn draw(&self);
    }
    
    struct Circle { radius: f64 }
    struct Rectangle { width: f64, height: f64 }
    
    impl Drawable for Circle {
        fn draw(&self) {
            println!("Drawing circle with radius {}", self.radius);
        }
    }
    
    impl Drawable for Rectangle {
        fn draw(&self) {
            println!("Drawing rectangle {}x{}", self.width, self.height);
        }
    }
    
    let shapes: Vec<Box<dyn Drawable>> = vec![
        Box::new(Circle { radius: 5.0 }),
        Box::new(Rectangle { width: 10.0, height: 20.0 }),
    ];
    
    for shape in &shapes {
        shape.draw();
    }
}
```

## 2. HashMap详解

### 2.1 HashMap基础

```rust
use std::collections::HashMap;

fn main() {
    // 创建空的HashMap
    let mut scores = HashMap::new();
    
    // 插入键值对
    scores.insert(String::from("Blue"), 10);
    scores.insert(String::from("Yellow"), 50);
    
    println!("Scores: {:?}", scores);
    
    // 使用collect方法创建HashMap
    let teams = vec![String::from("Blue"), String::from("Yellow")];
    let initial_scores = vec![10, 50];
    
    let scores2: HashMap<_, _> = teams.into_iter()
        .zip(initial_scores.into_iter())
        .collect();
    
    println!("Scores2: {:?}", scores2);
    
    // 使用from方法创建HashMap
    let scores3 = HashMap::from([
        ("Blue".to_string(), 10),
        ("Yellow".to_string(), 50),
    ]);
    
    println!("Scores3: {:?}", scores3);
}
```

### 2.2 访问HashMap中的值

```rust
use std::collections::HashMap;

fn main() {
    let mut scores = HashMap::new();
    scores.insert(String::from("Blue"), 10);
    scores.insert(String::from("Yellow"), 50);
    
    // 使用get方法获取值
    let team_name = String::from("Blue");
    let score = scores.get(&team_name);
    
    match score {
        Some(s) => println!("Blue team score: {}", s),
        None => println!("Blue team not found"),
    }
    
    // 使用get_or_insert获取或插入默认值
    let blue_score = scores.entry(String::from("Blue")).or_insert(0);
    println!("Blue score: {}", blue_score);
    
    let red_score = scores.entry(String::from("Red")).or_insert(25);
    println!("Red score: {}", red_score);
    
    println!("All scores: {:?}", scores);
    
    // 检查键是否存在
    if scores.contains_key("Blue") {
        println!("Blue team exists");
    }
    
    // 获取键值对的数量
    println!("Number of teams: {}", scores.len());
}
```

### 2.3 遍历HashMap

```rust
use std::collections::HashMap;

fn main() {
    let mut scores = HashMap::new();
    scores.insert(String::from("Blue"), 10);
    scores.insert(String::from("Yellow"), 50);
    scores.insert(String::from("Red"), 25);
    
    // 遍历键值对
    for (key, value) in &scores {
        println!("{}: {}", key, value);
    }
    
    // 只遍历键
    for key in scores.keys() {
        println!("Team: {}", key);
    }
    
    // 只遍历值
    for value in scores.values() {
        println!("Score: {}", value);
    }
    
    // 可变遍历值
    for value in scores.values_mut() {
        *value += 10;
    }
    
    println!("Updated scores: {:?}", scores);
}
```

### 2.4 更新HashMap

```rust
use std::collections::HashMap;

fn main() {
    let mut scores = HashMap::new();
    
    // 覆盖值
    scores.insert(String::from("Blue"), 10);
    scores.insert(String::from("Blue"), 25);  // 覆盖之前的值
    println!("Scores: {:?}", scores);
    
    // 只在键不存在时插入
    scores.entry(String::from("Yellow")).or_insert(50);
    scores.entry(String::from("Blue")).or_insert(50);  // 不会覆盖
    println!("Scores after or_insert: {:?}", scores);
    
    // 根据旧值更新
    let text = "hello world wonderful world";
    let mut map = HashMap::new();
    
    for word in text.split_whitespace() {
        let count = map.entry(word).or_insert(0);
        *count += 1;
    }
    
    println!("Word count: {:?}", map);
    
    // 使用and_modify进行条件更新
    let mut scores = HashMap::new();
    scores.insert("Blue", 10);
    
    scores.entry("Blue")
        .and_modify(|e| *e += 1)
        .or_insert(42);
    
    scores.entry("Yellow")
        .and_modify(|e| *e += 1)
        .or_insert(42);
    
    println!("Final scores: {:?}", scores);
}
```

### 2.5 HashMap的高级操作

```rust
use std::collections::HashMap;

fn main() {
    let mut map1 = HashMap::from([
        ("a", 1),
        ("b", 2),
        ("c", 3),
    ]);
    
    let map2 = HashMap::from([
        ("b", 20),
        ("c", 30),
        ("d", 40),
    ]);
    
    // 合并两个HashMap
    for (key, value) in map2 {
        map1.entry(key).and_modify(|e| *e += value).or_insert(value);
    }
    
    println!("Merged map: {:?}", map1);
    
    // 删除元素
    if let Some(removed) = map1.remove("a") {
        println!("Removed value: {}", removed);
    }
    
    // 保留满足条件的元素
    map1.retain(|&k, &mut v| k != "b" && v > 10);
    println!("After retain: {:?}", map1);
    
    // 清空HashMap
    map1.clear();
    println!("After clear: {:?}", map1);
    
    // 容量管理
    let mut map = HashMap::with_capacity(10);
    println!("Initial capacity: {}", map.capacity());
    
    for i in 0..5 {
        map.insert(i, i * 2);
    }
    
    println!("Map: {:?}", map);
    map.shrink_to_fit();
    println!("Capacity after shrink: {}", map.capacity());
}
```

### 2.6 自定义键类型

```rust
use std::collections::HashMap;
use std::hash::{Hash, Hasher};

#[derive(Debug, Eq, PartialEq)]
struct Person {
    name: String,
    age: u32,
}

impl Hash for Person {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.name.hash(state);
        self.age.hash(state);
    }
}

fn main() {
    let mut people = HashMap::new();
    
    let person1 = Person {
        name: "Alice".to_string(),
        age: 30,
    };
    
    let person2 = Person {
        name: "Bob".to_string(),
        age: 25,
    };
    
    people.insert(person1, "Engineer");
    people.insert(person2, "Designer");
    
    println!("People: {:?}", people);
    
    // 查找特定的人
    let alice = Person {
        name: "Alice".to_string(),
        age: 30,
    };
    
    if let Some(job) = people.get(&alice) {
        println!("Alice's job: {}", job);
    }
}
```

## 3. 字符串处理

### 3.1 字符串类型概述

```rust
fn main() {
    // String类型（可变，拥有所有权）
    let mut s1 = String::new();
    let s2 = String::from("hello");
    let s3 = "world".to_string();
    
    // &str类型（字符串切片，不可变引用）
    let s4: &str = "hello world";
    let s5: &str = &s2;  // String的引用
    
    println!("s1: '{}', s2: '{}', s3: '{}', s4: '{}', s5: '{}'", s1, s2, s3, s4, s5);
    
    // 字符串字面量
    let literal = "This is a string literal";
    
    // 原始字符串
    let raw_string = r"This is a raw string with \n and \t";
    let raw_multiline = r#"
        This is a raw string
        that spans multiple lines
        and can contain "quotes"
    "#;
    
    println!("Literal: {}", literal);
    println!("Raw: {}", raw_string);
    println!("Raw multiline: {}", raw_multiline);
}
```

### 3.2 字符串创建和转换

```rust
fn main() {
    // 不同的创建方式
    let s1 = String::new();
    let s2 = String::from("hello");
    let s3 = "hello".to_string();
    let s4 = "hello".to_owned();
    let s5 = String::with_capacity(10);
    
    // 从其他类型转换
    let number = 42;
    let s_from_number = number.to_string();
    let s_from_format = format!("The number is {}", number);
    
    // 从字节创建
    let bytes = vec![72, 101, 108, 108, 111]; // "Hello"
    let s_from_bytes = String::from_utf8(bytes).unwrap();
    
    // 从字节（不安全）
    let bytes_unsafe = vec![72, 101, 108, 108, 111];
    let s_from_bytes_unsafe = unsafe {
        String::from_utf8_unchecked(bytes_unsafe)
    };
    
    println!("From number: {}", s_from_number);
    println!("From format: {}", s_from_format);
    println!("From bytes: {}", s_from_bytes);
    println!("From bytes unsafe: {}", s_from_bytes_unsafe);
}
```

### 3.3 字符串操作

```rust
fn main() {
    let mut s = String::from("Hello");
    
    // 追加字符串
    s.push_str(", world");
    println!("After push_str: {}", s);
    
    // 追加字符
    s.push('!');
    println!("After push: {}", s);
    
    // 插入字符串
    s.insert_str(5, " beautiful");
    println!("After insert_str: {}", s);
    
    // 插入字符
    s.insert(0, '*');
    println!("After insert: {}", s);
    
    // 替换
    let s2 = s.replace("beautiful", "wonderful");
    println!("After replace: {}", s2);
    
    // 替换第一个匹配
    let s3 = s.replacen("l", "L", 2);
    println!("After replacen: {}", s3);
    
    // 删除
    let mut s4 = String::from("Hello, world!");
    s4.remove(5);  // 删除逗号
    println!("After remove: {}", s4);
    
    // 截断
    s4.truncate(5);
    println!("After truncate: {}", s4);
    
    // 清空
    let mut s5 = String::from("Hello");
    s5.clear();
    println!("After clear: '{}'", s5);
}
```

### 3.4 字符串连接

```rust
fn main() {
    // 使用+操作符
    let s1 = String::from("Hello, ");
    let s2 = String::from("world!");
    let s3 = s1 + &s2;  // s1被移动，不能再使用
    println!("s3: {}", s3);
    // println!("s1: {}", s1); // 错误！s1已被移动
    
    // 连接多个字符串
    let s1 = String::from("tic");
    let s2 = String::from("tac");
    let s3 = String::from("toe");
    let s = s1 + "-" + &s2 + "-" + &s3;
    println!("Connected: {}", s);
    
    // 使用format!宏
    let s1 = String::from("tic");
    let s2 = String::from("tac");
    let s3 = String::from("toe");
    let s = format!("{}-{}-{}", s1, s2, s3);
    println!("Formatted: {}", s);
    // s1, s2, s3仍然可用
    
    // 使用join方法
    let words = vec!["Hello", "beautiful", "world"];
    let sentence = words.join(" ");
    println!("Joined: {}", sentence);
    
    // 使用collect方法
    let words = vec!["Rust", "is", "awesome"];
    let sentence: String = words.iter()
        .map(|&s| s)
        .collect::<Vec<&str>>()
        .join(" ");
    println!("Collected: {}", sentence);
}
```

### 3.5 字符串切片和索引

```rust
fn main() {
    let s = String::from("Hello, 世界");
    
    // 字符串切片
    let hello = &s[0..5];
    println!("Slice: {}", hello);
    
    // 注意：不能直接索引字符串
    // let h = s[0]; // 错误！
    
    // 获取字符串长度（字节数）
    println!("Byte length: {}", s.len());
    
    // 获取字符数
    println!("Char count: {}", s.chars().count());
    
    // 遍历字符
    for c in s.chars() {
        println!("Char: {}", c);
    }
    
    // 遍历字节
    for b in s.bytes() {
        println!("Byte: {}", b);
    }
    
    // 获取字符索引
    for (i, c) in s.char_indices() {
        println!("Index: {}, Char: {}", i, c);
    }
    
    // 安全的字符串切片
    let s = "Hello, 世界";
    if let Some(end) = s.char_indices().nth(7).map(|(i, _)| i) {
        let slice = &s[..end];
        println!("Safe slice: {}", slice);
    }
}
```

### 3.6 字符串搜索和匹配

```rust
fn main() {
    let s = "The quick brown fox jumps over the lazy dog";
    
    // 检查是否包含子字符串
    if s.contains("fox") {
        println!("Found 'fox' in the string");
    }
    
    // 检查开头和结尾
    if s.starts_with("The") {
        println!("String starts with 'The'");
    }
    
    if s.ends_with("dog") {
        println!("String ends with 'dog'");
    }
    
    // 查找子字符串位置
    if let Some(pos) = s.find("fox") {
        println!("'fox' found at position: {}", pos);
    }
    
    // 查找最后一个匹配
    if let Some(pos) = s.rfind("the") {
        println!("Last 'the' found at position: {}", pos);
    }
    
    // 查找所有匹配
    let matches: Vec<_> = s.match_indices("the").collect();
    println!("All 'the' matches: {:?}", matches);
    
    // 分割字符串
    let words: Vec<&str> = s.split_whitespace().collect();
    println!("Words: {:?}", words);
    
    let parts: Vec<&str> = s.split(" the ").collect();
    println!("Split by ' the ': {:?}", parts);
    
    // 按行分割
    let multiline = "line1\nline2\nline3";
    let lines: Vec<&str> = multiline.lines().collect();
    println!("Lines: {:?}", lines);
}
```

### 3.7 字符串格式化

```rust
fn main() {
    let name = "Alice";
    let age = 30;
    let height = 1.65;
    
    // 基本格式化
    let intro = format!("My name is {} and I'm {} years old", name, age);
    println!("{}", intro);
    
    // 位置参数
    let msg = format!("{0} is {1} years old. {0} likes programming.", name, age);
    println!("{}", msg);
    
    // 命名参数
    let msg = format!("{name} is {age} years old and {height}m tall", 
                     name=name, age=age, height=height);
    println!("{}", msg);
    
    // 格式化选项
    println!("Decimal: {}", 42);
    println!("Binary: {:b}", 42);
    println!("Octal: {:o}", 42);
    println!("Hexadecimal: {:x}", 42);
    println!("Hexadecimal (uppercase): {:X}", 42);
    
    // 浮点数格式化
    println!("Float: {}", 3.14159);
    println!("Float (2 decimals): {:.2}", 3.14159);
    println!("Float (scientific): {:e}", 3.14159);
    
    // 填充和对齐
    println!("Left aligned: '{:<10}'", "hello");
    println!("Right aligned: '{:>10}'", "hello");
    println!("Center aligned: '{:^10}'", "hello");
    println!("Zero padded: '{:0>10}'", 42);
    
    // 调试格式化
    let vec = vec![1, 2, 3];
    println!("Debug: {:?}", vec);
    println!("Pretty debug: {:#?}", vec);
}
```

## 4. 迭代器详解

### 4.1 迭代器基础

```rust
fn main() {
    let v = vec![1, 2, 3, 4, 5];
    
    // 创建迭代器
    let iter = v.iter();  // 不可变引用迭代器
    
    // 使用for循环消费迭代器
    for item in iter {
        println!("Item: {}", item);
    }
    
    // 可变引用迭代器
    let mut v2 = vec![1, 2, 3, 4, 5];
    for item in v2.iter_mut() {
        *item *= 2;
    }
    println!("Modified: {:?}", v2);
    
    // 获取所有权的迭代器
    let v3 = vec![1, 2, 3, 4, 5];
    for item in v3.into_iter() {
        println!("Owned: {}", item);
    }
    // println!("{:?}", v3); // 错误！v3已被消费
    
    // 手动使用迭代器
    let v4 = vec![1, 2, 3];
    let mut iter = v4.iter();
    
    println!("First: {:?}", iter.next());
    println!("Second: {:?}", iter.next());
    println!("Third: {:?}", iter.next());
    println!("Fourth: {:?}", iter.next()); // None
}
```

### 4.2 迭代器适配器

```rust
fn main() {
    let v = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    
    // map - 转换每个元素
    let doubled: Vec<i32> = v.iter().map(|x| x * 2).collect();
    println!("Doubled: {:?}", doubled);
    
    // filter - 过滤元素
    let evens: Vec<&i32> = v.iter().filter(|&x| x % 2 == 0).collect();
    println!("Evens: {:?}", evens);
    
    // enumerate - 添加索引
    let with_index: Vec<(usize, &i32)> = v.iter().enumerate().collect();
    println!("With index: {:?}", with_index);
    
    // zip - 组合两个迭代器
    let names = vec!["Alice", "Bob", "Charlie"];
    let ages = vec![30, 25, 35];
    let people: Vec<(&str, &i32)> = names.iter().zip(ages.iter()).collect();
    println!("People: {:?}", people);
    
    // take - 取前n个元素
    let first_three: Vec<&i32> = v.iter().take(3).collect();
    println!("First three: {:?}", first_three);
    
    // skip - 跳过前n个元素
    let skip_three: Vec<&i32> = v.iter().skip(3).collect();
    println!("Skip three: {:?}", skip_three);
    
    // chain - 连接迭代器
    let v2 = vec![11, 12, 13];
    let chained: Vec<&i32> = v.iter().chain(v2.iter()).collect();
    println!("Chained: {:?}", chained);
    
    // rev - 反转迭代器
    let reversed: Vec<&i32> = v.iter().rev().collect();
    println!("Reversed: {:?}", reversed);
}
```

### 4.3 迭代器消费器

```rust
fn main() {
    let v = vec![1, 2, 3, 4, 5];
    
    // collect - 收集到集合
    let doubled: Vec<i32> = v.iter().map(|x| x * 2).collect();
    println!("Collected: {:?}", doubled);
    
    // reduce/fold - 累积操作
    let sum = v.iter().fold(0, |acc, x| acc + x);
    println!("Sum: {}", sum);
    
    let product = v.iter().reduce(|acc, x| acc * x);
    println!("Product: {:?}", product);
    
    // find - 查找第一个匹配的元素
    let found = v.iter().find(|&&x| x > 3);
    println!("Found: {:?}", found);
    
    // any/all - 检查条件
    let has_even = v.iter().any(|&x| x % 2 == 0);
    let all_positive = v.iter().all(|&x| x > 0);
    println!("Has even: {}, All positive: {}", has_even, all_positive);
    
    // count - 计数
    let count = v.iter().filter(|&&x| x > 2).count();
    println!("Count > 2: {}", count);
    
    // min/max - 最小值/最大值
    let min = v.iter().min();
    let max = v.iter().max();
    println!("Min: {:?}, Max: {:?}", min, max);
    
    // for_each - 对每个元素执行操作
    v.iter().for_each(|x| println!("Processing: {}", x));
    
    // partition - 分割
    let (evens, odds): (Vec<&i32>, Vec<&i32>) = v.iter()
        .partition(|&&x| x % 2 == 0);
    println!("Evens: {:?}, Odds: {:?}", evens, odds);
}
```

### 4.4 自定义迭代器

```rust
struct Counter {
    current: usize,
    max: usize,
}

impl Counter {
    fn new(max: usize) -> Counter {
        Counter { current: 0, max }
    }
}

impl Iterator for Counter {
    type Item = usize;
    
    fn next(&mut self) -> Option<Self::Item> {
        if self.current < self.max {
            let current = self.current;
            self.current += 1;
            Some(current)
        } else {
            None
        }
    }
}

fn main() {
    let counter = Counter::new(5);
    
    for num in counter {
        println!("Count: {}", num);
    }
    
    // 使用自定义迭代器的适配器
    let sum: usize = Counter::new(5)
        .map(|x| x * x)
        .filter(|&x| x > 5)
        .sum();
    
    println!("Sum of squares > 5: {}", sum);
    
    // 实现更复杂的迭代器
    struct Fibonacci {
        current: u64,
        next: u64,
    }
    
    impl Fibonacci {
        fn new() -> Fibonacci {
            Fibonacci { current: 0, next: 1 }
        }
    }
    
    impl Iterator for Fibonacci {
        type Item = u64;
        
        fn next(&mut self) -> Option<Self::Item> {
            let current = self.current;
            self.current = self.next;
            self.next = current + self.next;
            Some(current)
        }
    }
    
    // 生成前10个斐波那契数
    let fib_numbers: Vec<u64> = Fibonacci::new().take(10).collect();
    println!("Fibonacci: {:?}", fib_numbers);
}
```

### 4.5 迭代器性能优化

```rust
fn main() {
    let large_vec: Vec<i32> = (0..1_000_000).collect();
    
    // 使用迭代器链式操作（零成本抽象）
    let result: Vec<i32> = large_vec
        .iter()
        .filter(|&&x| x % 2 == 0)
        .map(|x| x * x)
        .take(1000)
        .cloned()
        .collect();
    
    println!("Processed {} items", result.len());
    
    // 惰性求值示例
    let iter = (0..1_000_000)
        .filter(|&x| x % 2 == 0)
        .map(|x| x * x);
    
    // 此时还没有进行任何计算
    println!("Iterator created");
    
    // 只有在消费时才进行计算
    let first_five: Vec<i32> = iter.take(5).collect();
    println!("First five: {:?}", first_five);
    
    // 使用迭代器避免中间集合
    fn sum_of_squares_bad(v: &[i32]) -> i32 {
        v.iter()
            .map(|x| x * x)
            .collect::<Vec<_>>()  // 创建中间向量
            .iter()
            .sum()
    }
    
    fn sum_of_squares_good(v: &[i32]) -> i32 {
        v.iter()
            .map(|x| x * x)
            .sum()  // 直接求和，无中间集合
    }
    
    let test_vec = vec![1, 2, 3, 4, 5];
    println!("Sum (bad): {}", sum_of_squares_bad(&test_vec));
    println!("Sum (good): {}", sum_of_squares_good(&test_vec));
}
```

## 5. 实践示例

### 5.1 文本分析器

```rust
use std::collections::HashMap;

struct TextAnalyzer {
    text: String,
}

impl TextAnalyzer {
    fn new(text: String) -> Self {
        TextAnalyzer { text }
    }
    
    fn word_count(&self) -> HashMap<String, usize> {
        self.text
            .split_whitespace()
            .map(|word| word.to_lowercase().trim_matches(|c: char| !c.is_alphabetic()).to_string())
            .filter(|word| !word.is_empty())
            .fold(HashMap::new(), |mut acc, word| {
                *acc.entry(word).or_insert(0) += 1;
                acc
            })
    }
    
    fn character_frequency(&self) -> HashMap<char, usize> {
        self.text
            .chars()
            .filter(|c| c.is_alphabetic())
            .map(|c| c.to_lowercase().next().unwrap())
            .fold(HashMap::new(), |mut acc, c| {
                *acc.entry(c).or_insert(0) += 1;
                acc
            })
    }
    
    fn longest_words(&self, n: usize) -> Vec<String> {
        let mut words: Vec<String> = self.text
            .split_whitespace()
            .map(|word| word.trim_matches(|c: char| !c.is_alphabetic()).to_string())
            .filter(|word| !word.is_empty())
            .collect();
        
        words.sort_by(|a, b| b.len().cmp(&a.len()));
        words.truncate(n);
        words
    }
    
    fn statistics(&self) -> (usize, usize, usize, usize) {
        let char_count = self.text.chars().count();
        let word_count = self.text.split_whitespace().count();
        let line_count = self.text.lines().count();
        let sentence_count = self.text.matches(&['.', '!', '?'][..]).count();
        
        (char_count, word_count, line_count, sentence_count)
    }
}

fn main() {
    let text = "Hello world! This is a sample text for analysis. \
                Hello appears twice in this text. \
                We will analyze word frequency and other statistics.";
    
    let analyzer = TextAnalyzer::new(text.to_string());
    
    // 词频统计
    let word_counts = analyzer.word_count();
    println!("Word frequencies:");
    for (word, count) in &word_counts {
        println!("  {}: {}", word, count);
    }
    
    // 字符频率
    let char_freq = analyzer.character_frequency();
    println!("\nCharacter frequencies:");
    let mut chars: Vec<_> = char_freq.iter().collect();
    chars.sort_by(|a, b| b.1.cmp(a.1));
    for (ch, count) in chars.iter().take(5) {
        println!("  {}: {}", ch, count);
    }
    
    // 最长的单词
    let longest = analyzer.longest_words(3);
    println!("\nLongest words: {:?}", longest);
    
    // 统计信息
    let (chars, words, lines, sentences) = analyzer.statistics();
    println!("\nStatistics:");
    println!("  Characters: {}", chars);
    println!("  Words: {}", words);
    println!("  Lines: {}", lines);
    println!("  Sentences: {}", sentences);
}
```

### 5.2 数据处理管道

```rust
use std::collections::HashMap;

#[derive(Debug, Clone)]
struct Person {
    name: String,
    age: u32,
    city: String,
    salary: u32,
}

struct DataProcessor {
    people: Vec<Person>,
}

impl DataProcessor {
    fn new() -> Self {
        DataProcessor { people: Vec::new() }
    }
    
    fn add_person(&mut self, person: Person) {
        self.people.push(person);
    }
    
    fn load_sample_data(&mut self) {
        let sample_data = vec![
            Person { name: "Alice".to_string(), age: 30, city: "New York".to_string(), salary: 75000 },
            Person { name: "Bob".to_string(), age: 25, city: "San Francisco".to_string(), salary: 85000 },
            Person { name: "Charlie".to_string(), age: 35, city: "New York".to_string(), salary: 65000 },
            Person { name: "Diana".to_string(), age: 28, city: "Los Angeles".to_string(), salary: 70000 },
            Person { name: "Eve".to_string(), age: 32, city: "San Francisco".to_string(), salary: 90000 },
        ];
        
        self.people.extend(sample_data);
    }
    
    // 按城市分组
    fn group_by_city(&self) -> HashMap<String, Vec<&Person>> {
        self.people
            .iter()
            .fold(HashMap::new(), |mut acc, person| {
                acc.entry(person.city.clone()).or_insert_with(Vec::new).push(person);
                acc
            })
    }
    
    // 计算平均工资
    fn average_salary(&self) -> f64 {
        if self.people.is_empty() {
            return 0.0;
        }
        
        let total: u32 = self.people.iter().map(|p| p.salary).sum();
        total as f64 / self.people.len() as f64
    }
    
    // 按城市计算平均工资
    fn average_salary_by_city(&self) -> HashMap<String, f64> {
        self.group_by_city()
            .into_iter()
            .map(|(city, people)| {
                let avg = people.iter().map(|p| p.salary).sum::<u32>() as f64 / people.len() as f64;
                (city, avg)
            })
            .collect()
    }
    
    // 过滤和排序
    fn high_earners(&self, threshold: u32) -> Vec<&Person> {
        let mut high_earners: Vec<&Person> = self.people
            .iter()
            .filter(|p| p.salary > threshold)
            .collect();
        
        high_earners.sort_by(|a, b| b.salary.cmp(&a.salary));
        high_earners
    }
    
    // 年龄分布
    fn age_distribution(&self) -> HashMap<String, usize> {
        self.people
            .iter()
            .map(|p| {
                match p.age {
                    0..=25 => "18-25",
                    26..=35 => "26-35",
                    36..=45 => "36-45",
                    _ => "46+",
                }
            })
            .fold(HashMap::new(), |mut acc, age_group| {
                *acc.entry(age_group.to_string()).or_insert(0) += 1;
                acc
            })
    }
    
    // 复杂查询：每个城市最高薪水的人
    fn highest_earner_by_city(&self) -> HashMap<String, &Person> {
        self.group_by_city()
            .into_iter()
            .filter_map(|(city, people)| {
                people.iter().max_by_key(|p| p.salary).map(|p| (city, *p))
            })
            .collect()
    }
}

fn main() {
    let mut processor = DataProcessor::new();
    processor.load_sample_data();
    
    println!("=== Data Analysis Results ===\n");
    
    // 总体统计
    println!("Total people: {}", processor.people.len());
    println!("Average salary: ${:.2}", processor.average_salary());
    
    // 按城市分组
    println!("\n--- People by City ---");
    for (city, people) in processor.group_by_city() {
        println!("{}: {} people", city, people.len());
    }
    
    // 城市平均工资
    println!("\n--- Average Salary by City ---");
    for (city, avg_salary) in processor.average_salary_by_city() {
        println!("{}: ${:.2}", city, avg_salary);
    }
    
    // 高收入者
    println!("\n--- High Earners (>$70,000) ---");
    for person in processor.high_earners(70000) {
        println!("{}: ${} ({})", person.name, person.salary, person.city);
    }
    
    // 年龄分布
    println!("\n--- Age Distribution ---");
    for (age_group, count) in processor.age_distribution() {
        println!("{}: {} people", age_group, count);
    }
    
    // 每个城市最高薪水
    println!("\n--- Highest Earner by City ---");
    for (city, person) in processor.highest_earner_by_city() {
        println!("{}: {} (${:})", city, person.name, person.salary);
    }
}
```

## 6. 性能考虑

### 6.1 集合类型选择指南

```rust
use std::collections::{HashMap, BTreeMap, HashSet, BTreeSet, VecDeque};
use std::time::Instant;

fn performance_comparison() {
    let n = 100_000;
    
    // Vector vs VecDeque
    println!("=== Vector vs VecDeque ===");
    
    // Vector - 在末尾添加元素
    let start = Instant::now();
    let mut vec = Vec::new();
    for i in 0..n {
        vec.push(i);
    }
    println!("Vector push: {:?}", start.elapsed());
    
    // VecDeque - 在两端添加元素
    let start = Instant::now();
    let mut deque = VecDeque::new();
    for i in 0..n {
        if i % 2 == 0 {
            deque.push_back(i);
        } else {
            deque.push_front(i);
        }
    }
    println!("VecDeque push both ends: {:?}", start.elapsed());
    
    // HashMap vs BTreeMap
    println!("\n=== HashMap vs BTreeMap ===");
    
    let start = Instant::now();
    let mut hash_map = HashMap::new();
    for i in 0..n {
        hash_map.insert(i, i * 2);
    }
    println!("HashMap insert: {:?}", start.elapsed());
    
    let start = Instant::now();
    let mut btree_map = BTreeMap::new();
    for i in 0..n {
        btree_map.insert(i, i * 2);
    }
    println!("BTreeMap insert: {:?}", start.elapsed());
    
    // 查找性能
    let start = Instant::now();
    for i in 0..1000 {
        hash_map.get(&(i * 100));
    }
    println!("HashMap lookup: {:?}", start.elapsed());
    
    let start = Instant::now();
    for i in 0..1000 {
        btree_map.get(&(i * 100));
    }
    println!("BTreeMap lookup: {:?}", start.elapsed());
}

fn main() {
    performance_comparison();
    
    // 选择指南
    println!("\n=== Collection Selection Guide ===");
    println!("Vector: 顺序访问，末尾操作，已知大小");
    println!("VecDeque: 双端队列，两端操作");
    println!("HashMap: 快速查找，无序，哈希键");
    println!("BTreeMap: 有序映射，范围查询");
    println!("HashSet: 快速去重，无序");
    println!("BTreeSet: 有序集合，范围操作");
}
```

### 6.2 内存优化技巧

```rust
fn memory_optimization_examples() {
    // 预分配容量
    let mut vec_with_capacity = Vec::with_capacity(1000);
    let mut vec_without_capacity = Vec::new();
    
    println!("Initial capacities:");
    println!("With capacity: {}", vec_with_capacity.capacity());
    println!("Without capacity: {}", vec_without_capacity.capacity());
    
    // 字符串优化
    // 使用&str而不是String（当不需要所有权时）
    fn process_text_bad(text: String) -> String {
        text.to_uppercase()
    }
    
    fn process_text_good(text: &str) -> String {
        text.to_uppercase()
    }
    
    // 使用Cow（Clone on Write）
    use std::borrow::Cow;
    
    fn maybe_modify(input: &str, should_modify: bool) -> Cow<str> {
        if should_modify {
            Cow::Owned(input.to_uppercase())
        } else {
            Cow::Borrowed(input)
        }
    }
    
    let text = "hello world";
    let result1 = maybe_modify(text, false);  // 不分配内存
    let result2 = maybe_modify(text, true);   // 分配内存
    
    println!("Result1: {}", result1);
    println!("Result2: {}", result2);
    
    // 使用Box减少栈内存使用
    enum Message {
        Small(u8),
        Large(Box<[u8; 1000]>),  // 大数据放在堆上
    }
    
    // 使用引用计数共享数据
    use std::rc::Rc;
    
    let shared_data = Rc::new(vec![1, 2, 3, 4, 5]);
    let reference1 = Rc::clone(&shared_data);
    let reference2 = Rc::clone(&shared_data);
    
    println!("Reference count: {}", Rc::strong_count(&shared_data));
}
```

## 7. 最佳实践

### 7.1 集合使用最佳实践

```rust
use std::collections::HashMap;

// 好的实践：使用适当的集合类型和方法
fn best_practices_examples() {
    // 1. 预分配容量（当知道大概大小时）
    let mut vec = Vec::with_capacity(100);
    let mut map = HashMap::with_capacity(50);
    
    // 2. 使用entry API避免重复查找
    let mut word_count = HashMap::new();
    let words = vec!["hello", "world", "hello", "rust"];
    
    // 好的方式
    for word in words {
        *word_count.entry(word).or_insert(0) += 1;
    }
    
    // 避免的方式（效率较低）
    // for word in words {
    //     if word_count.contains_key(word) {
    //         *word_count.get_mut(word).unwrap() += 1;
    //     } else {
    //         word_count.insert(word, 1);
    //     }
    // }
    
    // 3. 使用迭代器而不是索引（当可能时）
    let numbers = vec![1, 2, 3, 4, 5];
    
    // 好的方式
    let sum: i32 = numbers.iter().sum();
    
    // 避免的方式（除非需要索引）
    // let mut sum = 0;
    // for i in 0..numbers.len() {
    //     sum += numbers[i];
    // }
    
    // 4. 使用drain避免不必要的克隆
    let mut source = vec![1, 2, 3, 4, 5];
    let mut target = Vec::new();
    
    // 移动元素而不是复制
    target.extend(source.drain(2..4));
    println!("Source: {:?}, Target: {:?}", source, target);
    
    // 5. 使用retain而不是filter + collect（当修改原集合时）
    let mut numbers = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
    numbers.retain(|&x| x % 2 == 0);
    println!("Even numbers: {:?}", numbers);
}

fn main() {
    best_practices_examples();
}
```

### 7.2 错误处理和安全性

```rust
use std::collections::HashMap;

fn safe_collection_operations() {
    let mut map = HashMap::new();
    map.insert("key1", "value1");
    map.insert("key2", "value2");
    
    // 安全的访问方式
    match map.get("key1") {
        Some(value) => println!("Found: {}", value),
        None => println!("Key not found"),
    }
    
    // 使用get_or_insert_with进行惰性初始化
    let value = map.entry("key3").or_insert_with(|| {
        println!("Computing expensive value...");
        "computed_value"
    });
    
    // 安全的Vector操作
    let mut vec = vec![1, 2, 3];
    
    // 安全的pop
    while let Some(item) = vec.pop() {
        println!("Popped: {}", item);
    }
    
    // 安全的索引访问
    let vec = vec![1, 2, 3, 4, 5];
    if let Some(item) = vec.get(10) {
        println!("Item at index 10: {}", item);
    } else {
        println!("Index 10 is out of bounds");
    }
    
    // 使用chunks_exact避免不完整的块
    let data = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
    for chunk in data.chunks_exact(3) {
        println!("Complete chunk: {:?}", chunk);
    }
    
    // 处理剩余元素
    let remainder = data.chunks_exact(3).remainder();
    if !remainder.is_empty() {
        println!("Remainder: {:?}", remainder);
    }
}
```

## 8. 总结

集合类型是Rust编程的基础：

1. **Vector**提供动态数组功能，适合顺序访问和末尾操作
2. **HashMap**提供快速键值查找，适合无序映射
3. **字符串处理**需要理解String和&str的区别，掌握各种操作方法
4. **迭代器**提供函数式编程能力，支持链式操作和惰性求值
5. **性能优化**通过选择合适的集合类型和操作方法
6. **内存管理**通过预分配、引用计数等技术优化内存使用

这些概念在嵌入式开发中特别重要，因为需要高效的数据处理和严格的内存管理。

## 练习题

1. 实现一个LRU（最近最少使用）缓存，使用HashMap和双向链表
2. 创建一个文本搜索引擎，支持多关键词搜索和结果排序
3. 实现一个数据分析工具，处理CSV格式的数据并生成统计报告
4. 设计一个内存高效的图数据结构，支持节点和边的增删改查操作
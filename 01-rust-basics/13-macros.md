# 第13章：宏系统

## 概述

Rust的宏系统是一个强大的元编程工具，允许你在编译时生成代码。与C/C++的预处理器宏不同，Rust的宏是类型安全的，并且在语法树级别操作。Rust提供了两种主要的宏类型：声明式宏（declarative macros）和过程宏（procedural macros）。

## 学习目标

通过本章学习，你将掌握：
- 声明式宏的定义和使用
- 宏模式匹配和重复
- 过程宏的三种类型
- 宏的调试和最佳实践
- 常用宏的实现原理
- 宏的高级技巧和应用场景

## 1. 声明式宏基础

### 1.1 简单宏定义

```rust
// 最简单的宏
macro_rules! say_hello {
    () => {
        println!("Hello, World!");
    };
}

// 带参数的宏
macro_rules! create_function {
    ($func_name:ident) => {
        fn $func_name() {
            println!("You called {:?}()", stringify!($func_name));
        }
    };
}

fn main() {
    say_hello!();
    
    create_function!(foo);
    create_function!(bar);
    
    foo();
    bar();
}
```

### 1.2 宏模式匹配

```rust
macro_rules! test {
    // 匹配单个标识符
    ($left:ident) => {
        println!("Single identifier: {}", stringify!($left));
    };
    
    // 匹配表达式
    ($left:expr) => {
        println!("Expression: {}", $left);
    };
    
    // 匹配多个参数
    ($left:expr; and $right:expr) => {
        println!("Left: {}, Right: {}", $left, $right);
    };
    
    // 匹配类型
    ($left:expr; as $t:ty) => {
        let result: $t = $left;
        println!("Converted to {}: {}", stringify!($t), result);
    };
}

fn main() {
    test!(hello);
    test!(1 + 1);
    test!(1 + 1; and 2 + 2);
    test!(42; as f64);
}
```

### 1.3 重复模式

```rust
// 创建向量的宏
macro_rules! vec_of_strings {
    ($($x:expr),*) => {
        {
            let mut temp_vec = Vec::new();
            $(
                temp_vec.push($x.to_string());
            )*
            temp_vec
        }
    };
}

// 创建哈希映射的宏
macro_rules! hashmap {
    ($($key:expr => $val:expr),*) => {
        {
            let mut map = std::collections::HashMap::new();
            $(
                map.insert($key, $val);
            )*
            map
        }
    };
}

fn main() {
    let v = vec_of_strings!["hello", "world", "rust"];
    println!("{:?}", v);
    
    let map = hashmap! {
        "name" => "Alice",
        "age" => "30",
        "city" => "New York"
    };
    println!("{:?}", map);
}
```

### 1.4 高级模式匹配

```rust
// 支持可选参数的宏
macro_rules! find_min {
    ($x:expr) => ($x);
    ($x:expr, $($y:expr),+) => (
        std::cmp::min($x, find_min!($($y),+))
    );
}

// 支持不同分隔符的宏
macro_rules! calculate {
    (eval $e:expr) => {
        {
            let val: usize = $e;
            println!("{} = {}", stringify!{$e}, val);
        }
    };
    
    (eval $e:expr, $(eval $es:expr),+) => {
        {
            calculate! { eval $e }
            calculate! { $(eval $es),+ }
        }
    };
}

fn main() {
    let min = find_min!(1, 2, 3, 4, 5);
    println!("Minimum: {}", min);
    
    calculate! {
        eval 1 + 2,
        eval 3 * 4,
        eval (2 + 3) * 4
    }
}
```

## 2. 宏的指示符类型

### 2.1 各种指示符

```rust
macro_rules! test_designators {
    // item: 项目（函数、结构体、模块等）
    ($i:item) => {
        $i
    };
    
    // block: 代码块
    ($b:block) => {
        $b
    };
    
    // stmt: 语句
    ($s:stmt) => {
        $s
    };
    
    // pat: 模式
    ($p:pat) => {
        match 42 {
            $p => println!("Pattern matched!"),
            _ => println!("Pattern not matched"),
        }
    };
    
    // expr: 表达式
    ($e:expr) => {
        println!("Expression result: {}", $e);
    };
    
    // ty: 类型
    ($t:ty) => {
        {
            let _: $t = Default::default();
            println!("Type: {}", stringify!($t));
        }
    };
    
    // ident: 标识符
    ($id:ident) => {
        println!("Identifier: {}", stringify!($id));
    };
    
    // path: 路径
    ($path:path) => {
        println!("Path: {}", stringify!($path));
    };
    
    // meta: 元数据
    ($m:meta) => {
        println!("Meta: {}", stringify!($m));
    };
    
    // tt: token tree
    ($tt:tt) => {
        println!("Token tree: {}", stringify!($tt));
    };
}

fn main() {
    // item
    test_designators! {
        fn hello() {
            println!("Hello from macro-generated function!");
        }
    }
    hello();
    
    // block
    test_designators! {
        {
            let x = 5;
            println!("Block result: {}", x);
        }
    }
    
    // expr
    test_designators!(1 + 2 * 3);
    
    // ty
    test_designators!(Vec<i32>);
    
    // ident
    test_designators!(my_variable);
    
    // path
    test_designators!(std::collections::HashMap);
    
    // pat
    test_designators!(42);
    
    // tt
    test_designators!((hello, world));
}
```

### 2.2 复杂的宏示例

```rust
// 实现一个简单的DSL
macro_rules! sql_select {
    (SELECT $($field:ident),+ FROM $table:ident WHERE $condition:expr) => {
        {
            let fields = vec![$(stringify!($field)),+];
            let table = stringify!($table);
            let condition = stringify!($condition);
            
            format!("SELECT {} FROM {} WHERE {}", 
                   fields.join(", "), 
                   table, 
                   condition)
        }
    };
    
    (SELECT $($field:ident),+ FROM $table:ident) => {
        {
            let fields = vec![$(stringify!($field)),+];
            let table = stringify!($table);
            
            format!("SELECT {} FROM {}", 
                   fields.join(", "), 
                   table)
        }
    };
}

// 创建结构体和实现的宏
macro_rules! create_struct_with_new {
    ($name:ident { $($field:ident: $type:ty),* }) => {
        struct $name {
            $($field: $type,)*
        }
        
        impl $name {
            fn new($($field: $type),*) -> Self {
                $name {
                    $($field,)*
                }
            }
            
            fn describe(&self) -> String {
                format!("{} {{ {} }}", 
                        stringify!($name),
                        vec![$(format!("{}: {:?}", stringify!($field), self.$field)),*].join(", "))
            }
        }
    };
}

fn main() {
    // SQL DSL示例
    let query1 = sql_select!(SELECT name, age FROM users WHERE age > 18);
    println!("Query 1: {}", query1);
    
    let query2 = sql_select!(SELECT id, name, email FROM customers);
    println!("Query 2: {}", query2);
    
    // 结构体生成示例
    create_struct_with_new!(Person {
        name: String,
        age: u32,
        email: String
    });
    
    let person = Person::new(
        "Alice".to_string(),
        30,
        "alice@example.com".to_string()
    );
    
    println!("{}", person.describe());
}
```

## 3. 过程宏基础

### 3.1 函数式过程宏

```rust
// 在Cargo.toml中需要添加：
// [lib]
// proc-macro = true
//
// [dependencies]
// proc-macro2 = "1.0"
// quote = "1.0"
// syn = { version = "2.0", features = ["full"] }

use proc_macro::TokenStream;
use quote::quote;
use syn;

// 函数式过程宏
#[proc_macro]
pub fn make_answer(_item: TokenStream) -> TokenStream {
    "fn answer() -> u32 { 42 }".parse().unwrap()
}

// 使用示例（在另一个crate中）
/*
use my_macro::make_answer;

make_answer!();

fn main() {
    println!("The answer is: {}", answer());
}
*/
```

### 3.2 派生宏

```rust
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput};

// 派生宏示例
#[proc_macro_derive(HelloMacro)]
pub fn hello_macro_derive(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = input.ident;
    
    let expanded = quote! {
        impl HelloMacro for #name {
            fn hello_macro() {
                println!("Hello, Macro! My name is {}!", stringify!(#name));
            }
        }
    };
    
    TokenStream::from(expanded)
}

// 带属性的派生宏
#[proc_macro_derive(Builder, attributes(builder))]
pub fn derive_builder(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let builder_name = format!("{}Builder", name);
    let builder_ident = syn::Ident::new(&builder_name, name.span());
    
    let fields = if let syn::Data::Struct(syn::DataStruct {
        fields: syn::Fields::Named(syn::FieldsNamed { ref named, .. }),
        ..
    }) = input.data {
        named
    } else {
        unimplemented!();
    };
    
    let builder_fields = fields.iter().map(|f| {
        let name = &f.ident;
        let ty = &f.ty;
        quote! { #name: Option<#ty> }
    });
    
    let builder_methods = fields.iter().map(|f| {
        let name = &f.ident;
        let ty = &f.ty;
        quote! {
            pub fn #name(&mut self, #name: #ty) -> &mut Self {
                self.#name = Some(#name);
                self
            }
        }
    });
    
    let build_fields = fields.iter().map(|f| {
        let name = &f.ident;
        quote! {
            #name: self.#name.clone().ok_or(concat!(stringify!(#name), " is not set"))?
        }
    });
    
    let expanded = quote! {
        pub struct #builder_ident {
            #(#builder_fields,)*
        }
        
        impl #builder_ident {
            #(#builder_methods)*
            
            pub fn build(&self) -> Result<#name, Box<dyn std::error::Error>> {
                Ok(#name {
                    #(#build_fields,)*
                })
            }
        }
        
        impl #name {
            pub fn builder() -> #builder_ident {
                #builder_ident {
                    #(#name: None,)*
                }
            }
        }
    };
    
    TokenStream::from(expanded)
}

// 使用示例
/*
use my_macro::{HelloMacro, Builder};

pub trait HelloMacro {
    fn hello_macro();
}

#[derive(HelloMacro)]
struct Pancakes;

#[derive(Builder)]
struct Command {
    executable: String,
    args: Vec<String>,
    env: Vec<String>,
    current_dir: String,
}

fn main() {
    Pancakes::hello_macro();
    
    let command = Command::builder()
        .executable("cargo".to_owned())
        .args(vec!["build".to_owned(), "--release".to_owned()])
        .env(vec![])
        .current_dir("src".to_owned())
        .build()
        .unwrap();
}
*/
```

### 3.3 属性宏

```rust
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn};

// 属性宏示例
#[proc_macro_attribute]
pub fn route(args: TokenStream, input: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(input as ItemFn);
    let args = args.to_string();
    
    let fn_name = &input_fn.sig.ident;
    let fn_block = &input_fn.block;
    let fn_vis = &input_fn.vis;
    let fn_sig = &input_fn.sig;
    
    let expanded = quote! {
        #fn_vis #fn_sig {
            println!("Route: {}", #args);
            #fn_block
        }
    };
    
    TokenStream::from(expanded)
}

// 计时宏
#[proc_macro_attribute]
pub fn timed(_args: TokenStream, input: TokenStream) -> TokenStream {
    let input_fn = parse_macro_input!(input as ItemFn);
    
    let fn_name = &input_fn.sig.ident;
    let fn_name_str = fn_name.to_string();
    let fn_block = &input_fn.block;
    let fn_vis = &input_fn.vis;
    let fn_sig = &input_fn.sig;
    
    let expanded = quote! {
        #fn_vis #fn_sig {
            let start = std::time::Instant::now();
            let result = (|| #fn_block)();
            let duration = start.elapsed();
            println!("Function '{}' took: {:?}", #fn_name_str, duration);
            result
        }
    };
    
    TokenStream::from(expanded)
}

// 使用示例
/*
use my_macro::{route, timed};

#[route("/api/users")]
fn get_users() -> String {
    "Getting users...".to_string()
}

#[timed]
fn expensive_computation() -> u64 {
    (0..1_000_000).sum()
}

fn main() {
    let users = get_users();
    println!("{}", users);
    
    let result = expensive_computation();
    println!("Result: {}", result);
}
*/
```

## 4. 宏调试和测试

### 4.1 宏调试技巧

```rust
// 调试宏的输出
macro_rules! debug_macro {
    ($($tokens:tt)*) => {
        {
            println!("Macro input: {}", stringify!($($tokens)*));
            $($tokens)*
        }
    };
}

// 编译时打印宏展开
macro_rules! show_expansion {
    ($($tokens:tt)*) => {
        {
            compile_error!(stringify!($($tokens)*));
        }
    };
}

// 条件编译的调试宏
macro_rules! debug_print {
    ($($tokens:tt)*) => {
        #[cfg(debug_assertions)]
        {
            println!("[DEBUG] {}", format!($($tokens)*));
        }
    };
}

fn main() {
    debug_macro! {
        let x = 5;
        println!("x = {}", x);
    }
    
    debug_print!("This is a debug message: {}", 42);
    
    // 取消注释下面的行来查看宏展开
    // show_expansion! {
    //     let y = 10;
    //     println!("y = {}", y);
    // }
}
```

### 4.2 宏测试

```rust
// 测试宏的功能
macro_rules! assert_tokens_eq {
    ($left:expr, $right:expr) => {
        assert_eq!(stringify!($left), stringify!($right));
    };
}

macro_rules! test_macro {
    ($name:ident, $input:tt, $expected:expr) => {
        #[test]
        fn $name() {
            let result = stringify!($input);
            assert_eq!(result, $expected);
        }
    };
}

// 生成测试用例的宏
macro_rules! generate_tests {
    ($($name:ident: $value:expr,)*) => {
        $(
            #[test]
            fn $name() {
                assert!($value);
            }
        )*
    };
}

generate_tests! {
    test_addition: 2 + 2 == 4,
    test_subtraction: 5 - 3 == 2,
    test_multiplication: 3 * 4 == 12,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    test_macro!(test_simple_expr, 1 + 1, "1 + 1");
    test_macro!(test_complex_expr, (2 * 3) + 4, "(2 * 3) + 4");
    
    #[test]
    fn test_assert_tokens_eq() {
        assert_tokens_eq!(hello, hello);
        assert_tokens_eq!(1 + 1, 1 + 1);
    }
}

fn main() {
    println!("Macro testing examples");
}
```

## 5. 实用宏示例

### 5.1 配置宏

```rust
use std::collections::HashMap;

// 配置构建宏
macro_rules! config {
    ($($key:ident: $value:expr),* $(,)?) => {
        {
            let mut config = HashMap::new();
            $(
                config.insert(stringify!($key), $value.to_string());
            )*
            config
        }
    };
}

// 环境变量宏
macro_rules! env_or_default {
    ($env_var:expr, $default:expr) => {
        std::env::var($env_var).unwrap_or_else(|_| $default.to_string())
    };
}

// 特性标志宏
macro_rules! feature_flag {
    ($flag:expr, $enabled:block, $disabled:block) => {
        if cfg!(feature = $flag) {
            $enabled
        } else {
            $disabled
        }
    };
}

fn main() {
    let app_config = config! {
        host: "localhost",
        port: "8080",
        debug: "true",
        max_connections: "100",
    };
    
    println!("Config: {:?}", app_config);
    
    let database_url = env_or_default!("DATABASE_URL", "sqlite://memory");
    println!("Database URL: {}", database_url);
    
    feature_flag!("advanced_logging", {
        println!("Advanced logging is enabled");
    }, {
        println!("Basic logging is enabled");
    });
}
```

### 5.2 错误处理宏

```rust
// 快速错误定义宏
macro_rules! define_error {
    ($name:ident, $($variant:ident($msg:expr)),*) => {
        #[derive(Debug)]
        enum $name {
            $($variant,)*
        }
        
        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                match self {
                    $(Self::$variant => write!(f, $msg),)*
                }
            }
        }
        
        impl std::error::Error for $name {}
    };
}

// 结果包装宏
macro_rules! try_or_return {
    ($expr:expr, $error:expr) => {
        match $expr {
            Ok(val) => val,
            Err(_) => return Err($error),
        }
    };
}

// 链式错误处理宏
macro_rules! chain_errors {
    ($($expr:expr),*) => {
        {
            $(
                if let Err(e) = $expr {
                    eprintln!("Error in {}: {}", stringify!($expr), e);
                    return Err(e.into());
                }
            )*
            Ok(())
        }
    };
}

define_error!(MyError,
    NetworkError("Network connection failed"),
    ParseError("Failed to parse data"),
    ValidationError("Data validation failed")
);

fn risky_operation() -> Result<String, MyError> {
    // 模拟可能失败的操作
    Ok("Success".to_string())
}

fn another_operation() -> Result<i32, MyError> {
    Ok(42)
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let result = try_or_return!(risky_operation(), MyError::NetworkError);
    println!("Result: {}", result);
    
    // 使用链式错误处理
    chain_errors!(
        risky_operation().map(|_| ()),
        another_operation().map(|_| ())
    )?;
    
    println!("All operations completed successfully");
    Ok(())
}
```

### 5.3 性能测试宏

```rust
use std::time::{Duration, Instant};

// 基准测试宏
macro_rules! benchmark {
    ($name:expr, $code:block) => {
        {
            let start = Instant::now();
            let result = $code;
            let duration = start.elapsed();
            println!("Benchmark '{}': {:?}", $name, duration);
            result
        }
    };
}

// 多次运行基准测试
macro_rules! benchmark_n {
    ($name:expr, $n:expr, $code:block) => {
        {
            let mut total_duration = Duration::new(0, 0);
            let mut results = Vec::new();
            
            for i in 0..$n {
                let start = Instant::now();
                let result = $code;
                let duration = start.elapsed();
                total_duration += duration;
                results.push(result);
            }
            
            let avg_duration = total_duration / $n;
            println!("Benchmark '{}' (avg of {} runs): {:?}", $name, $n, avg_duration);
            results
        }
    };
}

// 比较基准测试
macro_rules! compare_benchmarks {
    ($($name:expr => $code:block),*) => {
        {
            let mut results = Vec::new();
            $(
                let start = Instant::now();
                let result = $code;
                let duration = start.elapsed();
                results.push(($name, duration, result));
                println!("'{}': {:?}", $name, duration);
            )*
            
            // 找出最快的
            if let Some((fastest_name, fastest_time, _)) = results.iter().min_by_key(|(_, duration, _)| duration) {
                println!("Fastest: '{}' with {:?}", fastest_name, fastest_time);
            }
            
            results
        }
    };
}

fn fibonacci_recursive(n: u32) -> u64 {
    match n {
        0 => 0,
        1 => 1,
        _ => fibonacci_recursive(n - 1) + fibonacci_recursive(n - 2),
    }
}

fn fibonacci_iterative(n: u32) -> u64 {
    if n <= 1 {
        return n as u64;
    }
    
    let mut a = 0;
    let mut b = 1;
    
    for _ in 2..=n {
        let temp = a + b;
        a = b;
        b = temp;
    }
    
    b
}

fn main() {
    // 单次基准测试
    let result = benchmark!("Vector creation", {
        (0..1000).collect::<Vec<i32>>()
    });
    println!("Created vector with {} elements", result.len());
    
    // 多次运行基准测试
    benchmark_n!("Small fibonacci", 5, {
        fibonacci_iterative(20)
    });
    
    // 比较不同实现
    compare_benchmarks! {
        "Recursive Fibonacci" => { fibonacci_recursive(30) },
        "Iterative Fibonacci" => { fibonacci_iterative(30) }
    };
}
```

## 6. 高级宏技巧

### 6.1 宏递归和计数

```rust
// 计算参数数量的宏
macro_rules! count {
    () => (0usize);
    ($head:tt $($tail:tt)*) => (1usize + count!($($tail)*));
}

// TT muncher模式
macro_rules! parse_pairs {
    () => {};
    ($key:ident = $value:expr; $($rest:tt)*) => {
        println!("{} = {}", stringify!($key), $value);
        parse_pairs!($($rest)*);
    };
}

// 递归宏实现编译时计算
macro_rules! factorial {
    (0) => (1);
    ($n:expr) => ($n * factorial!($n - 1));
}

// 生成递归数据结构
macro_rules! nested_struct {
    ($name:ident) => {
        struct $name;
    };
    ($name:ident, $($rest:ident),+) => {
        struct $name {
            inner: nested_struct!($($rest),+),
        }
    };
}

fn main() {
    println!("Count of (a, b, c, d): {}", count!(a b c d));
    println!("Count of empty: {}", count!());
    
    parse_pairs! {
        name = "Alice";
        age = 30;
        city = "New York";
    }
    
    // 注意：factorial宏在编译时计算，但有递归限制
    // println!("Factorial of 5: {}", factorial!(5));
    
    // 嵌套结构体
    nested_struct!(Level1, Level2, Level3);
    let nested = Level1 {
        inner: Level2 {
            inner: Level3,
        },
    };
}
```

### 6.2 宏中的类型操作

```rust
// 类型转换宏
macro_rules! impl_from {
    ($from:ty => $to:ty, $method:ident) => {
        impl From<$from> for $to {
            fn from(value: $from) -> Self {
                Self::$method(value)
            }
        }
    };
}

// 为多个类型实现trait
macro_rules! impl_trait_for_types {
    ($trait:ident for $($type:ty),*) => {
        $(
            impl $trait for $type {
                fn method(&self) -> String {
                    format!("Implementation for {}", stringify!($type))
                }
            }
        )*
    };
}

// 泛型宏
macro_rules! create_generic_struct {
    ($name:ident<$($generic:ident),*>) => {
        struct $name<$($generic),*> {
            $(
                $generic: $generic,
            )*
        }
        
        impl<$($generic),*> $name<$($generic),*> {
            fn new($($generic: $generic),*) -> Self {
                Self {
                    $($generic,)*
                }
            }
        }
    };
}

trait MyTrait {
    fn method(&self) -> String;
}

struct MyStruct(i32);

impl MyStruct {
    fn from_int(value: i32) -> Self {
        MyStruct(value)
    }
}

impl_from!(i32 => MyStruct, from_int);
impl_trait_for_types!(MyTrait for i32, f64, String);

create_generic_struct!(Container<T>);
create_generic_struct!(Pair<T, U>);

fn main() {
    let s: MyStruct = 42.into();
    println!("MyStruct: {}", s.0);
    
    println!("i32 trait: {}", 42.method());
    println!("f64 trait: {}", 3.14.method());
    println!("String trait: {}", "hello".to_string().method());
    
    let container = Container::new(42);
    let pair = Pair::new("hello", 42);
}
```

### 6.3 宏中的条件编译

```rust
// 平台特定的宏
macro_rules! platform_specific {
    (windows: $windows_code:block, unix: $unix_code:block) => {
        #[cfg(target_os = "windows")]
        $windows_code
        
        #[cfg(target_family = "unix")]
        $unix_code
    };
}

// 特性标志条件宏
macro_rules! feature_specific {
    ($feature:expr, $enabled:block $(, $disabled:block)?) => {
        #[cfg(feature = $feature)]
        $enabled
        
        $(
            #[cfg(not(feature = $feature))]
            $disabled
        )?
    };
}

// 编译时配置宏
macro_rules! compile_time_config {
    ($($key:ident = $value:expr),*) => {
        $(
            const $key: &str = $value;
        )*
        
        fn get_config() -> std::collections::HashMap<&'static str, &'static str> {
            let mut config = std::collections::HashMap::new();
            $(
                config.insert(stringify!($key), $key);
            )*
            config
        }
    };
}

fn main() {
    platform_specific! {
        windows: {
            println!("Running on Windows");
        },
        unix: {
            println!("Running on Unix-like system");
        }
    }
    
    feature_specific!("advanced_features", {
        println!("Advanced features are enabled");
    }, {
        println!("Advanced features are disabled");
    });
    
    compile_time_config! {
        VERSION = "1.0.0",
        BUILD_DATE = "2024-01-01",
        AUTHOR = "Rust Developer"
    }
    
    let config = get_config();
    for (key, value) in config {
        println!("{}: {}", key, value);
    }
}
```

## 7. 宏的最佳实践

### 7.1 宏设计原则

```rust
// 1. 使用描述性的宏名称
macro_rules! create_getter_setter {
    ($struct_name:ident, $field:ident: $field_type:ty) => {
        impl $struct_name {
            pub fn $field(&self) -> &$field_type {
                &self.$field
            }
            
            paste::paste! {
                pub fn [<set_ $field>](&mut self, value: $field_type) {
                    self.$field = value;
                }
            }
        }
    };
}

// 2. 提供清晰的错误消息
macro_rules! assert_positive {
    ($value:expr) => {
        if $value <= 0 {
            panic!("Expected positive value, got: {}", $value);
        }
    };
    ($value:expr, $message:expr) => {
        if $value <= 0 {
            panic!("{}: {}", $message, $value);
        }
    };
}

// 3. 支持多种调用方式
macro_rules! flexible_macro {
    // 单个参数
    ($single:expr) => {
        println!("Single: {}", $single);
    };
    
    // 多个参数
    ($first:expr, $($rest:expr),+) => {
        println!("First: {}", $first);
        $(println!("Rest: {}", $rest);)+
    };
    
    // 带标签的参数
    (label: $label:expr, value: $value:expr) => {
        println!("{}: {}", $label, $value);
    };
}

// 4. 文档化宏
/// 创建一个简单的构建器模式结构体
/// 
/// # 示例
/// 
/// ```
/// simple_builder!(Person {
///     name: String,
///     age: u32
/// });
/// 
/// let person = Person::builder()
///     .name("Alice".to_string())
///     .age(30)
///     .build();
/// ```
macro_rules! simple_builder {
    ($name:ident { $($field:ident: $type:ty),* }) => {
        pub struct $name {
            $(pub $field: $type,)*
        }
        
        paste::paste! {
            pub struct [<$name Builder>] {
                $($field: Option<$type>,)*
            }
            
            impl [<$name Builder>] {
                pub fn new() -> Self {
                    Self {
                        $($field: None,)*
                    }
                }
                
                $(
                    pub fn $field(mut self, $field: $type) -> Self {
                        self.$field = Some($field);
                        self
                    }
                )*
                
                pub fn build(self) -> Result<$name, String> {
                    Ok($name {
                        $(
                            $field: self.$field.ok_or_else(|| {
                                format!("Field '{}' is required", stringify!($field))
                            })?,
                        )*
                    })
                }
            }
            
            impl $name {
                pub fn builder() -> [<$name Builder>] {
                    [<$name Builder>]::new()
                }
            }
        }
    };
}

fn main() {
    flexible_macro!(42);
    flexible_macro!(1, 2, 3);
    flexible_macro!(label: "test", value: 100);
    
    assert_positive!(5);
    assert_positive!(10, "Custom error message");
    
    // 注意：这需要paste crate来工作
    // simple_builder!(User {
    //     name: String,
    //     email: String
    // });
}
```

### 7.2 性能考虑

```rust
// 避免不必要的克隆
macro_rules! efficient_clone {
    ($value:expr) => {
        // 不好的做法
        // $value.clone()
        
        // 更好的做法：让调用者决定是否克隆
        &$value
    };
}

// 编译时计算
macro_rules! compile_time_size {
    ($type:ty) => {
        const _: () = {
            const SIZE: usize = std::mem::size_of::<$type>();
            if SIZE > 1024 {
                panic!("Type is too large");
            }
        };
    };
}

// 零成本抽象
macro_rules! zero_cost_wrapper {
    ($name:ident, $inner:ty) => {
        #[repr(transparent)]
        struct $name($inner);
        
        impl $name {
            #[inline(always)]
            fn new(value: $inner) -> Self {
                Self(value)
            }
            
            #[inline(always)]
            fn into_inner(self) -> $inner {
                self.0
            }
        }
        
        impl std::ops::Deref for $name {
            type Target = $inner;
            
            #[inline(always)]
            fn deref(&self) -> &Self::Target {
                &self.0
            }
        }
    };
}

zero_cost_wrapper!(UserId, u64);
zero_cost_wrapper!(ProductId, u32);

compile_time_size!(u64);
// compile_time_size!([u8; 2048]); // 这会导致编译错误

fn main() {
    let user_id = UserId::new(12345);
    println!("User ID: {}", *user_id);
    
    let product_id = ProductId::new(67890);
    println!("Product ID: {}", product_id.into_inner());
}
```

## 8. 练习题

### 练习1：实现一个状态机宏

```rust
// 状态机宏实现
macro_rules! state_machine {
    (
        $name:ident {
            states: [$($state:ident),*],
            events: [$($event:ident),*],
            transitions: {
                $($from:ident --$evt:ident--> $to:ident),*
            },
            initial: $initial:ident
        }
    ) => {
        #[derive(Debug, Clone, Copy, PartialEq)]
        enum State {
            $($state,)*
        }
        
        #[derive(Debug, Clone, Copy, PartialEq)]
        enum Event {
            $($event,)*
        }
        
        struct $name {
            current_state: State,
        }
        
        impl $name {
            fn new() -> Self {
                Self {
                    current_state: State::$initial,
                }
            }
            
            fn current_state(&self) -> State {
                self.current_state
            }
            
            fn handle_event(&mut self, event: Event) -> Result<State, String> {
                let new_state = match (self.current_state, event) {
                    $(
                        (State::$from, Event::$evt) => State::$to,
                    )*
                    _ => return Err(format!(
                        "Invalid transition from {:?} with event {:?}",
                        self.current_state, event
                    )),
                };
                
                self.current_state = new_state;
                Ok(new_state)
            }
        }
    };
}

// 使用状态机宏
state_machine!(TrafficLight {
    states: [Red, Yellow, Green],
    events: [Timer, Emergency],
    transitions: {
        Red --Timer--> Green,
        Green --Timer--> Yellow,
        Yellow --Timer--> Red,
        Red --Emergency--> Yellow,
        Green --Emergency--> Yellow,
        Yellow --Emergency--> Red
    },
    initial: Red
});

fn main() {
    let mut traffic_light = TrafficLight::new();
    println!("Initial state: {:?}", traffic_light.current_state());
    
    // 正常流程
    traffic_light.handle_event(Event::Timer).unwrap();
    println!("After Timer: {:?}", traffic_light.current_state());
    
    traffic_light.handle_event(Event::Timer).unwrap();
    println!("After Timer: {:?}", traffic_light.current_state());
    
    // 紧急情况
    traffic_light.handle_event(Event::Emergency).unwrap();
    println!("After Emergency: {:?}", traffic_light.current_state());
    
    // 无效转换
    if let Err(e) = traffic_light.handle_event(Event::Timer) {
        println!("Error: {}", e);
    }
}
```

### 练习2：实现一个查询构建器宏

```rust
use std::collections::HashMap;

// 查询构建器宏
macro_rules! query_builder {
    (
        table: $table:ident,
        fields: [$($field:ident),*],
        $(where: $where_clause:expr,)?
        $(order_by: $order_field:ident $order_dir:ident,)?
        $(limit: $limit:expr,)?
    ) => {
        {
            let mut query = String::new();
            
            // SELECT clause
            query.push_str("SELECT ");
            let fields = vec![$(stringify!($field)),*];
            query.push_str(&fields.join(", "));
            
            // FROM clause
            query.push_str(" FROM ");
            query.push_str(stringify!($table));
            
            // WHERE clause
            $(
                query.push_str(" WHERE ");
                query.push_str($where_clause);
            )?
            
            // ORDER BY clause
            $(
                query.push_str(" ORDER BY ");
                query.push_str(stringify!($order_field));
                query.push_str(" ");
                query.push_str(stringify!($order_dir));
            )?
            
            // LIMIT clause
            $(
                query.push_str(" LIMIT ");
                query.push_str(&$limit.to_string());
            )?
            
            query
        }
    };
}

// 更高级的查询构建器
macro_rules! advanced_query {
    (
        SELECT $($field:ident),+
        FROM $table:ident
        $(WHERE $($where_field:ident $op:tt $value:expr)and+)?
        $(ORDER BY $order_field:ident $order_dir:ident)?
        $(LIMIT $limit:expr)?
    ) => {
        {
            let mut query = String::new();
            let mut params = Vec::new();
            
            // SELECT
            query.push_str("SELECT ");
            let fields = vec![$(stringify!($field)),+];
            query.push_str(&fields.join(", "));
            
            // FROM
            query.push_str(" FROM ");
            query.push_str(stringify!($table));
            
            // WHERE
            $(
                query.push_str(" WHERE ");
                let conditions = vec![
                    $(format!("{} {} ?", stringify!($where_field), stringify!($op))),+
                ];
                query.push_str(&conditions.join(" AND "));
                $(params.push($value.to_string());)+
            )?
            
            // ORDER BY
            $(
                query.push_str(" ORDER BY ");
                query.push_str(stringify!($order_field));
                query.push_str(" ");
                query.push_str(stringify!($order_dir));
            )?
            
            // LIMIT
            $(
                query.push_str(" LIMIT ");
                query.push_str(&$limit.to_string());
            )?
            
            (query, params)
        }
    };
}

fn main() {
    // 基本查询
    let query1 = query_builder! {
        table: users,
        fields: [id, name, email],
        where: "age > 18",
        order_by: name ASC,
        limit: 10,
    };
    println!("Query 1: {}", query1);
    
    // 简单查询
    let query2 = query_builder! {
        table: products,
        fields: [name, price],
    };
    println!("Query 2: {}", query2);
    
    // 高级查询
    let (query3, params) = advanced_query! {
        SELECT id, name, email
        FROM users
        WHERE age > 18 and status == "active"
        ORDER BY name ASC
        LIMIT 5
    };
    println!("Query 3: {}", query3);
    println!("Params: {:?}", params);
}
```

### 练习3：实现一个配置验证宏

```rust
use std::collections::HashMap;

// 配置验证宏
macro_rules! validate_config {
    (
        $config:expr,
        {
            $(
                $field:ident: $field_type:ty $(= $default:expr)? $(where $validator:expr)?
            ),* $(,)?
        }
    ) => {
        {
            let mut errors = Vec::new();
            let mut validated_config = HashMap::new();
            
            $(
                let field_name = stringify!($field);
                
                match $config.get(field_name) {
                    Some(value) => {
                        // 尝试解析值
                        match value.parse::<$field_type>() {
                            Ok(parsed_value) => {
                                // 运行验证器（如果提供）
                                $(
                                    if !($validator)(&parsed_value) {
                                        errors.push(format!(
                                            "Validation failed for field '{}': {}",
                                            field_name,
                                            value
                                        ));
                                    } else {
                                        validated_config.insert(
                                            field_name.to_string(),
                                            parsed_value.to_string()
                                        );
                                    }
                                )?
                                
                                // 如果没有验证器，直接添加
                                #[allow(unreachable_code)]
                                {
                                    validated_config.insert(
                                        field_name.to_string(),
                                        parsed_value.to_string()
                                    );
                                }
                            }
                            Err(_) => {
                                errors.push(format!(
                                    "Failed to parse field '{}' as {}: {}",
                                    field_name,
                                    stringify!($field_type),
                                    value
                                ));
                            }
                        }
                    }
                    None => {
                        // 使用默认值（如果提供）
                        $(
                            validated_config.insert(
                                field_name.to_string(),
                                $default.to_string()
                            );
                        )?
                        
                        // 如果没有默认值，报错
                        #[allow(unreachable_code)]
                        {
                            errors.push(format!("Missing required field: {}", field_name));
                        }
                    }
                }
            )*
            
            if errors.is_empty() {
                Ok(validated_config)
            } else {
                Err(errors)
            }
        }
    };
}

// 配置构建宏
macro_rules! config_struct {
    (
        $name:ident {
            $(
                $field:ident: $field_type:ty $(= $default:expr)?
            ),* $(,)?
        }
    ) => {
        #[derive(Debug)]
        struct $name {
            $(pub $field: $field_type,)*
        }
        
        impl $name {
            fn from_map(config: HashMap<String, String>) -> Result<Self, Vec<String>> {
                let mut errors = Vec::new();
                
                $(
                    let $field = match config.get(stringify!($field)) {
                        Some(value) => {
                            match value.parse::<$field_type>() {
                                Ok(parsed) => parsed,
                                Err(_) => {
                                    errors.push(format!(
                                        "Failed to parse {}: {}",
                                        stringify!($field),
                                        value
                                    ));
                                    $(return Err(errors);)?
                                    #[allow(unreachable_code)]
                                    Default::default()
                                }
                            }
                        }
                        None => {
                            $($default)?
                            #[allow(unreachable_code)]
                            {
                                errors.push(format!("Missing field: {}", stringify!($field)));
                                Default::default()
                            }
                        }
                    };
                )*
                
                if errors.is_empty() {
                    Ok(Self {
                        $($field,)*
                    })
                } else {
                    Err(errors)
                }
            }
        }
    };
}

fn main() {
    let mut config = HashMap::new();
    config.insert("port".to_string(), "8080".to_string());
    config.insert("host".to_string(), "localhost".to_string());
    config.insert("max_connections".to_string(), "100".to_string());
    config.insert("timeout".to_string(), "30".to_string());
    
    // 验证配置
    let result = validate_config!(config, {
        port: u16 where |&x| x > 1024 && x < 65536,
        host: String,
        max_connections: u32 = 50 where |&x| x > 0 && x <= 1000,
        timeout: u64 where |&x| x > 0,
        debug: bool = true,
    });
    
    match result {
        Ok(validated) => {
            println!("Validated config: {:?}", validated);
        }
        Err(errors) => {
            println!("Validation errors:");
            for error in errors {
                println!("  - {}", error);
            }
        }
    }
    
    // 使用配置结构体
    config_struct!(ServerConfig {
        port: u16 = 8080,
        host: String = "localhost".to_string(),
        max_connections: u32 = 100,
    });
    
    match ServerConfig::from_map(config) {
        Ok(server_config) => {
            println!("Server config: {:?}", server_config);
        }
        Err(errors) => {
            println!("Config errors: {:?}", errors);
        }
    }
}
```

## 总结

通过本章的学习，你应该掌握了：

### 核心概念
- 声明式宏和过程宏的区别和用途
- 宏模式匹配和重复语法
- 宏的指示符类型和使用场景
- 宏的调试和测试方法

### 实用技能
- 编写简单和复杂的声明式宏
- 理解过程宏的基本结构
- 使用宏进行代码生成和DSL设计
- 宏的性能优化和最佳实践

### 最佳实践
- 宏的命名和文档化
- 错误处理和用户友好的错误消息
- 宏的可维护性和可读性
- 避免宏的过度使用

### 高级特性
- 宏递归和编译时计算
- 条件编译和平台特定代码
- 零成本抽象的实现
- 复杂DSL和状态机的设计

宏系统是Rust的强大特性之一，合理使用可以大大提高代码的表达力和可维护性，但也要注意避免过度复杂化。
# 第10章：模块系统

## 概述

Rust的模块系统是组织代码的强大工具，它提供了命名空间、封装和代码重用的机制。通过模块系统，你可以将相关的功能组织在一起，控制代码的可见性，并构建可维护的大型项目。

## 学习目标

通过本章学习，你将掌握：
- 模块的定义和使用
- 可见性规则和访问控制
- `use`语句的各种用法
- 包（Package）和crate的概念
- 文件系统模块组织
- 模块系统的最佳实践

## 1. 模块基础

### 1.1 模块定义

```rust
// 内联模块定义
mod front_of_house {
    mod hosting {
        fn add_to_waitlist() {}
        fn seat_at_table() {}
    }

    mod serving {
        fn take_order() {}
        fn serve_order() {}
        fn take_payment() {}
    }
}

// 使用模块中的函数
fn eat_at_restaurant() {
    // 绝对路径
    crate::front_of_house::hosting::add_to_waitlist();
    
    // 相对路径
    front_of_house::hosting::add_to_waitlist();
}
```

### 1.2 模块树结构

```rust
// 模块树示例
mod restaurant {
    mod front_of_house {
        pub mod hosting {
            pub fn add_to_waitlist() {
                println!("Adding customer to waitlist");
            }
            
            pub fn seat_at_table() {
                println!("Seating customer at table");
            }
        }
        
        pub mod serving {
            pub fn take_order() {
                println!("Taking order");
            }
            
            pub fn serve_order() {
                println!("Serving order");
            }
            
            fn take_payment() {
                println!("Taking payment");
            }
        }
    }
    
    mod back_of_house {
        pub struct Breakfast {
            pub toast: String,
            seasonal_fruit: String,
        }
        
        impl Breakfast {
            pub fn summer(toast: &str) -> Breakfast {
                Breakfast {
                    toast: String::from(toast),
                    seasonal_fruit: String::from("peaches"),
                }
            }
        }
        
        pub enum Appetizer {
            Soup,
            Salad,
        }
        
        fn fix_incorrect_order() {
            cook_order();
            super::front_of_house::serving::serve_order();
        }
        
        fn cook_order() {
            println!("Cooking order");
        }
    }
    
    pub fn eat_at_restaurant() {
        // 订购早餐
        let mut meal = back_of_house::Breakfast::summer("Rye");
        meal.toast = String::from("Wheat");
        println!("I'd like {} toast please", meal.toast);
        
        // 下面这行会编译错误，因为seasonal_fruit是私有的
        // meal.seasonal_fruit = String::from("blueberries");
        
        // 订购开胃菜
        let order1 = back_of_house::Appetizer::Soup;
        let order2 = back_of_house::Appetizer::Salad;
        
        // 调用前台服务
        front_of_house::hosting::add_to_waitlist();
        front_of_house::hosting::seat_at_table();
        front_of_house::serving::take_order();
        front_of_house::serving::serve_order();
    }
}

fn main() {
    restaurant::eat_at_restaurant();
}
```

### 1.3 路径和可见性

```rust
mod sound {
    pub mod instrument {
        pub fn clarinet() {
            println!("Playing clarinet");
        }
        
        fn violin() {
            println!("Playing violin");
        }
    }
    
    mod voice {
        pub fn soprano() {
            println!("Singing soprano");
        }
    }
}

mod performance_group {
    pub use crate::sound::instrument::clarinet;
    
    pub fn clarinet_trio() {
        // 使用绝对路径
        crate::sound::instrument::clarinet();
        
        // 使用相对路径
        super::sound::instrument::clarinet();
        
        // 使用重新导出的路径
        clarinet();
    }
}

fn main() {
    // 绝对路径调用
    crate::sound::instrument::clarinet();
    
    // 通过performance_group调用
    performance_group::clarinet_trio();
    
    // 直接使用重新导出的函数
    performance_group::clarinet();
}
```

## 2. 可见性规则

### 2.1 pub关键字

```rust
mod my_module {
    // 私有函数（默认）
    fn private_function() {
        println!("This is private");
    }
    
    // 公有函数
    pub fn public_function() {
        println!("This is public");
        private_function(); // 模块内部可以调用私有函数
    }
    
    // 公有结构体
    pub struct PublicStruct {
        pub public_field: String,
        private_field: i32,
    }
    
    impl PublicStruct {
        pub fn new(name: &str) -> PublicStruct {
            PublicStruct {
                public_field: String::from(name),
                private_field: 42,
            }
        }
        
        pub fn get_private_field(&self) -> i32 {
            self.private_field
        }
    }
    
    // 公有枚举（所有变体都是公有的）
    pub enum PublicEnum {
        Variant1,
        Variant2(String),
        Variant3 { field: i32 },
    }
    
    // 嵌套模块
    pub mod nested {
        pub fn nested_function() {
            println!("Nested function");
            super::private_function(); // 可以访问父模块的私有函数
        }
    }
}

fn main() {
    // 调用公有函数
    my_module::public_function();
    
    // 创建公有结构体
    let mut s = my_module::PublicStruct::new("test");
    s.public_field = String::from("modified");
    println!("Public field: {}", s.public_field);
    println!("Private field: {}", s.get_private_field());
    
    // 使用公有枚举
    let enum_val = my_module::PublicEnum::Variant2(String::from("hello"));
    
    // 调用嵌套模块的函数
    my_module::nested::nested_function();
    
    // 下面这些调用会编译错误
    // my_module::private_function(); // 私有函数不可访问
    // s.private_field = 10; // 私有字段不可访问
}
```

### 2.2 pub(crate)和其他可见性修饰符

```rust
mod outer_module {
    pub(crate) fn crate_visible() {
        println!("Visible within the crate");
    }
    
    pub(super) fn parent_visible() {
        println!("Visible to parent module");
    }
    
    pub(self) fn self_visible() {
        println!("Visible within this module only");
    }
    
    pub mod inner_module {
        pub(in crate::outer_module) fn limited_scope() {
            println!("Visible within outer_module");
        }
        
        pub fn test_visibility() {
            // 可以调用父模块的pub(super)函数
            super::parent_visible();
            
            // 可以调用同一模块的函数
            super::self_visible();
            
            // 可以调用crate级别的函数
            super::crate_visible();
            
            // 可以调用限定作用域的函数
            limited_scope();
        }
    }
}

// 在crate根级别可以调用pub(crate)函数
fn main() {
    outer_module::crate_visible();
    outer_module::inner_module::test_visibility();
    
    // 下面这些调用会编译错误
    // outer_module::parent_visible(); // pub(super)不可见
    // outer_module::self_visible(); // pub(self)不可见
    // outer_module::inner_module::limited_scope(); // 作用域限制
}
```

## 3. use语句

### 3.1 基本use用法

```rust
mod front_of_house {
    pub mod hosting {
        pub fn add_to_waitlist() {
            println!("Adding to waitlist");
        }
        
        pub fn seat_at_table() {
            println!("Seating at table");
        }
    }
}

// 引入模块
use crate::front_of_house::hosting;

// 引入特定函数
use crate::front_of_house::hosting::add_to_waitlist;

// 引入多个项目
use crate::front_of_house::hosting::{seat_at_table, add_to_waitlist as add_customer};

// 引入所有公有项目
use crate::front_of_house::hosting::*;

fn main() {
    // 使用引入的模块
    hosting::add_to_waitlist();
    hosting::seat_at_table();
    
    // 使用引入的函数
    add_to_waitlist();
    
    // 使用别名
    add_customer();
    
    // 使用通配符引入的函数
    seat_at_table();
}
```

### 3.2 use的惯用法

```rust
use std::collections::HashMap;
use std::fmt::Result;
use std::io::Result as IoResult;

// 对于函数，通常引入父模块
use std::collections;

// 对于结构体、枚举和其他项目，通常直接引入
use std::collections::HashMap;

fn function1() -> Result {
    // fmt::Result
    Ok(())
}

fn function2() -> IoResult<()> {
    // io::Result
    Ok(())
}

fn main() {
    // 使用引入的HashMap
    let mut map = HashMap::new();
    map.insert("key", "value");
    
    // 使用模块路径调用函数（推荐）
    let mut set = collections::HashSet::new();
    set.insert("item");
    
    println!("Map: {:?}", map);
    println!("Set: {:?}", set);
}
```

### 3.3 重新导出（Re-exporting）

```rust
mod front_of_house {
    pub mod hosting {
        pub fn add_to_waitlist() {
            println!("Adding to waitlist");
        }
    }
}

// 重新导出，使外部代码可以直接访问
pub use crate::front_of_house::hosting;

// 创建便利的API
pub use crate::front_of_house::hosting::add_to_waitlist as add_customer;

// 在lib.rs中，这样的重新导出很常见
pub mod utils {
    pub fn helper_function() {
        println!("Helper function");
    }
}

// 重新导出utils模块的内容
pub use utils::*;

fn main() {
    // 外部代码可以直接使用
    hosting::add_to_waitlist();
    add_customer();
    helper_function();
}
```

## 4. 文件系统模块

### 4.1 将模块分离到不同文件

```rust
// src/lib.rs 或 src/main.rs
mod front_of_house; // 声明模块，Rust会查找src/front_of_house.rs

pub use crate::front_of_house::hosting;

pub fn eat_at_restaurant() {
    hosting::add_to_waitlist();
    hosting::seat_at_table();
}
```

```rust
// src/front_of_house.rs
pub mod hosting; // 声明子模块，Rust会查找src/front_of_house/hosting.rs

pub mod serving {
    pub fn take_order() {
        println!("Taking order");
    }
    
    pub fn serve_order() {
        println!("Serving order");
    }
    
    pub fn take_payment() {
        println!("Taking payment");
    }
}
```

```rust
// src/front_of_house/hosting.rs
pub fn add_to_waitlist() {
    println!("Adding customer to waitlist");
}

pub fn seat_at_table() {
    println!("Seating customer at table");
}
```

### 4.2 模块文件组织结构

```
src/
├── main.rs
├── lib.rs
├── front_of_house.rs
├── front_of_house/
│   ├── hosting.rs
│   └── serving.rs
├── back_of_house/
│   ├── mod.rs
│   ├── kitchen.rs
│   └── inventory.rs
└── utils/
    ├── mod.rs
    ├── string_utils.rs
    └── math_utils.rs
```

```rust
// src/back_of_house/mod.rs
pub mod kitchen;
pub mod inventory;

pub struct Breakfast {
    pub toast: String,
    seasonal_fruit: String,
}

impl Breakfast {
    pub fn summer(toast: &str) -> Breakfast {
        Breakfast {
            toast: String::from(toast),
            seasonal_fruit: String::from("peaches"),
        }
    }
}
```

```rust
// src/back_of_house/kitchen.rs
pub fn cook_order() {
    println!("Cooking order in the kitchen");
}

pub fn prepare_ingredients() {
    println!("Preparing ingredients");
}
```

```rust
// src/back_of_house/inventory.rs
use std::collections::HashMap;

pub struct Inventory {
    items: HashMap<String, u32>,
}

impl Inventory {
    pub fn new() -> Self {
        Inventory {
            items: HashMap::new(),
        }
    }
    
    pub fn add_item(&mut self, name: String, quantity: u32) {
        *self.items.entry(name).or_insert(0) += quantity;
    }
    
    pub fn get_quantity(&self, name: &str) -> u32 {
        *self.items.get(name).unwrap_or(&0)
    }
    
    pub fn use_item(&mut self, name: &str, quantity: u32) -> bool {
        if let Some(current) = self.items.get_mut(name) {
            if *current >= quantity {
                *current -= quantity;
                return true;
            }
        }
        false
    }
}
```

## 5. 包和Crate

### 5.1 Crate类型

```rust
// Binary crate (src/main.rs)
use restaurant::eat_at_restaurant;

fn main() {
    eat_at_restaurant();
}
```

```rust
// Library crate (src/lib.rs)
mod front_of_house;
mod back_of_house;

pub use crate::front_of_house::hosting;
pub use crate::back_of_house::{Breakfast, Inventory};

pub fn eat_at_restaurant() {
    // 使用前台服务
    hosting::add_to_waitlist();
    hosting::seat_at_table();
    
    // 订购早餐
    let mut meal = Breakfast::summer("Rye");
    meal.toast = String::from("Wheat");
    println!("I'd like {} toast please", meal.toast);
    
    // 管理库存
    let mut inventory = Inventory::new();
    inventory.add_item("Bread".to_string(), 10);
    inventory.add_item("Eggs".to_string(), 24);
    
    println!("Bread quantity: {}", inventory.get_quantity("Bread"));
    
    if inventory.use_item("Bread", 2) {
        println!("Used 2 bread");
    }
}
```

### 5.2 Cargo.toml配置

```toml
[package]
name = "restaurant"
version = "0.1.0"
edition = "2021"

# 库crate配置
[lib]
name = "restaurant"
path = "src/lib.rs"

# 二进制crate配置
[[bin]]
name = "restaurant_app"
path = "src/main.rs"

# 多个二进制文件
[[bin]]
name = "server"
path = "src/bin/server.rs"

[[bin]]
name = "client"
path = "src/bin/client.rs"

[dependencies]
serde = { version = "1.0", features = ["derive"] }
tokio = { version = "1.0", features = ["full"] }
```

### 5.3 工作空间（Workspace）

```toml
# Cargo.toml (workspace root)
[workspace]
members = [
    "restaurant-lib",
    "restaurant-server",
    "restaurant-client",
    "common-utils"
]

[workspace.dependencies]
serde = "1.0"
tokio = "1.0"
```

```toml
# restaurant-lib/Cargo.toml
[package]
name = "restaurant-lib"
version = "0.1.0"
edition = "2021"

[dependencies]
serde = { workspace = true, features = ["derive"] }
common-utils = { path = "../common-utils" }
```

```toml
# restaurant-server/Cargo.toml
[package]
name = "restaurant-server"
version = "0.1.0"
edition = "2021"

[dependencies]
restaurant-lib = { path = "../restaurant-lib" }
tokio = { workspace = true, features = ["full"] }
common-utils = { path = "../common-utils" }
```

## 6. 实践示例

### 6.1 构建一个简单的Web服务器模块

```rust
// src/lib.rs
pub mod server;
pub mod request;
pub mod response;
pub mod router;
pub mod middleware;

pub use server::Server;
pub use request::Request;
pub use response::Response;
pub use router::Router;

// 便利的重新导出
pub mod prelude {
    pub use crate::{Server, Request, Response, Router};
    pub use crate::middleware::{Middleware, Logger, Cors};
}
```

```rust
// src/server.rs
use crate::{Request, Response, Router};
use std::net::{TcpListener, TcpStream};
use std::io::prelude::*;
use std::thread;

pub struct Server {
    address: String,
    router: Router,
}

impl Server {
    pub fn new(address: &str) -> Self {
        Server {
            address: address.to_string(),
            router: Router::new(),
        }
    }
    
    pub fn router(&mut self) -> &mut Router {
        &mut self.router
    }
    
    pub fn run(&self) -> std::io::Result<()> {
        let listener = TcpListener::bind(&self.address)?;
        println!("Server running on {}", self.address);
        
        for stream in listener.incoming() {
            let stream = stream?;
            let router = self.router.clone();
            
            thread::spawn(move || {
                handle_connection(stream, router);
            });
        }
        
        Ok(())
    }
}

fn handle_connection(mut stream: TcpStream, router: Router) {
    let mut buffer = [0; 1024];
    stream.read(&mut buffer).unwrap();
    
    let request = Request::from_raw(&buffer);
    let response = router.handle(request);
    
    stream.write(response.to_bytes().as_bytes()).unwrap();
    stream.flush().unwrap();
}
```

```rust
// src/request.rs
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct Request {
    pub method: String,
    pub path: String,
    pub headers: HashMap<String, String>,
    pub body: String,
}

impl Request {
    pub fn from_raw(buffer: &[u8]) -> Self {
        let request_string = String::from_utf8_lossy(buffer);
        let lines: Vec<&str> = request_string.lines().collect();
        
        if lines.is_empty() {
            return Request::default();
        }
        
        // 解析请求行
        let request_line_parts: Vec<&str> = lines[0].split_whitespace().collect();
        let method = request_line_parts.get(0).unwrap_or(&"GET").to_string();
        let path = request_line_parts.get(1).unwrap_or(&"/").to_string();
        
        // 解析头部
        let mut headers = HashMap::new();
        let mut body_start = 0;
        
        for (i, line) in lines.iter().enumerate().skip(1) {
            if line.is_empty() {
                body_start = i + 1;
                break;
            }
            
            if let Some(colon_pos) = line.find(':') {
                let key = line[..colon_pos].trim().to_string();
                let value = line[colon_pos + 1..].trim().to_string();
                headers.insert(key, value);
            }
        }
        
        // 解析主体
        let body = if body_start < lines.len() {
            lines[body_start..].join("\n")
        } else {
            String::new()
        };
        
        Request {
            method,
            path,
            headers,
            body,
        }
    }
    
    pub fn get_header(&self, name: &str) -> Option<&String> {
        self.headers.get(name)
    }
}

impl Default for Request {
    fn default() -> Self {
        Request {
            method: "GET".to_string(),
            path: "/".to_string(),
            headers: HashMap::new(),
            body: String::new(),
        }
    }
}
```

```rust
// src/response.rs
use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct Response {
    pub status_code: u16,
    pub status_text: String,
    pub headers: HashMap<String, String>,
    pub body: String,
}

impl Response {
    pub fn new(status_code: u16, body: String) -> Self {
        let status_text = match status_code {
            200 => "OK",
            404 => "Not Found",
            500 => "Internal Server Error",
            _ => "Unknown",
        }.to_string();
        
        let mut headers = HashMap::new();
        headers.insert("Content-Type".to_string(), "text/html".to_string());
        headers.insert("Content-Length".to_string(), body.len().to_string());
        
        Response {
            status_code,
            status_text,
            headers,
            body,
        }
    }
    
    pub fn ok(body: String) -> Self {
        Self::new(200, body)
    }
    
    pub fn not_found() -> Self {
        Self::new(404, "404 Not Found".to_string())
    }
    
    pub fn internal_error() -> Self {
        Self::new(500, "500 Internal Server Error".to_string())
    }
    
    pub fn set_header(&mut self, name: String, value: String) {
        self.headers.insert(name, value);
    }
    
    pub fn to_bytes(&self) -> String {
        let mut response = format!("HTTP/1.1 {} {}\r\n", self.status_code, self.status_text);
        
        for (key, value) in &self.headers {
            response.push_str(&format!("{}: {}\r\n", key, value));
        }
        
        response.push_str("\r\n");
        response.push_str(&self.body);
        
        response
    }
}
```

```rust
// src/router.rs
use crate::{Request, Response};
use std::collections::HashMap;

pub type Handler = Box<dyn Fn(Request) -> Response + Send + Sync>;

#[derive(Clone)]
pub struct Router {
    routes: HashMap<String, HashMap<String, Handler>>,
}

impl Router {
    pub fn new() -> Self {
        Router {
            routes: HashMap::new(),
        }
    }
    
    pub fn get<F>(&mut self, path: &str, handler: F)
    where
        F: Fn(Request) -> Response + Send + Sync + 'static,
    {
        self.add_route("GET", path, Box::new(handler));
    }
    
    pub fn post<F>(&mut self, path: &str, handler: F)
    where
        F: Fn(Request) -> Response + Send + Sync + 'static,
    {
        self.add_route("POST", path, Box::new(handler));
    }
    
    fn add_route(&mut self, method: &str, path: &str, handler: Handler) {
        self.routes
            .entry(method.to_string())
            .or_insert_with(HashMap::new)
            .insert(path.to_string(), handler);
    }
    
    pub fn handle(&self, request: Request) -> Response {
        if let Some(method_routes) = self.routes.get(&request.method) {
            if let Some(handler) = method_routes.get(&request.path) {
                return handler(request);
            }
        }
        
        Response::not_found()
    }
}
```

```rust
// src/middleware.rs
use crate::{Request, Response};

pub trait Middleware {
    fn process(&self, request: Request, next: Box<dyn Fn(Request) -> Response>) -> Response;
}

pub struct Logger;

impl Middleware for Logger {
    fn process(&self, request: Request, next: Box<dyn Fn(Request) -> Response>) -> Response {
        println!("{} {}", request.method, request.path);
        let response = next(request);
        println!("Response: {}", response.status_code);
        response
    }
}

pub struct Cors {
    allowed_origins: Vec<String>,
}

impl Cors {
    pub fn new(origins: Vec<String>) -> Self {
        Cors {
            allowed_origins: origins,
        }
    }
}

impl Middleware for Cors {
    fn process(&self, request: Request, next: Box<dyn Fn(Request) -> Response>) -> Response {
        let mut response = next(request);
        
        response.set_header(
            "Access-Control-Allow-Origin".to_string(),
            self.allowed_origins.join(", "),
        );
        response.set_header(
            "Access-Control-Allow-Methods".to_string(),
            "GET, POST, PUT, DELETE".to_string(),
        );
        
        response
    }
}
```

### 6.2 使用示例

```rust
// examples/basic_server.rs
use web_server::prelude::*;

fn main() -> std::io::Result<()> {
    let mut server = Server::new("127.0.0.1:8080");
    
    // 添加路由
    server.router().get("/", |_req| {
        Response::ok("<h1>Welcome to Rust Web Server!</h1>".to_string())
    });
    
    server.router().get("/about", |_req| {
        Response::ok("<h1>About Page</h1><p>This is a simple web server built with Rust.</p>".to_string())
    });
    
    server.router().post("/api/data", |req| {
        println!("Received data: {}", req.body);
        Response::ok(r#"{"status": "success"}"#.to_string())
    });
    
    server.run()
}
```

## 7. 最佳实践

### 7.1 模块组织原则

```rust
// 按功能组织模块
mod user {
    pub mod authentication;
    pub mod profile;
    pub mod permissions;
}

mod product {
    pub mod catalog;
    pub mod inventory;
    pub mod pricing;
}

mod order {
    pub mod cart;
    pub mod checkout;
    pub mod fulfillment;
}

// 公共工具模块
mod utils {
    pub mod validation;
    pub mod formatting;
    pub mod encryption;
}

// 错误处理模块
mod error {
    pub mod types;
    pub mod handlers;
}

// 配置模块
mod config {
    pub mod database;
    pub mod server;
    pub mod logging;
}
```

### 7.2 API设计

```rust
// src/lib.rs - 清晰的公共API
pub mod core {
    pub use crate::user::User;
    pub use crate::product::Product;
    pub use crate::order::Order;
}

pub mod services {
    pub use crate::user::UserService;
    pub use crate::product::ProductService;
    pub use crate::order::OrderService;
}

pub mod errors {
    pub use crate::error::types::*;
}

// 便利的预导入模块
pub mod prelude {
    pub use crate::core::*;
    pub use crate::services::*;
    pub use crate::errors::*;
}

// 重新导出最常用的类型
pub use crate::core::*;
```

### 7.3 测试模块组织

```rust
// src/user/mod.rs
pub mod authentication;
pub mod profile;

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_user_creation() {
        // 测试用户创建
    }
}
```

```rust
// tests/integration_tests.rs
use my_crate::prelude::*;

#[test]
fn test_full_workflow() {
    // 集成测试
}
```

### 7.4 文档和示例

```rust
//! # Restaurant Management System
//! 
//! This crate provides a comprehensive restaurant management system
//! with modules for front-of-house operations, kitchen management,
//! and inventory tracking.
//! 
//! ## Quick Start
//! 
//! ```rust
//! use restaurant::prelude::*;
//! 
//! let mut restaurant = Restaurant::new("My Restaurant");
//! restaurant.add_table(Table::new(4)); // 4-person table
//! restaurant.open_for_business();
//! ```
//! 
//! ## Modules
//! 
//! - [`front_of_house`] - Customer-facing operations
//! - [`back_of_house`] - Kitchen and food preparation
//! - [`inventory`] - Stock and supply management
//! - [`reporting`] - Analytics and reports

pub mod front_of_house;
pub mod back_of_house;
pub mod inventory;
pub mod reporting;

/// Convenience re-exports for common types
pub mod prelude {
    pub use crate::front_of_house::{Table, Reservation};
    pub use crate::back_of_house::{Kitchen, Order};
    pub use crate::inventory::Inventory;
}

/// Main restaurant struct that coordinates all operations
/// 
/// # Examples
/// 
/// ```rust
/// use restaurant::Restaurant;
/// 
/// let restaurant = Restaurant::new("Rusty's Diner");
/// assert_eq!(restaurant.name(), "Rusty's Diner");
/// ```
pub struct Restaurant {
    name: String,
    // ... other fields
}

impl Restaurant {
    /// Creates a new restaurant with the given name
    pub fn new(name: &str) -> Self {
        Restaurant {
            name: name.to_string(),
        }
    }
    
    /// Returns the restaurant's name
    pub fn name(&self) -> &str {
        &self.name
    }
}
```

## 总结

通过本章的学习，你应该掌握了：

### 核心概念
- 模块的定义和嵌套结构
- 可见性规则和访问控制
- 绝对路径和相对路径的使用
- 包和crate的概念

### 实用技能
- 使用`use`语句简化代码
- 组织大型项目的模块结构
- 文件系统模块的创建和管理
- 重新导出API的设计

### 最佳实践
- 按功能组织模块
- 设计清晰的公共API
- 合理使用可见性修饰符
- 文档和示例的编写

### 高级特性
- 工作空间的配置和使用
- 中间件模式的实现
- 模块化架构设计
- 测试模块的组织

Rust的模块系统为构建大型、可维护的应用程序提供了强大的工具。通过合理的模块组织和API设计，你可以创建既易于使用又易于维护的代码库。
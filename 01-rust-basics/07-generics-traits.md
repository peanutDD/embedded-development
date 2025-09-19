# 泛型与特征

## 概述

泛型（Generics）和特征（Traits）是Rust类型系统的核心特性，它们提供了代码复用和抽象的强大机制。泛型允许我们编写可以处理多种类型的代码，而特征定义了类型必须实现的行为。这两个特性结合使用，可以创建既灵活又类型安全的代码。

## 学习目标

- 掌握泛型函数、结构体和枚举的定义和使用
- 理解特征的定义、实现和使用方法
- 学会使用特征约束限制泛型类型
- 掌握特征对象的概念和动态分发
- 理解关联类型和泛型的区别
- 学会设计和实现复杂的泛型系统
- 掌握高级特征特性如默认实现、超特征等

## 1. 泛型基础

### 1.1 泛型函数

```rust
// 基本泛型函数
fn largest<T: PartialOrd + Copy>(list: &[T]) -> T {
    let mut largest = list[0];
    
    for &item in list {
        if item > largest {
            largest = item;
        }
    }
    
    largest
}

// 多个泛型参数
fn compare_and_display<T, U>(t: &T, u: &U) -> bool 
where
    T: std::fmt::Display + PartialEq<U>,
    U: std::fmt::Display,
{
    println!("Comparing {} and {}", t, u);
    t == u
}

// 泛型函数的不同约束方式
fn process_data<T>(data: T) -> T 
where 
    T: Clone + std::fmt::Debug,
{
    println!("Processing: {:?}", data);
    data.clone()
}

fn main() {
    let number_list = vec![34, 50, 25, 100, 65];
    let result = largest(&number_list);
    println!("The largest number is {}", result);
    
    let char_list = vec!['y', 'm', 'a', 'q'];
    let result = largest(&char_list);
    println!("The largest char is {}", result);
    
    // 使用多泛型参数函数
    let x = 5;
    let y = 5.0;
    // compare_and_display(&x, &y); // 这会编译错误，因为i32和f64不能直接比较
    
    let processed = process_data(vec![1, 2, 3]);
    println!("Processed data: {:?}", processed);
}
```

### 1.2 泛型结构体

```rust
// 单个泛型参数的结构体
#[derive(Debug)]
struct Point<T> {
    x: T,
    y: T,
}

impl<T> Point<T> {
    fn new(x: T, y: T) -> Self {
        Point { x, y }
    }
    
    fn x(&self) -> &T {
        &self.x
    }
    
    fn y(&self) -> &T {
        &self.y
    }
}

// 为特定类型实现方法
impl Point<f32> {
    fn distance_from_origin(&self) -> f32 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }
}

// 多个泛型参数的结构体
#[derive(Debug)]
struct Pair<T, U> {
    first: T,
    second: U,
}

impl<T, U> Pair<T, U> {
    fn new(first: T, second: U) -> Self {
        Pair { first, second }
    }
    
    fn into_tuple(self) -> (T, U) {
        (self.first, self.second)
    }
    
    // 混合泛型方法
    fn mixup<V, W>(self, other: Pair<V, W>) -> Pair<T, W> {
        Pair {
            first: self.first,
            second: other.second,
        }
    }
}

// 带约束的泛型结构体
#[derive(Debug)]
struct Container<T> 
where 
    T: Clone + std::fmt::Display,
{
    value: T,
}

impl<T> Container<T> 
where 
    T: Clone + std::fmt::Display,
{
    fn new(value: T) -> Self {
        Container { value }
    }
    
    fn display_and_clone(&self) -> T {
        println!("Value: {}", self.value);
        self.value.clone()
    }
}

fn main() {
    // 使用泛型结构体
    let integer_point = Point::new(5, 10);
    let float_point = Point::new(1.0, 4.0);
    
    println!("Integer point: {:?}", integer_point);
    println!("Float point: {:?}", float_point);
    println!("Distance from origin: {}", float_point.distance_from_origin());
    
    // 使用多泛型参数结构体
    let pair = Pair::new("hello", 42);
    println!("Pair: {:?}", pair);
    
    let pair2 = Pair::new(1.5, true);
    let mixed = pair.mixup(pair2);
    println!("Mixed pair: {:?}", mixed);
    
    // 使用带约束的泛型结构体
    let container = Container::new("Hello, World!".to_string());
    let cloned_value = container.display_and_clone();
    println!("Cloned: {}", cloned_value);
}
```

### 1.3 泛型枚举

```rust
// 标准库中的Option和Result就是泛型枚举的例子
// 自定义泛型枚举
#[derive(Debug)]
enum MyResult<T, E> {
    Ok(T),
    Err(E),
}

impl<T, E> MyResult<T, E> {
    fn is_ok(&self) -> bool {
        matches!(self, MyResult::Ok(_))
    }
    
    fn is_err(&self) -> bool {
        matches!(self, MyResult::Err(_))
    }
    
    fn unwrap(self) -> T 
    where 
        E: std::fmt::Debug,
    {
        match self {
            MyResult::Ok(value) => value,
            MyResult::Err(error) => panic!("Called unwrap on an Err value: {:?}", error),
        }
    }
    
    fn map<U, F>(self, f: F) -> MyResult<U, E> 
    where 
        F: FnOnce(T) -> U,
    {
        match self {
            MyResult::Ok(value) => MyResult::Ok(f(value)),
            MyResult::Err(error) => MyResult::Err(error),
        }
    }
}

// 更复杂的泛型枚举
#[derive(Debug)]
enum Tree<T> {
    Empty,
    Node {
        value: T,
        left: Box<Tree<T>>,
        right: Box<Tree<T>>,
    },
}

impl<T> Tree<T> 
where 
    T: PartialOrd + Clone,
{
    fn new() -> Self {
        Tree::Empty
    }
    
    fn insert(&mut self, value: T) {
        match self {
            Tree::Empty => {
                *self = Tree::Node {
                    value,
                    left: Box::new(Tree::Empty),
                    right: Box::new(Tree::Empty),
                };
            }
            Tree::Node { value: node_value, left, right } => {
                if value < *node_value {
                    left.insert(value);
                } else {
                    right.insert(value);
                }
            }
        }
    }
    
    fn contains(&self, value: &T) -> bool {
        match self {
            Tree::Empty => false,
            Tree::Node { value: node_value, left, right } => {
                if value == node_value {
                    true
                } else if value < node_value {
                    left.contains(value)
                } else {
                    right.contains(value)
                }
            }
        }
    }
}

fn main() {
    // 使用自定义Result
    let success: MyResult<i32, String> = MyResult::Ok(42);
    let failure: MyResult<i32, String> = MyResult::Err("Something went wrong".to_string());
    
    println!("Success is ok: {}", success.is_ok());
    println!("Failure is err: {}", failure.is_err());
    
    let doubled = success.map(|x| x * 2);
    println!("Doubled: {:?}", doubled);
    
    // 使用泛型树
    let mut tree = Tree::new();
    tree.insert(5);
    tree.insert(3);
    tree.insert(7);
    tree.insert(1);
    tree.insert(9);
    
    println!("Tree contains 3: {}", tree.contains(&3));
    println!("Tree contains 6: {}", tree.contains(&6));
    println!("Tree: {:?}", tree);
}
```

## 2. 特征基础

### 2.1 特征定义和实现

```rust
// 定义特征
trait Summary {
    fn summarize(&self) -> String;
    
    // 默认实现
    fn summarize_author(&self) -> String {
        String::from("(Read more...)")
    }
    
    // 使用其他方法的默认实现
    fn summarize_with_author(&self) -> String {
        format!("Summary: {}, Author: {}", self.summarize(), self.summarize_author())
    }
}

// 为结构体实现特征
struct NewsArticle {
    headline: String,
    location: String,
    author: String,
    content: String,
}

impl Summary for NewsArticle {
    fn summarize(&self) -> String {
        format!("{}, by {} ({})", self.headline, self.author, self.location)
    }
    
    fn summarize_author(&self) -> String {
        format!("@{}", self.author)
    }
}

struct Tweet {
    username: String,
    content: String,
    reply: bool,
    retweet: bool,
}

impl Summary for Tweet {
    fn summarize(&self) -> String {
        format!("{}: {}", self.username, self.content)
    }
    
    fn summarize_author(&self) -> String {
        format!("@{}", self.username)
    }
}

// 只实现必需的方法，使用默认实现
struct BlogPost {
    title: String,
    content: String,
}

impl Summary for BlogPost {
    fn summarize(&self) -> String {
        format!("Blog: {}", self.title)
    }
}

fn main() {
    let article = NewsArticle {
        headline: "Penguins win the Stanley Cup Championship!".to_string(),
        location: "Pittsburgh, PA, USA".to_string(),
        author: "Iceburgh".to_string(),
        content: "The Pittsburgh Penguins once again are the best hockey team in the NHL.".to_string(),
    };
    
    let tweet = Tweet {
        username: "horse_ebooks".to_string(),
        content: "of course, as you probably already know, people".to_string(),
        reply: false,
        retweet: false,
    };
    
    let blog = BlogPost {
        title: "Learning Rust".to_string(),
        content: "Rust is a great programming language...".to_string(),
    };
    
    println!("Article: {}", article.summarize());
    println!("Tweet: {}", tweet.summarize_with_author());
    println!("Blog: {}", blog.summarize_with_author());
}
```

### 2.2 特征作为参数

```rust
// 使用impl Trait语法
fn notify(item: &impl Summary) {
    println!("Breaking news! {}", item.summarize());
}

// 使用特征约束语法
fn notify_generic<T: Summary>(item: &T) {
    println!("Breaking news! {}", item.summarize());
}

// 多个特征约束
fn notify_multiple(item: &(impl Summary + std::fmt::Display)) {
    println!("Breaking news! {}", item.summarize());
}

// 使用where子句
fn notify_where<T>(item: &T) 
where 
    T: Summary + std::fmt::Display,
{
    println!("Breaking news! {}", item.summarize());
}

// 多个参数的特征约束
fn notify_two<T, U>(t: &T, u: &U) -> String 
where 
    T: Summary + std::fmt::Display,
    U: Clone + std::fmt::Debug,
{
    format!("t: {}, u: {:?}", t.summarize(), u)
}

// 返回实现特征的类型
fn returns_summarizable() -> impl Summary {
    Tweet {
        username: "horse_ebooks".to_string(),
        content: "of course, as you probably already know, people".to_string(),
        reply: false,
        retweet: false,
    }
}

// 条件性实现特征
struct Pair<T> {
    x: T,
    y: T,
}

impl<T> Pair<T> {
    fn new(x: T, y: T) -> Self {
        Self { x, y }
    }
}

impl<T: std::fmt::Display + PartialOrd> Pair<T> {
    fn cmp_display(&self) {
        if self.x >= self.y {
            println!("The largest member is x = {}", self.x);
        } else {
            println!("The largest member is y = {}", self.y);
        }
    }
}

fn main() {
    let tweet = Tweet {
        username: "horse_ebooks".to_string(),
        content: "of course, as you probably already know, people".to_string(),
        reply: false,
        retweet: false,
    };
    
    notify(&tweet);
    notify_generic(&tweet);
    
    let returned_tweet = returns_summarizable();
    println!("Returned: {}", returned_tweet.summarize());
    
    let pair = Pair::new(10, 20);
    pair.cmp_display();
}
```

### 2.3 标准库中的重要特征

```rust
use std::fmt;

// 实现Display特征
struct Point {
    x: i32,
    y: i32,
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

// 实现Debug特征（通常使用derive）
#[derive(Debug)]
struct Rectangle {
    width: u32,
    height: u32,
}

// 手动实现Debug
impl fmt::Debug for Point {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Point")
            .field("x", &self.x)
            .field("y", &self.y)
            .finish()
    }
}

// 实现Clone特征
#[derive(Clone)]
struct Person {
    name: String,
    age: u32,
}

// 手动实现Clone
impl Clone for Point {
    fn clone(&self) -> Self {
        Point {
            x: self.x,
            y: self.y,
        }
    }
}

// 实现PartialEq和Eq
#[derive(PartialEq, Eq)]
struct Book {
    title: String,
    author: String,
}

// 手动实现PartialEq
impl PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

// 实现PartialOrd和Ord
#[derive(PartialEq, Eq, PartialOrd, Ord)]
struct Student {
    name: String,
    grade: u32,
}

// 实现From和Into特征
struct Kilometers(f64);
struct Miles(f64);

impl From<Miles> for Kilometers {
    fn from(miles: Miles) -> Self {
        Kilometers(miles.0 * 1.60934)
    }
}

// Into会自动实现
// impl Into<Kilometers> for Miles { ... }

// 实现TryFrom和TryInto
use std::convert::TryFrom;

#[derive(Debug)]
struct PositiveNumber(u32);

#[derive(Debug)]
enum PositiveNumberError {
    Negative,
}

impl TryFrom<i32> for PositiveNumber {
    type Error = PositiveNumberError;
    
    fn try_from(value: i32) -> Result<Self, Self::Error> {
        if value >= 0 {
            Ok(PositiveNumber(value as u32))
        } else {
            Err(PositiveNumberError::Negative)
        }
    }
}

fn main() {
    let point = Point { x: 1, y: 2 };
    println!("Point: {}", point);  // 使用Display
    println!("Point debug: {:?}", point);  // 使用Debug
    
    let cloned_point = point.clone();
    println!("Cloned: {:?}", cloned_point);
    
    let point2 = Point { x: 1, y: 2 };
    println!("Points equal: {}", point == point2);
    
    // 使用From/Into
    let miles = Miles(10.0);
    let km: Kilometers = miles.into();  // 或者 Kilometers::from(miles)
    println!("10 miles = {} km", km.0);
    
    // 使用TryFrom/TryInto
    let positive = PositiveNumber::try_from(42);
    let negative = PositiveNumber::try_from(-5);
    
    println!("Positive: {:?}", positive);
    println!("Negative: {:?}", negative);
}
```

## 3. 高级特征特性

### 3.1 关联类型

```rust
// 使用关联类型的迭代器特征
trait Iterator {
    type Item;  // 关联类型
    
    fn next(&mut self) -> Option<Self::Item>;
    
    // 默认实现可以使用关联类型
    fn collect<B: FromIterator<Self::Item>>(self) -> B 
    where 
        Self: Sized,
    {
        FromIterator::from_iter(self)
    }
}

// 实现自定义迭代器
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
    type Item = usize;  // 指定关联类型
    
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

// 关联类型 vs 泛型的比较
trait Graph {
    type Node;
    type Edge;
    
    fn has_edge(&self, from: &Self::Node, to: &Self::Node) -> bool;
    fn edges(&self, from: &Self::Node) -> Vec<Self::Edge>;
}

// 如果使用泛型，会是这样：
// trait Graph<Node, Edge> {
//     fn has_edge(&self, from: &Node, to: &Node) -> bool;
//     fn edges(&self, from: &Node) -> Vec<Edge>;
// }

// 实现Graph特征
struct SimpleGraph {
    edges: Vec<(usize, usize)>,
}

impl Graph for SimpleGraph {
    type Node = usize;
    type Edge = (usize, usize);
    
    fn has_edge(&self, from: &Self::Node, to: &Self::Node) -> bool {
        self.edges.contains(&(*from, *to))
    }
    
    fn edges(&self, from: &Self::Node) -> Vec<Self::Edge> {
        self.edges.iter()
            .filter(|(f, _)| f == from)
            .cloned()
            .collect()
    }
}

fn main() {
    let mut counter = Counter::new(5);
    
    while let Some(value) = counter.next() {
        println!("Counter: {}", value);
    }
    
    // 使用collect方法
    let values: Vec<usize> = Counter::new(3).collect();
    println!("Collected: {:?}", values);
    
    let graph = SimpleGraph {
        edges: vec![(0, 1), (1, 2), (0, 2), (2, 0)],
    };
    
    println!("Has edge 0->1: {}", graph.has_edge(&0, &1));
    println!("Edges from 0: {:?}", graph.edges(&0));
}
```

### 3.2 默认泛型类型参数

```rust
use std::ops::Add;

// 定义带默认泛型参数的特征
trait MyAdd<Rhs = Self> {
    type Output;
    
    fn add(self, rhs: Rhs) -> Self::Output;
}

// 为Point实现Add特征
#[derive(Debug, PartialEq)]
struct Point {
    x: i32,
    y: i32,
}

impl Add for Point {
    type Output = Point;
    
    fn add(self, other: Point) -> Point {
        Point {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

// 实现Point与标量的加法
impl Add<i32> for Point {
    type Output = Point;
    
    fn add(self, scalar: i32) -> Point {
        Point {
            x: self.x + scalar,
            y: self.y + scalar,
        }
    }
}

// 自定义类型的例子
struct Millimeters(u32);
struct Meters(u32);

impl Add<Meters> for Millimeters {
    type Output = Millimeters;
    
    fn add(self, other: Meters) -> Millimeters {
        Millimeters(self.0 + (other.0 * 1000))
    }
}

fn main() {
    let p1 = Point { x: 1, y: 0 };
    let p2 = Point { x: 2, y: 3 };
    
    let p3 = p1 + p2;
    println!("Point addition: {:?}", p3);
    
    let p4 = Point { x: 1, y: 2 };
    let p5 = p4 + 5;  // 使用标量加法
    println!("Point + scalar: {:?}", p5);
    
    let mm = Millimeters(1000);
    let m = Meters(1);
    let result = mm + m;
    println!("Millimeters result: {}", result.0);
}
```

### 3.3 完全限定语法和消歧义

```rust
trait Pilot {
    fn fly(&self);
    fn name() -> String;
}

trait Wizard {
    fn fly(&self);
    fn name() -> String;
}

struct Human;

impl Pilot for Human {
    fn fly(&self) {
        println!("This is your captain speaking.");
    }
    
    fn name() -> String {
        String::from("Captain")
    }
}

impl Wizard for Human {
    fn fly(&self) {
        println!("Up!");
    }
    
    fn name() -> String {
        String::from("Merlin")
    }
}

impl Human {
    fn fly(&self) {
        println!("*waving arms furiously*");
    }
    
    fn name() -> String {
        String::from("Human")
    }
}

// 演示孤儿规则的绕过
trait OutlinePrint: fmt::Display {
    fn outline_print(&self) {
        let output = self.to_string();
        let len = output.len();
        println!("{}", "*".repeat(len + 4));
        println!("*{}*", " ".repeat(len + 2));
        println!("* {} *", output);
        println!("*{}*", " ".repeat(len + 2));
        println!("{}", "*".repeat(len + 4));
    }
}

use std::fmt;

struct Point {
    x: i32,
    y: i32,
}

impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

impl OutlinePrint for Point {}

// newtype模式
struct Wrapper(Vec<String>);

impl fmt::Display for Wrapper {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "[{}]", self.0.join(", "))
    }
}

fn main() {
    let person = Human;
    
    // 调用不同的fly方法
    person.fly();                    // 调用Human的方法
    Pilot::fly(&person);            // 调用Pilot的方法
    Wizard::fly(&person);           // 调用Wizard的方法
    
    // 对于关联函数，需要完全限定语法
    println!("Human name: {}", Human::name());
    println!("Pilot name: {}", <Human as Pilot>::name());
    println!("Wizard name: {}", <Human as Wizard>::name());
    
    // 使用超特征
    let point = Point { x: 1, y: 3 };
    point.outline_print();
    
    // 使用newtype模式
    let w = Wrapper(vec![String::from("hello"), String::from("world")]);
    println!("w = {}", w);
}
```

### 3.4 超特征（Supertraits）

```rust
use std::fmt;

// 定义超特征
trait OutlinePrint: fmt::Display {
    fn outline_print(&self) {
        let output = self.to_string();
        let len = output.len();
        println!("{}", "*".repeat(len + 4));
        println!("*{}*", " ".repeat(len + 2));
        println!("* {} *", output);
        println!("*{}*", " ".repeat(len + 2));
        println!("{}", "*".repeat(len + 4));
    }
}

// 更复杂的超特征示例
trait Animal {
    fn name(&self) -> &str;
    fn noise(&self) -> &str;
}

trait Dog: Animal {
    fn breed(&self) -> &str;
    
    fn bark(&self) {
        println!("{} says {}", self.name(), self.noise());
    }
}

trait ServiceDog: Dog {
    fn service_type(&self) -> &str;
    
    fn work(&self) {
        println!("{} is working as a {}", self.name(), self.service_type());
    }
}

// 实现特征层次结构
struct GoldenRetriever {
    name: String,
}

impl Animal for GoldenRetriever {
    fn name(&self) -> &str {
        &self.name
    }
    
    fn noise(&self) -> &str {
        "Woof!"
    }
}

impl Dog for GoldenRetriever {
    fn breed(&self) -> &str {
        "Golden Retriever"
    }
}

impl ServiceDog for GoldenRetriever {
    fn service_type(&self) -> &str {
        "Guide Dog"
    }
}

// 使用特征约束的函数
fn train_service_dog<T: ServiceDog>(dog: &T) {
    println!("Training {} ({})", dog.name(), dog.breed());
    dog.bark();
    dog.work();
}

fn main() {
    let buddy = GoldenRetriever {
        name: "Buddy".to_string(),
    };
    
    train_service_dog(&buddy);
}
```

## 4. 特征对象和动态分发

### 4.1 特征对象基础

```rust
trait Draw {
    fn draw(&self);
}

struct Circle {
    radius: f64,
}

struct Rectangle {
    width: f64,
    height: f64,
}

impl Draw for Circle {
    fn draw(&self) {
        println!("Drawing a circle with radius {}", self.radius);
    }
}

impl Draw for Rectangle {
    fn draw(&self) {
        println!("Drawing a rectangle {}x{}", self.width, self.height);
    }
}

// 使用特征对象的结构体
struct Screen {
    components: Vec<Box<dyn Draw>>,
}

impl Screen {
    fn new() -> Self {
        Screen {
            components: Vec::new(),
        }
    }
    
    fn add_component(&mut self, component: Box<dyn Draw>) {
        self.components.push(component);
    }
    
    fn run(&self) {
        for component in self.components.iter() {
            component.draw();
        }
    }
}

// 对象安全的特征
trait Clone2 {
    fn clone_box(&self) -> Box<dyn Clone2>;
}

impl<T> Clone2 for T 
where 
    T: 'static + Clone + Clone2,
{
    fn clone_box(&self) -> Box<dyn Clone2> {
        Box::new(self.clone())
    }
}

// 不是对象安全的特征示例
trait NotObjectSafe {
    fn generic_method<T>(&self, t: T);  // 泛型方法
    fn return_self() -> Self;           // 返回Self
}

fn main() {
    let mut screen = Screen::new();
    
    screen.add_component(Box::new(Circle { radius: 5.0 }));
    screen.add_component(Box::new(Rectangle { width: 10.0, height: 20.0 }));
    
    screen.run();
    
    // 使用特征对象的向量
    let shapes: Vec<Box<dyn Draw>> = vec![
        Box::new(Circle { radius: 3.0 }),
        Box::new(Rectangle { width: 5.0, height: 8.0 }),
    ];
    
    for shape in &shapes {
        shape.draw();
    }
    
    // 使用引用的特征对象
    let circle = Circle { radius: 2.0 };
    let rectangle = Rectangle { width: 4.0, height: 6.0 };
    
    let shapes_ref: Vec<&dyn Draw> = vec![&circle, &rectangle];
    
    for shape in &shapes_ref {
        shape.draw();
    }
}
```

### 4.2 动态分发 vs 静态分发

```rust
trait Animal {
    fn make_sound(&self);
    fn name(&self) -> &str;
}

struct Dog {
    name: String,
}

struct Cat {
    name: String,
}

impl Animal for Dog {
    fn make_sound(&self) {
        println!("Woof!");
    }
    
    fn name(&self) -> &str {
        &self.name
    }
}

impl Animal for Cat {
    fn make_sound(&self) {
        println!("Meow!");
    }
    
    fn name(&self) -> &str {
        &self.name
    }
}

// 静态分发 - 编译时确定类型
fn static_dispatch<T: Animal>(animal: &T) {
    println!("{} says:", animal.name());
    animal.make_sound();
}

// 动态分发 - 运行时确定类型
fn dynamic_dispatch(animal: &dyn Animal) {
    println!("{} says:", animal.name());
    animal.make_sound();
}

// 返回特征对象
fn create_animal(animal_type: &str) -> Box<dyn Animal> {
    match animal_type {
        "dog" => Box::new(Dog { name: "Buddy".to_string() }),
        "cat" => Box::new(Cat { name: "Whiskers".to_string() }),
        _ => panic!("Unknown animal type"),
    }
}

// 存储不同类型的动物
struct Zoo {
    animals: Vec<Box<dyn Animal>>,
}

impl Zoo {
    fn new() -> Self {
        Zoo { animals: Vec::new() }
    }
    
    fn add_animal(&mut self, animal: Box<dyn Animal>) {
        self.animals.push(animal);
    }
    
    fn feeding_time(&self) {
        println!("Feeding time at the zoo!");
        for animal in &self.animals {
            println!("Feeding {}", animal.name());
            animal.make_sound();
        }
    }
}

fn main() {
    let dog = Dog { name: "Rex".to_string() };
    let cat = Cat { name: "Fluffy".to_string() };
    
    // 静态分发
    static_dispatch(&dog);
    static_dispatch(&cat);
    
    // 动态分发
    dynamic_dispatch(&dog);
    dynamic_dispatch(&cat);
    
    // 运行时创建动物
    let runtime_dog = create_animal("dog");
    let runtime_cat = create_animal("cat");
    
    dynamic_dispatch(runtime_dog.as_ref());
    dynamic_dispatch(runtime_cat.as_ref());
    
    // 使用动物园
    let mut zoo = Zoo::new();
    zoo.add_animal(Box::new(Dog { name: "Max".to_string() }));
    zoo.add_animal(Box::new(Cat { name: "Shadow".to_string() }));
    zoo.add_animal(create_animal("dog"));
    
    zoo.feeding_time();
}
```

## 5. 实践示例

### 5.1 插件系统

```rust
use std::collections::HashMap;

// 定义插件特征
trait Plugin: Send + Sync {
    fn name(&self) -> &str;
    fn version(&self) -> &str;
    fn execute(&self, input: &str) -> Result<String, String>;
    fn description(&self) -> &str {
        "No description available"
    }
}

// 插件管理器
struct PluginManager {
    plugins: HashMap<String, Box<dyn Plugin>>,
}

impl PluginManager {
    fn new() -> Self {
        PluginManager {
            plugins: HashMap::new(),
        }
    }
    
    fn register_plugin(&mut self, plugin: Box<dyn Plugin>) {
        let name = plugin.name().to_string();
        self.plugins.insert(name, plugin);
    }
    
    fn execute_plugin(&self, name: &str, input: &str) -> Result<String, String> {
        match self.plugins.get(name) {
            Some(plugin) => plugin.execute(input),
            None => Err(format!("Plugin '{}' not found", name)),
        }
    }
    
    fn list_plugins(&self) {
        println!("Available plugins:");
        for plugin in self.plugins.values() {
            println!("  {} v{}: {}", 
                    plugin.name(), 
                    plugin.version(), 
                    plugin.description());
        }
    }
}

// 具体插件实现
struct UpperCasePlugin;

impl Plugin for UpperCasePlugin {
    fn name(&self) -> &str {
        "uppercase"
    }
    
    fn version(&self) -> &str {
        "1.0.0"
    }
    
    fn execute(&self, input: &str) -> Result<String, String> {
        Ok(input.to_uppercase())
    }
    
    fn description(&self) -> &str {
        "Converts text to uppercase"
    }
}

struct ReversePlugin;

impl Plugin for ReversePlugin {
    fn name(&self) -> &str {
        "reverse"
    }
    
    fn version(&self) -> &str {
        "1.0.0"
    }
    
    fn execute(&self, input: &str) -> Result<String, String> {
        Ok(input.chars().rev().collect())
    }
    
    fn description(&self) -> &str {
        "Reverses the input text"
    }
}

struct WordCountPlugin;

impl Plugin for WordCountPlugin {
    fn name(&self) -> &str {
        "wordcount"
    }
    
    fn version(&self) -> &str {
        "1.0.0"
    }
    
    fn execute(&self, input: &str) -> Result<String, String> {
        let count = input.split_whitespace().count();
        Ok(format!("Word count: {}", count))
    }
    
    fn description(&self) -> &str {
        "Counts the number of words in the input"
    }
}

fn main() {
    let mut manager = PluginManager::new();
    
    // 注册插件
    manager.register_plugin(Box::new(UpperCasePlugin));
    manager.register_plugin(Box::new(ReversePlugin));
    manager.register_plugin(Box::new(WordCountPlugin));
    
    manager.list_plugins();
    
    let test_input = "Hello, World! This is a test.";
    
    // 执行插件
    match manager.execute_plugin("uppercase", test_input) {
        Ok(result) => println!("Uppercase: {}", result),
        Err(e) => println!("Error: {}", e),
    }
    
    match manager.execute_plugin("reverse", test_input) {
        Ok(result) => println!("Reverse: {}", result),
        Err(e) => println!("Error: {}", e),
    }
    
    match manager.execute_plugin("wordcount", test_input) {
        Ok(result) => println!("{}", result),
        Err(e) => println!("Error: {}", e),
    }
}
```

### 5.2 序列化框架

```rust
use std::collections::HashMap;

// 定义序列化特征
trait Serialize {
    fn serialize(&self) -> String;
}

trait Deserialize: Sized {
    fn deserialize(data: &str) -> Result<Self, String>;
}

// 为基本类型实现序列化
impl Serialize for i32 {
    fn serialize(&self) -> String {
        self.to_string()
    }
}

impl Deserialize for i32 {
    fn deserialize(data: &str) -> Result<Self, String> {
        data.parse().map_err(|e| format!("Failed to parse i32: {}", e))
    }
}

impl Serialize for String {
    fn serialize(&self) -> String {
        format!("\"{}\"", self.replace("\"", "\\\""))
    }
}

impl Deserialize for String {
    fn deserialize(data: &str) -> Result<Self, String> {
        if data.starts_with('"') && data.ends_with('"') {
            let content = &data[1..data.len()-1];
            Ok(content.replace("\\\"", "\""))
        } else {
            Err("String must be quoted".to_string())
        }
    }
}

// 为Vec实现序列化
impl<T: Serialize> Serialize for Vec<T> {
    fn serialize(&self) -> String {
        let items: Vec<String> = self.iter().map(|item| item.serialize()).collect();
        format!("[{}]", items.join(","))
    }
}

impl<T: Deserialize> Deserialize for Vec<T> {
    fn deserialize(data: &str) -> Result<Self, String> {
        if !data.starts_with('[') || !data.ends_with(']') {
            return Err("Array must be enclosed in brackets".to_string());
        }
        
        let content = &data[1..data.len()-1].trim();
        if content.is_empty() {
            return Ok(Vec::new());
        }
        
        let items: Result<Vec<T>, String> = content
            .split(',')
            .map(|item| T::deserialize(item.trim()))
            .collect();
        
        items
    }
}

// 自定义结构体
#[derive(Debug, PartialEq)]
struct Person {
    name: String,
    age: i32,
    hobbies: Vec<String>,
}

impl Serialize for Person {
    fn serialize(&self) -> String {
        format!(
            "{{\"name\":{},\"age\":{},\"hobbies\":{}}}",
            self.name.serialize(),
            self.age.serialize(),
            self.hobbies.serialize()
        )
    }
}

impl Deserialize for Person {
    fn deserialize(data: &str) -> Result<Self, String> {
        // 简化的JSON解析（实际应用中应使用专门的JSON库）
        if !data.starts_with('{') || !data.ends_with('}') {
            return Err("Object must be enclosed in braces".to_string());
        }
        
        // 这里是一个简化的实现，实际应用中需要更复杂的解析逻辑
        let content = &data[1..data.len()-1];
        let mut name = String::new();
        let mut age = 0;
        let mut hobbies = Vec::new();
        
        // 简化的字段解析
        for part in content.split(',') {
            let kv: Vec<&str> = part.splitn(2, ':').collect();
            if kv.len() != 2 {
                continue;
            }
            
            let key = kv[0].trim().trim_matches('"');
            let value = kv[1].trim();
            
            match key {
                "name" => name = String::deserialize(value)?,
                "age" => age = i32::deserialize(value)?,
                "hobbies" => hobbies = Vec::<String>::deserialize(value)?,
                _ => {}
            }
        }
        
        Ok(Person { name, age, hobbies })
    }
}

// 泛型序列化器
struct Serializer;

impl Serializer {
    fn to_string<T: Serialize>(value: &T) -> String {
        value.serialize()
    }
    
    fn from_string<T: Deserialize>(data: &str) -> Result<T, String> {
        T::deserialize(data)
    }
}

fn main() {
    let person = Person {
        name: "Alice".to_string(),
        age: 30,
        hobbies: vec!["reading".to_string(), "swimming".to_string()],
    };
    
    // 序列化
    let serialized = Serializer::to_string(&person);
    println!("Serialized: {}", serialized);
    
    // 反序列化
    match Serializer::from_string::<Person>(&serialized) {
        Ok(deserialized) => {
            println!("Deserialized: {:?}", deserialized);
            println!("Equal: {}", person == deserialized);
        }
        Err(e) => println!("Deserialization error: {}", e),
    }
    
    // 测试其他类型
    let numbers = vec![1, 2, 3, 4, 5];
    let serialized_numbers = Serializer::to_string(&numbers);
    println!("Serialized numbers: {}", serialized_numbers);
    
    let deserialized_numbers: Result<Vec<i32>, String> = 
        Serializer::from_string(&serialized_numbers);
    println!("Deserialized numbers: {:?}", deserialized_numbers);
}
```

## 6. 性能考虑

### 6.1 零成本抽象

```rust
use std::time::Instant;

// 泛型版本（静态分发）
fn process_generic<T: std::fmt::Display>(items: &[T]) {
    for item in items {
        // 在编译时，这会被单态化为具体类型
        println!("{}", item);
    }
}

// 特征对象版本（动态分发）
fn process_dynamic(items: &[&dyn std::fmt::Display]) {
    for item in items {
        // 运行时通过虚函数表调用
        println!("{}", item);
    }
}

// 性能测试
fn performance_comparison() {
    let numbers: Vec<i32> = (0..1_000_000).collect();
    
    // 测试泛型版本
    let start = Instant::now();
    process_generic(&numbers);
    let generic_time = start.elapsed();
    
    // 测试特征对象版本
    let trait_objects: Vec<&dyn std::fmt::Display> = 
        numbers.iter().map(|n| n as &dyn std::fmt::Display).collect();
    
    let start = Instant::now();
    process_dynamic(&trait_objects);
    let dynamic_time = start.elapsed();
    
    println!("Generic time: {:?}", generic_time);
    println!("Dynamic time: {:?}", dynamic_time);
}

// 编译时优化示例
trait Calculate {
    fn calculate(&self) -> i32;
}

struct SimpleCalculator {
    value: i32,
}

impl Calculate for SimpleCalculator {
    #[inline]  // 提示编译器内联
    fn calculate(&self) -> i32 {
        self.value * 2
    }
}

// 使用const泛型进行编译时计算
struct Array<T, const N: usize> {
    data: [T; N],
}

impl<T, const N: usize> Array<T, N> {
    fn new(data: [T; N]) -> Self {
        Array { data }
    }
    
    fn len(&self) -> usize {
        N  // 编译时常量
    }
}

fn main() {
    // 演示零成本抽象
    let calc = SimpleCalculator { value: 42 };
    let result = calc.calculate();  // 可能被内联
    println!("Result: {}", result);
    
    // 使用const泛型
    let arr = Array::new([1, 2, 3, 4, 5]);
    println!("Array length: {}", arr.len());  // 编译时已知
    
    // 性能比较（注释掉以避免大量输出）
    // performance_comparison();
}
```

### 6.2 编译时优化技巧

```rust
// 使用关联常量
trait Config {
    const MAX_SIZE: usize;
    const DEFAULT_VALUE: i32;
}

struct MyConfig;

impl Config for MyConfig {
    const MAX_SIZE: usize = 1024;
    const DEFAULT_VALUE: i32 = 42;
}

// 使用const泛型进行编译时检查
fn process_array<T, const N: usize>(arr: &[T; N]) -> usize 
where
    T: std::fmt::Debug,
{
    println!("Processing array of size {}", N);
    N
}

// 使用PhantomData进行零大小类型标记
use std::marker::PhantomData;

struct TypedId<T> {
    id: u64,
    _marker: PhantomData<T>,
}

impl<T> TypedId<T> {
    fn new(id: u64) -> Self {
        TypedId {
            id,
            _marker: PhantomData,
        }
    }
    
    fn value(&self) -> u64 {
        self.id
    }
}

struct User;
struct Product;

// 类型安全的ID系统
type UserId = TypedId<User>;
type ProductId = TypedId<Product>;

fn get_user(id: UserId) -> String {
    format!("User with ID: {}", id.value())
}

fn get_product(id: ProductId) -> String {
    format!("Product with ID: {}", id.value())
}

fn main() {
    // 使用关联常量
    println!("Max size: {}", MyConfig::MAX_SIZE);
    println!("Default value: {}", MyConfig::DEFAULT_VALUE);
    
    // 使用const泛型
    let arr = [1, 2, 3, 4, 5];
    let size = process_array(&arr);
    println!("Processed size: {}", size);
    
    // 使用类型安全的ID
    let user_id = UserId::new(123);
    let product_id = ProductId::new(456);
    
    println!("{}", get_user(user_id));
    println!("{}", get_product(product_id));
    
    // 这会编译错误：类型不匹配
    // println!("{}", get_user(product_id));
}
```

## 7. 最佳实践

### 7.1 特征设计原则

```rust
// 好的实践：小而专注的特征
trait Read {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, std::io::Error>;
}

trait Write {
    fn write(&mut self, buf: &[u8]) -> Result<usize, std::io::Error>;
    fn flush(&mut self) -> Result<(), std::io::Error>;
}

// 组合特征
trait ReadWrite: Read + Write {}

// 避免：过大的特征
// trait BadFileOperations {
//     fn read(&mut self, buf: &mut [u8]) -> Result<usize, std::io::Error>;
//     fn write(&mut self, buf: &[u8]) -> Result<usize, std::io::Error>;
//     fn seek(&mut self, pos: u64) -> Result<u64, std::io::Error>;
//     fn metadata(&self) -> Result<Metadata, std::io::Error>;
//     fn permissions(&self) -> Result<Permissions, std::io::Error>;
//     // ... 更多方法
// }

// 好的实践：使用关联类型而不是泛型（当只有一种合理的类型时）
trait Iterator {
    type Item;
    fn next(&mut self) -> Option<Self::Item>;
}

// 避免：不必要的泛型
// trait BadIterator<T> {
//     fn next(&mut self) -> Option<T>;
// }

// 好的实践：提供有用的默认实现
trait Logger {
    fn log(&self, message: &str);
    
    fn log_info(&self, message: &str) {
        self.log(&format!("INFO: {}", message));
    }
    
    fn log_error(&self, message: &str) {
        self.log(&format!("ERROR: {}", message));
    }
}

struct ConsoleLogger;

impl Logger for ConsoleLogger {
    fn log(&self, message: &str) {
        println!("{}", message);
    }
}

// 好的实践：使用构建器模式与泛型
struct HttpRequest<S> {
    url: String,
    method: String,
    headers: std::collections::HashMap<String, String>,
    _state: std::marker::PhantomData<S>,
}

struct Incomplete;
struct Complete;

impl HttpRequest<Incomplete> {
    fn new() -> Self {
        HttpRequest {
            url: String::new(),
            method: "GET".to_string(),
            headers: std::collections::HashMap::new(),
            _state: std::marker::PhantomData,
        }
    }
    
    fn url(mut self, url: &str) -> Self {
        self.url = url.to_string();
        self
    }
    
    fn method(mut self, method: &str) -> Self {
        self.method = method.to_string();
        self
    }
    
    fn build(self) -> HttpRequest<Complete> {
        HttpRequest {
            url: self.url,
            method: self.method,
            headers: self.headers,
            _state: std::marker::PhantomData,
        }
    }
}

impl HttpRequest<Complete> {
    fn execute(&self) -> String {
        format!("Executing {} request to {}", self.method, self.url)
    }
}

fn main() {
    let logger = ConsoleLogger;
    logger.log_info("Application started");
    logger.log_error("Something went wrong");
    
    // 类型安全的构建器
    let request = HttpRequest::new()
        .url("https://api.example.com/users")
        .method("POST")
        .build();
    
    println!("{}", request.execute());
    
    // 这会编译错误：不能在未完成的请求上调用execute
    // let incomplete = HttpRequest::new().url("test");
    // incomplete.execute();
}
```

### 7.2 错误处理和泛型

```rust
use std::fmt;

// 定义通用错误特征
trait AppError: fmt::Debug + fmt::Display + Send + Sync + 'static {
    fn error_code(&self) -> u32;
    fn error_type(&self) -> &'static str;
}

// 具体错误类型
#[derive(Debug)]
struct ValidationError {
    field: String,
    message: String,
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Validation error in field '{}': {}", self.field, self.message)
    }
}

impl AppError for ValidationError {
    fn error_code(&self) -> u32 {
        1001
    }
    
    fn error_type(&self) -> &'static str {
        "ValidationError"
    }
}

#[derive(Debug)]
struct DatabaseError {
    query: String,
    cause: String,
}

impl fmt::Display for DatabaseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Database error executing '{}': {}", self.query, self.cause)
    }
}

impl AppError for DatabaseError {
    fn error_code(&self) -> u32 {
        2001
    }
    
    fn error_type(&self) -> &'static str {
        "DatabaseError"
    }
}

// 泛型结果类型
type AppResult<T> = Result<T, Box<dyn AppError>>;

// 使用泛型的服务
struct Service<R, D> 
where 
    R: Repository,
    D: Validator,
{
    repository: R,
    validator: D,
}

trait Repository {
    fn save(&self, data: &str) -> AppResult<String>;
    fn find(&self, id: &str) -> AppResult<Option<String>>;
}

trait Validator {
    fn validate(&self, data: &str) -> AppResult<()>;
}

impl<R, D> Service<R, D> 
where 
    R: Repository,
    D: Validator,
{
    fn new(repository: R, validator: D) -> Self {
        Service { repository, validator }
    }
    
    fn create_item(&self, data: &str) -> AppResult<String> {
        self.validator.validate(data)?;
        self.repository.save(data)
    }
}

// 具体实现
struct InMemoryRepository;

impl Repository for InMemoryRepository {
    fn save(&self, data: &str) -> AppResult<String> {
        if data.is_empty() {
            Err(Box::new(DatabaseError {
                query: "INSERT".to_string(),
                cause: "Empty data".to_string(),
            }))
        } else {
            Ok(format!("saved_{}", data))
        }
    }
    
    fn find(&self, id: &str) -> AppResult<Option<String>> {
        Ok(Some(format!("item_{}", id)))
    }
}

struct SimpleValidator;

impl Validator for SimpleValidator {
    fn validate(&self, data: &str) -> AppResult<()> {
        if data.len() < 3 {
            Err(Box::new(ValidationError {
                field: "data".to_string(),
                message: "Must be at least 3 characters".to_string(),
            }))
        } else {
            Ok(())
        }
    }
}

fn main() {
    let repository = InMemoryRepository;
    let validator = SimpleValidator;
    let service = Service::new(repository, validator);
    
    // 测试成功情况
    match service.create_item("valid_data") {
        Ok(result) => println!("Success: {}", result),
        Err(e) => println!("Error: {} (Code: {})", e, e.error_code()),
    }
    
    // 测试验证失败
    match service.create_item("ab") {
        Ok(result) => println!("Success: {}", result),
        Err(e) => println!("Error: {} (Code: {})", e, e.error_code()),
    }
    
    // 测试数据库错误
    match service.create_item("") {
        Ok(result) => println!("Success: {}", result),
        Err(e) => println!("Error: {} (Code: {})", e, e.error_code()),
    }
}
```

## 8. 总结

泛型与特征是Rust类型系统的核心：

1. **泛型**提供类型参数化，实现代码复用和类型安全
2. **特征**定义共同行为，支持多态和抽象
3. **特征约束**限制泛型类型，确保类型具有所需能力
4. **关联类型**简化特征定义，避免不必要的泛型参数
5. **特征对象**支持动态分发，实现运行时多态
6. **零成本抽象**通过编译时优化保证性能
7. **设计原则**包括小而专注的特征、合理使用泛型等

这些概念在嵌入式开发中特别重要，因为它们提供了高级抽象能力的同时保持了零运行时开销。

## 练习题

1. 设计一个通用的缓存系统，支持不同的存储后端和序列化方式
2. 实现一个类型安全的状态机，使用泛型和特征确保状态转换的正确性
3. 创建一个插件化的数据处理管道，支持动态加载和链式处理
4. 设计一个通用的配置管理系统，支持多种配置源和格式
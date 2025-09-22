// Rust高级练习题
// 涵盖泛型、特征、生命周期、智能指针、并发等高级概念
// 适合有一定Rust基础的学习者

#![allow(dead_code, unused_variables)]

use std::rc::Rc;
use std::cell::RefCell;
use std::sync::{Arc, Mutex};
use std::thread;
use std::collections::HashMap;

fn main() {
    println!("=== Rust 高级练习题 ===");
    println!("这些练习涵盖了Rust的高级特性");
    println!("使用 cargo test advanced_exercises 运行测试");
    println!();
}

// ============================================================================
// 练习1-5: 泛型和特征 (难度: 中高级)
// ============================================================================

/// 练习1: 泛型结构体
/// 
/// 要求:
/// 1. 定义一个泛型容器Container<T>
/// 2. 实现方法: new(), add(), get(), len()
/// 3. 容器应该能存储任何类型的值
#[derive(Debug)]
struct Container<T> {
    // TODO: 定义字段
    items: Vec<T>,
}

impl<T> Container<T> {
    // TODO: 实现new方法
    fn new() -> Self {
        Container { items: Vec::new() }
    }
    
    // TODO: 实现add方法
    fn add(&mut self, item: T) {
        self.items.push(item);
    }
    
    // TODO: 实现get方法
    fn get(&self, index: usize) -> Option<&T> {
        self.items.get(index)
    }
    
    // TODO: 实现len方法
    fn len(&self) -> usize {
        self.items.len()
    }
}

/// 练习2: 自定义特征
/// 
/// 要求:
/// 1. 定义一个Drawable特征，包含draw方法
/// 2. 为Circle和Rectangle实现这个特征
/// 3. 实现一个函数，可以绘制任何实现了Drawable的对象
trait Drawable {
    // TODO: 定义draw方法
    fn draw(&self) -> String;
}

#[derive(Debug)]
struct Circle {
    radius: f64,
}

#[derive(Debug)]
struct Rectangle {
    width: f64,
    height: f64,
}

impl Drawable for Circle {
    // TODO: 实现draw方法
    fn draw(&self) -> String {
        format!("Drawing a circle with radius {}", self.radius)
    }
}

impl Drawable for Rectangle {
    // TODO: 实现draw方法
    fn draw(&self) -> String {
        format!("Drawing a rectangle {}x{}", self.width, self.height)
    }
}

// TODO: 实现draw_shape函数
fn draw_shape(shape: &dyn Drawable) -> String {
    shape.draw()
}

/// 练习3: 特征约束
/// 
/// 要求:
/// 实现一个函数，找到向量中的最大值和最小值
/// 使用特征约束确保类型可以比较
fn find_min_max<T>(vec: &[T]) -> Option<(&T, &T)>
where
    T: PartialOrd,
{
    // TODO: 实现函数
    if vec.is_empty() {
        return None;
    }
    
    let mut min = &vec[0];
    let mut max = &vec[0];
    
    for item in vec.iter() {
        if item < min {
            min = item;
        }
        if item > max {
            max = item;
        }
    }
    
    Some((min, max))
}

/// 练习4: 关联类型
/// 
/// 要求:
/// 定义一个Iterator特征的简化版本，使用关联类型
trait SimpleIterator {
    type Item;
    
    fn next(&mut self) -> Option<Self::Item>;
}

struct Counter {
    current: usize,
    max: usize,
}

impl Counter {
    fn new(max: usize) -> Self {
        Counter { current: 0, max }
    }
}

impl SimpleIterator for Counter {
    type Item = usize;
    
    fn next(&mut self) -> Option<Self::Item> {
        // TODO: 实现next方法
        if self.current < self.max {
            let current = self.current;
            self.current += 1;
            Some(current)
        } else {
            None
        }
    }
}

/// 练习5: 默认泛型参数
/// 
/// 要求:
/// 定义一个Point结构体，默认使用f64类型，但也可以指定其他数值类型
#[derive(Debug, PartialEq)]
struct Point<T = f64> {
    x: T,
    y: T,
}

impl<T> Point<T> {
    fn new(x: T, y: T) -> Self {
        Point { x, y }
    }
}

impl<T> Point<T>
where
    T: std::ops::Add<Output = T> + std::ops::Mul<Output = T> + std::ops::Sub<Output = T> + Copy,
{
    fn distance_squared(&self, other: &Point<T>) -> T {
        // TODO: 计算两点间距离的平方
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        dx * dx + dy * dy
    }
}

// ============================================================================
// 练习6-10: 生命周期 (难度: 高级)
// ============================================================================

/// 练习6: 基本生命周期
/// 
/// 要求:
/// 实现一个函数，返回两个字符串切片中较长的那个
/// 正确标注生命周期参数
fn longest<'a>(s1: &'a str, s2: &'a str) -> &'a str {
    // TODO: 实现函数
    if s1.len() > s2.len() { s1 } else { s2 }
}

/// 练习7: 结构体生命周期
/// 
/// 要求:
/// 定义一个ImportantExcerpt结构体，包含一个字符串切片
/// 正确标注生命周期参数
#[derive(Debug)]
struct ImportantExcerpt<'a> {
    // TODO: 定义字段
    part: &'a str,
}

impl<'a> ImportantExcerpt<'a> {
    // TODO: 实现new方法
    fn new(text: &'a str) -> Self {
        ImportantExcerpt { part: text }
    }
    
    // TODO: 实现get_part方法
    fn get_part(&self) -> &str {
        self.part
    }
    
    // 生命周期省略规则示例
    fn announce_and_return_part(&self, announcement: &str) -> &str {
        println!("Attention please: {}", announcement);
        self.part
    }
}

/// 练习8: 复杂生命周期
/// 
/// 要求:
/// 实现一个函数，在文本中查找第一个单词
/// 返回单词和其在原文本中的位置
fn first_word_with_position(text: &str) -> Option<(&str, usize)> {
    // TODO: 实现函数
    let trimmed = text.trim_start();
    let start_pos = text.len() - trimmed.len();
    
    if let Some(end) = trimmed.find(char::is_whitespace) {
        Some((&trimmed[..end], start_pos))
    } else if !trimmed.is_empty() {
        Some((trimmed, start_pos))
    } else {
        None
    }
}

/// 练习9: 静态生命周期
/// 
/// 要求:
/// 实现一个函数，返回一个静态字符串或输入字符串中较长的那个
fn longest_with_static(s: &str) -> &str {
    const STATIC_STR: &'static str = "This is a static string";
    
    // TODO: 实现函数
    if s.len() > STATIC_STR.len() { s } else { STATIC_STR }
}

/// 练习10: 生命周期子类型
/// 
/// 要求:
/// 实现一个解析器结构体，可以解析字符串并返回解析结果
struct Parser<'a> {
    input: &'a str,
    position: usize,
}

impl<'a> Parser<'a> {
    fn new(input: &'a str) -> Self {
        Parser { input, position: 0 }
    }
    
    fn parse_word(&mut self) -> Option<&'a str> {
        // TODO: 实现解析单词的逻辑
        let remaining = &self.input[self.position..];
        let trimmed = remaining.trim_start();
        
        if trimmed.is_empty() {
            return None;
        }
        
        let start = self.position + (remaining.len() - trimmed.len());
        
        if let Some(end_offset) = trimmed.find(char::is_whitespace) {
            let end = start + end_offset;
            self.position = end;
            Some(&self.input[start..end])
        } else {
            self.position = self.input.len();
            Some(&self.input[start..])
        }
    }
}

// ============================================================================
// 练习11-15: 智能指针 (难度: 高级)
// ============================================================================

/// 练习11: Box智能指针
/// 
/// 要求:
/// 使用Box创建一个递归数据结构（二叉树）
#[derive(Debug)]
enum BinaryTree<T> {
    Empty,
    Node {
        value: T,
        left: Box<BinaryTree<T>>,
        right: Box<BinaryTree<T>>,
    },
}

impl<T> BinaryTree<T> {
    fn new() -> Self {
        BinaryTree::Empty
    }
    
    fn leaf(value: T) -> Self {
        BinaryTree::Node {
            value,
            left: Box::new(BinaryTree::Empty),
            right: Box::new(BinaryTree::Empty),
        }
    }
    
    fn node(value: T, left: BinaryTree<T>, right: BinaryTree<T>) -> Self {
        BinaryTree::Node {
            value,
            left: Box::new(left),
            right: Box::new(right),
        }
    }
}

impl<T> BinaryTree<T>
where
    T: PartialOrd,
{
    fn insert(&mut self, value: T) {
        // TODO: 实现插入逻辑
        match self {
            BinaryTree::Empty => {
                *self = BinaryTree::leaf(value);
            }
            BinaryTree::Node { value: node_value, left, right } => {
                if value <= *node_value {
                    left.insert(value);
                } else {
                    right.insert(value);
                }
            }
        }
    }
}

/// 练习12: Rc智能指针
/// 
/// 要求:
/// 使用Rc创建一个可以被多个所有者共享的数据结构
#[derive(Debug)]
struct Node {
    value: i32,
    children: Vec<Rc<Node>>,
}

impl Node {
    fn new(value: i32) -> Rc<Self> {
        Rc::new(Node {
            value,
            children: Vec::new(),
        })
    }
    
    // 注意：由于Rc是不可变的，我们需要使用RefCell来实现可变性
    // 这里简化处理，只提供创建节点的方法
}

/// 练习13: RefCell内部可变性
/// 
/// 要求:
/// 使用RefCell实现一个可以在不可变引用下修改的计数器
#[derive(Debug)]
struct RefCellCounter {
    value: RefCell<i32>,
}

impl RefCellCounter {
    fn new() -> Self {
        RefCellCounter {
            value: RefCell::new(0),
        }
    }
    
    fn increment(&self) {
        // TODO: 实现增加计数的逻辑
        *self.value.borrow_mut() += 1;
    }
    
    fn get(&self) -> i32 {
        // TODO: 实现获取计数的逻辑
        *self.value.borrow()
    }
}

/// 练习14: Rc<RefCell<T>>组合
/// 
/// 要求:
/// 创建一个可以被多个所有者共享且可修改的列表
type SharedList<T> = Rc<RefCell<Vec<T>>>;

fn create_shared_list<T>() -> SharedList<T> {
    // TODO: 创建共享列表
    Rc::new(RefCell::new(Vec::new()))
}

fn add_to_shared_list<T>(list: &SharedList<T>, item: T) {
    // TODO: 向共享列表添加元素
    list.borrow_mut().push(item);
}

fn get_shared_list_len<T>(list: &SharedList<T>) -> usize {
    // TODO: 获取共享列表长度
    list.borrow().len()
}

/// 练习15: 自定义智能指针
/// 
/// 要求:
/// 实现一个简单的智能指针，具有自动解引用功能
struct MyBox<T> {
    value: T,
}

impl<T> MyBox<T> {
    fn new(value: T) -> Self {
        MyBox { value }
    }
}

impl<T> std::ops::Deref for MyBox<T> {
    type Target = T;
    
    fn deref(&self) -> &Self::Target {
        // TODO: 实现解引用
        &self.value
    }
}

// ============================================================================
// 练习16-20: 并发编程 (难度: 高级)
// ============================================================================

/// 练习16: 基本线程
/// 
/// 要求:
/// 创建多个线程，每个线程计算一部分数据，最后汇总结果
fn parallel_sum(numbers: Vec<i32>, num_threads: usize) -> i32 {
    // TODO: 实现并行求和
    let chunk_size = (numbers.len() + num_threads - 1) / num_threads;
    let mut handles = vec![];
    
    for chunk in numbers.chunks(chunk_size) {
        let chunk = chunk.to_vec();
        let handle = thread::spawn(move || {
            chunk.iter().sum::<i32>()
        });
        handles.push(handle);
    }
    
    handles.into_iter()
        .map(|handle| handle.join().unwrap())
        .sum()
}

/// 练习17: Arc和Mutex
/// 
/// 要求:
/// 使用Arc和Mutex实现一个线程安全的计数器
#[derive(Debug)]
struct ThreadSafeCounter {
    value: Arc<Mutex<i32>>,
}

impl ThreadSafeCounter {
    fn new() -> Self {
        ThreadSafeCounter {
            value: Arc::new(Mutex::new(0)),
        }
    }
    
    fn increment(&self) {
        // TODO: 实现线程安全的增加操作
        let mut value = self.value.lock().unwrap();
        *value += 1;
    }
    
    fn get(&self) -> i32 {
        // TODO: 实现线程安全的获取操作
        *self.value.lock().unwrap()
    }
    
    fn clone(&self) -> Self {
        ThreadSafeCounter {
            value: Arc::clone(&self.value),
        }
    }
}

/// 练习18: 通道通信
/// 
/// 要求:
/// 使用通道在线程间传递消息
fn producer_consumer_example() -> Vec<i32> {
    use std::sync::mpsc;
    use std::time::Duration;
    
    let (tx, rx) = mpsc::channel();
    
    // 生产者线程
    let producer = thread::spawn(move || {
        for i in 1..=5 {
            tx.send(i).unwrap();
            thread::sleep(Duration::from_millis(100));
        }
    });
    
    // 消费者（主线程）
    let mut results = Vec::new();
    for received in rx {
        results.push(received);
    }
    
    producer.join().unwrap();
    results
}

/// 练习19: 多生产者单消费者
/// 
/// 要求:
/// 创建多个生产者线程，一个消费者线程
fn multiple_producers() -> Vec<String> {
    use std::sync::mpsc;
    
    let (tx, rx) = mpsc::channel();
    let mut handles = vec![];
    
    // 创建多个生产者
    for i in 0..3 {
        let tx_clone = tx.clone();
        let handle = thread::spawn(move || {
            for j in 0..3 {
                let message = format!("Producer {} - Message {}", i, j);
                tx_clone.send(message).unwrap();
            }
        });
        handles.push(handle);
    }
    
    // 关闭原始发送者
    drop(tx);
    
    // 等待所有生产者完成
    for handle in handles {
        handle.join().unwrap();
    }
    
    // 收集所有消息
    let mut messages = Vec::new();
    for received in rx {
        messages.push(received);
    }
    
    messages.sort(); // 排序以便测试
    messages
}

/// 练习20: 共享状态并发
/// 
/// 要求:
/// 实现一个线程安全的缓存系统
#[derive(Debug)]
struct ThreadSafeCache<K, V>
where
    K: Eq + std::hash::Hash + Clone,
    V: Clone,
{
    data: Arc<Mutex<HashMap<K, V>>>,
}

impl<K, V> ThreadSafeCache<K, V>
where
    K: Eq + std::hash::Hash + Clone,
    V: Clone,
{
    fn new() -> Self {
        ThreadSafeCache {
            data: Arc::new(Mutex::new(HashMap::new())),
        }
    }
    
    fn get(&self, key: &K) -> Option<V> {
        // TODO: 实现线程安全的获取操作
        let data = self.data.lock().unwrap();
        data.get(key).cloned()
    }
    
    fn insert(&self, key: K, value: V) {
        // TODO: 实现线程安全的插入操作
        let mut data = self.data.lock().unwrap();
        data.insert(key, value);
    }
    
    fn clone(&self) -> Self {
        ThreadSafeCache {
            data: Arc::clone(&self.data),
        }
    }
}

// ============================================================================
// 测试用例
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_container() {
        let mut container = Container::new();
        container.add(1);
        container.add(2);
        container.add(3);
        
        assert_eq!(container.len(), 3);
        assert_eq!(container.get(1), Some(&2));
        assert_eq!(container.get(5), None);
    }
    
    #[test]
    fn test_drawable() {
        let circle = Circle { radius: 5.0 };
        let rectangle = Rectangle { width: 10.0, height: 20.0 };
        
        assert!(draw_shape(&circle).contains("circle"));
        assert!(draw_shape(&rectangle).contains("rectangle"));
    }
    
    #[test]
    fn test_find_min_max() {
        let numbers = vec![3, 1, 4, 1, 5, 9, 2, 6];
        let (min, max) = find_min_max(&numbers).unwrap();
        assert_eq!(*min, 1);
        assert_eq!(*max, 9);
        
        let empty: Vec<i32> = vec![];
        assert_eq!(find_min_max(&empty), None);
    }
    
    #[test]
    fn test_counter() {
        let mut counter = Counter::new(3);
        assert_eq!(counter.next(), Some(0));
        assert_eq!(counter.next(), Some(1));
        assert_eq!(counter.next(), Some(2));
        assert_eq!(counter.next(), None);
    }
    
    #[test]
    fn test_point() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(3.0, 4.0);
        
        // 距离的平方应该是25.0
        assert_eq!(p1.distance_squared(&p2), 25.0);
        
        // 测试默认类型
        let p3: Point = Point::new(1.0, 2.0);
        assert_eq!(p3.x, 1.0);
    }
    
    #[test]
    fn test_longest() {
        assert_eq!(longest("short", "longer"), "longer");
        assert_eq!(longest("same", "size"), "same");
    }
    
    #[test]
    fn test_important_excerpt() {
        let text = "This is important text";
        let excerpt = ImportantExcerpt::new(text);
        assert_eq!(excerpt.get_part(), text);
    }
    
    #[test]
    fn test_first_word_with_position() {
        assert_eq!(first_word_with_position("hello world"), Some(("hello", 0)));
        assert_eq!(first_word_with_position("  hello world"), Some(("hello", 2)));
        assert_eq!(first_word_with_position(""), None);
        assert_eq!(first_word_with_position("single"), Some(("single", 0)));
    }
    
    #[test]
    fn test_parser() {
        let mut parser = Parser::new("hello world rust");
        assert_eq!(parser.parse_word(), Some("hello"));
        assert_eq!(parser.parse_word(), Some("world"));
        assert_eq!(parser.parse_word(), Some("rust"));
        assert_eq!(parser.parse_word(), None);
    }
    
    #[test]
    fn test_binary_tree() {
        let mut tree = BinaryTree::new();
        tree.insert(5);
        tree.insert(3);
        tree.insert(7);
        
        // 简单测试树的结构
        match tree {
            BinaryTree::Node { value, .. } => assert_eq!(value, 5),
            _ => panic!("Expected node"),
        }
    }
    
    #[test]
    fn test_counter_refcell() {
        let counter = RefCellCounter::new();
        assert_eq!(counter.get(), 0);
        
        counter.increment();
        counter.increment();
        assert_eq!(counter.get(), 2);
    }
    
    #[test]
    fn test_shared_list() {
        let list = create_shared_list();
        add_to_shared_list(&list, 1);
        add_to_shared_list(&list, 2);
        
        assert_eq!(get_shared_list_len(&list), 2);
    }
    
    #[test]
    fn test_my_box() {
        let boxed = MyBox::new(5);
        assert_eq!(*boxed, 5);
    }
    
    #[test]
    fn test_parallel_sum() {
        let numbers = (1..=100).collect();
        let result = parallel_sum(numbers, 4);
        assert_eq!(result, 5050); // 1+2+...+100 = 5050
    }
    
    #[test]
    fn test_thread_safe_counter() {
        let counter = ThreadSafeCounter::new();
        let counter_clone = counter.clone();
        
        let handle = thread::spawn(move || {
            for _ in 0..10 {
                counter_clone.increment();
            }
        });
        
        for _ in 0..10 {
            counter.increment();
        }
        
        handle.join().unwrap();
        assert_eq!(counter.get(), 20);
    }
    
    #[test]
    fn test_producer_consumer() {
        let results = producer_consumer_example();
        assert_eq!(results, vec![1, 2, 3, 4, 5]);
    }
    
    #[test]
    fn test_multiple_producers() {
        let messages = multiple_producers();
        assert_eq!(messages.len(), 9); // 3 producers * 3 messages each
        
        // 检查是否包含预期的消息
        assert!(messages.iter().any(|msg| msg.contains("Producer 0")));
        assert!(messages.iter().any(|msg| msg.contains("Producer 1")));
        assert!(messages.iter().any(|msg| msg.contains("Producer 2")));
    }
    
    #[test]
    fn test_thread_safe_cache() {
        let cache = ThreadSafeCache::new();
        cache.insert("key1", "value1");
        cache.insert("key2", "value2");
        
        assert_eq!(cache.get(&"key1"), Some("value1"));
        assert_eq!(cache.get(&"key3"), None);
        
        // 测试克隆
        let cache_clone = cache.clone();
        assert_eq!(cache_clone.get(&"key2"), Some("value2"));
    }
}

// ============================================================================
// 超级挑战练习 (难度: 专家级)
// ============================================================================

/// 超级挑战1: 实现一个简单的异步执行器
/// 
/// 要求:
/// 使用Future trait实现一个简单的异步任务执行器
/// 注意：这是一个高级练习，需要深入理解Rust的异步编程
use std::future::Future;
use std::pin::Pin;
use std::task::{Context, Poll, Waker};

struct SimpleFuture {
    completed: bool,
}

impl SimpleFuture {
    fn new() -> Self {
        SimpleFuture { completed: false }
    }
}

impl Future for SimpleFuture {
    type Output = String;
    
    fn poll(mut self: Pin<&mut Self>, _cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.completed {
            Poll::Ready("Future completed!".to_string())
        } else {
            self.completed = true;
            Poll::Pending
        }
    }
}

/// 超级挑战2: 实现一个类型安全的状态机
/// 
/// 要求:
/// 使用Rust的类型系统实现一个编译时检查的状态机
mod state_machine {
    pub struct Locked;
    pub struct Unlocked;
    
    pub struct Door<State> {
        state: std::marker::PhantomData<State>,
    }
    
    impl Door<Locked> {
        pub fn new() -> Self {
            Door { state: std::marker::PhantomData }
        }
        
        pub fn unlock(self) -> Door<Unlocked> {
            Door { state: std::marker::PhantomData }
        }
    }
    
    impl Door<Unlocked> {
        pub fn lock(self) -> Door<Locked> {
            Door { state: std::marker::PhantomData }
        }
        
        pub fn open(&self) -> &str {
            "Door is open!"
        }
    }
}

#[cfg(test)]
mod super_challenge_tests {
    use super::*;
    use super::state_machine::*;
    
    #[test]
    fn test_state_machine() {
        let door = Door::new(); // Locked state
        let door = door.unlock(); // Unlocked state
        assert_eq!(door.open(), "Door is open!");
        let _door = door.lock(); // Back to locked state
    }
}
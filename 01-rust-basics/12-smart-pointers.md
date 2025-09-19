# 第12章：智能指针

## 概述

智能指针是Rust中一类特殊的数据结构，它们不仅包含数据的地址，还包含额外的元数据和功能。与普通引用不同，智能指针通常拥有它们指向的数据。Rust标准库提供了多种智能指针，每种都有其特定的用途和特性。

## 学习目标

通过本章学习，你将掌握：
- `Box<T>`：堆分配和递归类型
- `Rc<T>`和`Arc<T>`：引用计数智能指针
- `RefCell<T>`和`Mutex<T>`：内部可变性
- `Weak<T>`：弱引用和循环引用处理
- 自定义智能指针的实现
- 智能指针的性能考虑和最佳实践

## 1. Box<T> - 堆分配

### 1.1 基本使用

```rust
fn main() {
    // 在堆上分配一个整数
    let b = Box::new(5);
    println!("b = {}", b);
    
    // Box实现了Deref trait，可以像引用一样使用
    let x = 5;
    let y = Box::new(x);
    
    assert_eq!(5, x);
    assert_eq!(5, *y);
    
    // 当Box离开作用域时，堆内存会被自动释放
}
```

### 1.2 递归类型

```rust
// 使用Box实现链表
#[derive(Debug)]
enum List {
    Cons(i32, Box<List>),
    Nil,
}

use List::{Cons, Nil};

impl List {
    fn new() -> List {
        Nil
    }
    
    fn prepend(self, elem: i32) -> List {
        Cons(elem, Box::new(self))
    }
    
    fn len(&self) -> usize {
        match self {
            Cons(_, tail) => 1 + tail.len(),
            Nil => 0,
        }
    }
    
    fn stringify(&self) -> String {
        match self {
            Cons(head, tail) => {
                format!("{}, {}", head, tail.stringify())
            }
            Nil => {
                format!("Nil")
            }
        }
    }
}

fn main() {
    let mut list = List::new();
    
    list = list.prepend(1);
    list = list.prepend(2);
    list = list.prepend(3);
    
    println!("linked list has length: {}", list.len());
    println!("{}", list.stringify());
}
```

### 1.3 大型数据结构

```rust
// 避免栈溢出的大型结构
struct LargeStruct {
    data: [u8; 1_000_000], // 1MB的数据
}

fn main() {
    // 直接在栈上创建可能导致栈溢出
    // let large = LargeStruct { data: [0; 1_000_000] };
    
    // 使用Box在堆上分配
    let large = Box::new(LargeStruct { data: [0; 1_000_000] });
    
    println!("Large struct created on heap");
    
    // 传递Box而不是移动整个结构
    process_large_struct(large);
}

fn process_large_struct(data: Box<LargeStruct>) {
    println!("Processing large struct of size: {}", data.data.len());
}
```

### 1.4 Box的高级用法

```rust
use std::fmt::Display;

// 特征对象
fn print_it(item: Box<dyn Display>) {
    println!("{}", item);
}

// 自定义智能指针行为
struct MyBox<T>(T);

impl<T> MyBox<T> {
    fn new(x: T) -> MyBox<T> {
        MyBox(x)
    }
}

use std::ops::Deref;

impl<T> Deref for MyBox<T> {
    type Target = T;
    
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> Drop for MyBox<T> {
    fn drop(&mut self) {
        println!("Dropping MyBox");
    }
}

fn main() {
    // 特征对象的使用
    let string = Box::new(String::from("Hello"));
    let number = Box::new(42);
    
    print_it(string);
    print_it(number);
    
    // 自定义智能指针
    let x = 5;
    let y = MyBox::new(x);
    
    assert_eq!(5, x);
    assert_eq!(5, *y); // 自动解引用
    
    // 强制解引用转换
    let m = MyBox::new(String::from("Rust"));
    hello(&m); // &MyBox<String> -> &String -> &str
}

fn hello(name: &str) {
    println!("Hello, {}!", name);
}
```

## 2. Rc<T> - 引用计数

### 2.1 基本使用

```rust
use std::rc::Rc;

fn main() {
    let a = Rc::new(5);
    println!("Reference count of a: {}", Rc::strong_count(&a));
    
    let b = Rc::clone(&a);
    println!("Reference count after creating b: {}", Rc::strong_count(&a));
    
    {
        let c = Rc::clone(&a);
        println!("Reference count after creating c: {}", Rc::strong_count(&a));
    }
    
    println!("Reference count after c goes out of scope: {}", Rc::strong_count(&a));
}
```

### 2.2 共享数据结构

```rust
use std::rc::Rc;

#[derive(Debug)]
enum List {
    Cons(i32, Rc<List>),
    Nil,
}

use List::{Cons, Nil};

fn main() {
    let a = Rc::new(Cons(5, Rc::new(Cons(10, Rc::new(Nil)))));
    println!("count after creating a = {}", Rc::strong_count(&a));
    
    let b = Cons(3, Rc::clone(&a));
    println!("count after creating b = {}", Rc::strong_count(&a));
    
    {
        let c = Cons(4, Rc::clone(&a));
        println!("count after creating c = {}", Rc::strong_count(&a));
    }
    
    println!("count after c goes out of scope = {}", Rc::strong_count(&a));
}
```

### 2.3 树形数据结构

```rust
use std::rc::Rc;
use std::cell::RefCell;

#[derive(Debug)]
struct Node {
    value: i32,
    children: RefCell<Vec<Rc<Node>>>,
}

impl Node {
    fn new(value: i32) -> Rc<Node> {
        Rc::new(Node {
            value,
            children: RefCell::new(vec![]),
        })
    }
    
    fn add_child(self: &Rc<Self>, child: Rc<Node>) {
        self.children.borrow_mut().push(child);
    }
}

fn main() {
    let root = Node::new(1);
    let child1 = Node::new(2);
    let child2 = Node::new(3);
    let grandchild = Node::new(4);
    
    root.add_child(Rc::clone(&child1));
    root.add_child(Rc::clone(&child2));
    child1.add_child(grandchild);
    
    println!("Root: {:?}", root);
    println!("Child1 ref count: {}", Rc::strong_count(&child1));
    println!("Child2 ref count: {}", Rc::strong_count(&child2));
}
```

## 3. Arc<T> - 原子引用计数

### 3.1 多线程共享

```rust
use std::sync::Arc;
use std::thread;

fn main() {
    let data = Arc::new(vec![1, 2, 3, 4, 5]);
    let mut handles = vec![];
    
    for i in 0..3 {
        let data = Arc::clone(&data);
        let handle = thread::spawn(move || {
            println!("Thread {}: {:?}", i, data);
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Main thread: {:?}", data);
}
```

### 3.2 Arc与Mutex结合

```rust
use std::sync::{Arc, Mutex};
use std::thread;

fn main() {
    let counter = Arc::new(Mutex::new(0));
    let mut handles = vec![];
    
    for _ in 0..10 {
        let counter = Arc::clone(&counter);
        let handle = thread::spawn(move || {
            let mut num = counter.lock().unwrap();
            *num += 1;
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Result: {}", *counter.lock().unwrap());
}
```

### 3.3 Arc的性能考虑

```rust
use std::sync::Arc;
use std::thread;
use std::time::Instant;

fn compare_arc_performance() {
    const NUM_THREADS: usize = 8;
    const NUM_CLONES: usize = 1_000_000;
    
    // 测试Arc克隆性能
    let data = Arc::new(vec![1, 2, 3, 4, 5]);
    let start = Instant::now();
    
    let mut handles = vec![];
    for _ in 0..NUM_THREADS {
        let data = Arc::clone(&data);
        let handle = thread::spawn(move || {
            for _ in 0..NUM_CLONES / NUM_THREADS {
                let _clone = Arc::clone(&data);
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Arc cloning took: {:?}", start.elapsed());
    println!("Final reference count: {}", Arc::strong_count(&data));
}

fn main() {
    compare_arc_performance();
}
```

## 4. RefCell<T> - 内部可变性

### 4.1 基本使用

```rust
use std::cell::RefCell;

fn main() {
    let data = RefCell::new(5);
    
    // 不可变借用
    let borrowed = data.borrow();
    println!("Value: {}", *borrowed);
    drop(borrowed); // 显式释放借用
    
    // 可变借用
    let mut borrowed_mut = data.borrow_mut();
    *borrowed_mut += 10;
    drop(borrowed_mut);
    
    println!("Modified value: {}", *data.borrow());
}
```

### 4.2 运行时借用检查

```rust
use std::cell::RefCell;

fn main() {
    let data = RefCell::new(5);
    
    // 这会在运行时panic
    let _borrow1 = data.borrow();
    // let _borrow2 = data.borrow_mut(); // 这行会panic
    
    // 安全的借用检查
    match data.try_borrow_mut() {
        Ok(mut borrowed) => {
            *borrowed += 1;
            println!("Successfully borrowed and modified: {}", *borrowed);
        }
        Err(_) => {
            println!("Could not borrow mutably");
        }
    }
}
```

### 4.3 Mock对象模式

```rust
use std::cell::RefCell;

pub trait Messenger {
    fn send(&self, msg: &str);
}

pub struct LimitTracker<'a, T: Messenger> {
    messenger: &'a T,
    value: usize,
    max: usize,
}

impl<'a, T> LimitTracker<'a, T>
where
    T: Messenger,
{
    pub fn new(messenger: &'a T, max: usize) -> LimitTracker<'a, T> {
        LimitTracker {
            messenger,
            value: 0,
            max,
        }
    }
    
    pub fn set_value(&mut self, value: usize) {
        self.value = value;
        
        let percentage_of_max = self.value as f64 / self.max as f64;
        
        if percentage_of_max >= 1.0 {
            self.messenger.send("Error: You are over your quota!");
        } else if percentage_of_max >= 0.9 {
            self.messenger.send("Urgent warning: You've used up over 90% of your quota!");
        } else if percentage_of_max >= 0.75 {
            self.messenger.send("Warning: You've used up over 75% of your quota");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    
    struct MockMessenger {
        sent_messages: RefCell<Vec<String>>,
    }
    
    impl MockMessenger {
        fn new() -> MockMessenger {
            MockMessenger {
                sent_messages: RefCell::new(vec![]),
            }
        }
    }
    
    impl Messenger for MockMessenger {
        fn send(&self, message: &str) {
            self.sent_messages.borrow_mut().push(String::from(message));
        }
    }
    
    #[test]
    fn it_sends_an_over_75_percent_warning_message() {
        let mock_messenger = MockMessenger::new();
        let mut limit_tracker = LimitTracker::new(&mock_messenger, 100);
        
        limit_tracker.set_value(80);
        
        assert_eq!(mock_messenger.sent_messages.borrow().len(), 1);
    }
}

fn main() {
    struct ConsoleMessenger;
    
    impl Messenger for ConsoleMessenger {
        fn send(&self, msg: &str) {
            println!("Message: {}", msg);
        }
    }
    
    let messenger = ConsoleMessenger;
    let mut tracker = LimitTracker::new(&messenger, 100);
    
    tracker.set_value(50);
    tracker.set_value(80);
    tracker.set_value(95);
    tracker.set_value(105);
}
```

## 5. Rc<RefCell<T>> 组合模式

### 5.1 可变共享数据

```rust
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug)]
enum List {
    Cons(Rc<RefCell<i32>>, Rc<List>),
    Nil,
}

use List::{Cons, Nil};

fn main() {
    let value = Rc::new(RefCell::new(5));
    
    let a = Rc::new(Cons(Rc::clone(&value), Rc::new(Nil)));
    let b = Cons(Rc::new(RefCell::new(3)), Rc::clone(&a));
    let c = Cons(Rc::new(RefCell::new(4)), Rc::clone(&a));
    
    *value.borrow_mut() += 10;
    
    println!("a after = {:?}", a);
    println!("b after = {:?}", b);
    println!("c after = {:?}", c);
}
```

### 5.2 图数据结构

```rust
use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[derive(Debug)]
struct Node {
    value: i32,
    parent: RefCell<Weak<Node>>,
    children: RefCell<Vec<Rc<Node>>>,
}

impl Node {
    fn new(value: i32) -> Rc<Node> {
        Rc::new(Node {
            value,
            parent: RefCell::new(Weak::new()),
            children: RefCell::new(vec![]),
        })
    }
    
    fn add_child(parent: &Rc<Node>, child: Rc<Node>) {
        child.parent.borrow_mut().replace(Rc::downgrade(parent));
        parent.children.borrow_mut().push(child);
    }
}

fn main() {
    let leaf = Node::new(3);
    
    println!(
        "leaf strong = {}, weak = {}",
        Rc::strong_count(&leaf),
        Rc::weak_count(&leaf),
    );
    
    {
        let branch = Node::new(5);
        Node::add_child(&branch, Rc::clone(&leaf));
        
        println!(
            "branch strong = {}, weak = {}",
            Rc::strong_count(&branch),
            Rc::weak_count(&branch),
        );
        
        println!(
            "leaf strong = {}, weak = {}",
            Rc::strong_count(&leaf),
            Rc::weak_count(&leaf),
        );
        
        println!("leaf parent = {:?}", leaf.parent.borrow().upgrade());
    }
    
    println!("leaf parent = {:?}", leaf.parent.borrow().upgrade());
    println!(
        "leaf strong = {}, weak = {}",
        Rc::strong_count(&leaf),
        Rc::weak_count(&leaf),
    );
}
```

## 6. 弱引用 Weak<T>

### 6.1 避免循环引用

```rust
use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[derive(Debug)]
struct Parent {
    children: RefCell<Vec<Rc<Child>>>,
}

#[derive(Debug)]
struct Child {
    parent: RefCell<Weak<Parent>>,
    name: String,
}

fn main() {
    let parent = Rc::new(Parent {
        children: RefCell::new(vec![]),
    });
    
    let child1 = Rc::new(Child {
        parent: RefCell::new(Rc::downgrade(&parent)),
        name: "Child 1".to_string(),
    });
    
    let child2 = Rc::new(Child {
        parent: RefCell::new(Rc::downgrade(&parent)),
        name: "Child 2".to_string(),
    });
    
    parent.children.borrow_mut().push(Rc::clone(&child1));
    parent.children.borrow_mut().push(Rc::clone(&child2));
    
    // 访问父节点
    if let Some(parent_ref) = child1.parent.borrow().upgrade() {
        println!("Child1's parent has {} children", parent_ref.children.borrow().len());
    }
    
    println!("Parent strong count: {}", Rc::strong_count(&parent));
    println!("Child1 strong count: {}", Rc::strong_count(&child1));
}
```

### 6.2 缓存系统

```rust
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::{Rc, Weak};

struct Cache<K, V> {
    data: RefCell<HashMap<K, Weak<V>>>,
}

impl<K, V> Cache<K, V>
where
    K: std::hash::Hash + Eq + Clone,
{
    fn new() -> Self {
        Cache {
            data: RefCell::new(HashMap::new()),
        }
    }
    
    fn get(&self, key: &K) -> Option<Rc<V>> {
        let mut data = self.data.borrow_mut();
        
        // 清理已经被释放的弱引用
        if let Some(weak_ref) = data.get(key) {
            if let Some(strong_ref) = weak_ref.upgrade() {
                return Some(strong_ref);
            } else {
                data.remove(key);
            }
        }
        
        None
    }
    
    fn insert(&self, key: K, value: Rc<V>) {
        let weak_ref = Rc::downgrade(&value);
        self.data.borrow_mut().insert(key, weak_ref);
    }
    
    fn cleanup(&self) {
        let mut data = self.data.borrow_mut();
        data.retain(|_, weak_ref| weak_ref.strong_count() > 0);
    }
}

#[derive(Debug)]
struct ExpensiveResource {
    id: u32,
    data: Vec<u8>,
}

impl ExpensiveResource {
    fn new(id: u32) -> Self {
        println!("Creating expensive resource {}", id);
        ExpensiveResource {
            id,
            data: vec![0; 1000], // 模拟大量数据
        }
    }
}

fn main() {
    let cache: Cache<u32, ExpensiveResource> = Cache::new();
    
    // 第一次访问，创建资源
    let resource1 = {
        if let Some(cached) = cache.get(&1) {
            cached
        } else {
            let new_resource = Rc::new(ExpensiveResource::new(1));
            cache.insert(1, Rc::clone(&new_resource));
            new_resource
        }
    };
    
    println!("Resource 1: {:?}", resource1.id);
    
    // 第二次访问，从缓存获取
    let resource1_again = cache.get(&1).unwrap();
    println!("Resource 1 again: {:?}", resource1_again.id);
    
    // 释放强引用
    drop(resource1);
    drop(resource1_again);
    
    // 现在缓存中的弱引用应该无效了
    assert!(cache.get(&1).is_none());
    
    cache.cleanup();
    println!("Cache cleaned up");
}
```

## 7. 自定义智能指针

### 7.1 实现Deref trait

```rust
use std::ops::Deref;

struct MyBox<T>(T);

impl<T> MyBox<T> {
    fn new(x: T) -> MyBox<T> {
        MyBox(x)
    }
}

impl<T> Deref for MyBox<T> {
    type Target = T;
    
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

// 可变解引用
use std::ops::DerefMut;

impl<T> DerefMut for MyBox<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

fn main() {
    let x = 5;
    let y = MyBox::new(x);
    
    assert_eq!(5, x);
    assert_eq!(5, *y);
    
    let mut z = MyBox::new(String::from("Hello"));
    z.push_str(", world!");
    println!("{}", *z);
}
```

### 7.2 实现Drop trait

```rust
struct CustomSmartPointer {
    data: String,
}

impl Drop for CustomSmartPointer {
    fn drop(&mut self) {
        println!("Dropping CustomSmartPointer with data `{}`!", self.data);
    }
}

fn main() {
    let c = CustomSmartPointer {
        data: String::from("my stuff"),
    };
    
    let d = CustomSmartPointer {
        data: String::from("other stuff"),
    };
    
    println!("CustomSmartPointers created.");
    
    // 手动释放
    drop(c);
    println!("CustomSmartPointer dropped before the end of main.");
}
```

### 7.3 引用计数智能指针

```rust
use std::cell::RefCell;
use std::ops::Deref;

struct MyRc<T> {
    data: *const RefCell<RcInner<T>>,
}

struct RcInner<T> {
    value: T,
    ref_count: usize,
}

impl<T> MyRc<T> {
    fn new(value: T) -> Self {
        let inner = Box::new(RefCell::new(RcInner {
            value,
            ref_count: 1,
        }));
        
        MyRc {
            data: Box::into_raw(inner),
        }
    }
    
    fn clone(&self) -> Self {
        unsafe {
            (*self.data).borrow_mut().ref_count += 1;
        }
        
        MyRc {
            data: self.data,
        }
    }
    
    fn strong_count(&self) -> usize {
        unsafe {
            (*self.data).borrow().ref_count
        }
    }
}

impl<T> Deref for MyRc<T> {
    type Target = T;
    
    fn deref(&self) -> &Self::Target {
        unsafe {
            &(*self.data).borrow().value
        }
    }
}

impl<T> Drop for MyRc<T> {
    fn drop(&mut self) {
        unsafe {
            let mut inner = (*self.data).borrow_mut();
            inner.ref_count -= 1;
            
            if inner.ref_count == 0 {
                drop(inner);
                Box::from_raw(self.data as *mut RefCell<RcInner<T>>);
            }
        }
    }
}

// 注意：这个实现不是线程安全的，仅用于演示
unsafe impl<T> Send for MyRc<T> where T: Send {}
unsafe impl<T> Sync for MyRc<T> where T: Sync {}

fn main() {
    let a = MyRc::new(5);
    println!("Reference count: {}", a.strong_count());
    
    let b = a.clone();
    println!("Reference count after clone: {}", a.strong_count());
    
    {
        let c = a.clone();
        println!("Reference count in inner scope: {}", a.strong_count());
    }
    
    println!("Reference count after inner scope: {}", a.strong_count());
    println!("Value: {}", *a);
}
```

## 8. 性能考虑和最佳实践

### 8.1 智能指针性能比较

```rust
use std::rc::Rc;
use std::sync::Arc;
use std::time::Instant;

fn benchmark_smart_pointers() {
    const ITERATIONS: usize = 1_000_000;
    
    // Box性能测试
    let start = Instant::now();
    for _ in 0..ITERATIONS {
        let _b = Box::new(42);
    }
    println!("Box allocation: {:?}", start.elapsed());
    
    // Rc性能测试
    let start = Instant::now();
    let rc = Rc::new(42);
    for _ in 0..ITERATIONS {
        let _clone = Rc::clone(&rc);
    }
    println!("Rc cloning: {:?}", start.elapsed());
    
    // Arc性能测试
    let start = Instant::now();
    let arc = Arc::new(42);
    for _ in 0..ITERATIONS {
        let _clone = Arc::clone(&arc);
    }
    println!("Arc cloning: {:?}", start.elapsed());
}

fn main() {
    benchmark_smart_pointers();
}
```

### 8.2 内存使用优化

```rust
use std::rc::Rc;
use std::mem;

#[derive(Debug)]
struct LargeData {
    data: [u8; 1024],
}

#[derive(Debug)]
struct OptimizedNode {
    // 使用Rc共享大数据
    shared_data: Rc<LargeData>,
    // 节点特有的小数据
    node_id: u32,
}

impl OptimizedNode {
    fn new(shared_data: Rc<LargeData>, node_id: u32) -> Self {
        OptimizedNode {
            shared_data,
            node_id,
        }
    }
}

fn main() {
    println!("Size of LargeData: {}", mem::size_of::<LargeData>());
    println!("Size of Rc<LargeData>: {}", mem::size_of::<Rc<LargeData>>());
    println!("Size of OptimizedNode: {}", mem::size_of::<OptimizedNode>());
    
    // 创建共享数据
    let shared = Rc::new(LargeData { data: [0; 1024] });
    
    // 创建多个节点共享同一份数据
    let nodes: Vec<OptimizedNode> = (0..10)
        .map(|i| OptimizedNode::new(Rc::clone(&shared), i))
        .collect();
    
    println!("Created {} nodes sharing data", nodes.len());
    println!("Reference count: {}", Rc::strong_count(&shared));
}
```

### 8.3 避免循环引用

```rust
use std::cell::RefCell;
use std::rc::{Rc, Weak};

// 错误的设计 - 会导致循环引用
#[derive(Debug)]
struct BadParent {
    children: RefCell<Vec<Rc<BadChild>>>,
}

#[derive(Debug)]
struct BadChild {
    parent: RefCell<Option<Rc<BadParent>>>, // 强引用导致循环
}

// 正确的设计 - 使用弱引用
#[derive(Debug)]
struct GoodParent {
    children: RefCell<Vec<Rc<GoodChild>>>,
}

#[derive(Debug)]
struct GoodChild {
    parent: RefCell<Weak<GoodParent>>, // 弱引用避免循环
    name: String,
}

impl GoodParent {
    fn new() -> Rc<Self> {
        Rc::new(GoodParent {
            children: RefCell::new(vec![]),
        })
    }
    
    fn add_child(self: &Rc<Self>, name: String) -> Rc<GoodChild> {
        let child = Rc::new(GoodChild {
            parent: RefCell::new(Rc::downgrade(self)),
            name,
        });
        
        self.children.borrow_mut().push(Rc::clone(&child));
        child
    }
}

impl GoodChild {
    fn get_parent(&self) -> Option<Rc<GoodParent>> {
        self.parent.borrow().upgrade()
    }
}

fn main() {
    let parent = GoodParent::new();
    let child1 = parent.add_child("Alice".to_string());
    let child2 = parent.add_child("Bob".to_string());
    
    println!("Parent has {} children", parent.children.borrow().len());
    
    if let Some(parent_ref) = child1.get_parent() {
        println!("Child1's parent has {} children", parent_ref.children.borrow().len());
    }
    
    println!("Parent strong count: {}", Rc::strong_count(&parent));
    println!("Child1 strong count: {}", Rc::strong_count(&child1));
    
    // 当parent和children离开作用域时，内存会被正确释放
}
```

## 9. 实践示例

### 9.1 实现一个简单的DOM树

```rust
use std::cell::RefCell;
use std::rc::{Rc, Weak};

#[derive(Debug)]
struct Element {
    tag: String,
    attributes: RefCell<std::collections::HashMap<String, String>>,
    children: RefCell<Vec<Rc<Element>>>,
    parent: RefCell<Weak<Element>>,
}

impl Element {
    fn new(tag: &str) -> Rc<Self> {
        Rc::new(Element {
            tag: tag.to_string(),
            attributes: RefCell::new(std::collections::HashMap::new()),
            children: RefCell::new(vec![]),
            parent: RefCell::new(Weak::new()),
        })
    }
    
    fn set_attribute(&self, name: &str, value: &str) {
        self.attributes.borrow_mut().insert(name.to_string(), value.to_string());
    }
    
    fn get_attribute(&self, name: &str) -> Option<String> {
        self.attributes.borrow().get(name).cloned()
    }
    
    fn append_child(self: &Rc<Self>, child: Rc<Element>) {
        child.parent.borrow_mut().replace(Rc::downgrade(self));
        self.children.borrow_mut().push(child);
    }
    
    fn remove_child(&self, child: &Rc<Element>) {
        self.children.borrow_mut().retain(|c| !Rc::ptr_eq(c, child));
        child.parent.borrow_mut().take();
    }
    
    fn get_parent(&self) -> Option<Rc<Element>> {
        self.parent.borrow().upgrade()
    }
    
    fn find_by_tag(&self, tag: &str) -> Vec<Rc<Element>> {
        let mut result = vec![];
        
        if self.tag == tag {
            // 注意：这里我们不能返回self的Rc，因为我们只有&self
            // 在实际实现中，可能需要不同的设计
        }
        
        for child in self.children.borrow().iter() {
            result.extend(child.find_by_tag(tag));
        }
        
        result
    }
    
    fn to_html(&self) -> String {
        let mut html = format!("<{}", self.tag);
        
        for (key, value) in self.attributes.borrow().iter() {
            html.push_str(&format!(" {}=\"{}\"", key, value));
        }
        
        if self.children.borrow().is_empty() {
            html.push_str(" />");
        } else {
            html.push('>');
            
            for child in self.children.borrow().iter() {
                html.push_str(&child.to_html());
            }
            
            html.push_str(&format!("</{}>", self.tag));
        }
        
        html
    }
}

fn main() {
    // 创建DOM树
    let html = Element::new("html");
    let head = Element::new("head");
    let body = Element::new("body");
    let title = Element::new("title");
    let div = Element::new("div");
    let p = Element::new("p");
    
    // 设置属性
    div.set_attribute("class", "container");
    div.set_attribute("id", "main");
    p.set_attribute("style", "color: blue;");
    
    // 构建树结构
    html.append_child(Rc::clone(&head));
    html.append_child(Rc::clone(&body));
    head.append_child(title);
    body.append_child(Rc::clone(&div));
    div.append_child(p);
    
    // 输出HTML
    println!("{}", html.to_html());
    
    // 验证父子关系
    if let Some(parent) = div.get_parent() {
        println!("Div's parent tag: {}", parent.tag);
    }
    
    println!("HTML children count: {}", html.children.borrow().len());
    println!("Body children count: {}", body.children.borrow().len());
}
```

### 9.2 实现一个缓存系统

```rust
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::{Rc, Weak};
use std::time::{Duration, Instant};

struct CacheEntry<V> {
    value: V,
    created_at: Instant,
    ttl: Duration,
}

impl<V> CacheEntry<V> {
    fn new(value: V, ttl: Duration) -> Self {
        CacheEntry {
            value,
            created_at: Instant::now(),
            ttl,
        }
    }
    
    fn is_expired(&self) -> bool {
        self.created_at.elapsed() > self.ttl
    }
}

struct Cache<K, V> {
    data: RefCell<HashMap<K, Weak<CacheEntry<V>>>>,
    default_ttl: Duration,
}

impl<K, V> Cache<K, V>
where
    K: std::hash::Hash + Eq + Clone,
{
    fn new(default_ttl: Duration) -> Self {
        Cache {
            data: RefCell::new(HashMap::new()),
            default_ttl,
        }
    }
    
    fn get(&self, key: &K) -> Option<Rc<CacheEntry<V>>> {
        let mut data = self.data.borrow_mut();
        
        if let Some(weak_entry) = data.get(key) {
            if let Some(entry) = weak_entry.upgrade() {
                if !entry.is_expired() {
                    return Some(entry);
                }
            }
            // 移除过期或已释放的条目
            data.remove(key);
        }
        
        None
    }
    
    fn insert(&self, key: K, value: V) -> Rc<CacheEntry<V>> {
        self.insert_with_ttl(key, value, self.default_ttl)
    }
    
    fn insert_with_ttl(&self, key: K, value: V, ttl: Duration) -> Rc<CacheEntry<V>> {
        let entry = Rc::new(CacheEntry::new(value, ttl));
        let weak_entry = Rc::downgrade(&entry);
        
        self.data.borrow_mut().insert(key, weak_entry);
        entry
    }
    
    fn cleanup(&self) {
        let mut data = self.data.borrow_mut();
        data.retain(|_, weak_entry| {
            if let Some(entry) = weak_entry.upgrade() {
                !entry.is_expired()
            } else {
                false
            }
        });
    }
    
    fn size(&self) -> usize {
        self.data.borrow().len()
    }
}

// 使用示例
#[derive(Debug, Clone)]
struct User {
    id: u32,
    name: String,
    email: String,
}

impl User {
    fn new(id: u32, name: &str, email: &str) -> Self {
        User {
            id,
            name: name.to_string(),
            email: email.to_string(),
        }
    }
}

fn main() {
    let cache: Cache<u32, User> = Cache::new(Duration::from_secs(2));
    
    // 插入用户数据
    let user1 = cache.insert(1, User::new(1, "Alice", "alice@example.com"));
    let user2 = cache.insert_with_ttl(2, User::new(2, "Bob", "bob@example.com"), Duration::from_secs(1));
    
    println!("Cache size: {}", cache.size());
    
    // 获取缓存数据
    if let Some(cached_user) = cache.get(&1) {
        println!("Found user: {:?}", cached_user.value);
    }
    
    // 等待部分数据过期
    std::thread::sleep(Duration::from_millis(1500));
    
    // user2应该已经过期
    if cache.get(&2).is_none() {
        println!("User 2 has expired");
    }
    
    // user1仍然有效
    if let Some(cached_user) = cache.get(&1) {
        println!("User 1 still valid: {:?}", cached_user.value.name);
    }
    
    // 清理过期条目
    cache.cleanup();
    println!("Cache size after cleanup: {}", cache.size());
    
    // 释放强引用
    drop(user1);
    drop(user2);
    
    // 再次清理
    cache.cleanup();
    println!("Cache size after dropping references: {}", cache.size());
}
```

## 10. 练习题

### 练习1：实现一个双向链表

```rust
use std::cell::RefCell;
use std::rc::{Rc, Weak};

type NodeRef<T> = Rc<RefCell<Node<T>>>;
type WeakNodeRef<T> = Weak<RefCell<Node<T>>>;

#[derive(Debug)]
struct Node<T> {
    data: T,
    next: Option<NodeRef<T>>,
    prev: Option<WeakNodeRef<T>>,
}

#[derive(Debug)]
struct DoublyLinkedList<T> {
    head: Option<NodeRef<T>>,
    tail: Option<WeakNodeRef<T>>,
    length: usize,
}

impl<T> Node<T> {
    fn new(data: T) -> NodeRef<T> {
        Rc::new(RefCell::new(Node {
            data,
            next: None,
            prev: None,
        }))
    }
}

impl<T> DoublyLinkedList<T> {
    fn new() -> Self {
        DoublyLinkedList {
            head: None,
            tail: None,
            length: 0,
        }
    }
    
    fn push_front(&mut self, data: T) {
        let new_node = Node::new(data);
        
        match self.head.take() {
            Some(old_head) => {
                old_head.borrow_mut().prev = Some(Rc::downgrade(&new_node));
                new_node.borrow_mut().next = Some(old_head);
                self.head = Some(new_node);
            }
            None => {
                self.tail = Some(Rc::downgrade(&new_node));
                self.head = Some(new_node);
            }
        }
        
        self.length += 1;
    }
    
    fn push_back(&mut self, data: T) {
        let new_node = Node::new(data);
        
        match self.tail.take() {
            Some(old_tail) => {
                if let Some(old_tail_rc) = old_tail.upgrade() {
                    old_tail_rc.borrow_mut().next = Some(Rc::clone(&new_node));
                    new_node.borrow_mut().prev = Some(Rc::downgrade(&old_tail_rc));
                    self.tail = Some(Rc::downgrade(&new_node));
                }
            }
            None => {
                self.head = Some(Rc::clone(&new_node));
                self.tail = Some(Rc::downgrade(&new_node));
            }
        }
        
        self.length += 1;
    }
    
    fn pop_front(&mut self) -> Option<T> {
        self.head.take().map(|old_head| {
            match old_head.borrow_mut().next.take() {
                Some(new_head) => {
                    new_head.borrow_mut().prev = None;
                    self.head = Some(new_head);
                }
                None => {
                    self.tail = None;
                }
            }
            
            self.length -= 1;
            
            // 从Rc<RefCell<Node<T>>>中提取T
            Rc::try_unwrap(old_head).ok().unwrap().into_inner().data
        })
    }
    
    fn len(&self) -> usize {
        self.length
    }
    
    fn is_empty(&self) -> bool {
        self.length == 0
    }
}

fn main() {
    let mut list = DoublyLinkedList::new();
    
    // 测试push_front
    list.push_front(1);
    list.push_front(2);
    list.push_front(3);
    
    println!("Length: {}", list.len());
    
    // 测试push_back
    list.push_back(0);
    list.push_back(-1);
    
    println!("Length after push_back: {}", list.len());
    
    // 测试pop_front
    while let Some(value) = list.pop_front() {
        println!("Popped: {}", value);
    }
    
    println!("Final length: {}", list.len());
    println!("Is empty: {}", list.is_empty());
}
```

### 练习2：实现一个观察者模式

```rust
use std::cell::RefCell;
use std::rc::{Rc, Weak};

trait Observer<T> {
    fn update(&self, data: &T);
}

struct Subject<T> {
    observers: RefCell<Vec<Weak<dyn Observer<T>>>>,
    data: RefCell<T>,
}

impl<T> Subject<T> {
    fn new(initial_data: T) -> Rc<Self> {
        Rc::new(Subject {
            observers: RefCell::new(vec![]),
            data: RefCell::new(initial_data),
        })
    }
    
    fn attach(&self, observer: Weak<dyn Observer<T>>) {
        self.observers.borrow_mut().push(observer);
    }
    
    fn detach(&self, observer: &Weak<dyn Observer<T>>) {
        self.observers.borrow_mut().retain(|obs| {
            !obs.ptr_eq(observer)
        });
    }
    
    fn notify(&self) {
        let data = self.data.borrow();
        let mut observers = self.observers.borrow_mut();
        
        // 清理已释放的观察者
        observers.retain(|obs| obs.strong_count() > 0);
        
        // 通知所有观察者
        for observer in observers.iter() {
            if let Some(obs) = observer.upgrade() {
                obs.update(&*data);
            }
        }
    }
    
    fn set_data(&self, new_data: T) {
        *self.data.borrow_mut() = new_data;
        self.notify();
    }
    
    fn get_data(&self) -> T
    where
        T: Clone,
    {
        self.data.borrow().clone()
    }
}

// 具体观察者实现
struct Logger {
    name: String,
}

impl Logger {
    fn new(name: &str) -> Rc<Self> {
        Rc::new(Logger {
            name: name.to_string(),
        })
    }
}

impl Observer<i32> for Logger {
    fn update(&self, data: &i32) {
        println!("{} received update: {}", self.name, data);
    }
}

struct Validator {
    min_value: i32,
}

impl Validator {
    fn new(min_value: i32) -> Rc<Self> {
        Rc::new(Validator { min_value })
    }
}

impl Observer<i32> for Validator {
    fn update(&self, data: &i32) {
        if *data < self.min_value {
            println!("Validator: Warning! Value {} is below minimum {}", data, self.min_value);
        } else {
            println!("Validator: Value {} is valid", data);
        }
    }
}

fn main() {
    let subject = Subject::new(0);
    
    // 创建观察者
    let logger1 = Logger::new("Logger1");
    let logger2 = Logger::new("Logger2");
    let validator = Validator::new(10);
    
    // 注册观察者
    subject.attach(Rc::downgrade(&logger1) as Weak<dyn Observer<i32>>);
    subject.attach(Rc::downgrade(&logger2) as Weak<dyn Observer<i32>>);
    subject.attach(Rc::downgrade(&validator) as Weak<dyn Observer<i32>>);
    
    // 更新数据
    println!("Setting data to 5:");
    subject.set_data(5);
    
    println!("\nSetting data to 15:");
    subject.set_data(15);
    
    // 释放一个观察者
    drop(logger1);
    
    println!("\nAfter dropping logger1, setting data to 20:");
    subject.set_data(20);
    
    println!("\nCurrent data: {}", subject.get_data());
}
```

### 练习3：实现一个简单的图数据结构

```rust
use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::rc::{Rc, Weak};

type NodeRef<T> = Rc<RefCell<GraphNode<T>>>;
type WeakNodeRef<T> = Weak<RefCell<GraphNode<T>>>;

#[derive(Debug)]
struct GraphNode<T> {
    id: usize,
    data: T,
    edges: RefCell<Vec<NodeRef<T>>>,
}

#[derive(Debug)]
struct Graph<T> {
    nodes: RefCell<HashMap<usize, NodeRef<T>>>,
    next_id: RefCell<usize>,
}

impl<T> GraphNode<T> {
    fn new(id: usize, data: T) -> NodeRef<T> {
        Rc::new(RefCell::new(GraphNode {
            id,
            data,
            edges: RefCell::new(vec![]),
        }))
    }
    
    fn add_edge(&self, target: NodeRef<T>) {
        self.edges.borrow_mut().push(target);
    }
    
    fn get_neighbors(&self) -> Vec<NodeRef<T>> {
        self.edges.borrow().clone()
    }
}

impl<T> Graph<T> {
    fn new() -> Self {
        Graph {
            nodes: RefCell::new(HashMap::new()),
            next_id: RefCell::new(0),
        }
    }
    
    fn add_node(&self, data: T) -> usize {
        let id = *self.next_id.borrow();
        *self.next_id.borrow_mut() += 1;
        
        let node = GraphNode::new(id, data);
        self.nodes.borrow_mut().insert(id, node);
        
        id
    }
    
    fn get_node(&self, id: usize) -> Option<NodeRef<T>> {
        self.nodes.borrow().get(&id).cloned()
    }
    
    fn add_edge(&self, from: usize, to: usize) -> Result<(), &'static str> {
        let from_node = self.get_node(from).ok_or("Source node not found")?;
        let to_node = self.get_node(to).ok_or("Target node not found")?;
        
        from_node.borrow().add_edge(to_node);
        Ok(())
    }
    
    fn bfs(&self, start_id: usize) -> Vec<usize>
    where
        T: std::fmt::Debug,
    {
        let mut visited = HashSet::new();
        let mut queue = VecDeque::new();
        let mut result = vec![];
        
        if let Some(start_node) = self.get_node(start_id) {
            queue.push_back(start_node);
            visited.insert(start_id);
            
            while let Some(current) = queue.pop_front() {
                let current_id = current.borrow().id;
                result.push(current_id);
                
                for neighbor in current.borrow().get_neighbors() {
                    let neighbor_id = neighbor.borrow().id;
                    if !visited.contains(&neighbor_id) {
                        visited.insert(neighbor_id);
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        
        result
    }
    
    fn dfs(&self, start_id: usize) -> Vec<usize>
    where
        T: std::fmt::Debug,
    {
        let mut visited = HashSet::new();
        let mut result = vec![];
        
        if let Some(start_node) = self.get_node(start_id) {
            self.dfs_recursive(&start_node, &mut visited, &mut result);
        }
        
        result
    }
    
    fn dfs_recursive(
        &self,
        node: &NodeRef<T>,
        visited: &mut HashSet<usize>,
        result: &mut Vec<usize>,
    ) where
        T: std::fmt::Debug,
    {
        let node_id = node.borrow().id;
        if visited.contains(&node_id) {
            return;
        }
        
        visited.insert(node_id);
        result.push(node_id);
        
        for neighbor in node.borrow().get_neighbors() {
            self.dfs_recursive(&neighbor, visited, result);
        }
    }
    
    fn node_count(&self) -> usize {
        self.nodes.borrow().len()
    }
}

fn main() {
    let graph: Graph<&str> = Graph::new();
    
    // 添加节点
    let node_a = graph.add_node("A");
    let node_b = graph.add_node("B");
    let node_c = graph.add_node("C");
    let node_d = graph.add_node("D");
    let node_e = graph.add_node("E");
    
    // 添加边
    graph.add_edge(node_a, node_b).unwrap();
    graph.add_edge(node_a, node_c).unwrap();
    graph.add_edge(node_b, node_d).unwrap();
    graph.add_edge(node_c, node_d).unwrap();
    graph.add_edge(node_d, node_e).unwrap();
    
    println!("Graph has {} nodes", graph.node_count());
    
    // BFS遍历
    println!("BFS from node {}: {:?}", node_a, graph.bfs(node_a));
    
    // DFS遍历
    println!("DFS from node {}: {:?}", node_a, graph.dfs(node_a));
    
    // 访问节点数据
    if let Some(node) = graph.get_node(node_a) {
        println!("Node {} data: {}", node_a, node.borrow().data);
    }
}
```

## 总结

通过本章的学习，你应该掌握了：

### 核心概念
- 智能指针的基本概念和用途
- `Box<T>`、`Rc<T>`、`Arc<T>`、`RefCell<T>`的使用场景
- 内部可变性和运行时借用检查
- 弱引用和循环引用的处理

### 实用技能
- 选择合适的智能指针类型
- 实现自定义智能指针
- 避免内存泄漏和循环引用
- 性能优化和内存使用优化

### 最佳实践
- 优先使用`Box<T>`进行简单的堆分配
- 使用`Rc<T>`/`Arc<T>`进行数据共享
- 谨慎使用`RefCell<T>`和内部可变性
- 使用`Weak<T>`打破循环引用

### 高级特性
- 自定义`Deref`和`Drop`实现
- 智能指针的组合使用模式
- 复杂数据结构的设计
- 观察者模式和图算法的实现

智能指针是Rust内存管理的核心工具，掌握它们的使用将帮助你构建安全、高效的Rust程序。
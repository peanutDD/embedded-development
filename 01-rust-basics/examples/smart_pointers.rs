// Rust智能指针示例：展示Box、Rc、Arc、RefCell、Weak等智能指针的使用

use std::cell::{Cell, RefCell};
use std::rc::{Rc, Weak};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Duration;

// 1. Box<T> - 堆分配智能指针
#[derive(Debug)]
struct Node {
  value: i32,
  next: Option<Box<Node>>,
}

impl Node {
  fn new(value: i32) -> Self {
    Node { value, next: None }
  }

  fn append(&mut self, value: i32) {
    match &mut self.next {
      Some(next_node) => next_node.append(value),
      None => self.next = Some(Box::new(Node::new(value))),
    }
  }

  fn print_list(&self) {
    print!("{}", self.value);
    if let Some(next) = &self.next {
      print!(" -> ");
      next.print_list();
    }
  }
}

// 2. 递归数据结构
#[derive(Debug)]
enum List {
  Cons(i32, Box<List>),
  Nil,
}

impl List {
  fn new() -> Self {
    List::Nil
  }

  fn prepend(self, value: i32) -> Self {
    List::Cons(value, Box::new(self))
  }

  fn len(&self) -> usize {
    match self {
      List::Cons(_, tail) => 1 + tail.len(),
      List::Nil => 0,
    }
  }

  fn stringify(&self) -> String {
    match self {
      List::Cons(head, tail) => {
        format!("{}, {}", head, tail.stringify())
      }
      List::Nil => "Nil".to_string(),
    }
  }
}

// 3. Rc<T> - 引用计数智能指针
#[derive(Debug)]
struct SharedNode {
  value: i32,
  children: Vec<Rc<SharedNode>>,
}

impl SharedNode {
  fn new(value: i32) -> Rc<Self> {
    Rc::new(SharedNode {
      value,
      children: Vec::new(),
    })
  }

  fn add_child(self: &Rc<Self>, child: Rc<SharedNode>) -> Rc<Self> {
    let mut new_children = self.children.clone();
    new_children.push(child);
    Rc::new(SharedNode {
      value: self.value,
      children: new_children,
    })
  }
}

// 4. 循环引用问题和Weak<T>解决方案
#[derive(Debug)]
struct Parent {
  children: RefCell<Vec<Rc<Child>>>,
}

#[derive(Debug)]
struct Child {
  parent: RefCell<Weak<Parent>>,
  name: String,
}

impl Parent {
  fn new() -> Rc<Self> {
    Rc::new(Parent {
      children: RefCell::new(Vec::new()),
    })
  }

  fn add_child(self: &Rc<Self>, name: String) -> Rc<Child> {
    let child = Rc::new(Child {
      parent: RefCell::new(Rc::downgrade(self)),
      name,
    });

    self.children.borrow_mut().push(child.clone());
    child
  }
}

impl Child {
  fn parent(&self) -> Option<Rc<Parent>> {
    self.parent.borrow().upgrade()
  }
}

// 5. RefCell<T> - 内部可变性
#[derive(Debug)]
struct Counter {
  value: RefCell<i32>,
}

impl Counter {
  fn new(initial: i32) -> Self {
    Counter {
      value: RefCell::new(initial),
    }
  }

  fn increment(&self) {
    let mut value = self.value.borrow_mut();
    *value += 1;
  }

  fn get(&self) -> i32 {
    *self.value.borrow()
  }

  fn add(&self, amount: i32) {
    *self.value.borrow_mut() += amount;
  }
}

// 6. Cell<T> - 简单内部可变性
#[derive(Debug)]
struct SimpleCounter {
  value: Cell<i32>,
}

impl SimpleCounter {
  fn new(initial: i32) -> Self {
    SimpleCounter {
      value: Cell::new(initial),
    }
  }

  fn increment(&self) {
    let current = self.value.get();
    self.value.set(current + 1);
  }

  fn get(&self) -> i32 {
    self.value.get()
  }
}

// 7. Arc<T> - 原子引用计数（线程安全）
#[derive(Debug)]
struct SharedData {
  id: usize,
  data: String,
}

impl SharedData {
  fn new(id: usize, data: String) -> Self {
    SharedData { id, data }
  }

  fn process(&self) {
    println!("处理数据 {}: {}", self.id, self.data);
    thread::sleep(Duration::from_millis(100));
  }
}

// 8. Arc<Mutex<T>> - 线程安全的可变共享数据
#[derive(Debug)]
struct ThreadSafeCounter {
  value: Arc<Mutex<i32>>,
}

impl ThreadSafeCounter {
  fn new(initial: i32) -> Self {
    ThreadSafeCounter {
      value: Arc::new(Mutex::new(initial)),
    }
  }

  fn increment(&self) {
    let mut value = self.value.lock().unwrap();
    *value += 1;
  }

  fn get(&self) -> i32 {
    *self.value.lock().unwrap()
  }

  fn clone_counter(&self) -> Self {
    ThreadSafeCounter {
      value: Arc::clone(&self.value),
    }
  }
}

// 9. Arc<RwLock<T>> - 读写锁
#[derive(Debug)]
struct ReadWriteData {
  data: Arc<RwLock<Vec<i32>>>,
}

impl ReadWriteData {
  fn new() -> Self {
    ReadWriteData {
      data: Arc::new(RwLock::new(Vec::new())),
    }
  }

  fn add(&self, value: i32) {
    let mut data = self.data.write().unwrap();
    data.push(value);
  }

  fn read(&self) -> Vec<i32> {
    let data = self.data.read().unwrap();
    data.clone()
  }

  fn len(&self) -> usize {
    let data = self.data.read().unwrap();
    data.len()
  }

  fn clone_data(&self) -> Self {
    ReadWriteData {
      data: Arc::clone(&self.data),
    }
  }
}

// 10. 自定义智能指针
struct MyBox<T> {
  data: T,
}

impl<T> MyBox<T> {
  fn new(data: T) -> Self {
    MyBox { data }
  }
}

impl<T> std::ops::Deref for MyBox<T> {
  type Target = T;

  fn deref(&self) -> &Self::Target {
    &self.data
  }
}

impl<T> std::ops::DerefMut for MyBox<T> {
  fn deref_mut(&mut self) -> &mut Self::Target {
    &mut self.data
  }
}

impl<T> Drop for MyBox<T> {
  fn drop(&mut self) {
    println!("MyBox被销毁");
  }
}

// 11. 智能指针组合使用
type SharedMutableData = Rc<RefCell<Vec<String>>>;

fn create_shared_data() -> SharedMutableData {
  Rc::new(RefCell::new(vec![
    "数据1".to_string(),
    "数据2".to_string(),
    "数据3".to_string(),
  ]))
}

fn modify_shared_data(data: &SharedMutableData, new_item: String) {
  data.borrow_mut().push(new_item);
}

fn read_shared_data(data: &SharedMutableData) -> Vec<String> {
  data.borrow().clone()
}

// 12. 内存泄漏检测
fn demonstrate_memory_leak() {
  let parent = Parent::new();
  let child1 = parent.add_child("Child1".to_string());
  let child2 = parent.add_child("Child2".to_string());

  println!("父节点有 {} 个子节点", parent.children.borrow().len());
  println!("子节点1: {}", child1.name);
  println!("子节点2: {}", child2.name);

  // 检查引用计数
  println!("父节点引用计数: {}", Rc::strong_count(&parent));
  println!("子节点1引用计数: {}", Rc::strong_count(&child1));

  // 检查弱引用
  if let Some(parent_ref) = child1.parent() {
    println!("子节点1可以访问父节点");
    println!(
      "通过子节点访问的父节点引用计数: {}",
      Rc::strong_count(&parent_ref)
    );
  }
}

fn main() {
  println!("=== Rust智能指针示例 ===\n");

  // 1. Box<T>示例
  println!("1. Box<T> - 堆分配");
  let mut list = Node::new(1);
  list.append(2);
  list.append(3);
  print!("链表: ");
  list.print_list();
  println!();

  // 递归数据结构
  let list = List::new().prepend(1).prepend(2).prepend(3);
  println!("递归列表: {}", list.stringify());
  println!("列表长度: {}", list.len());

  // 2. Rc<T>示例
  println!("\n2. Rc<T> - 引用计数");
  let root = SharedNode::new(1);
  let child1 = SharedNode::new(2);
  let child2 = SharedNode::new(3);

  let root_with_children = root.add_child(child1.clone()).add_child(child2.clone());
  println!("根节点: {:?}", root_with_children);
  println!("child1引用计数: {}", Rc::strong_count(&child1));
  println!("child2引用计数: {}", Rc::strong_count(&child2));

  // 3. RefCell<T>示例
  println!("\n3. RefCell<T> - 内部可变性");
  let counter = Counter::new(0);
  println!("初始值: {}", counter.get());
  counter.increment();
  counter.add(5);
  println!("修改后: {}", counter.get());

  // 4. Cell<T>示例
  println!("\n4. Cell<T> - 简单内部可变性");
  let simple_counter = SimpleCounter::new(10);
  println!("初始值: {}", simple_counter.get());
  simple_counter.increment();
  println!("递增后: {}", simple_counter.get());

  // 5. Arc<T>多线程示例
  println!("\n5. Arc<T> - 多线程共享");
  let shared_data = Arc::new(SharedData::new(1, "共享数据".to_string()));
  let mut handles = vec![];

  for i in 0..3 {
    let data_clone = Arc::clone(&shared_data);
    let handle = thread::spawn(move || {
      println!("线程 {} 开始处理", i);
      data_clone.process();
      println!("线程 {} 完成处理", i);
    });
    handles.push(handle);
  }

  for handle in handles {
    handle.join().unwrap();
  }

  // 6. Arc<Mutex<T>>示例
  println!("\n6. Arc<Mutex<T>> - 线程安全计数器");
  let counter = ThreadSafeCounter::new(0);
  let mut handles = vec![];

  for i in 0..5 {
    let counter_clone = counter.clone_counter();
    let handle = thread::spawn(move || {
      for _ in 0..10 {
        counter_clone.increment();
      }
      println!("线程 {} 完成", i);
    });
    handles.push(handle);
  }

  for handle in handles {
    handle.join().unwrap();
  }

  println!("最终计数: {}", counter.get());

  // 7. Arc<RwLock<T>>示例
  println!("\n7. Arc<RwLock<T>> - 读写锁");
  let rw_data = ReadWriteData::new();
  let mut handles = vec![];

  // 写入线程
  for i in 0..3 {
    let data_clone = rw_data.clone_data();
    let handle = thread::spawn(move || {
      for j in 0..5 {
        data_clone.add(i * 10 + j);
      }
      println!("写入线程 {} 完成", i);
    });
    handles.push(handle);
  }

  // 读取线程
  for i in 0..2 {
    let data_clone = rw_data.clone_data();
    let handle = thread::spawn(move || {
      thread::sleep(Duration::from_millis(50));
      let data = data_clone.read();
      println!("读取线程 {} 读到 {} 个元素", i, data.len());
    });
    handles.push(handle);
  }

  for handle in handles {
    handle.join().unwrap();
  }

  println!("最终数据: {:?}", rw_data.read());

  // 8. 自定义智能指针
  println!("\n8. 自定义智能指针");
  let my_box = MyBox::new("Hello, MyBox!".to_string());
  println!("MyBox内容: {}", *my_box);
  println!("MyBox长度: {}", my_box.len());

  // 9. 智能指针组合
  println!("\n9. 智能指针组合使用");
  let shared_data = create_shared_data();
  let data_clone1 = Rc::clone(&shared_data);
  let data_clone2 = Rc::clone(&shared_data);

  modify_shared_data(&data_clone1, "新数据1".to_string());
  modify_shared_data(&data_clone2, "新数据2".to_string());

  let final_data = read_shared_data(&shared_data);
  println!("共享数据: {:?}", final_data);
  println!("引用计数: {}", Rc::strong_count(&shared_data));

  // 10. 循环引用和内存泄漏
  println!("\n10. 循环引用处理");
  demonstrate_memory_leak();

  println!("\n=== 智能指针最佳实践 ===");
  println!("1. Box<T>: 堆分配单一所有者数据");
  println!("2. Rc<T>: 单线程多所有者数据");
  println!("3. Arc<T>: 多线程多所有者数据");
  println!("4. RefCell<T>: 运行时借用检查的内部可变性");
  println!("5. Cell<T>: Copy类型的内部可变性");
  println!("6. Mutex<T>: 线程安全的互斥访问");
  println!("7. RwLock<T>: 读写分离的线程安全访问");
  println!("8. Weak<T>: 避免循环引用的弱引用");
}

// 智能指针工具函数
mod smart_pointer_utils {
  use super::*;

  // 安全的Rc克隆
  pub fn safe_rc_clone<T>(rc: &Rc<T>) -> Option<Rc<T>> {
    if Rc::strong_count(rc) < 100 {
      // 防止过多引用
      Some(Rc::clone(rc))
    } else {
      None
    }
  }

  // 安全的Arc克隆
  pub fn safe_arc_clone<T>(arc: &Arc<T>) -> Option<Arc<T>> {
    if Arc::strong_count(arc) < 100 {
      Some(Arc::clone(arc))
    } else {
      None
    }
  }

  // RefCell安全借用
  pub fn safe_borrow<T>(cell: &RefCell<T>) -> Option<std::cell::Ref<T>> {
    cell.try_borrow().ok()
  }

  // RefCell安全可变借用
  pub fn safe_borrow_mut<T>(cell: &RefCell<T>) -> Option<std::cell::RefMut<T>> {
    cell.try_borrow_mut().ok()
  }

  // Mutex安全锁定
  pub fn safe_lock<T>(mutex: &Mutex<T>) -> Option<std::sync::MutexGuard<T>> {
    mutex.try_lock().ok()
  }

  // RwLock安全读锁
  pub fn safe_read<T>(rw_lock: &RwLock<T>) -> Option<std::sync::RwLockReadGuard<T>> {
    rw_lock.try_read().ok()
  }

  // RwLock安全写锁
  pub fn safe_write<T>(rw_lock: &RwLock<T>) -> Option<std::sync::RwLockWriteGuard<T>> {
    rw_lock.try_write().ok()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_box_linked_list() {
    let mut list = Node::new(1);
    list.append(2);
    list.append(3);
    assert_eq!(list.value, 1);
  }

  #[test]
  fn test_recursive_list() {
    let list = List::new().prepend(1).prepend(2).prepend(3);
    assert_eq!(list.len(), 3);
  }

  #[test]
  fn test_rc_reference_counting() {
    let data = Rc::new(42);
    let data2 = Rc::clone(&data);
    assert_eq!(Rc::strong_count(&data), 2);
    drop(data2);
    assert_eq!(Rc::strong_count(&data), 1);
  }

  #[test]
  fn test_refcell_interior_mutability() {
    let counter = Counter::new(0);
    counter.increment();
    counter.add(5);
    assert_eq!(counter.get(), 6);
  }

  #[test]
  fn test_cell_simple_mutability() {
    let counter = SimpleCounter::new(10);
    counter.increment();
    assert_eq!(counter.get(), 11);
  }

  #[test]
  fn test_shared_mutable_data() {
    let data = create_shared_data();
    modify_shared_data(&data, "测试数据".to_string());
    let result = read_shared_data(&data);
    assert_eq!(result.len(), 4);
    assert_eq!(result[3], "测试数据");
  }

  #[test]
  fn test_parent_child_relationship() {
    let parent = Parent::new();
    let child = parent.add_child("测试子节点".to_string());

    assert_eq!(parent.children.borrow().len(), 1);
    assert_eq!(child.name, "测试子节点");
    assert!(child.parent().is_some());
  }
}

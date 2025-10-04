//! 综合练习
//!
//! 本文件包含结合多个Rust概念的综合练习，包括：
//! - 多个概念的综合应用
//! - 实际项目场景模拟
//! - 性能优化练习
//! - 错误处理最佳实践
//! - 设计模式实现

#![allow(dead_code, unused_variables)]

use std::cell::RefCell;
use std::collections::{HashMap, HashSet, VecDeque};
use std::fmt;
use std::rc::Rc;

fn main() {
  println!("=== 综合练习 ===");
  println!("运行 cargo test comprehensive 来测试你的答案");
  println!();

  // 示例运行
  let mut library = Library::new();
  let book = Book::new(1, "Rust编程语言".to_string(), "Steve Klabnik".to_string());
  library.add_book(book);
  println!("图书馆中有 {} 本书", library.book_count());
}

// ============================================================================
// 综合练习1: 图书管理系统
// ============================================================================

/// 图书结构体
#[derive(Debug, Clone, PartialEq)]
struct Book {
  id: u32,
  title: String,
  author: String,
  isbn: Option<String>,
  available: bool,
}

impl Book {
  fn new(id: u32, title: String, author: String) -> Self {
    Book {
      id,
      title,
      author,
      isbn: None,
      available: true,
    }
  }

  fn with_isbn(mut self, isbn: String) -> Self {
    self.isbn = Some(isbn);
    self
  }

  fn borrow_book(&mut self) -> Result<(), String> {
    if self.available {
      self.available = false;
      Ok(())
    } else {
      Err("图书已被借出".to_string())
    }
  }

  fn return_book(&mut self) -> Result<(), String> {
    if !self.available {
      self.available = true;
      Ok(())
    } else {
      Err("图书未被借出".to_string())
    }
  }
}

/// 用户结构体
#[derive(Debug, Clone, PartialEq)]
struct User {
  id: u32,
  name: String,
  email: String,
  borrowed_books: Vec<u32>, // 存储借阅的书籍ID
}

impl User {
  fn new(id: u32, name: String, email: String) -> Self {
    User {
      id,
      name,
      email,
      borrowed_books: Vec::new(),
    }
  }

  fn borrow_book(&mut self, book_id: u32) -> Result<(), String> {
    if self.borrowed_books.len() >= 5 {
      return Err("借阅数量已达上限".to_string());
    }

    if self.borrowed_books.contains(&book_id) {
      return Err("已借阅此书".to_string());
    }

    self.borrowed_books.push(book_id);
    Ok(())
  }

  fn return_book(&mut self, book_id: u32) -> Result<(), String> {
    if let Some(pos) = self.borrowed_books.iter().position(|&id| id == book_id) {
      self.borrowed_books.remove(pos);
      Ok(())
    } else {
      Err("未借阅此书".to_string())
    }
  }
}

/// 图书馆系统
#[derive(Debug)]
struct Library {
  books: HashMap<u32, Book>,
  users: HashMap<u32, User>,
  next_book_id: u32,
  next_user_id: u32,
}

impl Library {
  fn new() -> Self {
    Library {
      books: HashMap::new(),
      users: HashMap::new(),
      next_book_id: 1,
      next_user_id: 1,
    }
  }

  fn add_book(&mut self, mut book: Book) -> u32 {
    let id = self.next_book_id;
    book.id = id;
    self.books.insert(id, book);
    self.next_book_id += 1;
    id
  }

  fn add_user(&mut self, mut user: User) -> u32 {
    let id = self.next_user_id;
    user.id = id;
    self.users.insert(id, user);
    self.next_user_id += 1;
    id
  }

  fn borrow_book(&mut self, user_id: u32, book_id: u32) -> Result<(), String> {
    // 检查用户是否存在
    let user = self
      .users
      .get_mut(&user_id)
      .ok_or("用户不存在".to_string())?;

    // 检查图书是否存在
    let book = self
      .books
      .get_mut(&book_id)
      .ok_or("图书不存在".to_string())?;

    // 尝试借阅
    user.borrow_book(book_id)?;
    book.borrow_book()?;

    Ok(())
  }

  fn return_book(&mut self, user_id: u32, book_id: u32) -> Result<(), String> {
    let user = self
      .users
      .get_mut(&user_id)
      .ok_or("用户不存在".to_string())?;

    let book = self
      .books
      .get_mut(&book_id)
      .ok_or("图书不存在".to_string())?;

    user.return_book(book_id)?;
    book.return_book()?;

    Ok(())
  }

  fn search_books(&self, query: &str) -> Vec<&Book> {
    self
      .books
      .values()
      .filter(|book| {
        book.title.to_lowercase().contains(&query.to_lowercase())
          || book.author.to_lowercase().contains(&query.to_lowercase())
      })
      .collect()
  }

  fn available_books(&self) -> Vec<&Book> {
    self.books.values().filter(|book| book.available).collect()
  }

  fn book_count(&self) -> usize {
    self.books.len()
  }

  fn user_count(&self) -> usize {
    self.users.len()
  }
}

// ============================================================================
// 综合练习2: 任务调度系统
// ============================================================================

/// 任务优先级
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum Priority {
  Low = 1,
  Medium = 2,
  High = 3,
  Critical = 4,
}

/// 任务状态
#[derive(Debug, Clone, PartialEq)]
enum TaskStatus {
  Pending,
  Running,
  Completed,
  Failed(String),
}

/// 任务结构体
#[derive(Debug, Clone)]
struct Task {
  id: u32,
  name: String,
  description: String,
  priority: Priority,
  status: TaskStatus,
  dependencies: Vec<u32>,  // 依赖的任务ID
  estimated_duration: u32, // 预估时间（分钟）
}

impl Task {
  fn new(id: u32, name: String, description: String, priority: Priority) -> Self {
    Task {
      id,
      name,
      description,
      priority,
      status: TaskStatus::Pending,
      dependencies: Vec::new(),
      estimated_duration: 0,
    }
  }

  fn with_dependencies(mut self, deps: Vec<u32>) -> Self {
    self.dependencies = deps;
    self
  }

  fn with_duration(mut self, duration: u32) -> Self {
    self.estimated_duration = duration;
    self
  }

  fn can_start(&self, completed_tasks: &HashSet<u32>) -> bool {
    self.status == TaskStatus::Pending
      && self
        .dependencies
        .iter()
        .all(|&dep| completed_tasks.contains(&dep))
  }

  fn start(&mut self) -> Result<(), String> {
    if self.status != TaskStatus::Pending {
      return Err("任务不在待执行状态".to_string());
    }
    self.status = TaskStatus::Running;
    Ok(())
  }

  fn complete(&mut self) -> Result<(), String> {
    if self.status != TaskStatus::Running {
      return Err("任务不在执行状态".to_string());
    }
    self.status = TaskStatus::Completed;
    Ok(())
  }

  fn fail(&mut self, reason: String) -> Result<(), String> {
    if self.status != TaskStatus::Running {
      return Err("任务不在执行状态".to_string());
    }
    self.status = TaskStatus::Failed(reason);
    Ok(())
  }
}

/// 任务调度器
#[derive(Debug)]
struct TaskScheduler {
  tasks: HashMap<u32, Task>,
  completed_tasks: HashSet<u32>,
  next_task_id: u32,
}

impl TaskScheduler {
  fn new() -> Self {
    TaskScheduler {
      tasks: HashMap::new(),
      completed_tasks: HashSet::new(),
      next_task_id: 1,
    }
  }

  fn add_task(&mut self, mut task: Task) -> u32 {
    let id = self.next_task_id;
    task.id = id;
    self.tasks.insert(id, task);
    self.next_task_id += 1;
    id
  }

  fn get_ready_tasks(&self) -> Vec<&Task> {
    self
      .tasks
      .values()
      .filter(|task| task.can_start(&self.completed_tasks))
      .collect()
  }

  fn get_next_task(&self) -> Option<&Task> {
    self
      .get_ready_tasks()
      .into_iter()
      .max_by_key(|task| task.priority)
  }

  fn start_task(&mut self, task_id: u32) -> Result<(), String> {
    let task = self
      .tasks
      .get_mut(&task_id)
      .ok_or("任务不存在".to_string())?;

    if !task.can_start(&self.completed_tasks) {
      return Err("任务依赖未满足".to_string());
    }

    task.start()
  }

  fn complete_task(&mut self, task_id: u32) -> Result<(), String> {
    let task = self
      .tasks
      .get_mut(&task_id)
      .ok_or("任务不存在".to_string())?;

    task.complete()?;
    self.completed_tasks.insert(task_id);
    Ok(())
  }

  fn get_task_statistics(&self) -> TaskStatistics {
    let mut stats = TaskStatistics::default();

    for task in self.tasks.values() {
      match task.status {
        TaskStatus::Pending => stats.pending += 1,
        TaskStatus::Running => stats.running += 1,
        TaskStatus::Completed => stats.completed += 1,
        TaskStatus::Failed(_) => stats.failed += 1,
      }
      stats.total_estimated_time += task.estimated_duration;
    }

    stats
  }
}

#[derive(Debug, Default)]
struct TaskStatistics {
  pending: usize,
  running: usize,
  completed: usize,
  failed: usize,
  total_estimated_time: u32,
}

// ============================================================================
// 综合练习3: 缓存系统
// ============================================================================

/// LRU缓存实现
struct LRUCache<K, V> {
  capacity: usize,
  cache: HashMap<K, Rc<RefCell<Node<K, V>>>>,
  head: Option<Rc<RefCell<Node<K, V>>>>,
  tail: Option<Rc<RefCell<Node<K, V>>>>,
}

struct Node<K, V> {
  key: K,
  value: V,
  prev: Option<Rc<RefCell<Node<K, V>>>>,
  next: Option<Rc<RefCell<Node<K, V>>>>,
}

impl<K, V> Node<K, V> {
  fn new(key: K, value: V) -> Self {
    Node {
      key,
      value,
      prev: None,
      next: None,
    }
  }
}

impl<K, V> LRUCache<K, V>
where
  K: Clone + std::hash::Hash + Eq,
  V: Clone,
{
  fn new(capacity: usize) -> Self {
    LRUCache {
      capacity,
      cache: HashMap::new(),
      head: None,
      tail: None,
    }
  }

  fn get(&mut self, key: &K) -> Option<V> {
    if let Some(node) = self.cache.get(key).cloned() {
      self.move_to_front(node.clone());
      Some(node.borrow().value.clone())
    } else {
      None
    }
  }

  fn put(&mut self, key: K, value: V) {
    if let Some(node) = self.cache.get(&key).cloned() {
      node.borrow_mut().value = value;
      self.move_to_front(node);
    } else {
      let new_node = Rc::new(RefCell::new(Node::new(key.clone(), value)));

      if self.cache.len() >= self.capacity {
        self.remove_tail();
      }

      self.add_to_front(new_node.clone());
      self.cache.insert(key, new_node);
    }
  }

  fn move_to_front(&mut self, node: Rc<RefCell<Node<K, V>>>) {
    self.remove_node(node.clone());
    self.add_to_front(node);
  }

  fn add_to_front(&mut self, node: Rc<RefCell<Node<K, V>>>) {
    if let Some(head) = &self.head {
      node.borrow_mut().next = Some(head.clone());
      head.borrow_mut().prev = Some(node.clone());
    } else {
      self.tail = Some(node.clone());
    }
    self.head = Some(node);
  }

  fn remove_node(&mut self, node: Rc<RefCell<Node<K, V>>>) {
        let prev = node.borrow().prev.clone();
        let next = node.borrow().next.clone();
        
        if let Some(prev_node) = &prev {
            prev_node.borrow_mut().next = next.clone();
        } else {
            self.head = next.clone();
        }
        
        if let Some(next_node) = &next {
            next_node.borrow_mut().prev = prev;
        } else {
            self.tail = prev;
        }
        
        node.borrow_mut().prev = None;
        node.borrow_mut().next = None;
    }

  fn remove_tail(&mut self) {
    if let Some(tail) = &self.tail {
      let key = tail.borrow().key.clone();
      self.cache.remove(&key);
      self.remove_node(tail.clone());
    }
  }

  fn len(&self) -> usize {
    self.cache.len()
  }

  fn is_empty(&self) -> bool {
    self.cache.is_empty()
  }
}

// ============================================================================
// 综合练习4: 表达式解析器和计算器
// ============================================================================

/// 词法分析器的Token
#[derive(Debug, PartialEq, Clone)]
enum Token {
  Number(f64),
  Plus,
  Minus,
  Multiply,
  Divide,
  LeftParen,
  RightParen,
  EOF,
}

/// 词法分析器
struct Lexer {
  input: Vec<char>,
  position: usize,
}

impl Lexer {
  fn new(input: &str) -> Self {
    Lexer {
      input: input.chars().collect(),
      position: 0,
    }
  }

  fn next_token(&mut self) -> Result<Token, String> {
    self.skip_whitespace();

    if self.position >= self.input.len() {
      return Ok(Token::EOF);
    }

    let ch = self.input[self.position];

    match ch {
      '+' => {
        self.position += 1;
        Ok(Token::Plus)
      }
      '-' => {
        self.position += 1;
        Ok(Token::Minus)
      }
      '*' => {
        self.position += 1;
        Ok(Token::Multiply)
      }
      '/' => {
        self.position += 1;
        Ok(Token::Divide)
      }
      '(' => {
        self.position += 1;
        Ok(Token::LeftParen)
      }
      ')' => {
        self.position += 1;
        Ok(Token::RightParen)
      }
      '0'..='9' | '.' => self.read_number(),
      _ => Err(format!("意外的字符: {}", ch)),
    }
  }

  fn read_number(&mut self) -> Result<Token, String> {
    let start = self.position;

    while self.position < self.input.len() {
      let ch = self.input[self.position];
      if ch.is_ascii_digit() || ch == '.' {
        self.position += 1;
      } else {
        break;
      }
    }

    let number_str: String = self.input[start..self.position].iter().collect();
    let number = number_str
      .parse::<f64>()
      .map_err(|_| format!("无效的数字: {}", number_str))?;

    Ok(Token::Number(number))
  }

  fn skip_whitespace(&mut self) {
    while self.position < self.input.len() && self.input[self.position].is_whitespace() {
      self.position += 1;
    }
  }
}

/// 递归下降解析器
struct Parser {
  lexer: Lexer,
  current_token: Token,
}

impl Parser {
  fn new(mut lexer: Lexer) -> Result<Self, String> {
    let current_token = lexer.next_token()?;
    Ok(Parser {
      lexer,
      current_token,
    })
  }

  fn parse(&mut self) -> Result<f64, String> {
    let result = self.expression()?;
    if self.current_token != Token::EOF {
      return Err("表达式解析不完整".to_string());
    }
    Ok(result)
  }

  fn expression(&mut self) -> Result<f64, String> {
    let mut result = self.term()?;

    while matches!(self.current_token, Token::Plus | Token::Minus) {
      let op = self.current_token.clone();
      self.consume_token()?;
      let right = self.term()?;

      match op {
        Token::Plus => result += right,
        Token::Minus => result -= right,
        _ => unreachable!(),
      }
    }

    Ok(result)
  }

  fn term(&mut self) -> Result<f64, String> {
    let mut result = self.factor()?;

    while matches!(self.current_token, Token::Multiply | Token::Divide) {
      let op = self.current_token.clone();
      self.consume_token()?;
      let right = self.factor()?;

      match op {
        Token::Multiply => result *= right,
        Token::Divide => {
          if right == 0.0 {
            return Err("除数不能为零".to_string());
          }
          result /= right;
        }
        _ => unreachable!(),
      }
    }

    Ok(result)
  }

  fn factor(&mut self) -> Result<f64, String> {
    match &self.current_token {
      Token::Number(n) => {
        let result = *n;
        self.consume_token()?;
        Ok(result)
      }
      Token::LeftParen => {
        self.consume_token()?; // consume '('
        let result = self.expression()?;
        if self.current_token != Token::RightParen {
          return Err("缺少右括号".to_string());
        }
        self.consume_token()?; // consume ')'
        Ok(result)
      }
      Token::Minus => {
        self.consume_token()?; // consume '-'
        let result = self.factor()?;
        Ok(-result)
      }
      _ => Err(format!("意外的token: {:?}", self.current_token)),
    }
  }

  fn consume_token(&mut self) -> Result<(), String> {
    self.current_token = self.lexer.next_token()?;
    Ok(())
  }
}

/// 计算器主函数
fn calculate(expression: &str) -> Result<f64, String> {
  let lexer = Lexer::new(expression);
  let mut parser = Parser::new(lexer)?;
  parser.parse()
}

// ============================================================================
// 测试用例
// ============================================================================

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_library_system() {
    let mut library = Library::new();

    // 添加图书
    let book_id = library.add_book(Book::new(
      0,
      "Rust编程语言".to_string(),
      "Steve Klabnik".to_string(),
    ));

    // 添加用户
    let user_id = library.add_user(User::new(
      0,
      "Alice".to_string(),
      "alice@example.com".to_string(),
    ));

    // 借阅图书
    assert!(library.borrow_book(user_id, book_id).is_ok());

    // 重复借阅应该失败
    assert!(library.borrow_book(user_id, book_id).is_err());

    // 归还图书
    assert!(library.return_book(user_id, book_id).is_ok());

    // 搜索图书
    let results = library.search_books("Rust");
    assert_eq!(results.len(), 1);
  }

  #[test]
  fn test_task_scheduler() {
    let mut scheduler = TaskScheduler::new();

    // 添加任务
    let task1_id = scheduler.add_task(Task::new(
      0,
      "任务1".to_string(),
      "第一个任务".to_string(),
      Priority::High,
    ));

    let task2_id = scheduler.add_task(
      Task::new(
        0,
        "任务2".to_string(),
        "第二个任务".to_string(),
        Priority::Medium,
      )
      .with_dependencies(vec![task1_id]),
    );

    // 获取下一个任务（应该是task1，因为task2有依赖）
    let next_task = scheduler.get_next_task().unwrap();
    assert_eq!(next_task.id, task1_id);

    // 开始并完成task1
    scheduler.start_task(task1_id).unwrap();
    scheduler.complete_task(task1_id).unwrap();

    // 现在task2应该可以执行了
    let next_task = scheduler.get_next_task().unwrap();
    assert_eq!(next_task.id, task2_id);
  }

  #[test]
  fn test_lru_cache() {
    let mut cache = LRUCache::new(2);

    cache.put("a", 1);
    cache.put("b", 2);

    assert_eq!(cache.get(&"a"), Some(1));
    assert_eq!(cache.get(&"b"), Some(2));

    // 添加第三个元素，应该淘汰最久未使用的
    cache.put("c", 3);

    // "a"被访问过，所以"b"应该被淘汰
    assert_eq!(cache.get(&"a"), Some(1));
    assert_eq!(cache.get(&"b"), None);
    assert_eq!(cache.get(&"c"), Some(3));
  }

  #[test]
  fn test_calculator() {
    assert_eq!(calculate("2 + 3").unwrap(), 5.0);
    assert_eq!(calculate("2 * 3 + 4").unwrap(), 10.0);
    assert_eq!(calculate("(2 + 3) * 4").unwrap(), 20.0);
    assert_eq!(calculate("10 / 2 - 3").unwrap(), 2.0);
    assert_eq!(calculate("-5 + 3").unwrap(), -2.0);

    // 测试错误情况
    assert!(calculate("10 / 0").is_err());
    assert!(calculate("2 + ").is_err());
    assert!(calculate("(2 + 3").is_err());
  }
}

// ============================================================================
// 性能测试和基准测试
// ============================================================================

#[cfg(test)]
mod performance_tests {
  use super::*;
  use std::time::Instant;

  #[test]
  fn benchmark_library_operations() {
    let mut library = Library::new();

    // 添加大量图书
    let start = Instant::now();
    for i in 0..1000 {
      library.add_book(Book::new(
        0,
        format!("Book {}", i),
        format!("Author {}", i % 100),
      ));
    }
    let duration = start.elapsed();
    println!("添加1000本书耗时: {:?}", duration);

    // 搜索测试
    let start = Instant::now();
    let results = library.search_books("Book 500");
    let duration = start.elapsed();
    println!("搜索耗时: {:?}, 结果数: {}", duration, results.len());
  }

  #[test]
  fn benchmark_lru_cache() {
    let mut cache = LRUCache::new(100);

    // 填充缓存
    let start = Instant::now();
    for i in 0..100 {
      cache.put(i, i * 2);
    }
    let duration = start.elapsed();
    println!("填充100个元素耗时: {:?}", duration);

    // 随机访问测试
    let start = Instant::now();
    for i in 0..1000 {
      cache.get(&(i % 100));
    }
    let duration = start.elapsed();
    println!("1000次随机访问耗时: {:?}", duration);
  }

  #[test]
  fn benchmark_calculator() {
    let expressions = vec![
      "1 + 2 * 3",
      "(1 + 2) * 3",
      "10 / 2 + 3 * 4",
      "((1 + 2) * 3 - 4) / 5",
      "1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10",
    ];

    let start = Instant::now();
    for _ in 0..1000 {
      for expr in &expressions {
        let _ = calculate(expr);
      }
    }
    let duration = start.elapsed();
    println!("计算5000个表达式耗时: {:?}", duration);
  }
}

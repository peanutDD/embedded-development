// 高级所有权示例：展示复杂的所有权转移和借用场景

use std::collections::HashMap;

// 1. 所有权转移和返回
fn take_and_give_back(s: String) -> String {
  println!("接收到字符串: {}", s);
  s // 返回所有权
}

// 2. 借用和可变借用
fn calculate_length(s: &String) -> usize {
  s.len() // 借用不会获取所有权
}

fn change_string(s: &mut String) {
  s.push_str(", world!");
}

// 3. 切片使用
fn first_word(s: &str) -> &str {
  let bytes = s.as_bytes();

  for (i, &item) in bytes.iter().enumerate() {
    if item == b' ' {
      return &s[0..i];
    }
  }

  &s[..]
}

// 4. 结构体中的所有权
#[derive(Debug)]
struct User {
  username: String,
  email: String,
  sign_in_count: u64,
  active: bool,
}

impl User {
  fn new(username: String, email: String) -> User {
    User {
      username,
      email,
      sign_in_count: 1,
      active: true,
    }
  }

  // 借用self
  fn get_info(&self) -> String {
    format!("用户: {}, 邮箱: {}", self.username, self.email)
  }

  // 可变借用self
  fn increment_sign_in(&mut self) {
    self.sign_in_count += 1;
  }

  // 获取所有权
  fn into_username(self) -> String {
    self.username
  }
}

// 5. 生命周期示例
fn longest<'a>(x: &'a str, y: &'a str) -> &'a str {
  if x.len() > y.len() {
    x
  } else {
    y
  }
}

// 6. 结构体生命周期
#[derive(Debug)]
struct ImportantExcerpt<'a> {
  part: &'a str,
}

impl<'a> ImportantExcerpt<'a> {
  fn level(&self) -> i32 {
    3
  }

  fn announce_and_return_part(&self, announcement: &str) -> &str {
    println!("注意！{}", announcement);
    self.part
  }
}

// 7. 智能指针示例
use std::cell::RefCell;
use std::rc::Rc;

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

// 8. 错误处理与所有权
#[derive(Debug)]
enum MyError {
  InvalidInput(String),
  ProcessingError,
}

fn process_data(data: Vec<i32>) -> Result<Vec<i32>, MyError> {
  if data.is_empty() {
    return Err(MyError::InvalidInput("数据不能为空".to_string()));
  }

  let processed: Result<Vec<i32>, _> = data
    .into_iter()
    .map(|x| {
      if x < 0 {
        Err(MyError::ProcessingError)
      } else {
        Ok(x * 2)
      }
    })
    .collect();

  processed
}

// 9. 闭包与所有权
fn closure_examples() {
  let mut list = vec![1, 2, 3];
  println!("闭包调用前: {:?}", list);

  // 不可变借用
  let only_borrows = || println!("从闭包中: {:?}", list);
  only_borrows();

  // 可变借用
  let mut borrows_mutably = || list.push(7);
  borrows_mutably();
  println!("闭包调用后: {:?}", list);

  // 获取所有权
  let moves_list = move || {
    println!("从move闭包中: {:?}", list);
  };
  moves_list();
  // println!("{:?}", list); // 这里会编译错误，因为list已被移动
}

// 10. 高级模式匹配
fn advanced_pattern_matching() {
  let numbers = vec![1, 2, 3, 4, 5];

  match numbers.as_slice() {
    [] => println!("空向量"),
    [x] => println!("单个元素: {}", x),
    [x, y] => println!("两个元素: {}, {}", x, y),
    [x, .., y] => println!("首尾元素: {}, {}", x, y),
  }

  // 解构结构体
  let user = User::new("alice".to_string(), "alice@example.com".to_string());
  let User {
    username, email, ..
  } = user;
  println!("解构得到: {}, {}", username, email);
}

fn main() {
  println!("=== 高级所有权示例 ===\n");

  // 1. 所有权转移
  let s1 = String::from("hello");
  let s2 = take_and_give_back(s1);
  println!("返回的字符串: {}\n", s2);

  // 2. 借用示例
  let s3 = String::from("hello");
  let len = calculate_length(&s3);
  println!("字符串 '{}' 的长度是 {}", s3, len);

  let mut s4 = String::from("hello");
  change_string(&mut s4);
  println!("修改后的字符串: {}\n", s4);

  // 3. 切片示例
  let sentence = "hello world rust programming";
  let word = first_word(sentence);
  println!("第一个单词: {}\n", word);

  // 4. 结构体示例
  let mut user = User::new("bob".to_string(), "bob@example.com".to_string());
  println!("用户信息: {}", user.get_info());
  user.increment_sign_in();
  println!("登录次数: {}", user.sign_in_count);

  // 注意：这里user的所有权被转移了
  let username = user.into_username();
  println!("提取的用户名: {}\n", username);

  // 5. 生命周期示例
  let string1 = String::from("abcd");
  let string2 = "xyz";
  let result = longest(string1.as_str(), string2);
  println!("更长的字符串: {}\n", result);

  // 6. 结构体生命周期
  let novel = String::from("Call me Ishmael. Some years ago...");
  let first_sentence = novel.split('.').next().expect("Could not find a '.'");
  let excerpt = ImportantExcerpt {
    part: first_sentence,
  };
  println!("重要摘录: {:?}\n", excerpt);

  // 7. 智能指针示例
  let root = Node::new(1);
  let child1 = Node::new(2);
  let child2 = Node::new(3);

  root.add_child(child1);
  root.add_child(child2);
  println!("节点树: {:?}\n", root);

  // 8. 错误处理示例
  let data = vec![1, 2, 3, 4, 5];
  match process_data(data) {
    Ok(processed) => println!("处理结果: {:?}", processed),
    Err(e) => println!("处理错误: {:?}", e),
  }

  let empty_data = vec![];
  match process_data(empty_data) {
    Ok(processed) => println!("处理结果: {:?}", processed),
    Err(e) => println!("处理错误: {:?}\n", e),
  }

  // 9. 闭包示例
  closure_examples();

  // 10. 高级模式匹配
  println!("\n=== 高级模式匹配 ===");
  advanced_pattern_matching();

  println!("\n=== 内存安全演示 ===");
  memory_safety_demo();
}

// 内存安全演示
fn memory_safety_demo() {
  // Rust防止的常见内存错误

  // 1. 防止悬垂指针
  let reference_to_nothing = dangle_prevention();
  println!("安全的引用: {}", reference_to_nothing);

  // 2. 防止数据竞争
  use std::sync::{Arc, Mutex};
  use std::thread;

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

  println!("计数器结果: {}", *counter.lock().unwrap());
}

fn dangle_prevention() -> String {
  // 这个函数返回String而不是&str，避免悬垂引用
  String::from("这是一个安全的字符串")
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_first_word() {
    assert_eq!(first_word("hello world"), "hello");
    assert_eq!(first_word("hello"), "hello");
  }

  #[test]
  fn test_user_creation() {
    let user = User::new("test".to_string(), "test@example.com".to_string());
    assert_eq!(user.username, "test");
    assert_eq!(user.sign_in_count, 1);
    assert!(user.active);
  }

  #[test]
  fn test_process_data() {
    let data = vec![1, 2, 3];
    let result = process_data(data).unwrap();
    assert_eq!(result, vec![2, 4, 6]);

    let empty_data = vec![];
    assert!(process_data(empty_data).is_err());
  }
}

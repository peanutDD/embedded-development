//! 第2章：所有权系统练习
//!
//! 本文件包含Rust所有权系统的专门练习，包括：
//! - 所有权转移 (Move)
//! - 借用 (Borrowing)
//! - 引用 (References)
//! - 生命周期基础
//! - 切片 (Slices)

#![allow(dead_code, unused_variables)]

fn main() {
  println!("=== 第2章：所有权系统练习 ===");
  println!("运行 cargo test exercise_02 来测试你的答案");
  println!();

  // 示例运行
  let s = String::from("hello");
  println!("字符串长度: {}", string_length(&s));
  println!("字符串仍然可用: {}", s);
}

// ============================================================================
// 练习1: 理解所有权转移
// ============================================================================

/// 练习1: 修复所有权问题
///
/// 下面的代码有所有权问题，请修复它
/// 要求：函数调用后，原始变量仍然可以使用
fn exercise_1() {
  let s1 = String::from("hello");
  let len = string_length(&s1); // 使用借用而不是转移所有权
  println!("字符串 '{}' 的长度是 {}", s1, len); // s1 仍然可用
}

fn string_length(s: &String) -> usize {
  s.len()
}

// ============================================================================
// 练习2: 可变借用
// ============================================================================

/// 练习2: 实现字符串修改函数
///
/// 要求：
/// 1. 函数接受一个可变引用
/// 2. 在字符串末尾添加 "!"
/// 3. 不返回任何值
fn add_exclamation(s: &mut String) {
  s.push('!');
}

/// 练习3: 借用规则
///
/// 修复下面的借用冲突
fn exercise_3() {
  let mut s = String::from("hello");

  // 只能有一个可变引用
  let r1 = &mut s;
  r1.push_str(" world");
  println!("{}", r1);

  // 在r1使用完后，可以创建新的引用
  let r2 = &s;
  println!("{}", r2);
}

// ============================================================================
// 练习4: 切片操作
// ============================================================================

/// 练习4: 实现获取第一个单词的函数
///
/// 要求：
/// 1. 返回字符串中第一个单词的切片
/// 2. 单词以空格分隔
/// 3. 如果没有空格，返回整个字符串
fn first_word(s: &str) -> &str {
  let bytes = s.as_bytes();

  for (i, &item) in bytes.iter().enumerate() {
    if item == b' ' {
      return &s[0..i];
    }
  }

  &s[..]
}

/// 练习5: 数组切片
///
/// 实现一个函数，返回数组中最大值的索引
fn find_max_index(arr: &[i32]) -> Option<usize> {
  if arr.is_empty() {
    return None;
  }

  let mut max_index = 0;
  for (i, &value) in arr.iter().enumerate() {
    if value > arr[max_index] {
      max_index = i;
    }
  }

  Some(max_index)
}

// ============================================================================
// 练习6: 复杂的借用场景
// ============================================================================

/// 练习6: 实现安全的向量操作
///
/// 要求：
/// 1. 函数接受一个向量的可变引用
/// 2. 移除所有偶数
/// 3. 对剩余元素排序
fn filter_and_sort_odds(vec: &mut Vec<i32>) {
  // 保留奇数
  vec.retain(|&x| x % 2 != 0);
  // 排序
  vec.sort();
}

/// 练习7: 字符串分割
///
/// 实现一个函数，将字符串按指定分隔符分割
/// 返回一个包含所有部分的向量
fn split_string(s: &str, delimiter: char) -> Vec<&str> {
  s.split(delimiter).collect()
}

// ============================================================================
// 练习8: 生命周期基础
// ============================================================================

/// 练习8: 比较字符串长度
///
/// 实现一个函数，返回两个字符串中较长的那个
fn longer_string<'a>(s1: &'a str, s2: &'a str) -> &'a str {
  if s1.len() > s2.len() {
    s1
  } else {
    s2
  }
}

/// 练习9: 结构体中的引用
///
/// 定义一个包含字符串引用的结构体
#[derive(Debug)]
struct TextHolder<'a> {
  text: &'a str,
}

impl<'a> TextHolder<'a> {
  fn new(text: &'a str) -> Self {
    TextHolder { text }
  }

  fn get_text(&self) -> &str {
    self.text
  }

  fn get_length(&self) -> usize {
    self.text.len()
  }
}

// ============================================================================
// 练习10: 高级所有权模式
// ============================================================================

/// 练习10: 实现一个简单的缓存
///
/// 要求：
/// 1. 存储字符串和对应的长度
/// 2. 避免重复计算
use std::collections::HashMap;

struct StringLengthCache {
  cache: HashMap<String, usize>,
}

impl StringLengthCache {
  fn new() -> Self {
    StringLengthCache {
      cache: HashMap::new(),
    }
  }

  fn get_length(&mut self, s: &str) -> usize {
    if let Some(&length) = self.cache.get(s) {
      length
    } else {
      let length = s.len();
      self.cache.insert(s.to_string(), length);
      length
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
  fn test_string_length() {
    let s = String::from("hello world");
    assert_eq!(string_length(&s), 11);
    // s 仍然可用
    assert_eq!(s, "hello world");
  }

  #[test]
  fn test_add_exclamation() {
    let mut s = String::from("hello");
    add_exclamation(&mut s);
    assert_eq!(s, "hello!");
  }

  #[test]
  fn test_first_word() {
    assert_eq!(first_word("hello world"), "hello");
    assert_eq!(first_word("hello"), "hello");
    assert_eq!(first_word(""), "");
    assert_eq!(first_word("rust programming"), "rust");
  }

  #[test]
  fn test_find_max_index() {
    assert_eq!(find_max_index(&[1, 3, 2, 5, 4]), Some(3));
    assert_eq!(find_max_index(&[5]), Some(0));
    assert_eq!(find_max_index(&[]), None);
    assert_eq!(find_max_index(&[1, 1, 1]), Some(0));
  }

  #[test]
  fn test_filter_and_sort_odds() {
    let mut vec = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
    filter_and_sort_odds(&mut vec);
    assert_eq!(vec, vec![1, 3, 5, 7, 9]);

    let mut vec2 = vec![9, 1, 7, 3, 5];
    filter_and_sort_odds(&mut vec2);
    assert_eq!(vec2, vec![1, 3, 5, 7, 9]);
  }

  #[test]
  fn test_split_string() {
    let result = split_string("a,b,c,d", ',');
    assert_eq!(result, vec!["a", "b", "c", "d"]);

    let result2 = split_string("hello world", ' ');
    assert_eq!(result2, vec!["hello", "world"]);
  }

  #[test]
  fn test_longer_string() {
    assert_eq!(longer_string("hello", "world"), "hello");
    assert_eq!(longer_string("hi", "hello"), "hello");
    assert_eq!(longer_string("same", "size"), "same"); // 相等时返回第一个
  }

  #[test]
  fn test_text_holder() {
    let text = "Hello, Rust!";
    let holder = TextHolder::new(text);
    assert_eq!(holder.get_text(), "Hello, Rust!");
    assert_eq!(holder.get_length(), 12);
  }

  #[test]
  fn test_string_length_cache() {
    let mut cache = StringLengthCache::new();

    // 第一次计算
    assert_eq!(cache.get_length("hello"), 5);

    // 第二次应该从缓存获取
    assert_eq!(cache.get_length("hello"), 5);

    // 不同字符串
    assert_eq!(cache.get_length("world"), 5);
    assert_eq!(cache.get_length("rust"), 4);
  }
}

// ============================================================================
// 挑战练习
// ============================================================================

/// 挑战1: 实现一个安全的字符串构建器
///
/// 要求：
/// 1. 可以添加字符串片段
/// 2. 可以获取当前内容的引用
/// 3. 可以清空内容
/// 4. 线程安全不是必需的
pub struct StringBuilder {
  content: String,
}

impl StringBuilder {
  pub fn new() -> Self {
    StringBuilder {
      content: String::new(),
    }
  }

  pub fn append(&mut self, s: &str) -> &mut Self {
    self.content.push_str(s);
    self
  }

  pub fn append_char(&mut self, c: char) -> &mut Self {
    self.content.push(c);
    self
  }

  pub fn as_str(&self) -> &str {
    &self.content
  }

  pub fn clear(&mut self) -> &mut Self {
    self.content.clear();
    self
  }

  pub fn len(&self) -> usize {
    self.content.len()
  }

  pub fn is_empty(&self) -> bool {
    self.content.is_empty()
  }
}

/// 挑战2: 实现一个引用计数的不可变字符串
///
/// 类似于Rc<str>，但是自己实现
use std::rc::Rc;

#[derive(Debug, Clone)]
pub struct SharedString {
  inner: Rc<String>,
}

impl SharedString {
  pub fn new(s: &str) -> Self {
    SharedString {
      inner: Rc::new(s.to_string()),
    }
  }

  pub fn as_str(&self) -> &str {
    &self.inner
  }

  pub fn len(&self) -> usize {
    self.inner.len()
  }

  pub fn strong_count(&self) -> usize {
    Rc::strong_count(&self.inner)
  }
}

#[cfg(test)]
mod challenge_tests {
  use super::*;

  #[test]
  fn test_string_builder() {
    let mut builder = StringBuilder::new();

    builder
      .append("Hello")
      .append_char(' ')
      .append("World")
      .append_char('!');

    assert_eq!(builder.as_str(), "Hello World!");
    assert_eq!(builder.len(), 12);
    assert!(!builder.is_empty());

    builder.clear();
    assert_eq!(builder.as_str(), "");
    assert_eq!(builder.len(), 0);
    assert!(builder.is_empty());
  }

  #[test]
  fn test_shared_string() {
    let s1 = SharedString::new("Hello, Rust!");
    let s2 = s1.clone();
    let s3 = s1.clone();

    assert_eq!(s1.as_str(), "Hello, Rust!");
    assert_eq!(s2.as_str(), "Hello, Rust!");
    assert_eq!(s3.as_str(), "Hello, Rust!");

    assert_eq!(s1.len(), 13);
    assert_eq!(s1.strong_count(), 3);
    assert_eq!(s2.strong_count(), 3);
    assert_eq!(s3.strong_count(), 3);
  }
}

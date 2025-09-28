//! 所有权系统工具和示例
//!
//! 本模块提供了Rust所有权系统的实用工具，特别适用于嵌入式开发中的内存管理。

use crate::error_handling::{Error, Result};

/// 安全的缓冲区，演示所有权和借用
pub struct SafeBuffer {
  data: Vec<u8>,
  capacity: usize,
}

impl SafeBuffer {
  /// 创建新的安全缓冲区
  pub fn new(capacity: usize) -> Result<Self> {
    if capacity == 0 {
      return Err(Error::InvalidInput("容量不能为0".to_string()));
    }

    Ok(SafeBuffer {
      data: Vec::with_capacity(capacity),
      capacity,
    })
  }

  /// 获取缓冲区长度
  pub fn len(&self) -> usize {
    self.data.len()
  }

  /// 检查缓冲区是否为空
  pub fn is_empty(&self) -> bool {
    self.data.is_empty()
  }

  /// 获取容量
  pub fn capacity(&self) -> usize {
    self.capacity
  }

  /// 写入数据（移动语义）
  pub fn write(&mut self, data: Vec<u8>) -> Result<()> {
    if data.len() > self.capacity {
      return Err(Error::BufferOverflow);
    }
    self.data = data;
    Ok(())
  }

  /// 追加数据（借用语义）
  pub fn append(&mut self, data: &[u8]) -> Result<()> {
    if self.data.len() + data.len() > self.capacity {
      return Err(Error::BufferOverflow);
    }
    self.data.extend_from_slice(data);
    Ok(())
  }

  /// 读取数据（不可变借用）
  pub fn read(&self) -> &[u8] {
    &self.data
  }

  /// 获取可变切片（可变借用）
  pub fn as_mut_slice(&mut self) -> &mut [u8] {
    &mut self.data
  }

  /// 清空缓冲区
  pub fn clear(&mut self) {
    self.data.clear();
  }

  /// 消费缓冲区，返回内部数据
  pub fn into_inner(self) -> Vec<u8> {
    self.data
  }
}

/// 生命周期演示：字符串引用管理器
pub struct StringRefManager<'a> {
  refs: Vec<&'a str>,
}

impl<'a> StringRefManager<'a> {
  /// 创建新的引用管理器
  pub fn new() -> Self {
    StringRefManager { refs: Vec::new() }
  }

  /// 添加字符串引用
  pub fn add_ref(&mut self, s: &'a str) {
    self.refs.push(s);
  }

  /// 获取所有引用
  pub fn get_refs(&self) -> &[&'a str] {
    &self.refs
  }

  /// 查找最长的字符串
  pub fn find_longest(&self) -> Option<&'a str> {
    self.refs.iter().max_by_key(|s| s.len()).copied()
  }
}

impl<'a> Default for StringRefManager<'a> {
  fn default() -> Self {
    Self::new()
  }
}

#[cfg(feature = "std")]
use std::cell::RefCell;
/// 智能指针示例：引用计数的缓冲区
#[cfg(feature = "std")]
use std::rc::Rc;

#[cfg(feature = "std")]
pub type SharedBuffer = Rc<RefCell<SafeBuffer>>;

#[cfg(feature = "std")]
impl SafeBuffer {
  /// 创建共享缓冲区
  pub fn new_shared(capacity: usize) -> Result<SharedBuffer> {
    let buffer = Self::new(capacity)?;
    Ok(Rc::new(RefCell::new(buffer)))
  }
}

/// 移动语义演示
pub fn demonstrate_move() -> Result<Vec<u8>> {
  let mut buffer = SafeBuffer::new(1024)?;
  buffer.append(b"Hello, World!")?;

  // 移动缓冲区的所有权
  let data = buffer.into_inner();

  // buffer 在这里已经不可用了
  Ok(data)
}

/// 借用语义演示
pub fn demonstrate_borrow(buffer: &mut SafeBuffer) -> Result<usize> {
  buffer.append(b"Additional data")?;
  Ok(buffer.len())
}

/// 生命周期演示
pub fn demonstrate_lifetime<'a>(s1: &'a str, s2: &'a str) -> &'a str {
  if s1.len() > s2.len() {
    s1
  } else {
    s2
  }
}

/// 零拷贝字符串处理
pub fn zero_copy_string_processing(input: &str) -> Vec<&str> {
  input
    .split_whitespace()
    .filter(|word| word.len() > 3)
    .collect()
}

/// 内存池分配器（嵌入式友好）
#[cfg(feature = "embedded")]
pub struct MemoryPool<const N: usize> {
  pool: [u8; N],
  allocated: [bool; N],
}

#[cfg(feature = "embedded")]
impl<const N: usize> MemoryPool<N> {
  /// 创建新的内存池
  pub const fn new() -> Self {
    MemoryPool {
      pool: [0; N],
      allocated: [false; N],
    }
  }

  /// 分配内存块
  pub fn allocate(&mut self, size: usize) -> Option<&mut [u8]> {
    if size == 0 || size > N {
      return None;
    }

    // 简单的线性搜索分配算法
    for i in 0..=(N - size) {
      if self.allocated[i..i + size]
        .iter()
        .all(|&allocated| !allocated)
      {
        // 标记为已分配
        for j in i..i + size {
          self.allocated[j] = true;
        }
        return Some(&mut self.pool[i..i + size]);
      }
    }

    None
  }

  /// 释放内存块
  pub fn deallocate(&mut self, ptr: *mut u8, size: usize) {
    let pool_start = self.pool.as_ptr() as usize;
    let ptr_addr = ptr as usize;

    if ptr_addr >= pool_start && ptr_addr < pool_start + N {
      let offset = ptr_addr - pool_start;
      if offset + size <= N {
        for i in offset..offset + size {
          self.allocated[i] = false;
        }
      }
    }
  }

  /// 获取可用内存大小
  pub fn available_memory(&self) -> usize {
    self
      .allocated
      .iter()
      .filter(|&&allocated| !allocated)
      .count()
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_safe_buffer() {
    let mut buffer = SafeBuffer::new(100).unwrap();
    assert_eq!(buffer.len(), 0);
    assert!(buffer.is_empty());

    buffer.append(b"Hello").unwrap();
    assert_eq!(buffer.len(), 5);
    assert!(!buffer.is_empty());

    let data = buffer.read();
    assert_eq!(data, b"Hello");
  }

  #[test]
  fn test_buffer_overflow() {
    let mut buffer = SafeBuffer::new(5).unwrap();
    let result = buffer.append(b"Hello, World!");
    assert!(result.is_err());
  }

  #[test]
  fn test_string_ref_manager() {
    let mut manager = StringRefManager::new();
    manager.add_ref("hello");
    manager.add_ref("world");
    manager.add_ref("rust");

    let longest = manager.find_longest().unwrap();
    assert_eq!(longest, "hello");
  }

  #[test]
  fn test_demonstrate_lifetime() {
    let s1 = "short";
    let s2 = "longer string";
    let result = demonstrate_lifetime(s1, s2);
    assert_eq!(result, s2);
  }

  #[test]
  fn test_zero_copy_processing() {
    let input = "this is a test string with some words";
    let words = zero_copy_string_processing(input);
    assert_eq!(
      words,
      vec!["this", "test", "string", "with", "some", "words"]
    );
  }

  #[cfg(feature = "embedded")]
  #[test]
  fn test_memory_pool() {
    let mut pool = MemoryPool::<1024>::new();

    let block1 = pool.allocate(100).unwrap();
    assert_eq!(block1.len(), 100);

    let block2 = pool.allocate(200).unwrap();
    assert_eq!(block2.len(), 200);

    assert_eq!(pool.available_memory(), 1024 - 300);
  }

  #[cfg(feature = "std")]
  #[test]
  fn test_shared_buffer() {
    let shared = SafeBuffer::new_shared(100).unwrap();

    {
      let mut buffer = shared.borrow_mut();
      buffer.append(b"Hello").unwrap();
    }

    {
      let buffer = shared.borrow();
      assert_eq!(buffer.read(), b"Hello");
    }
  }
}

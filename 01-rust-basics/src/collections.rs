//! # 高效集合操作
//!
//! 本模块提供了针对嵌入式环境优化的集合操作工具。
//!
//! ## 特性
//!
//! - **无堆分配**: 使用栈分配的固定大小集合
//! - **零成本抽象**: 编译时优化的迭代器
//! - **内存安全**: 边界检查和类型安全
//! - **性能可预测**: 常数时间复杂度操作

use core::mem::MaybeUninit;
use core::ptr;

/// 固定大小的栈分配向量
pub struct StackVec<T, const N: usize> {
  data: [MaybeUninit<T>; N],
  len: usize,
}

impl<T, const N: usize> StackVec<T, N> {
  /// 创建新的空向量
  pub const fn new() -> Self {
    Self {
      data: unsafe { MaybeUninit::uninit().assume_init() },
      len: 0,
    }
  }

  /// 获取向量长度
  pub const fn len(&self) -> usize {
    self.len
  }

  /// 检查向量是否为空
  pub const fn is_empty(&self) -> bool {
    self.len == 0
  }

  /// 获取向量容量
  pub const fn capacity(&self) -> usize {
    N
  }

  /// 向向量末尾添加元素
  pub fn push(&mut self, value: T) -> Result<(), T> {
    if self.len >= N {
      return Err(value);
    }

    self.data[self.len] = MaybeUninit::new(value);
    self.len += 1;
    Ok(())
  }

  /// 从向量末尾移除元素
  pub fn pop(&mut self) -> Option<T> {
    if self.len == 0 {
      return None;
    }

    self.len -= 1;
    Some(unsafe { self.data[self.len].assume_init_read() })
  }

  /// 获取指定索引的元素引用
  pub fn get(&self, index: usize) -> Option<&T> {
    if index < self.len {
      Some(unsafe { self.data[index].assume_init_ref() })
    } else {
      None
    }
  }

  /// 获取指定索引的可变元素引用
  pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
    if index < self.len {
      Some(unsafe { self.data[index].assume_init_mut() })
    } else {
      None
    }
  }

  /// 清空向量
  pub fn clear(&mut self) {
    while self.pop().is_some() {}
  }

  /// 获取迭代器
  pub fn iter(&self) -> StackVecIter<T> {
    StackVecIter {
      data: &self.data[..self.len],
      index: 0,
    }
  }
}

impl<T, const N: usize> Drop for StackVec<T, N> {
  fn drop(&mut self) {
    self.clear();
  }
}

/// 栈向量迭代器
pub struct StackVecIter<'a, T> {
  data: &'a [MaybeUninit<T>],
  index: usize,
}

impl<'a, T> Iterator for StackVecIter<'a, T> {
  type Item = &'a T;

  fn next(&mut self) -> Option<Self::Item> {
    if self.index < self.data.len() {
      let item = unsafe { self.data[self.index].assume_init_ref() };
      self.index += 1;
      Some(item)
    } else {
      None
    }
  }
}

/// 固定大小的环形缓冲区
pub struct RingBuffer<T, const N: usize> {
  data: [MaybeUninit<T>; N],
  head: usize,
  tail: usize,
  full: bool,
}

impl<T, const N: usize> RingBuffer<T, N> {
  /// 创建新的环形缓冲区
  pub const fn new() -> Self {
    Self {
      data: unsafe { MaybeUninit::uninit().assume_init() },
      head: 0,
      tail: 0,
      full: false,
    }
  }

  /// 检查缓冲区是否为空
  pub const fn is_empty(&self) -> bool {
    !self.full && self.head == self.tail
  }

  /// 检查缓冲区是否已满
  pub const fn is_full(&self) -> bool {
    self.full
  }

  /// 获取缓冲区长度
  pub const fn len(&self) -> usize {
    if self.full {
      N
    } else if self.head >= self.tail {
      self.head - self.tail
    } else {
      N - self.tail + self.head
    }
  }

  /// 向缓冲区添加元素
  pub fn push(&mut self, value: T) -> Option<T> {
    let old_value = if self.full {
      Some(unsafe { self.data[self.head].assume_init_read() })
    } else {
      None
    };

    self.data[self.head] = MaybeUninit::new(value);
    self.head = (self.head + 1) % N;

    if self.head == self.tail {
      self.full = true;
      if old_value.is_some() {
        self.tail = (self.tail + 1) % N;
      }
    }

    old_value
  }

  /// 从缓冲区移除元素
  pub fn pop(&mut self) -> Option<T> {
    if self.is_empty() {
      return None;
    }

    let value = unsafe { self.data[self.tail].assume_init_read() };
    self.tail = (self.tail + 1) % N;
    self.full = false;

    Some(value)
  }
}

impl<T, const N: usize> Drop for RingBuffer<T, N> {
  fn drop(&mut self) {
    while self.pop().is_some() {}
  }
}

/// 集合操作工具函数
pub mod utils {
  use super::*;

  /// 在切片中查找元素
  pub fn find<T: PartialEq>(slice: &[T], target: &T) -> Option<usize> {
    slice.iter().position(|x| x == target)
  }

  /// 对切片进行二分查找
  pub fn binary_search<T: Ord>(slice: &[T], target: &T) -> Result<usize, usize> {
    slice.binary_search(target)
  }

  /// 计算切片的最大值
  pub fn max<T: Ord + Copy>(slice: &[T]) -> Option<T> {
    slice.iter().max().copied()
  }

  /// 计算切片的最小值
  pub fn min<T: Ord + Copy>(slice: &[T]) -> Option<T> {
    slice.iter().min().copied()
  }

  /// 对切片进行分区
  pub fn partition<T, F>(slice: &mut [T], mut predicate: F) -> usize
  where
    F: FnMut(&T) -> bool,
  {
    let mut left = 0;
    let mut right = slice.len();

    while left < right {
      if predicate(&slice[left]) {
        left += 1;
      } else {
        right -= 1;
        slice.swap(left, right);
      }
    }

    left
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_stack_vec() {
    let mut vec: StackVec<i32, 4> = StackVec::new();

    assert!(vec.is_empty());
    assert_eq!(vec.len(), 0);
    assert_eq!(vec.capacity(), 4);

    vec.push(1).unwrap();
    vec.push(2).unwrap();
    vec.push(3).unwrap();

    assert_eq!(vec.len(), 3);
    assert_eq!(vec.get(1), Some(&2));

    assert_eq!(vec.pop(), Some(3));
    assert_eq!(vec.len(), 2);
  }

  #[test]
  fn test_ring_buffer() {
    let mut buffer: RingBuffer<i32, 3> = RingBuffer::new();

    assert!(buffer.is_empty());
    assert!(!buffer.is_full());

    buffer.push(1);
    buffer.push(2);
    buffer.push(3);

    assert!(buffer.is_full());
    assert_eq!(buffer.len(), 3);

    // 覆盖最旧的元素
    let old = buffer.push(4);
    assert_eq!(old, Some(1));

    assert_eq!(buffer.pop(), Some(2));
    assert_eq!(buffer.pop(), Some(3));
    assert_eq!(buffer.pop(), Some(4));
    assert!(buffer.is_empty());
  }

  #[test]
  fn test_utils() {
    let data = [1, 3, 5, 7, 9];

    assert_eq!(utils::find(&data, &5), Some(2));
    assert_eq!(utils::find(&data, &6), None);

    assert_eq!(utils::binary_search(&data, &5), Ok(2));
    assert_eq!(utils::binary_search(&data, &6), Err(3));

    assert_eq!(utils::max(&data), Some(9));
    assert_eq!(utils::min(&data), Some(1));

    let mut data = [1, 4, 2, 5, 3];
    let pivot = utils::partition(&mut data, |&x| x < 4);
    assert_eq!(pivot, 3);
  }
}

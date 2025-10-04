//! 性能基准测试
//!
//! 本文件包含各种Rust性能基准测试，用于：
//! - 测试不同算法的性能
//! - 比较不同数据结构的效率
//! - 验证优化效果
//! - 学习性能分析技巧

#![allow(dead_code, unused_variables)]

use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::time::{Duration, Instant};

fn main() {
  println!("=== 性能基准测试 ===");
  println!("运行 cargo test --release benchmarks 来执行基准测试");
  println!();

  // 运行一些基准测试示例
  run_sorting_benchmarks();
  run_collection_benchmarks();
}

// ============================================================================
// 排序算法基准测试
// ============================================================================

/// 冒泡排序
fn bubble_sort<T: PartialOrd + Clone>(arr: &mut [T]) {
  let len = arr.len();
  for i in 0..len {
    for j in 0..len - 1 - i {
      if arr[j] > arr[j + 1] {
        arr.swap(j, j + 1);
      }
    }
  }
}

/// 选择排序
fn selection_sort<T: PartialOrd + Clone>(arr: &mut [T]) {
  let len = arr.len();
  for i in 0..len {
    let mut min_idx = i;
    for j in i + 1..len {
      if arr[j] < arr[min_idx] {
        min_idx = j;
      }
    }
    arr.swap(i, min_idx);
  }
}

/// 插入排序
fn insertion_sort<T: PartialOrd + Clone>(arr: &mut [T]) {
  for i in 1..arr.len() {
    let mut j = i;
    while j > 0 && arr[j - 1] > arr[j] {
      arr.swap(j - 1, j);
      j -= 1;
    }
  }
}

/// 快速排序
fn quick_sort<T: PartialOrd + Clone>(arr: &mut [T]) {
  if arr.len() <= 1 {
    return;
  }

  let pivot_index = partition(arr);
  let (left, right) = arr.split_at_mut(pivot_index);
  quick_sort(left);
  quick_sort(&mut right[1..]);
}

fn partition<T: PartialOrd + Clone>(arr: &mut [T]) -> usize {
  let len = arr.len();
  let pivot_index = len - 1;
  let mut i = 0;

  for j in 0..pivot_index {
    if arr[j] <= arr[pivot_index] {
      arr.swap(i, j);
      i += 1;
    }
  }

  arr.swap(i, pivot_index);
  i
}

/// 归并排序
fn merge_sort<T: PartialOrd + Clone + Copy>(arr: &mut [T]) {
    let len = arr.len();
    if len <= 1 {
        return;
    }
    
    let mid = len / 2;
    merge_sort(&mut arr[..mid]);
    merge_sort(&mut arr[mid..]);
    
    let mut temp = arr.to_vec();
    merge(&arr[..mid], &arr[mid..], &mut temp);
    arr.copy_from_slice(&temp);
}

fn merge<T: PartialOrd + Clone>(left: &[T], right: &[T], result: &mut [T]) {
  let mut i = 0;
  let mut j = 0;
  let mut k = 0;

  while i < left.len() && j < right.len() {
    if left[i] <= right[j] {
      result[k] = left[i].clone();
      i += 1;
    } else {
      result[k] = right[j].clone();
      j += 1;
    }
    k += 1;
  }

  while i < left.len() {
    result[k] = left[i].clone();
    i += 1;
    k += 1;
  }

  while j < right.len() {
    result[k] = right[j].clone();
    j += 1;
    k += 1;
  }
}

/// 基准测试函数
fn benchmark_sort<F>(name: &str, sort_fn: F, data: &[i32]) -> Duration
where
  F: Fn(&mut [i32]),
{
  let mut test_data = data.to_vec();
  let start = Instant::now();
  sort_fn(&mut test_data);
  let duration = start.elapsed();

  // 验证排序结果
  assert!(
    test_data.windows(2).all(|w| w[0] <= w[1]),
    "排序失败: {}",
    name
  );

  duration
}

/// 运行排序基准测试
fn run_sorting_benchmarks() {
  println!("=== 排序算法基准测试 ===");

  // 生成测试数据
  let sizes = vec![100, 1000, 5000];

  for size in sizes {
    println!("\n数据规模: {}", size);

    // 随机数据
        let mut random_data: Vec<i32> = (0..size).map(|i| (i % 1000) as i32).collect();

    // 已排序数据
    let mut sorted_data: Vec<i32> = (0..size).collect();

    // 逆序数据
    let mut reverse_data: Vec<i32> = (0..size).rev().collect();

    println!("随机数据:");
    println!(
      "  冒泡排序: {:?}",
      benchmark_sort("bubble_sort", bubble_sort, &random_data)
    );
    println!(
      "  选择排序: {:?}",
      benchmark_sort("selection_sort", selection_sort, &random_data)
    );
    println!(
      "  插入排序: {:?}",
      benchmark_sort("insertion_sort", insertion_sort, &random_data)
    );
    println!(
      "  快速排序: {:?}",
      benchmark_sort("quick_sort", quick_sort, &random_data)
    );
    println!(
      "  归并排序: {:?}",
      benchmark_sort("merge_sort", merge_sort, &random_data)
    );
    println!(
      "  标准排序: {:?}",
      benchmark_sort("std_sort", |arr| arr.sort(), &random_data)
    );
  }
}

// ============================================================================
// 集合类型基准测试
// ============================================================================

/// 基准测试结果
#[derive(Debug)]
struct BenchmarkResult {
  name: String,
  insert_time: Duration,
  lookup_time: Duration,
  remove_time: Duration,
}

/// Vec基准测试
fn benchmark_vec(size: usize) -> BenchmarkResult {
  let mut vec = Vec::new();

  // 插入测试
  let start = Instant::now();
  for i in 0..size {
    vec.push(i);
  }
  let insert_time = start.elapsed();

  // 查找测试
  let start = Instant::now();
  for i in 0..size {
    let _ = vec.iter().find(|&&x| x == i);
  }
  let lookup_time = start.elapsed();

  // 删除测试（从末尾删除）
  let start = Instant::now();
  for _ in 0..size {
    vec.pop();
  }
  let remove_time = start.elapsed();

  BenchmarkResult {
    name: "Vec".to_string(),
    insert_time,
    lookup_time,
    remove_time,
  }
}

/// HashMap基准测试
fn benchmark_hashmap(size: usize) -> BenchmarkResult {
  let mut map = HashMap::new();

  // 插入测试
  let start = Instant::now();
  for i in 0..size {
    map.insert(i, i);
  }
  let insert_time = start.elapsed();

  // 查找测试
  let start = Instant::now();
  for i in 0..size {
    let _ = map.get(&i);
  }
  let lookup_time = start.elapsed();

  // 删除测试
  let start = Instant::now();
  for i in 0..size {
    map.remove(&i);
  }
  let remove_time = start.elapsed();

  BenchmarkResult {
    name: "HashMap".to_string(),
    insert_time,
    lookup_time,
    remove_time,
  }
}

/// BTreeMap基准测试
fn benchmark_btreemap(size: usize) -> BenchmarkResult {
  let mut map = BTreeMap::new();

  // 插入测试
  let start = Instant::now();
  for i in 0..size {
    map.insert(i, i);
  }
  let insert_time = start.elapsed();

  // 查找测试
  let start = Instant::now();
  for i in 0..size {
    let _ = map.get(&i);
  }
  let lookup_time = start.elapsed();

  // 删除测试
  let start = Instant::now();
  for i in 0..size {
    map.remove(&i);
  }
  let remove_time = start.elapsed();

  BenchmarkResult {
    name: "BTreeMap".to_string(),
    insert_time,
    lookup_time,
    remove_time,
  }
}

/// HashSet基准测试
fn benchmark_hashset(size: usize) -> BenchmarkResult {
  let mut set = HashSet::new();

  // 插入测试
  let start = Instant::now();
  for i in 0..size {
    set.insert(i);
  }
  let insert_time = start.elapsed();

  // 查找测试
  let start = Instant::now();
  for i in 0..size {
    let _ = set.contains(&i);
  }
  let lookup_time = start.elapsed();

  // 删除测试
  let start = Instant::now();
  for i in 0..size {
    set.remove(&i);
  }
  let remove_time = start.elapsed();

  BenchmarkResult {
    name: "HashSet".to_string(),
    insert_time,
    lookup_time,
    remove_time,
  }
}

/// BTreeSet基准测试
fn benchmark_btreeset(size: usize) -> BenchmarkResult {
  let mut set = BTreeSet::new();

  // 插入测试
  let start = Instant::now();
  for i in 0..size {
    set.insert(i);
  }
  let insert_time = start.elapsed();

  // 查找测试
  let start = Instant::now();
  for i in 0..size {
    let _ = set.contains(&i);
  }
  let lookup_time = start.elapsed();

  // 删除测试
  let start = Instant::now();
  for i in 0..size {
    set.remove(&i);
  }
  let remove_time = start.elapsed();

  BenchmarkResult {
    name: "BTreeSet".to_string(),
    insert_time,
    lookup_time,
    remove_time,
  }
}

/// VecDeque基准测试
fn benchmark_vecdeque(size: usize) -> BenchmarkResult {
  let mut deque = VecDeque::new();

  // 插入测试（从两端插入）
  let start = Instant::now();
  for i in 0..size {
    if i % 2 == 0 {
      deque.push_back(i);
    } else {
      deque.push_front(i);
    }
  }
  let insert_time = start.elapsed();

  // 查找测试
  let start = Instant::now();
  for i in 0..size {
    let _ = deque.iter().find(|&&x| x == i);
  }
  let lookup_time = start.elapsed();

  // 删除测试（从两端删除）
  let start = Instant::now();
  for i in 0..size {
    if i % 2 == 0 {
      deque.pop_back();
    } else {
      deque.pop_front();
    }
  }
  let remove_time = start.elapsed();

  BenchmarkResult {
    name: "VecDeque".to_string(),
    insert_time,
    lookup_time,
    remove_time,
  }
}

/// 运行集合基准测试
fn run_collection_benchmarks() {
  println!("\n=== 集合类型基准测试 ===");

  let sizes = vec![1000, 10000];

  for size in sizes {
    println!("\n数据规模: {}", size);

    let results = vec![
      benchmark_vec(size),
      benchmark_hashmap(size),
      benchmark_btreemap(size),
      benchmark_hashset(size),
      benchmark_btreeset(size),
      benchmark_vecdeque(size),
    ];

    println!(
      "{:<12} {:<12} {:<12} {:<12}",
      "类型", "插入时间", "查找时间", "删除时间"
    );
    println!("{}", "-".repeat(50));

    for result in results {
      println!(
        "{:<12} {:<12?} {:<12?} {:<12?}",
        result.name, result.insert_time, result.lookup_time, result.remove_time
      );
    }
  }
}

// ============================================================================
// 内存分配基准测试
// ============================================================================

/// 测试不同的内存分配模式
fn benchmark_memory_allocation() {
  println!("\n=== 内存分配基准测试 ===");

  let size = 10000;

  // Vec预分配 vs 动态增长
  let start = Instant::now();
  let mut vec_dynamic = Vec::new();
  for i in 0..size {
    vec_dynamic.push(i);
  }
  let dynamic_time = start.elapsed();

  let start = Instant::now();
  let mut vec_preallocated = Vec::with_capacity(size);
  for i in 0..size {
    vec_preallocated.push(i);
  }
  let preallocated_time = start.elapsed();

  println!("Vec动态增长: {:?}", dynamic_time);
  println!("Vec预分配: {:?}", preallocated_time);
  println!(
    "性能提升: {:.2}x",
    dynamic_time.as_nanos() as f64 / preallocated_time.as_nanos() as f64
  );

  // 字符串拼接测试
  let strings = vec!["hello", "world", "rust", "programming"];

  // 使用 + 操作符
  let start = Instant::now();
  let mut result = String::new();
  for s in &strings {
    result = result + s;
  }
  let concat_time = start.elapsed();

  // 使用 push_str
  let start = Instant::now();
  let mut result = String::new();
  for s in &strings {
    result.push_str(s);
  }
  let push_str_time = start.elapsed();

  // 使用 format!
  let start = Instant::now();
  let result = format!("{}{}{}{}", strings[0], strings[1], strings[2], strings[3]);
  let format_time = start.elapsed();

  println!("\n字符串拼接:");
  println!("+ 操作符: {:?}", concat_time);
  println!("push_str: {:?}", push_str_time);
  println!("format!: {:?}", format_time);
}

// ============================================================================
// 算法复杂度验证
// ============================================================================

/// 验证算法的时间复杂度
fn verify_time_complexity() {
  println!("\n=== 算法复杂度验证 ===");

  let sizes = vec![100, 200, 400, 800, 1600];

  println!("线性搜索 O(n):");
  for size in &sizes {
    let data: Vec<i32> = (0..*size).collect();
    let target = size - 1;

    let start = Instant::now();
        let _ = data.iter().find(|&&x| x == target);
        let duration = start.elapsed();

    println!("  大小 {}: {:?}", size, duration);
  }

  println!("\n二分搜索 O(log n):");
  for size in &sizes {
    let data: Vec<i32> = (0..*size).collect();
    let target = size - 1;

    let start = Instant::now();
        let _ = data.binary_search(&target);
        let duration = start.elapsed();

    println!("  大小 {}: {:?}", size, duration);
  }
}

// ============================================================================
// 并发性能测试
// ============================================================================

/// 并发性能测试
fn benchmark_concurrency() {
  use std::sync::{Arc, Mutex};
  use std::thread;

  println!("\n=== 并发性能测试 ===");

  let data = Arc::new(Mutex::new(0));
  let num_threads = 4;
  let operations_per_thread = 10000;

  // 单线程基准
  let start = Instant::now();
  let mut counter = 0;
  for _ in 0..(num_threads * operations_per_thread) {
    counter += 1;
  }
  let single_thread_time = start.elapsed();

  // 多线程测试
  let start = Instant::now();
  let handles: Vec<_> = (0..num_threads)
    .map(|_| {
      let data = Arc::clone(&data);
      thread::spawn(move || {
        for _ in 0..operations_per_thread {
          let mut num = data.lock().unwrap();
          *num += 1;
        }
      })
    })
    .collect();

  for handle in handles {
    handle.join().unwrap();
  }
  let multi_thread_time = start.elapsed();

  println!("单线程时间: {:?}", single_thread_time);
  println!("多线程时间: {:?}", multi_thread_time);
  println!(
    "并发效率: {:.2}%",
    (single_thread_time.as_nanos() as f64 / multi_thread_time.as_nanos() as f64) * 100.0
  );
}

// ============================================================================
// 测试用例
// ============================================================================

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_sorting_algorithms() {
    let mut data = vec![64, 34, 25, 12, 22, 11, 90];
    let expected = vec![11, 12, 22, 25, 34, 64, 90];

    let mut test_data = data.clone();
    bubble_sort(&mut test_data);
    assert_eq!(test_data, expected);

    let mut test_data = data.clone();
    selection_sort(&mut test_data);
    assert_eq!(test_data, expected);

    let mut test_data = data.clone();
    insertion_sort(&mut test_data);
    assert_eq!(test_data, expected);

    let mut test_data = data.clone();
    quick_sort(&mut test_data);
    assert_eq!(test_data, expected);

    let mut test_data = data.clone();
    merge_sort(&mut test_data);
    assert_eq!(test_data, expected);
  }

  #[test]
  fn benchmark_small_collections() {
    let size = 100;

    let vec_result = benchmark_vec(size);
    let hashmap_result = benchmark_hashmap(size);
    let btreemap_result = benchmark_btreemap(size);

    println!("Vec插入时间: {:?}", vec_result.insert_time);
    println!("HashMap插入时间: {:?}", hashmap_result.insert_time);
    println!("BTreeMap插入时间: {:?}", btreemap_result.insert_time);

    // 对于小规模数据，HashMap通常比BTreeMap快
    // 但这个测试主要是为了验证基准测试函数正常工作
    assert!(vec_result.insert_time > Duration::from_nanos(0));
    assert!(hashmap_result.insert_time > Duration::from_nanos(0));
    assert!(btreemap_result.insert_time > Duration::from_nanos(0));
  }

  #[test]
  fn test_memory_allocation_patterns() {
    let size = 1000;

    // 测试预分配的优势
    let start = Instant::now();
    let mut vec_dynamic = Vec::new();
    for i in 0..size {
      vec_dynamic.push(i);
    }
    let dynamic_time = start.elapsed();

    let start = Instant::now();
    let mut vec_preallocated = Vec::with_capacity(size);
    for i in 0..size {
      vec_preallocated.push(i);
    }
    let preallocated_time = start.elapsed();

    println!("动态分配: {:?}", dynamic_time);
    println!("预分配: {:?}", preallocated_time);

    // 预分配通常更快（但在小规模测试中差异可能不明显）
    assert_eq!(vec_dynamic.len(), size);
    assert_eq!(vec_preallocated.len(), size);
  }
}

// ============================================================================
// 实用工具函数
// ============================================================================

/// 生成随机数据
mod rand {
  use std::collections::hash_map::DefaultHasher;
  use std::hash::{Hash, Hasher};

  static mut SEED: u64 = 1;

  pub fn random<T>() -> T
  where
    T: From<u64>,
  {
    unsafe {
      SEED = SEED.wrapping_mul(1103515245).wrapping_add(12345);
      T::from(SEED)
    }
  }

  pub fn set_seed(seed: u64) {
    unsafe {
      SEED = seed;
    }
  }
}

/// 性能分析器
struct Profiler {
  name: String,
  start_time: Instant,
}

impl Profiler {
  fn new(name: &str) -> Self {
    Profiler {
      name: name.to_string(),
      start_time: Instant::now(),
    }
  }

  fn elapsed(&self) -> Duration {
    self.start_time.elapsed()
  }
}

impl Drop for Profiler {
  fn drop(&mut self) {
    println!("{}: {:?}", self.name, self.elapsed());
  }
}

/// 宏：简化基准测试
macro_rules! benchmark {
  ($name:expr, $code:block) => {{
    let _profiler = Profiler::new($name);
    $code
  }};
}

#[cfg(test)]
mod benchmark_macro_tests {
  use super::*;

  #[test]
  fn test_benchmark_macro() {
    benchmark!("测试基准测试宏", {
      let mut sum = 0;
      for i in 0..1000 {
        sum += i;
      }
      assert_eq!(sum, 499500);
    });
  }
}

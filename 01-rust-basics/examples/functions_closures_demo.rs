// 函数与闭包演示程序
// 展示Rust的函数定义、参数传递、闭包等概念

fn main() {
  println!("=== Rust 函数与闭包演示 ===\n");

  // 1. 基本函数
  basic_functions_demo();

  // 2. 函数参数和返回值
  parameters_and_returns_demo();

  // 3. 闭包基础
  closures_basic_demo();

  // 4. 闭包捕获
  closure_capture_demo();

  // 5. 函数指针
  function_pointers_demo();

  // 6. 高阶函数
  higher_order_functions_demo();
}

fn basic_functions_demo() {
  println!("1. 基本函数演示:");

  // 无参数无返回值
  greet();

  // 有参数无返回值
  greet_person("Alice");

  // 有参数有返回值
  let sum = add(5, 3);
  println!("  5 + 3 = {}", sum);

  // 表达式作为返回值
  let result = multiply_by_two(4);
  println!("  4 * 2 = {}", result);

  println!();
}

fn greet() {
  println!("  Hello, World!");
}

fn greet_person(name: &str) {
  println!("  Hello, {}!", name);
}

fn add(a: i32, b: i32) -> i32 {
  a + b // 表达式，没有分号
}

fn multiply_by_two(x: i32) -> i32 {
  x * 2
}

fn parameters_and_returns_demo() {
  println!("2. 参数和返回值演示:");

  // 多个返回值（使用元组）
  let (quotient, remainder) = divide_with_remainder(17, 5);
  println!("  17 ÷ 5 = {} 余 {}", quotient, remainder);

  // 引用参数
  let mut numbers = vec![1, 2, 3, 4, 5];
  println!("  修改前: {:?}", numbers);
  modify_vector(&mut numbers);
  println!("  修改后: {:?}", numbers);

  // 所有权转移
  let s = String::from("hello");
  let len = take_ownership_and_return_length(s);
  println!("  字符串长度: {}", len);
  // println!("{}", s); // 这行会编译错误，因为所有权已转移

  println!();
}

fn divide_with_remainder(dividend: i32, divisor: i32) -> (i32, i32) {
  (dividend / divisor, dividend % divisor)
}

fn modify_vector(vec: &mut Vec<i32>) {
  vec.push(6);
  vec[0] = 10;
}

fn take_ownership_and_return_length(s: String) -> usize {
  s.len()
}

fn closures_basic_demo() {
  println!("3. 闭包基础演示:");

  // 基本闭包语法
  let add_one = |x| x + 1;
  println!("  闭包 add_one(5) = {}", add_one(5));

  // 显式类型注解
  let multiply: fn(i32, i32) -> i32 = |x, y| x * y;
  println!("  闭包 multiply(3, 4) = {}", multiply(3, 4));

  // 多行闭包
  let complex_calculation = |x: i32| {
    let temp = x * 2;
    let result = temp + 10;
    result
  };
  println!(
    "  复杂计算 complex_calculation(5) = {}",
    complex_calculation(5)
  );

  // 使用闭包处理集合
  let numbers = vec![1, 2, 3, 4, 5];
  let doubled: Vec<i32> = numbers.iter().map(|x| x * 2).collect();
  println!("  原数组: {:?}", numbers);
  println!("  翻倍后: {:?}", doubled);

  println!();
}

fn closure_capture_demo() {
  println!("4. 闭包捕获演示:");

  // 捕获不可变引用
  let x = 4;
  let equal_to_x = |z| z == x;
  let y = 4;
  println!("  {} == {} ? {}", y, x, equal_to_x(y));

  // 捕获可变引用
  let mut count = 0;
  let mut increment = || {
    count += 1;
    println!("    计数器: {}", count);
  };

  increment();
  increment();
  increment();

  // 捕获所有权
  let name = String::from("Alice");
  let greet_closure = move || {
    println!("  Hello, {}!", name);
  };
  greet_closure();
  // println!("{}", name); // 这行会编译错误，因为所有权已转移

  println!();
}

fn function_pointers_demo() {
  println!("5. 函数指针演示:");

  // 函数指针
  let operation: fn(i32, i32) -> i32 = add;
  println!("  使用函数指针: {}", operation(10, 20));

  // 函数作为参数
  let result1 = apply_operation(5, 3, add);
  let result2 = apply_operation(5, 3, |a, b| a * b);

  println!("  加法结果: {}", result1);
  println!("  乘法结果: {}", result2);

  // 返回函数指针
  let op = get_operation(true);
  println!("  动态选择操作: {}", op(8, 2));

  println!();
}

fn apply_operation<F>(a: i32, b: i32, op: F) -> i32
where
  F: Fn(i32, i32) -> i32,
{
  op(a, b)
}

fn get_operation(add_mode: bool) -> fn(i32, i32) -> i32 {
  if add_mode {
    add
  } else {
    |a, b| a - b
  }
}

fn higher_order_functions_demo() {
  println!("6. 高阶函数演示:");

  let numbers = vec![1, 2, 3, 4, 5, 6, 7, 8, 9, 10];

  // filter: 过滤偶数
  let evens: Vec<i32> = numbers.iter().filter(|&x| x % 2 == 0).cloned().collect();
  println!("  偶数: {:?}", evens);

  // map: 平方
  let squares: Vec<i32> = numbers.iter().map(|x| x * x).collect();
  println!("  平方: {:?}", squares);

  // reduce: 求和
  let sum: i32 = numbers.iter().fold(0, |acc, x| acc + x);
  println!("  总和: {}", sum);

  // 链式操作
  let result: Vec<i32> = numbers
    .iter()
    .filter(|&x| x % 2 == 0) // 过滤偶数
    .map(|x| x * x) // 平方
    .filter(|&x| x > 10) // 过滤大于10的
    .collect();
  println!("  链式操作结果: {:?}", result);

  // 自定义高阶函数
  let processed = process_numbers(&numbers, |x| x * 3, |x| x > 15);
  println!("  自定义处理结果: {:?}", processed);

  println!();
}

fn process_numbers<F, P>(numbers: &[i32], transform: F, predicate: P) -> Vec<i32>
where
  F: Fn(i32) -> i32,
  P: Fn(i32) -> bool,
{
  numbers
    .iter()
    .map(|&x| transform(x))
    .filter(|&x| predicate(x))
    .collect()
}

// 递归函数示例
fn factorial(n: u64) -> u64 {
  match n {
    0 | 1 => 1,
    _ => n * factorial(n - 1),
  }
}

fn fibonacci(n: u32) -> u32 {
  match n {
    0 => 0,
    1 => 1,
    _ => fibonacci(n - 1) + fibonacci(n - 2),
  }
}

// 泛型函数示例
fn find_largest<T: PartialOrd + Copy>(list: &[T]) -> T {
  let mut largest = list[0];

  for &item in list {
    if item > largest {
      largest = item;
    }
  }

  largest
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn test_add() {
    assert_eq!(add(2, 3), 5);
  }

  #[test]
  fn test_factorial() {
    assert_eq!(factorial(0), 1);
    assert_eq!(factorial(1), 1);
    assert_eq!(factorial(5), 120);
  }

  #[test]
  fn test_fibonacci() {
    assert_eq!(fibonacci(0), 0);
    assert_eq!(fibonacci(1), 1);
    assert_eq!(fibonacci(10), 55);
  }

  #[test]
  fn test_find_largest() {
    let numbers = vec![34, 50, 25, 100, 65];
    assert_eq!(find_largest(&numbers), 100);

    let chars = vec!['y', 'm', 'a', 'q'];
    assert_eq!(find_largest(&chars), 'y');
  }
}

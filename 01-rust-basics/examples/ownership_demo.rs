// 所有权系统演示程序
// 展示Rust的所有权、借用和生命周期概念

fn main() {
  println!("=== Rust 所有权系统演示 ===\n");

  // 1. 所有权转移
  ownership_transfer_demo();

  // 2. 借用和引用
  borrowing_demo();

  // 3. 可变借用
  mutable_borrowing_demo();

  // 4. 生命周期
  lifetime_demo();

  // 5. 字符串所有权
  string_ownership_demo();
}

fn ownership_transfer_demo() {
  println!("1. 所有权转移演示:");

  let s1 = String::from("hello");
  println!("  s1 = {}", s1);

  // 所有权转移给s2
  let s2 = s1;
  println!("  s2 = {}", s2);
  // println!("  s1 = {}", s1); // 这行会编译错误，因为s1的所有权已转移

  // 克隆可以避免所有权转移
  let s3 = s2.clone();
  println!("  s3 = {} (克隆)", s3);
  println!("  s2 = {} (原始)", s2);

  println!();
}

fn borrowing_demo() {
  println!("2. 借用演示:");

  let s = String::from("hello world");

  // 不可变借用
  let len = calculate_length(&s);
  println!("  字符串 '{}' 的长度是 {}", s, len);

  // 多个不可变借用是允许的
  let r1 = &s;
  let r2 = &s;
  println!("  r1: {}, r2: {}", r1, r2);

  println!();
}

fn calculate_length(s: &String) -> usize {
  s.len()
} // s离开作用域，但因为它不拥有引用的值，所以什么也不会发生

fn mutable_borrowing_demo() {
  println!("3. 可变借用演示:");

  let mut s = String::from("hello");
  println!("  修改前: {}", s);

  change(&mut s);
  println!("  修改后: {}", s);

  // 可变借用的限制
  let r1 = &mut s;
  // let r2 = &mut s; // 这行会编译错误，不能同时有两个可变借用
  println!("  可变引用: {}", r1);

  println!();
}

fn change(some_string: &mut String) {
  some_string.push_str(", world");
}

fn lifetime_demo() {
  println!("4. 生命周期演示:");

  let string1 = String::from("abcd");
  let string2 = "xyz";

  let result = longest(string1.as_str(), string2);
  println!("  最长的字符串是: {}", result);

  // 生命周期和作用域
  let r;
  {
    let x = 5;
    r = &x;
    println!("  内部作用域中 r = {}", r);
  }
  // println!("  外部作用域中 r = {}", r); // 这行会编译错误

  println!();
}

// 生命周期注解示例
fn longest<'a>(x: &'a str, y: &'a str) -> &'a str {
  if x.len() > y.len() {
    x
  } else {
    y
  }
}

fn string_ownership_demo() {
  println!("5. 字符串所有权演示:");

  // 字符串字面量 vs String
  let s1 = "hello"; // 字符串字面量，存储在程序的二进制文件中
  let s2 = String::from("hello"); // String类型，存储在堆上

  println!("  字符串字面量: {}", s1);
  println!("  String类型: {}", s2);

  // 字符串切片
  let s = String::from("hello world");
  let hello = &s[0..5];
  let world = &s[6..11];

  println!("  原字符串: {}", s);
  println!("  切片1: {}", hello);
  println!("  切片2: {}", world);

  // 函数参数中的字符串
  takes_ownership(s2); // s2的所有权被转移
                       // println!("{}", s2); // 这行会编译错误

  makes_copy(s1); // s1是Copy类型，所以这里是复制
  println!("  s1仍然可用: {}", s1);

  println!();
}

fn takes_ownership(some_string: String) {
  println!("  接收所有权: {}", some_string);
} // some_string离开作用域并被丢弃

fn makes_copy(some_integer: &str) {
  println!("  复制值: {}", some_integer);
} // some_integer离开作用域，但没有特殊之处

// 返回值和所有权
fn gives_ownership() -> String {
  let some_string = String::from("hello");
  some_string // 返回some_string并将所有权转移给调用函数
}

fn takes_and_gives_back(a_string: String) -> String {
  a_string // 返回a_string并将所有权转移给调用函数
}

// 结构体中的所有权
#[derive(Debug)]
struct User {
  username: String,
  email: String,
  sign_in_count: u64,
  active: bool,
}

fn struct_ownership_demo() {
  let user1 = User {
    email: String::from("someone@example.com"),
    username: String::from("someusername123"),
    active: true,
    sign_in_count: 1,
  };

  // 部分移动
  let user2 = User {
    email: String::from("another@example.com"),
    username: String::from("anotherusername567"),
    ..user1 // 使用user1的其他字段
  };

  // user1的active和sign_in_count仍然可用，但email和username已被移动
  println!("user1 is active: {}", user1.active);
  // println!("user1 email: {}", user1.email); // 这行会编译错误

  println!("user2: {:?}", user2);
}

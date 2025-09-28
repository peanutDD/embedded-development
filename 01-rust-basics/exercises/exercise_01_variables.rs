//! 练习1：变量和类型
//!
//! 完成以下练习来熟悉Rust的变量声明和类型系统

// TODO: 修复编译错误并完成练习

fn main() {
  println!("=== 练习1：变量和类型 ===");

  // 练习 1.1: 修复变量声明
  // 错误：尝试修改不可变变量
  // let x = 5;
  // x = 10; // 这行会导致编译错误
  // println!("x = {}", x);

  // TODO: 修复上面的代码，使其能够编译通过
  // 提示：使用 mut 关键字

  // 练习 1.2: 类型推断和显式类型
  // TODO: 为以下变量添加正确的类型注解
  let a = 42; // TODO: 添加 i32 类型注解
  let b = 3.14; // TODO: 添加 f64 类型注解
  let c = true; // TODO: 添加 bool 类型注解
  let d = 'R'; // TODO: 添加 char 类型注解

  println!("a = {}, b = {}, c = {}, d = {}", a, b, c, d);

  // 练习 1.3: 元组和数组
  // TODO: 创建一个包含三个不同类型元素的元组
  // let tuple = (); // 替换为正确的元组

  // TODO: 使用模式匹配解构元组
  // let (x, y, z) = tuple;
  // println!("元组元素: {}, {}, {}", x, y, z);

  // TODO: 创建一个包含5个整数的数组
  // let array = []; // 替换为正确的数组
  // println!("数组第一个元素: {}", array[0]);

  // 练习 1.4: 字符串
  // TODO: 创建一个 String 类型的变量并添加文本
  // let mut s = String::new();
  // TODO: 向字符串添加 "Hello, "
  // TODO: 向字符串添加 "Rust!"
  // println!("字符串: {}", s);

  // 练习 1.5: 常量
  // TODO: 在函数外部定义一个常量 MAX_SIZE，值为 1000
  // 然后在这里使用它
  // println!("最大尺寸: {}", MAX_SIZE);

  // 练习 1.6: 变量遮蔽
  let x = 5;
  println!("第一个 x: {}", x);

  // TODO: 使用变量遮蔽将 x 重新定义为字符串类型
  // let x = ???;
  // println!("遮蔽后的 x: {}", x);

  // 练习 1.7: 类型转换
  let integer = 42;
  // TODO: 将整数转换为浮点数
  // let float_value = ???;
  // println!("转换后的浮点数: {}", float_value);

  // 练习 1.8: 溢出处理
  // TODO: 使用 checked_add 方法安全地进行加法运算
  let a: u8 = 200;
  let b: u8 = 100;
  // let result = ???;
  // match result {
  //     Some(sum) => println!("和: {}", sum),
  //     None => println!("溢出!"),
  // }
}

// TODO: 在这里定义 MAX_SIZE 常量

// 练习答案（取消注释来查看答案）
/*
// 练习 1.1 答案:
let mut x = 5;
x = 10;
println!("x = {}", x);

// 练习 1.2 答案:
let a: i32 = 42;
let b: f64 = 3.14;
let c: bool = true;
let d: char = 'R';

// 练习 1.3 答案:
let tuple = (42, 3.14, "hello");
let (x, y, z) = tuple;
println!("元组元素: {}, {}, {}", x, y, z);

let array = [1, 2, 3, 4, 5];
println!("数组第一个元素: {}", array[0]);

// 练习 1.4 答案:
let mut s = String::new();
s.push_str("Hello, ");
s.push_str("Rust!");
println!("字符串: {}", s);

// 练习 1.5 答案:
const MAX_SIZE: u32 = 1000;

// 练习 1.6 答案:
let x = "现在是字符串";
println!("遮蔽后的 x: {}", x);

// 练习 1.7 答案:
let float_value = integer as f64;
println!("转换后的浮点数: {}", float_value);

// 练习 1.8 答案:
let result = a.checked_add(b);
match result {
    Some(sum) => println!("和: {}", sum),
    None => println!("溢出!"),
}
*/

//! 控制流演示示例
//! 
//! 这个示例展示了Rust中的各种控制流结构

fn main() {
    println!("=== Rust 控制流演示 ===");
    
    // 1. if 表达式
    if_expressions();
    
    // 2. 循环
    loops_demo();
    
    // 3. match 表达式
    match_expressions();
    
    // 4. if let 和 while let
    if_let_while_let();
}

/// 演示 if 表达式
fn if_expressions() {
    println!("\n--- if 表达式 ---");
    
    let number = 6;
    
    // 基本 if
    if number < 5 {
        println!("条件为真");
    } else {
        println!("条件为假");
    }
    
    // 多重条件
    if number % 4 == 0 {
        println!("数字能被 4 整除");
    } else if number % 3 == 0 {
        println!("数字能被 3 整除");
    } else if number % 2 == 0 {
        println!("数字能被 2 整除");
    } else {
        println!("数字不能被 4、3 或 2 整除");
    }
    
    // if 是表达式，可以用于赋值
    let condition = true;
    let number = if condition { 5 } else { 6 };
    println!("if 表达式的值: {}", number);
    
    // 复杂的 if 表达式
    let x = 10;
    let result = if x > 0 {
        if x > 5 {
            "大于5"
        } else {
            "大于0但小于等于5"
        }
    } else if x < 0 {
        "小于0"
    } else {
        "等于0"
    };
    println!("复杂 if 表达式结果: {}", result);
}

/// 演示循环
fn loops_demo() {
    println!("\n--- 循环 ---");
    
    // loop 循环
    println!("loop 循环:");
    let mut counter = 0;
    let result = loop {
        counter += 1;
        if counter == 3 {
            println!("  计数器: {}", counter);
        }
        if counter == 5 {
            break counter * 2; // 从循环返回值
        }
    };
    println!("loop 返回值: {}", result);
    
    // while 循环
    println!("while 循环:");
    let mut number = 3;
    while number != 0 {
        println!("  倒计时: {}!", number);
        number -= 1;
    }
    println!("  发射!!!");
    
    // for 循环遍历集合
    println!("for 循环遍历数组:");
    let a = [10, 20, 30, 40, 50];
    for element in a {
        println!("  值: {}", element);
    }
    
    // for 循环使用范围
    println!("for 循环使用范围:");
    for number in (1..4).rev() {
        println!("  {}!", number);
    }
    println!("  发射!!!");
    
    // 嵌套循环和标签
    println!("嵌套循环和标签:");
    'outer: for i in 1..=3 {
        for j in 1..=3 {
            if i == 2 && j == 2 {
                println!("  跳出外层循环 at i={}, j={}", i, j);
                break 'outer;
            }
            println!("  i={}, j={}", i, j);
        }
    }
    
    // 循环中的 continue
    println!("使用 continue:");
    for i in 1..=5 {
        if i % 2 == 0 {
            continue;
        }
        println!("  奇数: {}", i);
    }
}

/// 演示 match 表达式
fn match_expressions() {
    println!("\n--- match 表达式 ---");
    
    // 基本 match
    let number = 3;
    match number {
        1 => println!("一"),
        2 => println!("二"),
        3 => println!("三"),
        _ => println!("其他"),
    }
    
    // match 返回值
    let number = 6;
    let description = match number {
        1 => "一",
        2 => "二",
        3 => "三",
        4 | 5 | 6 => "四到六", // 多个模式
        7..=10 => "七到十",   // 范围模式
        _ => "其他",
    };
    println!("数字 {} 的描述: {}", number, description);
    
    // 匹配 Option
    let some_number = Some(5);
    match some_number {
        Some(x) if x < 5 => println!("小于五: {}", x),
        Some(x) => println!("数字: {}", x),
        None => println!("没有值"),
    }
    
    // 匹配元组
    let point = (0, 5);
    match point {
        (0, y) => println!("在 y 轴上，y = {}", y),
        (x, 0) => println!("在 x 轴上，x = {}", x),
        (x, y) => println!("不在轴上: ({}, {})", x, y),
    }
    
    // 解构结构体
    struct Point {
        x: i32,
        y: i32,
    }
    
    let p = Point { x: 0, y: 7 };
    match p {
        Point { x, y: 0 } => println!("在 x 轴上，x = {}", x),
        Point { x: 0, y } => println!("在 y 轴上，y = {}", y),
        Point { x, y } => println!("不在轴上: ({}, {})", x, y),
    }
    
    // 匹配枚举
    enum Message {
        Quit,
        Move { x: i32, y: i32 },
        Write(String),
        ChangeColor(i32, i32, i32),
    }
    
    let msg = Message::ChangeColor(0, 160, 255);
    match msg {
        Message::Quit => println!("退出消息"),
        Message::Move { x, y } => println!("移动到 ({}, {})", x, y),
        Message::Write(text) => println!("写入文本: {}", text),
        Message::ChangeColor(r, g, b) => {
            println!("改变颜色为红: {}, 绿: {}, 蓝: {}", r, g, b)
        }
    }
    
    // 匹配守卫
    let num = Some(4);
    match num {
        Some(x) if x < 5 => println!("小于五: {}", x),
        Some(x) => println!("大于等于五: {}", x),
        None => (),
    }
    
    // @ 绑定
    enum Message2 {
        Hello { id: i32 },
    }
    
    let msg = Message2::Hello { id: 5 };
    match msg {
        Message2::Hello { id: id_variable @ 3..=7 } => {
            println!("找到了一个在范围内的 id: {}", id_variable)
        }
        Message2::Hello { id: 10..=12 } => {
            println!("找到了另一个范围内的 id")
        }
        Message2::Hello { id } => {
            println!("找到了其他 id: {}", id)
        }
    }
}

/// 演示 if let 和 while let
fn if_let_while_let() {
    println!("\n--- if let 和 while let ---");
    
    // if let
    let some_u8_value = Some(3u8);
    if let Some(3) = some_u8_value {
        println!("值是三");
    }
    
    // if let 与 else
    let some_u8_value = Some(0u8);
    if let Some(3) = some_u8_value {
        println!("值是三");
    } else {
        println!("值不是三");
    }
    
    // while let
    let mut stack = Vec::new();
    stack.push(1);
    stack.push(2);
    stack.push(3);
    
    println!("使用 while let 弹出栈中的元素:");
    while let Some(top) = stack.pop() {
        println!("  弹出: {}", top);
    }
    
    // for 循环中的模式
    let v = vec!['a', 'b', 'c'];
    println!("使用 for 循环和模式:");
    for (index, value) in v.iter().enumerate() {
        println!("  索引 {} 处的值是 {}", index, value);
    }
    
    // let 语句中的模式
    let (x, y, z) = (1, 2, 3);
    println!("解构赋值: x={}, y={}, z={}", x, y, z);
    
    // 函数参数中的模式
    let point = (3, 5);
    print_coordinates(&point);
}

/// 在函数参数中使用模式
fn print_coordinates(&(x, y): &(i32, i32)) {
    println!("当前位置: ({}, {})", x, y);
}
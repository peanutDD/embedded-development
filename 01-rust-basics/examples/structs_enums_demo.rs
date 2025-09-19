// 结构体与枚举演示程序
// 展示Rust的结构体定义、方法、枚举类型等概念

#[derive(Debug, Clone, PartialEq)]
struct Point {
    x: f64,
    y: f64,
}

#[derive(Debug)]
struct Rectangle {
    width: f64,
    height: f64,
}

#[derive(Debug)]
struct Circle {
    center: Point,
    radius: f64,
}

// 元组结构体
#[derive(Debug)]
struct Color(u8, u8, u8);

// 单元结构体
#[derive(Debug)]
struct Unit;

// 枚举定义
#[derive(Debug, PartialEq)]
enum Shape {
    Circle { center: Point, radius: f64 },
    Rectangle { width: f64, height: f64 },
    Triangle { a: Point, b: Point, c: Point },
}

#[derive(Debug)]
enum Message {
    Quit,
    Move { x: i32, y: i32 },
    Write(String),
    ChangeColor(u8, u8, u8),
}

// 带泛型的枚举
#[derive(Debug)]
enum Result<T, E> {
    Ok(T),
    Err(E),
}

fn main() {
    println!("=== Rust 结构体与枚举演示 ===\n");
    
    // 1. 结构体基础
    struct_basics_demo();
    
    // 2. 结构体方法
    struct_methods_demo();
    
    // 3. 枚举基础
    enum_basics_demo();
    
    // 4. 模式匹配
    pattern_matching_demo();
    
    // 5. Option和Result
    option_result_demo();
    
    // 6. 高级特性
    advanced_features_demo();
}

impl Point {
    // 关联函数（类似静态方法）
    fn new(x: f64, y: f64) -> Point {
        Point { x, y }
    }
    
    fn origin() -> Point {
        Point { x: 0.0, y: 0.0 }
    }
    
    // 实例方法
    fn distance_from_origin(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }
    
    fn distance_to(&self, other: &Point) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
    
    // 可变方法
    fn translate(&mut self, dx: f64, dy: f64) {
        self.x += dx;
        self.y += dy;
    }
    
    // 消费self的方法
    fn into_tuple(self) -> (f64, f64) {
        (self.x, self.y)
    }
}

impl Rectangle {
    fn new(width: f64, height: f64) -> Rectangle {
        Rectangle { width, height }
    }
    
    fn area(&self) -> f64 {
        self.width * self.height
    }
    
    fn perimeter(&self) -> f64 {
        2.0 * (self.width + self.height)
    }
    
    fn can_hold(&self, other: &Rectangle) -> bool {
        self.width > other.width && self.height > other.height
    }
    
    fn square(size: f64) -> Rectangle {
        Rectangle {
            width: size,
            height: size,
        }
    }
}

impl Circle {
    fn new(center: Point, radius: f64) -> Circle {
        Circle { center, radius }
    }
    
    fn area(&self) -> f64 {
        std::f64::consts::PI * self.radius.powi(2)
    }
    
    fn circumference(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.radius
    }
}

impl Shape {
    fn area(&self) -> f64 {
        match self {
            Shape::Circle { radius, .. } => std::f64::consts::PI * radius.powi(2),
            Shape::Rectangle { width, height } => width * height,
            Shape::Triangle { a, b, c } => {
                // 使用海伦公式计算三角形面积
                let side_a = a.distance_to(b);
                let side_b = b.distance_to(c);
                let side_c = c.distance_to(a);
                let s = (side_a + side_b + side_c) / 2.0;
                (s * (s - side_a) * (s - side_b) * (s - side_c)).sqrt()
            }
        }
    }
    
    fn perimeter(&self) -> f64 {
        match self {
            Shape::Circle { radius, .. } => 2.0 * std::f64::consts::PI * radius,
            Shape::Rectangle { width, height } => 2.0 * (width + height),
            Shape::Triangle { a, b, c } => {
                a.distance_to(b) + b.distance_to(c) + c.distance_to(a)
            }
        }
    }
}

impl Message {
    fn process(&self) {
        match self {
            Message::Quit => println!("  退出程序"),
            Message::Move { x, y } => println!("  移动到坐标 ({}, {})", x, y),
            Message::Write(text) => println!("  写入文本: {}", text),
            Message::ChangeColor(r, g, b) => println!("  改变颜色为 RGB({}, {}, {})", r, g, b),
        }
    }
}

fn struct_basics_demo() {
    println!("1. 结构体基础演示:");
    
    // 创建结构体实例
    let point1 = Point { x: 3.0, y: 4.0 };
    let point2 = Point::new(1.0, 2.0);
    let origin = Point::origin();
    
    println!("  point1: {:?}", point1);
    println!("  point2: {:?}", point2);
    println!("  origin: {:?}", origin);
    
    // 结构体更新语法
    let point3 = Point { x: 5.0, ..point1 };
    println!("  point3: {:?}", point3);
    
    // 元组结构体
    let red = Color(255, 0, 0);
    let green = Color(0, 255, 0);
    println!("  红色: {:?}", red);
    println!("  绿色: {:?}", green);
    
    // 解构
    let Color(r, g, b) = red;
    println!("  红色分量: R={}, G={}, B={}", r, g, b);
    
    println!();
}

fn struct_methods_demo() {
    println!("2. 结构体方法演示:");
    
    let mut point = Point::new(3.0, 4.0);
    println!("  初始点: {:?}", point);
    println!("  到原点距离: {:.2}", point.distance_from_origin());
    
    let other_point = Point::new(6.0, 8.0);
    println!("  到另一点距离: {:.2}", point.distance_to(&other_point));
    
    // 修改点的位置
    point.translate(1.0, 1.0);
    println!("  平移后: {:?}", point);
    
    // 矩形示例
    let rect1 = Rectangle::new(30.0, 50.0);
    let rect2 = Rectangle::new(10.0, 40.0);
    let square = Rectangle::square(25.0);
    
    println!("  矩形1: {:?}", rect1);
    println!("  矩形1面积: {}", rect1.area());
    println!("  矩形1周长: {}", rect1.perimeter());
    println!("  矩形1能容纳矩形2吗? {}", rect1.can_hold(&rect2));
    println!("  正方形: {:?}", square);
    
    // 圆形示例
    let circle = Circle::new(Point::origin(), 5.0);
    println!("  圆形: {:?}", circle);
    println!("  圆形面积: {:.2}", circle.area());
    println!("  圆形周长: {:.2}", circle.circumference());
    
    println!();
}

fn enum_basics_demo() {
    println!("3. 枚举基础演示:");
    
    // 创建枚举实例
    let circle = Shape::Circle {
        center: Point::new(0.0, 0.0),
        radius: 5.0,
    };
    
    let rectangle = Shape::Rectangle {
        width: 10.0,
        height: 20.0,
    };
    
    let triangle = Shape::Triangle {
        a: Point::new(0.0, 0.0),
        b: Point::new(3.0, 0.0),
        c: Point::new(1.5, 2.6),
    };
    
    println!("  圆形: {:?}", circle);
    println!("  圆形面积: {:.2}", circle.area());
    
    println!("  矩形: {:?}", rectangle);
    println!("  矩形面积: {:.2}", rectangle.area());
    
    println!("  三角形: {:?}", triangle);
    println!("  三角形面积: {:.2}", triangle.area());
    
    // 消息枚举
    let messages = vec![
        Message::Quit,
        Message::Move { x: 10, y: 20 },
        Message::Write(String::from("Hello, Rust!")),
        Message::ChangeColor(255, 128, 0),
    ];
    
    println!("  处理消息:");
    for message in messages {
        message.process();
    }
    
    println!();
}

fn pattern_matching_demo() {
    println!("4. 模式匹配演示:");
    
    let shapes = vec![
        Shape::Circle { center: Point::origin(), radius: 3.0 },
        Shape::Rectangle { width: 4.0, height: 6.0 },
        Shape::Triangle {
            a: Point::new(0.0, 0.0),
            b: Point::new(4.0, 0.0),
            c: Point::new(2.0, 3.0),
        },
    ];
    
    for (i, shape) in shapes.iter().enumerate() {
        match shape {
            Shape::Circle { center, radius } => {
                println!("  形状{}: 圆形，中心({:.1}, {:.1})，半径{:.1}", 
                         i + 1, center.x, center.y, radius);
            }
            Shape::Rectangle { width, height } => {
                println!("  形状{}: 矩形，宽{:.1}，高{:.1}", i + 1, width, height);
            }
            Shape::Triangle { a, b, c } => {
                println!("  形状{}: 三角形，顶点({:.1},{:.1}), ({:.1},{:.1}), ({:.1},{:.1})", 
                         i + 1, a.x, a.y, b.x, b.y, c.x, c.y);
            }
        }
        
        // 使用if let简化匹配
        if let Shape::Circle { radius, .. } = shape {
            if *radius > 2.0 {
                println!("    这是一个大圆形！");
            }
        }
    }
    
    println!();
}

fn option_result_demo() {
    println!("5. Option和Result演示:");
    
    // Option示例
    let numbers = vec![1, 2, 3, 4, 5];
    
    match find_number(&numbers, 3) {
        Some(index) => println!("  找到数字3在索引{}", index),
        None => println!("  未找到数字3"),
    }
    
    match find_number(&numbers, 10) {
        Some(index) => println!("  找到数字10在索引{}", index),
        None => println!("  未找到数字10"),
    }
    
    // 使用Option的方法
    let maybe_number = Some(42);
    let doubled = maybe_number.map(|x| x * 2);
    println!("  42翻倍: {:?}", doubled);
    
    let default_value = maybe_number.unwrap_or(0);
    println!("  默认值: {}", default_value);
    
    // Result示例
    match divide(10.0, 2.0) {
        Ok(result) => println!("  10 ÷ 2 = {}", result),
        Err(error) => println!("  错误: {}", error),
    }
    
    match divide(10.0, 0.0) {
        Ok(result) => println!("  10 ÷ 0 = {}", result),
        Err(error) => println!("  错误: {}", error),
    }
    
    // 链式操作
    let result = Some(5)
        .map(|x| x * 2)
        .and_then(|x| if x > 5 { Some(x) } else { None })
        .unwrap_or(0);
    println!("  链式操作结果: {}", result);
    
    println!();
}

fn find_number(numbers: &[i32], target: i32) -> Option<usize> {
    for (index, &number) in numbers.iter().enumerate() {
        if number == target {
            return Some(index);
        }
    }
    None
}

fn divide(a: f64, b: f64) -> Result<f64, String> {
    if b == 0.0 {
        Err(String::from("除数不能为零"))
    } else {
        Ok(a / b)
    }
}

fn advanced_features_demo() {
    println!("6. 高级特性演示:");
    
    // 嵌套枚举
    #[derive(Debug)]
    enum WebEvent {
        PageLoad,
        PageUnload,
        KeyPress(char),
        Paste(String),
        Click { x: i64, y: i64 },
    }
    
    let events = vec![
        WebEvent::PageLoad,
        WebEvent::KeyPress('x'),
        WebEvent::Paste(String::from("my text")),
        WebEvent::Click { x: 20, y: 80 },
        WebEvent::PageUnload,
    ];
    
    for event in events {
        match event {
            WebEvent::PageLoad => println!("  页面加载"),
            WebEvent::PageUnload => println!("  页面卸载"),
            WebEvent::KeyPress(c) => println!("  按键: '{}'", c),
            WebEvent::Paste(s) => println!("  粘贴: \"{}\"", s),
            WebEvent::Click { x, y } => println!("  点击坐标: ({}, {})", x, y),
        }
    }
    
    // 带有方法的枚举
    #[derive(Debug)]
    enum TrafficLight {
        Red,
        Yellow,
        Green,
    }
    
    impl TrafficLight {
        fn duration(&self) -> u32 {
            match self {
                TrafficLight::Red => 30,
                TrafficLight::Yellow => 5,
                TrafficLight::Green => 25,
            }
        }
        
        fn next(&self) -> TrafficLight {
            match self {
                TrafficLight::Red => TrafficLight::Green,
                TrafficLight::Yellow => TrafficLight::Red,
                TrafficLight::Green => TrafficLight::Yellow,
            }
        }
    }
    
    let mut light = TrafficLight::Red;
    for _ in 0..4 {
        println!("  交通灯: {:?}, 持续时间: {}秒", light, light.duration());
        light = light.next();
    }
    
    println!();
}

// 自定义错误类型
#[derive(Debug)]
enum MathError {
    DivisionByZero,
    NegativeSquareRoot,
    Overflow,
}

impl std::fmt::Display for MathError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            MathError::DivisionByZero => write!(f, "除数不能为零"),
            MathError::NegativeSquareRoot => write!(f, "负数不能开平方根"),
            MathError::Overflow => write!(f, "数值溢出"),
        }
    }
}

fn safe_divide(a: f64, b: f64) -> Result<f64, MathError> {
    if b == 0.0 {
        Err(MathError::DivisionByZero)
    } else {
        Ok(a / b)
    }
}

fn safe_sqrt(x: f64) -> Result<f64, MathError> {
    if x < 0.0 {
        Err(MathError::NegativeSquareRoot)
    } else {
        Ok(x.sqrt())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_point_distance() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(3.0, 4.0);
        assert_eq!(p1.distance_to(&p2), 5.0);
    }
    
    #[test]
    fn test_rectangle_area() {
        let rect = Rectangle::new(10.0, 20.0);
        assert_eq!(rect.area(), 200.0);
    }
    
    #[test]
    fn test_shape_area() {
        let circle = Shape::Circle {
            center: Point::origin(),
            radius: 1.0,
        };
        let area = circle.area();
        assert!((area - std::f64::consts::PI).abs() < 0.001);
    }
    
    #[test]
    fn test_find_number() {
        let numbers = vec![1, 2, 3, 4, 5];
        assert_eq!(find_number(&numbers, 3), Some(2));
        assert_eq!(find_number(&numbers, 10), None);
    }
    
    #[test]
    fn test_safe_divide() {
        assert_eq!(safe_divide(10.0, 2.0), Ok(5.0));
        assert!(matches!(safe_divide(10.0, 0.0), Err(MathError::DivisionByZero)));
    }
}
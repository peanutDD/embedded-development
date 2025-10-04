//! 第5章：结构体和枚举练习
//!
//! 本文件包含Rust结构体和枚举的专门练习，包括：
//! - 结构体定义和实例化
//! - 方法和关联函数
//! - 元组结构体
//! - 单元结构体
//! - 枚举定义和使用
//! - 模式匹配
//! - Option和Result

#![allow(dead_code, unused_variables)]

fn main() {
  println!("=== 第5章：结构体和枚举练习 ===");
  println!("运行 cargo test exercise_05 来测试你的答案");
  println!();

  // 示例运行
  let rect = Rectangle::new(10.0, 5.0);
  println!("矩形面积: {}", rect.area());

  let color = Color::RGB(255, 0, 0);
  println!("颜色: {:?}", color);
}

// ============================================================================
// 练习1: 基础结构体
// ============================================================================

/// 练习1: 定义一个表示点的结构体
#[derive(Debug, PartialEq, Clone, Copy)]
struct Point {
  x: f64,
  y: f64,
}

impl Point {
  /// 创建新的点
  fn new(x: f64, y: f64) -> Self {
    Point { x, y }
  }

  /// 计算到原点的距离
  fn distance_from_origin(&self) -> f64 {
    (self.x * self.x + self.y * self.y).sqrt()
  }

  /// 计算到另一个点的距离
  fn distance_to(&self, other: &Point) -> f64 {
    let dx = self.x - other.x;
    let dy = self.y - other.y;
    (dx * dx + dy * dy).sqrt()
  }

  /// 移动点
  fn translate(&mut self, dx: f64, dy: f64) {
    self.x += dx;
    self.y += dy;
  }
}

/// 练习2: 矩形结构体
#[derive(Debug, PartialEq)]
struct Rectangle {
  width: f64,
  height: f64,
}

impl Rectangle {
  /// 创建新的矩形
  fn new(width: f64, height: f64) -> Self {
    Rectangle { width, height }
  }

  /// 创建正方形
  fn square(size: f64) -> Self {
    Rectangle {
      width: size,
      height: size,
    }
  }

  /// 计算面积
  fn area(&self) -> f64 {
    self.width * self.height
  }

  /// 计算周长
  fn perimeter(&self) -> f64 {
    2.0 * (self.width + self.height)
  }

  /// 判断是否为正方形
  fn is_square(&self) -> bool {
    (self.width - self.height).abs() < f64::EPSILON
  }

  /// 缩放矩形
  fn scale(&mut self, factor: f64) {
    self.width *= factor;
    self.height *= factor;
  }
}

// ============================================================================
// 练习3: 复杂结构体
// ============================================================================

/// 练习3: 学生信息管理
#[derive(Debug, PartialEq, Clone)]
struct Student {
  id: u32,
  name: String,
  age: u8,
  grades: Vec<f64>,
}

impl Student {
  /// 创建新学生
  fn new(id: u32, name: String, age: u8) -> Self {
    Student {
      id,
      name,
      age,
      grades: Vec::new(),
    }
  }

  /// 添加成绩
  fn add_grade(&mut self, grade: f64) {
    if grade >= 0.0 && grade <= 100.0 {
      self.grades.push(grade);
    }
  }

  /// 计算平均成绩
  fn average_grade(&self) -> Option<f64> {
    if self.grades.is_empty() {
      None
    } else {
      let sum: f64 = self.grades.iter().sum();
      Some(sum / self.grades.len() as f64)
    }
  }

  /// 获取最高成绩
  fn highest_grade(&self) -> Option<f64> {
    self.grades.iter().copied().fold(None, |acc, grade| {
      Some(acc.map_or(grade, |max| if grade > max { grade } else { max }))
    })
  }

  /// 判断是否及格（平均分>=60）
  fn is_passing(&self) -> bool {
    self.average_grade().map_or(false, |avg| avg >= 60.0)
  }
}

// ============================================================================
// 练习4: 元组结构体
// ============================================================================

/// 练习4: 颜色表示（RGB）
#[derive(Debug, PartialEq, Clone, Copy)]
struct RGB(u8, u8, u8);

impl RGB {
  /// 创建新颜色
  fn new(r: u8, g: u8, b: u8) -> Self {
    RGB(r, g, b)
  }

  /// 获取红色分量
  fn red(&self) -> u8 {
    self.0
  }

  /// 获取绿色分量
  fn green(&self) -> u8 {
    self.1
  }

  /// 获取蓝色分量
  fn blue(&self) -> u8 {
    self.2
  }

  /// 转换为十六进制字符串
  fn to_hex(&self) -> String {
    format!("#{:02X}{:02X}{:02X}", self.0, self.1, self.2)
  }

  /// 计算亮度
  fn brightness(&self) -> f64 {
    (0.299 * self.0 as f64 + 0.587 * self.1 as f64 + 0.114 * self.2 as f64) / 255.0
  }
}

/// 练习5: 3D向量
#[derive(Debug, PartialEq, Clone, Copy)]
struct Vector3D(f64, f64, f64);

impl Vector3D {
  /// 创建新向量
  fn new(x: f64, y: f64, z: f64) -> Self {
    Vector3D(x, y, z)
  }

  /// 零向量
  fn zero() -> Self {
    Vector3D(0.0, 0.0, 0.0)
  }

  /// 计算长度
  fn magnitude(&self) -> f64 {
    (self.0 * self.0 + self.1 * self.1 + self.2 * self.2).sqrt()
  }

  /// 标准化向量
  fn normalize(&self) -> Option<Vector3D> {
    let mag = self.magnitude();
    if mag > f64::EPSILON {
      Some(Vector3D(self.0 / mag, self.1 / mag, self.2 / mag))
    } else {
      None
    }
  }

  /// 点积
  fn dot(&self, other: &Vector3D) -> f64 {
    self.0 * other.0 + self.1 * other.1 + self.2 * other.2
  }

  /// 叉积
  fn cross(&self, other: &Vector3D) -> Vector3D {
    Vector3D(
      self.1 * other.2 - self.2 * other.1,
      self.2 * other.0 - self.0 * other.2,
      self.0 * other.1 - self.1 * other.0,
    )
  }
}

// ============================================================================
// 练习6: 基础枚举
// ============================================================================

/// 练习6: 方向枚举
#[derive(Debug, PartialEq, Clone, Copy)]
enum Direction {
  North,
  South,
  East,
  West,
}

impl Direction {
  /// 获取相反方向
  fn opposite(&self) -> Direction {
    match self {
      Direction::North => Direction::South,
      Direction::South => Direction::North,
      Direction::East => Direction::West,
      Direction::West => Direction::East,
    }
  }

  /// 顺时针旋转90度
  fn turn_right(&self) -> Direction {
    match self {
      Direction::North => Direction::East,
      Direction::East => Direction::South,
      Direction::South => Direction::West,
      Direction::West => Direction::North,
    }
  }

  /// 逆时针旋转90度
  fn turn_left(&self) -> Direction {
    match self {
      Direction::North => Direction::West,
      Direction::West => Direction::South,
      Direction::South => Direction::East,
      Direction::East => Direction::North,
    }
  }
}

// ============================================================================
// 练习7: 带数据的枚举
// ============================================================================

/// 练习7: 几何图形枚举
#[derive(Debug, PartialEq)]
enum Shape {
  Circle { radius: f64 },
  Rectangle { width: f64, height: f64 },
  Triangle { base: f64, height: f64 },
}

impl Shape {
  /// 计算面积
  fn area(&self) -> f64 {
    match self {
      Shape::Circle { radius } => std::f64::consts::PI * radius * radius,
      Shape::Rectangle { width, height } => width * height,
      Shape::Triangle { base, height } => 0.5 * base * height,
    }
  }

  /// 计算周长
  fn perimeter(&self) -> f64 {
    match self {
      Shape::Circle { radius } => 2.0 * std::f64::consts::PI * radius,
      Shape::Rectangle { width, height } => 2.0 * (width + height),
      Shape::Triangle { base, height } => {
        // 假设是等腰三角形，计算两边长度
        let side = (height * height + (base / 2.0) * (base / 2.0)).sqrt();
        base + 2.0 * side
      }
    }
  }
}

/// 练习8: 颜色枚举
#[derive(Debug, PartialEq, Clone)]
enum Color {
  RGB(u8, u8, u8),
  HSV(f64, f64, f64),
  Named(String),
}

impl Color {
  /// 转换为RGB
  fn to_rgb(&self) -> RGB {
    match self {
      Color::RGB(r, g, b) => RGB(*r, *g, *b),
      Color::HSV(h, s, v) => {
        // 简化的HSV到RGB转换
        let c = v * s;
        let x = c * (1.0 - ((h / 60.0) % 2.0 - 1.0).abs());
        let m = v - c;

        let (r, g, b) = if *h < 60.0 {
          (c, x, 0.0)
        } else if *h < 120.0 {
          (x, c, 0.0)
        } else if *h < 180.0 {
          (0.0, c, x)
        } else if *h < 240.0 {
          (0.0, x, c)
        } else if *h < 300.0 {
          (x, 0.0, c)
        } else {
          (c, 0.0, x)
        };

        RGB(
          ((r + m) * 255.0) as u8,
          ((g + m) * 255.0) as u8,
          ((b + m) * 255.0) as u8,
        )
      }
      Color::Named(name) => {
        // 简单的命名颜色映射
        match name.to_lowercase().as_str() {
          "red" => RGB(255, 0, 0),
          "green" => RGB(0, 255, 0),
          "blue" => RGB(0, 0, 255),
          "white" => RGB(255, 255, 255),
          "black" => RGB(0, 0, 0),
          _ => RGB(128, 128, 128), // 默认灰色
        }
      }
    }
  }
}

// ============================================================================
// 练习9: Option和Result的使用
// ============================================================================

/// 练习9: 安全的数组访问
fn safe_get<T>(vec: &[T], index: usize) -> Option<&T> {
  vec.get(index)
}

/// 练习10: 安全的除法
fn safe_divide(dividend: f64, divisor: f64) -> Result<f64, String> {
  if divisor == 0.0 {
    Err("除数不能为零".to_string())
  } else if divisor.is_nan() || dividend.is_nan() {
    Err("输入不能为NaN".to_string())
  } else {
    Ok(dividend / divisor)
  }
}

/// 练习11: 解析配置
#[derive(Debug, PartialEq)]
struct Config {
  host: String,
  port: u16,
  debug: bool,
}

#[derive(Debug, PartialEq)]
enum ConfigError {
  MissingField(String),
  InvalidValue(String),
  ParseError(String),
}

impl Config {
  fn from_string(config_str: &str) -> Result<Config, ConfigError> {
    let mut host = None;
    let mut port = None;
    let mut debug = None;

    for line in config_str.lines() {
      let line = line.trim();
      if line.is_empty() || line.starts_with('#') {
        continue;
      }

      if let Some((key, value)) = line.split_once('=') {
        let key = key.trim();
        let value = value.trim();

        match key {
          "host" => host = Some(value.to_string()),
          "port" => {
            port = Some(
              value
                .parse()
                .map_err(|_| ConfigError::ParseError(format!("无法解析端口: {}", value)))?,
            );
          }
          "debug" => {
            debug = Some(
              value
                .parse()
                .map_err(|_| ConfigError::ParseError(format!("无法解析debug: {}", value)))?,
            );
          }
          _ => {} // 忽略未知字段
        }
      }
    }

    Ok(Config {
      host: host.ok_or_else(|| ConfigError::MissingField("host".to_string()))?,
      port: port.ok_or_else(|| ConfigError::MissingField("port".to_string()))?,
      debug: debug.unwrap_or(false),
    })
  }
}

// ============================================================================
// 练习12: 复杂的枚举应用
// ============================================================================

/// 练习12: 表达式求值器
#[derive(Debug, PartialEq, Clone)]
enum Expr {
  Number(f64),
  Add(Box<Expr>, Box<Expr>),
  Subtract(Box<Expr>, Box<Expr>),
  Multiply(Box<Expr>, Box<Expr>),
  Divide(Box<Expr>, Box<Expr>),
}

impl Expr {
  /// 求值
  fn eval(&self) -> Result<f64, String> {
    match self {
      Expr::Number(n) => Ok(*n),
      Expr::Add(left, right) => {
        let l = left.eval()?;
        let r = right.eval()?;
        Ok(l + r)
      }
      Expr::Subtract(left, right) => {
        let l = left.eval()?;
        let r = right.eval()?;
        Ok(l - r)
      }
      Expr::Multiply(left, right) => {
        let l = left.eval()?;
        let r = right.eval()?;
        Ok(l * r)
      }
      Expr::Divide(left, right) => {
        let l = left.eval()?;
        let r = right.eval()?;
        if r == 0.0 {
          Err("除数不能为零".to_string())
        } else {
          Ok(l / r)
        }
      }
    }
  }

  /// 简化表达式
  fn simplify(self) -> Expr {
    match self {
      Expr::Add(left, right) => {
        let left = left.simplify();
        let right = right.simplify();
        match (&left, &right) {
          (Expr::Number(0.0), _) => right,
          (_, Expr::Number(0.0)) => left,
          (Expr::Number(a), Expr::Number(b)) => Expr::Number(a + b),
          _ => Expr::Add(Box::new(left), Box::new(right)),
        }
      }
      Expr::Multiply(left, right) => {
        let left = left.simplify();
        let right = right.simplify();
        match (&left, &right) {
          (Expr::Number(0.0), _) | (_, Expr::Number(0.0)) => Expr::Number(0.0),
          (Expr::Number(1.0), _) => right,
          (_, Expr::Number(1.0)) => left,
          (Expr::Number(a), Expr::Number(b)) => Expr::Number(a * b),
          _ => Expr::Multiply(Box::new(left), Box::new(right)),
        }
      }
      _ => self, // 其他情况暂不简化
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
  fn test_point() {
    let mut p = Point::new(3.0, 4.0);
    assert_eq!(p.distance_from_origin(), 5.0);

    let p2 = Point::new(0.0, 0.0);
    assert_eq!(p.distance_to(&p2), 5.0);

    p.translate(1.0, 1.0);
    assert_eq!(p, Point::new(4.0, 5.0));
  }

  #[test]
  fn test_rectangle() {
    let mut rect = Rectangle::new(4.0, 3.0);
    assert_eq!(rect.area(), 12.0);
    assert_eq!(rect.perimeter(), 14.0);
    assert!(!rect.is_square());

    let square = Rectangle::square(5.0);
    assert!(square.is_square());

    rect.scale(2.0);
    assert_eq!(rect.area(), 48.0);
  }

  #[test]
  fn test_student() {
    let mut student = Student::new(1, "Alice".to_string(), 20);
    assert_eq!(student.average_grade(), None);

    student.add_grade(85.0);
    student.add_grade(90.0);
    student.add_grade(78.0);

    assert_eq!(student.average_grade(), Some(84.33333333333333));
    assert_eq!(student.highest_grade(), Some(90.0));
    assert!(student.is_passing());
  }

  #[test]
  fn test_rgb() {
    let color = RGB::new(255, 128, 0);
    assert_eq!(color.red(), 255);
    assert_eq!(color.green(), 128);
    assert_eq!(color.blue(), 0);
    assert_eq!(color.to_hex(), "#FF8000");
  }

  #[test]
  fn test_vector3d() {
    let v1 = Vector3D::new(1.0, 0.0, 0.0);
    let v2 = Vector3D::new(0.0, 1.0, 0.0);

    assert_eq!(v1.magnitude(), 1.0);
    assert_eq!(v1.dot(&v2), 0.0);
    assert_eq!(v1.cross(&v2), Vector3D::new(0.0, 0.0, 1.0));
  }

  #[test]
  fn test_direction() {
    let dir = Direction::North;
    assert_eq!(dir.opposite(), Direction::South);
    assert_eq!(dir.turn_right(), Direction::East);
    assert_eq!(dir.turn_left(), Direction::West);
  }

  #[test]
  fn test_shape() {
    let circle = Shape::Circle { radius: 2.0 };
    assert!((circle.area() - (std::f64::consts::PI * 4.0)).abs() < 1e-10);

    let rect = Shape::Rectangle {
      width: 3.0,
      height: 4.0,
    };
    assert_eq!(rect.area(), 12.0);
    assert_eq!(rect.perimeter(), 14.0);
  }

  #[test]
  fn test_color() {
    let red = Color::Named("red".to_string());
    assert_eq!(red.to_rgb(), RGB(255, 0, 0));

    let rgb_color = Color::RGB(100, 150, 200);
    assert_eq!(rgb_color.to_rgb(), RGB(100, 150, 200));
  }

  #[test]
  fn test_safe_get() {
    let vec = vec![1, 2, 3, 4, 5];
    assert_eq!(safe_get(&vec, 2), Some(&3));
    assert_eq!(safe_get(&vec, 10), None);
  }

  #[test]
  fn test_safe_divide() {
    assert_eq!(safe_divide(10.0, 2.0), Ok(5.0));
    assert!(safe_divide(10.0, 0.0).is_err());
  }

  #[test]
  fn test_config() {
    let config_str = "host=localhost\nport=8080\ndebug=true";
    let config = Config::from_string(config_str).unwrap();

    assert_eq!(config.host, "localhost");
    assert_eq!(config.port, 8080);
    assert_eq!(config.debug, true);
  }

  #[test]
  fn test_expr() {
    // 2 + 3 * 4
    let expr = Expr::Add(
      Box::new(Expr::Number(2.0)),
      Box::new(Expr::Multiply(
        Box::new(Expr::Number(3.0)),
        Box::new(Expr::Number(4.0)),
      )),
    );

    assert_eq!(expr.eval(), Ok(14.0));

    // 简化测试: 0 + 5 = 5
    let simple_expr = Expr::Add(Box::new(Expr::Number(0.0)), Box::new(Expr::Number(5.0)));

    let simplified = simple_expr.simplify();
    assert_eq!(simplified, Expr::Number(5.0));
  }
}

// ============================================================================
// 挑战练习
// ============================================================================

/// 挑战1: 实现一个简单的JSON值表示
#[derive(Debug, PartialEq, Clone)]
enum JsonValue {
  Null,
  Bool(bool),
  Number(f64),
  String(String),
  Array(Vec<JsonValue>),
  Object(std::collections::HashMap<String, JsonValue>),
}

impl JsonValue {
  /// 获取类型名称
  fn type_name(&self) -> &'static str {
    match self {
      JsonValue::Null => "null",
      JsonValue::Bool(_) => "boolean",
      JsonValue::Number(_) => "number",
      JsonValue::String(_) => "string",
      JsonValue::Array(_) => "array",
      JsonValue::Object(_) => "object",
    }
  }

  /// 判断是否为真值
  fn is_truthy(&self) -> bool {
    match self {
      JsonValue::Null => false,
      JsonValue::Bool(b) => *b,
      JsonValue::Number(n) => *n != 0.0,
      JsonValue::String(s) => !s.is_empty(),
      JsonValue::Array(arr) => !arr.is_empty(),
      JsonValue::Object(obj) => !obj.is_empty(),
    }
  }
}

/// 挑战2: 实现一个状态机
#[derive(Debug, PartialEq, Clone)]
enum State {
  Idle,
  Processing,
  Complete,
  Error(String),
}

#[derive(Debug, PartialEq, Clone)]
enum Event {
  Start,
  Process,
  Finish,
  Fail(String),
  Reset,
}

struct StateMachine {
  current_state: State,
}

impl StateMachine {
  fn new() -> Self {
    StateMachine {
      current_state: State::Idle,
    }
  }

  fn handle_event(&mut self, event: Event) -> Result<(), String> {
    let new_state = match (&self.current_state, event) {
      (State::Idle, Event::Start) => State::Processing,
      (State::Processing, Event::Process) => State::Processing,
      (State::Processing, Event::Finish) => State::Complete,
      (State::Processing, Event::Fail(msg)) => State::Error(msg),
      (_, Event::Reset) => State::Idle,
      _ => return Err(format!("无效的状态转换: {:?}", self.current_state)),
    };

    self.current_state = new_state;
    Ok(())
  }

  fn current_state(&self) -> &State {
    &self.current_state
  }
}

#[cfg(test)]
mod challenge_tests {
  use super::*;

  #[test]
  fn test_json_value() {
    let null_val = JsonValue::Null;
    assert_eq!(null_val.type_name(), "null");
    assert!(!null_val.is_truthy());

    let bool_val = JsonValue::Bool(true);
    assert_eq!(bool_val.type_name(), "boolean");
    assert!(bool_val.is_truthy());

    let empty_array = JsonValue::Array(vec![]);
    assert!(!empty_array.is_truthy());

    let non_empty_array = JsonValue::Array(vec![JsonValue::Number(1.0)]);
    assert!(non_empty_array.is_truthy());
  }

  #[test]
  fn test_state_machine() {
    let mut sm = StateMachine::new();
    assert_eq!(sm.current_state(), &State::Idle);

    sm.handle_event(Event::Start).unwrap();
    assert_eq!(sm.current_state(), &State::Processing);

    sm.handle_event(Event::Finish).unwrap();
    assert_eq!(sm.current_state(), &State::Complete);

    sm.handle_event(Event::Reset).unwrap();
    assert_eq!(sm.current_state(), &State::Idle);

    // 测试错误状态
    sm.handle_event(Event::Start).unwrap();
    sm.handle_event(Event::Fail("测试错误".to_string()))
      .unwrap();
    assert_eq!(sm.current_state(), &State::Error("测试错误".to_string()));
  }
}

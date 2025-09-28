// Rust宏系统示例：展示声明式宏、过程宏等高级特性

// 1. 基础声明式宏
macro_rules! say_hello {
  () => {
    println!("Hello, World!");
  };
}

// 2. 带参数的宏
macro_rules! create_function {
  ($func_name:ident) => {
    fn $func_name() {
      println!("你调用了函数: {}", stringify!($func_name));
    }
  };
}

// 3. 重载宏（多个模式）
macro_rules! test {
  ($left:expr; and $right:expr) => {
    println!(
      "{:?} and {:?} is {:?}",
      stringify!($left),
      stringify!($right),
      $left && $right
    )
  };
  ($left:expr; or $right:expr) => {
    println!(
      "{:?} or {:?} is {:?}",
      stringify!($left),
      stringify!($right),
      $left || $right
    )
  };
}

// 4. 可变参数宏
macro_rules! find_min {
    ($x:expr) => ($x);
    ($x:expr, $($y:expr),+) => (
        std::cmp::min($x, find_min!($($y),+))
    )
}

// 5. 创建数据结构的宏
macro_rules! hash_map {
    ($($key:expr => $val:expr),*) => {
        {
            let mut map = std::collections::HashMap::new();
            $(
                map.insert($key, $val);
            )*
            map
        }
    };
}

// 6. 条件编译宏
macro_rules! debug_print {
    ($($arg:tt)*) => {
        #[cfg(debug_assertions)]
        println!("[DEBUG] {}", format!($($arg)*));
    };
}

// 7. 生成测试用例的宏
macro_rules! test_case {
  ($name:ident: $input:expr => $expected:expr) => {
    #[test]
    fn $name() {
      assert_eq!(double($input), $expected);
    }
  };
}

fn double(x: i32) -> i32 {
  x * 2
}

// 8. 创建枚举和匹配的宏
macro_rules! create_enum {
    ($name:ident { $($variant:ident),* }) => {
        #[derive(Debug, PartialEq)]
        enum $name {
            $($variant),*
        }

        impl $name {
            fn variants() -> Vec<$name> {
                vec![$($name::$variant),*]
            }

            fn name(&self) -> &'static str {
                match self {
                    $($name::$variant => stringify!($variant)),*
                }
            }
        }
    };
}

// 9. 实现特征的宏
macro_rules! impl_display {
  ($type:ty) => {
    impl std::fmt::Display for $type {
      fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{:?}", self)
      }
    }
  };
}

// 10. 日志宏
macro_rules! log {
    (info: $($arg:tt)*) => {
        println!("[INFO] {}", format!($($arg)*));
    };
    (warn: $($arg:tt)*) => {
        println!("[WARN] {}", format!($($arg)*));
    };
    (error: $($arg:tt)*) => {
        eprintln!("[ERROR] {}", format!($($arg)*));
    };
}

// 11. 计算宏（编译时计算）
macro_rules! calculate {
    (eval $e:expr) => {
        {
            let val: usize = $e; // 强制编译时计算
            val
        }
    };
}

// 12. 生成getter和setter的宏
macro_rules! getter_setter {
  ($field:ident: $type:ty) => {
    paste::paste! {
        pub fn [<get_ $field>](&self) -> &$type {
            &self.$field
        }

        pub fn [<set_ $field>](&mut self, value: $type) {
            self.$field = value;
        }
    }
  };
}

// 13. 单例模式宏
macro_rules! singleton {
  ($name:ident, $type:ty, $init:expr) => {
    pub struct $name;

    impl $name {
      pub fn instance() -> &'static $type {
        static mut INSTANCE: Option<$type> = None;
        static ONCE: std::sync::Once = std::sync::Once::new();

        unsafe {
          ONCE.call_once(|| {
            INSTANCE = Some($init);
          });
          INSTANCE.as_ref().unwrap()
        }
      }
    }
  };
}

// 14. 构建器模式宏
macro_rules! builder {
    ($name:ident {
        $($field:ident: $type:ty),*
    }) => {
        paste::paste! {
            #[derive(Default)]
            pub struct [<$name Builder>] {
                $($field: Option<$type>),*
            }

            impl [<$name Builder>] {
                pub fn new() -> Self {
                    Self::default()
                }

                $(
                    pub fn $field(mut self, value: $type) -> Self {
                        self.$field = Some(value);
                        self
                    }
                )*

                pub fn build(self) -> Result<$name, String> {
                    Ok($name {
                        $(
                            $field: self.$field.ok_or_else(||
                                format!("Missing field: {}", stringify!($field))
                            )?
                        ),*
                    })
                }
            }

            #[derive(Debug)]
            pub struct $name {
                $($field: $type),*
            }
        }
    };
}

// 15. 状态机宏
macro_rules! state_machine {
    ($name:ident {
        states: [$($state:ident),*],
        transitions: {
            $($from:ident -> $to:ident on $event:ident),*
        }
    }) => {
        paste::paste! {
            #[derive(Debug, Clone, PartialEq)]
            pub enum [<$name State>] {
                $($state),*
            }

            #[derive(Debug, Clone)]
            pub enum [<$name Event>] {
                $($event),*
            }

            pub struct $name {
                state: [<$name State>],
            }

            impl $name {
                pub fn new(initial_state: [<$name State>]) -> Self {
                    Self { state: initial_state }
                }

                pub fn current_state(&self) -> &[<$name State>] {
                    &self.state
                }

                pub fn handle_event(&mut self, event: [<$name Event>]) -> Result<(), String> {
                    let new_state = match (&self.state, &event) {
                        $(
                            ([<$name State>]::$from, [<$name Event>]::$event) =>
                                [<$name State>]::$to,
                        )*
                        _ => return Err(format!("Invalid transition from {:?} on {:?}",
                            self.state, event)),
                    };

                    self.state = new_state;
                    Ok(())
                }
            }
        }
    };
}

// 使用宏创建示例
create_function!(foo);
create_function!(bar);

create_enum!(Color { Red, Green, Blue });

#[derive(Debug)]
struct Point {
  x: i32,
  y: i32,
}

impl_display!(Point);

// 使用构建器宏
builder!(Person {
  name: String,
  age: u32,
  email: String
});

// 使用状态机宏
state_machine!(TrafficLight {
    states: [Red, Yellow, Green],
    transitions: {
        Red -> Green on Go,
        Green -> Yellow on Caution,
        Yellow -> Red on Stop
    }
});

// 使用单例宏
singleton!(Config, std::collections::HashMap<String, String>, {
    let mut map = std::collections::HashMap::new();
    map.insert("version".to_string(), "1.0.0".to_string());
    map.insert("debug".to_string(), "true".to_string());
    map
});

fn main() {
  println!("=== Rust宏系统示例 ===\n");

  // 1. 基础宏使用
  say_hello!();

  // 2. 函数生成宏
  foo();
  bar();

  // 3. 重载宏
  test!(1i32 + 1 == 2i32; and 2i32 * 2 == 4i32);
  test!(true; or false);

  // 4. 可变参数宏
  println!("最小值: {}", find_min!(1u32, 2u32, 3u32, 4u32));

  // 5. HashMap创建宏
  let map = hash_map! {
      "name" => "Alice",
      "age" => "30",
      "city" => "Beijing"
  };
  println!("HashMap: {:?}", map);

  // 6. 调试打印宏
  debug_print!("这是一个调试消息: {}", 42);

  // 7. 枚举使用
  let color = Color::Red;
  println!("颜色: {:?}, 名称: {}", color, color.name());
  println!("所有颜色: {:?}", Color::variants());

  // 8. Display特征
  let point = Point { x: 10, y: 20 };
  println!("点: {}", point);

  // 9. 日志宏
  log!(info: "应用程序启动");
  log!(warn: "内存使用率: {}%", 85);
  log!(error: "连接失败: {}", "timeout");

  // 10. 编译时计算
  const RESULT: usize = calculate!(eval 2 + 3 * 4);
  println!("编译时计算结果: {}", RESULT);

  // 11. 构建器模式
  match PersonBuilder::new()
    .name("张三".to_string())
    .age(25)
    .email("zhangsan@example.com".to_string())
    .build()
  {
    Ok(person) => println!("构建的人员: {:?}", person),
    Err(e) => println!("构建失败: {}", e),
  }

  // 12. 状态机
  let mut traffic_light = TrafficLight::new(TrafficLightState::Red);
  println!("初始状态: {:?}", traffic_light.current_state());

  match traffic_light.handle_event(TrafficLightEvent::Go) {
    Ok(()) => println!("转换后状态: {:?}", traffic_light.current_state()),
    Err(e) => println!("状态转换失败: {}", e),
  }

  // 13. 单例使用
  let config = Config::instance();
  println!("配置: {:?}", config);

  println!("\n=== 宏的优势 ===");
  println!("1. 代码生成和重复消除");
  println!("2. 编译时计算和优化");
  println!("3. DSL（领域特定语言）创建");
  println!("4. 类型安全的代码生成");
  println!("5. 零运行时开销");
}

// 宏调试和展开示例
macro_rules! debug_macro {
    ($($arg:tt)*) => {
        {
            println!("宏输入: {}", stringify!($($arg)*));
            $($arg)*
        }
    };
}

// 高级宏技巧
macro_rules! count {
    () => (0usize);
    ( $x:tt $($xs:tt)* ) => (1usize + count!($($xs)*));
}

macro_rules! reverse {
    ([] $($reversed:tt)*) => {
        [$($reversed)*]
    };
    ([$first:tt $($rest:tt)*] $($reversed:tt)*) => {
        reverse!([$($rest)*] $first $($reversed)*)
    };
}

#[cfg(test)]
mod tests {
  use super::*;

  // 使用测试宏
  test_case!(test_double_2: 2 => 4);
  test_case!(test_double_5: 5 => 10);
  test_case!(test_double_0: 0 => 0);

  #[test]
  fn test_find_min() {
    assert_eq!(find_min!(3, 1, 4, 1, 5), 1);
  }

  #[test]
  fn test_hash_map_macro() {
    let map = hash_map! {"a" => 1, "b" => 2};
    assert_eq!(map.get("a"), Some(&1));
    assert_eq!(map.get("b"), Some(&2));
  }

  #[test]
  fn test_count_macro() {
    assert_eq!(count!(), 0);
    assert_eq!(count!(a), 1);
    assert_eq!(count!(a b c), 3);
  }

  #[test]
  fn test_color_enum() {
    let red = Color::Red;
    assert_eq!(red.name(), "Red");
    assert_eq!(Color::variants().len(), 3);
  }
}

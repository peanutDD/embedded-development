//! # 函数式编程工具
//! 
//! 本模块提供了函数式编程的工具和示例，专为嵌入式开发优化。
//! 
//! ## 特性
//! 
//! - **高阶函数**: 函数作为参数和返回值
//! - **闭包优化**: 零成本抽象的闭包
//! - **函数组合**: 函数组合和管道操作
//! - **惰性求值**: 迭代器和惰性计算

use core::marker::PhantomData;

/// 函数组合器，用于组合两个函数
pub struct Compose<F, G, A, B, C> {
    f: F,
    g: G,
    _phantom: PhantomData<(A, B, C)>,
}

impl<F, G, A, B, C> Compose<F, G, A, B, C>
where
    F: Fn(A) -> B,
    G: Fn(B) -> C,
{
    /// 创建新的函数组合器
    pub fn new(f: F, g: G) -> Self {
        Self {
            f,
            g,
            _phantom: PhantomData,
        }
    }
    
    /// 执行组合函数
    pub fn call(&self, input: A) -> C {
        (self.g)((self.f)(input))
    }
}

/// 管道操作符，用于链式调用
pub trait Pipe<T> {
    /// 将值传递给函数
    fn pipe<F, U>(self, f: F) -> U
    where
        F: FnOnce(T) -> U;
}

impl<T> Pipe<T> for T {
    fn pipe<F, U>(self, f: F) -> U
    where
        F: FnOnce(T) -> U,
    {
        f(self)
    }
}

/// 柯里化函数工具
pub struct Curry<F, A, B, C> {
    f: F,
    _phantom: PhantomData<(A, B, C)>,
}

impl<F, A, B, C> Curry<F, A, B, C> {
    /// 创建柯里化函数
    pub fn new(f: F) -> Self {
        Self { 
            f,
            _phantom: PhantomData,
        }
    }
}

impl<F, A, B, C> Curry<F, A, B, C>
where
    F: Fn(A, B) -> C,
    A: Clone + 'static,
{
    /// 部分应用第一个参数
    pub fn partial(&self, a: A) -> impl Fn(B) -> C + '_
    {
        move |b| (self.f)(a.clone(), b)
    }
}

/// 记忆化函数装饰器
#[cfg(feature = "std")]
pub struct Memoize<F, K, V> {
    f: F,
    cache: std::collections::HashMap<K, V>,
}

#[cfg(feature = "std")]
impl<F, K, V> Memoize<F, K, V>
where
    K: std::hash::Hash + Eq + Clone,
    V: Clone,
    F: Fn(&K) -> V,
{
    /// 创建记忆化函数
    pub fn new(f: F) -> Self {
        Self {
            f,
            cache: std::collections::HashMap::new(),
        }
    }
    
    /// 调用记忆化函数
    pub fn call(&mut self, key: K) -> V {
        if let Some(value) = self.cache.get(&key) {
            value.clone()
        } else {
            let value = (self.f)(&key);
            self.cache.insert(key, value.clone());
            value
        }
    }
}

/// 函数式编程工具函数
pub mod utils {
    /// 恒等函数
    pub fn identity<T>(x: T) -> T {
        x
    }
    
    /// 常量函数
    pub fn constant<T: Clone>(value: T) -> impl Fn() -> T {
        move || value.clone()
    }
    
    /// 翻转二元函数的参数顺序
    pub fn flip<F, A, B, C>(f: F) -> impl Fn(B, A) -> C
    where
        F: Fn(A, B) -> C,
    {
        move |b, a| f(a, b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_compose() {
        let add_one = |x: i32| x + 1;
        let multiply_two = |x: i32| x * 2;
        
        let composed = Compose::new(add_one, multiply_two);
        assert_eq!(composed.call(5), 12); // (5 + 1) * 2 = 12
    }
    
    #[test]
    fn test_pipe() {
        let result = 5
            .pipe(|x| x + 1)
            .pipe(|x| x * 2);
        
        assert_eq!(result, 12);
    }
    
    #[test]
    fn test_curry() {
        let add = |a: i32, b: i32| a + b;
        let curried = Curry::new(add);
        let add_five = curried.partial(5);
        
        assert_eq!(add_five(3), 8);
    }
    
    #[test]
    fn test_utils() {
        assert_eq!(utils::identity(42), 42);
        
        let const_fn = utils::constant(100);
        assert_eq!(const_fn(), 100);
        
        let subtract = |a: i32, b: i32| a - b;
        let flipped = utils::flip(subtract);
        assert_eq!(flipped(3, 10), 7); // 10 - 3 = 7
    }
}
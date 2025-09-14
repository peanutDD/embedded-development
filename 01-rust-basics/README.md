# Rust 基础语法教程

## 概述

本章节将全面介绍Rust编程语言的基础语法，为嵌入式开发打下坚实的基础。我们将从最基本的概念开始，逐步深入到高级特性。

## 学习目标

完成本章节后，你将掌握：
- Rust的基本语法和数据类型
- 所有权系统和借用检查
- 模式匹配和错误处理
- 泛型、特征和生命周期
- 并发编程基础
- 宏系统和元编程

## 章节内容

### 1. [变量与数据类型](./01-variables-types.md)
- 变量声明和可变性
- 基本数据类型
- 复合数据类型
- 类型推断和类型注解

### 2. [所有权系统](./02-ownership.md)
- 所有权规则
- 移动语义
- 借用和引用
- 生命周期基础

### 3. [控制流](./03-control-flow.md)
- 条件语句
- 循环结构
- 模式匹配
- 错误处理

### 4. [函数与闭包](./04-functions-closures.md)
- 函数定义和调用
- 参数传递
- 返回值
- 闭包和函数指针

### 5. [结构体与枚举](./05-structs-enums.md)
- 结构体定义和使用
- 方法和关联函数
- 枚举类型
- Option和Result

### 6. [集合类型](./06-collections.md)
- Vector
- HashMap
- 字符串处理
- 迭代器

### 7. [泛型与特征](./07-generics-traits.md)
- 泛型函数和结构体
- 特征定义和实现
- 特征对象
- 关联类型

### 8. [生命周期详解](./08-lifetimes.md)
- 生命周期注解
- 生命周期省略规则
- 静态生命周期
- 高级生命周期特性

### 9. [错误处理](./09-error-handling.md)
- Result类型详解
- 错误传播
- 自定义错误类型
- 错误处理最佳实践

### 10. [模块系统](./10-modules.md)
- 模块定义和使用
- 可见性规则
- use语句
- 包和crate

### 11. [并发编程](./11-concurrency.md)
- 线程创建和管理
- 消息传递
- 共享状态并发
- Sync和Send特征

### 12. [智能指针](./12-smart-pointers.md)
- Box<T>
- Rc<T>和Arc<T>
- RefCell<T>和Mutex<T>
- 自定义智能指针

### 13. [宏系统](./13-macros.md)
- 声明式宏
- 过程宏
- 宏的高级用法
- 元编程技巧

### 14. [异步编程](./14-async-programming.md)
- async/await语法
- Future特征
- 异步运行时
- 异步编程模式

### 15. [unsafe Rust](./15-unsafe-rust.md)
- unsafe关键字
- 原始指针
- 调用unsafe函数
- 实现unsafe特征

## 实践项目

每个章节都包含相应的代码示例和练习项目：

- **基础练习**: 巩固语法知识的小程序
- **综合项目**: 结合多个概念的实战项目
- **嵌入式预备**: 为嵌入式开发做准备的特定练习

## 代码示例结构

```
01-rust-basics/
├── examples/
│   ├── 01-variables/
│   ├── 02-ownership/
│   ├── 03-control-flow/
│   └── ...
├── exercises/
│   ├── basic/
│   ├── intermediate/
│   └── advanced/
└── projects/
    ├── calculator/
    ├── text-processor/
    └── mini-database/
```

## 学习建议

1. **循序渐进**: 按照章节顺序学习，每个概念都建立在前面的基础上
2. **动手实践**: 运行所有代码示例，完成练习题
3. **理解原理**: 不仅要知道怎么写，更要理解为什么这样写
4. **查阅文档**: 养成查阅官方文档的习惯
5. **编写测试**: 为你的代码编写单元测试

## 嵌入式相关重点

在学习过程中，特别注意以下与嵌入式开发相关的概念：

- **零成本抽象**: Rust如何在不牺牲性能的情况下提供高级特性
- **内存安全**: 如何在没有垃圾回收器的情况下保证内存安全
- **编译时检查**: 如何利用类型系统在编译时捕获错误
- **资源管理**: RAII模式和自动资源管理
- **并发安全**: 如何安全地处理并发和中断

## 推荐资源

- [The Rust Programming Language](https://doc.rust-lang.org/book/)
- [Rust by Example](https://doc.rust-lang.org/rust-by-example/)
- [The Rustonomicon](https://doc.rust-lang.org/nomicon/)
- [Rust Reference](https://doc.rust-lang.org/reference/)

---

**准备好开始你的Rust学习之旅了吗？让我们从第一章开始！** 🦀
# Rust基础练习题

本目录包含了Rust基础概念的练习题，分为基础练习和高级练习两个级别。

## 练习文件说明

### 1. basic_exercises.rs
- **难度级别**: 初级到中级
- **涵盖章节**: 第1-6章基础概念
- **练习数量**: 20个基础练习 + 2个挑战练习
- **主要内容**:
  - 变量与数据类型
  - 控制流结构
  - 函数定义与调用
  - 所有权与借用
  - 结构体与枚举
  - 集合类型操作

### 2. advanced_exercises.rs
- **难度级别**: 中级到高级
- **涵盖章节**: 第7-15章高级概念
- **练习数量**: 20个高级练习 + 2个超级挑战
- **主要内容**:
  - 泛型与特征
  - 生命周期管理
  - 智能指针应用
  - 并发编程
  - 宏编程
  - 不安全Rust

## 如何使用练习题

### 运行练习
```bash
# 运行基础练习
cargo run --bin basic_exercises

# 运行高级练习
cargo run --bin advanced_exercises

# 运行特定练习函数
cargo test exercise_1 --bin basic_exercises
```

### 测试练习
```bash
# 运行所有测试
cargo test

# 运行特定文件的测试
cargo test --bin basic_exercises
cargo test --bin advanced_exercises

# 运行特定练习的测试
cargo test test_exercise_1
```

### 调试练习
```bash
# 使用调试模式运行
cargo run --bin basic_exercises -- --debug

# 查看详细输出
RUST_LOG=debug cargo run --bin basic_exercises
```

## 练习完成指南

### 基础练习 (basic_exercises.rs)

**第1-5题**: 变量与数据类型
- 重点理解可变性、类型推断、类型转换
- 练习不同数据类型的使用场景

**第6-10题**: 控制流与函数
- 掌握if/else、loop、while、for的使用
- 理解函数参数传递和返回值

**第11-15题**: 所有权系统
- 深入理解move、borrow、lifetime
- 练习引用和可变引用的规则

**第16-20题**: 复合数据类型
- 熟练使用struct、enum、Vec、HashMap
- 理解模式匹配和解构

**挑战题**: 综合应用
- 结合多个概念解决复杂问题
- 注重代码的可读性和性能

### 高级练习 (advanced_exercises.rs)

**第1-5题**: 泛型与特征
- 掌握泛型函数和结构体的定义
- 理解特征的定义和实现

**第6-10题**: 生命周期
- 理解生命周期参数的作用
- 解决复杂的借用检查问题

**第11-15题**: 智能指针
- 熟练使用Box、Rc、RefCell
- 理解内部可变性和循环引用

**第16-20题**: 并发编程
- 掌握线程、通道、共享状态
- 理解Send和Sync特征

**超级挑战**: 高级应用
- 宏编程和元编程
- 不安全Rust的正确使用

## 学习策略

### 1. 循序渐进
- 先完成基础练习，再进行高级练习
- 每完成一个练习，理解其背后的概念

### 2. 实践导向
- 不要只是看答案，尝试自己实现
- 运行代码，观察输出和错误信息

### 3. 深入理解
- 使用`cargo expand`查看宏展开
- 使用`cargo asm`查看生成的汇编代码
- 使用`cargo bench`测试性能

### 4. 错误学习
- 仔细阅读编译器错误信息
- 理解借用检查器的提示
- 学会使用`clippy`进行代码检查

## 性能分析

### 基准测试
```bash
# 运行性能基准测试
cargo bench

# 生成性能报告
cargo bench -- --output-format html
```

### 内存分析
```bash
# 使用valgrind分析内存使用
valgrind --tool=massif cargo run --bin basic_exercises

# 使用heaptrack分析堆内存
heaptrack cargo run --bin basic_exercises
```

## 嵌入式开发重点

这些练习特别强调了嵌入式开发中的关键概念：

### 内存效率
- 零分配算法实现
- 栈内存优化技巧
- 编译时计算

### 性能可预测性
- 避免动态分配
- 理解编译器优化
- 控制代码大小

### 安全性
- 类型安全保证
- 内存安全检查
- 并发安全模式

## 常见问题解答

### Q: 练习题太难怎么办？
A: 建议先回顾对应章节的理论知识，然后查看示例代码，最后再尝试练习。

### Q: 如何检查答案是否正确？
A: 每个练习都有对应的测试用例，运行`cargo test`即可验证。

### Q: 可以修改练习题吗？
A: 当然可以！鼓励你修改和扩展练习题，这是最好的学习方式。

### Q: 练习题与嵌入式开发有什么关系？
A: 这些练习强调了嵌入式开发中的核心概念：内存安全、性能可预测、零成本抽象等。

## 下一步

完成练习后，建议：
1. 查看[项目实践](../projects/README.md)进行综合应用
2. 学习[嵌入式环境搭建](../../02-embedded-setup/README.md)
3. 参与开源项目，应用所学知识
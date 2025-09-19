# Rust基础示例代码

本目录包含了Rust基础概念的实用示例代码，每个文件都对应相关的章节内容。

## 示例文件说明

### 1. ownership_demo.rs
- **对应章节**: 第3章 - 所有权系统
- **内容**: 演示Rust的所有权、借用、生命周期等核心概念
- **运行方式**: `cargo run --example ownership_demo`

### 2. functions_closures_demo.rs
- **对应章节**: 第4章 - 函数与闭包
- **内容**: 展示函数定义、闭包使用、高阶函数等
- **运行方式**: `cargo run --example functions_closures_demo`

### 3. structs_enums_demo.rs
- **对应章节**: 第5章 - 结构体与枚举
- **内容**: 结构体、枚举、模式匹配的实际应用
- **运行方式**: `cargo run --example structs_enums_demo`

### 4. collections_demo.rs
- **对应章节**: 第6章 - 集合类型
- **内容**: Vector、HashMap、HashSet等集合的使用技巧
- **运行方式**: `cargo run --example collections_demo`

### 5. error_handling_demo.rs
- **对应章节**: 第8章 - 错误处理
- **内容**: Result、Option、错误传播的最佳实践
- **运行方式**: `cargo run --example error_handling_demo`

## 如何运行示例

### 方法1: 使用cargo run
```bash
# 运行特定示例
cargo run --example ownership_demo
cargo run --example functions_closures_demo
# ... 其他示例
```

### 方法2: 直接编译运行
```bash
# 编译单个文件
rustc examples/ownership_demo.rs -o ownership_demo
./ownership_demo
```

### 方法3: 使用rustc查看编译输出
```bash
# 查看编译后的汇编代码
rustc --emit asm examples/ownership_demo.rs
```

## 学习建议

1. **按顺序学习**: 建议按照文件编号顺序学习，每个示例都建立在前面的基础上
2. **动手实践**: 不要只是阅读代码，尝试修改和运行每个示例
3. **理解输出**: 仔细观察每个示例的输出，理解Rust的行为
4. **实验变化**: 尝试修改代码参数，观察不同的结果
5. **性能分析**: 使用`cargo bench`或`perf`工具分析性能特征

## 嵌入式开发相关

这些示例特别关注了嵌入式开发中的重要概念：

- **零成本抽象**: 展示Rust如何在编译时优化高级抽象
- **内存安全**: 演示如何在不使用垃圾回收的情况下保证内存安全
- **性能可预测**: 展示Rust代码的性能特征和优化技巧
- **资源管理**: 演示RAII和智能指针在资源管理中的应用

## 下一步

完成这些示例后，建议继续学习：
- [练习题目录](../exercises/README.md) - 巩固所学知识
- [项目实践](../projects/README.md) - 综合应用练习
- [嵌入式环境搭建](../../02-embedded-setup/README.md) - 进入嵌入式开发
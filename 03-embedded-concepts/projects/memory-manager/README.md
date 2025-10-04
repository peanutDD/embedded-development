# 嵌入式内存管理器

本项目展示了嵌入式系统中各种内存管理技术的实现，包括不同类型的内存分配器、内存池管理、内存保护和性能优化技术。

## 项目概述

嵌入式系统的内存管理面临着独特的挑战：
- 有限的内存资源
- 实时性要求
- 内存碎片问题
- 确定性分配需求
- 功耗考虑

本项目提供了多种内存管理策略的实现和比较。

## 功能特性

### 内存分配器
- **Buddy分配器**: 减少外部碎片的经典算法
- **Slab分配器**: 针对固定大小对象的高效分配
- **Pool分配器**: 预分配内存池管理
- **Stack分配器**: 栈式内存分配

### 内存管理功能
- 内存碎片分析和整理
- 内存使用统计和监控
- 内存泄漏检测
- 内存访问保护
- 动态内存调试

### 性能优化
- 零分配算法实现
- 内存预分配策略
- 缓存友好的数据结构
- 内存对齐优化

## 项目结构

```
memory-manager/
├── src/
│   ├── lib.rs                 # 库入口
│   ├── allocators/            # 各种分配器实现
│   │   ├── mod.rs
│   │   ├── buddy.rs           # Buddy分配器
│   │   ├── slab.rs            # Slab分配器
│   │   ├── pool.rs            # Pool分配器
│   │   └── stack.rs           # Stack分配器
│   ├── memory/                # 内存管理核心
│   │   ├── mod.rs
│   │   ├── manager.rs         # 内存管理器
│   │   ├── tracker.rs         # 内存跟踪
│   │   ├── statistics.rs      # 统计信息
│   │   └── protection.rs      # 内存保护
│   ├── utils/                 # 工具函数
│   │   ├── mod.rs
│   │   ├── alignment.rs       # 内存对齐
│   │   ├── bitmap.rs          # 位图操作
│   │   └── debug.rs           # 调试工具
│   └── bin/                   # 示例程序
│       ├── buddy_allocator_demo.rs
│       ├── slab_allocator_demo.rs
│       ├── pool_allocator_demo.rs
│       └── memory_benchmark.rs
├── tests/                     # 测试用例
├── benches/                   # 性能测试
├── memory.x                   # 内存布局
└── README.md
```

## 快速开始

### 构建项目

```bash
# 构建所有组件
cargo build

# 构建发布版本
cargo build --release

# 构建特定分配器
cargo build --features buddy-allocator
```

### 运行示例

```bash
# Buddy分配器演示
cargo run --bin buddy_allocator_demo

# Slab分配器演示
cargo run --bin slab_allocator_demo

# 内存池演示
cargo run --bin pool_allocator_demo

# 性能基准测试
cargo run --bin memory_benchmark --release
```

### 运行测试

```bash
# 运行所有测试
cargo test

# 运行特定测试
cargo test buddy_allocator
cargo test memory_manager

# 运行性能测试
cargo bench
```

## 内存分配器详解

### 1. Buddy分配器

Buddy分配器通过将内存分割成2的幂次大小的块来管理内存，有效减少外部碎片。

**特点**:
- 快速分配和释放
- 减少外部碎片
- 支持内存合并
- 适合通用内存分配

**使用场景**:
- 通用内存管理
- 大小变化较大的对象
- 需要内存合并的场景

### 2. Slab分配器

Slab分配器为固定大小的对象提供高效的分配和释放。

**特点**:
- O(1)时间复杂度
- 无内部碎片
- 缓存友好
- 适合频繁分配/释放

**使用场景**:
- 网络数据包缓冲区
- 任务控制块
- 消息队列节点

### 3. Pool分配器

Pool分配器预先分配一大块内存，然后从中分配固定大小的块。

**特点**:
- 确定性分配时间
- 无动态分配开销
- 内存使用可预测
- 适合实时系统

**使用场景**:
- 实时系统
- 安全关键应用
- 资源受限环境

### 4. Stack分配器

Stack分配器按栈的方式管理内存，只能按LIFO顺序释放。

**特点**:
- 极快的分配速度
- 零碎片
- 简单实现
- 有序释放限制

**使用场景**:
- 临时对象分配
- 函数调用栈
- 嵌套作用域

## 内存管理最佳实践

### 1. 选择合适的分配器

```rust
// 根据使用模式选择分配器
match allocation_pattern {
    AllocationPattern::VariableSize => use_buddy_allocator(),
    AllocationPattern::FixedSize => use_slab_allocator(),
    AllocationPattern::Predictable => use_pool_allocator(),
    AllocationPattern::Temporary => use_stack_allocator(),
}
```

### 2. 内存预分配

```rust
// 在系统初始化时预分配内存
fn system_init() {
    // 预分配网络缓冲区
    network_buffer_pool.preallocate(100);
    
    // 预分配任务栈
    task_stack_pool.preallocate(10);
}
```

### 3. 内存监控

```rust
// 定期检查内存使用情况
fn memory_health_check() {
    let stats = memory_manager.get_statistics();
    
    if stats.fragmentation_ratio > 0.3 {
        // 触发内存整理
        memory_manager.defragment();
    }
    
    if stats.usage_ratio > 0.9 {
        // 内存使用率过高，触发警告
        trigger_low_memory_warning();
    }
}
```

## 性能优化技巧

### 1. 内存对齐

```rust
// 确保数据结构按缓存行对齐
#[repr(align(64))]
struct CacheAlignedData {
    data: [u8; 64],
}
```

### 2. 零分配设计

```rust
// 使用栈分配的数据结构
use heapless::Vec;

fn process_data() -> heapless::Vec<u8, 256> {
    let mut buffer = heapless::Vec::new();
    // 处理数据，无堆分配
    buffer
}
```

### 3. 内存池复用

```rust
// 复用内存池中的对象
impl ObjectPool<T> {
    fn get_or_create(&mut self) -> &mut T {
        self.pool.pop().unwrap_or_else(|| {
            T::new()
        })
    }
    
    fn return_object(&mut self, obj: T) {
        if self.pool.len() < self.max_size {
            self.pool.push(obj);
        }
    }
}
```

## 调试和分析

### 内存泄漏检测

```bash
# 启用内存跟踪
cargo run --features allocation-tracking

# 生成内存使用报告
cargo run --bin memory_analyzer
```

### 性能分析

```bash
# 运行性能基准测试
cargo bench

# 生成性能报告
cargo run --bin memory_benchmark --release > performance_report.txt
```

## 配置选项

### 编译时配置

```toml
[features]
# 启用调试功能
debug = ["memory-debug", "allocation-tracking"]

# 选择分配器
buddy = ["buddy-allocator"]
slab = ["slab-allocator"]
pool = ["pool-allocator"]
```

### 运行时配置

```rust
// 配置内存管理器
let config = MemoryManagerConfig {
    heap_size: 64 * 1024,           // 64KB堆
    max_allocations: 1000,          // 最大分配数
    enable_statistics: true,        // 启用统计
    enable_protection: false,       // 禁用保护（性能优化）
};
```

## 注意事项

1. **实时性**: 某些分配器可能有不确定的执行时间
2. **内存碎片**: 长时间运行可能导致内存碎片
3. **线程安全**: 多线程环境需要额外的同步机制
4. **调试开销**: 调试功能会影响性能
5. **内存对齐**: 不正确的对齐可能影响性能

## 扩展阅读

- [嵌入式系统内存管理](../../docs/02-memory-management.md)
- [实时系统设计](../../docs/04-realtime-systems.md)
- [性能优化技术](../../../16-performance-optimization/README.md)

## 许可证

本项目采用 MIT 或 Apache-2.0 双重许可证。
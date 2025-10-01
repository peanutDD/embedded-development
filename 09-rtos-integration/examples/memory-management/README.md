# 内存管理示例 (Memory Management Example)

这个示例演示了嵌入式系统中的各种内存管理技术，包括堆分配、内存池、栈监控、内存泄漏检测和碎片化管理。

## 功能特性

### 🔧 核心功能
- **堆内存管理**: 使用链表分配器进行动态内存分配
- **内存池管理**: 固定大小块的高效内存分配
- **栈使用监控**: 实时监控栈使用情况和溢出检测
- **内存泄漏检测**: 跟踪和报告内存泄漏
- **碎片化分析**: 检测和处理内存碎片化问题
- **分配统计**: 详细的内存分配和使用统计

### 📊 监控功能
- **实时统计**: 分配次数、字节数、峰值使用量
- **性能分析**: 分配失败率、碎片化程度
- **安全检查**: 栈溢出检测、双重释放检测
- **资源跟踪**: 活跃分配、泄漏检测

### 🛡️ 安全特性
- **边界检查**: 防止缓冲区溢出
- **栈保护**: 栈溢出检测和报告
- **内存保护**: 检测非法内存访问
- **错误处理**: 优雅的内存分配失败处理

## 硬件要求

### 开发板
- **MCU**: STM32F407VG (Cortex-M4F)
- **Flash**: 1MB
- **RAM**: 128KB SRAM + 64KB CCM RAM
- **调试接口**: SWD

### 外设连接
- **LED**: PD12 (板载LED)
- **调试**: RTT (Real-Time Transfer)

### 内存布局
```
 Flash (1MB):  0x08000000 - 0x080FFFFF
 SRAM (128KB): 0x20000000 - 0x2001FFFF  
 CCM (64KB):   0x10000000 - 0x1000FFFF  (用于堆)
 Stack:        8KB (可配置)
 Heap:         32KB (CCM RAM中)
```

## 项目结构

```
memory_management/
├── Cargo.toml              # 项目配置和依赖
├── README.md               # 项目文档
├── src/
│   └── main.rs            # 主程序
├── examples/              # 示例程序
│   ├── heap_allocator.rs  # 堆分配器示例
│   ├── memory_pools.rs    # 内存池示例
│   ├── stack_monitor.rs   # 栈监控示例
│   ├── memory_protection.rs # 内存保护示例
│   └── leak_detection.rs  # 泄漏检测示例
└── memory.x               # 链接脚本
```

## 代码说明

### 内存分配器

```rust
// 全局堆分配器
#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// 堆内存区域 (使用CCM RAM)
static mut HEAP: [u8; 32 * 1024] = [0; 32 * 1024];

// 初始化堆
unsafe {
    ALLOCATOR.lock().init(HEAP.as_mut_ptr(), HEAP.len());
}
```

### 内存池管理

```rust
// 内存池配置
type PoolNode = Node<[u8; 64]>;
static mut POOL_MEMORY: [PoolNode; 16] = [Node::new(); 16];
static POOL: Pool<PoolNode> = Pool::new();

// 使用内存池
if let Some(mut block) = POOL.alloc() {
    // 使用内存块
    block.fill(0xAA);
    // 释放回池
    POOL.free(block);
}
```

### 栈监控

```rust
struct StackMonitor {
    stack_start: usize,
    stack_size: usize,
    max_usage: usize,
    overflow_count: u32,
}

impl StackMonitor {
    fn update(&mut self) {
        let current_sp: usize;
        unsafe {
            asm!("mov {}, sp", out(reg) current_sp);
        }
        // 计算栈使用量和检测溢出
    }
}
```

### 泄漏检测

```rust
struct LeakDetector {
    allocations: BTreeMap<usize, (usize, u32)>,
    next_alloc_id: u32,
}

impl LeakDetector {
    fn record_allocation(&mut self, ptr: usize, size: usize) {
        self.allocations.insert(ptr, (size, self.next_alloc_id));
        self.next_alloc_id += 1;
    }
    
    fn check_leaks(&self) -> Vec<(usize, usize, u32), 32> {
        // 返回所有未释放的分配
    }
}
```

## 测试场景

### 1. 堆分配测试
- 动态Vector分配
- 字符串分配
- 大块内存分配
- 自动内存释放

### 2. 内存池测试
- 固定大小块分配
- 池耗尽处理
- 批量分配和释放

### 3. 栈使用测试
- 递归函数调用
- 大型局部变量
- 栈使用量监控

### 4. 泄漏检测测试
- 故意内存泄漏
- 泄漏统计和报告
- 泄漏位置跟踪

### 5. 碎片化测试
- 不同大小内存分配
- 部分内存释放
- 碎片化影响分析

## 编译和烧录

### 环境准备
```bash
# 安装Rust嵌入式工具链
rustup target add thumbv7em-none-eabihf
cargo install cargo-embed probe-run

# 安装调试工具
cargo install cargo-binutils
rustup component add llvm-tools-preview
```

### 编译项目
```bash
# 进入项目目录
cd examples/memory_management

# 检查代码
cargo check

# 编译发布版本
cargo build --release

# 查看二进制大小
cargo size --release
```

### 烧录和调试
```bash
# 使用probe-run烧录和运行
cargo run --release

# 使用cargo-embed烧录
cargo embed --release

# 生成反汇编
cargo objdump --release -- -d > disassembly.txt
```

## 预期行为

### 启动序列
1. **初始化阶段**:
   ```
   Memory Management Example Starting...
   Heap initialized: 32768 bytes
   Memory pool initialized: 16 blocks
   Leak detector initialized
   ```

2. **运行阶段**:
   - LED以1Hz频率闪烁
   - 每秒执行不同的内存测试
   - RTT输出详细的测试结果

### 测试输出示例
```
Testing heap allocation...
Vec: [0, 1, 4, 9, 16, 25, 36, 49, 64, 81]
String: Hello, heap!
Boxed value: 42
Large buffer capacity: 1024

Testing memory pools...
Pool block allocated and filled
Pool block freed
Allocated pool block 0
All pool blocks freed

=== Memory Statistics ===
Heap allocations: 15 total, 2 current
Heap bytes: 2048 total, 256 current, 1024 peak
Stack usage: 25% (2048/8192 bytes)
=========================
```

## 学习要点

### 内存管理概念
1. **堆 vs 栈**: 理解不同内存区域的特点
2. **动态分配**: 运行时内存分配的优缺点
3. **内存池**: 固定大小分配的性能优势
4. **碎片化**: 内存碎片的产生和影响
5. **泄漏检测**: 内存泄漏的识别和预防

### 嵌入式最佳实践
1. **预分配策略**: 启动时分配所需内存
2. **池化管理**: 使用内存池减少碎片化
3. **栈监控**: 防止栈溢出导致系统崩溃
4. **资源跟踪**: 监控内存使用情况
5. **错误处理**: 优雅处理内存不足情况

### 性能优化
1. **分配器选择**: 根据使用模式选择合适的分配器
2. **内存对齐**: 利用硬件对齐提高访问效率
3. **缓存友好**: 考虑缓存行大小和访问模式
4. **零拷贝**: 减少不必要的内存复制
5. **批量操作**: 批量分配和释放提高效率

## 性能分析

### 内存使用
- **代码段**: ~15KB (包含分配器和监控代码)
- **静态数据**: ~35KB (堆 + 内存池)
- **栈使用**: 动态，最大8KB
- **运行时开销**: <5% CPU时间

### 分配性能
- **堆分配**: ~100-500 CPU周期
- **池分配**: ~10-50 CPU周期
- **栈分配**: ~1-5 CPU周期
- **监控开销**: ~10-20 CPU周期

## 扩展建议

### 功能扩展
1. **多种分配器**: 实现buddy、slab等分配器
2. **内存压缩**: 实现内存碎片整理
3. **RTOS集成**: 与FreeRTOS/RTIC集成
4. **网络缓冲**: 专用网络数据包池
5. **DMA缓冲**: DMA安全的内存区域

### 性能优化
1. **硬件加速**: 利用MPU进行内存保护
2. **缓存优化**: 考虑L1/L2缓存特性
3. **原子操作**: 无锁数据结构
4. **SIMD优化**: 向量化内存操作
5. **预取优化**: 预取常用内存块

### 安全增强
1. **内存加密**: 敏感数据加密存储
2. **访问控制**: 基于MPU的访问控制
3. **完整性检查**: 内存完整性验证
4. **隔离机制**: 任务间内存隔离
5. **审计日志**: 内存操作审计

## 故障排除

### 常见问题

1. **编译错误**:
   ```bash
   error: linking with `rust-lld` failed
   ```
   **解决**: 检查链接脚本和内存配置

2. **栈溢出**:
   ```
   Stack overflow detected! Usage: 8500 bytes
   ```
   **解决**: 增加栈大小或优化递归深度

3. **堆耗尽**:
   ```
   Memory allocation failed!
   ```
   **解决**: 增加堆大小或优化内存使用

4. **内存泄漏**:
   ```
   Detected 5 memory leaks
   Total leaked bytes: 640
   ```
   **解决**: 检查未配对的分配/释放

### 调试技巧

1. **RTT日志**: 使用RTT输出详细调试信息
2. **内存映射**: 检查链接器生成的内存映射
3. **静态分析**: 使用clippy检查潜在问题
4. **动态分析**: 运行时监控内存使用
5. **硬件调试**: 使用调试器检查内存状态

### 性能调优

1. **分配器调优**: 根据使用模式选择分配器参数
2. **池大小优化**: 根据实际需求调整池大小
3. **监控频率**: 平衡监控精度和性能开销
4. **编译优化**: 使用适当的优化级别
5. **代码剖析**: 识别性能瓶颈

## 相关资源

### 文档
- [Rust Embedded Book](https://docs.rust-embedded.org/book/)
- [STM32F4xx HAL Documentation](https://docs.rs/stm32f4xx-hal/)
- [Heapless Documentation](https://docs.rs/heapless/)
- [Linked List Allocator](https://docs.rs/linked_list_allocator/)

### 工具
- [probe-run](https://github.com/knurling-rs/probe-run)
- [cargo-embed](https://github.com/probe-rs/cargo-embed)
- [RTT Target](https://docs.rs/rtt-target/)
- [Memory Profiler](https://github.com/nokia/memory-profiler)

### 参考实现
- [Embassy](https://embassy.dev/) - 异步嵌入式框架
- [RTIC](https://rtic.rs/) - 实时中断驱动并发
- [Tock OS](https://www.tockos.org/) - 安全嵌入式操作系统
- [FreeRTOS](https://www.freertos.org/) - 实时操作系统

---

**注意**: 这个示例仅用于学习目的。在生产环境中使用时，请根据具体需求进行适当的安全性和可靠性验证。
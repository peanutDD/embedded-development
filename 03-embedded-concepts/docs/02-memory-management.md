# 嵌入式系统内存管理

## 概述

内存管理是嵌入式系统开发中的核心概念。由于嵌入式系统通常具有有限的内存资源，有效的内存管理策略对系统性能和稳定性至关重要。

## 内存类型

### 1. Flash 存储器
Flash 存储器是非易失性存储器，主要用于存储程序代码和常量数据。

#### 特点
- **非易失性**: 断电后数据不丢失
- **只读性**: 正常运行时只能读取，不能写入
- **擦写限制**: 有限的擦写次数（通常 10,000 - 100,000 次）
- **块擦除**: 必须按块（通常 1KB-64KB）擦除

#### 使用场景
```rust
// 在 Flash 中存储常量数据
#[link_section = ".rodata"]
static LOOKUP_TABLE: [u16; 256] = [
    0x0000, 0x0001, 0x0004, 0x0009, // x^2 查找表
    // ... 更多数据
];

// 程序代码自动存储在 Flash 中
fn calculate_checksum(data: &[u8]) -> u16 {
    data.iter().fold(0u16, |acc, &byte| acc.wrapping_add(byte as u16))
}
```

### 2. SRAM (静态随机存取存储器)
SRAM 是易失性存储器，用于存储运行时数据。

#### 特点
- **易失性**: 断电后数据丢失
- **快速访问**: 访问速度快，无等待状态
- **随机访问**: 可以随机读写任意地址
- **有限容量**: 通常几KB到几MB

#### 内存区域划分
```rust
// 栈区域 - 存储局部变量和函数调用信息
fn function_with_local_vars() {
    let local_array: [u32; 100] = [0; 100]; // 在栈上分配
    let local_var = 42u32;
}

// 静态区域 - 存储全局变量和静态变量
static mut GLOBAL_COUNTER: u32 = 0;
static CONSTANT_DATA: u32 = 0x12345678;

// 堆区域 - 动态分配（在 no_std 环境中需要特殊处理）
extern crate alloc;
use alloc::vec::Vec;

fn dynamic_allocation() {
    let mut dynamic_vec = Vec::new();
    dynamic_vec.push(1);
    dynamic_vec.push(2);
}
```

### 3. 特殊内存区域

#### EEPROM/Flash 数据区
```rust
// 使用 EEPROM 存储配置数据
struct Config {
    device_id: u32,
    calibration_data: [f32; 8],
    flags: u16,
}

// 在链接脚本中定义 EEPROM 区域
// .eeprom : { *(.eeprom*) } > EEPROM
#[link_section = ".eeprom"]
static mut DEVICE_CONFIG: Config = Config {
    device_id: 0,
    calibration_data: [0.0; 8],
    flags: 0,
};
```

## 内存布局

### 1. 链接脚本 (Linker Script)
链接脚本定义了程序在内存中的布局。

```ld
/* memory.x - STM32F4xx 内存布局 */
MEMORY
{
  /* Flash 存储器 - 512KB */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  
  /* SRAM - 128KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
  
  /* CCM RAM - 64KB (仅 CPU 可访问) */
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

/* 程序段定义 */
SECTIONS
{
  /* 向量表 */
  .vector_table ORIGIN(FLASH) :
  {
    LONG(ORIGIN(RAM) + LENGTH(RAM)); /* 初始栈指针 */
    KEEP(*(.vector_table.reset_vector));
    KEEP(*(.vector_table.exceptions));
    KEEP(*(.vector_table.interrupts));
  } > FLASH

  /* 程序代码 */
  .text :
  {
    *(.text .text.*);
  } > FLASH

  /* 只读数据 */
  .rodata :
  {
    *(.rodata .rodata.*);
  } > FLASH

  /* 初始化数据 */
  .data : AT(ADDR(.rodata) + SIZEOF(.rodata))
  {
    _sdata = .;
    *(.data .data.*);
    _edata = .;
  } > RAM

  /* 未初始化数据 */
  .bss :
  {
    _sbss = .;
    *(.bss .bss.*);
    _ebss = .;
  } > RAM
}
```

### 2. 内存初始化
```rust
// 启动时的内存初始化代码
#[no_mangle]
pub unsafe extern "C" fn Reset() -> ! {
    // 初始化 RAM 中的 .data 段
    let mut src = &_sidata as *const u32;
    let mut dst = &mut _sdata as *mut u32;
    let end = &_edata as *const u32;
    
    while dst < end {
        dst.write_volatile(src.read_volatile());
        src = src.offset(1);
        dst = dst.offset(1);
    }
    
    // 清零 .bss 段
    let mut dst = &mut _sbss as *mut u32;
    let end = &_ebss as *const u32;
    
    while dst < end {
        dst.write_volatile(0);
        dst = dst.offset(1);
    }
    
    // 跳转到主程序
    main();
}
```

## 栈管理

### 1. 栈的作用
- 存储函数参数和返回地址
- 存储局部变量
- 保存寄存器状态（中断时）

### 2. 栈溢出检测
```rust
// 栈溢出检测
extern "C" {
    static mut _stack_start: u32;
    static mut _stack_end: u32;
}

fn check_stack_usage() -> usize {
    let current_sp: u32;
    unsafe {
        asm!("mov {}, sp", out(reg) current_sp);
        let stack_start = &_stack_start as *const u32 as u32;
        let stack_end = &_stack_end as *const u32 as u32;
        
        if current_sp < stack_end {
            panic!("Stack overflow detected!");
        }
        
        (stack_start - current_sp) as usize
    }
}

// 栈保护
#[no_mangle]
pub extern "C" fn __stack_chk_fail() {
    panic!("Stack smashing detected!");
}
```

### 3. 多任务栈管理
```rust
// RTOS 中的任务栈管理
struct TaskControlBlock {
    stack_pointer: *mut u32,
    stack_base: *mut u32,
    stack_size: usize,
    // ... 其他字段
}

impl TaskControlBlock {
    fn new(stack_size: usize) -> Self {
        let stack = alloc_stack(stack_size);
        Self {
            stack_pointer: unsafe { stack.add(stack_size) },
            stack_base: stack,
            stack_size,
        }
    }
    
    fn check_stack_overflow(&self) -> bool {
        let current_usage = unsafe {
            self.stack_base.offset_from(self.stack_pointer) as usize
        };
        current_usage >= self.stack_size
    }
}
```

## 堆管理

### 1. 嵌入式系统中的堆
在 `no_std` 环境中，默认没有堆分配器。需要手动实现或使用第三方库。

```rust
// 使用 linked_list_allocator
use linked_list_allocator::LockedHeap;

#[global_allocator]
static ALLOCATOR: LockedHeap = LockedHeap::empty();

// 定义堆内存区域
static mut HEAP: [u8; 1024] = [0; 1024];

fn init_heap() {
    unsafe {
        ALLOCATOR.lock().init(HEAP.as_ptr() as usize, HEAP.len());
    }
}
```

### 2. 内存池管理
```rust
// 固定大小内存池
use heapless::pool::{Pool, Node};

// 定义内存池节点
static mut MEMORY: [Node<[u8; 64]>; 16] = [Node::new(); 16];
static POOL: Pool<[u8; 64]> = Pool::new();

fn init_memory_pool() {
    unsafe {
        POOL.grow_exact(&mut MEMORY);
    }
}

fn allocate_buffer() -> Option<heapless::pool::Box<[u8; 64]>> {
    POOL.alloc().ok().map(|node| node.init([0; 64]))
}
```

## 内存保护

### 1. MPU (Memory Protection Unit)
```rust
// 配置 MPU 保护关键内存区域
fn configure_mpu() {
    let mpu = unsafe { &*cortex_m::peripheral::MPU::ptr() };
    
    // 禁用 MPU
    mpu.ctrl.write(0);
    
    // 配置区域 0: Flash (只读)
    mpu.rbar.write(0x08000000 | 0x10); // 区域 0, 有效
    mpu.rasr.write(
        (0b011 << 24) |  // AP: 只读
        (0b10010 << 1) | // 大小: 512KB
        1                // 启用
    );
    
    // 配置区域 1: SRAM (读写)
    mpu.rbar.write(0x20000000 | 0x11); // 区域 1, 有效
    mpu.rasr.write(
        (0b011 << 24) |  // AP: 读写
        (0b10001 << 1) | // 大小: 128KB
        1                // 启用
    );
    
    // 启用 MPU
    mpu.ctrl.write(0x05); // 启用 MPU 和默认内存映射
}
```

### 2. 内存访问检查
```rust
// 安全的内存访问函数
fn safe_memory_read<T>(addr: usize) -> Result<T, MemoryError> 
where
    T: Copy,
{
    // 检查地址对齐
    if addr % core::mem::align_of::<T>() != 0 {
        return Err(MemoryError::Misaligned);
    }
    
    // 检查地址范围
    if !is_valid_address(addr, core::mem::size_of::<T>()) {
        return Err(MemoryError::InvalidAddress);
    }
    
    unsafe {
        Ok(core::ptr::read_volatile(addr as *const T))
    }
}

fn is_valid_address(addr: usize, size: usize) -> bool {
    // 检查是否在有效的内存范围内
    let end_addr = addr + size - 1;
    
    // Flash 区域
    if addr >= 0x08000000 && end_addr <= 0x0807FFFF {
        return true;
    }
    
    // SRAM 区域
    if addr >= 0x20000000 && end_addr <= 0x2001FFFF {
        return true;
    }
    
    // 外设区域
    if addr >= 0x40000000 && end_addr <= 0x5FFFFFFF {
        return true;
    }
    
    false
}

#[derive(Debug)]
enum MemoryError {
    Misaligned,
    InvalidAddress,
    AccessViolation,
}
```

## 内存优化技巧

### 1. 编译器优化
```rust
// 使用 #[repr] 控制结构体布局
#[repr(C)]
struct PackedStruct {
    a: u8,
    b: u32,
    c: u16,
}

#[repr(packed)]
struct TightlyPackedStruct {
    a: u8,
    b: u32,
    c: u16,
}

// 使用 const 泛型减少代码重复
fn process_array<const N: usize>(arr: &[u8; N]) -> u32 {
    arr.iter().map(|&x| x as u32).sum()
}
```

### 2. 零拷贝技术
```rust
// 使用引用避免数据拷贝
fn process_data(data: &[u8]) -> u32 {
    // 直接处理引用，不拷贝数据
    data.iter().fold(0u32, |acc, &byte| acc.wrapping_add(byte as u32))
}

// 使用 MaybeUninit 避免初始化开销
use core::mem::MaybeUninit;

fn create_large_array() -> [u32; 1000] {
    let mut arr: [MaybeUninit<u32>; 1000] = unsafe {
        MaybeUninit::uninit().assume_init()
    };
    
    for (i, elem) in arr.iter_mut().enumerate() {
        elem.write(i as u32);
    }
    
    unsafe { core::mem::transmute(arr) }
}
```

### 3. 内存使用分析
```rust
// 编译时内存使用统计
#[used]
#[link_section = ".memory_stats"]
static MEMORY_STATS: MemoryStats = MemoryStats {
    flash_usage: env!("FLASH_USAGE").parse().unwrap_or(0),
    ram_usage: env!("RAM_USAGE").parse().unwrap_or(0),
};

struct MemoryStats {
    flash_usage: u32,
    ram_usage: u32,
}

// 运行时内存监控
fn print_memory_usage() {
    let heap_used = ALLOCATOR.lock().used();
    let stack_used = check_stack_usage();
    
    println!("Heap used: {} bytes", heap_used);
    println!("Stack used: {} bytes", stack_used);
}
```

## 总结

嵌入式系统的内存管理需要考虑以下关键点：

1. **内存类型**: 理解 Flash、SRAM、EEPROM 的特点和用途
2. **内存布局**: 通过链接脚本合理规划内存使用
3. **栈管理**: 防止栈溢出，合理设置栈大小
4. **堆管理**: 在资源受限环境中谨慎使用动态分配
5. **内存保护**: 使用 MPU 等机制保护关键内存区域
6. **优化技巧**: 通过编译器优化和编程技巧减少内存使用

掌握这些概念和技术，能够帮助开发者在资源受限的嵌入式环境中构建高效、稳定的系统。
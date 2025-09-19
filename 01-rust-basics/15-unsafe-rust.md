# 第15章：Unsafe Rust

## 概述

Unsafe Rust是Rust语言中的一个特殊子集，它允许程序员绕过Rust的安全检查，直接操作内存和执行一些在安全Rust中不被允许的操作。虽然unsafe代码打破了Rust的安全保证，但它是系统编程、性能优化和与C语言互操作的重要工具。

理解unsafe Rust不仅能帮助你编写更高效的代码，还能让你更深入地理解Rust的安全模型。

## 学习目标

通过本章学习，你将掌握：
- unsafe关键字的使用场景和原则
- 原始指针的操作和内存管理
- 与C语言的互操作（FFI）
- 内联汇编的基本使用
- unsafe代码的安全抽象设计
- 内存布局和对齐的控制

## 1. Unsafe基础

### 1.1 什么是Unsafe Rust

```rust
fn main() {
    // 安全的Rust代码
    let mut x = 5;
    let r1 = &x;
    let r2 = &x;
    println!("r1: {}, r2: {}", r1, r2);
    
    // unsafe代码块
    unsafe {
        // 在这里可以执行"不安全"的操作
        let raw_ptr = &x as *const i32;
        println!("Raw pointer value: {}", *raw_ptr);
    }
}
```

### 1.2 Unsafe的五种超能力

```rust
use std::slice;

fn demonstrate_unsafe_superpowers() {
    println!("=== Unsafe Rust的五种超能力 ===");
    
    // 1. 解引用原始指针
    unsafe {
        let x = 42;
        let raw_ptr = &x as *const i32;
        println!("1. 解引用原始指针: {}", *raw_ptr);
    }
    
    // 2. 调用unsafe函数或方法
    unsafe {
        let result = dangerous_function();
        println!("2. 调用unsafe函数: {}", result);
    }
    
    // 3. 访问或修改可变静态变量
    unsafe {
        COUNTER += 1;
        println!("3. 修改静态变量: {}", COUNTER);
    }
    
    // 4. 实现unsafe trait
    // (在后面的示例中展示)
    
    // 5. 访问union的字段
    let u = MyUnion { i: 42 };
    unsafe {
        println!("5. 访问union字段: {}", u.i);
    }
}

unsafe fn dangerous_function() -> i32 {
    println!("执行危险操作...");
    100
}

static mut COUNTER: i32 = 0;

union MyUnion {
    i: i32,
    f: f32,
}

fn main() {
    demonstrate_unsafe_superpowers();
}
```

### 1.3 原始指针基础

```rust
fn raw_pointer_basics() {
    println!("=== 原始指针基础 ===");
    
    let mut num = 5;
    
    // 创建原始指针
    let r1 = &num as *const i32;        // 不可变原始指针
    let r2 = &mut num as *mut i32;      // 可变原始指针
    
    // 从任意内存地址创建原始指针（危险！）
    let address = 0x012345usize;
    let r3 = address as *const i32;
    
    println!("原始指针地址:");
    println!("r1: {:p}", r1);
    println!("r2: {:p}", r2);
    println!("r3: {:p}", r3);
    
    // 解引用原始指针（需要unsafe）
    unsafe {
        println!("r1指向的值: {}", *r1);
        println!("r2指向的值: {}", *r2);
        
        // 通过可变原始指针修改值
        *r2 = 10;
        println!("修改后的值: {}", num);
    }
    
    // 原始指针的算术运算
    unsafe {
        let arr = [1, 2, 3, 4, 5];
        let ptr = arr.as_ptr();
        
        println!("数组元素:");
        for i in 0..arr.len() {
            let element_ptr = ptr.add(i);
            println!("  arr[{}] = {}", i, *element_ptr);
        }
    }
}

fn main() {
    raw_pointer_basics();
}
```

## 2. 内存操作

### 2.1 手动内存管理

```rust
use std::alloc::{alloc, dealloc, Layout};
use std::ptr;

struct ManualVec<T> {
    ptr: *mut T,
    len: usize,
    capacity: usize,
}

impl<T> ManualVec<T> {
    fn new() -> Self {
        Self {
            ptr: ptr::null_mut(),
            len: 0,
            capacity: 0,
        }
    }
    
    fn with_capacity(capacity: usize) -> Self {
        if capacity == 0 {
            return Self::new();
        }
        
        let layout = Layout::array::<T>(capacity).unwrap();
        let ptr = unsafe { alloc(layout) as *mut T };
        
        if ptr.is_null() {
            panic!("内存分配失败");
        }
        
        Self {
            ptr,
            len: 0,
            capacity,
        }
    }
    
    fn push(&mut self, item: T) {
        if self.len == self.capacity {
            self.grow();
        }
        
        unsafe {
            ptr::write(self.ptr.add(self.len), item);
        }
        self.len += 1;
    }
    
    fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            None
        } else {
            self.len -= 1;
            unsafe {
                Some(ptr::read(self.ptr.add(self.len)))
            }
        }
    }
    
    fn get(&self, index: usize) -> Option<&T> {
        if index < self.len {
            unsafe {
                Some(&*self.ptr.add(index))
            }
        } else {
            None
        }
    }
    
    fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        if index < self.len {
            unsafe {
                Some(&mut *self.ptr.add(index))
            }
        } else {
            None
        }
    }
    
    fn grow(&mut self) {
        let new_capacity = if self.capacity == 0 { 1 } else { self.capacity * 2 };
        let new_layout = Layout::array::<T>(new_capacity).unwrap();
        
        let new_ptr = if self.capacity == 0 {
            unsafe { alloc(new_layout) as *mut T }
        } else {
            let old_layout = Layout::array::<T>(self.capacity).unwrap();
            unsafe {
                std::alloc::realloc(
                    self.ptr as *mut u8,
                    old_layout,
                    new_layout.size(),
                ) as *mut T
            }
        };
        
        if new_ptr.is_null() {
            panic!("内存重新分配失败");
        }
        
        self.ptr = new_ptr;
        self.capacity = new_capacity;
    }
    
    fn len(&self) -> usize {
        self.len
    }
    
    fn capacity(&self) -> usize {
        self.capacity
    }
}

impl<T> Drop for ManualVec<T> {
    fn drop(&mut self) {
        // 先drop所有元素
        while let Some(_) = self.pop() {}
        
        // 释放内存
        if self.capacity != 0 {
            let layout = Layout::array::<T>(self.capacity).unwrap();
            unsafe {
                dealloc(self.ptr as *mut u8, layout);
            }
        }
    }
}

fn manual_memory_example() {
    println!("=== 手动内存管理示例 ===");
    
    let mut vec = ManualVec::new();
    
    // 添加元素
    for i in 0..10 {
        vec.push(i);
        println!("添加 {}, 长度: {}, 容量: {}", i, vec.len(), vec.capacity());
    }
    
    // 访问元素
    for i in 0..vec.len() {
        if let Some(value) = vec.get(i) {
            println!("vec[{}] = {}", i, value);
        }
    }
    
    // 修改元素
    if let Some(value) = vec.get_mut(5) {
        *value = 999;
        println!("修改 vec[5] = {}", value);
    }
    
    // 弹出元素
    while let Some(value) = vec.pop() {
        println!("弹出: {}", value);
    }
}

fn main() {
    manual_memory_example();
}
```

### 2.2 内存布局控制

```rust
use std::mem;

// 控制结构体布局
#[repr(C)]
struct CCompatible {
    a: u8,
    b: u32,
    c: u16,
}

#[repr(packed)]
struct Packed {
    a: u8,
    b: u32,
    c: u16,
}

#[repr(align(16))]
struct Aligned {
    a: u8,
    b: u32,
}

fn memory_layout_example() {
    println!("=== 内存布局控制 ===");
    
    // 默认布局
    #[derive(Debug)]
    struct Default {
        a: u8,
        b: u32,
        c: u16,
    }
    
    println!("Default struct:");
    println!("  Size: {}", mem::size_of::<Default>());
    println!("  Align: {}", mem::align_of::<Default>());
    
    println!("C-compatible struct:");
    println!("  Size: {}", mem::size_of::<CCompatible>());
    println!("  Align: {}", mem::align_of::<CCompatible>());
    
    println!("Packed struct:");
    println!("  Size: {}", mem::size_of::<Packed>());
    println!("  Align: {}", mem::align_of::<Packed>());
    
    println!("Aligned struct:");
    println!("  Size: {}", mem::size_of::<Aligned>());
    println!("  Align: {}", mem::align_of::<Aligned>());
    
    // 字段偏移量
    let default_instance = Default { a: 1, b: 2, c: 3 };
    let base_ptr = &default_instance as *const Default as *const u8;
    
    unsafe {
        let a_ptr = &default_instance.a as *const u8;
        let b_ptr = &default_instance.b as *const u32 as *const u8;
        let c_ptr = &default_instance.c as *const u16 as *const u8;
        
        println!("Field offsets:");
        println!("  a: {}", a_ptr.offset_from(base_ptr));
        println!("  b: {}", b_ptr.offset_from(base_ptr));
        println!("  c: {}", c_ptr.offset_from(base_ptr));
    }
}

fn main() {
    memory_layout_example();
}
```

### 2.3 Union类型

```rust
use std::mem;

union FloatOrInt {
    f: f32,
    i: u32,
}

union LargeUnion {
    small: u8,
    medium: u32,
    large: [u8; 16],
}

fn union_example() {
    println!("=== Union类型示例 ===");
    
    // 基本union使用
    let mut float_or_int = FloatOrInt { f: 3.14 };
    
    unsafe {
        println!("作为float: {}", float_or_int.f);
        println!("作为int: {}", float_or_int.i);
        println!("作为int (hex): 0x{:08x}", float_or_int.i);
    }
    
    // 修改union
    unsafe {
        float_or_int.i = 0x42000000;
        println!("修改后作为float: {}", float_or_int.f);
    }
    
    // 大小和对齐
    println!("FloatOrInt size: {}", mem::size_of::<FloatOrInt>());
    println!("FloatOrInt align: {}", mem::align_of::<FloatOrInt>());
    
    let large = LargeUnion { large: [0; 16] };
    println!("LargeUnion size: {}", mem::size_of::<LargeUnion>());
    
    unsafe {
        println!("large.small: {}", large.small);
    }
}

// 实现Copy trait的union
#[derive(Copy, Clone)]
union CopyableUnion {
    x: i32,
    y: f32,
}

fn main() {
    union_example();
}
```

## 3. 与C语言互操作（FFI）

### 3.1 调用C函数

```rust
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int};

// 声明外部C函数
extern "C" {
    fn strlen(s: *const c_char) -> usize;
    fn strcmp(s1: *const c_char, s2: *const c_char) -> c_int;
    fn malloc(size: usize) -> *mut std::ffi::c_void;
    fn free(ptr: *mut std::ffi::c_void);
}

fn c_interop_example() {
    println!("=== C语言互操作示例 ===");
    
    // 使用C字符串
    let rust_string = "Hello, C world!";
    let c_string = CString::new(rust_string).expect("CString::new failed");
    
    unsafe {
        let len = strlen(c_string.as_ptr());
        println!("C strlen result: {}", len);
    }
    
    // 比较C字符串
    let str1 = CString::new("hello").unwrap();
    let str2 = CString::new("world").unwrap();
    let str3 = CString::new("hello").unwrap();
    
    unsafe {
        let cmp1 = strcmp(str1.as_ptr(), str2.as_ptr());
        let cmp2 = strcmp(str1.as_ptr(), str3.as_ptr());
        
        println!("strcmp('hello', 'world'): {}", cmp1);
        println!("strcmp('hello', 'hello'): {}", cmp2);
    }
    
    // 使用C的malloc/free
    unsafe {
        let size = 1024;
        let ptr = malloc(size);
        
        if !ptr.is_null() {
            println!("成功分配 {} 字节内存", size);
            
            // 使用内存...
            let byte_ptr = ptr as *mut u8;
            *byte_ptr = 42;
            println!("写入值: {}", *byte_ptr);
            
            free(ptr);
            println!("内存已释放");
        } else {
            println!("内存分配失败");
        }
    }
}

// 为C代码提供Rust函数
#[no_mangle]
pub extern "C" fn rust_function(x: c_int, y: c_int) -> c_int {
    x + y
}

#[no_mangle]
pub extern "C" fn rust_string_length(s: *const c_char) -> usize {
    if s.is_null() {
        return 0;
    }
    
    unsafe {
        CStr::from_ptr(s).to_bytes().len()
    }
}

fn main() {
    c_interop_example();
}
```

### 3.2 创建C兼容的API

```rust
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_void};
use std::ptr;
use std::slice;

// C兼容的结构体
#[repr(C)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}

#[repr(C)]
pub struct Array {
    data: *mut c_int,
    len: usize,
    capacity: usize,
}

// C兼容的函数
#[no_mangle]
pub extern "C" fn point_new(x: f64, y: f64) -> *mut Point {
    let point = Box::new(Point { x, y });
    Box::into_raw(point)
}

#[no_mangle]
pub extern "C" fn point_distance(p1: *const Point, p2: *const Point) -> f64 {
    if p1.is_null() || p2.is_null() {
        return -1.0;
    }
    
    unsafe {
        let p1 = &*p1;
        let p2 = &*p2;
        
        let dx = p1.x - p2.x;
        let dy = p1.y - p2.y;
        
        (dx * dx + dy * dy).sqrt()
    }
}

#[no_mangle]
pub extern "C" fn point_free(point: *mut Point) {
    if !point.is_null() {
        unsafe {
            let _ = Box::from_raw(point);
        }
    }
}

#[no_mangle]
pub extern "C" fn array_new(capacity: usize) -> *mut Array {
    let layout = std::alloc::Layout::array::<c_int>(capacity).unwrap();
    let data = unsafe { std::alloc::alloc(layout) as *mut c_int };
    
    if data.is_null() {
        return ptr::null_mut();
    }
    
    let array = Box::new(Array {
        data,
        len: 0,
        capacity,
    });
    
    Box::into_raw(array)
}

#[no_mangle]
pub extern "C" fn array_push(array: *mut Array, value: c_int) -> c_int {
    if array.is_null() {
        return -1;
    }
    
    unsafe {
        let array = &mut *array;
        
        if array.len >= array.capacity {
            return -1; // 数组已满
        }
        
        *array.data.add(array.len) = value;
        array.len += 1;
        
        0 // 成功
    }
}

#[no_mangle]
pub extern "C" fn array_get(array: *const Array, index: usize) -> c_int {
    if array.is_null() {
        return 0;
    }
    
    unsafe {
        let array = &*array;
        
        if index >= array.len {
            return 0;
        }
        
        *array.data.add(index)
    }
}

#[no_mangle]
pub extern "C" fn array_free(array: *mut Array) {
    if array.is_null() {
        return;
    }
    
    unsafe {
        let array = Box::from_raw(array);
        
        if !array.data.is_null() {
            let layout = std::alloc::Layout::array::<c_int>(array.capacity).unwrap();
            std::alloc::dealloc(array.data as *mut u8, layout);
        }
    }
}

fn test_c_api() {
    println!("=== 测试C兼容API ===");
    
    unsafe {
        // 测试Point API
        let p1 = point_new(0.0, 0.0);
        let p2 = point_new(3.0, 4.0);
        
        let distance = point_distance(p1, p2);
        println!("点之间的距离: {}", distance);
        
        point_free(p1);
        point_free(p2);
        
        // 测试Array API
        let array = array_new(5);
        
        for i in 0..5 {
            array_push(array, i * 10);
        }
        
        for i in 0..5 {
            let value = array_get(array, i);
            println!("array[{}] = {}", i, value);
        }
        
        array_free(array);
    }
}

fn main() {
    test_c_api();
}
```

## 4. 内联汇编

### 4.1 基本内联汇编

```rust
use std::arch::asm;

fn inline_assembly_examples() {
    println!("=== 内联汇编示例 ===");
    
    // 基本内联汇编
    let x: u64;
    unsafe {
        asm!("mov {}, 42", out(reg) x);
    }
    println!("汇编设置的值: {}", x);
    
    // 使用输入和输出
    let a = 10u64;
    let b = 20u64;
    let result: u64;
    
    unsafe {
        asm!(
            "add {result}, {a}, {b}",
            a = in(reg) a,
            b = in(reg) b,
            result = out(reg) result,
        );
    }
    println!("{} + {} = {}", a, b, result);
    
    // 修改寄存器
    let mut x = 5u64;
    unsafe {
        asm!(
            "add {x}, {x}, {x}",
            x = inout(reg) x,
        );
    }
    println!("x * 2 = {}", x);
}

// CPU特性检测
fn cpu_features() {
    println!("=== CPU特性检测 ===");
    
    #[cfg(target_arch = "x86_64")]
    unsafe {
        let mut eax: u32;
        let mut ebx: u32;
        let mut ecx: u32;
        let mut edx: u32;
        
        // CPUID指令
        asm!(
            "cpuid",
            inout("eax") 1u32 => eax,
            out("ebx") ebx,
            out("ecx") ecx,
            out("edx") edx,
        );
        
        println!("CPUID结果:");
        println!("  EAX: 0x{:08x}", eax);
        println!("  EBX: 0x{:08x}", ebx);
        println!("  ECX: 0x{:08x}", ecx);
        println!("  EDX: 0x{:08x}", edx);
        
        // 检查SSE支持
        let sse_supported = (edx & (1 << 25)) != 0;
        println!("  SSE支持: {}", sse_supported);
    }
    
    #[cfg(not(target_arch = "x86_64"))]
    {
        println!("CPU特性检测仅在x86_64架构上可用");
    }
}

// 高性能计数器
fn performance_counter() -> u64 {
    let counter: u64;
    
    #[cfg(target_arch = "x86_64")]
    unsafe {
        asm!("rdtsc", out("rax") counter, out("rdx") _);
    }
    
    #[cfg(not(target_arch = "x86_64"))]
    {
        counter = 0;
    }
    
    counter
}

fn benchmark_with_rdtsc() {
    println!("=== 使用RDTSC进行性能测试 ===");
    
    let start = performance_counter();
    
    // 执行一些计算
    let mut sum = 0u64;
    for i in 0..1000000 {
        sum += i;
    }
    
    let end = performance_counter();
    
    println!("计算结果: {}", sum);
    println!("CPU周期数: {}", end.wrapping_sub(start));
}

fn main() {
    inline_assembly_examples();
    println!();
    
    cpu_features();
    println!();
    
    benchmark_with_rdtsc();
}
```

## 5. Unsafe Trait和高级特性

### 5.1 实现Unsafe Trait

```rust
use std::marker::PhantomData;
use std::ptr::NonNull;

// 自定义unsafe trait
unsafe trait UnsafeTrait {
    fn dangerous_method(&self);
}

struct SafeStruct {
    value: i32,
}

// 实现unsafe trait需要unsafe关键字
unsafe impl UnsafeTrait for SafeStruct {
    fn dangerous_method(&self) {
        println!("执行危险操作，值: {}", self.value);
    }
}

// 实现Send和Sync
struct MyBox<T> {
    ptr: NonNull<T>,
    _marker: PhantomData<T>,
}

impl<T> MyBox<T> {
    fn new(value: T) -> Self {
        let boxed = Box::new(value);
        let ptr = NonNull::new(Box::into_raw(boxed)).unwrap();
        
        Self {
            ptr,
            _marker: PhantomData,
        }
    }
    
    fn get(&self) -> &T {
        unsafe { self.ptr.as_ref() }
    }
    
    fn get_mut(&mut self) -> &mut T {
        unsafe { self.ptr.as_mut() }
    }
}

impl<T> Drop for MyBox<T> {
    fn drop(&mut self) {
        unsafe {
            let _ = Box::from_raw(self.ptr.as_ptr());
        }
    }
}

// 手动实现Send和Sync（需要确保安全性）
unsafe impl<T: Send> Send for MyBox<T> {}
unsafe impl<T: Sync> Sync for MyBox<T> {}

fn unsafe_trait_example() {
    println!("=== Unsafe Trait示例 ===");
    
    let safe_struct = SafeStruct { value: 42 };
    safe_struct.dangerous_method();
    
    let my_box = MyBox::new(100);
    println!("MyBox值: {}", my_box.get());
}

fn main() {
    unsafe_trait_example();
}
```

### 5.2 零成本抽象

```rust
use std::marker::PhantomData;
use std::ops::{Deref, DerefMut};

// 类型状态模式
struct Locked;
struct Unlocked;

struct StateMachine<State> {
    data: i32,
    _state: PhantomData<State>,
}

impl StateMachine<Locked> {
    fn new(data: i32) -> Self {
        Self {
            data,
            _state: PhantomData,
        }
    }
    
    fn unlock(self) -> StateMachine<Unlocked> {
        println!("解锁状态机");
        StateMachine {
            data: self.data,
            _state: PhantomData,
        }
    }
}

impl StateMachine<Unlocked> {
    fn get_data(&self) -> i32 {
        self.data
    }
    
    fn set_data(&mut self, data: i32) {
        self.data = data;
    }
    
    fn lock(self) -> StateMachine<Locked> {
        println!("锁定状态机");
        StateMachine {
            data: self.data,
            _state: PhantomData,
        }
    }
}

// 零成本包装器
#[repr(transparent)]
struct Wrapper<T>(T);

impl<T> Wrapper<T> {
    fn new(value: T) -> Self {
        Self(value)
    }
    
    fn into_inner(self) -> T {
        self.0
    }
}

impl<T> Deref for Wrapper<T> {
    type Target = T;
    
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T> DerefMut for Wrapper<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

fn zero_cost_abstractions() {
    println!("=== 零成本抽象示例 ===");
    
    // 类型状态模式
    let machine = StateMachine::<Locked>::new(42);
    let mut unlocked = machine.unlock();
    
    println!("数据: {}", unlocked.get_data());
    unlocked.set_data(100);
    println!("修改后数据: {}", unlocked.get_data());
    
    let _locked = unlocked.lock();
    
    // 零成本包装器
    let wrapped = Wrapper::new(vec![1, 2, 3, 4, 5]);
    println!("包装器长度: {}", wrapped.len());
    
    let inner = wrapped.into_inner();
    println!("内部向量: {:?}", inner);
}

fn main() {
    zero_cost_abstractions();
}
```

## 6. 安全抽象设计

### 6.1 安全的Unsafe封装

```rust
use std::ptr;
use std::alloc::{alloc, dealloc, Layout};

pub struct SafeVec<T> {
    ptr: *mut T,
    len: usize,
    capacity: usize,
}

impl<T> SafeVec<T> {
    pub fn new() -> Self {
        Self {
            ptr: ptr::null_mut(),
            len: 0,
            capacity: 0,
        }
    }
    
    pub fn with_capacity(capacity: usize) -> Self {
        if capacity == 0 {
            return Self::new();
        }
        
        let layout = Layout::array::<T>(capacity).unwrap();
        let ptr = unsafe { alloc(layout) as *mut T };
        
        if ptr.is_null() {
            panic!("内存分配失败");
        }
        
        Self {
            ptr,
            len: 0,
            capacity,
        }
    }
    
    pub fn push(&mut self, item: T) {
        if self.len == self.capacity {
            self.grow();
        }
        
        unsafe {
            ptr::write(self.ptr.add(self.len), item);
        }
        self.len += 1;
    }
    
    pub fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            None
        } else {
            self.len -= 1;
            unsafe {
                Some(ptr::read(self.ptr.add(self.len)))
            }
        }
    }
    
    pub fn get(&self, index: usize) -> Option<&T> {
        if index < self.len {
            unsafe {
                Some(&*self.ptr.add(index))
            }
        } else {
            None
        }
    }
    
    pub fn get_mut(&mut self, index: usize) -> Option<&mut T> {
        if index < self.len {
            unsafe {
                Some(&mut *self.ptr.add(index))
            }
        } else {
            None
        }
    }
    
    pub fn len(&self) -> usize {
        self.len
    }
    
    pub fn capacity(&self) -> usize {
        self.capacity
    }
    
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    
    // 私有方法，包含unsafe代码
    fn grow(&mut self) {
        let new_capacity = if self.capacity == 0 { 1 } else { self.capacity * 2 };
        let new_layout = Layout::array::<T>(new_capacity).unwrap();
        
        let new_ptr = if self.capacity == 0 {
            unsafe { alloc(new_layout) as *mut T }
        } else {
            let old_layout = Layout::array::<T>(self.capacity).unwrap();
            unsafe {
                std::alloc::realloc(
                    self.ptr as *mut u8,
                    old_layout,
                    new_layout.size(),
                ) as *mut T
            }
        };
        
        if new_ptr.is_null() {
            panic!("内存重新分配失败");
        }
        
        self.ptr = new_ptr;
        self.capacity = new_capacity;
    }
}

impl<T> Drop for SafeVec<T> {
    fn drop(&mut self) {
        // 先drop所有元素
        while let Some(_) = self.pop() {}
        
        // 释放内存
        if self.capacity != 0 {
            let layout = Layout::array::<T>(self.capacity).unwrap();
            unsafe {
                dealloc(self.ptr as *mut u8, layout);
            }
        }
    }
}

// 实现迭代器
impl<T> IntoIterator for SafeVec<T> {
    type Item = T;
    type IntoIter = SafeVecIntoIter<T>;
    
    fn into_iter(self) -> Self::IntoIter {
        SafeVecIntoIter {
            vec: self,
            index: 0,
        }
    }
}

pub struct SafeVecIntoIter<T> {
    vec: SafeVec<T>,
    index: usize,
}

impl<T> Iterator for SafeVecIntoIter<T> {
    type Item = T;
    
    fn next(&mut self) -> Option<Self::Item> {
        if self.index < self.vec.len {
            let item = unsafe {
                ptr::read(self.vec.ptr.add(self.index))
            };
            self.index += 1;
            Some(item)
        } else {
            None
        }
    }
}

impl<T> Drop for SafeVecIntoIter<T> {
    fn drop(&mut self) {
        // 清理剩余元素
        while self.next().is_some() {}
    }
}

fn safe_abstraction_example() {
    println!("=== 安全抽象示例 ===");
    
    let mut vec = SafeVec::new();
    
    // 添加元素
    for i in 0..10 {
        vec.push(i);
    }
    
    println!("向量长度: {}", vec.len());
    println!("向量容量: {}", vec.capacity());
    
    // 访问元素
    for i in 0..vec.len() {
        if let Some(value) = vec.get(i) {
            println!("vec[{}] = {}", i, value);
        }
    }
    
    // 使用迭代器
    println!("使用迭代器:");
    for (i, value) in vec.into_iter().enumerate() {
        println!("  item {}: {}", i, value);
    }
}

fn main() {
    safe_abstraction_example();
}
```

### 6.2 不变量和契约

```rust
use std::ptr::NonNull;
use std::marker::PhantomData;

/// 一个保证非空的指针包装器
pub struct NonNullPtr<T> {
    ptr: NonNull<T>,
    _marker: PhantomData<T>,
}

impl<T> NonNullPtr<T> {
    /// 创建新的非空指针
    /// 
    /// # Safety
    /// 调用者必须确保指针有效且非空
    pub unsafe fn new_unchecked(ptr: *mut T) -> Self {
        Self {
            ptr: NonNull::new_unchecked(ptr),
            _marker: PhantomData,
        }
    }
    
    /// 安全地创建非空指针
    pub fn new(ptr: *mut T) -> Option<Self> {
        NonNull::new(ptr).map(|ptr| Self {
            ptr,
            _marker: PhantomData,
        })
    }
    
    /// 获取原始指针
    pub fn as_ptr(&self) -> *mut T {
        self.ptr.as_ptr()
    }
    
    /// 安全地解引用
    pub unsafe fn as_ref(&self) -> &T {
        self.ptr.as_ref()
    }
    
    /// 安全地获取可变引用
    pub unsafe fn as_mut(&mut self) -> &mut T {
        self.ptr.as_mut()
    }
}

/// 一个保证边界检查的数组访问器
pub struct BoundsCheckedArray<T> {
    data: Vec<T>,
}

impl<T> BoundsCheckedArray<T> {
    pub fn new(data: Vec<T>) -> Self {
        Self { data }
    }
    
    /// 安全的索引访问
    pub fn get(&self, index: usize) -> Option<&T> {
        self.data.get(index)
    }
    
    /// 不安全但快速的索引访问
    /// 
    /// # Safety
    /// 调用者必须确保index < self.len()
    pub unsafe fn get_unchecked(&self, index: usize) -> &T {
        debug_assert!(index < self.data.len(), "索引越界");
        self.data.get_unchecked(index)
    }
    
    pub fn len(&self) -> usize {
        self.data.len()
    }
}

/// 演示不变量保持
struct CircularBuffer<T> {
    buffer: Vec<Option<T>>,
    head: usize,
    tail: usize,
    size: usize,
    capacity: usize,
}

impl<T> CircularBuffer<T> {
    pub fn new(capacity: usize) -> Self {
        assert!(capacity > 0, "容量必须大于0");
        
        let mut buffer = Vec::with_capacity(capacity);
        buffer.resize_with(capacity, || None);
        
        Self {
            buffer,
            head: 0,
            tail: 0,
            size: 0,
            capacity,
        }
    }
    
    pub fn push(&mut self, item: T) -> Result<(), T> {
        if self.size == self.capacity {
            return Err(item);
        }
        
        self.buffer[self.tail] = Some(item);
        self.tail = (self.tail + 1) % self.capacity;
        self.size += 1;
        
        // 不变量检查
        debug_assert!(self.size <= self.capacity);
        debug_assert!(self.head < self.capacity);
        debug_assert!(self.tail < self.capacity);
        
        Ok(())
    }
    
    pub fn pop(&mut self) -> Option<T> {
        if self.size == 0 {
            return None;
        }
        
        let item = self.buffer[self.head].take();
        self.head = (self.head + 1) % self.capacity;
        self.size -= 1;
        
        // 不变量检查
        debug_assert!(self.size < self.capacity);
        debug_assert!(self.head < self.capacity);
        debug_assert!(self.tail < self.capacity);
        
        item
    }
    
    pub fn len(&self) -> usize {
        self.size
    }
    
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }
    
    pub fn is_full(&self) -> bool {
        self.size == self.capacity
    }
}

fn invariants_example() {
    println!("=== 不变量和契约示例 ===");
    
    // 非空指针示例
    let value = 42;
    let ptr = &value as *const i32 as *mut i32;
    
    if let Some(non_null) = NonNullPtr::new(ptr) {
        unsafe {
            println!("非空指针值: {}", *non_null.as_ref());
        }
    }
    
    // 边界检查数组
    let array = BoundsCheckedArray::new(vec![1, 2, 3, 4, 5]);
    
    // 安全访问
    if let Some(value) = array.get(2) {
        println!("安全访问 array[2]: {}", value);
    }
    
    // 不安全但快速的访问
    unsafe {
        let value = array.get_unchecked(2);
        println!("快速访问 array[2]: {}", value);
    }
    
    // 循环缓冲区
    let mut buffer = CircularBuffer::new(3);
    
    // 填充缓冲区
    for i in 1..=3 {
        buffer.push(i).unwrap();
        println!("推入 {}, 大小: {}", i, buffer.len());
    }
    
    // 尝试推入更多元素
    if let Err(item) = buffer.push(4) {
        println!("缓冲区已满，无法推入 {}", item);
    }
    
    // 弹出元素
    while let Some(item) = buffer.pop() {
        println!("弹出 {}, 剩余大小: {}", item, buffer.len());
    }
}

fn main() {
    invariants_example();
}
```

## 7. 练习题

### 练习1：实现一个unsafe的链表
```rust
use std::ptr;

struct Node<T> {
    data: T,
    next: *mut Node<T>,
}

pub struct LinkedList<T> {
    head: *mut Node<T>,
    len: usize,
}

impl<T> LinkedList<T> {
    pub fn new() -> Self {
        Self {
            head: ptr::null_mut(),
            len: 0,
        }
    }
    
    pub fn push_front(&mut self, data: T) {
        let new_node = Box::into_raw(Box::new(Node {
            data,
            next: self.head,
        }));
        
        self.head = new_node;
        self.len += 1;
    }
    
    pub fn pop_front(&mut self) -> Option<T> {
        if self.head.is_null() {
            None
        } else {
            unsafe {
                let old_head = Box::from_raw(self.head);
                self.head = old_head.next;
                self.len -= 1;
                Some(old_head.data)
            }
        }
    }
    
    pub fn len(&self) -> usize {
        self.len
    }
    
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

impl<T> Drop for LinkedList<T> {
    fn drop(&mut self) {
        while let Some(_) = self.pop_front() {}
    }
}
```

### 练习2：实现一个内存池分配器
```rust
use std::alloc::{alloc, dealloc, Layout};
use std::ptr;

pub struct MemoryPool {
    memory: *mut u8,
    block_size: usize,
    block_count: usize,
    free_list: *mut *mut u8,
}

impl MemoryPool {
    pub fn new(block_size: usize, block_count: usize) -> Self {
        let total_size = block_size * block_count;
        let layout = Layout::from_size_align(total_size, 8).unwrap();
        
        let memory = unsafe { alloc(layout) };
        if memory.is_null() {
            panic!("内存分配失败");
        }
        
        // 初始化空闲列表
        let mut pool = Self {
            memory,
            block_size,
            block_count,
            free_list: ptr::null_mut(),
        };
        
        pool.initialize_free_list();
        pool
    }
    
    fn initialize_free_list(&mut self) {
        unsafe {
            for i in 0..self.block_count {
                let block = self.memory.add(i * self.block_size) as *mut *mut u8;
                
                if i == self.block_count - 1 {
                    *block = ptr::null_mut();
                } else {
                    *block = self.memory.add((i + 1) * self.block_size) as *mut u8;
                }
            }
            
            self.free_list = self.memory as *mut *mut u8;
        }
    }
    
    pub fn allocate(&mut self) -> Option<*mut u8> {
        if self.free_list.is_null() {
            None
        } else {
            unsafe {
                let block = self.free_list as *mut u8;
                self.free_list = *(self.free_list);
                Some(block)
            }
        }
    }
    
    pub fn deallocate(&mut self, ptr: *mut u8) {
        unsafe {
            let block = ptr as *mut *mut u8;
            *block = self.free_list as *mut u8;
            self.free_list = block;
        }
    }
}

impl Drop for MemoryPool {
    fn drop(&mut self) {
        if !self.memory.is_null() {
            let total_size = self.block_size * self.block_count;
            let layout = Layout::from_size_align(total_size, 8).unwrap();
            unsafe {
                dealloc(self.memory, layout);
            }
        }
    }
}
```

### 练习3：实现一个原子引用计数
```rust
use std::sync::atomic::{AtomicUsize, Ordering};
use std::ptr::NonNull;

pub struct AtomicRc<T> {
    ptr: NonNull<RcBox<T>>,
}

struct RcBox<T> {
    ref_count: AtomicUsize,
    data: T,
}

impl<T> AtomicRc<T> {
    pub fn new(data: T) -> Self {
        let boxed = Box::new(RcBox {
            ref_count: AtomicUsize::new(1),
            data,
        });
        
        Self {
            ptr: NonNull::new(Box::into_raw(boxed)).unwrap(),
        }
    }
    
    pub fn clone(&self) -> Self {
        unsafe {
            let rc_box = self.ptr.as_ref();
            rc_box.ref_count.fetch_add(1, Ordering::Relaxed);
        }
        
        Self { ptr: self.ptr }
    }
    
    pub fn strong_count(&self) -> usize {
        unsafe {
            self.ptr.as_ref().ref_count.load(Ordering::Relaxed)
        }
    }
}

impl<T> std::ops::Deref for AtomicRc<T> {
    type Target = T;
    
    fn deref(&self) -> &Self::Target {
        unsafe {
            &self.ptr.as_ref().data
        }
    }
}

impl<T> Drop for AtomicRc<T> {
    fn drop(&mut self) {
        unsafe {
            let rc_box = self.ptr.as_ref();
            let old_count = rc_box.ref_count.fetch_sub(1, Ordering::Release);
            
            if old_count == 1 {
                std::sync::atomic::fence(Ordering::Acquire);
                let _ = Box::from_raw(self.ptr.as_ptr());
            }
        }
    }
}

unsafe impl<T: Send + Sync> Send for AtomicRc<T> {}
unsafe impl<T: Send + Sync> Sync for AtomicRc<T> {}
```

## 总结

Unsafe Rust是一个强大但危险的工具，它允许我们：

### 核心能力
- 解引用原始指针
- 调用unsafe函数
- 访问可变静态变量
- 实现unsafe trait
- 访问union字段

### 应用场景
- 系统编程和底层优化
- 与C语言的互操作
- 实现安全抽象的内部机制
- 性能关键代码的优化

### 最佳实践
- 最小化unsafe代码的范围
- 提供安全的公共API
- 维护数据结构的不变量
- 充分的文档和测试
- 使用工具检查内存安全

### 安全原则
- 理解Rust的所有权和借用规则
- 确保内存安全和线程安全
- 避免数据竞争和悬垂指针
- 正确处理异常和错误情况

记住，unsafe并不意味着代码是错误的，而是意味着编译器无法验证其安全性。作为程序员，你需要承担确保代码安全的责任。在使用unsafe时，要格外小心，并始终考虑是否有更安全的替代方案。
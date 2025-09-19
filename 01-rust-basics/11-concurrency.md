# 第11章：并发编程

## 概述

Rust的并发编程模型以安全性为核心，通过所有权系统和类型系统在编译时防止数据竞争。Rust提供了多种并发原语，包括线程、消息传递、共享状态等，让你能够编写高性能且安全的并发程序。

## 学习目标

通过本章学习，你将掌握：
- 线程的创建和管理
- 消息传递并发模式
- 共享状态并发和同步原语
- `Send`和`Sync`特征的理解和应用
- 并发编程的最佳实践和性能优化

## 1. 线程基础

### 1.1 创建和管理线程

```rust
use std::thread;
use std::time::Duration;

fn main() {
    // 创建新线程
    let handle = thread::spawn(|| {
        for i in 1..10 {
            println!("hi number {} from the spawned thread!", i);
            thread::sleep(Duration::from_millis(1));
        }
    });

    // 主线程执行
    for i in 1..5 {
        println!("hi number {} from the main thread!", i);
        thread::sleep(Duration::from_millis(1));
    }

    // 等待子线程完成
    handle.join().unwrap();
}
```

### 1.2 线程间数据传递

```rust
use std::thread;

fn main() {
    let v = vec![1, 2, 3];

    // 使用move关键字转移所有权
    let handle = thread::spawn(move || {
        println!("Here's a vector: {:?}", v);
    });

    handle.join().unwrap();
}
```

### 1.3 线程构建器

```rust
use std::thread;

fn main() {
    let builder = thread::Builder::new()
        .name("worker-thread".into())
        .stack_size(32 * 1024); // 32KB stack

    let handle = builder.spawn(|| {
        println!("Hello from named thread: {}", thread::current().name().unwrap());
        
        // 执行一些工作
        let result = (1..1000).sum::<i32>();
        println!("Calculation result: {}", result);
        
        result
    }).unwrap();

    match handle.join() {
        Ok(result) => println!("Thread completed with result: {}", result),
        Err(e) => println!("Thread panicked: {:?}", e),
    }
}
```

### 1.4 线程本地存储

```rust
use std::cell::RefCell;
use std::thread;

thread_local! {
    static COUNTER: RefCell<u32> = RefCell::new(1);
}

fn main() {
    COUNTER.with(|c| {
        *c.borrow_mut() = 2;
    });

    let handle = thread::spawn(|| {
        COUNTER.with(|c| {
            *c.borrow_mut() = 3;
        });

        COUNTER.with(|c| {
            println!("Thread counter: {}", *c.borrow());
        });
    });

    handle.join().unwrap();

    COUNTER.with(|c| {
        println!("Main counter: {}", *c.borrow());
    });
}
```

## 2. 消息传递

### 2.1 基本通道使用

```rust
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

fn main() {
    // 创建通道
    let (tx, rx) = mpsc::channel();

    thread::spawn(move || {
        let vals = vec![
            String::from("hi"),
            String::from("from"),
            String::from("the"),
            String::from("thread"),
        ];

        for val in vals {
            tx.send(val).unwrap();
            thread::sleep(Duration::from_secs(1));
        }
    });

    // 接收消息
    for received in rx {
        println!("Got: {}", received);
    }
}
```

### 2.2 多生产者单消费者

```rust
use std::sync::mpsc;
use std::thread;

fn main() {
    let (tx, rx) = mpsc::channel();
    let tx1 = tx.clone();

    // 第一个生产者
    thread::spawn(move || {
        let vals = vec![
            String::from("hi"),
            String::from("from"),
            String::from("the"),
            String::from("thread"),
        ];

        for val in vals {
            tx1.send(val).unwrap();
        }
    });

    // 第二个生产者
    thread::spawn(move || {
        let vals = vec![
            String::from("more"),
            String::from("messages"),
            String::from("for"),
            String::from("you"),
        ];

        for val in vals {
            tx.send(val).unwrap();
        }
    });

    // 消费者
    for received in rx {
        println!("Got: {}", received);
    }
}
```

### 2.3 同步通道

```rust
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

fn main() {
    // 创建同步通道，缓冲区大小为0（完全同步）
    let (tx, rx) = mpsc::sync_channel(0);

    let handle = thread::spawn(move || {
        println!("Sending...");
        tx.send("Hello").unwrap();
        println!("Sent!");
        
        thread::sleep(Duration::from_secs(2));
        
        println!("Sending again...");
        tx.send("World").unwrap();
        println!("Sent again!");
    });

    thread::sleep(Duration::from_secs(1));
    println!("Receiving...");
    let msg1 = rx.recv().unwrap();
    println!("Received: {}", msg1);

    thread::sleep(Duration::from_secs(1));
    println!("Receiving again...");
    let msg2 = rx.recv().unwrap();
    println!("Received: {}", msg2);

    handle.join().unwrap();
}
```

### 2.4 通道选择

```rust
use std::sync::mpsc;
use std::thread;
use std::time::Duration;

fn main() {
    let (tx1, rx1) = mpsc::channel();
    let (tx2, rx2) = mpsc::channel();

    // 生产者1
    thread::spawn(move || {
        thread::sleep(Duration::from_millis(500));
        tx1.send("Message from channel 1").unwrap();
    });

    // 生产者2
    thread::spawn(move || {
        thread::sleep(Duration::from_millis(300));
        tx2.send("Message from channel 2").unwrap();
    });

    // 使用select!宏（需要crossbeam-channel crate）
    // 这里展示手动轮询的方式
    loop {
        match rx1.try_recv() {
            Ok(msg) => {
                println!("Received from channel 1: {}", msg);
                break;
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => break,
        }

        match rx2.try_recv() {
            Ok(msg) => {
                println!("Received from channel 2: {}", msg);
                break;
            }
            Err(mpsc::TryRecvError::Empty) => {}
            Err(mpsc::TryRecvError::Disconnected) => break,
        }

        thread::sleep(Duration::from_millis(50));
    }
}
```

## 3. 共享状态并发

### 3.1 Mutex（互斥锁）

```rust
use std::sync::{Arc, Mutex};
use std::thread;

fn main() {
    // 使用Arc和Mutex共享数据
    let counter = Arc::new(Mutex::new(0));
    let mut handles = vec![];

    for _ in 0..10 {
        let counter = Arc::clone(&counter);
        let handle = thread::spawn(move || {
            let mut num = counter.lock().unwrap();
            *num += 1;
        });
        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }

    println!("Result: {}", *counter.lock().unwrap());
}
```

### 3.2 RwLock（读写锁）

```rust
use std::sync::{Arc, RwLock};
use std::thread;
use std::time::Duration;

fn main() {
    let data = Arc::new(RwLock::new(vec![1, 2, 3, 4, 5]));
    let mut handles = vec![];

    // 多个读者
    for i in 0..3 {
        let data = Arc::clone(&data);
        let handle = thread::spawn(move || {
            let reader = data.read().unwrap();
            println!("Reader {}: {:?}", i, *reader);
            thread::sleep(Duration::from_millis(100));
        });
        handles.push(handle);
    }

    // 一个写者
    let data_writer = Arc::clone(&data);
    let writer_handle = thread::spawn(move || {
        thread::sleep(Duration::from_millis(50));
        let mut writer = data_writer.write().unwrap();
        writer.push(6);
        println!("Writer: Added 6 to the vector");
    });
    handles.push(writer_handle);

    for handle in handles {
        handle.join().unwrap();
    }

    println!("Final data: {:?}", *data.read().unwrap());
}
```

### 3.3 Atomic类型

```rust
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;

fn main() {
    let counter = Arc::new(AtomicUsize::new(0));
    let mut handles = vec![];

    for _ in 0..10 {
        let counter = Arc::clone(&counter);
        let handle = thread::spawn(move || {
            for _ in 0..1000 {
                counter.fetch_add(1, Ordering::SeqCst);
            }
        });
        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }

    println!("Result: {}", counter.load(Ordering::SeqCst));
}
```

### 3.4 条件变量

```rust
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;

fn main() {
    let pair = Arc::new((Mutex::new(false), Condvar::new()));
    let pair2 = Arc::clone(&pair);

    // 等待线程
    thread::spawn(move || {
        let (lock, cvar) = &*pair2;
        let mut started = lock.lock().unwrap();
        
        while !*started {
            println!("Waiting for condition...");
            started = cvar.wait(started).unwrap();
        }
        
        println!("Condition met! Proceeding...");
    });

    // 通知线程
    thread::sleep(Duration::from_secs(2));
    
    let (lock, cvar) = &*pair;
    let mut started = lock.lock().unwrap();
    *started = true;
    cvar.notify_one();
    
    println!("Notified waiting thread");
    
    thread::sleep(Duration::from_secs(1));
}
```

## 4. Send和Sync特征

### 4.1 Send特征

```rust
use std::thread;
use std::rc::Rc;

fn main() {
    // i32实现了Send，可以在线程间传递
    let x = 5;
    thread::spawn(move || {
        println!("Value: {}", x);
    }).join().unwrap();

    // Rc<T>没有实现Send，不能在线程间传递
    let rc = Rc::new(5);
    // 下面的代码会编译错误
    // thread::spawn(move || {
    //     println!("RC value: {}", *rc);
    // });
}
```

### 4.2 Sync特征

```rust
use std::sync::{Arc, Mutex};
use std::thread;
use std::cell::RefCell;

fn main() {
    // Mutex<T>实现了Sync，可以在多个线程间共享引用
    let data = Arc::new(Mutex::new(0));
    let mut handles = vec![];

    for _ in 0..3 {
        let data = Arc::clone(&data);
        let handle = thread::spawn(move || {
            let mut num = data.lock().unwrap();
            *num += 1;
        });
        handles.push(handle);
    }

    for handle in handles {
        handle.join().unwrap();
    }

    // RefCell<T>没有实现Sync，不能在多个线程间共享引用
    let cell = RefCell::new(0);
    // 下面的代码会编译错误
    // let cell_ref = &cell;
    // thread::spawn(move || {
    //     *cell_ref.borrow_mut() += 1;
    // });
}
```

### 4.3 自定义类型的Send和Sync

```rust
use std::marker::PhantomData;
use std::thread;

// 自动实现Send和Sync（因为所有字段都实现了Send和Sync）
#[derive(Debug)]
struct SafeStruct {
    data: i32,
    name: String,
}

// 手动实现Send和Sync
struct CustomStruct {
    data: *const i32, // 原始指针不实现Send/Sync
    _marker: PhantomData<i32>,
}

// 如果我们确定这个类型是线程安全的，可以手动实现
unsafe impl Send for CustomStruct {}
unsafe impl Sync for CustomStruct {}

impl CustomStruct {
    fn new(value: i32) -> Self {
        let boxed = Box::new(value);
        CustomStruct {
            data: Box::into_raw(boxed),
            _marker: PhantomData,
        }
    }
    
    fn get(&self) -> i32 {
        unsafe { *self.data }
    }
}

impl Drop for CustomStruct {
    fn drop(&mut self) {
        unsafe {
            Box::from_raw(self.data as *mut i32);
        }
    }
}

fn main() {
    let safe = SafeStruct {
        data: 42,
        name: "Safe".to_string(),
    };

    thread::spawn(move || {
        println!("Safe struct: {:?}", safe);
    }).join().unwrap();

    let custom = CustomStruct::new(100);
    thread::spawn(move || {
        println!("Custom struct value: {}", custom.get());
    }).join().unwrap();
}
```

## 5. 高级并发模式

### 5.1 工作窃取队列

```rust
use std::sync::{Arc, Mutex};
use std::thread;
use std::collections::VecDeque;

struct WorkStealingQueue<T> {
    queues: Vec<Arc<Mutex<VecDeque<T>>>>,
    current: usize,
}

impl<T> WorkStealingQueue<T> {
    fn new(num_workers: usize) -> Self {
        let mut queues = Vec::new();
        for _ in 0..num_workers {
            queues.push(Arc::new(Mutex::new(VecDeque::new())));
        }
        
        WorkStealingQueue {
            queues,
            current: 0,
        }
    }
    
    fn push(&mut self, item: T) {
        let queue = &self.queues[self.current];
        queue.lock().unwrap().push_back(item);
        self.current = (self.current + 1) % self.queues.len();
    }
    
    fn steal(&self, worker_id: usize) -> Option<T> {
        // 先尝试从自己的队列取任务
        if let Some(item) = self.queues[worker_id].lock().unwrap().pop_front() {
            return Some(item);
        }
        
        // 然后尝试从其他队列窃取任务
        for i in 0..self.queues.len() {
            if i != worker_id {
                if let Some(item) = self.queues[i].lock().unwrap().pop_back() {
                    return Some(item);
                }
            }
        }
        
        None
    }
}

fn main() {
    let num_workers = 4;
    let mut queue = WorkStealingQueue::new(num_workers);
    
    // 添加任务
    for i in 0..20 {
        queue.push(i);
    }
    
    let queue = Arc::new(queue);
    let mut handles = vec![];
    
    for worker_id in 0..num_workers {
        let queue = Arc::clone(&queue);
        let handle = thread::spawn(move || {
            let mut processed = 0;
            
            while let Some(task) = queue.steal(worker_id) {
                println!("Worker {} processing task {}", worker_id, task);
                thread::sleep(std::time::Duration::from_millis(100));
                processed += 1;
            }
            
            println!("Worker {} processed {} tasks", worker_id, processed);
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
}
```

### 5.2 生产者-消费者模式

```rust
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;
use std::collections::VecDeque;

struct BoundedQueue<T> {
    queue: Mutex<VecDeque<T>>,
    not_empty: Condvar,
    not_full: Condvar,
    capacity: usize,
}

impl<T> BoundedQueue<T> {
    fn new(capacity: usize) -> Self {
        BoundedQueue {
            queue: Mutex::new(VecDeque::new()),
            not_empty: Condvar::new(),
            not_full: Condvar::new(),
            capacity,
        }
    }
    
    fn push(&self, item: T) {
        let mut queue = self.queue.lock().unwrap();
        
        while queue.len() >= self.capacity {
            queue = self.not_full.wait(queue).unwrap();
        }
        
        queue.push_back(item);
        self.not_empty.notify_one();
    }
    
    fn pop(&self) -> T {
        let mut queue = self.queue.lock().unwrap();
        
        while queue.is_empty() {
            queue = self.not_empty.wait(queue).unwrap();
        }
        
        let item = queue.pop_front().unwrap();
        self.not_full.notify_one();
        item
    }
}

fn main() {
    let queue = Arc::new(BoundedQueue::new(5));
    let mut handles = vec![];
    
    // 生产者
    for i in 0..3 {
        let queue = Arc::clone(&queue);
        let handle = thread::spawn(move || {
            for j in 0..5 {
                let item = format!("Producer-{}-Item-{}", i, j);
                println!("Producing: {}", item);
                queue.push(item);
                thread::sleep(Duration::from_millis(200));
            }
        });
        handles.push(handle);
    }
    
    // 消费者
    for i in 0..2 {
        let queue = Arc::clone(&queue);
        let handle = thread::spawn(move || {
            for _ in 0..7 {
                let item = queue.pop();
                println!("Consumer-{} consumed: {}", i, item);
                thread::sleep(Duration::from_millis(300));
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
}
```

### 5.3 线程池实现

```rust
use std::sync::{Arc, Mutex, mpsc};
use std::thread;

type Job = Box<dyn FnOnce() + Send + 'static>;

enum Message {
    NewJob(Job),
    Terminate,
}

struct Worker {
    id: usize,
    thread: Option<thread::JoinHandle<()>>,
}

impl Worker {
    fn new(id: usize, receiver: Arc<Mutex<mpsc::Receiver<Message>>>) -> Worker {
        let thread = thread::spawn(move || loop {
            let message = receiver.lock().unwrap().recv().unwrap();
            
            match message {
                Message::NewJob(job) => {
                    println!("Worker {} got a job; executing.", id);
                    job();
                }
                Message::Terminate => {
                    println!("Worker {} was told to terminate.", id);
                    break;
                }
            }
        });
        
        Worker {
            id,
            thread: Some(thread),
        }
    }
}

pub struct ThreadPool {
    workers: Vec<Worker>,
    sender: mpsc::Sender<Message>,
}

impl ThreadPool {
    pub fn new(size: usize) -> ThreadPool {
        assert!(size > 0);
        
        let (sender, receiver) = mpsc::channel();
        let receiver = Arc::new(Mutex::new(receiver));
        let mut workers = Vec::with_capacity(size);
        
        for id in 0..size {
            workers.push(Worker::new(id, Arc::clone(&receiver)));
        }
        
        ThreadPool { workers, sender }
    }
    
    pub fn execute<F>(&self, f: F)
    where
        F: FnOnce() + Send + 'static,
    {
        let job = Box::new(f);
        self.sender.send(Message::NewJob(job)).unwrap();
    }
}

impl Drop for ThreadPool {
    fn drop(&mut self) {
        println!("Sending terminate message to all workers.");
        
        for _ in &self.workers {
            self.sender.send(Message::Terminate).unwrap();
        }
        
        println!("Shutting down all workers.");
        
        for worker in &mut self.workers {
            println!("Shutting down worker {}", worker.id);
            
            if let Some(thread) = worker.thread.take() {
                thread.join().unwrap();
            }
        }
    }
}

fn main() {
    let pool = ThreadPool::new(4);
    
    for i in 0..8 {
        pool.execute(move || {
            println!("Task {} is running on thread {:?}", i, thread::current().id());
            thread::sleep(std::time::Duration::from_secs(1));
            println!("Task {} completed", i);
        });
    }
    
    thread::sleep(std::time::Duration::from_secs(5));
    println!("Shutting down.");
}
```

## 6. 性能考虑和最佳实践

### 6.1 避免过度同步

```rust
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Instant;

// 不好的做法：过度同步
fn bad_example() {
    let data = Arc::new(Mutex::new(0));
    let mut handles = vec![];
    
    let start = Instant::now();
    
    for _ in 0..1000 {
        let data = Arc::clone(&data);
        let handle = thread::spawn(move || {
            for _ in 0..1000 {
                let mut num = data.lock().unwrap();
                *num += 1;
                // 锁持有时间过长
                thread::sleep(std::time::Duration::from_nanos(1));
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Bad example took: {:?}", start.elapsed());
}

// 好的做法：减少锁竞争
fn good_example() {
    let data = Arc::new(Mutex::new(0));
    let mut handles = vec![];
    
    let start = Instant::now();
    
    for _ in 0..1000 {
        let data = Arc::clone(&data);
        let handle = thread::spawn(move || {
            let mut local_sum = 0;
            for _ in 0..1000 {
                local_sum += 1;
                thread::sleep(std::time::Duration::from_nanos(1));
            }
            // 只在最后更新共享状态
            let mut num = data.lock().unwrap();
            *num += local_sum;
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Good example took: {:?}", start.elapsed());
}

fn main() {
    bad_example();
    good_example();
}
```

### 6.2 选择合适的同步原语

```rust
use std::sync::{Arc, Mutex, RwLock};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::thread;
use std::time::Instant;

fn compare_synchronization_primitives() {
    const NUM_THREADS: usize = 8;
    const NUM_OPERATIONS: usize = 1_000_000;
    
    // 测试Mutex
    let mutex_counter = Arc::new(Mutex::new(0usize));
    let start = Instant::now();
    let mut handles = vec![];
    
    for _ in 0..NUM_THREADS {
        let counter = Arc::clone(&mutex_counter);
        let handle = thread::spawn(move || {
            for _ in 0..NUM_OPERATIONS / NUM_THREADS {
                let mut num = counter.lock().unwrap();
                *num += 1;
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Mutex took: {:?}", start.elapsed());
    
    // 测试AtomicUsize
    let atomic_counter = Arc::new(AtomicUsize::new(0));
    let start = Instant::now();
    let mut handles = vec![];
    
    for _ in 0..NUM_THREADS {
        let counter = Arc::clone(&atomic_counter);
        let handle = thread::spawn(move || {
            for _ in 0..NUM_OPERATIONS / NUM_THREADS {
                counter.fetch_add(1, Ordering::Relaxed);
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Atomic took: {:?}", start.elapsed());
    
    // 测试RwLock（读多写少场景）
    let rwlock_data = Arc::new(RwLock::new(vec![0; 1000]));
    let start = Instant::now();
    let mut handles = vec![];
    
    // 大部分是读操作
    for _ in 0..NUM_THREADS - 1 {
        let data = Arc::clone(&rwlock_data);
        let handle = thread::spawn(move || {
            for _ in 0..NUM_OPERATIONS / NUM_THREADS {
                let _reader = data.read().unwrap();
                // 模拟读操作
            }
        });
        handles.push(handle);
    }
    
    // 少量写操作
    let data = Arc::clone(&rwlock_data);
    let handle = thread::spawn(move || {
        for i in 0..NUM_OPERATIONS / NUM_THREADS {
            let mut writer = data.write().unwrap();
            writer[i % writer.len()] += 1;
        }
    });
    handles.push(handle);
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("RwLock took: {:?}", start.elapsed());
}

fn main() {
    compare_synchronization_primitives();
}
```

### 6.3 内存排序和原子操作

```rust
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::Arc;
use std::thread;

fn memory_ordering_example() {
    let flag = Arc::new(AtomicBool::new(false));
    let data = Arc::new(AtomicUsize::new(0));
    
    let flag_clone = Arc::clone(&flag);
    let data_clone = Arc::clone(&data);
    
    // 写线程
    let writer = thread::spawn(move || {
        data_clone.store(42, Ordering::Relaxed);
        flag_clone.store(true, Ordering::Release); // Release语义
    });
    
    // 读线程
    let reader = thread::spawn(move || {
        while !flag.load(Ordering::Acquire) { // Acquire语义
            thread::yield_now();
        }
        let value = data.load(Ordering::Relaxed);
        println!("Read value: {}", value); // 保证读到42
    });
    
    writer.join().unwrap();
    reader.join().unwrap();
}

fn compare_and_swap_example() {
    let value = Arc::new(AtomicUsize::new(0));
    let mut handles = vec![];
    
    for _ in 0..10 {
        let value = Arc::clone(&value);
        let handle = thread::spawn(move || {
            loop {
                let current = value.load(Ordering::Acquire);
                let new_value = current + 1;
                
                // 原子性的比较并交换
                match value.compare_exchange_weak(
                    current,
                    new_value,
                    Ordering::Release,
                    Ordering::Relaxed,
                ) {
                    Ok(_) => break,
                    Err(_) => continue, // 重试
                }
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Final value: {}", value.load(Ordering::Acquire));
}

fn main() {
    memory_ordering_example();
    compare_and_swap_example();
}
```

## 7. 实践示例

### 7.1 并行计算：矩阵乘法

```rust
use std::sync::{Arc, Mutex};
use std::thread;

type Matrix = Vec<Vec<f64>>;

fn matrix_multiply_sequential(a: &Matrix, b: &Matrix) -> Matrix {
    let rows_a = a.len();
    let cols_a = a[0].len();
    let cols_b = b[0].len();
    
    let mut result = vec![vec![0.0; cols_b]; rows_a];
    
    for i in 0..rows_a {
        for j in 0..cols_b {
            for k in 0..cols_a {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    
    result
}

fn matrix_multiply_parallel(a: &Matrix, b: &Matrix, num_threads: usize) -> Matrix {
    let rows_a = a.len();
    let cols_b = b[0].len();
    
    let result = Arc::new(Mutex::new(vec![vec![0.0; cols_b]; rows_a]));
    let a = Arc::new(a.clone());
    let b = Arc::new(b.clone());
    
    let mut handles = vec![];
    let chunk_size = (rows_a + num_threads - 1) / num_threads;
    
    for thread_id in 0..num_threads {
        let start_row = thread_id * chunk_size;
        let end_row = std::cmp::min(start_row + chunk_size, rows_a);
        
        if start_row >= rows_a {
            break;
        }
        
        let result = Arc::clone(&result);
        let a = Arc::clone(&a);
        let b = Arc::clone(&b);
        
        let handle = thread::spawn(move || {
            let mut local_result = vec![vec![0.0; cols_b]; end_row - start_row];
            
            for i in 0..(end_row - start_row) {
                for j in 0..cols_b {
                    for k in 0..a[0].len() {
                        local_result[i][j] += a[start_row + i][k] * b[k][j];
                    }
                }
            }
            
            // 将本地结果合并到全局结果
            let mut global_result = result.lock().unwrap();
            for i in 0..(end_row - start_row) {
                for j in 0..cols_b {
                    global_result[start_row + i][j] = local_result[i][j];
                }
            }
        });
        
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    Arc::try_unwrap(result).unwrap().into_inner().unwrap()
}

fn main() {
    // 创建测试矩阵
    let size = 100;
    let a: Matrix = (0..size)
        .map(|i| (0..size).map(|j| (i * size + j) as f64).collect())
        .collect();
    let b: Matrix = (0..size)
        .map(|i| (0..size).map(|j| ((i + j) % 10) as f64).collect())
        .collect();
    
    // 顺序计算
    let start = std::time::Instant::now();
    let _result_seq = matrix_multiply_sequential(&a, &b);
    let seq_time = start.elapsed();
    
    // 并行计算
    let start = std::time::Instant::now();
    let _result_par = matrix_multiply_parallel(&a, &b, 4);
    let par_time = start.elapsed();
    
    println!("Sequential time: {:?}", seq_time);
    println!("Parallel time: {:?}", par_time);
    println!("Speedup: {:.2}x", seq_time.as_secs_f64() / par_time.as_secs_f64());
}
```

### 7.2 Web服务器并发处理

```rust
use std::io::prelude::*;
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use std::collections::HashMap;

struct RequestCounter {
    counts: Arc<Mutex<HashMap<String, usize>>>,
}

impl RequestCounter {
    fn new() -> Self {
        RequestCounter {
            counts: Arc::new(Mutex::new(HashMap::new())),
        }
    }
    
    fn increment(&self, path: &str) {
        let mut counts = self.counts.lock().unwrap();
        *counts.entry(path.to_string()).or_insert(0) += 1;
    }
    
    fn get_stats(&self) -> HashMap<String, usize> {
        self.counts.lock().unwrap().clone()
    }
}

fn handle_connection(mut stream: TcpStream, counter: Arc<RequestCounter>) {
    let mut buffer = [0; 1024];
    stream.read(&mut buffer).unwrap();
    
    let request = String::from_utf8_lossy(&buffer[..]);
    let request_line = request.lines().next().unwrap_or("");
    let path = request_line.split_whitespace().nth(1).unwrap_or("/");
    
    counter.increment(path);
    
    let (status_line, contents) = match path {
        "/" => ("HTTP/1.1 200 OK", "<h1>Welcome to Rust Web Server!</h1>"),
        "/slow" => {
            thread::sleep(Duration::from_secs(2));
            ("HTTP/1.1 200 OK", "<h1>Slow response</h1>")
        }
        "/stats" => {
            let stats = counter.get_stats();
            let stats_html = format!(
                "<h1>Request Statistics</h1><ul>{}</ul>",
                stats.iter()
                    .map(|(path, count)| format!("<li>{}: {}</li>", path, count))
                    .collect::<Vec<_>>()
                    .join("")
            );
            ("HTTP/1.1 200 OK", &stats_html)
        }
        _ => ("HTTP/1.1 404 NOT FOUND", "<h1>404 Not Found</h1>"),
    };
    
    let response = format!(
        "{}\r\nContent-Length: {}\r\n\r\n{}",
        status_line,
        contents.len(),
        contents
    );
    
    stream.write(response.as_bytes()).unwrap();
    stream.flush().unwrap();
}

fn main() -> std::io::Result<()> {
    let listener = TcpListener::bind("127.0.0.1:7878")?;
    let counter = Arc::new(RequestCounter::new());
    
    println!("Server running on http://127.0.0.1:7878");
    
    for stream in listener.incoming() {
        let stream = stream?;
        let counter = Arc::clone(&counter);
        
        thread::spawn(move || {
            handle_connection(stream, counter);
        });
    }
    
    Ok(())
}
```

## 8. 练习题

### 练习1：实现一个线程安全的计数器

```rust
use std::sync::{Arc, Mutex};
use std::thread;

struct SafeCounter {
    value: Arc<Mutex<i32>>,
}

impl SafeCounter {
    fn new() -> Self {
        SafeCounter {
            value: Arc::new(Mutex::new(0)),
        }
    }
    
    fn increment(&self) {
        let mut val = self.value.lock().unwrap();
        *val += 1;
    }
    
    fn decrement(&self) {
        let mut val = self.value.lock().unwrap();
        *val -= 1;
    }
    
    fn get(&self) -> i32 {
        *self.value.lock().unwrap()
    }
    
    fn clone(&self) -> Self {
        SafeCounter {
            value: Arc::clone(&self.value),
        }
    }
}

fn main() {
    let counter = SafeCounter::new();
    let mut handles = vec![];
    
    // 10个线程，每个增加100次
    for _ in 0..10 {
        let counter = counter.clone();
        let handle = thread::spawn(move || {
            for _ in 0..100 {
                counter.increment();
            }
        });
        handles.push(handle);
    }
    
    // 5个线程，每个减少50次
    for _ in 0..5 {
        let counter = counter.clone();
        let handle = thread::spawn(move || {
            for _ in 0..50 {
                counter.decrement();
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
    
    println!("Final counter value: {}", counter.get());
    // 应该输出: 10 * 100 - 5 * 50 = 750
}
```

### 练习2：实现生产者-消费者模式

```rust
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::Duration;
use std::collections::VecDeque;

struct Buffer<T> {
    queue: Mutex<VecDeque<T>>,
    not_empty: Condvar,
    not_full: Condvar,
    capacity: usize,
}

impl<T> Buffer<T> {
    fn new(capacity: usize) -> Self {
        Buffer {
            queue: Mutex::new(VecDeque::new()),
            not_empty: Condvar::new(),
            not_full: Condvar::new(),
            capacity,
        }
    }
    
    fn produce(&self, item: T) {
        let mut queue = self.queue.lock().unwrap();
        
        while queue.len() >= self.capacity {
            queue = self.not_full.wait(queue).unwrap();
        }
        
        queue.push_back(item);
        self.not_empty.notify_one();
    }
    
    fn consume(&self) -> T {
        let mut queue = self.queue.lock().unwrap();
        
        while queue.is_empty() {
            queue = self.not_empty.wait(queue).unwrap();
        }
        
        let item = queue.pop_front().unwrap();
        self.not_full.notify_one();
        item
    }
}

fn main() {
    let buffer = Arc::new(Buffer::new(3));
    let mut handles = vec![];
    
    // 生产者
    for i in 0..2 {
        let buffer = Arc::clone(&buffer);
        let handle = thread::spawn(move || {
            for j in 0..5 {
                let item = format!("Item-{}-{}", i, j);
                println!("Producing: {}", item);
                buffer.produce(item);
                thread::sleep(Duration::from_millis(100));
            }
        });
        handles.push(handle);
    }
    
    // 消费者
    for i in 0..3 {
        let buffer = Arc::clone(&buffer);
        let handle = thread::spawn(move || {
            for _ in 0..3 {
                let item = buffer.consume();
                println!("Consumer-{} consumed: {}", i, item);
                thread::sleep(Duration::from_millis(200));
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.join().unwrap();
    }
}
```

### 练习3：实现一个简单的任务调度器

```rust
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use std::collections::BinaryHeap;
use std::cmp::Ordering;

type Task = Box<dyn FnOnce() + Send>;

#[derive(Debug)]
struct ScheduledTask {
    task: Option<Task>,
    execute_at: Instant,
    id: usize,
}

impl PartialEq for ScheduledTask {
    fn eq(&self, other: &Self) -> bool {
        self.execute_at == other.execute_at && self.id == other.id
    }
}

impl Eq for ScheduledTask {}

impl PartialOrd for ScheduledTask {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ScheduledTask {
    fn cmp(&self, other: &Self) -> Ordering {
        // 反转排序，使最早的任务在堆顶
        other.execute_at.cmp(&self.execute_at)
            .then_with(|| other.id.cmp(&self.id))
    }
}

struct TaskScheduler {
    tasks: Arc<Mutex<BinaryHeap<ScheduledTask>>>,
    condvar: Arc<Condvar>,
    next_id: Arc<Mutex<usize>>,
    running: Arc<Mutex<bool>>,
}

impl TaskScheduler {
    fn new() -> Self {
        TaskScheduler {
            tasks: Arc::new(Mutex::new(BinaryHeap::new())),
            condvar: Arc::new(Condvar::new()),
            next_id: Arc::new(Mutex::new(0)),
            running: Arc::new(Mutex::new(true)),
        }
    }
    
    fn schedule<F>(&self, delay: Duration, task: F)
    where
        F: FnOnce() + Send + 'static,
    {
        let execute_at = Instant::now() + delay;
        let id = {
            let mut next_id = self.next_id.lock().unwrap();
            let id = *next_id;
            *next_id += 1;
            id
        };
        
        let scheduled_task = ScheduledTask {
            task: Some(Box::new(task)),
            execute_at,
            id,
        };
        
        self.tasks.lock().unwrap().push(scheduled_task);
        self.condvar.notify_one();
    }
    
    fn run(&self) {
        loop {
            let mut tasks = self.tasks.lock().unwrap();
            
            if !*self.running.lock().unwrap() {
                break;
            }
            
            if let Some(mut scheduled_task) = tasks.peek_mut() {
                let now = Instant::now();
                if scheduled_task.execute_at <= now {
                    let mut task = tasks.pop().unwrap();
                    drop(tasks); // 释放锁
                    
                    if let Some(task_fn) = task.task.take() {
                        println!("Executing task {} at {:?}", task.id, now);
                        task_fn();
                    }
                    continue;
                }
                
                // 等待直到下一个任务时间
                let wait_time = scheduled_task.execute_at - now;
                let _ = self.condvar.wait_timeout(tasks, wait_time);
            } else {
                // 没有任务，等待新任务
                tasks = self.condvar.wait(tasks).unwrap();
            }
        }
    }
    
    fn stop(&self) {
        *self.running.lock().unwrap() = false;
        self.condvar.notify_all();
    }
}

fn main() {
    let scheduler = Arc::new(TaskScheduler::new());
    
    // 启动调度器线程
    let scheduler_clone = Arc::clone(&scheduler);
    let scheduler_handle = thread::spawn(move || {
        scheduler_clone.run();
    });
    
    // 调度一些任务
    scheduler.schedule(Duration::from_millis(100), || {
        println!("Task 1 executed!");
    });
    
    scheduler.schedule(Duration::from_millis(50), || {
        println!("Task 2 executed!");
    });
    
    scheduler.schedule(Duration::from_millis(200), || {
        println!("Task 3 executed!");
    });
    
    scheduler.schedule(Duration::from_millis(150), || {
        println!("Task 4 executed!");
    });
    
    // 等待任务执行
    thread::sleep(Duration::from_millis(300));
    
    // 停止调度器
    scheduler.stop();
    scheduler_handle.join().unwrap();
    
    println!("Scheduler stopped");
}
```

## 总结

通过本章的学习，你应该掌握了：

### 核心概念
- 线程的创建、管理和同步
- 消息传递和共享状态两种并发模式
- `Send`和`Sync`特征的含义和应用
- 各种同步原语的使用场景

### 实用技能
- 选择合适的并发模式和同步原语
- 实现线程安全的数据结构
- 避免常见的并发问题（死锁、数据竞争等）
- 性能优化和最佳实践

### 最佳实践
- 优先使用消息传递而非共享状态
- 最小化锁的持有时间
- 使用原子操作处理简单的共享数据
- 合理设计并发架构

### 高级特性
- 内存排序和原子操作的细节
- 自定义同步原语的实现
- 工作窃取和任务调度算法
- 并行计算模式

Rust的并发编程模型通过类型系统保证了内存安全和线程安全，让你能够编写高性能且可靠的并发程序。掌握这些概念和技术将帮助你构建可扩展的系统和应用。
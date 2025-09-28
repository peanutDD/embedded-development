// Rust并发编程示例：展示线程、消息传递、共享状态等概念

use std::collections::HashMap;
use std::sync::{mpsc, Arc, Condvar, Mutex, RwLock};
use std::thread;
use std::time::Duration;

// 1. 基础线程创建和管理
fn basic_threading() {
  println!("=== 基础线程示例 ===");

  // 创建线程
  let handle = thread::spawn(|| {
    for i in 1..10 {
      println!("子线程中的数字: {}", i);
      thread::sleep(Duration::from_millis(1));
    }
  });

  // 主线程工作
  for i in 1..5 {
    println!("主线程中的数字: {}", i);
    thread::sleep(Duration::from_millis(1));
  }

  // 等待子线程完成
  handle.join().unwrap();
  println!("所有线程完成\n");
}

// 2. 线程间消息传递
fn message_passing() {
  println!("=== 消息传递示例 ===");

  let (tx, rx) = mpsc::channel();

  // 生产者线程
  let producer = thread::spawn(move || {
    let vals = vec![
      String::from("消息1"),
      String::from("消息2"),
      String::from("消息3"),
      String::from("消息4"),
    ];

    for val in vals {
      tx.send(val).unwrap();
      thread::sleep(Duration::from_millis(100));
    }
  });

  // 消费者（主线程）
  for received in rx {
    println!("接收到: {}", received);
  }

  producer.join().unwrap();
  println!("消息传递完成\n");
}

// 3. 多生产者单消费者
fn multiple_producers() {
  println!("=== 多生产者示例 ===");

  let (tx, rx) = mpsc::channel();

  // 创建多个生产者
  let mut handles = vec![];

  for id in 0..3 {
    let tx_clone = tx.clone();
    let handle = thread::spawn(move || {
      for i in 0..5 {
        let msg = format!("生产者{}: 消息{}", id, i);
        tx_clone.send(msg).unwrap();
        thread::sleep(Duration::from_millis(50));
      }
    });
    handles.push(handle);
  }

  // 释放原始发送者
  drop(tx);

  // 接收所有消息
  for received in rx {
    println!("接收到: {}", received);
  }

  // 等待所有生产者完成
  for handle in handles {
    handle.join().unwrap();
  }
  println!("多生产者完成\n");
}

// 4. 共享状态：Mutex
fn shared_state_mutex() {
  println!("=== 共享状态Mutex示例 ===");

  let counter = Arc::new(Mutex::new(0));
  let mut handles = vec![];

  for i in 0..10 {
    let counter = Arc::clone(&counter);
    let handle = thread::spawn(move || {
      let mut num = counter.lock().unwrap();
      *num += 1;
      println!("线程{}: 计数器 = {}", i, *num);
      thread::sleep(Duration::from_millis(10));
    });
    handles.push(handle);
  }

  for handle in handles {
    handle.join().unwrap();
  }

  println!("最终计数器值: {}\n", *counter.lock().unwrap());
}

// 5. 读写锁RwLock
fn rwlock_example() {
  println!("=== 读写锁示例 ===");

  let data = Arc::new(RwLock::new(HashMap::new()));
  let mut handles = vec![];

  // 写入线程
  for i in 0..3 {
    let data = Arc::clone(&data);
    let handle = thread::spawn(move || {
      let mut map = data.write().unwrap();
      map.insert(format!("key{}", i), i * 10);
      println!("写入线程{}: 插入 key{} = {}", i, i, i * 10);
      thread::sleep(Duration::from_millis(50));
    });
    handles.push(handle);
  }

  // 读取线程
  for i in 0..5 {
    let data = Arc::clone(&data);
    let handle = thread::spawn(move || {
      thread::sleep(Duration::from_millis(20)); // 等待一些数据被写入
      let map = data.read().unwrap();
      println!("读取线程{}: 当前数据 = {:?}", i, *map);
    });
    handles.push(handle);
  }

  for handle in handles {
    handle.join().unwrap();
  }
  println!("读写锁示例完成\n");
}

// 6. 条件变量Condvar
fn condvar_example() {
  println!("=== 条件变量示例 ===");

  let pair = Arc::new((Mutex::new(false), Condvar::new()));
  let pair2 = Arc::clone(&pair);

  // 等待线程
  let waiter = thread::spawn(move || {
    let (lock, cvar) = &*pair2;
    let mut started = lock.lock().unwrap();

    while !*started {
      println!("等待线程: 等待条件满足...");
      started = cvar.wait(started).unwrap();
    }

    println!("等待线程: 条件已满足，继续执行");
  });

  // 通知线程
  thread::sleep(Duration::from_millis(100));
  let (lock, cvar) = &*pair;
  let mut started = lock.lock().unwrap();
  *started = true;
  cvar.notify_one();
  println!("通知线程: 已发送通知");

  waiter.join().unwrap();
  println!("条件变量示例完成\n");
}

// 7. 线程池模拟
struct ThreadPool {
  workers: Vec<Worker>,
  sender: mpsc::Sender<Job>,
}

type Job = Box<dyn FnOnce() + Send + 'static>;

impl ThreadPool {
  fn new(size: usize) -> ThreadPool {
    assert!(size > 0);

    let (sender, receiver) = mpsc::channel();
    let receiver = Arc::new(Mutex::new(receiver));
    let mut workers = Vec::with_capacity(size);

    for id in 0..size {
      workers.push(Worker::new(id, Arc::clone(&receiver)));
    }

    ThreadPool { workers, sender }
  }

  fn execute<F>(&self, f: F)
  where
    F: FnOnce() + Send + 'static,
  {
    let job = Box::new(f);
    self.sender.send(job).unwrap();
  }
}

struct Worker {
  id: usize,
  thread: thread::JoinHandle<()>,
}

impl Worker {
  fn new(id: usize, receiver: Arc<Mutex<mpsc::Receiver<Job>>>) -> Worker {
    let thread = thread::spawn(move || loop {
      let job = receiver.lock().unwrap().recv().unwrap();
      println!("工作线程 {} 开始执行任务", id);
      job();
    });

    Worker { id, thread }
  }
}

fn thread_pool_example() {
  println!("=== 线程池示例 ===");

  let pool = ThreadPool::new(4);

  for i in 0..8 {
    pool.execute(move || {
      println!("任务 {} 正在执行", i);
      thread::sleep(Duration::from_millis(100));
      println!("任务 {} 完成", i);
    });
  }

  thread::sleep(Duration::from_millis(1000));
  println!("线程池示例完成\n");
}

// 8. 原子操作
use std::sync::atomic::{AtomicUsize, Ordering};

fn atomic_operations() {
  println!("=== 原子操作示例 ===");

  let counter = Arc::new(AtomicUsize::new(0));
  let mut handles = vec![];

  for i in 0..10 {
    let counter = Arc::clone(&counter);
    let handle = thread::spawn(move || {
      for _ in 0..1000 {
        counter.fetch_add(1, Ordering::SeqCst);
      }
      println!("线程 {} 完成", i);
    });
    handles.push(handle);
  }

  for handle in handles {
    handle.join().unwrap();
  }

  println!("原子计数器最终值: {}\n", counter.load(Ordering::SeqCst));
}

// 9. 异步任务模拟（使用线程）
fn async_task_simulation() {
  println!("=== 异步任务模拟 ===");

  let (tx, rx) = mpsc::channel();

  // 模拟异步任务
  let tasks = vec![
    ("下载文件A", 200),
    ("处理数据B", 150),
    ("上传结果C", 300),
    ("发送通知D", 100),
  ];

  let mut handles = vec![];

  for (task_name, duration) in tasks {
    let tx = tx.clone();
    let handle = thread::spawn(move || {
      println!("开始任务: {}", task_name);
      thread::sleep(Duration::from_millis(duration));
      tx.send(format!("任务完成: {}", task_name)).unwrap();
    });
    handles.push(handle);
  }

  drop(tx); // 关闭发送端

  // 收集结果
  for result in rx {
    println!("收到结果: {}", result);
  }

  for handle in handles {
    handle.join().unwrap();
  }
  println!("所有异步任务完成\n");
}

// 10. 生产者-消费者模式
fn producer_consumer_pattern() {
  println!("=== 生产者-消费者模式 ===");

  let buffer = Arc::new(Mutex::new(Vec::new()));
  let not_empty = Arc::new(Condvar::new());
  let not_full = Arc::new(Condvar::new());
  const BUFFER_SIZE: usize = 5;

  // 生产者
  let buffer_producer = Arc::clone(&buffer);
  let not_empty_producer = Arc::clone(&not_empty);
  let not_full_producer = Arc::clone(&not_full);

  let producer = thread::spawn(move || {
    for i in 0..10 {
      let mut buf = buffer_producer.lock().unwrap();

      // 等待缓冲区不满
      while buf.len() >= BUFFER_SIZE {
        buf = not_full_producer.wait(buf).unwrap();
      }

      buf.push(i);
      println!("生产者: 生产了 {}, 缓冲区大小: {}", i, buf.len());

      not_empty_producer.notify_one();
      thread::sleep(Duration::from_millis(50));
    }
  });

  // 消费者
  let buffer_consumer = Arc::clone(&buffer);
  let not_empty_consumer = Arc::clone(&not_empty);
  let not_full_consumer = Arc::clone(&not_full);

  let consumer = thread::spawn(move || {
    for _ in 0..10 {
      let mut buf = buffer_consumer.lock().unwrap();

      // 等待缓冲区不空
      while buf.is_empty() {
        buf = not_empty_consumer.wait(buf).unwrap();
      }

      let item = buf.remove(0);
      println!("消费者: 消费了 {}, 缓冲区大小: {}", item, buf.len());

      not_full_consumer.notify_one();
      thread::sleep(Duration::from_millis(100));
    }
  });

  producer.join().unwrap();
  consumer.join().unwrap();
  println!("生产者-消费者模式完成\n");
}

fn main() {
  println!("Rust并发编程综合示例\n");

  // 1. 基础线程
  basic_threading();

  // 2. 消息传递
  message_passing();

  // 3. 多生产者
  multiple_producers();

  // 4. 共享状态
  shared_state_mutex();

  // 5. 读写锁
  rwlock_example();

  // 6. 条件变量
  condvar_example();

  // 7. 线程池
  thread_pool_example();

  // 8. 原子操作
  atomic_operations();

  // 9. 异步任务模拟
  async_task_simulation();

  // 10. 生产者-消费者
  producer_consumer_pattern();

  println!("=== 并发安全性总结 ===");
  println!("Rust通过以下机制保证并发安全:");
  println!("1. 所有权系统防止数据竞争");
  println!("2. Send和Sync特征标记线程安全性");
  println!("3. 编译时检查防止并发错误");
  println!("4. 类型系统确保内存安全");
}

#[cfg(test)]
mod tests {
  use super::*;
  use std::sync::atomic::{AtomicBool, Ordering};

  #[test]
  fn test_atomic_operations() {
    let flag = AtomicBool::new(false);
    assert!(!flag.load(Ordering::SeqCst));

    flag.store(true, Ordering::SeqCst);
    assert!(flag.load(Ordering::SeqCst));
  }

  #[test]
  fn test_message_passing() {
    let (tx, rx) = mpsc::channel();

    thread::spawn(move || {
      tx.send("测试消息").unwrap();
    });

    let received = rx.recv().unwrap();
    assert_eq!(received, "测试消息");
  }

  #[test]
  fn test_shared_counter() {
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

    assert_eq!(*counter.lock().unwrap(), 10);
  }
}

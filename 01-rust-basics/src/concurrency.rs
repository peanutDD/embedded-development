//! # 并发编程工具
//! 
//! 本模块提供了针对嵌入式和标准环境的并发编程工具。
//! 
//! ## 特性
//! 
//! - **线程安全**: 提供线程安全的数据结构
//! - **无锁编程**: 原子操作和无锁数据结构
//! - **消息传递**: 通道和消息队列
//! - **同步原语**: 互斥锁、信号量等

use std::sync::{Arc, Mutex, RwLock, Condvar};
use std::sync::mpsc::{self, Sender, Receiver};
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::thread;
use std::time::Duration;
use crate::error_handling::{Error, Result};

/// 线程安全的计数器
pub struct AtomicCounter {
    value: AtomicUsize,
}

impl AtomicCounter {
    /// 创建新的原子计数器
    pub fn new(initial: usize) -> Self {
        Self {
            value: AtomicUsize::new(initial),
        }
    }
    
    /// 获取当前值
    pub fn get(&self) -> usize {
        self.value.load(Ordering::SeqCst)
    }
    
    /// 设置值
    pub fn set(&self, value: usize) {
        self.value.store(value, Ordering::SeqCst);
    }
    
    /// 原子递增
    pub fn increment(&self) -> usize {
        self.value.fetch_add(1, Ordering::SeqCst)
    }
    
    /// 原子递减
    pub fn decrement(&self) -> usize {
        self.value.fetch_sub(1, Ordering::SeqCst)
    }
    
    /// 原子加法
    pub fn add(&self, value: usize) -> usize {
        self.value.fetch_add(value, Ordering::SeqCst)
    }
    
    /// 比较并交换
    pub fn compare_and_swap(&self, current: usize, new: usize) -> usize {
        self.value.compare_exchange(current, new, Ordering::SeqCst, Ordering::SeqCst)
            .unwrap_or(current)
    }
}

/// 线程安全的标志
pub struct AtomicFlag {
    flag: AtomicBool,
}

impl AtomicFlag {
    /// 创建新的原子标志
    pub fn new(initial: bool) -> Self {
        Self {
            flag: AtomicBool::new(initial),
        }
    }
    
    /// 获取标志状态
    pub fn is_set(&self) -> bool {
        self.flag.load(Ordering::SeqCst)
    }
    
    /// 设置标志
    pub fn set(&self) {
        self.flag.store(true, Ordering::SeqCst);
    }
    
    /// 清除标志
    pub fn clear(&self) {
        self.flag.store(false, Ordering::SeqCst);
    }
    
    /// 切换标志状态
    pub fn toggle(&self) -> bool {
        !self.flag.fetch_xor(true, Ordering::SeqCst)
    }
    
    /// 测试并设置
    pub fn test_and_set(&self) -> bool {
        self.flag.swap(true, Ordering::SeqCst)
    }
}

/// 工作线程池
pub struct ThreadPool {
    workers: Vec<Worker>,
    sender: Option<Sender<Job>>,
}

type Job = Box<dyn FnOnce() + Send + 'static>;

impl ThreadPool {
    /// 创建新的线程池
    pub fn new(size: usize) -> Result<ThreadPool> {
        if size == 0 {
            return Err(Error::InvalidArgument("Thread pool size must be greater than 0"));
        }
        
        let (sender, receiver) = mpsc::channel();
        let receiver = Arc::new(Mutex::new(receiver));
        let mut workers = Vec::with_capacity(size);
        
        for id in 0..size {
            workers.push(Worker::new(id, Arc::clone(&receiver))?);
        }
        
        Ok(ThreadPool {
            workers,
            sender: Some(sender),
        })
    }
    
    /// 执行任务
    pub fn execute<F>(&self, f: F) -> Result<()>
    where
        F: FnOnce() + Send + 'static,
    {
        let job = Box::new(f);
        
        if let Some(ref sender) = self.sender {
            sender.send(job).map_err(|_| Error::CommunicationError)?;
        }
        
        Ok(())
    }
    
    /// 获取工作线程数量
    pub fn size(&self) -> usize {
        self.workers.len()
    }
}

impl Drop for ThreadPool {
    fn drop(&mut self) {
        drop(self.sender.take());
        
        for worker in &mut self.workers {
            if let Some(thread) = worker.thread.take() {
                thread.join().unwrap();
            }
        }
    }
}

struct Worker {
    id: usize,
    thread: Option<thread::JoinHandle<()>>,
}

impl Worker {
    fn new(id: usize, receiver: Arc<Mutex<Receiver<Job>>>) -> Result<Worker> {
        let thread = thread::spawn(move || loop {
            let job = receiver.lock().unwrap().recv();
            
            match job {
                Ok(job) => {
                    job();
                }
                Err(_) => {
                    break;
                }
            }
        });
        
        Ok(Worker {
            id,
            thread: Some(thread),
        })
    }
}

/// 生产者-消费者队列
pub struct ProducerConsumer<T> {
    queue: Arc<Mutex<Vec<T>>>,
    not_empty: Arc<Condvar>,
    not_full: Arc<Condvar>,
    capacity: usize,
}

impl<T> ProducerConsumer<T> {
    /// 创建新的生产者-消费者队列
    pub fn new(capacity: usize) -> Self {
        Self {
            queue: Arc::new(Mutex::new(Vec::new())),
            not_empty: Arc::new(Condvar::new()),
            not_full: Arc::new(Condvar::new()),
            capacity,
        }
    }
    
    /// 生产者添加项目
    pub fn produce(&self, item: T) -> Result<()> {
        let mut queue = self.queue.lock().unwrap();
        
        while queue.len() >= self.capacity {
            queue = self.not_full.wait(queue).unwrap();
        }
        
        queue.push(item);
        self.not_empty.notify_one();
        
        Ok(())
    }
    
    /// 消费者获取项目
    pub fn consume(&self) -> Result<T> {
        let mut queue = self.queue.lock().unwrap();
        
        while queue.is_empty() {
            queue = self.not_empty.wait(queue).unwrap();
        }
        
        let item = queue.remove(0);
        self.not_full.notify_one();
        
        Ok(item)
    }
    
    /// 尝试生产（非阻塞）
    pub fn try_produce(&self, item: T) -> Result<()> {
        let mut queue = self.queue.lock().unwrap();
        
        if queue.len() >= self.capacity {
            return Err(Error::InsufficientResources);
        }
        
        queue.push(item);
        self.not_empty.notify_one();
        
        Ok(())
    }
    
    /// 尝试消费（非阻塞）
    pub fn try_consume(&self) -> Result<T> {
        let mut queue = self.queue.lock().unwrap();
        
        if queue.is_empty() {
            return Err(Error::InsufficientResources);
        }
        
        let item = queue.remove(0);
        self.not_full.notify_one();
        
        Ok(item)
    }
    
    /// 获取队列长度
    pub fn len(&self) -> usize {
        self.queue.lock().unwrap().len()
    }
    
    /// 检查队列是否为空
    pub fn is_empty(&self) -> bool {
        self.queue.lock().unwrap().is_empty()
    }
}

/// 读写锁包装器
pub struct RwLockWrapper<T> {
    data: RwLock<T>,
}

impl<T> RwLockWrapper<T> {
    /// 创建新的读写锁包装器
    pub fn new(data: T) -> Self {
        Self {
            data: RwLock::new(data),
        }
    }
    
    /// 获取读锁
    pub fn read<F, R>(&self, f: F) -> Result<R>
    where
        F: FnOnce(&T) -> R,
    {
        let guard = self.data.read().map_err(|_| Error::CommunicationError)?;
        Ok(f(&*guard))
    }
    
    /// 获取写锁
    pub fn write<F, R>(&self, f: F) -> Result<R>
    where
        F: FnOnce(&mut T) -> R,
    {
        let mut guard = self.data.write().map_err(|_| Error::CommunicationError)?;
        Ok(f(&mut *guard))
    }
}

/// 并发工具函数
pub mod utils {
    use super::*;
    
    /// 并行映射
    pub fn parallel_map<T, U, F>(data: Vec<T>, f: F) -> Result<Vec<U>>
    where
        T: Send + 'static,
        U: Send + 'static,
        F: Fn(T) -> U + Send + Sync + 'static,
    {
        let pool = ThreadPool::new(4)?;
        let f = Arc::new(f);
        let results = Arc::new(Mutex::new(Vec::new()));
        
        for item in data {
            let f = Arc::clone(&f);
            let results = Arc::clone(&results);
            
            pool.execute(move || {
                let result = f(item);
                results.lock().unwrap().push(result);
            })?;
        }
        
        // 等待所有任务完成
        thread::sleep(Duration::from_millis(100));
        
        let results = Arc::try_unwrap(results)
            .map_err(|_| Error::CommunicationError)?
            .into_inner()
            .map_err(|_| Error::CommunicationError)?;
        
        Ok(results)
    }
    
    /// 并行过滤
    pub fn parallel_filter<T, F>(data: Vec<T>, predicate: F) -> Result<Vec<T>>
    where
        T: Send + Clone + 'static,
        F: Fn(&T) -> bool + Send + Sync + 'static,
    {
        let pool = ThreadPool::new(4)?;
        let predicate = Arc::new(predicate);
        let results = Arc::new(Mutex::new(Vec::new()));
        
        for item in data {
            let predicate = Arc::clone(&predicate);
            let results = Arc::clone(&results);
            let item_clone = item.clone();
            
            pool.execute(move || {
                if predicate(&item_clone) {
                    results.lock().unwrap().push(item);
                }
            })?;
        }
        
        // 等待所有任务完成
        thread::sleep(Duration::from_millis(100));
        
        let results = Arc::try_unwrap(results)
            .map_err(|_| Error::CommunicationError)?
            .into_inner()
            .map_err(|_| Error::CommunicationError)?;
        
        Ok(results)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_atomic_counter() {
        let counter = AtomicCounter::new(0);
        
        assert_eq!(counter.get(), 0);
        
        counter.increment();
        assert_eq!(counter.get(), 1);
        
        counter.add(5);
        assert_eq!(counter.get(), 6);
        
        counter.decrement();
        assert_eq!(counter.get(), 5);
    }
    
    #[test]
    fn test_atomic_flag() {
        let flag = AtomicFlag::new(false);
        
        assert!(!flag.is_set());
        
        flag.set();
        assert!(flag.is_set());
        
        flag.clear();
        assert!(!flag.is_set());
        
        let old = flag.test_and_set();
        assert!(!old);
        assert!(flag.is_set());
    }
    
    #[test]
    fn test_thread_pool() {
        let pool = ThreadPool::new(4).unwrap();
        let counter = Arc::new(AtomicCounter::new(0));
        
        for _ in 0..10 {
            let counter = Arc::clone(&counter);
            pool.execute(move || {
                counter.increment();
            }).unwrap();
        }
        
        // 等待任务完成
        thread::sleep(Duration::from_millis(100));
        
        assert_eq!(counter.get(), 10);
    }
    
    #[test]
    fn test_producer_consumer() {
        let pc = ProducerConsumer::new(5);
        
        pc.produce(1).unwrap();
        pc.produce(2).unwrap();
        
        assert_eq!(pc.len(), 2);
        
        let item = pc.consume().unwrap();
        assert_eq!(item, 1);
        
        let item = pc.consume().unwrap();
        assert_eq!(item, 2);
        
        assert!(pc.is_empty());
    }
    
    #[test]
    fn test_rw_lock_wrapper() {
        let wrapper = RwLockWrapper::new(vec![1, 2, 3]);
        
        let len = wrapper.read(|data| data.len()).unwrap();
        assert_eq!(len, 3);
        
        wrapper.write(|data| data.push(4)).unwrap();
        
        let len = wrapper.read(|data| data.len()).unwrap();
        assert_eq!(len, 4);
    }
}
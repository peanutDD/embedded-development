# 第14章：异步编程

## 概述

异步编程是现代系统编程中的重要概念，特别适用于I/O密集型应用。Rust的异步编程模型基于Future trait和async/await语法，提供了零成本的异步抽象。与传统的线程模型相比，异步编程可以用更少的系统资源处理大量并发任务。

## 学习目标

通过本章学习，你将掌握：
- async/await语法和Future trait
- 异步运行时的选择和使用
- 异步I/O操作和网络编程
- 异步并发模式和同步原语
- 异步错误处理和调试技巧
- 性能优化和最佳实践

## 1. 异步基础概念

### 1.1 Future trait和async/await

```rust
use std::future::Future;
use std::pin::Pin;
use std::task::{Context, Poll};
use std::time::{Duration, Instant};

// 手动实现Future
struct TimerFuture {
    when: Instant,
}

impl TimerFuture {
    fn new(duration: Duration) -> Self {
        TimerFuture {
            when: Instant::now() + duration,
        }
    }
}

impl Future for TimerFuture {
    type Output = ();
    
    fn poll(self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if Instant::now() >= self.when {
            Poll::Ready(())
        } else {
            // 在实际实现中，这里应该注册一个唤醒器
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

// 使用async/await语法
async fn simple_async_function() -> i32 {
    println!("Starting async function");
    
    // 模拟异步操作
    TimerFuture::new(Duration::from_millis(100)).await;
    
    println!("Async function completed");
    42
}

async fn async_with_error() -> Result<String, &'static str> {
    TimerFuture::new(Duration::from_millis(50)).await;
    
    if true {
        Ok("Success".to_string())
    } else {
        Err("Something went wrong")
    }
}

#[tokio::main]
async fn main() {
    println!("Starting main");
    
    let result = simple_async_function().await;
    println!("Result: {}", result);
    
    match async_with_error().await {
        Ok(value) => println!("Success: {}", value),
        Err(e) => println!("Error: {}", e),
    }
}
```

### 1.2 异步运行时比较

```rust
// Tokio - 最流行的异步运行时
use tokio::time::{sleep, Duration};

#[tokio::main]
async fn tokio_example() {
    println!("Tokio runtime example");
    
    let task1 = tokio::spawn(async {
        sleep(Duration::from_millis(100)).await;
        println!("Task 1 completed");
        1
    });
    
    let task2 = tokio::spawn(async {
        sleep(Duration::from_millis(200)).await;
        println!("Task 2 completed");
        2
    });
    
    let (result1, result2) = tokio::join!(task1, task2);
    println!("Results: {:?}, {:?}", result1.unwrap(), result2.unwrap());
}

// async-std - 另一个流行的异步运行时
/*
use async_std::task;

#[async_std::main]
async fn async_std_example() {
    println!("async-std runtime example");
    
    let task1 = task::spawn(async {
        task::sleep(Duration::from_millis(100)).await;
        println!("Task 1 completed");
        1
    });
    
    let task2 = task::spawn(async {
        task::sleep(Duration::from_millis(200)).await;
        println!("Task 2 completed");
        2
    });
    
    let (result1, result2) = futures::join!(task1, task2);
    println!("Results: {}, {}", result1, result2);
}
*/

fn main() {
    // 选择运行时
    tokio_example();
}
```

### 1.3 异步函数的组合

```rust
use tokio::time::{sleep, Duration};

async fn fetch_data(id: u32) -> Result<String, &'static str> {
    println!("Fetching data for ID: {}", id);
    sleep(Duration::from_millis(100)).await;
    
    if id % 2 == 0 {
        Ok(format!("Data for ID {}", id))
    } else {
        Err("Failed to fetch data")
    }
}

async fn process_data(data: String) -> String {
    println!("Processing: {}", data);
    sleep(Duration::from_millis(50)).await;
    format!("Processed: {}", data)
}

async fn sequential_processing() {
    println!("=== Sequential Processing ===");
    
    for id in 1..=3 {
        match fetch_data(id).await {
            Ok(data) => {
                let processed = process_data(data).await;
                println!("Result: {}", processed);
            }
            Err(e) => println!("Error for ID {}: {}", id, e),
        }
    }
}

async fn concurrent_processing() {
    println!("=== Concurrent Processing ===");
    
    let futures: Vec<_> = (1..=3)
        .map(|id| async move {
            match fetch_data(id).await {
                Ok(data) => {
                    let processed = process_data(data).await;
                    Some(processed)
                }
                Err(e) => {
                    println!("Error for ID {}: {}", id, e);
                    None
                }
            }
        })
        .collect();
    
    let results = futures::future::join_all(futures).await;
    
    for (i, result) in results.into_iter().enumerate() {
        if let Some(processed) = result {
            println!("Result {}: {}", i + 1, processed);
        }
    }
}

#[tokio::main]
async fn main() {
    let start = std::time::Instant::now();
    sequential_processing().await;
    println!("Sequential took: {:?}\n", start.elapsed());
    
    let start = std::time::Instant::now();
    concurrent_processing().await;
    println!("Concurrent took: {:?}", start.elapsed());
}
```

## 2. 异步I/O操作

### 2.1 文件I/O

```rust
use tokio::fs::{File, OpenOptions};
use tokio::io::{AsyncReadExt, AsyncWriteExt, BufReader, AsyncBufReadExt};
use std::path::Path;

async fn write_file_example() -> Result<(), Box<dyn std::error::Error>> {
    let mut file = File::create("async_example.txt").await?;
    
    file.write_all(b"Hello, async world!\n").await?;
    file.write_all(b"This is line 2\n").await?;
    file.write_all(b"This is line 3\n").await?;
    
    file.flush().await?;
    println!("File written successfully");
    
    Ok(())
}

async fn read_file_example() -> Result<(), Box<dyn std::error::Error>> {
    // 读取整个文件
    let contents = tokio::fs::read_to_string("async_example.txt").await?;
    println!("File contents:\n{}", contents);
    
    // 逐行读取
    let file = File::open("async_example.txt").await?;
    let reader = BufReader::new(file);
    let mut lines = reader.lines();
    
    println!("Reading line by line:");
    while let Some(line) = lines.next_line().await? {
        println!("Line: {}", line);
    }
    
    Ok(())
}

async fn append_file_example() -> Result<(), Box<dyn std::error::Error>> {
    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open("async_example.txt")
        .await?;
    
    file.write_all(b"Appended line\n").await?;
    println!("Line appended successfully");
    
    Ok(())
}

async fn file_operations_concurrent() -> Result<(), Box<dyn std::error::Error>> {
    let write_task = tokio::spawn(async {
        write_file_example().await
    });
    
    let read_task = tokio::spawn(async {
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        read_file_example().await
    });
    
    let append_task = tokio::spawn(async {
        tokio::time::sleep(tokio::time::Duration::from_millis(200)).await;
        append_file_example().await
    });
    
    let (write_result, read_result, append_result) = 
        tokio::join!(write_task, read_task, append_task);
    
    write_result??;
    read_result??;
    append_result??;
    
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    file_operations_concurrent().await?;
    
    // 清理
    if Path::new("async_example.txt").exists() {
        tokio::fs::remove_file("async_example.txt").await?;
        println!("Cleanup completed");
    }
    
    Ok(())
}
```

### 2.2 网络编程

```rust
use tokio::net::{TcpListener, TcpStream};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use std::error::Error;

// 简单的TCP服务器
async fn tcp_server() -> Result<(), Box<dyn Error>> {
    let listener = TcpListener::bind("127.0.0.1:8080").await?;
    println!("Server listening on 127.0.0.1:8080");
    
    loop {
        let (socket, addr) = listener.accept().await?;
        println!("New client connected: {}", addr);
        
        // 为每个连接创建一个任务
        tokio::spawn(async move {
            if let Err(e) = handle_client(socket).await {
                println!("Error handling client {}: {}", addr, e);
            }
        });
    }
}

async fn handle_client(mut socket: TcpStream) -> Result<(), Box<dyn Error>> {
    let mut buffer = [0; 1024];
    
    loop {
        let n = socket.read(&mut buffer).await?;
        
        if n == 0 {
            println!("Client disconnected");
            break;
        }
        
        let message = String::from_utf8_lossy(&buffer[..n]);
        println!("Received: {}", message.trim());
        
        // 回显消息
        let response = format!("Echo: {}", message);
        socket.write_all(response.as_bytes()).await?;
    }
    
    Ok(())
}

// TCP客户端
async fn tcp_client() -> Result<(), Box<dyn Error>> {
    let mut stream = TcpStream::connect("127.0.0.1:8080").await?;
    println!("Connected to server");
    
    // 发送消息
    let messages = vec!["Hello", "World", "Async", "Rust"];
    
    for message in messages {
        stream.write_all(message.as_bytes()).await?;
        
        let mut buffer = [0; 1024];
        let n = stream.read(&mut buffer).await?;
        let response = String::from_utf8_lossy(&buffer[..n]);
        println!("Server response: {}", response);
        
        tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
    }
    
    Ok(())
}

// HTTP客户端示例
async fn http_client_example() -> Result<(), Box<dyn Error>> {
    let client = reqwest::Client::new();
    
    // 并发发送多个HTTP请求
    let urls = vec![
        "https://httpbin.org/delay/1",
        "https://httpbin.org/delay/2",
        "https://httpbin.org/delay/1",
    ];
    
    let futures: Vec<_> = urls
        .into_iter()
        .enumerate()
        .map(|(i, url)| {
            let client = client.clone();
            async move {
                let start = std::time::Instant::now();
                let response = client.get(url).send().await?;
                let status = response.status();
                let duration = start.elapsed();
                
                Ok::<_, reqwest::Error>((i, status, duration))
            }
        })
        .collect();
    
    let results = futures::future::join_all(futures).await;
    
    for result in results {
        match result {
            Ok((i, status, duration)) => {
                println!("Request {}: {} in {:?}", i, status, duration);
            }
            Err(e) => println!("Request failed: {}", e),
        }
    }
    
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // 启动服务器（在后台）
    let server_handle = tokio::spawn(async {
        if let Err(e) = tcp_server().await {
            println!("Server error: {}", e);
        }
    });
    
    // 等待服务器启动
    tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
    
    // 运行客户端
    let client_handle = tokio::spawn(async {
        if let Err(e) = tcp_client().await {
            println!("Client error: {}", e);
        }
    });
    
    // HTTP客户端示例
    let http_handle = tokio::spawn(async {
        if let Err(e) = http_client_example().await {
            println!("HTTP client error: {}", e);
        }
    });
    
    // 等待客户端完成
    let _ = tokio::join!(client_handle, http_handle);
    
    // 取消服务器任务
    server_handle.abort();
    
    Ok(())
}
```

### 2.3 WebSocket示例

```rust
use tokio_tungstenite::{connect_async, tungstenite::protocol::Message};
use futures_util::{SinkExt, StreamExt};

async fn websocket_client() -> Result<(), Box<dyn std::error::Error>> {
    let url = "wss://echo.websocket.org";
    
    println!("Connecting to {}", url);
    let (ws_stream, _) = connect_async(url).await?;
    println!("WebSocket handshake has been successfully completed");
    
    let (mut write, mut read) = ws_stream.split();
    
    // 发送消息的任务
    let send_task = tokio::spawn(async move {
        for i in 1..=5 {
            let message = format!("Hello WebSocket! Message {}", i);
            println!("Sending: {}", message);
            
            if let Err(e) = write.send(Message::Text(message)).await {
                println!("Error sending message: {}", e);
                break;
            }
            
            tokio::time::sleep(tokio::time::Duration::from_secs(1)).await;
        }
        
        // 发送关闭消息
        let _ = write.send(Message::Close(None)).await;
    });
    
    // 接收消息的任务
    let receive_task = tokio::spawn(async move {
        while let Some(message) = read.next().await {
            match message {
                Ok(Message::Text(text)) => {
                    println!("Received: {}", text);
                }
                Ok(Message::Close(_)) => {
                    println!("Connection closed");
                    break;
                }
                Err(e) => {
                    println!("Error receiving message: {}", e);
                    break;
                }
                _ => {}
            }
        }
    });
    
    // 等待两个任务完成
    let _ = tokio::join!(send_task, receive_task);
    
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    websocket_client().await?;
    Ok(())
}
```

## 3. 异步并发模式

### 3.1 任务管理和取消

```rust
use tokio::time::{sleep, Duration, timeout};
use tokio::sync::oneshot;
use std::time::Instant;

async fn long_running_task(id: u32) -> Result<String, &'static str> {
    println!("Task {} started", id);
    
    for i in 1..=10 {
        sleep(Duration::from_millis(200)).await;
        println!("Task {} progress: {}/10", id, i);
        
        // 模拟可能的错误
        if id == 2 && i == 5 {
            return Err("Task 2 failed at step 5");
        }
    }
    
    Ok(format!("Task {} completed successfully", id))
}

async fn task_with_timeout() {
    println!("=== Task with Timeout ===");
    
    let result = timeout(Duration::from_secs(1), long_running_task(1)).await;
    
    match result {
        Ok(Ok(message)) => println!("Success: {}", message),
        Ok(Err(e)) => println!("Task error: {}", e),
        Err(_) => println!("Task timed out"),
    }
}

async fn task_cancellation() {
    println!("=== Task Cancellation ===");
    
    let (tx, rx) = oneshot::channel();
    
    let task_handle = tokio::spawn(async move {
        tokio::select! {
            result = long_running_task(3) => {
                match result {
                    Ok(msg) => println!("Task completed: {}", msg),
                    Err(e) => println!("Task failed: {}", e),
                }
            }
            _ = rx => {
                println!("Task was cancelled");
            }
        }
    });
    
    // 让任务运行一段时间后取消
    sleep(Duration::from_millis(800)).await;
    let _ = tx.send(());
    
    let _ = task_handle.await;
}

async fn graceful_shutdown() {
    println!("=== Graceful Shutdown ===");
    
    let (shutdown_tx, mut shutdown_rx) = tokio::sync::broadcast::channel(1);
    let mut handles = vec![];
    
    // 启动多个工作任务
    for i in 1..=3 {
        let mut shutdown_rx = shutdown_tx.subscribe();
        let handle = tokio::spawn(async move {
            loop {
                tokio::select! {
                    _ = sleep(Duration::from_millis(300)) => {
                        println!("Worker {} is working...", i);
                    }
                    _ = shutdown_rx.recv() => {
                        println!("Worker {} received shutdown signal", i);
                        break;
                    }
                }
            }
            println!("Worker {} shutting down gracefully", i);
        });
        handles.push(handle);
    }
    
    // 让工作任务运行一段时间
    sleep(Duration::from_secs(2)).await;
    
    // 发送关闭信号
    println!("Sending shutdown signal...");
    let _ = shutdown_tx.send(());
    
    // 等待所有任务完成
    for handle in handles {
        let _ = handle.await;
    }
    
    println!("All workers have shut down");
}

#[tokio::main]
async fn main() {
    task_with_timeout().await;
    println!();
    
    task_cancellation().await;
    println!();
    
    graceful_shutdown().await;
}
```

### 3.2 异步同步原语

```rust
use tokio::sync::{Mutex, RwLock, Semaphore, Barrier};
use std::sync::Arc;
use tokio::time::{sleep, Duration};

async fn mutex_example() {
    println!("=== Mutex Example ===");
    
    let counter = Arc::new(Mutex::new(0));
    let mut handles = vec![];
    
    for i in 0..5 {
        let counter = Arc::clone(&counter);
        let handle = tokio::spawn(async move {
            for _ in 0..10 {
                let mut num = counter.lock().await;
                *num += 1;
                println!("Task {} incremented counter to {}", i, *num);
                drop(num); // 显式释放锁
                
                sleep(Duration::from_millis(10)).await;
            }
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.await.unwrap();
    }
    
    println!("Final counter value: {}", *counter.lock().await);
}

async fn rwlock_example() {
    println!("=== RwLock Example ===");
    
    let data = Arc::new(RwLock::new(vec![1, 2, 3, 4, 5]));
    let mut handles = vec![];
    
    // 多个读取任务
    for i in 0..3 {
        let data = Arc::clone(&data);
        let handle = tokio::spawn(async move {
            let reader = data.read().await;
            println!("Reader {} sees: {:?}", i, *reader);
            sleep(Duration::from_millis(100)).await;
        });
        handles.push(handle);
    }
    
    // 一个写入任务
    let data_clone = Arc::clone(&data);
    let write_handle = tokio::spawn(async move {
        sleep(Duration::from_millis(50)).await;
        let mut writer = data_clone.write().await;
        writer.push(6);
        println!("Writer added element, new data: {:?}", *writer);
    });
    handles.push(write_handle);
    
    for handle in handles {
        handle.await.unwrap();
    }
}

async fn semaphore_example() {
    println!("=== Semaphore Example ===");
    
    let semaphore = Arc::new(Semaphore::new(2)); // 最多2个并发任务
    let mut handles = vec![];
    
    for i in 0..5 {
        let semaphore = Arc::clone(&semaphore);
        let handle = tokio::spawn(async move {
            let _permit = semaphore.acquire().await.unwrap();
            println!("Task {} acquired permit", i);
            
            // 模拟工作
            sleep(Duration::from_millis(500)).await;
            
            println!("Task {} releasing permit", i);
            // permit在这里自动释放
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.await.unwrap();
    }
}

async fn barrier_example() {
    println!("=== Barrier Example ===");
    
    let barrier = Arc::new(Barrier::new(3));
    let mut handles = vec![];
    
    for i in 0..3 {
        let barrier = Arc::clone(&barrier);
        let handle = tokio::spawn(async move {
            println!("Task {} is preparing...", i);
            
            // 模拟不同的准备时间
            sleep(Duration::from_millis((i + 1) * 200)).await;
            
            println!("Task {} is ready, waiting for others...", i);
            barrier.wait().await;
            
            println!("Task {} is proceeding!", i);
        });
        handles.push(handle);
    }
    
    for handle in handles {
        handle.await.unwrap();
    }
}

#[tokio::main]
async fn main() {
    mutex_example().await;
    println!();
    
    rwlock_example().await;
    println!();
    
    semaphore_example().await;
    println!();
    
    barrier_example().await;
}
```

### 3.3 消息传递

```rust
use tokio::sync::{mpsc, oneshot, broadcast, watch};
use tokio::time::{sleep, Duration};

async fn mpsc_example() {
    println!("=== MPSC (Multi-Producer, Single-Consumer) ===");
    
    let (tx, mut rx) = mpsc::channel(32);
    
    // 多个生产者
    for i in 0..3 {
        let tx = tx.clone();
        tokio::spawn(async move {
            for j in 0..5 {
                let message = format!("Message {}-{}", i, j);
                if tx.send(message).await.is_err() {
                    println!("Producer {} failed to send", i);
                    break;
                }
                sleep(Duration::from_millis(100)).await;
            }
            println!("Producer {} finished", i);
        });
    }
    
    // 释放原始发送者
    drop(tx);
    
    // 单个消费者
    let consumer = tokio::spawn(async move {
        while let Some(message) = rx.recv().await {
            println!("Received: {}", message);
        }
        println!("Consumer finished");
    });
    
    consumer.await.unwrap();
}

async fn oneshot_example() {
    println!("=== Oneshot Channel ===");
    
    let (tx, rx) = oneshot::channel();
    
    let sender_task = tokio::spawn(async move {
        sleep(Duration::from_millis(500)).await;
        let result = "Computation result";
        if tx.send(result).is_err() {
            println!("Failed to send result");
        }
    });
    
    let receiver_task = tokio::spawn(async move {
        match rx.await {
            Ok(result) => println!("Received result: {}", result),
            Err(_) => println!("Sender was dropped"),
        }
    });
    
    let _ = tokio::join!(sender_task, receiver_task);
}

async fn broadcast_example() {
    println!("=== Broadcast Channel ===");
    
    let (tx, _rx) = broadcast::channel(16);
    
    // 多个订阅者
    let mut handles = vec![];
    for i in 0..3 {
        let mut rx = tx.subscribe();
        let handle = tokio::spawn(async move {
            while let Ok(message) = rx.recv().await {
                println!("Subscriber {} received: {}", i, message);
            }
            println!("Subscriber {} finished", i);
        });
        handles.push(handle);
    }
    
    // 发布者
    let publisher = tokio::spawn(async move {
        for i in 0..5 {
            let message = format!("Broadcast message {}", i);
            if tx.send(message).is_err() {
                println!("No active receivers");
                break;
            }
            sleep(Duration::from_millis(200)).await;
        }
        println!("Publisher finished");
    });
    
    publisher.await.unwrap();
    
    // 等待订阅者完成
    for handle in handles {
        handle.await.unwrap();
    }
}

async fn watch_example() {
    println!("=== Watch Channel ===");
    
    let (tx, mut rx) = watch::channel("initial");
    
    // 观察者任务
    let watcher1 = tokio::spawn(async move {
        while rx.changed().await.is_ok() {
            let value = rx.borrow().clone();
            println!("Watcher 1 sees: {}", value);
        }
        println!("Watcher 1 finished");
    });
    
    let mut rx2 = tx.subscribe();
    let watcher2 = tokio::spawn(async move {
        while rx2.changed().await.is_ok() {
            let value = rx2.borrow().clone();
            println!("Watcher 2 sees: {}", value);
        }
        println!("Watcher 2 finished");
    });
    
    // 更新值
    let updater = tokio::spawn(async move {
        for i in 1..=5 {
            sleep(Duration::from_millis(300)).await;
            let new_value = format!("value-{}", i);
            if tx.send(new_value).is_err() {
                println!("No active watchers");
                break;
            }
        }
        println!("Updater finished");
    });
    
    let _ = tokio::join!(watcher1, watcher2, updater);
}

#[tokio::main]
async fn main() {
    mpsc_example().await;
    println!();
    
    oneshot_example().await;
    println!();
    
    broadcast_example().await;
    println!();
    
    watch_example().await;
}
```

## 4. 异步错误处理

### 4.1 错误传播和处理

```rust
use tokio::time::{sleep, Duration};
use std::error::Error;
use std::fmt;

#[derive(Debug)]
enum AsyncError {
    Network(String),
    Timeout,
    Parse(String),
    Database(String),
}

impl fmt::Display for AsyncError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            AsyncError::Network(msg) => write!(f, "Network error: {}", msg),
            AsyncError::Timeout => write!(f, "Operation timed out"),
            AsyncError::Parse(msg) => write!(f, "Parse error: {}", msg),
            AsyncError::Database(msg) => write!(f, "Database error: {}", msg),
        }
    }
}

impl Error for AsyncError {}

async fn fetch_user_data(user_id: u32) -> Result<String, AsyncError> {
    println!("Fetching user data for ID: {}", user_id);
    
    // 模拟网络延迟
    sleep(Duration::from_millis(100)).await;
    
    // 模拟不同的错误情况
    match user_id {
        1 => Ok("User 1 data".to_string()),
        2 => Err(AsyncError::Network("Connection refused".to_string())),
        3 => Err(AsyncError::Timeout),
        4 => Err(AsyncError::Parse("Invalid JSON".to_string())),
        _ => Err(AsyncError::Database("User not found".to_string())),
    }
}

async fn process_user_data(data: String) -> Result<String, AsyncError> {
    println!("Processing user data: {}", data);
    
    sleep(Duration::from_millis(50)).await;
    
    if data.contains("1") {
        Ok(format!("Processed: {}", data))
    } else {
        Err(AsyncError::Parse("Invalid data format".to_string()))
    }
}

async fn handle_user_request(user_id: u32) -> Result<String, AsyncError> {
    let data = fetch_user_data(user_id).await?;
    let processed = process_user_data(data).await?;
    Ok(processed)
}

async fn error_handling_patterns() {
    println!("=== Error Handling Patterns ===");
    
    // 1. 基本错误处理
    match handle_user_request(1).await {
        Ok(result) => println!("Success: {}", result),
        Err(e) => println!("Error: {}", e),
    }
    
    // 2. 错误恢复
    let user_ids = vec![1, 2, 3, 4, 5];
    for user_id in user_ids {
        match handle_user_request(user_id).await {
            Ok(result) => println!("User {}: {}", user_id, result),
            Err(AsyncError::Network(_)) => {
                println!("User {}: Network error, retrying...", user_id);
                // 重试逻辑
                sleep(Duration::from_millis(100)).await;
                match handle_user_request(user_id).await {
                    Ok(result) => println!("User {} (retry): {}", user_id, result),
                    Err(e) => println!("User {} (retry failed): {}", user_id, e),
                }
            }
            Err(e) => println!("User {}: {}", user_id, e),
        }
    }
}

async fn concurrent_error_handling() {
    println!("=== Concurrent Error Handling ===");
    
    let user_ids = vec![1, 2, 3, 4, 5];
    let futures: Vec<_> = user_ids
        .into_iter()
        .map(|id| async move {
            (id, handle_user_request(id).await)
        })
        .collect();
    
    let results = futures::future::join_all(futures).await;
    
    let mut successes = 0;
    let mut errors = 0;
    
    for (id, result) in results {
        match result {
            Ok(data) => {
                println!("Success for user {}: {}", id, data);
                successes += 1;
            }
            Err(e) => {
                println!("Error for user {}: {}", id, e);
                errors += 1;
            }
        }
    }
    
    println!("Summary: {} successes, {} errors", successes, errors);
}

// 使用 ? 操作符的链式错误处理
async fn chain_operations() -> Result<String, AsyncError> {
    let data1 = fetch_user_data(1).await?;
    let processed1 = process_user_data(data1).await?;
    
    let data2 = fetch_user_data(1).await?;
    let processed2 = process_user_data(data2).await?;
    
    Ok(format!("{} + {}", processed1, processed2))
}

#[tokio::main]
async fn main() {
    error_handling_patterns().await;
    println!();
    
    concurrent_error_handling().await;
    println!();
    
    match chain_operations().await {
        Ok(result) => println!("Chain result: {}", result),
        Err(e) => println!("Chain error: {}", e),
    }
}
```

### 4.2 超时和重试机制

```rust
use tokio::time::{sleep, timeout, Duration, Instant};
use std::error::Error;
use std::fmt;

#[derive(Debug)]
struct RetryableError {
    message: String,
    retryable: bool,
}

impl fmt::Display for RetryableError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl Error for RetryableError {}

impl RetryableError {
    fn new(message: &str, retryable: bool) -> Self {
        Self {
            message: message.to_string(),
            retryable,
        }
    }
    
    fn is_retryable(&self) -> bool {
        self.retryable
    }
}

async fn unreliable_operation(attempt: u32) -> Result<String, RetryableError> {
    println!("Attempt {} starting...", attempt);
    
    // 模拟网络延迟
    sleep(Duration::from_millis(200)).await;
    
    // 模拟不同的失败情况
    match attempt {
        1 => Err(RetryableError::new("Temporary network error", true)),
        2 => Err(RetryableError::new("Server overloaded", true)),
        3 => Ok("Success on third attempt".to_string()),
        4 => Err(RetryableError::new("Invalid credentials", false)),
        _ => Ok(format!("Success on attempt {}", attempt)),
    }
}

async fn retry_with_backoff<F, Fut, T, E>(
    mut operation: F,
    max_attempts: u32,
    initial_delay: Duration,
) -> Result<T, E>
where
    F: FnMut(u32) -> Fut,
    Fut: std::future::Future<Output = Result<T, E>>,
    E: Error + 'static,
{
    let mut delay = initial_delay;
    
    for attempt in 1..=max_attempts {
        match operation(attempt).await {
            Ok(result) => return Ok(result),
            Err(e) => {
                if attempt == max_attempts {
                    return Err(e);
                }
                
                // 检查是否可重试（如果错误类型支持）
                if let Some(retryable_error) = (&e as &dyn Error).downcast_ref::<RetryableError>() {
                    if !retryable_error.is_retryable() {
                        return Err(e);
                    }
                }
                
                println!("Attempt {} failed: {}, retrying in {:?}", attempt, e, delay);
                sleep(delay).await;
                
                // 指数退避
                delay = std::cmp::min(delay * 2, Duration::from_secs(30));
            }
        }
    }
    
    unreachable!()
}

async fn timeout_with_retry() -> Result<String, Box<dyn Error>> {
    println!("=== Timeout with Retry ===");
    
    let operation = |attempt| async move {
        // 为每次尝试设置超时
        timeout(Duration::from_millis(500), unreliable_operation(attempt)).await
            .map_err(|_| RetryableError::new("Operation timed out", true))?
    };
    
    retry_with_backoff(operation, 5, Duration::from_millis(100)).await
}

async fn parallel_with_timeout() -> Result<Vec<String>, Box<dyn Error>> {
    println!("=== Parallel Operations with Timeout ===");
    
    let operations = (1..=5).map(|i| {
        timeout(Duration::from_secs(1), async move {
            unreliable_operation(i).await
        })
    });
    
    let results = futures::future::join_all(operations).await;
    
    let mut successes = Vec::new();
    for (i, result) in results.into_iter().enumerate() {
        match result {
            Ok(Ok(value)) => {
                println!("Operation {} succeeded: {}", i + 1, value);
                successes.push(value);
            }
            Ok(Err(e)) => println!("Operation {} failed: {}", i + 1, e),
            Err(_) => println!("Operation {} timed out", i + 1),
        }
    }
    
    Ok(successes)
}

// 断路器模式
struct CircuitBreaker {
    failure_count: u32,
    failure_threshold: u32,
    last_failure_time: Option<Instant>,
    recovery_timeout: Duration,
}

impl CircuitBreaker {
    fn new(failure_threshold: u32, recovery_timeout: Duration) -> Self {
        Self {
            failure_count: 0,
            failure_threshold,
            last_failure_time: None,
            recovery_timeout,
        }
    }
    
    fn is_open(&self) -> bool {
        if self.failure_count >= self.failure_threshold {
            if let Some(last_failure) = self.last_failure_time {
                return last_failure.elapsed() < self.recovery_timeout;
            }
        }
        false
    }
    
    fn record_success(&mut self) {
        self.failure_count = 0;
        self.last_failure_time = None;
    }
    
    fn record_failure(&mut self) {
        self.failure_count += 1;
        self.last_failure_time = Some(Instant::now());
    }
}

async fn circuit_breaker_example() -> Result<(), Box<dyn Error>> {
    println!("=== Circuit Breaker Pattern ===");
    
    let mut circuit_breaker = CircuitBreaker::new(3, Duration::from_secs(2));
    
    for i in 1..=10 {
        if circuit_breaker.is_open() {
            println!("Attempt {}: Circuit breaker is open, skipping", i);
            sleep(Duration::from_millis(300)).await;
            continue;
        }
        
        match unreliable_operation(i).await {
            Ok(result) => {
                println!("Attempt {}: Success - {}", i, result);
                circuit_breaker.record_success();
            }
            Err(e) => {
                println!("Attempt {}: Failed - {}", i, e);
                circuit_breaker.record_failure();
            }
        }
        
        sleep(Duration::from_millis(300)).await;
    }
    
    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    match timeout_with_retry().await {
        Ok(result) => println!("Final result: {}", result),
        Err(e) => println!("Final error: {}", e),
    }
    println!();
    
    match parallel_with_timeout().await {
        Ok(results) => println!("Successful results: {:?}", results),
        Err(e) => println!("Error: {}", e),
    }
    println!();
    
    circuit_breaker_example().await?;
    
    Ok(())
}
```

## 5. 性能优化和调试

### 5.1 性能分析和优化

```rust
use tokio::time::{sleep, Duration, Instant};
use std::sync::Arc;
use tokio::sync::Semaphore;

async fn cpu_intensive_task(id: u32, work_size: u32) -> u64 {
    let start = Instant::now();
    
    // CPU密集型工作
    let mut result = 0u64;
    for i in 0..work_size {
        result = result.wrapping_add(i as u64 * id as u64);
        
        // 定期让出控制权
        if i % 10000 == 0 {
            tokio::task::yield_now().await;
        }
    }
    
    println!("Task {} completed in {:?}", id, start.elapsed());
    result
}

async fn io_intensive_task(id: u32, delay_ms: u64) -> String {
    let start = Instant::now();
    
    // 模拟I/O操作
    sleep(Duration::from_millis(delay_ms)).await;
    
    let result = format!("IO Task {} result", id);
    println!("IO Task {} completed in {:?}", id, start.elapsed());
    result
}

async fn benchmark_concurrent_vs_sequential() {
    println!("=== Concurrent vs Sequential Benchmark ===");
    
    // 顺序执行
    let start = Instant::now();
    for i in 1..=5 {
        io_intensive_task(i, 100).await;
    }
    let sequential_time = start.elapsed();
    println!("Sequential execution took: {:?}", sequential_time);
    
    // 并发执行
    let start = Instant::now();
    let futures: Vec<_> = (1..=5)
        .map(|i| io_intensive_task(i, 100))
        .collect();
    futures::future::join_all(futures).await;
    let concurrent_time = start.elapsed();
    println!("Concurrent execution took: {:?}", concurrent_time);
    
    println!("Speedup: {:.2}x", sequential_time.as_millis() as f64 / concurrent_time.as_millis() as f64);
}

async fn cpu_bound_optimization() {
    println!("=== CPU-bound Task Optimization ===");
    
    let cpu_count = num_cpus::get();
    println!("Available CPU cores: {}", cpu_count);
    
    // 使用spawn_blocking处理CPU密集型任务
    let start = Instant::now();
    let mut handles = vec![];
    
    for i in 1..=cpu_count {
        let handle = tokio::task::spawn_blocking(move || {
            let mut result = 0u64;
            for j in 0..1_000_000 {
                result = result.wrapping_add(j * i as u64);
            }
            result
        });
        handles.push(handle);
    }
    
    let results = futures::future::join_all(handles).await;
    let cpu_time = start.elapsed();
    
    println!("CPU-bound tasks completed in {:?}", cpu_time);
    println!("Results: {:?}", results.into_iter().map(|r| r.unwrap()).collect::<Vec<_>>());
}

async fn memory_efficient_streaming() {
    println!("=== Memory-efficient Streaming ===");
    
    use tokio_stream::{StreamExt, wrappers::IntervalStream};
    use tokio::time::interval;
    
    let mut stream = IntervalStream::new(interval(Duration::from_millis(100)))
        .take(10)
        .enumerate();
    
    while let Some((i, _)) = stream.next().await {
        // 处理流中的每个项目
        let processed = format!("Processed item {}", i);
        println!("{}", processed);
        
        // 模拟处理时间
        sleep(Duration::from_millis(50)).await;
    }
}

async fn connection_pooling_simulation() {
    println!("=== Connection Pooling Simulation ===");
    
    let pool_size = 3;
    let semaphore = Arc::new(Semaphore::new(pool_size));
    
    let mut handles = vec![];
    
    for i in 1..=10 {
        let semaphore = Arc::clone(&semaphore);
        let handle = tokio::spawn(async move {
            let _permit = semaphore.acquire().await.unwrap();
            println!("Request {} acquired connection", i);
            
            // 模拟数据库操作
            sleep(Duration::from_millis(200)).await;
            
            println!("Request {} completed", i);
        });
        handles.push(handle);
    }
    
    let start = Instant::now();
    futures::future::join_all(handles).await;
    println!("All requests completed in {:?}", start.elapsed());
}

#[tokio::main]
async fn main() {
    benchmark_concurrent_vs_sequential().await;
    println!();
    
    cpu_bound_optimization().await;
    println!();
    
    memory_efficient_streaming().await;
    println!();
    
    connection_pooling_simulation().await;
}
```

### 5.2 调试和监控

```rust
use tokio::time::{sleep, Duration, Instant};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;

// 简单的性能监控器
#[derive(Debug, Clone)]
struct PerformanceMonitor {
    task_count: Arc<AtomicU64>,
    total_duration: Arc<AtomicU64>,
}

impl PerformanceMonitor {
    fn new() -> Self {
        Self {
            task_count: Arc::new(AtomicU64::new(0)),
            total_duration: Arc::new(AtomicU64::new(0)),
        }
    }
    
    async fn monitor_task<F, T>(&self, name: &str, future: F) -> T
    where
        F: std::future::Future<Output = T>,
    {
        let start = Instant::now();
        println!("[MONITOR] Starting task: {}", name);
        
        let result = future.await;
        
        let duration = start.elapsed();
        self.task_count.fetch_add(1, Ordering::Relaxed);
        self.total_duration.fetch_add(duration.as_millis() as u64, Ordering::Relaxed);
        
        println!("[MONITOR] Task '{}' completed in {:?}", name, duration);
        result
    }
    
    fn print_stats(&self) {
        let count = self.task_count.load(Ordering::Relaxed);
        let total_ms = self.total_duration.load(Ordering::Relaxed);
        
        if count > 0 {
            let avg_ms = total_ms / count;
            println!("[STATS] Tasks: {}, Total: {}ms, Average: {}ms", count, total_ms, avg_ms);
        }
    }
}

async fn traced_async_function(id: u32, delay_ms: u64) -> String {
    println!("[TRACE] Function {} starting with delay {}ms", id, delay_ms);
    
    sleep(Duration::from_millis(delay_ms)).await;
    
    let result = format!("Result from function {}", id);
    println!("[TRACE] Function {} returning: {}", id, result);
    
    result
}

async fn debugging_example() {
    println!("=== Debugging Example ===");
    
    let monitor = PerformanceMonitor::new();
    
    // 监控多个任务
    let mut handles = vec![];
    
    for i in 1..=5 {
        let monitor = monitor.clone();
        let handle = tokio::spawn(async move {
            monitor.monitor_task(
                &format!("task-{}", i),
                traced_async_function(i, i * 50)
            ).await
        });
        handles.push(handle);
    }
    
    let results = futures::future::join_all(handles).await;
    
    for (i, result) in results.into_iter().enumerate() {
        match result {
            Ok(value) => println!("Task {} result: {}", i + 1, value),
            Err(e) => println!("Task {} failed: {:?}", i + 1, e),
        }
    }
    
    monitor.print_stats();
}

// 异步任务的生命周期跟踪
struct TaskTracker {
    name: String,
    start_time: Instant,
}

impl TaskTracker {
    fn new(name: &str) -> Self {
        println!("[LIFECYCLE] Task '{}' created", name);
        Self {
            name: name.to_string(),
            start_time: Instant::now(),
        }
    }
}

impl Drop for TaskTracker {
    fn drop(&mut self) {
        let duration = self.start_time.elapsed();
        println!("[LIFECYCLE] Task '{}' dropped after {:?}", self.name, duration);
    }
}

async fn lifecycle_tracking_example() {
    println!("=== Lifecycle Tracking Example ===");
    
    let _tracker = TaskTracker::new("main-task");
    
    let subtasks: Vec<_> = (1..=3)
        .map(|i| {
            tokio::spawn(async move {
                let _tracker = TaskTracker::new(&format!("subtask-{}", i));
                sleep(Duration::from_millis(i * 100)).await;
                format!("Subtask {} completed", i)
            })
        })
        .collect();
    
    let results = futures::future::join_all(subtasks).await;
    
    for result in results {
        match result {
            Ok(msg) => println!("{}", msg),
            Err(e) => println!("Subtask error: {:?}", e),
        }
    }
    
    // _tracker在这里被drop
}

// 错误跟踪和日志记录
async fn error_tracking_example() {
    println!("=== Error Tracking Example ===");
    
    async fn risky_operation(id: u32) -> Result<String, &'static str> {
        println!("[DEBUG] Starting risky operation {}", id);
        
        sleep(Duration::from_millis(100)).await;
        
        if id % 2 == 0 {
            let result = format!("Success for operation {}", id);
            println!("[DEBUG] Operation {} succeeded", id);
            Ok(result)
        } else {
            println!("[ERROR] Operation {} failed", id);
            Err("Operation failed")
        }
    }
    
    let operations: Vec<_> = (1..=5)
        .map(|i| async move {
            match risky_operation(i).await {
                Ok(result) => {
                    println!("[INFO] Operation {} result: {}", i, result);
                    Some(result)
                }
                Err(e) => {
                    println!("[ERROR] Operation {} error: {}", i, e);
                    None
                }
            }
        })
        .collect();
    
    let results = futures::future::join_all(operations).await;
    let successful_count = results.iter().filter(|r| r.is_some()).count();
    
    println!("[SUMMARY] {} out of {} operations succeeded", successful_count, results.len());
}

#[tokio::main]
async fn main() {
    debugging_example().await;
    println!();
    
    lifecycle_tracking_example().await;
    println!();
    
    error_tracking_example().await;
}
```

## 6. 实践项目

### 6.1 异步Web爬虫

```rust
use reqwest::Client;
use tokio::time::{sleep, Duration};
use std::collections::HashSet;
use std::sync::Arc;
use tokio::sync::Mutex;
use url::Url;

struct WebCrawler {
    client: Client,
    visited: Arc<Mutex<HashSet<String>>>,
    max_depth: usize,
    delay: Duration,
}

impl WebCrawler {
    fn new(max_depth: usize, delay_ms: u64) -> Self {
        Self {
            client: Client::builder()
                .timeout(Duration::from_secs(10))
                .build()
                .unwrap(),
            visited: Arc::new(Mutex::new(HashSet::new())),
            max_depth,
            delay: Duration::from_millis(delay_ms),
        }
    }
    
    async fn crawl(&self, url: &str, depth: usize) -> Result<Vec<String>, Box<dyn std::error::Error>> {
        if depth > self.max_depth {
            return Ok(vec![]);
        }
        
        // 检查是否已访问
        {
            let mut visited = self.visited.lock().await;
            if visited.contains(url) {
                return Ok(vec![]);
            }
            visited.insert(url.to_string());
        }
        
        println!("Crawling (depth {}): {}", depth, url);
        
        // 获取页面内容
        let response = self.client.get(url).send().await?;
        let content = response.text().await?;
        
        // 提取链接（简化版本）
        let links = self.extract_links(&content, url)?;
        
        // 延迟以避免过于频繁的请求
        sleep(self.delay).await;
        
        // 递归爬取链接
        let mut all_links = vec![url.to_string()];
        
        let futures: Vec<_> = links
            .into_iter()
            .take(5) // 限制每页的链接数量
            .map(|link| self.crawl(&link, depth + 1))
            .collect();
        
        let results = futures::future::join_all(futures).await;
        
        for result in results {
            match result {
                Ok(mut sub_links) => all_links.append(&mut sub_links),
                Err(e) => println!("Error crawling sub-link: {}", e),
            }
        }
        
        Ok(all_links)
    }
    
    fn extract_links(&self, content: &str, base_url: &str) -> Result<Vec<String>, Box<dyn std::error::Error>> {
        let base = Url::parse(base_url)?;
        let mut links = Vec::new();
        
        // 简单的链接提取（实际应用中应使用HTML解析器）
        for line in content.lines() {
            if let Some(start) = line.find("href=\"") {
                let start = start + 6;
                if let Some(end) = line[start..].find('"') {
                    let href = &line[start..start + end];
                    
                    // 解析相对URL
                    if let Ok(url) = base.join(href) {
                        let url_str = url.to_string();
                        if url_str.starts_with("http") && !url_str.contains("#") {
                            links.push(url_str);
                        }
                    }
                }
            }
        }
        
        Ok(links)
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let crawler = WebCrawler::new(2, 1000); // 最大深度2，延迟1秒
    
    let start_url = "https://httpbin.org/links/5"; // 测试URL
    
    match crawler.crawl(start_url, 0).await {
        Ok(links) => {
            println!("\nCrawled {} unique URLs:", links.len());
            for (i, link) in links.iter().enumerate() {
                println!("  {}: {}", i + 1, link);
            }
        }
        Err(e) => println!("Crawling failed: {}", e),
    }
    
    Ok(())
}
```

### 6.2 异步聊天服务器

```rust
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{broadcast, Mutex};
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use std::collections::HashMap;
use std::sync::Arc;
use std::net::SocketAddr;

type Tx = broadcast::Sender<String>;
type Rx = broadcast::Receiver<String>;

struct ChatServer {
    clients: Arc<Mutex<HashMap<SocketAddr, String>>>,
    tx: Tx,
}

impl ChatServer {
    fn new() -> Self {
        let (tx, _rx) = broadcast::channel(100);
        
        Self {
            clients: Arc::new(Mutex::new(HashMap::new())),
            tx,
        }
    }
    
    async fn run(&self, addr: &str) -> Result<(), Box<dyn std::error::Error>> {
        let listener = TcpListener::bind(addr).await?;
        println!("Chat server listening on {}", addr);
        
        loop {
            let (socket, addr) = listener.accept().await?;
            println!("New client connected: {}", addr);
            
            let clients = Arc::clone(&self.clients);
            let tx = self.tx.clone();
            let rx = self.tx.subscribe();
            
            tokio::spawn(async move {
                if let Err(e) = Self::handle_client(socket, addr, clients, tx, rx).await {
                    println!("Error handling client {}: {}", addr, e);
                }
            });
        }
    }
    
    async fn handle_client(
        socket: TcpStream,
        addr: SocketAddr,
        clients: Arc<Mutex<HashMap<SocketAddr, String>>>,
        tx: Tx,
        mut rx: Rx,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let (reader, mut writer) = socket.into_split();
        let mut reader = BufReader::new(reader);
        let mut line = String::new();
        
        // 获取用户名
        writer.write_all(b"Enter your username: ").await?;
        reader.read_line(&mut line).await?;
        let username = line.trim().to_string();
        line.clear();
        
        // 注册客户端
        {
            let mut clients_guard = clients.lock().await;
            clients_guard.insert(addr, username.clone());
        }
        
        // 广播用户加入消息
        let join_msg = format!("{} joined the chat", username);
        let _ = tx.send(join_msg);
        
        writer.write_all(b"Welcome to the chat! Type messages to send.\n").await?;
        
        // 处理消息的任务
        let clients_clone = Arc::clone(&clients);
        let tx_clone = tx.clone();
        let username_clone = username.clone();
        
        let read_task = tokio::spawn(async move {
            loop {
                line.clear();
                match reader.read_line(&mut line).await {
                    Ok(0) => break, // 连接关闭
                    Ok(_) => {
                        let message = line.trim();
                        if !message.is_empty() {
                            let formatted_msg = format!("{}: {}", username_clone, message);
                            let _ = tx_clone.send(formatted_msg);
                        }
                    }
                    Err(_) => break,
                }
            }
        });
        
        // 广播消息的任务
        let write_task = tokio::spawn(async move {
            while let Ok(msg) = rx.recv().await {
                if let Err(_) = writer.write_all(format!("{}\n", msg).as_bytes()).await {
                    break;
                }
            }
        });
        
        // 等待任一任务完成
        tokio::select! {
            _ = read_task => {},
            _ = write_task => {},
        }
        
        // 清理客户端
        {
            let mut clients_guard = clients.lock().await;
            clients_guard.remove(&addr);
        }
        
        // 广播用户离开消息
        let leave_msg = format!("{} left the chat", username);
        let _ = tx.send(leave_msg);
        
        println!("Client {} ({}) disconnected", addr, username);
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let server = ChatServer::new();
    server.run("127.0.0.1:8080").await?;
    Ok(())
}
```

## 7. 最佳实践

### 7.1 异步编程最佳实践

```rust
use tokio::time::{sleep, timeout, Duration};
use std::future::Future;
use std::pin::Pin;

// 1. 避免阻塞异步运行时
async fn bad_blocking_example() {
    // ❌ 错误：在异步函数中使用阻塞操作
    // std::thread::sleep(Duration::from_secs(1));
    
    // ✅ 正确：使用异步版本
    sleep(Duration::from_secs(1)).await;
}

// 2. 正确处理CPU密集型任务
async fn cpu_intensive_best_practice() {
    // ❌ 错误：在异步上下文中执行长时间CPU任务
    // let result = expensive_computation();
    
    // ✅ 正确：使用spawn_blocking
    let result = tokio::task::spawn_blocking(|| {
        expensive_computation()
    }).await.unwrap();
    
    println!("Result: {}", result);
}

fn expensive_computation() -> u64 {
    (0..1_000_000).sum()
}

// 3. 合理使用超时
async fn timeout_best_practice() {
    let operation = async {
        sleep(Duration::from_millis(500)).await;
        "Operation completed"
    };
    
    match timeout(Duration::from_secs(1), operation).await {
        Ok(result) => println!("Success: {}", result),
        Err(_) => println!("Operation timed out"),
    }
}

// 4. 错误处理最佳实践
async fn error_handling_best_practice() -> Result<String, Box<dyn std::error::Error>> {
    // 使用 ? 操作符进行错误传播
    let data = fetch_data().await?;
    let processed = process_data(data).await?;
    Ok(processed)
}

async fn fetch_data() -> Result<String, &'static str> {
    sleep(Duration::from_millis(100)).await;
    Ok("data".to_string())
}

async fn process_data(data: String) -> Result<String, &'static str> {
    sleep(Duration::from_millis(50)).await;
    Ok(format!("processed: {}", data))
}

// 5. 资源管理
struct AsyncResource {
    name: String,
}

impl AsyncResource {
    async fn new(name: &str) -> Result<Self, &'static str> {
        println!("Creating resource: {}", name);
        sleep(Duration::from_millis(100)).await;
        Ok(Self { name: name.to_string() })
    }
    
    async fn cleanup(&self) {
        println!("Cleaning up resource: {}", self.name);
        sleep(Duration::from_millis(50)).await;
    }
}

async fn resource_management_example() {
    let resource = AsyncResource::new("test-resource").await.unwrap();
    
    // 使用资源
    println!("Using resource: {}", resource.name);
    
    // 确保清理
    resource.cleanup().await;
}

// 6. 避免过度并发
async fn controlled_concurrency() {
    use tokio::sync::Semaphore;
    use std::sync::Arc;
    
    let semaphore = Arc::new(Semaphore::new(3)); // 最多3个并发任务
    let mut handles = vec![];
    
    for i in 1..=10 {
        let semaphore = Arc::clone(&semaphore);
        let handle = tokio::spawn(async move {
            let _permit = semaphore.acquire().await.unwrap();
            println!("Task {} is running", i);
            sleep(Duration::from_millis(200)).await;
            println!("Task {} completed", i);
        });
        handles.push(handle);
    }
    
    futures::future::join_all(handles).await;
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    bad_blocking_example().await;
    cpu_intensive_best_practice().await;
    timeout_best_practice().await;
    
    match error_handling_best_practice().await {
        Ok(result) => println!("Success: {}", result),
        Err(e) => println!("Error: {}", e),
    }
    
    resource_management_example().await;
    controlled_concurrency().await;
    
    Ok(())
}
```

## 8. 练习题

### 练习1：异步文件处理器
```rust
// 实现一个异步文件处理器，能够：
// 1. 并发读取多个文件
// 2. 对文件内容进行处理（如统计行数、字符数）
// 3. 将结果写入输出文件

use tokio::fs::{File, OpenOptions};
use tokio::io::{AsyncReadExt, AsyncWriteExt, BufReader, AsyncBufReadExt};
use std::path::Path;

struct FileProcessor;

impl FileProcessor {
    async fn process_files(input_files: Vec<&str>, output_file: &str) -> Result<(), Box<dyn std::error::Error>> {
        let mut results = Vec::new();
        
        // 并发处理所有文件
        let futures: Vec<_> = input_files
            .into_iter()
            .map(|file_path| Self::analyze_file(file_path))
            .collect();
        
        let analysis_results = futures::future::join_all(futures).await;
        
        for (i, result) in analysis_results.into_iter().enumerate() {
            match result {
                Ok(stats) => results.push(stats),
                Err(e) => println!("Error processing file {}: {}", i, e),
            }
        }
        
        // 写入结果
        Self::write_results(&results, output_file).await?;
        
        Ok(())
    }
    
    async fn analyze_file(file_path: &str) -> Result<FileStats, Box<dyn std::error::Error>> {
        let file = File::open(file_path).await?;
        let reader = BufReader::new(file);
        let mut lines = reader.lines();
        
        let mut stats = FileStats {
            file_name: file_path.to_string(),
            line_count: 0,
            char_count: 0,
            word_count: 0,
        };
        
        while let Some(line) = lines.next_line().await? {
            stats.line_count += 1;
            stats.char_count += line.len();
            stats.word_count += line.split_whitespace().count();
        }
        
        Ok(stats)
    }
    
    async fn write_results(results: &[FileStats], output_file: &str) -> Result<(), Box<dyn std::error::Error>> {
        let mut file = File::create(output_file).await?;
        
        file.write_all(b"File Analysis Results\n").await?;
        file.write_all(b"====================\n\n").await?;
        
        for stats in results {
            let report = format!(
                "File: {}\nLines: {}\nCharacters: {}\nWords: {}\n\n",
                stats.file_name, stats.line_count, stats.char_count, stats.word_count
            );
            file.write_all(report.as_bytes()).await?;
        }
        
        Ok(())
    }
}

#[derive(Debug)]
struct FileStats {
    file_name: String,
    line_count: usize,
    char_count: usize,
    word_count: usize,
}
```

### 练习2：异步任务调度器
```rust
// 实现一个简单的异步任务调度器

use tokio::time::{sleep, Duration, Instant};
use std::collections::VecDeque;
use std::sync::Arc;
use tokio::sync::Mutex;

#[derive(Debug, Clone)]
struct Task {
    id: u32,
    name: String,
    duration: Duration,
    priority: u8, // 0-255, 越小优先级越高
}

struct TaskScheduler {
    tasks: Arc<Mutex<VecDeque<Task>>>,
    max_concurrent: usize,
}

impl TaskScheduler {
    fn new(max_concurrent: usize) -> Self {
        Self {
            tasks: Arc::new(Mutex::new(VecDeque::new())),
            max_concurrent,
        }
    }
    
    async fn add_task(&self, task: Task) {
        let mut tasks = self.tasks.lock().await;
        
        // 按优先级插入任务
        let mut insert_pos = tasks.len();
        for (i, existing_task) in tasks.iter().enumerate() {
            if task.priority < existing_task.priority {
                insert_pos = i;
                break;
            }
        }
        
        tasks.insert(insert_pos, task);
    }
    
    async fn run(&self) {
        use tokio::sync::Semaphore;
        
        let semaphore = Arc::new(Semaphore::new(self.max_concurrent));
        let mut handles = vec![];
        
        loop {
            let task = {
                let mut tasks = self.tasks.lock().await;
                tasks.pop_front()
            };
            
            match task {
                Some(task) => {
                    let semaphore = Arc::clone(&semaphore);
                    let handle = tokio::spawn(async move {
                        let _permit = semaphore.acquire().await.unwrap();
                        Self::execute_task(task).await;
                    });
                    handles.push(handle);
                }
                None => {
                    // 等待所有任务完成
                    futures::future::join_all(handles).await;
                    break;
                }
            }
        }
    }
    
    async fn execute_task(task: Task) {
        let start = Instant::now();
        println!("Starting task {} ({})", task.id, task.name);
        
        sleep(task.duration).await;
        
        println!("Completed task {} in {:?}", task.id, start.elapsed());
    }
}
```

### 练习3：异步缓存系统
```rust
// 实现一个支持过期时间的异步缓存系统

use tokio::time::{sleep, Duration, Instant};
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::RwLock;
use std::hash::Hash;

#[derive(Debug, Clone)]
struct CacheEntry<V> {
    value: V,
    expires_at: Instant,
}

struct AsyncCache<K, V> {
    data: Arc<RwLock<HashMap<K, CacheEntry<V>>>>,
    default_ttl: Duration,
}

impl<K, V> AsyncCache<K, V>
where
    K: Eq + Hash + Clone + Send + Sync + 'static,
    V: Clone + Send + Sync + 'static,
{
    fn new(default_ttl: Duration) -> Self {
        let cache = Self {
            data: Arc::new(RwLock::new(HashMap::new())),
            default_ttl,
        };
        
        // 启动清理任务
        let data_clone = Arc::clone(&cache.data);
        tokio::spawn(async move {
            loop {
                sleep(Duration::from_secs(60)).await; // 每分钟清理一次
                Self::cleanup_expired(data_clone.clone()).await;
            }
        });
        
        cache
    }
    
    async fn get(&self, key: &K) -> Option<V> {
        let data = self.data.read().await;
        
        if let Some(entry) = data.get(key) {
            if Instant::now() < entry.expires_at {
                return Some(entry.value.clone());
            }
        }
        
        None
    }
    
    async fn set(&self, key: K, value: V) {
        self.set_with_ttl(key, value, self.default_ttl).await;
    }
    
    async fn set_with_ttl(&self, key: K, value: V, ttl: Duration) {
        let mut data = self.data.write().await;
        
        let entry = CacheEntry {
            value,
            expires_at: Instant::now() + ttl,
        };
        
        data.insert(key, entry);
    }
    
    async fn remove(&self, key: &K) -> Option<V> {
        let mut data = self.data.write().await;
        data.remove(key).map(|entry| entry.value)
    }
    
    async fn cleanup_expired(data: Arc<RwLock<HashMap<K, CacheEntry<V>>>>) {
        let mut data = data.write().await;
        let now = Instant::now();
        
        data.retain(|_, entry| now < entry.expires_at);
    }
    
    async fn size(&self) -> usize {
        let data = self.data.read().await;
        data.len()
    }
}
```

## 总结

异步编程是Rust中的一个强大特性，它允许我们编写高效的并发代码。通过本章的学习，你应该掌握了：

### 核心概念
- Future trait和async/await语法
- 异步运行时的选择和使用
- 异步函数的组合和错误处理

### 实用技能
- 异步I/O操作（文件、网络）
- 异步并发模式和同步原语
- 消息传递和任务管理

### 最佳实践
- 避免阻塞异步运行时
- 正确处理CPU密集型任务
- 合理使用超时和错误处理
- 资源管理和清理

### 高级特性
- 性能优化和调试技巧
- 自定义Future实现
- 异步生态系统的使用

异步编程虽然复杂，但掌握了这些概念和模式后，你就能编写出高性能、可扩展的Rust应用程序。记住要根据具体的使用场景选择合适的异步模式和工具。
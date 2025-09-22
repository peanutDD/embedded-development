//! 实用工具模块

use crate::*;
use heapless::Vec;

/// 数学工具
pub mod math {
    use super::*;
    
    pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
        a + (b - a) * t
    }
    
    pub fn clamp(value: f32, min: f32, max: f32) -> f32 {
        if value < min {
            min
        } else if value > max {
            max
        } else {
            value
        }
    }
    
    pub fn map_range(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> f32 {
        (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    }
    
    pub fn moving_average(values: &[f32], window_size: usize) -> Vec<f32, 64> {
        let mut result = Vec::new();
        if values.len() < window_size {
            return result;
        }
        
        for i in window_size..=values.len() {
            let window = &values[i-window_size..i];
            let sum: f32 = window.iter().sum();
            let avg = sum / window_size as f32;
            let _ = result.push(avg);
        }
        
        result
    }
}

/// 缓冲区工具
pub mod buffer {
    use super::*;
    
    pub struct CircularBuffer<T, const N: usize> {
        data: [Option<T>; N],
        head: usize,
        tail: usize,
        count: usize,
    }
    
    impl<T: Copy, const N: usize> CircularBuffer<T, N> {
        pub fn new() -> Self {
            Self {
                data: [None; N],
                head: 0,
                tail: 0,
                count: 0,
            }
        }
        
        pub fn push(&mut self, item: T) -> Result<()> {
            if self.count == N {
                return Err(Error::ResourceExhausted);
            }
            
            self.data[self.tail] = Some(item);
            self.tail = (self.tail + 1) % N;
            self.count += 1;
            Ok(())
        }
        
        pub fn pop(&mut self) -> Option<T> {
            if self.count == 0 {
                return None;
            }
            
            let item = self.data[self.head].take();
            self.head = (self.head + 1) % N;
            self.count -= 1;
            item
        }
        
        pub fn len(&self) -> usize {
            self.count
        }
        
        pub fn is_empty(&self) -> bool {
            self.count == 0
        }
        
        pub fn is_full(&self) -> bool {
            self.count == N
        }
    }
}
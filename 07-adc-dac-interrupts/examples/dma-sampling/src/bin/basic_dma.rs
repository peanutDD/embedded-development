#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac::{self, interrupt, Interrupt},
    prelude::*,
    adc::{Adc, AdcConfig, SampleTime},
    gpio::Analog,
    timer::{Timer, Event},
    serial::{Serial, Config},
    dma::{Stream0, StreamsTuple, Transfer, PeripheralToMemory, DmaFlag},
};
use nb::block;
use heapless::{String, Vec};
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;

use dma_sampling::{DmaSamplingManager, DmaSamplingConfig, DmaPriority, SamplingStats};

const BUFFER_SIZE: usize = 1024;

// 全局变量用于DMA中断处理
static DMA_MANAGER: Mutex<RefCell<Option<DmaSamplingManager>>> = 
    Mutex::new(RefCell::new(None));
static TRANSFER_COMPLETE: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    
    // 配置串口
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let mut serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(115200.bps()),
        &clocks,
    ).unwrap();
    
    // 配置ADC引脚
    let adc_pin = gpioa.pa0.into_analog(); // ADC1_IN0
    
    // 配置ADC
    let adc_config = AdcConfig::default()
        .sample_time(SampleTime::Cycles_480);
    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);
    
    // 配置DMA
    let dma = StreamsTuple::new(dp.DMA2);
    
    // 创建DMA采样配置
    let mut config = DmaSamplingConfig::default();
    config.buffer_size = BUFFER_SIZE;
    config.sample_rate = 10000; // 10kHz采样率
    config.circular_mode = true;
    config.priority = DmaPriority::High;
    config.channels.clear();
    config.channels.push(0).ok(); // 通道0
    
    // 创建DMA采样管理器
    let mut dma_manager = DmaSamplingManager::new(config);
    
    // 配置定时器触发ADC
    let mut timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    timer.start(10000.Hz()).unwrap(); // 10kHz触发频率
    
    // 启用DMA中断
    cp.NVIC.enable(Interrupt::DMA2_STREAM0);
    
    writeln!(serial, "基本DMA采样示例启动").ok();
    writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
    writeln!(serial, "采样率: 10kHz").ok();
    writeln!(serial, "缓冲区大小: {} 样本", BUFFER_SIZE).ok();
    writeln!(serial, "DMA模式: 循环模式").ok();
    writeln!(serial, "").ok();
    
    // 将DMA管理器移动到全局变量
    free(|cs| {
        *DMA_MANAGER.borrow(cs).borrow_mut() = Some(dma_manager);
    });
    
    // 启动DMA采样
    free(|cs| {
        if let Some(ref mut manager) = DMA_MANAGER.borrow(cs).borrow_mut().as_mut() {
            manager.configure_dma_transfer().ok();
            manager.start_sampling().ok();
        }
    });
    
    let mut stats_counter = 0u32;
    let vref = 3.3f32;
    
    loop {
        // 等待定时器事件 (用于定期输出统计信息)
        if timer.wait().is_ok() {
            stats_counter += 1;
            
            // 每1000次循环输出一次统计信息
            if stats_counter % 1000 == 0 {
                output_sampling_statistics(&mut serial, stats_counter);
            }
        }
        
        // 检查DMA传输完成
        free(|cs| {
            if *TRANSFER_COMPLETE.borrow(cs).borrow() {
                *TRANSFER_COMPLETE.borrow(cs).borrow_mut() = false;
                
                if let Some(ref mut manager) = DMA_MANAGER.borrow(cs).borrow_mut().as_mut() {
                    // 处理传输完成事件
                    manager.handle_transfer_complete().ok();
                    
                    // 处理采样数据
                    process_dma_buffer(&mut serial, manager, vref);
                }
            }
        });
        
        // 检查缓冲区溢出
        free(|cs| {
            if let Some(ref mut manager) = DMA_MANAGER.borrow(cs).borrow_mut().as_mut() {
                if manager.check_buffer_overrun() {
                    writeln!(serial, "警告: DMA缓冲区溢出!").ok();
                }
            }
        });
    }
}

/// 输出采样统计信息
fn output_sampling_statistics(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    counter: u32
) {
    free(|cs| {
        if let Some(ref manager) = DMA_MANAGER.borrow(cs).borrow().as_ref() {
            let stats = manager.get_sampling_stats();
            
            writeln!(serial, "\n=== DMA采样统计 ({}s) ===", counter / 1000).ok();
            writeln!(serial, "已采集样本: {}", stats.samples_collected).ok();
            writeln!(serial, "传输完成次数: {}", stats.transfer_complete_count).ok();
            writeln!(serial, "缓冲区溢出: {}", stats.buffer_overruns).ok();
            writeln!(serial, "当前缓冲区: {:?}", stats.current_buffer).ok();
            writeln!(serial, "缓冲区利用率: {:.1}%", stats.buffer_utilization).ok();
            
            // 计算实际采样率
            let actual_rate = stats.samples_collected as f32 / (counter as f32 / 1000.0);
            writeln!(serial, "实际采样率: {:.1} Hz", actual_rate).ok();
            writeln!(serial, "").ok();
        }
    });
}

/// 处理DMA缓冲区数据
fn process_dma_buffer(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    manager: &DmaSamplingManager,
    vref: f32
) {
    let buffer = manager.get_current_buffer();
    
    if buffer.is_empty() {
        return;
    }
    
    // 计算缓冲区统计信息
    let stats = calculate_buffer_statistics(buffer, vref);
    
    // 每10次传输完成输出一次详细信息
    let transfer_count = manager.get_sampling_stats().transfer_complete_count;
    if transfer_count % 10 == 0 {
        writeln!(serial, "传输#{:04}: 样本数={}, 均值={:.3}V, 最大={:.3}V, 最小={:.3}V",
                transfer_count, buffer.len(), stats.mean_voltage, 
                stats.max_voltage, stats.min_voltage).ok();
    }
    
    // 检测信号异常
    detect_signal_anomalies(serial, &stats, transfer_count);
    
    // 执行实时信号处理
    perform_realtime_processing(buffer);
}

/// 缓冲区统计信息
#[derive(Debug)]
struct BufferStatistics {
    mean_voltage: f32,
    max_voltage: f32,
    min_voltage: f32,
    std_deviation: f32,
    sample_count: usize,
}

/// 计算缓冲区统计信息
fn calculate_buffer_statistics(buffer: &Vec<u16, dma_sampling::MAX_BUFFER_SIZE>, vref: f32) -> BufferStatistics {
    if buffer.is_empty() {
        return BufferStatistics {
            mean_voltage: 0.0,
            max_voltage: 0.0,
            min_voltage: 0.0,
            std_deviation: 0.0,
            sample_count: 0,
        };
    }
    
    // 转换为电压值
    let voltages: Vec<f32, dma_sampling::MAX_BUFFER_SIZE> = buffer.iter()
        .map(|&adc_val| adc_to_voltage(adc_val, vref, 12))
        .collect();
    
    // 计算统计信息
    let sum: f32 = voltages.iter().sum();
    let mean = sum / voltages.len() as f32;
    
    let max_voltage = voltages.iter().fold(0.0f32, |a, &b| a.max(b));
    let min_voltage = voltages.iter().fold(vref, |a, &b| a.min(b));
    
    // 计算标准差
    let variance: f32 = voltages.iter()
        .map(|&v| {
            let diff = v - mean;
            diff * diff
        })
        .sum::<f32>() / voltages.len() as f32;
    
    use micromath::F32Ext;
    let std_deviation = variance.sqrt();
    
    BufferStatistics {
        mean_voltage: mean,
        max_voltage,
        min_voltage,
        std_deviation,
        sample_count: buffer.len(),
    }
}

/// 检测信号异常
fn detect_signal_anomalies(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    stats: &BufferStatistics,
    transfer_count: u32
) {
    // 检查电压范围
    if stats.max_voltage > 3.2 {
        writeln!(serial, "警告: 电压过高 {:.3}V (传输#{})", 
                stats.max_voltage, transfer_count).ok();
    }
    
    if stats.min_voltage < 0.1 {
        writeln!(serial, "警告: 电压过低 {:.3}V (传输#{})", 
                stats.min_voltage, transfer_count).ok();
    }
    
    // 检查信号稳定性
    if stats.std_deviation > 0.5 {
        writeln!(serial, "警告: 信号不稳定，标准差={:.3}V (传输#{})", 
                stats.std_deviation, transfer_count).ok();
    }
    
    // 检查直流偏移
    if stats.mean_voltage < 0.2 || stats.mean_voltage > 3.0 {
        writeln!(serial, "警告: 直流偏移异常 {:.3}V (传输#{})", 
                stats.mean_voltage, transfer_count).ok();
    }
}

/// 执行实时信号处理
fn perform_realtime_processing(buffer: &Vec<u16, dma_sampling::MAX_BUFFER_SIZE>) {
    // 这里可以实现各种实时信号处理算法
    // 例如：数字滤波、FFT分析、峰值检测等
    
    // 简单的移动平均滤波示例
    if buffer.len() >= 8 {
        let mut filtered_samples = Vec::<u16, 128>::new();
        
        for i in 4..buffer.len()-4 {
            let sum: u32 = (i-4..=i+4)
                .map(|idx| buffer[idx] as u32)
                .sum();
            let avg = (sum / 9) as u16;
            filtered_samples.push(avg).ok();
        }
        
        // 处理滤波后的数据...
    }
}

/// DMA中断处理函数
#[interrupt]
fn DMA2_STREAM0() {
    free(|cs| {
        // 检查传输完成标志
        // if DMA2.lisr.read().tcif0().bit_is_set() {
            // 设置传输完成标志
            *TRANSFER_COMPLETE.borrow(cs).borrow_mut() = true;
            
            // 清除中断标志
            // DMA2.lifcr.write(|w| w.ctcif0().set_bit());
        // }
        
        // 检查传输错误
        // if DMA2.lisr.read().teif0().bit_is_set() {
            // 处理传输错误
            // DMA2.lifcr.write(|w| w.cteif0().set_bit());
        // }
        
        // 检查FIFO错误
        // if DMA2.lisr.read().feif0().bit_is_set() {
            // 处理FIFO错误
            // DMA2.lifcr.write(|w| w.cfeif0().set_bit());
        // }
    });
}

/// ADC值转换为电压
fn adc_to_voltage(adc_value: u16, vref: f32, resolution: u8) -> f32 {
    let max_value = (1 << resolution) - 1;
    (adc_value as f32 / max_value as f32) * vref
}

/// DMA配置辅助函数
fn configure_dma_stream() {
    // 配置DMA2 Stream0用于ADC1
    // 1. 禁用DMA流
    // 2. 配置源地址 (ADC1->DR)
    // 3. 配置目标地址 (缓冲区)
    // 4. 配置传输大小
    // 5. 配置传输模式 (循环模式)
    // 6. 配置优先级
    // 7. 启用中断
    // 8. 启用DMA流
}

/// ADC配置辅助函数
fn configure_adc_for_dma() {
    // 配置ADC1用于DMA传输
    // 1. 启用DMA模式
    // 2. 配置连续转换模式
    // 3. 配置触发源
    // 4. 启用ADC
}

/// 性能监控结构
struct PerformanceMonitor {
    last_transfer_time: u32,
    transfer_intervals: Vec<u32, 100>,
    max_interval: u32,
    min_interval: u32,
}

impl PerformanceMonitor {
    fn new() -> Self {
        Self {
            last_transfer_time: 0,
            transfer_intervals: Vec::new(),
            max_interval: 0,
            min_interval: u32::MAX,
        }
    }
    
    fn record_transfer(&mut self, current_time: u32) {
        if self.last_transfer_time != 0 {
            let interval = current_time - self.last_transfer_time;
            
            if self.transfer_intervals.len() >= 100 {
                self.transfer_intervals.remove(0);
            }
            self.transfer_intervals.push(interval).ok();
            
            if interval > self.max_interval {
                self.max_interval = interval;
            }
            if interval < self.min_interval {
                self.min_interval = interval;
            }
        }
        
        self.last_transfer_time = current_time;
    }
    
    fn get_average_interval(&self) -> f32 {
        if self.transfer_intervals.is_empty() {
            return 0.0;
        }
        
        let sum: u32 = self.transfer_intervals.iter().sum();
        sum as f32 / self.transfer_intervals.len() as f32
    }
}
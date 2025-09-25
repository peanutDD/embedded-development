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
};
use nb::block;
use heapless::{String, Vec};
use core::fmt::Write;
use cortex_m::interrupt::{free, Mutex};
use core::cell::RefCell;

use dma_sampling::{HighSpeedSampler, DecimationFilter, DmaTransferMonitor};

const HIGH_SPEED_BUFFER_SIZE: usize = 4096;
const DECIMATION_FACTOR: usize = 8;
const OUTPUT_BUFFER_SIZE: usize = HIGH_SPEED_BUFFER_SIZE / DECIMATION_FACTOR;

// 高速采样缓冲区
static HIGH_SPEED_BUFFER: Mutex<RefCell<[u16; HIGH_SPEED_BUFFER_SIZE]>> = 
    Mutex::new(RefCell::new([0; HIGH_SPEED_BUFFER_SIZE]));
static BUFFER_READY: Mutex<RefCell<bool>> = Mutex::new(RefCell::new(false));
static SAMPLE_INDEX: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));

#[entry]
fn main() -> ! {
    // 获取设备外设
    let dp = pac::Peripherals::take().unwrap();
    let mut cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置时钟 - 使用最高频率
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
        .sysclk(168.mhz())  // 最大系统时钟
        .pclk1(42.mhz())    // APB1时钟
        .pclk2(84.mhz())    // APB2时钟
        .freeze();
    
    // 配置GPIO
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    
    // 配置串口
    let tx_pin = gpioa.pa2.into_alternate();
    let rx_pin = gpioa.pa3.into_alternate();
    let mut serial = Serial::new(
        dp.USART2,
        (tx_pin, rx_pin),
        Config::default().baudrate(921600.bps()), // 高波特率
        &clocks,
    ).unwrap();
    
    // 配置ADC引脚
    let adc_pin = gpioa.pa0.into_analog(); // ADC1_IN0
    
    // 配置高速ADC
    let adc_config = AdcConfig::default()
        .sample_time(SampleTime::Cycles_3); // 最快采样时间
    let mut adc = Adc::adc1(dp.ADC1, true, adc_config);
    
    // 配置高速采样定时器
    let mut sample_timer = Timer::new(dp.TIM2, &clocks).counter_hz();
    sample_timer.start(1000000.Hz()).unwrap(); // 1MHz采样率
    
    // 配置处理定时器
    let mut process_timer = Timer::new(dp.TIM3, &clocks).counter_hz();
    process_timer.start(50.Hz()).unwrap(); // 50Hz处理频率
    
    // 配置状态输出定时器
    let mut status_timer = Timer::new(dp.TIM4, &clocks).counter_hz();
    status_timer.start(5.Hz()).unwrap(); // 5Hz状态输出
    
    // 启用定时器中断
    cp.NVIC.enable(Interrupt::TIM2);
    
    writeln!(serial, "高速DMA采样示例启动").ok();
    writeln!(serial, "系统时钟: {} MHz", clocks.sysclk().raw() / 1_000_000).ok();
    writeln!(serial, "ADC通道: PA0 (ADC1_IN0)").ok();
    writeln!(serial, "采样率: 1MHz").ok();
    writeln!(serial, "缓冲区大小: {} 样本", HIGH_SPEED_BUFFER_SIZE).ok();
    writeln!(serial, "抽取因子: {}", DECIMATION_FACTOR).ok();
    writeln!(serial, "输出采样率: {} kHz", 1000 / DECIMATION_FACTOR).ok();
    writeln!(serial, "").ok();
    
    // 初始化高速采样器
    let mut high_speed_sampler = HighSpeedSampler::new(1000000, DECIMATION_FACTOR);
    
    // 初始化抽取滤波器
    let mut decimation_filter = DecimationFilter::new(DECIMATION_FACTOR);
    
    // 初始化DMA传输监控器
    let mut dma_monitor = DmaTransferMonitor::new();
    
    let mut sample_count = 0u32;
    let mut process_count = 0u32;
    let mut total_processed_samples = 0u32;
    let vref = 3.3f32;
    
    // 性能计数器
    let mut performance_counter = HighSpeedPerformanceCounter::new();
    
    loop {
        // 高速采样 (模拟DMA中断)
        if sample_timer.wait().is_ok() {
            sample_count += 1;
            
            // 模拟高速ADC读取
            let adc_value = simulate_high_speed_adc(sample_count);
            
            // 写入高速缓冲区
            let buffer_full = free(|cs| {
                let mut buffer = HIGH_SPEED_BUFFER.borrow(cs).borrow_mut();
                let mut index = SAMPLE_INDEX.borrow(cs).borrow_mut();
                
                buffer[*index] = adc_value;
                *index += 1;
                
                if *index >= HIGH_SPEED_BUFFER_SIZE {
                    *index = 0;
                    *BUFFER_READY.borrow(cs).borrow_mut() = true;
                    true
                } else {
                    false
                }
            });
            
            if buffer_full {
                performance_counter.record_buffer_fill();
            }
        }
        
        // 处理就绪的缓冲区
        if process_timer.wait().is_ok() {
            let buffer_data = free(|cs| {
                if *BUFFER_READY.borrow(cs).borrow() {
                    *BUFFER_READY.borrow(cs).borrow_mut() = false;
                    
                    let buffer = HIGH_SPEED_BUFFER.borrow(cs).borrow();
                    let mut data = Vec::<u16, HIGH_SPEED_BUFFER_SIZE>::new();
                    for &sample in buffer.iter() {
                        data.push(sample).ok();
                    }
                    Some(data)
                } else {
                    None
                }
            });
            
            if let Some(data) = buffer_data {
                process_count += 1;
                
                // 高速数据处理
                process_high_speed_data(&mut serial, &data, process_count, vref,
                                      &mut decimation_filter, &mut dma_monitor);
                
                total_processed_samples += data.len() as u32;
                performance_counter.record_processing_complete(data.len());
            }
        }
        
        // 定期输出系统状态
        if status_timer.wait().is_ok() {
            output_high_speed_status(&mut serial, sample_count, total_processed_samples,
                                   &performance_counter, &dma_monitor);
        }
    }
}

/// 处理高速数据
fn process_high_speed_data(
    serial: &mut Serial<pac::USART2, (stm32f4xx_hal::gpio::Pin<'A', 2>, stm32f4xx_hal::gpio::Pin<'A', 3>)>,
    data: &Vec<u16, HIGH_SPEED_BUFFER_SIZE>,
    process_count: u32,
    vref: f32,
    decimation_filter: &mut DecimationFilter,
    dma_monitor: &mut DmaTransferMonitor
) {
    // 更新DMA监控
    dma_monitor.update_transfer_stats(data.len(), true);
    
    // 应用抽取滤波
    let decimated_data = decimation_filter.process_buffer(data);
    
    // 计算高速统计信息
    let stats = calculate_high_speed_statistics(&decimated_data, vref);
    
    // 执行实时频谱分析
    let spectrum_analysis = perform_real_time_spectrum_analysis(&decimated_data);
    
    // 检测高频特征
    let high_freq_features = detect_high_frequency_features(&decimated_data);
    
    // 每20次处理输出详细信息
    if process_count % 20 == 0 {
        writeln!(serial, "\n=== 高速处理 #{:04} ===", process_count).ok();
        writeln!(serial, "原始样本: {}, 抽取后: {}", data.len(), decimated_data.len()).ok();
        writeln!(serial, "均值: {:.3}V, 峰峰值: {:.3}V", stats.mean, stats.peak_to_peak).ok();
        writeln!(serial, "带宽: {:.1}kHz, 主频: {:.1}kHz", 
                spectrum_analysis.bandwidth / 1000.0, 
                spectrum_analysis.dominant_freq / 1000.0).ok();
        writeln!(serial, "高频成分: {} 个", high_freq_features.len()).ok();
        writeln!(serial, "").ok();
    }
    
    // 高速信号质量评估
    let quality_metrics = assess_high_speed_signal_quality(&stats, &spectrum_analysis);
    
    // 异常检测
    detect_high_speed_anomalies(serial, &quality_metrics, process_count);
    
    // 实时压缩和存储 (模拟)
    perform_real_time_compression(&decimated_data, process_count);
}

/// 高速统计信息
#[derive(Debug)]
struct HighSpeedStatistics {
    mean: f32,
    max: f32,
    min: f32,
    peak_to_peak: f32,
    rms: f32,
    std_dev: f32,
    dynamic_range: f32,
    crest_factor: f32,
}

/// 计算高速统计信息
fn calculate_high_speed_statistics(data: &Vec<u16, OUTPUT_BUFFER_SIZE>, vref: f32) -> HighSpeedStatistics {
    if data.is_empty() {
        return HighSpeedStatistics {
            mean: 0.0, max: 0.0, min: 0.0, peak_to_peak: 0.0,
            rms: 0.0, std_dev: 0.0, dynamic_range: 0.0, crest_factor: 0.0,
        };
    }
    
    // 转换为电压
    let voltages: Vec<f32, OUTPUT_BUFFER_SIZE> = data.iter()
        .map(|&adc_val| adc_to_voltage(adc_val, vref, 12))
        .collect();
    
    // 基本统计
    let sum: f32 = voltages.iter().sum();
    let mean = sum / voltages.len() as f32;
    
    let max = voltages.iter().fold(0.0f32, |a, &b| a.max(b));
    let min = voltages.iter().fold(vref, |a, &b| a.min(b));
    let peak_to_peak = max - min;
    
    // RMS计算
    let sum_squares: f32 = voltages.iter().map(|&v| v * v).sum();
    use micromath::F32Ext;
    let rms = (sum_squares / voltages.len() as f32).sqrt();
    
    // 标准差
    let variance: f32 = voltages.iter()
        .map(|&v| {
            let diff = v - mean;
            diff * diff
        })
        .sum::<f32>() / voltages.len() as f32;
    let std_dev = variance.sqrt();
    
    // 动态范围 (dB)
    let{"file_path": "/Users/tyone/github/embedded-development/07-adc-dac-interrupts/examples/dma-sampling/src/bin/high_speed_sampling.rs", "explanation": "创建高速采样DMA示例程序", "content":
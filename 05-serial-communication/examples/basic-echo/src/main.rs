#![no_std]
#![no_main]

// 导入必要的库
use panic_halt as _;
use cortex_m_rt::entry;
use stm32f4xx_hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
    timer::{Timer, Event},
};
use rtt_target::{rprintln, rtt_init_print};
use heapless::{String, Vec};
use nb::block;

/// 串口缓冲区大小
const BUFFER_SIZE: usize = 256;

/// 欢迎消息
const WELCOME_MSG: &str = "\r\n🚀 STM32F4 串口回显服务器启动\r\n";
const PROMPT_MSG: &str = "📝 请输入命令 (输入 'help' 查看帮助): ";
const HELP_MSG: &str = "\r\n📋 可用命令:\r\n  help    - 显示帮助信息\r\n  info    - 显示系统信息\r\n  echo    - 回显测试\r\n  clear   - 清屏\r\n  reset   - 重启系统\r\n";

/// 主程序入口点
#[entry]
fn main() -> ! {
    // 初始化RTT调试输出
    rtt_init_print!();
    rprintln!("🚀 基础串口回显示例启动");
    rprintln!("硬件平台: STM32F407VG Discovery");
    rprintln!("串口配置: USART1, 115200-8-N-1");
    
    // 获取外设访问权限
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    
    // 配置系统时钟
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz())  // 使用外部8MHz晶振
        .sysclk(168.MHz()) // 系统时钟168MHz
        .freeze();
    
    rprintln!("⚡ 系统时钟配置完成: {}MHz", clocks.sysclk().raw() / 1_000_000);
    
    // 配置GPIO用于USART1
    let gpioa = dp.GPIOA.split();
    let tx_pin = gpioa.pa9.into_alternate(); // USART1_TX
    let rx_pin = gpioa.pa10.into_alternate(); // USART1_RX
    
    // 配置串口参数
    let serial_config = Config::default()
        .baudrate(115200.bps())
        .wordlength_8()
        .parity_none()
        .stopbits(stm32f4xx_hal::serial::StopBits::STOP1);
    
    // 初始化串口
    let mut serial = Serial::new(
        dp.USART1,
        (tx_pin, rx_pin),
        serial_config,
        &clocks,
    ).unwrap();
    
    rprintln!("📡 USART1配置完成");
    rprintln!("   波特率: 115200 bps");
    rprintln!("   数据位: 8位");
    rprintln!("   校验位: 无");
    rprintln!("   停止位: 1位");
    rprintln!("   引脚: PA9(TX), PA10(RX)");
    
    // 分离发送和接收端
    let (mut tx, mut rx) = serial.split();
    
    // 发送欢迎消息
    send_string(&mut tx, WELCOME_MSG);
    send_string(&mut tx, HELP_MSG);
    send_string(&mut tx, PROMPT_MSG);
    
    rprintln!("💡 串口回显服务器就绪，等待用户输入");
    
    // 输入缓冲区
    let mut input_buffer: Vec<u8, BUFFER_SIZE> = Vec::new();
    let mut command_count = 0u32;
    
    // 主循环
    loop {
        // 非阻塞读取字符
        match rx.read() {
            Ok(byte) => {
                match byte {
                    // 回车键 (CR) 或换行键 (LF)
                    b'\r' | b'\n' => {
                        if !input_buffer.is_empty() {
                            // 处理完整的命令
                            command_count += 1;
                            process_command(&mut tx, &input_buffer, command_count);
                            input_buffer.clear();
                            
                            // 显示新的提示符
                            send_string(&mut tx, "\r\n");
                            send_string(&mut tx, PROMPT_MSG);
                        }
                    },
                    
                    // 退格键 (Backspace)
                    b'\x08' | b'\x7f' => {
                        if !input_buffer.is_empty() {
                            input_buffer.pop();
                            // 发送退格序列：退格 + 空格 + 退格
                            send_string(&mut tx, "\x08 \x08");
                        }
                    },
                    
                    // 可打印字符
                    32..=126 => {
                        if input_buffer.len() < BUFFER_SIZE - 1 {
                            // 回显字符
                            block!(tx.write(byte)).ok();
                            
                            // 添加到缓冲区
                            input_buffer.push(byte).ok();
                        } else {
                            // 缓冲区满，发送警告
                            send_string(&mut tx, "\r\n⚠️  输入缓冲区已满！\r\n");
                            input_buffer.clear();
                            send_string(&mut tx, PROMPT_MSG);
                        }
                    },
                    
                    // 其他控制字符
                    _ => {
                        // 忽略其他控制字符
                        rprintln!("收到控制字符: 0x{:02X}", byte);
                    }
                }
            },
            
            Err(nb::Error::WouldBlock) => {
                // 没有数据可读，继续循环
                continue;
            },
            
            Err(nb::Error::Other(e)) => {
                // 串口错误
                rprintln!("串口读取错误: {:?}", e);
                send_string(&mut tx, "\r\n❌ 串口读取错误\r\n");
                send_string(&mut tx, PROMPT_MSG);
            }
        }
    }
}

/// 发送字符串到串口
fn send_string<T>(tx: &mut T, s: &str) 
where
    T: embedded_hal::serial::Write<u8>,
{
    for byte in s.bytes() {
        block!(tx.write(byte)).ok();
    }
}

/// 处理用户命令
fn process_command<T>(tx: &mut T, buffer: &[u8], cmd_count: u32)
where
    T: embedded_hal::serial::Write<u8>,
{
    // 将缓冲区转换为字符串
    let command = core::str::from_utf8(buffer).unwrap_or("<无效UTF-8>");
    let command = command.trim().to_lowercase();
    
    rprintln!("处理命令 #{}: '{}'", cmd_count, command);
    
    match command.as_str() {
        "help" => {
            send_string(tx, HELP_MSG);
        },
        
        "info" => {
            send_string(tx, "\r\n📊 系统信息:\r\n");
            send_string(tx, "   MCU: STM32F407VGT6\r\n");
            send_string(tx, "   架构: ARM Cortex-M4F\r\n");
            send_string(tx, "   主频: 168MHz\r\n");
            send_string(tx, "   Flash: 1MB\r\n");
            send_string(tx, "   SRAM: 128KB\r\n");
            
            // 格式化命令计数
            let mut count_str: String<32> = String::new();
            use core::fmt::Write;
            write!(count_str, "   已处理命令: {}\r\n", cmd_count).ok();
            send_string(tx, &count_str);
        },
        
        "echo" => {
            send_string(tx, "\r\n🔊 回显测试:\r\n");
            send_string(tx, "   输入: ");
            
            // 回显原始输入
            for &byte in buffer {
                if byte >= 32 && byte <= 126 {
                    block!(tx.write(byte)).ok();
                } else {
                    // 显示不可打印字符的十六进制值
                    let mut hex_str: String<8> = String::new();
                    use core::fmt::Write;
                    write!(hex_str, "[{:02X}]", byte).ok();
                    send_string(tx, &hex_str);
                }
            }
            send_string(tx, "\r\n   长度: ");
            
            let mut len_str: String<16> = String::new();
            use core::fmt::Write;
            write!(len_str, "{} 字节\r\n", buffer.len()).ok();
            send_string(tx, &len_str);
        },
        
        "clear" => {
            // 发送清屏序列 (ANSI escape code)
            send_string(tx, "\x1b[2J\x1b[H");
            send_string(tx, WELCOME_MSG);
        },
        
        "reset" => {
            send_string(tx, "\r\n🔄 系统重启中...\r\n");
            // 等待发送完成
            for _ in 0..100000 {
                cortex_m::asm::nop();
            }
            // 软件复位
            cortex_m::peripheral::SCB::sys_reset();
        },
        
        "" => {
            // 空命令，不做任何处理
        },
        
        _ => {
            send_string(tx, "\r\n❓ 未知命令: '");
            send_string(tx, command);
            send_string(tx, "'\r\n💡 输入 'help' 查看可用命令\r\n");
        }
    }
}

/// 错误处理函数
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    rprintln!("💥 程序崩溃: {:?}", info);
    loop {
        cortex_m::asm::wfi(); // 等待中断，降低功耗
    }
}
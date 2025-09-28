#![no_std]
#![no_main]

use panic_halt as _;

use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m;
use cortex_m_rt::entry;
use heapless::{
  spsc::{Consumer, Producer, Queue},
  String, Vec,
};
use stm32f4xx_hal::{
  gpio::{gpioa::PA2, gpioa::PA3, Alternate, AF7},
  interrupt,
  pac::{self, USART2},
  prelude::*,
  serial::{config::Config, Rx, Serial, Tx},
};

// 缓冲区大小配置
const TX_BUFFER_SIZE: usize = 512;
const RX_BUFFER_SIZE: usize = 256;
const COMMAND_BUFFER_SIZE: usize = 128;
const MAX_ARGS: usize = 8;

// 全局变量
static mut TX_QUEUE: Queue<u8, TX_BUFFER_SIZE> = Queue::new();
static mut RX_QUEUE: Queue<u8, RX_BUFFER_SIZE> = Queue::new();
static mut TX_PRODUCER: Option<Producer<u8, TX_BUFFER_SIZE>> = None;
static mut TX_CONSUMER: Option<Consumer<u8, TX_BUFFER_SIZE>> = None;
static mut RX_PRODUCER: Option<Producer<u8, RX_BUFFER_SIZE>> = None;
static mut RX_CONSUMER: Option<Consumer<u8, RX_BUFFER_SIZE>> = None;

static mut UART_TX: Option<Tx<USART2>> = None;
static mut UART_RX: Option<Rx<USART2>> = None;

static TX_READY: AtomicBool = AtomicBool::new(true);

// 命令处理结果
#[derive(Debug, PartialEq)]
enum CommandResult {
  Ok,
  InvalidCommand,
  InvalidArgs,
  ExecutionError,
}

// 系统状态
struct SystemState {
  led_state: bool,
  counter: u32,
  debug_mode: bool,
  auto_response: bool,
}

static mut SYSTEM_STATE: SystemState = SystemState {
  led_state: false,
  counter: 0,
  debug_mode: false,
  auto_response: true,
};

#[entry]
fn main() -> ! {
  let dp = pac::Peripherals::take().unwrap();
  let cp = cortex_m::Peripherals::take().unwrap();

  // 配置时钟
  let rcc = dp.RCC.constrain();
  let clocks = rcc.cfgr.sysclk(84.mhz()).freeze();

  // 配置GPIO
  let gpioa = dp.GPIOA.split();
  let tx_pin = gpioa.pa2.into_alternate::<AF7>();
  let rx_pin = gpioa.pa3.into_alternate::<AF7>();

  // 配置UART
  let serial = Serial::new(
    dp.USART2,
    (tx_pin, rx_pin),
    Config::default().baudrate(115200.bps()),
    &clocks,
  )
  .unwrap();

  let (tx, rx) = serial.split();

  // 初始化队列
  let (tx_prod, tx_cons) = unsafe { TX_QUEUE.split() };
  let (rx_prod, rx_cons) = unsafe { RX_QUEUE.split() };

  unsafe {
    TX_PRODUCER = Some(tx_prod);
    TX_CONSUMER = Some(tx_cons);
    RX_PRODUCER = Some(rx_prod);
    RX_CONSUMER = Some(rx_cons);
    UART_TX = Some(tx);
    UART_RX = Some(rx);
  }

  // 启用UART中断
  unsafe {
    if let Some(ref mut uart_rx) = UART_RX {
      uart_rx.listen();
    }
  }

  // 启用NVIC中断
  unsafe {
    pac::NVIC::unmask(pac::Interrupt::USART2);
  }

  // 发送启动消息
  send_welcome_message();

  let mut command_buffer: Vec<u8, COMMAND_BUFFER_SIZE> = Vec::new();
  let mut escape_sequence = false;
  let mut cursor_pos = 0usize;

  loop {
    // 处理接收到的数据
    while let Some(byte) = receive_byte() {
      match byte {
        // 回车键 - 执行命令
        b'\r' | b'\n' => {
          if !command_buffer.is_empty() {
            send_string("\r\n");
            execute_command(&command_buffer);
            command_buffer.clear();
            cursor_pos = 0;
          }
          send_prompt();
        }

        // 退格键
        b'\x08' | b'\x7f' => {
          if cursor_pos > 0 && !command_buffer.is_empty() {
            command_buffer.remove(cursor_pos - 1);
            cursor_pos -= 1;
            redraw_line(&command_buffer, cursor_pos);
          }
        }

        // ESC序列开始
        0x1B => {
          escape_sequence = true;
        }

        // Tab键 - 命令补全
        b'\t' => {
          if !escape_sequence {
            handle_tab_completion(&mut command_buffer, &mut cursor_pos);
          }
        }

        // Ctrl+C - 取消当前命令
        0x03 => {
          send_string("^C\r\n");
          command_buffer.clear();
          cursor_pos = 0;
          send_prompt();
        }

        // Ctrl+L - 清屏
        0x0C => {
          send_string("\x1B[2J\x1B[H");
          send_welcome_message();
          send_prompt();
        }

        // 可打印字符
        0x20..=0x7E => {
          if !escape_sequence && command_buffer.len() < COMMAND_BUFFER_SIZE - 1 {
            if cursor_pos < command_buffer.len() {
              command_buffer.insert(cursor_pos, byte).ok();
            } else {
              command_buffer.push(byte).ok();
            }
            cursor_pos += 1;
            redraw_line(&command_buffer, cursor_pos);
          }
          escape_sequence = false;
        }

        _ => {
          escape_sequence = false;
        }
      }
    }

    // 自动响应模式
    unsafe {
      if SYSTEM_STATE.auto_response {
        SYSTEM_STATE.counter += 1;
        if SYSTEM_STATE.counter >= 10_000_000 {
          SYSTEM_STATE.counter = 0;
          if SYSTEM_STATE.debug_mode {
            send_string("\r\n[DEBUG] Heartbeat\r\n");
            send_prompt();
          }
        }
      }
    }

    cortex_m::asm::nop();
  }
}

fn send_welcome_message() {
  send_string("\r\n");
  send_string("=====================================\r\n");
  send_string("  Advanced UART Command Parser\r\n");
  send_string("=====================================\r\n");
  send_string("Type 'help' for available commands\r\n");
  send_string("Use Tab for command completion\r\n");
  send_string("Ctrl+C to cancel, Ctrl+L to clear\r\n");
  send_string("=====================================\r\n");
}

fn send_prompt() {
  unsafe {
    if SYSTEM_STATE.debug_mode {
      send_string("[DEBUG] ");
    }
  }
  send_string("uart> ");
}

fn execute_command(command_buffer: &[u8]) {
  let cmd_str = match core::str::from_utf8(command_buffer) {
    Ok(s) => s.trim(),
    Err(_) => {
      send_string("Error: Invalid UTF-8 sequence\r\n");
      return;
    }
  };

  if cmd_str.is_empty() {
    return;
  }

  let args: Vec<&str, MAX_ARGS> = cmd_str.split_whitespace().collect();
  if args.is_empty() {
    return;
  }

  let result = match args[0] {
    "help" | "h" => cmd_help(&args),
    "status" | "st" => cmd_status(&args),
    "set" => cmd_set(&args),
    "get" => cmd_get(&args),
    "echo" => cmd_echo(&args),
    "led" => cmd_led(&args),
    "debug" => cmd_debug(&args),
    "reset" => cmd_reset(&args),
    "version" | "ver" => cmd_version(&args),
    "clear" | "cls" => cmd_clear(&args),
    "auto" => cmd_auto(&args),
    _ => {
      send_string("Unknown command: ");
      send_string(args[0]);
      send_string("\r\nType 'help' for available commands\r\n");
      CommandResult::InvalidCommand
    }
  };

  match result {
    CommandResult::Ok => {
      if unsafe { SYSTEM_STATE.debug_mode } {
        send_string("[DEBUG] Command executed successfully\r\n");
      }
    }
    CommandResult::InvalidArgs => {
      send_string("Error: Invalid arguments\r\n");
    }
    CommandResult::ExecutionError => {
      send_string("Error: Command execution failed\r\n");
    }
    CommandResult::InvalidCommand => {
      // Already handled above
    }
  }
}

fn cmd_help(_args: &[&str]) -> CommandResult {
  send_string("Available commands:\r\n");
  send_string("  help, h          - Show this help\r\n");
  send_string("  status, st       - Show system status\r\n");
  send_string("  set <key> <val>  - Set parameter\r\n");
  send_string("  get <key>        - Get parameter value\r\n");
  send_string("  echo <text>      - Echo text back\r\n");
  send_string("  led <on|off>     - Control LED\r\n");
  send_string("  debug <on|off>   - Toggle debug mode\r\n");
  send_string("  auto <on|off>    - Toggle auto response\r\n");
  send_string("  reset            - Reset system\r\n");
  send_string("  version, ver     - Show version info\r\n");
  send_string("  clear, cls       - Clear screen\r\n");
  CommandResult::Ok
}

fn cmd_status(_args: &[&str]) -> CommandResult {
  unsafe {
    let mut status: String<256> = String::new();
    use core::fmt::Write;

    write!(status, "System Status:\r\n").ok();
    write!(
      status,
      "  LED: {}\r\n",
      if SYSTEM_STATE.led_state { "ON" } else { "OFF" }
    )
    .ok();
    write!(status, "  Counter: {}\r\n", SYSTEM_STATE.counter).ok();
    write!(
      status,
      "  Debug Mode: {}\r\n",
      if SYSTEM_STATE.debug_mode { "ON" } else { "OFF" }
    )
    .ok();
    write!(
      status,
      "  Auto Response: {}\r\n",
      if SYSTEM_STATE.auto_response {
        "ON"
      } else {
        "OFF"
      }
    )
    .ok();

    send_string(&status);
  }
  CommandResult::Ok
}

fn cmd_set(args: &[&str]) -> CommandResult {
  if args.len() != 3 {
    send_string("Usage: set <key> <value>\r\n");
    send_string("Available keys: counter\r\n");
    return CommandResult::InvalidArgs;
  }

  match args[1] {
    "counter" => {
      if let Ok(value) = args[2].parse::<u32>() {
        unsafe {
          SYSTEM_STATE.counter = value;
        }
        send_string("Counter set to ");
        send_string(args[2]);
        send_string("\r\n");
        CommandResult::Ok
      } else {
        send_string("Error: Invalid number format\r\n");
        CommandResult::InvalidArgs
      }
    }
    _ => {
      send_string("Error: Unknown parameter '");
      send_string(args[1]);
      send_string("'\r\n");
      CommandResult::InvalidArgs
    }
  }
}

fn cmd_get(args: &[&str]) -> CommandResult {
  if args.len() != 2 {
    send_string("Usage: get <key>\r\n");
    send_string("Available keys: counter, led, debug, auto\r\n");
    return CommandResult::InvalidArgs;
  }

  unsafe {
    match args[1] {
      "counter" => {
        let mut response: String<64> = String::new();
        use core::fmt::Write;
        write!(response, "counter = {}\r\n", SYSTEM_STATE.counter).ok();
        send_string(&response);
      }
      "led" => {
        send_string("led = ");
        send_string(if SYSTEM_STATE.led_state { "on" } else { "off" });
        send_string("\r\n");
      }
      "debug" => {
        send_string("debug = ");
        send_string(if SYSTEM_STATE.debug_mode { "on" } else { "off" });
        send_string("\r\n");
      }
      "auto" => {
        send_string("auto = ");
        send_string(if SYSTEM_STATE.auto_response {
          "on"
        } else {
          "off"
        });
        send_string("\r\n");
      }
      _ => {
        send_string("Error: Unknown parameter '");
        send_string(args[1]);
        send_string("'\r\n");
        return CommandResult::InvalidArgs;
      }
    }
  }
  CommandResult::Ok
}

fn cmd_echo(args: &[&str]) -> CommandResult {
  if args.len() < 2 {
    send_string("Usage: echo <text>\r\n");
    return CommandResult::InvalidArgs;
  }

  send_string("Echo: ");
  for (i, arg) in args.iter().skip(1).enumerate() {
    if i > 0 {
      send_string(" ");
    }
    send_string(arg);
  }
  send_string("\r\n");
  CommandResult::Ok
}

fn cmd_led(args: &[&str]) -> CommandResult {
  if args.len() != 2 {
    send_string("Usage: led <on|off>\r\n");
    return CommandResult::InvalidArgs;
  }

  match args[1] {
    "on" | "1" | "true" => {
      unsafe {
        SYSTEM_STATE.led_state = true;
      }
      send_string("LED turned ON\r\n");
      CommandResult::Ok
    }
    "off" | "0" | "false" => {
      unsafe {
        SYSTEM_STATE.led_state = false;
      }
      send_string("LED turned OFF\r\n");
      CommandResult::Ok
    }
    _ => {
      send_string("Error: Use 'on' or 'off'\r\n");
      CommandResult::InvalidArgs
    }
  }
}

fn cmd_debug(args: &[&str]) -> CommandResult {
  if args.len() != 2 {
    send_string("Usage: debug <on|off>\r\n");
    return CommandResult::InvalidArgs;
  }

  match args[1] {
    "on" | "1" | "true" => {
      unsafe {
        SYSTEM_STATE.debug_mode = true;
      }
      send_string("Debug mode enabled\r\n");
      CommandResult::Ok
    }
    "off" | "0" | "false" => {
      unsafe {
        SYSTEM_STATE.debug_mode = false;
      }
      send_string("Debug mode disabled\r\n");
      CommandResult::Ok
    }
    _ => {
      send_string("Error: Use 'on' or 'off'\r\n");
      CommandResult::InvalidArgs
    }
  }
}

fn cmd_auto(args: &[&str]) -> CommandResult {
  if args.len() != 2 {
    send_string("Usage: auto <on|off>\r\n");
    return CommandResult::InvalidArgs;
  }

  match args[1] {
    "on" | "1" | "true" => {
      unsafe {
        SYSTEM_STATE.auto_response = true;
      }
      send_string("Auto response enabled\r\n");
      CommandResult::Ok
    }
    "off" | "0" | "false" => {
      unsafe {
        SYSTEM_STATE.auto_response = false;
      }
      send_string("Auto response disabled\r\n");
      CommandResult::Ok
    }
    _ => {
      send_string("Error: Use 'on' or 'off'\r\n");
      CommandResult::InvalidArgs
    }
  }
}

fn cmd_reset(_args: &[&str]) -> CommandResult {
  send_string("Resetting system...\r\n");
  unsafe {
    SYSTEM_STATE = SystemState {
      led_state: false,
      counter: 0,
      debug_mode: false,
      auto_response: true,
    };
  }
  send_string("System reset complete\r\n");
  CommandResult::Ok
}

fn cmd_version(_args: &[&str]) -> CommandResult {
  send_string("UART Command Parser v1.0.0\r\n");
  send_string("Built with Rust embedded\r\n");
  send_string("STM32F4xx HAL support\r\n");
  CommandResult::Ok
}

fn cmd_clear(_args: &[&str]) -> CommandResult {
  send_string("\x1B[2J\x1B[H");
  send_welcome_message();
  CommandResult::Ok
}

fn handle_tab_completion(
  command_buffer: &mut Vec<u8, COMMAND_BUFFER_SIZE>,
  cursor_pos: &mut usize,
) {
  let cmd_str = core::str::from_utf8(command_buffer).unwrap_or("");
  let commands = [
    "help", "status", "set", "get", "echo", "led", "debug", "reset", "version", "clear", "auto",
  ];

  let matches: Vec<&str, 16> = commands
    .iter()
    .filter(|cmd| cmd.starts_with(cmd_str))
    .copied()
    .collect();

  match matches.len() {
    0 => {
      // 没有匹配，发出提示音
      send_byte(0x07);
    }
    1 => {
      // 唯一匹配，自动补全
      command_buffer.clear();
      *cursor_pos = 0;
      for byte in matches[0].bytes() {
        command_buffer.push(byte).ok();
        *cursor_pos += 1;
      }
      redraw_line(command_buffer, *cursor_pos);
    }
    _ => {
      // 多个匹配，显示选项
      send_string("\r\nPossible completions:\r\n");
      for cmd in matches.iter() {
        send_string("  ");
        send_string(cmd);
        send_string("\r\n");
      }
      send_prompt();
      send_string(cmd_str);
    }
  }
}

fn redraw_line(command_buffer: &[u8], cursor_pos: usize) {
  // 清除当前行
  send_string("\r");
  send_prompt();

  // 重新绘制命令
  let cmd_str = core::str::from_utf8(command_buffer).unwrap_or("");
  send_string(cmd_str);

  // 移动光标到正确位置
  if cursor_pos < command_buffer.len() {
    let move_back = command_buffer.len() - cursor_pos;
    for _ in 0..move_back {
      send_string("\x1B[D");
    }
  }
}

fn send_byte(byte: u8) -> bool {
  unsafe {
    if let Some(ref mut producer) = TX_PRODUCER {
      match producer.enqueue(byte) {
        Ok(()) => {
          if TX_READY.load(Ordering::Relaxed) {
            start_transmission();
          }
          true
        }
        Err(_) => false,
      }
    } else {
      false
    }
  }
}

fn send_string(s: &str) -> usize {
  let mut sent_count = 0;
  for byte in s.bytes() {
    if send_byte(byte) {
      sent_count += 1;
    } else {
      break;
    }
  }
  sent_count
}

fn receive_byte() -> Option<u8> {
  unsafe {
    if let Some(ref mut consumer) = RX_CONSUMER {
      consumer.dequeue()
    } else {
      None
    }
  }
}

fn start_transmission() {
  unsafe {
    if let (Some(ref mut consumer), Some(ref mut uart_tx)) =
      (TX_CONSUMER.as_mut(), UART_TX.as_mut())
    {
      if let Some(byte) = consumer.dequeue() {
        TX_READY.store(false, Ordering::Relaxed);
        uart_tx.write(byte).ok();
        uart_tx.listen();
      }
    }
  }
}

#[interrupt]
fn USART2() {
  unsafe {
    if let Some(ref mut uart_rx) = UART_RX {
      if uart_rx.is_rxne() {
        if let Ok(byte) = uart_rx.read() {
          if let Some(ref mut producer) = RX_PRODUCER {
            producer.enqueue(byte).ok();
          }
        }
      }
    }

    if let Some(ref mut uart_tx) = UART_TX {
      if uart_tx.is_txe() {
        if let Some(ref mut consumer) = TX_CONSUMER {
          if let Some(byte) = consumer.dequeue() {
            uart_tx.write(byte).ok();
          } else {
            uart_tx.unlisten();
            TX_READY.store(true, Ordering::Relaxed);
          }
        }
      }
    }
  }
}

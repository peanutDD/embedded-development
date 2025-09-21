/* STM32F407VG 内存布局配置 */
/* 链接器脚本，定义Flash和RAM的内存映射 */

MEMORY
{
  /* Flash存储器 - 用于存储程序代码和常量数据 */
  /* STM32F407VG: 1MB Flash (0x08000000 - 0x080FFFFF) */
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  
  /* SRAM存储器 - 用于存储变量和堆栈 */
  /* STM32F407VG: 128KB SRAM (0x20000000 - 0x2001FFFF) */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

/* 程序入口点 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
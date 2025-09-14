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

/* 确保向量表位于Flash起始位置 */
ASSERT(ORIGIN(FLASH) == 0x08000000, "Flash起始地址必须为0x08000000");

/* 确保有足够的堆栈空间 */
ASSERT(LENGTH(RAM) >= 8K, "RAM大小必须至少8KB以确保基本运行");

/* 调试信息 */
/* Flash使用情况将在编译时显示 */
/* RAM使用情况将在编译时显示 */
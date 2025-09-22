/* STM32F407VG 内存布局 */
/* 可根据具体STM32型号调整 */

MEMORY
{
  /* Flash存储器 - 1MB */
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  
  /* SRAM - 128KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
  
  /* CCM RAM (Core Coupled Memory) - 64KB */
  /* 仅STM32F4系列部分型号支持 */
  CCMRAM : ORIGIN = 0x10000000, LENGTH = 64K
}

/* 栈顶位置 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* 堆大小配置 */
_heap_size = 32K;

/* 中断向量表位置 */
PROVIDE(_stext = ORIGIN(FLASH));

/* 引导加载程序配置 */
/* 如果使用引导加载程序，需要调整FLASH起始地址 */
/* FLASH : ORIGIN = 0x08008000, LENGTH = 992K */
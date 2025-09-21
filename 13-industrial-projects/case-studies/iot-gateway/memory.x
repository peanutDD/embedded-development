/* STM32F407VGTx 内存布局 */
MEMORY
{
  /* Flash memory: 1024KB */
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  
  /* RAM: 128KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}

/* 栈顶地址 (RAM 结束地址) */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
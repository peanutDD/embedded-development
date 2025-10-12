/* STM32F407VG 内存配置 */

/* FLASH 配置 - 起始地址 0x08000000，大小 512KB */
MEMORY
{
  /* 优化的 FLASH 区域划分 */
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
  
  /* 优化的 RAM 区域划分 */
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
  
  /* 为频繁访问的数据创建专用内存区域 */
  RAM_FAST (rwx) : ORIGIN = 0x20000000 + 96K - 16K, LENGTH = 16K
}

/* 栈起始地址 - 从 RAM 顶部开始，大小 16KB */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* 自定义内存区域属性 */
SECTIONS
{
  /* 优化的 .text 段 - 代码段 */
  .text :
  {
    KEEP(*(.isr_vector)) /* 中断向量表 */
    *(.text*)           /* 程序代码 */
    *(.rodata*)         /* 只读数据 */
    _etext = .;         /* 文本段结束标记 */
  } > FLASH

  /* 优化的数据段 - 初始化数据 */
  .data : AT (_etext)
  {
    _sdata = .;         /* 数据段开始标记 */
    *(.data*)           /* 初始化的全局变量 */
    _edata = .;         /* 数据段结束标记 */
  } > RAM

  /* 快速数据区域 - 用于频繁访问的变量 */
  .data.fast : AT (_edata)
  {
    _sdata_fast = .;    /* 快速数据段开始标记 */
    *(.data.fast*)      /* 标记为快速访问的数据 */
    _edata_fast = .;    /* 快速数据段结束标记 */
  } > RAM_FAST

  /* BSS 段 - 未初始化数据 */
  .bss :
  {
    _sbss = .;          /* BSS 段开始标记 */
    *(.bss*)            /* 未初始化的全局变量 */
    *(COMMON)
    _ebss = .;          /* BSS 段结束标记 */
  } > RAM

  /* 堆区域 - 从 BSS 段结束处开始，大小为剩余 RAM 空间减去栈空间 */
  _sheap = _ebss;
  _eheap = ORIGIN(RAM) + LENGTH(RAM) - LENGTH(RAM_FAST);

  /* 确保内存区域对齐 */
  . = ALIGN(4);
}
/* 
 * STM32F4xx 系列内存布局示例
 * 适用于 STM32F401/F411 等型号
 */

MEMORY
{
  /* Flash 存储器 - 512KB */
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  
  /* SRAM - 96KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}

/* 程序入口点 */
ENTRY(Reset);

/* 栈顶地址 (RAM 末尾) */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  /* 向量表必须位于 Flash 起始位置 */
  .vector_table ORIGIN(FLASH) :
  {
    /* 初始栈指针 */
    LONG(_stack_start);
    
    /* 复位向量 */
    KEEP(*(.vector_table.reset_vector));
    
    /* 异常向量 */
    KEEP(*(.vector_table.exceptions));
    
    /* 中断向量 */
    KEEP(*(.vector_table.interrupts));
  } > FLASH

  /* 程序代码段 */
  .text :
  {
    *(.text .text.*);
    
    /* 确保代码段对齐 */
    . = ALIGN(4);
  } > FLASH

  /* 只读数据段 */
  .rodata :
  {
    *(.rodata .rodata.*);
    
    /* 字符串常量 */
    *(.rodata.str1.1);
    *(.rodata.str1.4);
    
    . = ALIGN(4);
  } > FLASH

  /* 初始化数据段 (存储在 Flash，运行时复制到 RAM) */
  .data : AT(ADDR(.rodata) + SIZEOF(.rodata))
  {
    _sdata = .;
    *(.data .data.*);
    . = ALIGN(4);
    _edata = .;
  } > RAM

  /* 数据段在 Flash 中的位置 */
  _sidata = LOADADDR(.data);

  /* 未初始化数据段 (BSS) */
  .bss :
  {
    _sbss = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
    _ebss = .;
  } > RAM

  /* 堆区域 (可选) */
  .heap :
  {
    _sheap = .;
    . = . + 1K; /* 1KB 堆空间 */
    _eheap = .;
  } > RAM

  /* 栈区域 */
  .stack :
  {
    _estack = .;
    . = . + 2K; /* 2KB 栈空间 */
    _sstack = .;
  } > RAM

  /* 调试信息 (不占用 Flash 空间) */
  .debug_info 0 : { *(.debug_info) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_line 0 : { *(.debug_line) }
  .debug_str 0 : { *(.debug_str) }
  .debug_ranges 0 : { *(.debug_ranges) }

  /* 确保栈不会溢出到其他区域 */
  ASSERT(_sstack <= _stack_start, "Stack overflow into other memory regions");
  
  /* 确保程序不会超出 Flash 大小 */
  ASSERT(SIZEOF(.text) + SIZEOF(.rodata) + SIZEOF(.data) <= LENGTH(FLASH), 
         "Program too large for Flash memory");
}
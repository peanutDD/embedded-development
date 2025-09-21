/* 
 * Application 内存布局
 * Application 从 0x08008000 开始，大小 480KB
 */

MEMORY
{
  /* Application Flash - 480KB */
  FLASH : ORIGIN = 0x08008000, LENGTH = 480K
  
  /* SRAM - 96KB */
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}

/* 程序入口点 */
ENTRY(Reset);

/* 栈顶地址 */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  /* 向量表 */
  .vector_table ORIGIN(FLASH) :
  {
    LONG(_stack_start);
    KEEP(*(.vector_table.reset_vector));
    KEEP(*(.vector_table.exceptions));
    KEEP(*(.vector_table.interrupts));
  } > FLASH

  /* 应用程序代码段 */
  .text :
  {
    *(.text .text.*);
    . = ALIGN(4);
  } > FLASH

  /* 只读数据段 */
  .rodata :
  {
    *(.rodata .rodata.*);
    . = ALIGN(4);
  } > FLASH

  /* 初始化数据段 */
  .data : AT(ADDR(.rodata) + SIZEOF(.rodata))
  {
    _sdata = .;
    *(.data .data.*);
    . = ALIGN(4);
    _edata = .;
  } > RAM

  _sidata = LOADADDR(.data);

  /* 未初始化数据段 */
  .bss :
  {
    _sbss = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
    _ebss = .;
  } > RAM
}
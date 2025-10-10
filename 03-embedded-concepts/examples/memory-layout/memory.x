/* 最小化的链接器脚本 */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 512K
  RAM : ORIGIN = 0x20000000, LENGTH = 96K
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  /* 基本内存段定义 */
  .text : { *(.text*) }
  .rodata : { *(.rodata*) }
  .data : {
    _sdata = .;
    *(.data*);
    _edata = .;
  } > RAM AT > FLASH
  .bss : {
    _sbss = .;
    *(.bss*);
    *(COMMON);
    _ebss = .;
  } > RAM
}
/* Memory layout for STM32F407VGT6 */
MEMORY
{
  /* Flash memory: 1MB (0x100000 bytes) */
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  
  /* RAM: 128KB (0x20000 bytes) */
  RAM : ORIGIN = 0x20000000, LENGTH = 128K
}
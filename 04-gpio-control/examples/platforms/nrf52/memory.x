/* Memory layout for nRF52840 */
MEMORY
{
  /* Flash memory: 1MB (0x100000 bytes) */
  /* SoftDevice S140 occupies first 152KB (0x26000 bytes) */
  FLASH : ORIGIN = 0x00026000, LENGTH = 1024K - 152K
  
  /* RAM: 256KB (0x40000 bytes) */
  /* SoftDevice S140 occupies first 64KB (0x10000 bytes) */
  RAM : ORIGIN = 0x20010000, LENGTH = 256K - 64K
}
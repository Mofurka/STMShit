MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Обязательная секция для cortex-m-rt */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

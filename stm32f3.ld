/* Define memory regions. */
MEMORY
{
    rom (rx) : ORIGIN = 0x08000000, LENGTH = 64K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 12K
}

/* Include the common ld script. */
INCLUDE stm32/f3/libopencm3_stm32f3.ld

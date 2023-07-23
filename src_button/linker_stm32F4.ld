/*
    Entry Point from the Startup File
        * generally called from Reset_Handler
*/
ENTRY(Reset_Handler)

/*
    CPU memory configuration variables - this is architecture specific
    For the STM32F407Vx series we have the following
        * Flash - 1024kb
        * SRAM1 (Main Internal Memory) - 112KB
        * SRAM2 (Auxillary Internal Memory) - 16KB
        * CCM (Core-Coupled Memroy)- 64KB
        * Backup - 4KB 
        ------------------------------------------
        Flash   -  1024kb - 0x100000 - 1,048,576 bytes
        SRAM    -  128kb - 0x20000 - 131,072 bytes
*/
FLASH_SIZE =    0x100000;
SRAM_SIZE =     0x20000;

/*
    Structure
        * name [(attr)] : ORIGIN = origin, LENGTH = len
    
    Attributes
        r - read-only section
        x - exectuable section
        w - read/write section
        a - allocatable section
        i - initialized section
        l - same as i
        ! - invert any sense of the following attributes

    Resource
        *https://sourceware.org/binutils/docs/ld/MEMORY.html#MEMORY
*/
MEMORY
{
  FLASH(rx) : ORIGIN = 0x08000000, LENGTH = FLASH_SIZE
  SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = SRAM_SIZE
}

/*
    The sections informs the compiler how to properly combine the input files into a single .ELF
*/
SECTIONS
{
    /*
        The .text section contains the main exectuable code of the program - put into FLASH (ROM)
            * Vector Table
            * Initialization
            * Clean up code
    */
  .text :
  {
    KEEP(*(.isr_vector))

    *(.text .text.*)

    KEEP(*(.init)) 
    KEEP(*(.fini))

    *(.rodata .rodata*)

    . = ALIGN(4);
    _text_end = .;

  } > FLASH

  /*
    Store the data address from flash - Load Memory Address (LMA)
    Used as source to copy .data section from ROM to RAM in Startup File
    Start Initial Data
  */ 
  _si_data = LOADADDR(.data);

  /*
    Place data section at the end of text section - Virtual Memory Address (VMA)
    Record data boundary address in RAM - used in Startup to copy section from ROM to RAM
    Section contains initialized variables
        * const
        * static 
  */
  .data : AT(_text_end)
  {
    _data_start = .;

    *(.data .data.*)

    . = ALIGN(4);
    _data_end = .;
    __data_end__ = .;

  } > SRAM

  /*
    Block Started by Symbol
    Record bss boundary address in RAM - used in Startup File to zero out uninitialized variables
    Section contains uninitialized variables
    COMMON - gathers uninitialized variables not found in .bss files
  */
  
  .bss :
  {
    _bss_start = .;
    __bss_start__ = .;

    *(.bss .bss.*)

    *(COMMON)
			
    . = ALIGN(4);
    _bss_end = .;
    __bss_end__ = .;
    _end = .;
    __end__ = .;
  } > SRAM
  
}
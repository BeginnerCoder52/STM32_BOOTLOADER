/* APP_CURRENT LinkerScript.ld (STM32F103C8Tx - 128KB CHIP USED)
   - App Current code at 0x08004000
   - 55KB code region + 1KB metadata page
*/

ENTRY(Reset_Handler)

/* App Current base */
APP_CUR_START = 0x08004000;

/* RAM end for stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Heap/Stack sizes */
_Min_Heap_Size  = 0x200; /* 512B */
_Min_Stack_Size = 0x400; /* 1KB */

/* Memories definition */
MEMORY
{
  /* 55KB code region */
  FLASH_APP_CUR_CODE (rx) : ORIGIN = APP_CUR_START,           LENGTH = 55K
  /* 1KB metadata region */
  FLASH_APP_CUR_META (rx) : ORIGIN = APP_CUR_START + 55K,     LENGTH = 1K

  /* 20KB RAM */
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
}

/* Sections */
SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH_APP_CUR_CODE

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*)
    *(.glue_7) *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;
  } > FLASH_APP_CUR_CODE

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata) *(.rodata*)
    . = ALIGN(4);
  } > FLASH_APP_CUR_CODE

  .ARM.extab (READONLY) :
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } > FLASH_APP_CUR_CODE

  .ARM (READONLY) :
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } > FLASH_APP_CUR_CODE

  .preinit_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
    . = ALIGN(4);
  } > FLASH_APP_CUR_CODE

  .init_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
    . = ALIGN(4);
  } > FLASH_APP_CUR_CODE

  .fini_array (READONLY) :
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
  } > FLASH_APP_CUR_CODE

  _sidata = LOADADDR(.data);

  .data : AT (ADDR(.rodata) + SIZEOF(.rodata))
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    *(.RamFunc) *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } > RAM

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } > RAM

  /* Dedicated metadata page (1KB) at end of App Current */
  .app_cur_metadata ORIGIN(FLASH_APP_CUR_META) :
  {
    . = ALIGN(4);
    KEEP(*(.app_metadata))
    . = ALIGN(4);
  } > FLASH_APP_CUR_META

  /* Export metadata base to C */
  PROVIDE(__app_cur_meta_start = ORIGIN(FLASH_APP_CUR_META));
  PROVIDE(__app_cur_meta_end   = ORIGIN(FLASH_APP_CUR_META) + LENGTH(FLASH_APP_CUR_META));

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a  ( * )
    libgcc.a( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

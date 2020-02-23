/*-----------------------------------------------------------------------------
   Linker File
-------------------------------------------------------------------------------
   Author: Fernando Ojeda L.                                          Dic-2019
-------------------------------------------------------------------------------
   Description:
-------------------------------------------------------------------------------
   
   KEEP -- avoids a section to be removed by a garbage collector
   The ISR vector table has an offset VTOR = 0x20000000
   
   At boot time, the Reset Vector (isr_vector) must be mapped to ROM. However,
   most ARM microcontrollers provide an option to remap the memories to put RAM
   at the ARM vector addresses, so that the vectors can be dynamically changed 
   under software control.
 
-------------------------------------------------------------------------------*/



/*ENTRY(startup) */

MEMORY
{
RAM_ITCM(rx)       : ORIGIN = 0x00000000, LENGTH = 16K
FLASH_AXI(rx)      : ORIGIN = 0x08000000, LENGTH = 1M
RAM_DTCM (xrw)     : ORIGIN = 0x20010000, LENGTH = 240K   
}


SECTIONS
{
	.isr_vector :
	{
		. = ALIGN(4);
		KEEP(*(.isr_vector))        /* Startup code */
		. = ALIGN(4);
	}>FLASH_AXI
	
    .text : 
    {
        *(.text)                   /* Program code */
    }>FLASH_AXI
    
	.rodata : 
    {
        *(.rodata)                 /* Read only data */
    }>FLASH_AXI
    _DATA_ROM_START = .;
    
    .data :                         
    {
		_DATA_RAM_START = .;
        *(.data)                   /* Data memory, defines the runtime position in RAM */
        _DATA_RAM_END = .;		
    } >RAM_DTCM AT> FLASH_AXI     /* AT defines the loadtime position in ROM */

    .bss :
    {
		_BSS_START = .;            /* Indicates where BSS section starts in RAM */
        *(.bss)                    /* Zero-filled run time allocate data memory */
		_BSS_END = .;              /* Indicates where BSS section ends in RAM */
    }>RAM_DTCM
}



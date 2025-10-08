ENTRY(reset_handler)

MEMORY
{
	FLASH (rx): ORIGIN = 0x8000000, LENGTH = 512K
	SRAM (rwx): ORIGIN = 0x20000000, LENGTH = 112K
}

SECTIONS
{
	.isr_vector :
	{
		KEEP(*(.isr_vector)) /* do not let gc remove the section */
	} >FLASH

	.text :
	{
		. = ALIGN(4);

		*(.text*)
		*(.rodata*)
		*(.ARM.exidx)

		. = ALIGN(4);
		_etext = .;
	} >FLASH

	.data : 
	{
		. = ALIGN(4);
		_sdata = .;

		*(.data*)

		. = ALIGN(4);
		_edata = .;
	} >SRAM AT>FLASH

	.bss :
	{
		. = ALIGN(4);
		_sbss = .;

		*(.bss*)

		. = ALIGN(4);
		_ebss = .;
	} >SRAM
}
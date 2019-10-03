/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

/* QFAC */
/*
MEMORY
{
  FLASH (rx) : ORIGIN = 0x1b000, LENGTH = 0x40000
  RAM (rwx) :  ORIGIN = 0x20002080, LENGTH = 0x5f80
}
*/

/* QFAA */
MEMORY
{
  FLASH (rx) : ORIGIN = 0x1b000, LENGTH = 0x40000
  RAM (rwx) :  ORIGIN = 0x20002080, LENGTH = 0x1a00
}
/* QFAC*/
/*
SECTIONS
{
  .fs_data :
  {
    PROVIDE(__start_fs_data = .);
    KEEP(*(.fs_data))
    PROVIDE(__stop_fs_data = .);
  } > RAM
} INSERT AFTER .data;
*/

/* QFAA */
SECTIONS
{
  .fs_data_out ALIGN(4):
  {
    PROVIDE( __start_fs_data = .);
    KEEP(*(fs_data))
    PROVIDE( __stop_fs_data = .);
  } = 0
}


INCLUDE "nrf5x_common.ld"
/* Override heap end to reserve space */
PROVIDE(_heap_end = 0x3FCCF800);

SECTIONS
{
  .libif 0x3FCCF800 (NOLOAD) :
  {
    _libif_start = ABSOLUTE(.);
    KEEP(*(.libif))
    . = ALIGN(4);
    _libif_end = ABSOLUTE(.);
    . = . + 0x800;  /* Reserve exactly 2KB */
  }
}
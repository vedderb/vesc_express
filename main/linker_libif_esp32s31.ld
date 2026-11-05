/* Fixed native library interface address, matching VESC_IF in
   c_libs/vesc_c_if.h. Below the ROM stack reservation at 0x2F07AFB0;
   lispif_c_lib.c excludes this section from the heap. */
SECTIONS
{
  .libif 0x2F07A000 (NOLOAD) :
  {
    _libif_start = ABSOLUTE(.);
    KEEP(*(.libif))
    . = ALIGN(4);
    _libif_end = ABSOLUTE(.);
  }
}

/* Places the native library C interface table (the .libif section, defined in
   main/lispBM/src/extensions/vesc_c_if.c) at a fixed address so that
   out-of-tree native libraries can find it through the VESC_IF macro in
   main/lispBM/include/extensions/vesc_c_if.h.

   The address here must match VESC_IF for this target, and the block is
   excluded from the heap with SOC_RESERVE_MEMORY_REGION() in vesc_c_if.c.

   ESP32-P4: below the lowest usable-DRAM end across chip revisions
   (APP_USABLE_DIRAM_END = 0x4FF3AFC0 for rev < 3), so the same address
   works on every revision. */

SECTIONS
{
  .libif 0x4FF3A000 (NOLOAD) :
  {
    _libif_start = ABSOLUTE(.);
    KEEP(*(.libif))
    . = ALIGN(4);
    _libif_end = ABSOLUTE(.);
  }
}

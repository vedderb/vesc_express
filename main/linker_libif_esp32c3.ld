/* Places the native library C interface table (the .libif section, defined in
   main/lispBM/src/extensions/vesc_c_if.c) at a fixed address so that
   out-of-tree native libraries can find it through the VESC_IF macro in
   main/lispBM/include/extensions/vesc_c_if.h.

   The address here must match VESC_IF for this target, and the block is
   excluded from the heap with SOC_RESERVE_MEMORY_REGION() in vesc_c_if.c.

   ESP32-C3: top of the 0x3FCC0000 D/IRAM region, directly below the
   ROM-reserved area (APP_USABLE_DRAM_END = 0x3FCDC710). Placing it at the
   very top keeps the main DRAM heap region contiguous - a block in the
   middle would split it and make large allocations (e.g. the LispBM
   memory pool) fail even with plenty of total free heap. */

SECTIONS
{
  .libif 0x3FCDBE00 (NOLOAD) :
  {
    _libif_start = ABSOLUTE(.);
    KEEP(*(.libif))
    . = ALIGN(4);
    _libif_end = ABSOLUTE(.);
  }
}

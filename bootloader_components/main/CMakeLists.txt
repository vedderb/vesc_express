idf_component_register(SRCS "bootloader_start.c"
                    REQUIRES bootloader bootloader_support spi_flash)

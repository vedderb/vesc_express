(import "pkg::jpg_test_480_320@://vesc_packages/lib_files/files.vescpkg" 'jpg_test_480_320)

; sd0 clk cs reset dc spi-mhz
(disp-load-ili9488 6 5 19 18 7 40)

(disp-reset)
(ext-disp-orientation 1)

; Invert display. Must be used for some displays using this driver.
(ext-disp-cmd 0x21)

(disp-render-jpg jpg_test_480_320 0 0)

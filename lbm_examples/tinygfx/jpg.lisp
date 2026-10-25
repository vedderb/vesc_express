; Keep one display block uncommented. Adjust its GPIO pins for your board.

; ST7789: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
(import "pkg::jpg_test_320_240@://vesc_packages/lib_files/files.vescpkg" 'jpg)
(disp-load-st7789 6 5 19 18 7 40)
(disp-reset)
(def position '(0 0))

; ST7735: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
; (import "pkg::jpg_test_160_80@://vesc_packages/lib_files/files.vescpkg" 'jpg)
; (disp-load-st7735 6 5 19 18 7 40)
; (disp-reset)
; (ext-disp-orientation 1)
; (def position '(0 24))

; ILI9341: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
; (import "pkg::jpg_test_320_240@://vesc_packages/lib_files/files.vescpkg" 'jpg)
; (disp-load-ili9341 6 5 19 18 7 40)
; (disp-reset)
; (def position '(0 0))

; ILI9488: SD0=6 CLK=5 CS=19 RST=18 DC=7, 40 MHz
; (import "pkg::jpg_test_480_320@://vesc_packages/lib_files/files.vescpkg" 'jpg)
; (disp-load-ili9488 6 5 19 18 7 40)
; (disp-reset)
; (ext-disp-orientation 1)
; (ext-disp-cmd 0x21)
; (def position '(0 0))

; SSD1351: SD0=6 CLK=5 CS=19 RST=18 DC=7, 20 MHz
; (import "pkg::jpg_test_320_240@://vesc_packages/lib_files/files.vescpkg" 'jpg)
; (disp-load-ssd1351 6 5 19 18 7 20)
; (disp-reset)
; (def position '(-32 -64))

(disp-render-jpg jpg (ix position 0) (ix position 1))

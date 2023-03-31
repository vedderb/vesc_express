(import "pkg::jpg_test_320_240@://vesc_packages/lib_files/files.vescpkg" 'jpg_test_320_240)

(disp-load-st7789 6 5 19 18 7 40)
(disp-reset)

(disp-render-jpg jpg_test_320_240 0 0)

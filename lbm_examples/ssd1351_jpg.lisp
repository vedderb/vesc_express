(import "pkg::jpg_test_320_240@://vesc_packages/lib_files/files.vescpkg" 'jpg_test_320_240)

(disp-load-ssd1351 6 5 19 18 7 20)
(disp-reset)

(disp-render-jpg jpg_test_320_240 -32 -64)

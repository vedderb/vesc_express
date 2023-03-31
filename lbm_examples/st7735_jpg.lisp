(import "pkg::jpg_test_160_80@://vesc_packages/lib_files/files.vescpkg" 'jpg_test_160_80)

(disp-load-st7735 6 5 19 18 7 40)
(disp-reset)
(ext-disp-orientation 1)

(disp-render-jpg jpg_test_160_80 0 24)

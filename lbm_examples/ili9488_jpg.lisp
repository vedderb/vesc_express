(import "pkg::jpg_test_480_320@://vesc_packages/lib_files/files.vescpkg" 'jpg_test_480_320)

(disp-load-ili9488 6 5 19 18 7 40)
(disp-reset)
(ext-disp-orientation 1)

(disp-render-jpg jpg_test_480_320 0 0)

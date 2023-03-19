(import "img_320_240.jpg" 'img)

(disp-load-ili9341 6 5 19 18 7 40)
(disp-reset)

(disp-render-jpg img 0 0)

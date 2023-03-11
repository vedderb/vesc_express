
# Documentation for graphics library and display drivers

# Introduction

The graphics library provides a set of functions for drawing onto rectangular
images, called Image-buffers, from LispBM. 

Drivers for a specific display is loaded with the associated `disp-load` function:

1. disp-load-sh8501b
2. disp-load-ili9341
3. disp-load-ssd1306

Each disp-load function takes arguments specific to the display. These are
explained below.

Each display driver provides a "rendering", "clearing" and "reset"
function that is connected to the LispBM extensions `disp-render`,
`disp-clear` and `disp-reset` upon loading. The render, clear and
reset functions are the display dependent interface that must be
implemented per display.

## Image-buffers

The graphics library is based on image-buffers, that are rectangular arrays
of pixels where all pixels are of a certain format. The formats we support in
image-buffers are:
1. indexed2 - 2 colors (1 bit per pixel) 
2. indexed4 - 4 colors (2 bits per pixel)
3. rgb332   - 8Bit color
4. rgb565   - 16Bit color
5. rgb888   - 24Bit color

Image-buffers can be created in two ways using the graphics library:
1. `img-buffer` - allocates a blank image-buffer in lbm memory.
2. `img-buffer-from-bin` - creates an image-buffer from an image imported into the script.

For example, creating a 100x100 image-buffer of `indexed2` colors is done as:

```clj
(def my-img (img-buf 'indexed2 100 100))
```

When drawing on `indexed2` or `indexed4` image-buffers, colors are expressed
as an integer. 0,1 for `indexed2` and 0,1,2,3 for `indexed4`. For `rgb332`, `rgb565`
and `rgb888` image-buffers a color is given in hex notation 0xRRGGBB. 

Example that draws a line from pos (0,0) to pos (90,90) in color 1 on an `indexed2` image-buffer:

```clj
(img-line my-img 10 10 90 90 1)
```

When an image-buffer is rendered onto the display (using the
disp-render function) the colors of the image-buffer are mapped to the
color space of the display.  In the case of `indexed2` and `indexed4`
this mapping is performed using a list of target colors expressed in
RGB888 format. 

Note that the RAM requirement of a 100x100 image is:
1. at indexed2: 1250 Bytes
2. at indexed4: 2500 Bytes
3. at rgb332:   10000 Bytes
4. at rgb565:   20000 Bytes
5. at rgb888:   30000 Bytes

There are a number of function for drawing onto an image-buffer:
1. img-setpix
2. img-line
3. img-text
4. img-clear
5. img-circle
6. img-arc
7. img-circle-sector
8. img-circle-segment
9. img-rectangle
10. img-triangle
11. img-blit

The purpose of most of these are given by their names. `img-blit` draws an
image onto another image and can while doing so rotate and scale the image it
is drawing. More details on each of these functions are available in the later
reference sections. 

The functions that draw onto an image-buffer takes optional arguments
that specify extra attributes to apply when drawing. These attributes are:

1. `'(thickness w)`: Line thickness w 
2. `'(dotted d-len d-space-len)`: Dotted or dashed lines with `d-len` long line segments separated by `d-space-len`.
3. `'(filled)` : Specifies that the shape should be filled. 
4. `'(rounded radius)` : Specifies that the shape should have rounded corners. 
5. `'(scale scale-f)` : Scale the image by factor `scale-f`
6. `'(rotate rx ry deg)` : Rotate an image around point (`rx`,`ry`), `deg` degrees. 
 
Not all attributes are applicable to all drawing functions. The
applicable attributes for each drawing-function is listed in the
description of the function in the reference section. 

## Rendering and color mapping

An image-buffer can be rendered to the display using `disp-render`.
If the image is an `indexed2` or `indexed4` color image, it is when
rendering the image you decide what color each index in the image represents. 

For example you can draw an indexed2 image using a very red and a green color as:
```clj
(disp-render my-img 10 10 '(0x00FF00 0xFF0000))
```

The code above draws `my-img` at location (10,10). Zeroes in the image will be drawn
using color `0x00FF00`, maximum green, and ones will be drawn as `0xFF0000` maximum red.

For rgb332, rgb565 and rgb888 formats the color mapping list is ignored and
can be empty.

More advanced color mapping schemes can be represented using a color type
that is created using the `img-color` or `img-color-setpre` functions. The color
type can represent normal colors, such as 0xFF0000, but can also represent
gradients to be applied to the image in x or y direction upon rendering.

# Displays
 
## sh8501b

Resolution: 194 * 368
Colors: 16Bit
Interface: SPI

Compatible with all image formats supported by the graphics library.


### disp-load-sh8501b

```clj
(disp-load-sh8501b gpio-sd0 gpio-clk gpio-cs gpio-reset spi-mhz)
```

Loads the sh8501b driver. The driver uses hardware-SPI at rate `spi-mhz` on the
`gpio-sd0` and `gpio-clk` GPIO pins.

Example using GPIO pins 6,5,7 and 8 for sd0,clk,cs and reset running the
SPI clock at 40MHz:

```clj
(disp-load-sh8501b 6 5 7 8 40)
``` 

## ili9341

Resolution: 320 * 240
Colors: 16Bit
Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-ili9341

```clj
(disp-load-ili9341 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the ili9341 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
ili9341 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-ili9341 6 5 19 18 7 40)
```


## ssd1306

Resolution: 128 * 64
Colors: B/W  (1bpp)
Interface: I2C

Can display images of `indexed2` format and is limited to displaying
only full screen images starting at position (0, 0). 

### disp-load-ssd1306

```clj
(disp-load-ssd1306 gpio-sda gpio-scl i2c-khz)
```

Load the ssd1306 driver. The ssd1306 talks I2C over the GPIOs
`gpio-sda` (serial data) and `gpio-scl` (clock).

Example using GPIO pins 7 and 6 for serial data and clock. 

```clj
(disp-load-ssd1306 7 6)
```

# Common display operations

## disp-reset

```clj
(disp-reset)
```

Resets and clears the display.

## disp-clear

```clj
(disp-clear)
```

Clears the display by writing zero to all pixel locations.

```clj
(disp-clear color)

Clears the display to the given color. The color is expressed in hex rgb888 format
0xRRGGBB.

## disp-render

```clj
(disp-render img pos-x pos-y colors)
```

## disp-render-jpg



# Graphics library

## img-arc

## img-blit

## img-buffer

## img-buffer-from-bin

## img-circle

## img-circle-sector

## img-circle-segment

## img-clear

## img-color

## img-color-setpre

## img-dims

## img-line

## img-rectangle

## img-setpix

## img-text

## img-triangle

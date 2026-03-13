
# Documentation for display drivers

# Introduction

Drivers for a specific display is loaded with the associated `disp-load-x` function, where x is the name of the driver. Each disp-load function takes arguments specific to the display. These are
explained below.

Each display driver provides a "rendering", "clearing" and "reset"
function that is connected to the LispBM extensions `disp-render`,
`disp-clear` and `disp-reset` upon loading. The render, clear and
reset functions are the display dependent interface that are
implemented per display.

# Displays

## sh8501b

* Resolution: 194 * 368
* Colors: 16Bit
* Interface: SPI

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

## sh8601

* Resolution: 170 * 320
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-sh8601

```clj
(disp-load-sh8601 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the sh8601 driver. The driver uses hardware-SPI at rate `spi-mhz` on the
`gpio-sd0` and `gpio-clk` GPIO pins.

Example using GPIO pins 6,5,7 and 8 for sd0,clk,cs and reset running the
SPI clock at 40MHz and DC on GPIO 9:

```clj
(disp-load-sh8601 6 5 7 8 9 40)
```

## ili9341

* Resolution: 320 * 240
* Colors: 16Bit
* Interface: SPI

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

* Resolution: 128 * 64
* Colors: B/W  (1bpp)
* Interface: I2C

Can display images of `indexed2` format and is limited to displaying
only full screen images starting at position (0, 0).

### disp-load-ssd1306

```clj
(disp-load-ssd1306 gpio-sda gpio-scl i2c-hz)
```

Load the ssd1306 driver. The ssd1306 talks I2C over the GPIOs
`gpio-sda` (serial data) and `gpio-scl` (clock).

Example using GPIO pins 7 and 6 for serial data and clock.

```clj
(disp-load-ssd1306 7 6 700000)
```

## st7789

* Resolution: up to 320 * 240
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-st7789

```clj
(disp-load-st7789 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the st7789 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
st7789 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-st7789 6 5 19 18 7 40)
```

**Note**
Many st7789-based displays do not have the full resolution that the driver supports in the panel. Some of them also have an offset where the panel starts. The panel size and offset has to be taken into account when using disp-render.

## st7735

* Resolution: up to 162 * 132
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-st7735

```clj
(disp-load-st7735 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the st7735 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
st7789 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-st7789 6 5 19 18 7 40)
```

**Note**
Many st7735-based displays do not have the full resolution that the driver supports in the panel. Some of them also have an offset where the panel starts. The panel size and offset has to be taken into account when using disp-render.

## ili9488

* Resolution: 480 ** 320
* Colors: 24Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-ili9488

```clj
(disp-load-ili9488 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the ili9488 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
ili9488 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-ili9488 6 5 19 18 7 40)
```

## ssd1351

* Resolution: 128 * 128
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.

### disp-load-ssd1351

```clj
(disp-load-ssd1351 gpio-sd0 gpio-clk gpio-cs gpio-reset gpio-dc spi-mhz)
```

Loads the ssd1351 driver. The driver uses hardware-SPI at rate
`spi-mhz` on the `gpio-sd0` and `gpio-clk` GPIO pins. In addition, the
ssd1351 uses a data/command signal to discern between commands and
data. The data/command signal is mapped to GPIO `gpio-cs`.

Example using GPIO pins 6,5,19,18 and 7 for sd0,clk,cs,reset and dc.
The SPI clock is set to 40MHz.

```clj
(disp-load-ssd1351 6 5 19 18 7 40)
```

## icna3306

* Resolution: 194 * 368
* Colors: 16Bit
* Interface: SPI

Compatible with all image formats supported by the graphics library.


### disp-load-icna3306

```clj
(disp-load-icna3306 gpio-sd0 gpio-clk gpio-cs gpio-reset spi-mhz)
```

Loads the icna3306 driver. The driver uses hardware-SPI at rate `spi-mhz` on the
`gpio-sd0` and `gpio-clk` GPIO pins.

Example using GPIO pins 6,5,7 and 8 for sd0,clk,cs and reset running the
SPI clock at 40MHz:

```clj
(disp-load-icna3306 6 5 7 8 40)
```

# Documentation for touch library 

The following touch extensions are available:

1. `touch-load-cst816s`
2. `touch-load-gt911`
3. `touch-load-cst9217`
4. `touch-load-xpt2046`
5. `touch-read`
6. `touch-apply-transforms`
7. `touch-delete`

## Quick Start (I2C Controllers)

Use this for CST816S, GT911, and CST9217:

```clj
(touch-load-cst816s sda scl rst int width height [i2c-freq])
```

Arguments:

1. `sda`, `scl`: I2C pins
2. `rst`: reset pin, or `-1` if not used
3. `int`: interrupt pin, or `-1` if not used
4. `width`, `height`: touch panel resolution
5. `i2c-freq` (optional): I2C frequency, default is `400000`

For other I2C controllers, use their matching loader with the same argument pattern:

```clj
(touch-load-gt911 sda scl rst int width height [i2c-freq])
(touch-load-cst9217 sda scl rst int width height [i2c-freq])
```

## Quick Start (SPI - XPT2046)

```clj
(touch-load-xpt2046 spi-host mosi miso sclk cs int width height [spi-freq])
```

`spi-host` is typically `1` (SPI2_HOST) or `2` (SPI3_HOST, if available on your target).

## Reading Touch Data

```clj
(touch-read)
```

Return value:

1. `nil`: no touch point
2. `(x y strength track-id)`: one active touch point

In real projects, use either polling (`touch-read` in a loop) or interrupt events.

## Axis and Orientation Setup

Use this to match touch coordinates to display orientation:

```clj
(touch-apply-transforms swap-xy mirror-x mirror-y)
```

Each argument is `0` or `1`.

Start with:

```clj
(touch-apply-transforms 0 0 0)
```

Then test combinations until touch coordinates match your screen orientation.

This uses esp_lcd_touch transform APIs under the hood, so behavior stays consistent across controllers.

## Touch Events

To receive interrupt-driven touch events:

```clj
(event-register-handler (spawn touch-event-handler))
(event-enable 'event-touch-int)
```

Event format:

```clj
(event-touch-int driver-symbol pressed x y strength track-id)
```

`driver-symbol` is one of: `cst816s`, `gt911`, `cst9217`, `xpt2046`.

## How the CST816S Example Works

The example in `lbm_examples/touch_cst816s.lisp` does three things:

1. Loads the driver with board pins and panel size.
2. Applies coordinate transforms.
3. Shows both usage styles:
   - Polling with `touch-read`
   - Event-driven handling with `event-touch-int`

Inside the event handler, it stores the latest X/Y values and prints only when `pressed` is true. That keeps logs useful instead of flooding the terminal.

## Easy Way to Port a New Touch Driver

The easiest and safest way is to follow the existing structure in `main/touch/lispif_touch_extensions.c`.

1. Add the new ESP-IDF touch component in `main/idf_component.yml`.
2. Add the driver include in `main/touch/lispif_touch_extensions.c`.
3. Add a Lisp symbol (like `sym_cst816s`) and a new `touch-load-*` extension function.
4. Implement init using existing helpers:
   - I2C drivers: follow `touch_init_i2c_esp_lcd(...)`
   - SPI drivers: use `touch_init_xpt2046_esp_lcd(...)` as reference
5. In the new `touch-load-*` function:
   - Validate GPIO, resolution, and bus speed
   - Call `touch_runtime_init()`
   - Call `touch_delete_locked()` before loading a new driver
   - Return `touch_load_driver(...)` on success
6. Register the symbol and extension in `lispif_load_touch_extensions()`.

Why this approach is good:

1. `touch-read` stays unchanged
2. `event-touch-int` stays unchanged
3. `touch-apply-transforms` works immediately
4. Lisp scripts keep a stable API

## Test Checklist

After porting a driver, run this quick test:

1. `touch-load-*`
2. `touch-apply-transforms`
3. `touch-read`
4. `event-touch-int`
5. `touch-delete`

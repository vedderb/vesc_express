## VESC Express LVGL implementation plan

Based on the current `vesc_express` `main`, I would implement this as a **new peer subsystem**:

```text
main/
├── display/          existing physical display drivers
├── touch/            existing touch subsystem
├── lvgl/             NEW: LVGL runtime + LispBM bridge
├── hwconf/           board-specific hardware
├── lispBM/
└── ...
```

No themes, fonts, images, layouts, or dashboard-specific code go into firmware.

Current Express already has dedicated `display/` and `touch/` directories, and the display subsystem explicitly treats each panel driver as hardware implementing render/clear/reset operations.

### Target baseline

For the first implementation I would lock:

- **ESP-IDF 5.5.4** — this is the version current VESC Express explicitly develops against.
- **LVGL 9.5.0** — current stable LVGL release, released February 18, 2026 and supported through February 2027.
- ESP32-S3 first.
- RGB565 rendering initially.
- Partial rendering buffers rather than requiring a full framebuffer.
- Package-owned fonts/assets/themes.

---

# Phase 1 — Add LVGL to the build

### Goal

Get LVGL compiling in VESC Express without changing any display behavior.

### Changes

Add LVGL as an IDF component dependency, pinned to:

```text
9.5.0
```

The Espressif component registry currently provides LVGL 9.5.0 directly.

Create:

```text
main/lvgl/
├── lvgl_runtime.c
├── lvgl_runtime.h
└── lispif_lvgl_extensions.c
```

Update:

```text
main/CMakeLists.txt
main/idf_component.yml
```

Current Express still builds almost everything as one large component through `COMPONENT_SRCS`, so initially we should follow that architecture rather than refactoring the whole project.

### LVGL configuration

Start minimal:

- RGB565
- software rendering
- no demos
- no example widgets
- disable unused image codecs
- disable unused fonts
- no built-in theme requirement
- FreeRTOS
- PSRAM-aware allocations where useful

### Pass condition

Normal Express builds still compile for existing targets.

---

# Phase 2 — Create the LVGL runtime

This becomes the generic engine.

```text
main/lvgl/
├── lvgl_runtime.c
├── lvgl_runtime.h
```

Responsibilities:

- `lv_init()`
- LVGL tick
- LVGL task/timer handler
- mutex/locking
- lifecycle management
- memory statistics
- clean shutdown/restart if necessary

Importantly:

**The LVGL task is owned by Express, not by the VESC package.**

The package only manipulates objects.

Conceptually:

```text
FreeRTOS
   │
   └── LVGL task
          │
          └── lv_timer_handler()
```

### Pass condition

LVGL runs continuously without a display attached and without interfering with:

- LispBM
- CAN
- UART
- BLE/Wi-Fi
- watchdog

---

# Phase 3 — Fix the display abstraction

This is the one architectural change I think is worth making before connecting LVGL.

Currently individual display loaders register their hardware functions directly with LispBM through:

```text
lbm_display_extensions_set_callbacks(...)
```

That pattern is visible throughout `lispif_disp_extensions.c`.

I would introduce:

```text
main/display/display_backend.c
main/display/display_backend.h
```

Its job is simply:

```text
active physical display
```

with operations equivalent to:

```text
render
clear
reset

width
height
pixel format
```

Then the architecture becomes:

```text
                  ┌── existing Lisp graphics
                  │
Display driver ← backend
                  │
                  └── LVGL
```

rather than:

```text
LVGL → Lisp display extension → driver
```

### Existing drivers

We do **not** rewrite all of them immediately.

Convert one first, probably:

```text
disp_axs15231
```

because Express documentation already points to that as the current `esp_lcd` reference implementation.

### Pass condition

An existing Lisp package using:

```text
disp-render
disp-clear
```

works exactly as before.

That is our regression test.

---

# Phase 4 — Add the CO5300

Now add your panel normally:

```text
main/display/
├── disp_co5300.c
└── disp_co5300.h
```

This belongs here because **CO5300 is hardware**.

Ideally use Espressif's `esp_lcd` CO5300 component rather than carrying a custom implementation if its interface supports the Waveshare panel correctly.

The resulting hardware path is:

```text
ESP32-S3
   │
   ▼
QSPI
   │
   ▼
CO5300
   │
   ▼
display_backend
```

### Board hardware configuration

Add a dedicated `hwconf`, for example conceptually:

```text
main/hwconf/.../waveshare_amoled_175/
```

That defines:

- GPIOs
- ESP32-S3 target
- flash configuration
- PSRAM configuration
- board-specific PMIC setup
- display hardware setup

Express already uses `hwconf` plus `HW_INIT_HOOK` / `HW_POST_LISPIF_HOOK`, and those hooks are deliberately called around LispBM initialization.

### Pass condition

Before touching LVGL:

**ordinary `disp-render` must draw onto the CO5300.**

That proves the panel driver independently.

---

# Phase 5 — Connect LVGL to the display

Add:

```text
main/lvgl/lvgl_display.c
main/lvgl/lvgl_display.h
```

Architecture:

```text
LVGL
 │
 ▼
lvgl_display
 │
 ▼
display_backend
 │
 ▼
CO5300
```

`lvgl_display` owns:

- LVGL display instance
- draw buffers
- flush callback
- resolution
- rotation
- color format

### Buffer strategy

Start with partial buffers.

For 466×466 RGB565, a full framebuffer is roughly:

**434 KB.**

We don't need LVGL to permanently consume two of those.

I'd initially test something around:

```text
466 × 40 lines × 2 bytes ≈ 37 KB
```

per buffer.

Potentially:

```text
2 × ~37 KB ≈ 74 KB
```

with DMA-capable buffers.

Then benchmark 20, 40, 80 and full-height buffering later.

### Pass condition

Firmware-generated LVGL test object:

```text
Hello LVGL
```

appears on the physical AMOLED.

No LispBM yet.

---

# Phase 6 — Connect Express touch to LVGL

This should be relatively straightforward because current Express already exposes:

- CST816S
- GT911
- **CST9217**
- CST836U
- XPT2046

behind a common touch API.

Add:

```text
main/lvgl/lvgl_input.c
main/lvgl/lvgl_input.h
```

Architecture:

```text
CST9217
   │
   ▼
Express touch subsystem
   │
   ▼
lvgl_input
   │
   ▼
LVGL indev
```

Do **not** create an LVGL-specific CST9217 driver.

The current Express touch API already supports coordinate transforms and CST9217 specifically.

### Pass condition

Firmware test UI:

```text
┌────────────┐
│   BUTTON   │
└────────────┘
```

Touching the button changes its state.

At this point we have a completely functioning native LVGL Express firmware.

---

# Phase 7 — LispBM → LVGL bridge

Now add the important part:

```text
main/lvgl/
├── lvgl_objects.c
├── lvgl_objects.h
├── lispif_lvgl_extensions.c
└── lispif_lvgl_extensions.h
```

Express already supports additional extension-loader callbacks after its core Lisp extensions are loaded, so the LVGL bridge can integrate with the existing LispBM extension architecture rather than hacking it into the interpreter.

## Do not expose pointers

Never give Lisp:

```text
lv_obj_t *
```

Use handles:

```text
1
2
3
4
```

Internally:

```text
LVGL handle table

1 → lv_obj_t *
2 → lv_obj_t *
3 → lv_style_t *
```

That gives us:

- type checking
- stale-handle detection
- cleanup when a package terminates
- safer error handling
- future LVGL-version independence

---

# Phase 8 — Minimal Lisp API

Don't try to wrap LVGL's entire API.

The first package only needs enough to prove the model.

### Runtime

```text
lv-init
lv-ready?
lv-screen-width
lv-screen-height
```

### Objects

```text
lv-screen-create
lv-screen-load

lv-label-create
lv-button-create
lv-arc-create
lv-bar-create
lv-container-create
```

### Common properties

```text
lv-obj-pos
lv-obj-size
lv-obj-align

lv-label-text
lv-arc-value
lv-bar-value
```

### Styling

Basic runtime styling:

```text
lv-obj-bg-color
lv-obj-text-color
lv-obj-opacity
lv-obj-border-color
lv-obj-radius
```

### Lifecycle

```text
lv-obj-delete
lv-screen-delete
lv-clean
```

### Pass condition

A Lisp script creates:

```text
screen
  ├── label
  ├── arc
  └── button
```

without any UI defined in C.

---

# Phase 9 — Package resources

Only after basic objects work.

Add:

```text
main/lvgl/
├── lvgl_resources.c
└── lvgl_resources.h
```

Its job is **loading resources**, not storing them.

The package owns:

```text
test_lvgl.vescpkg
├── main.lisp
└── assets/
    ├── font.bin
    └── icon.bin
```

VESC packages already support Lisp scripts with included/imported files and compiled libraries.

First implementation:

```text
package import
      │
      ▼
LispBM binary data
      │
      ▼
lvgl_resources
      │
      ▼
LVGL
```

Then benchmark whether this creates undesirable copies.

**Do not modify the `.vescpkg` format initially.**

---

# Phase 10 — Package fonts

This is the first resource feature I'd implement.

Firmware contains:

**zero dashboard fonts.**

Package contains:

```text
fonts/
├── speed.bin
├── normal.bin
└── small.bin
```

The bridge provides something conceptually equivalent to:

```text
load font
assign font
release font
```

Then our test screen can prove that updating the `.vescpkg` changes typography without reflashing Express.

---

# Phase 11 — Package images

After fonts work:

```text
images/
├── warning.bin
├── bluetooth.bin
└── battery.bin
```

Initial formats should be intentionally limited.

I'd start with:

- native LVGL binary image
- RGB565
- optional alpha

PNG/JPEG can come later.

---

# Phase 12 — Events

Then expose LVGL events to LispBM.

Conceptually:

```text
LVGL event
    │
    ▼
Express LVGL bridge
    │
    ▼
LispBM event queue
```

Events such as:

```text
pressed
released
clicked
value-changed
long-pressed
gesture
```

The important rule is that **LVGL's task should never execute Lisp directly from an LVGL callback**.

Queue the event instead.

That keeps task ownership sane.

---

# Phase 13 — First actual `.vescpkg`

Now we build the test package.

I suggest making the first package deliberately simple:

```text
VESC LVGL Test
```

Screen:

```text
┌───────────────────────────┐
│        LVGL / VESC        │
│                           │
│            42             │
│         ╭──────╮          │
│        ╱        ╲         │
│       │   ARC    │        │
│        ╲        ╱         │
│         ╰──────╯          │
│                           │
│      [ TOUCH ME ]         │
└───────────────────────────┘
```

It should test:

1. Screen creation
2. Label creation
3. Font loaded from package
4. Arc
5. Button
6. Touch
7. Lisp event
8. Dynamically changing label
9. Dynamically changing arc
10. Clean package unload/reload

No motor telemetry yet.

That isolates LVGL/package functionality.

---

# Phase 14 — VESC telemetry test

Once that works, make package version 2 actually query VESC data.

Something like:

```text
          43
         km/h

      ╭────────╮
     ╱          ╲
    │    72%     │
     ╲          ╱
      ╰────────╯

  81.4 V     34 A
```

The package updates:

- speed
- voltage
- current
- controller temperature
- motor temperature
- battery percentage

Then we've proven the original concept:

```text
Generic VESC Express firmware

            +

Installable VESC UI package

            =

Fully programmable VESC display
```

---

# Final directory target

I'd aim for this:

```text
vesc_express/main/

├── display/
│   ├── display_backend.c
│   ├── display_backend.h
│   ├── disp_co5300.c
│   ├── disp_co5300.h
│   └── existing drivers...
│
├── touch/
│   └── existing Express touch system
│
├── lvgl/
│   ├── lvgl_runtime.c
│   ├── lvgl_runtime.h
│   ├── lvgl_display.c
│   ├── lvgl_display.h
│   ├── lvgl_input.c
│   ├── lvgl_input.h
│   ├── lvgl_objects.c
│   ├── lvgl_objects.h
│   ├── lvgl_resources.c
│   ├── lvgl_resources.h
│   ├── lispif_lvgl_extensions.c
│   └── lispif_lvgl_extensions.h
│
└── hwconf/
    └── ...
```

And externally:

```text
vesc_pkg/

lvgl_test/
├── main.lisp
├── assets/
│   ├── test_font.bin
│   └── test_icon.bin
└── lvgl_test.vescpkg
```

## Implementation order I recommend

The key is **not to start with the Lisp API**.

The sequence should be:

**LVGL builds → CO5300 works → LVGL renders → touch works → Lisp controls LVGL → package assets work → test `.vescpkg`.**

That gives us a working checkpoint after every layer and makes debugging much easier.

There is also a useful upstream precedent: PR #115, merged August 1, added the JD9165/display and multitouch work into `main`, so the present Express codebase is already moving toward more capable display hardware rather than treating displays as an external hack.

The **first actual coding milestone** should therefore be Phases 1–5: add LVGL 9.5.0, add the backend abstraction, add CO5300, and get a firmware-generated LVGL label onto the 466×466 panel. Once that boots reliably, we move into the Lisp/package side.

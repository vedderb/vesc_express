/*
	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "disp_st7701.h"

#if CONFIG_IDF_TARGET_ESP32P4

#include <string.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7701.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "soc/soc_caps.h"

#define LCD_H_RES 480
#define LCD_V_RES 800

#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
#define LCD_NUM_FBS 2
#define WAIT_FLUSH_DONE 0

static const char *TAG = "disp_st7701";

typedef struct {
    int width;
    int height;
    esp_lcd_panel_handle_t panel;
    esp_lcd_dsi_bus_handle_t dsi_bus;
    esp_lcd_panel_io_handle_t io;
    esp_ldo_channel_handle_t ldo_chan;
    SemaphoreHandle_t refresh_finish;
} st7701_jc4880_ctx_t;

static st7701_jc4880_ctx_t m_ctx = {
    .width = LCD_H_RES,
    .height = LCD_V_RES,
};

static int m_pin_rst = -1;
static int m_lane_mbps = 1300;
static int m_rotation = 0; // 0=0°, 1=90°CW, 2=180°, 3=270°CW
static bool m_orientation_registered = false;

#if SOC_MIPI_DSI_SUPPORTED
static const st7701_lcd_init_cmd_t vendor_specific_init_default[] = {
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, (uint8_t []){0x08}, 1, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t []){0x63, 0x00}, 2, 0},
    {0xC1, (uint8_t []){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t []){0x10, 0x08}, 2, 0},
    {0xCC, (uint8_t []){0x10}, 1, 0},

    {0xB0, (uint8_t []){0x80, 0x09, 0x53, 0x0C, 0xD0, 0x07, 0x0C, 0x09, 0x09, 0x28, 0x06, 0xD4, 0x13, 0x69, 0x2B, 0x71}, 16, 0},
    {0xB1, (uint8_t []){0x80, 0x94, 0x5A, 0x10, 0xD3, 0x06, 0x0A, 0x08, 0x08, 0x25, 0x03, 0xD3, 0x12, 0x66, 0x6A, 0x0D}, 16, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},

    {0xB0, (uint8_t []){0x5D}, 1, 0},
    {0xB1, (uint8_t []){0x58}, 1, 0},
    {0xB2, (uint8_t []){0x87}, 1, 0},
    {0xB3, (uint8_t []){0x80}, 1, 0},
    {0xB5, (uint8_t []){0x4E}, 1, 0},
    {0xB7, (uint8_t []){0x85}, 1, 0},
    {0xB8, (uint8_t []){0x21}, 1, 0},
    {0xB9, (uint8_t []){0x10, 0x1F}, 2, 0},
    {0xBB, (uint8_t []){0x03}, 1, 0},
    {0xBC, (uint8_t []){0x00}, 1, 0},

    {0xC1, (uint8_t []){0x78}, 1, 0},
    {0xC2, (uint8_t []){0x78}, 1, 0},
    {0xD0, (uint8_t []){0x88}, 1, 0},

    {0xE0, (uint8_t []){0x00, 0x3A, 0x02}, 3, 0},
    {0xE1, (uint8_t []){0x04, 0xA0, 0x00, 0xA0, 0x05, 0xA0, 0x00, 0xA0, 0x00, 0x40, 0x40}, 11, 0},
    {0xE2, (uint8_t []){0x30, 0x00, 0x40, 0x40, 0x32, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00}, 13, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t []){0x09, 0x2E, 0xA0, 0xA0, 0x0B, 0x30, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0}, 16, 0},
    {0xE6, (uint8_t []){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t []){0x08, 0x2D, 0xA0, 0xA0, 0x0A, 0x2F, 0xA0, 0xA0, 0x04, 0x29, 0xA0, 0xA0, 0x06, 0x2B, 0xA0, 0xA0}, 16, 0},

    {0xEB, (uint8_t []){0x00, 0x00, 0x4E, 0x4E, 0x00, 0x00, 0x00}, 7, 0},
    {0xEC, (uint8_t []){0x08, 0x01}, 2, 0},

    {0xED, (uint8_t []){0xB0, 0x2B, 0x98, 0xA4, 0x56, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x65, 0x4A, 0x89, 0xB2, 0x0B}, 16, 0},
    {0xEF, (uint8_t []){0x08, 0x08, 0x08, 0x45, 0x3F, 0x54}, 6, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},

    {0x11, (uint8_t []){0x00}, 1, 120},
    {0x29, (uint8_t []){0x00}, 1, 20},
};

IRAM_ATTR static bool notify_refresh_ready(esp_lcd_panel_handle_t panel, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
    (void)panel;
    (void)edata;

    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_ctx;
    BaseType_t need_yield = pdFALSE;
    xSemaphoreGiveFromISR(sem, &need_yield);
    return need_yield == pdTRUE;
}
#endif

static inline uint16_t rgb888_to_rgb565(uint32_t rgb) {
    uint16_t r = (uint16_t)((rgb >> 19) & 0x1F);
    uint16_t g = (uint16_t)((rgb >> 10) & 0x3F);
    uint16_t b = (uint16_t)((rgb >> 3) & 0x1F);
    return (uint16_t)((r << 11) | (g << 5) | b);
}

static void rotate_rgb565(const uint16_t *src, uint16_t *dst, int w, int h, int rotation) {
    if (rotation == 1) { // 90° CW: dst is h×w
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                dst[(uint32_t)x * h + (h - 1 - y)] = src[(uint32_t)y * w + x];
            }
        }
    } else if (rotation == 2) { // 180°
        int total = w * h;
        for (int i = 0; i < total; i++) {
            dst[total - 1 - i] = src[i];
        }
    } else if (rotation == 3) { // 270° CW: dst is h×w
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                dst[(uint32_t)(w - 1 - x) * h + y] = src[(uint32_t)y * w + x];
            }
        }
    }
}

static bool convert_to_rgb565(const image_buffer_t *img, color_t *colors, uint16_t *dst) {
    uint32_t num_pix = (uint32_t)img->width * (uint32_t)img->height;

    switch (img->fmt) {
    case indexed2:
        if (!colors) {
            return false;
        }
        for (uint32_t i = 0; i < num_pix; i++) {
            uint32_t byte = i >> 3;
            uint32_t bit = 7U - (i & 0x7U);
            uint32_t color_ind = (img->data[byte] >> bit) & 0x1U;
            uint32_t rgb = COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);
            dst[i] = rgb888_to_rgb565(rgb);
        }
        return true;

    case indexed4:
        if (!colors) {
            return false;
        }
        for (uint32_t i = 0; i < num_pix; i++) {
            uint32_t byte = i >> 2;
            uint32_t bit = (3U - (i & 0x3U)) * 2U;
            uint32_t color_ind = (img->data[byte] >> bit) & 0x3U;
            uint32_t rgb = COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);
            dst[i] = rgb888_to_rgb565(rgb);
        }
        return true;

    case indexed16:
        if (!colors) {
            return false;
        }
        for (uint32_t i = 0; i < num_pix; i++) {
            uint32_t byte = i >> 1;
            uint32_t bit = (1U - (i & 0x1U)) * 4U;
            uint32_t color_ind = (img->data[byte] >> bit) & 0xFU;
            uint32_t rgb = COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width);
            dst[i] = rgb888_to_rgb565(rgb);
        }
        return true;

    case rgb332:
        for (uint32_t i = 0; i < num_pix; i++) {
            uint8_t pix = img->data[i];
            uint32_t r = (uint32_t)((pix >> 5) & 0x7U);
            uint32_t g = (uint32_t)((pix >> 2) & 0x7U);
            uint32_t b = (uint32_t)(pix & 0x3U);
            uint32_t rgb = (r << 21) | (g << 13) | (b << 6);
            dst[i] = rgb888_to_rgb565(rgb);
        }
        return true;

    case rgb565:
        for (uint32_t i = 0; i < num_pix; i++) {
            dst[i] = (uint16_t)(((uint16_t)img->data[2 * i] << 8) | (uint16_t)img->data[2 * i + 1]);
        }
        return true;

    case rgb888:
        for (uint32_t i = 0; i < num_pix; i++) {
            uint32_t rgb = ((uint32_t)img->data[3 * i] << 16) |
                           ((uint32_t)img->data[3 * i + 1] << 8) |
                           (uint32_t)img->data[3 * i + 2];
            dst[i] = rgb888_to_rgb565(rgb);
        }
        return true;

    default:
        return false;
    }
}

bool disp_st7701_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
#if !SOC_MIPI_DSI_SUPPORTED
    (void)img;
    (void)x;
    (void)y;
    (void)colors;
    return false;
#else
    st7701_jc4880_ctx_t *ctx = &m_ctx;

    if (!ctx->panel || !img || !img->data) {
        return false;
    }

    uint32_t num_pix = (uint32_t)img->width * (uint32_t)img->height;
    uint16_t *frame = heap_caps_malloc(num_pix * sizeof(uint16_t), MALLOC_CAP_8BIT);
    if (!frame) {
        return false;
    }

    bool ok = convert_to_rgb565(img, colors, frame);
    if (!ok) {
        free(frame);
        return false;
    }

    int px, py, pw, ph;
    if (m_rotation == 0) {
        px = x; py = y;
        pw = img->width; ph = img->height;
        if ((uint32_t)px + (uint32_t)pw > (uint32_t)ctx->width ||
            (uint32_t)py + (uint32_t)ph > (uint32_t)ctx->height) {
            free(frame);
            return false;
        }
    } else if (m_rotation == 1) { // 90° CW: logical(x,y) -> physical(H-y-lh, x)
        px = LCD_H_RES - (int)y - (int)img->height;
        py = (int)x;
        pw = img->height; ph = img->width;
        if (px < 0 || py < 0 || px + pw > LCD_H_RES || py + ph > LCD_V_RES) {
            free(frame);
            return false;
        }
    } else if (m_rotation == 2) { // 180°
        px = LCD_H_RES - (int)x - (int)img->width;
        py = LCD_V_RES - (int)y - (int)img->height;
        pw = img->width; ph = img->height;
        if (px < 0 || py < 0 || px + pw > LCD_H_RES || py + ph > LCD_V_RES) {
            free(frame);
            return false;
        }
    } else { // 270° CW: logical(x,y) -> physical(y, W-x-lw)
        px = (int)y;
        py = LCD_V_RES - (int)x - (int)img->width;
        pw = img->height; ph = img->width;
        if (px < 0 || py < 0 || px + pw > LCD_H_RES || py + ph > LCD_V_RES) {
            free(frame);
            return false;
        }
    }

    uint16_t *send_buf = frame;
    uint16_t *rotated = NULL;
    if (m_rotation != 0) {
        rotated = heap_caps_malloc(num_pix * sizeof(uint16_t), MALLOC_CAP_8BIT);
        if (!rotated) {
            free(frame);
            return false;
        }
        rotate_rgb565(frame, rotated, img->width, img->height, m_rotation);
        send_buf = rotated;
    }

    if (ctx->refresh_finish) {
        xSemaphoreTake(ctx->refresh_finish, 0);
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(ctx->panel, px, py, px + pw, py + ph, send_buf);
    if (err == ESP_OK && ctx->refresh_finish) {
        xSemaphoreTake(ctx->refresh_finish, pdMS_TO_TICKS(1000));
    }

    free(frame);
    if (rotated) { free(rotated); }
    return err == ESP_OK;
#endif
}

void disp_st7701_clear(uint32_t color) {
#if !SOC_MIPI_DSI_SUPPORTED
    (void)color;
    return;
#else
    st7701_jc4880_ctx_t *ctx = &m_ctx;
    if (!ctx->panel) {
        return;
    }

    uint16_t c565 = rgb888_to_rgb565(color);
    const int rows_per_chunk = 16;
    uint16_t *chunk = heap_caps_malloc((size_t)ctx->width * rows_per_chunk * sizeof(uint16_t), MALLOC_CAP_8BIT);
    if (!chunk) {
        return;
    }

    for (int i = 0; i < (ctx->width * rows_per_chunk); i++) {
        chunk[i] = c565;
    }

    for (int row = 0; row < ctx->height; row += rows_per_chunk) {
        int h = (ctx->height - row > rows_per_chunk) ? rows_per_chunk : (ctx->height - row);

        if (ctx->refresh_finish) {
            xSemaphoreTake(ctx->refresh_finish, 0);
        }

        if (esp_lcd_panel_draw_bitmap(ctx->panel, 0, row, ctx->width, row + h, chunk) != ESP_OK) {
            break;
        }

        if (ctx->refresh_finish) {
            xSemaphoreTake(ctx->refresh_finish, pdMS_TO_TICKS(1000));
        }
    }

    free(chunk);
#endif
}

static void disp_st7701_init_internal(void);

void disp_st7701_reset(void) {
    disp_st7701_deinit();
    disp_st7701_init_internal();
}

lbm_value disp_st7701_ext_orientation(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_ARGN_NUMBER(1);
    int rot = lbm_dec_as_i32(args[0]);
    if (rot < 0 || rot > 3) {
        lbm_set_error_reason("Orientation must be 0, 1, 2 or 3");
        return ENC_SYM_EERROR;
    }

    m_rotation = rot;
    return ENC_SYM_TRUE;
}

void disp_st7701_init(int pin_rst, int lane_mbps) {
    m_pin_rst = pin_rst;
    m_lane_mbps = lane_mbps;
    
    disp_st7701_init_internal();

    lbm_add_extension("ext-disp-orientation", disp_st7701_ext_orientation);
}

static void disp_st7701_init_internal(void) {
#if !SOC_MIPI_DSI_SUPPORTED
    return;
#else
    st7701_jc4880_ctx_t *ctx = &m_ctx;

    if (ctx->panel) {
        return;
    }

    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &ctx->ldo_chan));

    esp_lcd_dsi_bus_config_t bus_cfg = ST7701_PANEL_BUS_DSI_2CH_CONFIG();
    bus_cfg.lane_bit_rate_mbps = (uint32_t)m_lane_mbps;
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &ctx->dsi_bus));

    esp_lcd_dbi_io_config_t dbi_cfg = ST7701_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(ctx->dsi_bus, &dbi_cfg, &ctx->io));

    esp_lcd_dpi_panel_config_t dpi_cfg = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 34,
        .virtual_channel = 0,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
#else
        .in_color_format = LCD_COLOR_FMT_RGB565,
#endif
        .num_fbs = LCD_NUM_FBS,
        .video_timing = {
            .h_size = LCD_H_RES,
            .v_size = LCD_V_RES,
            .hsync_back_porch = 42,
            .hsync_pulse_width = 12,
            .hsync_front_porch = 42,
            .vsync_back_porch = 8,
            .vsync_pulse_width = 2,
            .vsync_front_porch = 166,
        },
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
        .flags.use_dma2d = true,
#endif
    };
    dpi_cfg.video_timing.h_size = LCD_H_RES;
    dpi_cfg.video_timing.v_size = LCD_V_RES;

    st7701_vendor_config_t vendor_cfg = {
        .init_cmds = vendor_specific_init_default,
        .init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(vendor_specific_init_default[0]),
        .flags.use_mipi_interface = 1,
        .mipi_config = {
            .dsi_bus = ctx->dsi_bus,
            .dpi_config = &dpi_cfg,
        },
    };

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = m_pin_rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(ctx->io, &panel_cfg, &ctx->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(ctx->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(ctx->panel));

    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_enable_dma2d(ctx->panel));
    #endif

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(ctx->panel, true));

    ctx->refresh_finish = xSemaphoreCreateBinary();
    if (ctx->refresh_finish) {
        esp_lcd_dpi_panel_event_callbacks_t cbs = {
            .on_color_trans_done = notify_refresh_ready,
        };
        ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(ctx->panel, &cbs, ctx->refresh_finish));
    }

    ESP_LOGI(TAG, "ST7701 display initialized");
#endif
}

void disp_st7701_deinit(void) {
#if !SOC_MIPI_DSI_SUPPORTED
    return;
#else
    st7701_jc4880_ctx_t *ctx = &m_ctx;

    if (ctx->refresh_finish) {
        vSemaphoreDelete(ctx->refresh_finish);
        ctx->refresh_finish = NULL;
    }

    if (ctx->panel) {
        esp_lcd_panel_del(ctx->panel);
        ctx->panel = NULL;
    }

    if (ctx->io) {
        esp_lcd_panel_io_del(ctx->io);
        ctx->io = NULL;
    }

    if (ctx->dsi_bus) {
        esp_lcd_del_dsi_bus(ctx->dsi_bus);
        ctx->dsi_bus = NULL;
    }

    if (ctx->ldo_chan) {
        esp_ldo_release_channel(ctx->ldo_chan);
        ctx->ldo_chan = NULL;
    }

#endif
}

#endif

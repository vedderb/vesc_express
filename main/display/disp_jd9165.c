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

#include "disp_jd9165.h"

#include "soc/soc_caps.h"

#if SOC_MIPI_DSI_SUPPORTED

#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_lcd_jd9165.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "disp_jd9165"

#define DISPLAY_WIDTH_PHYS   1024
#define DISPLAY_HEIGHT_PHYS  600
#define FRAME_PIXELS         (DISPLAY_WIDTH_PHYS * DISPLAY_HEIGHT_PHYS)
#define FRAME_BUF_BYTES      (FRAME_PIXELS * 2)
#define MIPI_DSI_PHY_PWR_LDO_CHAN 3
#define MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500

static const jd9165_lcd_init_cmd_t vendor_specific_init_default[] = {
    // {cmd, { data }, data_size, delay_ms}
    {0x30, (uint8_t[]){0x00}, 1, 0},
    {0xF7, (uint8_t[]){0x49, 0x61, 0x02, 0x00}, 4, 0},
    {0x30, (uint8_t[]){0x01}, 1, 0},
    {0x04, (uint8_t[]){0x0C}, 1, 0},
    {0x05, (uint8_t[]){0x00}, 1, 0},
    {0x06, (uint8_t[]){0x00}, 1, 0},
    {0x0B, (uint8_t[]){0x11}, 1, 0},
    {0x17, (uint8_t[]){0x00}, 1, 0},
    {0x20, (uint8_t[]){0x04}, 1, 0},
    {0x1F, (uint8_t[]){0x05}, 1, 0},
    {0x23, (uint8_t[]){0x00}, 1, 0},
    {0x25, (uint8_t[]){0x19}, 1, 0},
    {0x28, (uint8_t[]){0x18}, 1, 0},
    {0x29, (uint8_t[]){0x04}, 1, 0},
    {0x2A, (uint8_t[]){0x01}, 1, 0},
    {0x2B, (uint8_t[]){0x04}, 1, 0},
    {0x2C, (uint8_t[]){0x01}, 1, 0},
    {0x30, (uint8_t[]){0x02}, 1, 0},
    {0x01, (uint8_t[]){0x22}, 1, 0},
    {0x03, (uint8_t[]){0x12}, 1, 0},
    {0x04, (uint8_t[]){0x00}, 1, 0},
    {0x05, (uint8_t[]){0x64}, 1, 0},
    {0x0A, (uint8_t[]){0x08}, 1, 0},
    {0x0B, (uint8_t[]){0x0A, 0x1A, 0x0B, 0x0D, 0x0D, 0x11, 0x10, 0x06, 0x08, 0x1F, 0x1D}, 11, 0},
    {0x0C, (uint8_t[]){0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x0D, (uint8_t[]){0x16, 0x1B, 0x0B, 0x0D, 0x0D, 0x11, 0x10, 0x07, 0x09, 0x1E, 0x1C}, 11, 0},
    {0x0E, (uint8_t[]){0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x0F, (uint8_t[]){0x16, 0x1B, 0x0D, 0x0B, 0x0D, 0x11, 0x10, 0x1C, 0x1E, 0x09, 0x07}, 11, 0},
    {0x10, (uint8_t[]){0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x11, (uint8_t[]){0x0A, 0x1A, 0x0D, 0x0B, 0x0D, 0x11, 0x10, 0x1D, 0x1F, 0x08, 0x06}, 11, 0},
    {0x12, (uint8_t[]){0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D}, 11, 0},
    {0x14, (uint8_t[]){0x00, 0x00, 0x11, 0x11}, 4, 0},
    {0x18, (uint8_t[]){0x99}, 1, 0},
    {0x30, (uint8_t[]){0x06}, 1, 0},
    {0x12, (uint8_t[]){0x36, 0x2C, 0x2E, 0x3C, 0x38, 0x35, 0x35, 0x32, 0x2E, 0x1D, 0x2B, 0x21, 0x16, 0x29}, 14, 0},
    {0x13, (uint8_t[]){0x36, 0x2C, 0x2E, 0x3C, 0x38, 0x35, 0x35, 0x32, 0x2E, 0x1D, 0x2B, 0x21, 0x16, 0x29}, 14, 0},
    {0x30, (uint8_t[]){0x0A}, 1, 0},
    {0x02, (uint8_t[]){0x4F}, 1, 0},
    {0x0B, (uint8_t[]){0x40}, 1, 0},
    {0x12, (uint8_t[]){0x3E}, 1, 0},
    {0x13, (uint8_t[]){0x78}, 1, 0},
    {0x30, (uint8_t[]){0x0D}, 1, 0},
    {0x0D, (uint8_t[]){0x04}, 1, 0},
    {0x10, (uint8_t[]){0x0C}, 1, 0},
    {0x11, (uint8_t[]){0x0C}, 1, 0},
    {0x12, (uint8_t[]){0x0C}, 1, 0},
    {0x13, (uint8_t[]){0x0C}, 1, 0},
    {0x30, (uint8_t[]){0x00}, 1, 0},
    {0x11, (uint8_t[]){0x00}, 1, 120},
    {0x29, (uint8_t[]){0x00}, 1, 50},
};

typedef struct {
    esp_lcd_panel_handle_t panel;
    esp_lcd_dsi_bus_handle_t dsi_bus;
    esp_lcd_panel_io_handle_t io;
    esp_ldo_channel_handle_t ldo_chan;
    uint16_t *frame_a;
    uint16_t *front;
} jd9165_ctx_t;

static jd9165_ctx_t m_ctx = {0};
static int m_pin_rst = -1;
static int m_lane_mbps = 750;

static inline uint16_t rgb888_to_rgb565(uint32_t rgb) {
    uint16_t r = (uint16_t)((rgb >> 19) & 0x1F);
    uint16_t g = (uint16_t)((rgb >> 10) & 0x3F);
    uint16_t b = (uint16_t)((rgb >> 3) & 0x1F);
    return (uint16_t)((r << 11) | (g << 5) | b);
}

static inline uint32_t pixel_to_rgb888(image_buffer_t *img, int x, int y, color_t *colors) {
    if (img->fmt == indexed2 || img->fmt == indexed4 || img->fmt == indexed16) {
        if (!colors) {
            return 0;
        }

        uint32_t ci = getpixel(img, x, y);
        return COLOR_TO_RGB888(colors[ci], x, y);
    }

    return getpixel(img, x, y);
}

static void disp_jd9165_deinit(void);
static void disp_jd9165_init_internal(void);

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_NUMBER_ALL();

    if (!m_ctx.io) {
        return ENC_SYM_EERROR;
    }

    if (argn == 0) {
        return ENC_SYM_TERROR;
    }

    uint8_t paras[32];
    int n = 0;
    uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);

    if (argn > 1) {
        n = (int)argn - 1;
        if (n > (int)sizeof(paras)) {
            n = (int)sizeof(paras);
        }

        for (int i = 0; i < n; i++) {
            paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
        }
    }

    esp_err_t err = esp_lcd_panel_io_tx_param(m_ctx.io, cmd, n > 0 ? paras : NULL, n);
    return (err == ESP_OK) ? ENC_SYM_TRUE : ENC_SYM_EERROR;
}

static lbm_value ext_disp_orientation(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_ARGN_NUMBER(1);

    if (!m_ctx.panel) {
        return ENC_SYM_EERROR;
    }

    uint32_t orientation = lbm_dec_as_u32(args[0]);
    bool mirror_x = false;
    bool mirror_y = false;

    switch (orientation) {
    case 0:
        break;
    case 1:
        mirror_x = true;
        break;
    case 2:
        mirror_y = true;
        break;
    case 3:
        mirror_x = true;
        mirror_y = true;
        break;
    default:
        return ENC_SYM_EERROR;
    }

    if (esp_lcd_panel_mirror(m_ctx.panel, mirror_x, mirror_y) != ESP_OK) {
        return ENC_SYM_EERROR;
    }

    return ENC_SYM_TRUE;
}

bool disp_jd9165_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
    if (!m_ctx.panel || !m_ctx.front) {
        return false;
    }

    if ((uint32_t)x + (uint32_t)img->width > DISPLAY_WIDTH_PHYS ||
            (uint32_t)y + (uint32_t)img->height > DISPLAY_HEIGHT_PHYS) {
        return false;
    }

    // Fast path: Lisp scene buffers are RGB565 and already packed for direct DMA.
    if (img->fmt == rgb565 && !colors) {
        return esp_lcd_panel_draw_bitmap(m_ctx.panel, x, y,
                x + img->width, y + img->height, img->data) == ESP_OK;
    }

    if (img->fmt == rgb565) {
        for (int yy = 0; yy < img->height; yy++) {
            const uint8_t *src = img->data + ((size_t)yy * (size_t)img->width * 2U);
            uint16_t *dst = m_ctx.front + (size_t)(y + yy) * DISPLAY_WIDTH_PHYS + x;
            for (int xx = 0; xx < img->width; xx++) {
                uint32_t pos = (uint32_t)xx * 2U;
                dst[xx] = (uint16_t)(((uint16_t)src[pos] << 8) | (uint16_t)src[pos + 1]);
            }
        }
    } else {
        for (int yy = 0; yy < img->height; yy++) {
            uint16_t *dst = m_ctx.front + (size_t)(y + yy) * DISPLAY_WIDTH_PHYS + x;
            for (int xx = 0; xx < img->width; xx++) {
                uint32_t rgb = pixel_to_rgb888(img, xx, yy, colors);
                dst[xx] = rgb888_to_rgb565(rgb);
            }
        }
    }

    if (esp_lcd_panel_draw_bitmap(m_ctx.panel, x, y,
            x + img->width, y + img->height,
            m_ctx.front + ((size_t)y * DISPLAY_WIDTH_PHYS + x)) != ESP_OK) {
        return false;
    }

    return true;
}

void disp_jd9165_clear(uint32_t color) {
    if (!m_ctx.panel || !m_ctx.front) {
        return;
    }

    uint16_t clear_color = rgb888_to_rgb565(color);
    for (size_t i = 0; i < FRAME_PIXELS; i++) {
        m_ctx.front[i] = clear_color;
    }

    esp_lcd_panel_draw_bitmap(m_ctx.panel, 0, 0,
            DISPLAY_WIDTH_PHYS, DISPLAY_HEIGHT_PHYS, m_ctx.front);
}

static void disp_jd9165_init_internal(void) {
    if (m_ctx.panel) {
        return;
    }

    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_cfg, &m_ctx.ldo_chan));

    esp_lcd_dsi_bus_config_t bus_cfg = JD9165_PANEL_BUS_DSI_2CH_CONFIG();
    bus_cfg.lane_bit_rate_mbps = (uint32_t)m_lane_mbps;
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &m_ctx.dsi_bus));

    esp_lcd_dbi_io_config_t dbi_cfg = JD9165_PANEL_IO_DBI_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(m_ctx.dsi_bus, &dbi_cfg, &m_ctx.io));

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
    esp_lcd_dpi_panel_config_t dpi_cfg = JD9165_1024_600_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
#else
    esp_lcd_dpi_panel_config_t dpi_cfg = JD9165_1024_600_PANEL_60HZ_DPI_CONFIG_CF(LCD_COLOR_FMT_RGB565);
#endif

    jd9165_vendor_config_t vendor_cfg = {
        .init_cmds = vendor_specific_init_default,
        .init_cmds_size = sizeof(vendor_specific_init_default) / sizeof(vendor_specific_init_default[0]),
        .mipi_config = {
            .dsi_bus = m_ctx.dsi_bus,
            .dpi_config = &dpi_cfg,
        },
    };

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = m_pin_rst,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9165(m_ctx.io, &panel_cfg, &m_ctx.panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(m_ctx.panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(m_ctx.panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(m_ctx.panel, true));

    if (!m_ctx.frame_a) {
        m_ctx.frame_a = (uint16_t *)heap_caps_malloc(FRAME_BUF_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

        if (!m_ctx.frame_a) {
            ESP_LOGE(TAG, "Framebuffer allocation failed");
            ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
        }

        memset(m_ctx.frame_a, 0, FRAME_BUF_BYTES);
        m_ctx.front = m_ctx.frame_a;
    }

    ESP_LOGI(TAG, "JD9165 display initialized");
}

static void disp_jd9165_deinit(void) {
    if (m_ctx.frame_a) {
        free(m_ctx.frame_a);
        m_ctx.frame_a = NULL;
    }

    m_ctx.front = NULL;

    if (m_ctx.panel) {
        esp_lcd_panel_del(m_ctx.panel);
        m_ctx.panel = NULL;
    }

    if (m_ctx.io) {
        esp_lcd_panel_io_del(m_ctx.io);
        m_ctx.io = NULL;
    }

    if (m_ctx.dsi_bus) {
        esp_lcd_del_dsi_bus(m_ctx.dsi_bus);
        m_ctx.dsi_bus = NULL;
    }

    if (m_ctx.ldo_chan) {
        esp_ldo_release_channel(m_ctx.ldo_chan);
        m_ctx.ldo_chan = NULL;
    }
}

void disp_jd9165_reset(void) {
    disp_jd9165_deinit();
    disp_jd9165_init_internal();
}

void disp_jd9165_init(int pin_rst, int lane_mbps) {
    m_pin_rst = pin_rst;
    m_lane_mbps = lane_mbps;

    disp_jd9165_init_internal();

    lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
    lbm_add_extension("ext-disp-orientation", ext_disp_orientation);
}

#endif

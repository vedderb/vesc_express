#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"

#include "disp_sh8601.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"

static int display_width = 170;
static int display_height = 320;
static int display_x_offset = 35;
static int display_y_offset = 0;

static int m_pin_reset = -1;
static int m_pin_dc = -1;

#if CONFIG_IDF_TARGET_ESP32S3
	#define DISP_REG_SET		GPIO.out_w1ts
	#define DISP_REG_CLR		GPIO.out_w1tc
#elif CONFIG_IDF_TARGET_ESP32C3
	#define DISP_REG_SET		GPIO.out_w1ts.val
	#define DISP_REG_CLR		GPIO.out_w1tc.val
#else
	#error "Unsupported target"
#endif

#define COMMAND() 		(DISP_REG_CLR = 1 << m_pin_dc)
#define DATA() 			(DISP_REG_SET = 1 << m_pin_dc)

static void command_start(uint8_t command) {
	COMMAND();
	hwspi_send_data(&command, 1);
	DATA();
}

static inline uint16_t to_disp_color(uint32_t rgb) {
	uint8_t b = (uint8_t)rgb;
	uint8_t g = (uint8_t)(rgb >> 8);
	uint8_t r = (uint8_t)(rgb >> 16);
	r >>= 3;
	g >>= 2;
	b >>= 3;

	uint8_t color_high = 0;
	color_high = g << 5;
	color_high |= b;

	uint8_t color_low = 0;
	color_low = r << 3;
	color_low |= g >> 3;

	uint16_t color = color_high;
	color |= (((uint16_t)color_low) << 8);
	return color;
}

static void blast_indexed2(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 3;
		int bit  = 7 - (i & 0x7);
		int color_ind = (data[byte] & (1 << bit)) >> bit;

		uint16_t c = to_disp_color(
						COLOR_TO_RGB888(colors[color_ind],
								i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed4(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 2;
		int bit = (3 - (i & 0x03)) * 2;
		int color_ind = (data[byte] & (0x03 << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed16(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 1;
		int bit = (1 - (i & 0x01)) * 4;
		int color_ind = (data[byte] & (0x0F << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (uint32_t i = 0; i < num_pix; i ++) {
		uint8_t pix = data[i];
		uint32_t r = (uint32_t)((pix >> 5) & 0x7);
		uint32_t g = (uint32_t)((pix >> 2) & 0x7);
		uint32_t b = (uint32_t)(pix & 0x3);
		uint32_t rgb888 = r << (16 + 5) | g << (8 + 5) | b << 6;
		uint16_t disp = to_disp_color(rgb888);
		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb565(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (uint32_t i = 0; i < num_pix; i ++) {
		uint16_t pix = (((uint16_t)data[2 * i]) << 8) | ((uint16_t)data[2 * i + 1]);

		uint32_t r = (uint32_t)(pix >> 11);
		uint32_t g = (uint32_t)((pix >> 5) & 0x3F);
		uint32_t b = (uint32_t)(pix & 0x1F);
		uint32_t rgb888 = r << (16 + 3) | g << (8 + 2) | b << 3;
		uint16_t disp = to_disp_color(rgb888);

		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb888(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (uint32_t i = 0; i < num_pix; i ++) {
		uint32_t r = data[3 * i];
		uint32_t g = data[3 * i + 1];
		uint32_t b = data[3 * i + 2];

		uint32_t rgb888 = r << 16 | g << 8 | b;
		uint16_t disp = to_disp_color(rgb888);

		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

bool disp_sh8601_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	if ((x + img->width) > display_width || (y + img->height) > display_height) {
		return false;
	}

	uint16_t cs = x + display_x_offset;
	uint16_t ce = cs + img->width - 1;
	uint16_t ps = y + display_y_offset;
	uint16_t pe = ps + img->height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_sh8601_command(0x2A, col, 4);
	disp_sh8601_command(0x2B, row, 4);

	uint32_t num_pix = img->width * img->height;

	hwspi_begin();
	switch(img->fmt) {
	case indexed2:
		if (!colors) {
			hwspi_end();
			return false;
		}
		blast_indexed2(img, colors);
		break;
	case indexed4:
		if (!colors) {
			hwspi_end();
			return false;
		}
		blast_indexed4(img, colors);
		break;
	case indexed16:
		if (!colors) {
			hwspi_end();
			return false;
		}
		blast_indexed16(img, colors);
		break;
	case rgb332:
		blast_rgb332(img->data, num_pix);
		break;
	case rgb565:
		blast_rgb565(img->data, num_pix);
		break;
	case rgb888:
		blast_rgb888(img->data, num_pix);
		break;
	default:
		hwspi_end();
		return false;
	}
	hwspi_end();

	return true;
}

void disp_sh8601_clear(uint32_t color) {
	uint16_t clear_color_disp = to_disp_color(color);

	uint16_t cs = display_x_offset;
	uint16_t ce = display_x_offset + display_width - 1;
	uint16_t ps = display_y_offset;
	uint16_t pe = ps + display_height - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_sh8601_command(0x2A, col, 4);
	disp_sh8601_command(0x2B, row, 4);

	hwspi_begin();
	command_start(0x2C);
	hwspi_data_stream_start();
	for (int i = 0; i < (display_width * display_height); i ++) {
		hwspi_data_stream_write((uint8_t)(clear_color_disp));
		hwspi_data_stream_write((uint8_t)(clear_color_disp >> 8));
	}
	hwspi_data_stream_finish();
	hwspi_end();
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
	LBM_CHECK_NUMBER_ALL();

	lbm_value res = ENC_SYM_TERROR;

	if (argn > 1) {
		uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
		uint8_t paras[12];
		for (int i = 0; i < argn - 1; i ++) {
			paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
		}

		disp_sh8601_command(cmd, paras, argn - 1);
		res = ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t) lbm_dec_as_u32(args[0]);
		disp_sh8601_command(cmd, 0, 0);
		res = ENC_SYM_TRUE;
	}

	return res;
}

static lbm_value ext_disp_orientation(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_ARGN_NUMBER(1);

    uint32_t orientation = lbm_dec_as_u32(args[0]);
    uint8_t madctl = 0;
    lbm_value res = ENC_SYM_TRUE;

    switch (orientation) {
        case 0: // normal portrait
            madctl = 0b00001000;
            display_width = 170;
            display_height = 320;
			display_x_offset = 35;
			display_y_offset = 0;
            break;
        case 1: // 90° CW
            madctl = 0b10101000;
            display_width = 320;
            display_height = 170;
			display_x_offset = 0;
			display_y_offset = 35;
            break;
        case 2: // 180° 
            madctl = 0b11001000;
            display_width = 170;
            display_height = 320;
			display_x_offset = 35;
			display_y_offset = 0;
            break;
        case 3: // 270° CW
            madctl = 0b01101000;
            display_width = 320;
            display_height = 170;
			display_x_offset = 0;
			display_y_offset = 35;
            break;
        default:
            return ENC_SYM_EERROR;
    }

    disp_sh8601_command(0x36, &madctl, 1);
    return res;
}

void disp_sh8601_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz) {
	hwspi_init(clock_mhz, 0, -1, pin_sd0, pin_clk, pin_cs);
	m_pin_reset = pin_reset;
	m_pin_dc = pin_dc;

	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask = BIT(m_pin_dc) | BIT(m_pin_reset);
	gpconf.mode = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpconf.intr_type =  GPIO_INTR_DISABLE;

	gpio_config(&gpconf);

	gpio_set_level(m_pin_reset, 1);
	gpio_set_level(m_pin_dc, 1);

	lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
	lbm_add_extension("ext-disp-orientation", ext_disp_orientation);
}

void disp_sh8601_command(uint8_t command, const uint8_t *args, int argn) {
	hwspi_begin();
	command_start(command);
	if (args != NULL && argn > 0) {
		hwspi_send_data(args, argn);
	}
	hwspi_end();
}

static const uint8_t init_cmds[][16] = {
	{2,  0x3A, 0x55},
	{6,  0xB2, 0x0C, 0x0C, 0x00, 0x33, 0x33},
	{2,  0xB7, 0x35},
	{2,  0xBB, 0x13},
	{2,  0xC0, 0x2C},
	{2,  0xC2, 0x01},
	{2,  0xC3, 0x0B},
	{2,  0xC4, 0x20},
	{2,  0xC6, 0x0F},
	{3,  0xD0, 0xA4, 0xA1},
	{2,  0xD6, 0xA1},
	{16, 0xE0, 0x00,0x03,0x07,0x08,0x07,0x15,0x2A,0x44,0x42,0x0A,0x17,0x18,0x25,0x27},
	{16, 0xE1, 0x00,0x03,0x08,0x07,0x07,0x23,0x2A,0x43,0x42,0x09,0x18,0x17,0x25,0x27},
};

void disp_sh8601_reset(void) {
	gpio_set_level(m_pin_reset, 0);
	vTaskDelay(5);
	gpio_set_level(m_pin_reset, 1);
	vTaskDelay(120);

	for (int i = 0; i < (int)(sizeof(init_cmds) / sizeof(init_cmds[0])); i ++) {
		int argn = init_cmds[i][0] - 1;
		const uint8_t *args = &init_cmds[i][2];
		uint8_t cmd = init_cmds[i][1];
		disp_sh8601_command(cmd, args, argn);
	}

	disp_sh8601_command(0x11, NULL, 0);
	vTaskDelay(120);
	disp_sh8601_command(0x21, NULL, 0);
	vTaskDelay(20);
	disp_sh8601_command(0x29, NULL, 0);
	vTaskDelay(120);

	disp_sh8601_clear(0);
}

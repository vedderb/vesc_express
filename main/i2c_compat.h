#ifndef MAIN_I2C_COMPAT_H_
#define MAIN_I2C_COMPAT_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "hal/i2c_types.h"

// Compatibility facade for legacy VESC I2C call sites. The implementation is
// backed by the ESP-IDF v6 i2c_master driver; do not include driver/i2c.h.

typedef struct {
	i2c_mode_t mode;
	int sda_io_num;
	int scl_io_num;
	gpio_pullup_t sda_pullup_en;
	gpio_pullup_t scl_pullup_en;
	struct {
		uint32_t clk_speed;
	} master;
} i2c_config_t;

typedef void *i2c_cmd_handle_t;

esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t *conf);
esp_err_t i2c_driver_install(
	i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len,
	int intr_alloc_flags
);
esp_err_t i2c_driver_delete(i2c_port_t i2c_num);
esp_err_t i2c_reset_tx_fifo(i2c_port_t i2c_num);
esp_err_t i2c_reset_rx_fifo(i2c_port_t i2c_num);

esp_err_t i2c_master_write_read_device(
	i2c_port_t i2c_num, uint8_t device_address, const uint8_t *write_buffer,
	size_t write_size, uint8_t *read_buffer, size_t read_size,
	TickType_t ticks_to_wait
);
esp_err_t i2c_master_read_from_device(
	i2c_port_t i2c_num, uint8_t device_address, uint8_t *read_buffer,
	size_t read_size, TickType_t ticks_to_wait
);
esp_err_t i2c_master_write_to_device(
	i2c_port_t i2c_num, uint8_t device_address, const uint8_t *write_buffer,
	size_t write_size, TickType_t ticks_to_wait
);

i2c_cmd_handle_t i2c_cmd_link_create(void);
void i2c_cmd_link_delete(i2c_cmd_handle_t cmd);
esp_err_t i2c_master_start(i2c_cmd_handle_t cmd);
esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd, uint8_t byte, bool ack_en);
esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd);
esp_err_t i2c_master_cmd_begin(
	i2c_port_t i2c_num, i2c_cmd_handle_t cmd, TickType_t ticks_to_wait
);

#endif /* MAIN_I2C_COMPAT_H_ */

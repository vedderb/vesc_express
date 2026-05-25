#include <stdlib.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "i2c_compat.h"
#include "soc/soc_caps.h"

typedef struct {
	bool config_valid;
	bool installed;
	i2c_config_t config;
	i2c_master_bus_handle_t bus;
	i2c_master_dev_handle_t devices[128];
	SemaphoreHandle_t mutex;
} i2c_port_state_t;

struct i2c_cmd_compat_t {
	bool address_valid;
	uint8_t address;
};

static i2c_port_state_t m_ports[SOC_I2C_NUM];

static esp_err_t check_port(i2c_port_t i2c_num) {
	return (i2c_num >= 0 && i2c_num < SOC_I2C_NUM) ? ESP_OK : ESP_ERR_INVALID_ARG;
}

static int ms_from_ticks(TickType_t ticks) {
	if (ticks == portMAX_DELAY) {
		return -1;
	}

	return (int)pdTICKS_TO_MS(ticks);
}

static esp_err_t ensure_mutex(i2c_port_state_t *state) {
	if (!state->mutex) {
		state->mutex = xSemaphoreCreateMutex();
		if (!state->mutex) {
			return ESP_ERR_NO_MEM;
		}
	}

	return ESP_OK;
}

static void remove_devices(i2c_port_state_t *state) {
	for (int i = 0; i < 128; i++) {
		if (state->devices[i]) {
			i2c_master_bus_rm_device(state->devices[i]);
			state->devices[i] = NULL;
		}
	}
}

static esp_err_t delete_bus_locked(i2c_port_state_t *state) {
	remove_devices(state);

	if (state->bus) {
		esp_err_t res = i2c_del_master_bus(state->bus);
		state->bus = NULL;
		state->installed = false;
		return res;
	}

	state->installed = false;
	return ESP_OK;
}

static esp_err_t install_bus_locked(i2c_port_t i2c_num, i2c_port_state_t *state) {
	if (!state->config_valid || state->config.mode != I2C_MODE_MASTER) {
		return ESP_ERR_INVALID_STATE;
	}

	delete_bus_locked(state);

	i2c_master_bus_config_t bus_config = {
		.i2c_port = i2c_num,
		.sda_io_num = state->config.sda_io_num,
		.scl_io_num = state->config.scl_io_num,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.intr_priority = 0,
		.trans_queue_depth = 0,
		.flags = {
			.enable_internal_pullup =
				(state->config.sda_pullup_en == GPIO_PULLUP_ENABLE) ||
				(state->config.scl_pullup_en == GPIO_PULLUP_ENABLE),
		},
	};

	esp_err_t res = i2c_new_master_bus(&bus_config, &state->bus);
	if (res == ESP_OK) {
		state->installed = true;
	}

	return res;
}

static esp_err_t get_device_locked(
	i2c_port_state_t *state, uint8_t device_address,
	i2c_master_dev_handle_t *device
) {
	if (!state->installed || !state->bus) {
		return ESP_ERR_INVALID_STATE;
	}

	if (device_address >= 128) {
		return ESP_ERR_INVALID_ARG;
	}

	if (!state->devices[device_address]) {
		i2c_device_config_t device_config = {
			.dev_addr_length = I2C_ADDR_BIT_LEN_7,
			.device_address = device_address,
			.scl_speed_hz = state->config.master.clk_speed,
			.scl_wait_us = 0,
		};

		esp_err_t res = i2c_master_bus_add_device(
			state->bus, &device_config, &state->devices[device_address]
		);
		if (res != ESP_OK) {
			return res;
		}
	}

	*device = state->devices[device_address];
	return ESP_OK;
}

esp_err_t i2c_param_config(i2c_port_t i2c_num, const i2c_config_t *conf) {
	if (check_port(i2c_num) != ESP_OK || !conf) {
		return ESP_ERR_INVALID_ARG;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	esp_err_t res = ensure_mutex(state);
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(state->mutex, portMAX_DELAY);
	state->config = *conf;
	state->config_valid = true;
	xSemaphoreGive(state->mutex);

	return ESP_OK;
}

esp_err_t i2c_driver_install(
	i2c_port_t i2c_num, i2c_mode_t mode, size_t slv_rx_buf_len, size_t slv_tx_buf_len,
	int intr_alloc_flags
) {
	(void)slv_rx_buf_len;
	(void)slv_tx_buf_len;
	(void)intr_alloc_flags;

	if (check_port(i2c_num) != ESP_OK || mode != I2C_MODE_MASTER) {
		return ESP_ERR_INVALID_ARG;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	esp_err_t res = ensure_mutex(state);
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(state->mutex, portMAX_DELAY);
	res = install_bus_locked(i2c_num, state);
	xSemaphoreGive(state->mutex);

	return res;
}

esp_err_t i2c_driver_delete(i2c_port_t i2c_num) {
	if (check_port(i2c_num) != ESP_OK) {
		return ESP_ERR_INVALID_ARG;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	esp_err_t res = ensure_mutex(state);
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(state->mutex, portMAX_DELAY);
	res = delete_bus_locked(state);
	xSemaphoreGive(state->mutex);

	return res;
}

esp_err_t i2c_reset_tx_fifo(i2c_port_t i2c_num) {
	if (check_port(i2c_num) != ESP_OK) {
		return ESP_ERR_INVALID_ARG;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	if (!state->bus) {
		return ESP_ERR_INVALID_STATE;
	}

	return i2c_master_bus_reset(state->bus);
}

esp_err_t i2c_reset_rx_fifo(i2c_port_t i2c_num) {
	return i2c_reset_tx_fifo(i2c_num);
}

esp_err_t i2c_master_write_read_device(
	i2c_port_t i2c_num, uint8_t device_address, const uint8_t *write_buffer,
	size_t write_size, uint8_t *read_buffer, size_t read_size,
	TickType_t ticks_to_wait
) {
	if (check_port(i2c_num) != ESP_OK || !read_buffer || read_size == 0 ||
		(!write_buffer && write_size > 0)) {
		return ESP_ERR_INVALID_ARG;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	esp_err_t res = ensure_mutex(state);
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(state->mutex, portMAX_DELAY);
	i2c_master_dev_handle_t device = NULL;
	res = get_device_locked(state, device_address, &device);
	if (res == ESP_OK) {
		res = i2c_master_transmit_receive(
			device, write_buffer, write_size, read_buffer, read_size,
			ms_from_ticks(ticks_to_wait)
		);
	}
	xSemaphoreGive(state->mutex);

	return res;
}

esp_err_t i2c_master_read_from_device(
	i2c_port_t i2c_num, uint8_t device_address, uint8_t *read_buffer,
	size_t read_size, TickType_t ticks_to_wait
) {
	if (check_port(i2c_num) != ESP_OK || !read_buffer || read_size == 0) {
		return ESP_ERR_INVALID_ARG;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	esp_err_t res = ensure_mutex(state);
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(state->mutex, portMAX_DELAY);
	i2c_master_dev_handle_t device = NULL;
	res = get_device_locked(state, device_address, &device);
	if (res == ESP_OK) {
		res = i2c_master_receive(device, read_buffer, read_size, ms_from_ticks(ticks_to_wait));
	}
	xSemaphoreGive(state->mutex);

	return res;
}

esp_err_t i2c_master_write_to_device(
	i2c_port_t i2c_num, uint8_t device_address, const uint8_t *write_buffer,
	size_t write_size, TickType_t ticks_to_wait
) {
	if (check_port(i2c_num) != ESP_OK || (!write_buffer && write_size > 0)) {
		return ESP_ERR_INVALID_ARG;
	}

	if (write_size == 0) {
		return ESP_OK;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	esp_err_t res = ensure_mutex(state);
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(state->mutex, portMAX_DELAY);
	i2c_master_dev_handle_t device = NULL;
	res = get_device_locked(state, device_address, &device);
	if (res == ESP_OK) {
		res = i2c_master_transmit(device, write_buffer, write_size, ms_from_ticks(ticks_to_wait));
	}
	xSemaphoreGive(state->mutex);

	return res;
}

i2c_cmd_handle_t i2c_cmd_link_create(void) {
	i2c_cmd_handle_t cmd = calloc(1, sizeof(struct i2c_cmd_compat_t));
	return cmd;
}

void i2c_cmd_link_delete(i2c_cmd_handle_t cmd) {
	free(cmd);
}

esp_err_t i2c_master_start(i2c_cmd_handle_t cmd) {
	return cmd ? ESP_OK : ESP_ERR_INVALID_ARG;
}

esp_err_t i2c_master_write_byte(i2c_cmd_handle_t cmd, uint8_t byte, bool ack_en) {
	(void)ack_en;

	if (!cmd) {
		return ESP_ERR_INVALID_ARG;
	}

	struct i2c_cmd_compat_t *compat = cmd;
	compat->address = byte >> 1;
	compat->address_valid = true;
	return ESP_OK;
}

esp_err_t i2c_master_stop(i2c_cmd_handle_t cmd) {
	return cmd ? ESP_OK : ESP_ERR_INVALID_ARG;
}

esp_err_t i2c_master_cmd_begin(
	i2c_port_t i2c_num, i2c_cmd_handle_t cmd, TickType_t ticks_to_wait
) {
	struct i2c_cmd_compat_t *compat = cmd;
	if (check_port(i2c_num) != ESP_OK || !compat || !compat->address_valid) {
		return ESP_ERR_INVALID_ARG;
	}

	i2c_port_state_t *state = &m_ports[i2c_num];
	esp_err_t res = ensure_mutex(state);
	if (res != ESP_OK) {
		return res;
	}

	xSemaphoreTake(state->mutex, portMAX_DELAY);
	if (!state->installed || !state->bus) {
		res = ESP_ERR_INVALID_STATE;
	} else {
		res = i2c_master_probe(state->bus, compat->address, ms_from_ticks(ticks_to_wait));
	}
	xSemaphoreGive(state->mutex);

	return res;
}

/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "lvgl_runtime.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lvgl.h"
#include "lvgl_display.h"

#define TAG "lvgl_runtime"

#define LVGL_TASK_STACK_SIZE 6144
#define LVGL_TASK_PRIORITY   3
#define LVGL_TASK_DELAY_MIN_MS 5U
#define LVGL_TASK_DELAY_MAX_MS 50U
#define LVGL_STOP_TIMEOUT_MS 2000U

static TaskHandle_t m_task;
static SemaphoreHandle_t m_stopped;
static volatile bool m_stop_requested;
static volatile bool m_ready;

static uint32_t tick_ms(void) {
	return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void lvgl_task(void *argument) {
	(void)argument;
	while (!m_stop_requested) {
		uint32_t delay_ms = lv_timer_handler();
		if (delay_ms < LVGL_TASK_DELAY_MIN_MS) {
			delay_ms = LVGL_TASK_DELAY_MIN_MS;
		} else if (delay_ms > LVGL_TASK_DELAY_MAX_MS) {
			delay_ms = LVGL_TASK_DELAY_MAX_MS;
		}
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(delay_ms));
	}

	xSemaphoreGive(m_stopped);
	vTaskDelete(NULL);
}

esp_err_t lvgl_runtime_start(void) {
	if (m_ready) {
		return ESP_ERR_INVALID_STATE;
	}

	m_stopped = xSemaphoreCreateBinary();
	if (!m_stopped) {
		return ESP_ERR_NO_MEM;
	}

	lv_init();
	lv_tick_set_cb(tick_ms);
	m_stop_requested = false;
	BaseType_t created = xTaskCreatePinnedToCore(
		lvgl_task, "lvgl", LVGL_TASK_STACK_SIZE, NULL,
		LVGL_TASK_PRIORITY, &m_task, tskNO_AFFINITY);
	if (created != pdPASS) {
		lv_deinit();
		vSemaphoreDelete(m_stopped);
		m_stopped = NULL;
		return ESP_ERR_NO_MEM;
	}

	m_ready = true;
	ESP_LOGI(TAG, "LVGL runtime started");
	return ESP_OK;
}

esp_err_t lvgl_runtime_stop(void) {
	if (!m_ready || !m_task || !m_stopped) {
		return ESP_ERR_INVALID_STATE;
	}

	m_stop_requested = true;
	xTaskNotifyGive(m_task);
	if (xSemaphoreTake(m_stopped, pdMS_TO_TICKS(LVGL_STOP_TIMEOUT_MS)) !=
			pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}

	lvgl_display_detach();
	lv_deinit();
	m_task = NULL;
	m_ready = false;
	m_stop_requested = false;
	vSemaphoreDelete(m_stopped);
	m_stopped = NULL;
	ESP_LOGI(TAG, "LVGL runtime stopped");
	return ESP_OK;
}

bool lvgl_runtime_is_ready(void) {
	return m_ready;
}

bool lvgl_runtime_lock(void) {
	if (!m_ready) {
		return false;
	}
	lv_lock();
	return true;
}

void lvgl_runtime_unlock(void) {
	if (m_ready) {
		lv_unlock();
	}
}

esp_err_t lvgl_runtime_get_memory_stats(
		lvgl_runtime_memory_stats_t *stats) {
	if (!stats) {
		return ESP_ERR_INVALID_ARG;
	}
	if (!lvgl_runtime_lock()) {
		return ESP_ERR_INVALID_STATE;
	}

	lv_mem_monitor_t monitor = {0};
	lv_mem_monitor(&monitor);
	stats->total_size = monitor.total_size;
	stats->free_size = monitor.free_size;
	stats->largest_free_block = monitor.free_biggest_size;
	stats->used_percent = monitor.used_pct;
	stats->fragmentation_percent = monitor.frag_pct;
	lvgl_runtime_unlock();
	return ESP_OK;
}

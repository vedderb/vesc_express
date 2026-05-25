/*
	Copyright 2026 JetFleet

	Storage/logging stub for hardware profiles that do not mount a file system.
 */

#include "log.h"

char *file_basepath = "";

bool log_init(void) {
	return false;
}

esp_err_t log_mount_card(int pin_mosi, int pin_miso, int pin_sck, int pin_cs, int freq) {
	(void)pin_mosi;
	(void)pin_miso;
	(void)pin_sck;
	(void)pin_cs;
	(void)freq;
	return ESP_ERR_NOT_SUPPORTED;
}

void log_unmount_card(void) {
}

bool log_mount_nand_flash(int pin_mosi, int pin_miso, int pin_sck, int pin_cs, int freq) {
	(void)pin_mosi;
	(void)pin_miso;
	(void)pin_sck;
	(void)pin_cs;
	(void)freq;
	return false;
}

void log_unmount_nand_flash(void) {
}

#ifdef HW_INTERNAL_FS
esp_err_t log_mount_storage(void) {
	return ESP_ERR_NOT_SUPPORTED;
}

void log_unmount_storage(void) {
}

esp_err_t log_storage_info(size_t *total, size_t *used) {
	if (total) {
		*total = 0;
	}

	if (used) {
		*used = 0;
	}

	return ESP_ERR_NOT_SUPPORTED;
}
#endif

void log_process_packet(unsigned char *data, unsigned int len) {
	(void)data;
	(void)len;
}

#include <stdbool.h>
#include <string.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "bootloader_init.h"
#include "bootloader_utility.h"
#include "bootloader_common.h"
#include "bootloader_flash_priv.h"

// Private functions
static int select_partition_number(bootloader_state_t *bs);

void __attribute__((noreturn)) call_start_cpu0(void) {
	if (bootloader_init() != ESP_OK) {
		bootloader_reset();
	}

#ifdef CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP
	// If this boot is a wake up from the deep sleep then go to the short way,
	// try to load the application which worked before deep sleep.
	// It skips a lot of checks due to it was done before (while first boot).
	bootloader_utility_load_boot_image_from_deep_sleep();
	// If it is not successful try to load an application as usual.
#endif

	bootloader_state_t bs = {0};
	int boot_index = select_partition_number(&bs);
	if (boot_index == INVALID_INDEX) {
		bootloader_reset();
	}

	bootloader_utility_load_boot_image(&bs, boot_index);
}

static int select_partition_number(bootloader_state_t *bs) {
	if (!bootloader_utility_load_partition_table(bs)) {
		return INVALID_INDEX;
	}

	return bootloader_utility_get_selected_boot_partition(bs);
}

struct _reent *__getreent(void) {
	return _GLOBAL_REENT;
}

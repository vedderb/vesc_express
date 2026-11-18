#include "hw_s31_twincan.h"
#include "esp_check.h"

void hw_init(void) {
	gpio_config_t leds = {
		.pin_bit_mask = (1ULL << LED_YELLOW1_PIN) | (1ULL << LED_YELLOW2_PIN),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&leds));
	LED_YELLOW1_OFF();
	LED_YELLOW2_OFF();
}

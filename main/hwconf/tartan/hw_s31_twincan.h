#ifndef HW_S31_TWINCAN_H_
#define HW_S31_TWINCAN_H_

#include "driver/gpio.h"

#define HW_NAME                     "S31 TwinCAN"
#define HW_TARGET                   "esp32s31_n16r16v"
#define HW_INIT_HOOK()              hw_init()

// UART
// #define HW_UART_COMM
// #define UART_NUM                    0
// #define UART_BAUDRATE               115200
// #define UART_TX                     58
// #define UART_RX                     59

// CAN1
#define CAN_TX_GPIO_NUM             38
#define CAN_RX_GPIO_NUM             39

// CAN2
#define CAN2_TX_GPIO_NUM            42
#define CAN2_RX_GPIO_NUM            40

// SDMMC
#define HW_SDMMC_SLOT              0
#define HW_SDMMC_WIDTH             4
#define HW_SDMMC_CD_PIN            35

// WS2812B
#define HW_RGBLED_PIN               43
#define HW_RGBLED_COUNT             1
#define HW_RGBLED_TIMING            1

// Yellow LEDs
#define LED_YELLOW1_PIN             60
#define LED_YELLOW2_PIN             61
#define LED_YELLOW1_ON()            gpio_set_level(LED_YELLOW1_PIN, 1)
#define LED_YELLOW1_OFF()           gpio_set_level(LED_YELLOW1_PIN, 0)
#define LED_YELLOW2_ON()            gpio_set_level(LED_YELLOW2_PIN, 1)
#define LED_YELLOW2_OFF()           gpio_set_level(LED_YELLOW2_PIN, 0)

void hw_init(void);

#endif /* HW_S31_TWINCAN_H_ */

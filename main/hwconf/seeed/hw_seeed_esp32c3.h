#ifndef MAIN_HWCONF_SEEED_HW_SEEED_ESP32C3_H_
#define MAIN_HWCONF_SEEED_HW_SEEED_ESP32C3_H_

#define HW_NAME						"Seeed ESP32-C3"
#define HW_TARGET					"esp32c3"

#define HW_INIT_HOOK()

// CAN
#define CAN_TX_GPIO_NUM				3 // Pin D1 (GPIO3)
#define CAN_RX_GPIO_NUM				2 // Pin D0 (GPIO2)

// UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						21
#define UART_RX						20

#endif /* MAIN_HWCONF_SEEED_HW_SEEED_ESP32C3_H_ */
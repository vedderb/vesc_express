#ifndef HW_DEVKIT_ESP32_16MB_H_
#define HW_DEVKIT_ESP32_16MB_H_

#define HW_NAME						"Devkit ESP32 16MB"
#define HW_TARGET					"esp32_16mb"
#define HW_UART_COMM

#define HW_INIT_HOOK()				hw_init()

// UART
#define UART_NUM					0
#define UART_BAUDRATE				115200
#define UART_TX						1
#define UART_RX						3

// Functions
void hw_init(void);

#endif /* HW_DEVKIT_ESP32_16MB_H_ */

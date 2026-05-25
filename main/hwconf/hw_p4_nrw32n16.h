#ifndef MAIN_HWCONF_VESC_HW_P4_NRW32N16_H_
#define MAIN_HWCONF_VESC_HW_P4_NRW32N16_H_

#define HW_NAME					"P4 NRW32N16"
#define HW_TARGET				"esp32p4_nrw32n16"
#define HW_NO_UART

#define HW_INIT_HOOK()			hw_init()

// CAN
// #define CAN_TX_GPIO_NUM				1
// #define CAN_RX_GPIO_NUM				0

// UART
// #define UART_NUM				0
// #define UART_BAUDRATE			115200
// #define UART_TX					21
// #define UART_RX					20

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_VESC_HW_P4_NRW32N16_H_ */
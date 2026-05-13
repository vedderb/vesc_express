#ifndef HW_C6_4M_H_
#define HW_C6_4M_H_

#define HW_NAME                 "Devkit C6 4M"
#define HW_TARGET               "esp32c6"
#define HW_UART_COMM

#define HW_INIT_HOOK()          hw_init()

// UART
#define UART_NUM                0
#define UART_BAUDRATE           115200
#define UART_TX                 16
#define UART_RX                 17

// Functions
void hw_init(void);

#endif /* HW_C6_4M_H_ */

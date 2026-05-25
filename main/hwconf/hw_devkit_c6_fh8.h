#ifndef MAIN_HWCONF_DEVKIT_C6_FH8_H_
#define MAIN_HWCONF_DEVKIT_C6_FH8_H_

#define HW_NAME                     "Devkit C6 FH8"
#define HW_TARGET                   "esp32c6_fh8"
#define HW_UART_COMM

#define HW_INIT_HOOK()              hw_init()

// UART
#define UART_NUM                    0
#define UART_BAUDRATE               115200
#define UART_TX                     16
#define UART_RX                     17

// Functions
void hw_init(void);

#endif /* MAIN_HWCONF_DEVKIT_C6_FH8_H_ */
#ifndef HW_DEVKIT_C6_H_
#define HW_DEVKIT_C6_H_

#define HW_NAME                 "Devkit C6"
#define HW_TARGET               "esp32c6"
#define HW_UART_COMM

#define HW_INIT_HOOK()          hw_init()

// CAN
#define CAN_TX_GPIO_NUM 1
#define CAN_RX_GPIO_NUM 0

// Functions
void hw_init(void);

#endif /* HW_DEVKIT_C6_H_ */

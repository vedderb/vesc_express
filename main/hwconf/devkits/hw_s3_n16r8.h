#ifndef HW_S3_N16R8_H_
#define HW_S3_N16R8_H_

#define HW_NAME                 "S3 N16R8"
#define HW_TARGET               "esp32s3_n16r8"
#define HW_UART_COMM

#define HW_INIT_HOOK()          hw_init()

// CAN
#define CAN_TX_GPIO_NUM			16
#define CAN_RX_GPIO_NUM			17

// UART
#define UART_NUM                0
#define UART_BAUDRATE           115200
#define UART_TX                 43
#define UART_RX                 44

// Functions
void hw_init(void);

#endif /* HW_S3_N16R8_H_ */

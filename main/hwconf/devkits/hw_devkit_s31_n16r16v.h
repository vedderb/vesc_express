#ifndef HW_DEVKIT_S31_N16R16V_H_
#define HW_DEVKIT_S31_N16R16V_H_

#define HW_NAME                 "Devkit S31 N16R16V"
#define HW_TARGET               "esp32s31_n16r16v"
#define HW_UART_COMM

// ESP32-S31 UART0 IOMUX pins. USB Serial/JTAG also carries VESC packets.
#define UART_NUM                0
#define UART_BAUDRATE           115200
#define UART_TX                 58
#define UART_RX                 59

#endif /* HW_DEVKIT_S31_N16R16V_H_ */

/*
	SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef MAIN_HWCONF_WAVESHARE_AMOLED_175_H_
#define MAIN_HWCONF_WAVESHARE_AMOLED_175_H_

#include "driver/gpio.h"

#define HW_NAME   "Waveshare AMOLED 1.75"
#define HW_TARGET "esp32s3_n16r8"

#define HW_NO_UART
#define HW_EARLY_LBM_INIT
#define USER_EXTENSION_STORAGE_SIZE 32
#define HW_INIT_HOOK()         hw_init()
#define HW_POST_LISPIF_HOOK() hw_post_lispif_init()

/* CAN pins exposed on the board header. */
#define CAN_TX_GPIO_NUM 43
#define CAN_RX_GPIO_NUM 44

/* Shared AXP2101/CST9217 I2C bus. Touch is connected in phase 6. */
#define WAVESHARE_I2C_SDA      15
#define WAVESHARE_I2C_SCL      14
#define WAVESHARE_I2C_CLOCK_HZ 400000

/* 466x466 CO5300 AMOLED over QSPI. */
#define WAVESHARE_DISP_DATA0         4
#define WAVESHARE_DISP_DATA1         5
#define WAVESHARE_DISP_DATA2         6
#define WAVESHARE_DISP_DATA3         7
#define WAVESHARE_DISP_CLOCK         38
#define WAVESHARE_DISP_CS            12
#define WAVESHARE_DISP_RESET         9
#define WAVESHARE_DISP_CLOCK_HZ      80000000
#define WAVESHARE_DISP_COLUMN_OFFSET 6

#define WAVESHARE_TOUCH_INTERRUPT 11
#define WAVESHARE_TOUCH_RESET     40

void hw_init(void);
void hw_post_lispif_init(void);

#endif /* MAIN_HWCONF_WAVESHARE_AMOLED_175_H_ */

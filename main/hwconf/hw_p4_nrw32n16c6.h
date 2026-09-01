#ifndef MAIN_HWCONF_HW_P4_NRW32N16C6_H_
#define MAIN_HWCONF_HW_P4_NRW32N16C6_H_

#define HW_NAME					"P4 NRW32N16C6"
#define HW_TARGET				"esp32p4_nrw32n16c6"
#define HW_NO_UART

#define HW_INIT_HOOK()			hw_init()

/* C6 SDIO uses GPIO14..19, so ADC inputs are moved to GPIO20..23. */
#define HW_ADC_CH0				ADC_CHANNEL_4
#define HW_ADC_CH1				ADC_CHANNEL_5
#define HW_ADC_CH2				ADC_CHANNEL_6
#define HW_ADC_CH3				ADC_CHANNEL_7
#define HW_ADC_CH4				ADC_CHANNEL_4

void hw_init(void);

#endif /* MAIN_HWCONF_HW_P4_NRW32N16C6_H_ */

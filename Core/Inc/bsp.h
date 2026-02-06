#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
extern "C" {
#endif

void rtc_Init(void);
void iwdt_Init(void);
void rcc_Init(void);
void uart_Init(void);
void i2c_Init(void);
void key_Init(void);
void led_Init(void);
void adc_Init(void);
void atim_Init(void);
void btim1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */


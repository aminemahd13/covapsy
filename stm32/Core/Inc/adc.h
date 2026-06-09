/*
 * adc.h: rear IR distance sensor on ADC1_IN2 (PA1).
 *
 * Sharp GP2xx analog IR on the Interface board X_IR_G port, routed
 * TELEM_G to Arduino A1 to STM32 PA1 (ADC1_IN2). X_IR_D lands on PA3
 * (Pi-link UART RX), so only X_IR_G is usable.
 *
 * Higher raw value = closer obstacle.
 */
#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

void     adc_rear_ir_init(void);
uint16_t adc_rear_ir_raw(void);   /* 12-bit (0..4095), higher = closer */

#endif /* ADC_H_ */

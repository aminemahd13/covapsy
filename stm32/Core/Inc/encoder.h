/*
 * encoder.h: optical fork wheel encoder on PA0 (Arduino A0 = TIM2_CH1, AF1).
 * TIM2 in external-clock mode counts pulses in hardware (no interrupts).
 * Pulse frequency is proportional to wheel speed; counts per metre is a
 * calibration constant (see ENC_PULSES_PER_M).
 */
#ifndef ENCODER_H_
#define ENCODER_H_

#include <stdint.h>

void     encoder_init(void);
uint32_t encoder_count(void);   /* 32-bit free-running pulse total */

#endif /* ENCODER_H_ */

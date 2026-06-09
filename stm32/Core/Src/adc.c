/*
 * adc.c: ADC1 single-channel polling for the rear IR sensor.
 * PA1 = ADC1_IN2, clocked HCLK/4. One channel, polled (~20 Hz). No DMA, no interrupts.
 */
#include "adc.h"
#include "stm32g4xx.h"

#define IR_ADC_CHANNEL 2u            /* PA1 = ADC1_IN2 */

static void adc_busy_delay(volatile uint32_t n)
{
    while (n--) { __asm volatile ("nop"); }
}

void adc_rear_ir_init(void)
{
    /* PA1 analog mode (MODER = 0b11), no pull. */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER |= (3u << (1 * 2));
    GPIOA->PUPDR &= ~(3u << (1 * 2));

    /* ADC12 kernel clock, common config: synchronous HCLK/4. */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
    ADC12_COMMON->CCR = (ADC12_COMMON->CCR & ~ADC_CCR_CKMODE)
                        | (3u << ADC_CCR_CKMODE_Pos);   /* 11 = HCLK/4 */

    /* Exit deep-power-down, enable voltage regulator, wait. */
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    adc_busy_delay(20000);                              /* > 20 us @170 MHz */

    /* Single-ended calibration, bounded loop. */
    ADC1->CR &= ~ADC_CR_ADCALDIF;
    ADC1->CR |= ADC_CR_ADCAL;
    for (uint32_t i = 0; i < 2000000u; i++) {
        if (!(ADC1->CR & ADC_CR_ADCAL)) break;
    }

    /* Enable ADC, wait until ready, bounded loop. */
    ADC1->ISR = ADC_ISR_ADRDY;                          /* clear stale flag */
    ADC1->CR |= ADC_CR_ADEN;
    for (uint32_t i = 0; i < 2000000u; i++) {
        if (ADC1->ISR & ADC_ISR_ADRDY) break;
    }

    /* Channel 2 sampling time = 247.5 cycles (SMP=110); long sample for high-impedance Sharp output. */
    ADC1->SMPR1 = (ADC1->SMPR1 & ~(7u << (3 * IR_ADC_CHANNEL)))
                  | (6u << (3 * IR_ADC_CHANNEL));

    /* Regular sequence: length 1, SQ1 = channel 2. */
    ADC1->SQR1 = (IR_ADC_CHANNEL << ADC_SQR1_SQ1_Pos);
}

uint16_t adc_rear_ir_raw(void)
{
    ADC1->ISR = ADC_ISR_EOC;            /* clear */
    ADC1->CR |= ADC_CR_ADSTART;
    /* Bounded wait for end of conversion. */
    for (uint32_t i = 0; i < 100000u; i++) {
        if (ADC1->ISR & ADC_ISR_EOC) break;
    }
    return (uint16_t)(ADC1->DR & 0xFFFFu);
}

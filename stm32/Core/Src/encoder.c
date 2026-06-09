/*
 * encoder.c: wheel-speed pulse counter on PA0 (TIM2_CH1, AF1) in external clock
 * mode 1. TIM2->CNT increments once per rising edge of the opto-fork pulse.
 */
#include "encoder.h"
#include "stm32g4xx.h"

#define ENC_PIN 0u    /* PA0 = TIM2_CH1, AF1 */

void encoder_init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    /* PA0 to AF1 (TIM2_CH1), AF input, no pull (Interface board pulls via R6). */
    GPIOA->MODER   = (GPIOA->MODER   & ~(3u << (ENC_PIN * 2))) | (2u << (ENC_PIN * 2));
    GPIOA->OTYPER &= ~(1u << ENC_PIN);
    GPIOA->OSPEEDR &= ~(3u << (ENC_PIN * 2));
    GPIOA->PUPDR  &= ~(3u << (ENC_PIN * 2));
    GPIOA->AFR[0]  = (GPIOA->AFR[0]  & ~(0xFu << (ENC_PIN * 4))) | (1u << (ENC_PIN * 4));

    TIM2->CR1 = 0;
    TIM2->PSC = 0;
    TIM2->ARR = 0xFFFFFFFFu;          /* 32-bit free-running */

    /* CC1 as input, IC1 mapped to TI1, with input filter for debounce. */
    TIM2->CCMR1 = (TIM2->CCMR1 & ~(TIM_CCMR1_CC1S | TIM_CCMR1_IC1F))
                  | (1u << TIM_CCMR1_CC1S_Pos)     /* CC1S = 01: IC1 on TI1 */
                  | (3u << TIM_CCMR1_IC1F_Pos);    /* filter: f_DTS/N, N=8 */
    TIM2->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC1NP);  /* count rising edges */

    /* External clock mode 1, clocked from TI1FP1. */
    TIM2->SMCR = (TIM2->SMCR & ~(TIM_SMCR_SMS | TIM_SMCR_TS))
                 | (7u << TIM_SMCR_SMS_Pos)        /* SMS = 111: ext clock mode 1 */
                 | (5u << TIM_SMCR_TS_Pos);        /* TS  = 101: TI1FP1 */

    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

uint32_t encoder_count(void)
{
    return TIM2->CNT;
}

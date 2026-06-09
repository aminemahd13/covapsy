/*
 * buzzer.c: passive buzzer on PB6 via TIM4_CH1 PWM.
 *
 * Timer clock 170 MHz, PSC=169 gives 1 MHz tick. ARR sets tone period,
 * CCR1 = ARR/2 is a 50% square wave, CCR1 = 0 is silent.
 *
 * Startup melody is non-blocking: buzzer_song_start() begins it,
 * buzzer_song_update(now) advances one note per main-loop pass.
 */
#include "buzzer.h"
#include "stm32g4xx.h"
#include <stdbool.h>

#define BUZZER_PIN 6u        /* PB6 = TIM4_CH1, AF2 */
#define TIM4_TICK_HZ 1000000u

void buzzer_init(void)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;

    /* PB6 to AF2 (TIM4_CH1), push-pull, low speed, no pull. */
    GPIOB->MODER   = (GPIOB->MODER   & ~(3u << (BUZZER_PIN * 2))) | (2u << (BUZZER_PIN * 2));
    GPIOB->OTYPER &= ~(1u << BUZZER_PIN);
    GPIOB->OSPEEDR &= ~(3u << (BUZZER_PIN * 2));
    GPIOB->PUPDR  &= ~(3u << (BUZZER_PIN * 2));
    GPIOB->AFR[0]  = (GPIOB->AFR[0]  & ~(0xFu << (BUZZER_PIN * 4))) | (2u << (BUZZER_PIN * 4));

    TIM4->PSC  = 169;             /* 170 MHz / 170 = 1 MHz */
    TIM4->ARR  = 1000u - 1u;      /* placeholder 1 kHz */
    TIM4->CCR1 = 0;               /* silent */
    /* PWM mode 1 on CH1 (OC1M=110), output-compare preload. */
    TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC1M) | (6u << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1  |= TIM_CR1_ARPE;
    TIM4->EGR   = TIM_EGR_UG;     /* latch preloaded values */
    TIM4->CR1  |= TIM_CR1_CEN;
}

void buzzer_tone(uint16_t freq_hz)
{
    if (freq_hz < 50u) { buzzer_off(); return; }
    uint32_t arr = TIM4_TICK_HZ / freq_hz;
    if (arr < 2u) arr = 2u;
    TIM4->ARR  = arr - 1u;
    TIM4->CCR1 = arr / 2u;        /* 50% duty */
    TIM4->EGR  = TIM_EGR_UG;
}

void buzzer_off(void)
{
    TIM4->CCR1 = 0;
}

/* C-major boot jingle. Rests use hz = 0. Sum = 3010 ms. */
static const struct { uint16_t hz, ms; } SONG[] = {
    {  523, 180 }, {  659, 180 }, {  784, 180 }, { 1047, 300 }, {    0,  70 },
    { 1319, 200 }, { 1568, 360 }, {    0,  80 },
    { 1319, 180 }, { 1047, 180 }, {  784, 180 }, {  880, 180 }, {  988, 200 },
    { 1047, 540 },
};
#define SONG_LEN ((uint16_t)(sizeof SONG / sizeof SONG[0]))

static uint16_t song_idx;
static uint32_t song_note_ms;
static bool     song_playing;

static void song_play_note(void)
{
    if (SONG[song_idx].hz) buzzer_tone(SONG[song_idx].hz);
    else                   buzzer_off();
}

void buzzer_song_start(uint32_t now_ms)
{
    song_idx     = 0;
    song_note_ms = now_ms;
    song_playing = true;
    song_play_note();
}

void buzzer_song_update(uint32_t now_ms)
{
    if (!song_playing) return;
    if ((now_ms - song_note_ms) < SONG[song_idx].ms) return;  /* note still playing */
    song_idx++;
    if (song_idx >= SONG_LEN) {        /* song done */
        buzzer_off();
        song_playing = false;
        return;
    }
    song_note_ms = now_ms;
    song_play_note();
}

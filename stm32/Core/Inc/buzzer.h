/*
 * buzzer.h: passive buzzer on PB6 (BUZZER_MCU) via TIM4_CH1 PWM.
 * PB6 = Arduino D6 = TIM4_CH1 (AF2). Passive buzzer needs a square-wave tone.
 * Bare-metal TIM4.
 */
#ifndef BUZZER_H_
#define BUZZER_H_

#include <stdint.h>

void buzzer_init(void);
void buzzer_tone(uint16_t freq_hz);          /* start a continuous tone */
void buzzer_off(void);                        /* silence */

/* Non-blocking ~3 s startup melody: call start() once at boot, then update()
 * each main-loop pass until it ends. */
void buzzer_song_start(uint32_t now_ms);
void buzzer_song_update(uint32_t now_ms);

#endif /* BUZZER_H_ */

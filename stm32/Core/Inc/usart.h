/*
 * usart.h
 *
 * LPUART1 wired to the Nucleo-32 G431KB onboard ST-LINK Virtual COM Port (UM2657).
 * Pins: PA2 = LPUART1_TX, PA3 = LPUART1_RX (AF12). Baud 115200, 8-N-1.
 * On the Pi this enumerates as /dev/ttyACM0 over the same USB cable used
 * for SWD flashing.
 *
 * NOTE: if the board variant actually wires the VCP to USART2 on PA2/PA3
 * (AF7), swap LPUART1 -> USART2 in usart.c and stm32g4xx_it.c. Pins stay
 * the same; only the peripheral and AF index change.
 */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;

void MX_LPUART1_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

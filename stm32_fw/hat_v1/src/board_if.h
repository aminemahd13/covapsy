#ifndef BOARD_IF_H
#define BOARD_IF_H

#include <stdbool.h>
#include <stdint.h>

#include "fw_config.h"

/*
 * Hardware abstraction interface.
 * Implement these functions with STM32 HAL calls for your exact pin mapping.
 * hat_v1 baseline: SPI3 on PB3/PB4/PB5, no wired NSS (software NSS).
 */

void Board_Init(void);
uint32_t Board_Millis(void);
void Board_DelayMs(uint32_t delay_ms);

bool Board_SpiReadFrame(uint8_t out_frame[FW_PROTOCOL_FRAME_SIZE]);
void Board_SpiSetReplyFrame(const uint8_t frame[FW_PROTOCOL_FRAME_SIZE]);

void Board_SetPropulsionDuty(float duty_percent);
void Board_SetSteeringDuty(float duty_percent);

float Board_ReadWheelSpeedMps(void);
bool Board_ReadRearObstacle(void);

void Board_Log(const char *message);

#endif /* BOARD_IF_H */

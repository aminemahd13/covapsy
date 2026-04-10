#ifndef BOARD_IF_H
#define BOARD_IF_H

#include <stdbool.h>
#include <stdint.h>

#include "fw_config.h"

/*
 * Hardware abstraction interface.
 * Implement these functions with STM32 HAL calls for your exact pin mapping.
 * usb_v1 baseline: USB serial link on UART/CDC transport.
 */

void Board_Init(void);
uint32_t Board_Millis(void);
void Board_DelayMs(uint32_t delay_ms);

bool Board_UsbReadLine(char *out_line, uint32_t out_line_size);
void Board_UsbWriteLine(const char *line);

void Board_SetPropulsionDuty(float duty_percent);
void Board_SetSteeringDuty(float duty_percent);

float Board_ReadWheelSpeedMps(void);
bool Board_ReadRearObstacle(void);

void Board_Log(const char *message);

#endif /* BOARD_IF_H */

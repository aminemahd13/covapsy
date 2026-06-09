/*
 * dynamixel.h
 *
 * Dynamixel Protocol 2.0 master for the XL430-W250-T on the X_AX12
 * connector of the COVAPSY Interface board.
 *
 * Path: STM32 PA9 (USART1_TX, AF7), J_TX jumper LEFT (STM32 side),
 * Hat, X_SIGNAUX ribbon, Interface board, R5 (1 kohm series),
 * X_AX12 pin 3 (DATA).
 *
 * USART1 runs single-wire half-duplex (USART_CR3 HDSEL=1): TX and RX
 * share PA9, direction switches in hardware.
 *
 * Factory defaults assumed: ID = 1, baud = 57600,
 * Position Control (operating mode 3). Change DXL_ID / DXL_BAUD below
 * if reconfigured.
 */
#ifndef DYNAMIXEL_H_
#define DYNAMIXEL_H_

#include <stdbool.h>
#include <stdint.h>

#define DXL_ID    1
#define DXL_BAUD  57600u

/* XL430-W250-T position 0..4095 over 360 degrees, center 2048.
 * Steering range +-DIR_ANGLE_MAX (18 deg default in moteurs.h). */
#define DXL_POS_CENTER  2048
#define DXL_POS_PER_DEG (4096.0f / 360.0f)

/* Init USART1 (half-duplex), set position-control mode, enable torque,
 * center the servo. */
void dxl_init(void);

/* Disable torque. */
bool dxl_torque_off(void);

/* Enable torque. */
bool dxl_torque_on(void);

/* Write goal position, in [0, 4095]. */
bool dxl_set_goal_position(int32_t position);

/* Convert steering angle (degrees) to a goal position around center and
 * send it. Clamped to +-18 deg. */
void dxl_set_steering_degres(float angle_degre);

/* Send a Robotis PING to id and read the status packet. Returns response
 * byte count (0 = timeout). Bytes copied into resp (up to resp_max). */
uint16_t dxl_ping(uint8_t id, uint8_t *resp, uint16_t resp_max);

/* Re-init USART1 at a different baud rate (for scanning). */
void dxl_set_baud(uint32_t baud);

/* Broadcast ping (ID 254) across plausible baud rates. Responders dump
 * raw bytes via DXL_RX. */
void dxl_scan(void);

#endif /* DYNAMIXEL_H_ */

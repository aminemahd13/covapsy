/*
 * bno055.h: Bosch BNO055 IMU, I2C1 @ 0x28 (bus shared with SH1106 OLED @ 0x3C).
 * NDOF fusion mode. Fused Euler heading (yaw) for display/telemetry only.
 */
#ifndef BNO055_H_
#define BNO055_H_

#include <stdbool.h>
#include <stdint.h>

/* Configure NDOF fusion mode. Returns true if chip ID matched and init writes accepted. */
bool bno055_init(void);

/* Read fused heading in whole degrees [0..359]. Returns false on I2C failure. */
bool bno055_read_heading_deg(int *out_deg);

#endif /* BNO055_H_ */

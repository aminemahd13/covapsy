/*
 * i2c.h: bare-metal I2C1 driver (no HAL).
 * I2C1 on PB7 (SDA, AF4) / PA15 (SCL, AF4), HSI16 clock, 100 kHz.
 * Used by the OLED and the BNO055 IMU. Drives registers directly.
 */
#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>

void i2c1_init(void);

/* Reset (re-init) the I2C1 peripheral when stuck. */
void i2c1_reset(void);

/* Blocking write of n bytes to a 7-bit address. Returns true on ACK. */
bool i2c1_write(uint8_t addr_7bit, const uint8_t *data, uint16_t n);

/* Register read: write reg (no STOP), repeated-START, read n bytes into buf.
 * Returns true on full success. */
bool i2c1_read(uint8_t addr_7bit, uint8_t reg, uint8_t *buf, uint16_t n);

/* Non-blocking DMA write for large bursts (OLED page data).
 * DMA1_Channel1 streams bytes into I2C1->TXDR. One transfer in flight at a
 * time. Caller's buffer must stay valid until the poll reports done. */

/* Init DMA channel and DMAMUX route for I2C1_TX. Call once after i2c1_init(). */
void i2c1_dma_init(void);

/* Start a non-blocking write of n bytes (n <= 255) to a 7-bit address.
 * Returns false if it couldn't start (bus busy, bad args, or one in flight). */
bool i2c1_write_dma_start(uint8_t addr_7bit, const uint8_t *data, uint16_t n);

/* Poll the in-flight DMA write: 1 = done, 0 = streaming, -1 = error (NACK). */
int i2c1_write_dma_poll(void);

/* Abort an in-flight DMA write (flush TXDR, force STOP, clear flags). On timeout. */
void i2c1_dma_abort(void);

/* Hard recovery for a jammed bus (slave holding SDA): 9-clock release + re-init. */
void i2c1_recover(void);

#endif /* I2C_H_ */

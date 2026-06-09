/*
 * BNO055 driver over bare-metal I2C1.
 * Init: configure ACC/GYR/MAG on page 1, return to page 0, set units and
 * normal power, enter NDOF fusion. Heading from Euler-H register
 * (0x1A, little-endian int16, 16 LSB per degree).
 */
#include "bno055.h"
#include "i2c.h"
#include "stm32g4xx_hal.h"   /* HAL_Delay */

#define REG_CHIP_ID          0x00u   /* should read 0xA0 */
#define REG_PAGE_ID          0x07u
#define REG_UNIT_SEL         0x3Bu
#define REG_PWR_MODE         0x3Eu
#define REG_OPR_MODE         0x3Du
#define REG_TEMP_SOURCE      0x40u
#define REG_EUL_HEADING_LSB  0x1Au

#define CHIP_ID_BNO055       0xA0u
#define OPR_MODE_NDOF        0x0Cu

/* Device address: 0x28 default, 0x29 if ADR pin high. */
static uint8_t s_addr = 0x28u;

static bool wr(uint8_t reg, uint8_t val)
{
    uint8_t pkt[2] = { reg, val };
    return i2c1_write(s_addr, pkt, 2);
}

bool bno055_init(void)
{
    /* Probe both addresses (0x28, 0x29) several times; I2C bus shares the noisy OLED line. */
    static const uint8_t addrs[2] = { 0x28u, 0x29u };
    uint8_t id = 0;
    bool found = false;
    for (int attempt = 0; attempt < 10 && !found; attempt++) {
        for (int a = 0; a < 2; a++) {
            if (i2c1_read(addrs[a], REG_CHIP_ID, &id, 1) && id == CHIP_ID_BNO055) {
                s_addr = addrs[a];
                found = true;
                break;
            }
        }
        if (!found) HAL_Delay(50);
    }
    if (!found) return false;

    /* Power-on default is CONFIG mode; config writes below are valid. */
    wr(REG_PAGE_ID, 1);          /* page 1: sensor config */
    wr(0x08, 0x08);              /* ACC_CONFIG */
    wr(0x0A, 0x23);              /* GYR_CONFIG_0 */
    wr(0x0B, 0x00);              /* GYR_CONFIG_1 */
    wr(0x09, 0x1B);              /* MAG_CONFIG */
    wr(REG_PAGE_ID, 0);          /* page 0: operation */
    wr(REG_TEMP_SOURCE, 0x01);
    wr(REG_UNIT_SEL, 0x01);
    wr(REG_PWR_MODE, 0x00);      /* normal */
    HAL_Delay(10);
    wr(REG_OPR_MODE, OPR_MODE_NDOF);
    HAL_Delay(20);               /* CONFIG to fusion mode switch time */
    return true;
}

bool bno055_read_heading_deg(int *out_deg)
{
    uint8_t b[2];
    if (!i2c1_read(s_addr, REG_EUL_HEADING_LSB, b, 2)) return false;
    int16_t raw = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    int deg = raw / 16;          /* 16 LSB per degree */
    deg %= 360;
    if (deg < 0) deg += 360;
    *out_deg = deg;
    return true;
}

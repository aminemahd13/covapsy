#ifndef FW_OLED_H
#define FW_OLED_H

#include <stdbool.h>
#include <stdint.h>

#include "fw_config.h"

typedef enum
{
    FW_OLED_CONTROLLER_SSD1306 = 0,
    FW_OLED_CONTROLLER_SH1106 = 1,
} fw_oled_controller_t;

typedef bool (*fw_oled_i2c_write_fn)(
    uint8_t address_7bit,
    const uint8_t *bytes,
    uint16_t size,
    void *user_data);

typedef struct
{
    fw_oled_i2c_write_fn i2c_write;
    void *user_data;
    uint8_t address_7bit;
    fw_oled_controller_t controller;
    bool ready;
} fw_oled_t;

void FwOled_InitState(
    fw_oled_t *oled,
    fw_oled_i2c_write_fn i2c_write,
    void *user_data);

bool FwOled_ProbeAndInit(fw_oled_t *oled, const uint8_t *candidate_addresses, uint8_t candidate_count);
bool FwOled_RenderLines(
    fw_oled_t *oled,
    const char lines[FW_LCD_LINE_COUNT][FW_LCD_LINE_CHARS + 1u]);

#endif /* FW_OLED_H */

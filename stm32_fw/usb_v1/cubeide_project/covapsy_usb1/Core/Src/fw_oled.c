#include "fw_oled.h"

#include <string.h>

static uint8_t to_upper_ascii(uint8_t ch)
{
    if (ch >= 'a' && ch <= 'z')
    {
        return (uint8_t)(ch - ('a' - 'A'));
    }
    return ch;
}

static bool write_bytes(fw_oled_t *oled, const uint8_t *bytes, uint16_t size)
{
    if (oled == 0 || oled->i2c_write == 0 || bytes == 0 || size == 0u)
    {
        return false;
    }
    return oled->i2c_write(oled->address_7bit, bytes, size, oled->user_data);
}

static bool write_cmd(fw_oled_t *oled, uint8_t cmd)
{
    uint8_t frame[2];
    frame[0] = 0x00u;
    frame[1] = cmd;
    return write_bytes(oled, frame, (uint16_t)sizeof(frame));
}

static bool write_page_data(fw_oled_t *oled, const uint8_t *cols128)
{
    uint8_t frame[1u + 128u];
    if (cols128 == 0)
    {
        return false;
    }
    frame[0] = 0x40u;
    (void)memcpy(&frame[1], cols128, 128u);
    return write_bytes(oled, frame, (uint16_t)sizeof(frame));
}

static void glyph_5x7(uint8_t ch, uint8_t out[5])
{
    out[0] = 0x00u;
    out[1] = 0x00u;
    out[2] = 0x00u;
    out[3] = 0x00u;
    out[4] = 0x00u;

    ch = to_upper_ascii(ch);
    switch (ch)
    {
    case '0':
        out[0] = 0x3Eu; out[1] = 0x51u; out[2] = 0x49u; out[3] = 0x45u; out[4] = 0x3Eu;
        return;
    case '1':
        out[0] = 0x00u; out[1] = 0x42u; out[2] = 0x7Fu; out[3] = 0x40u; out[4] = 0x00u;
        return;
    case '2':
        out[0] = 0x42u; out[1] = 0x61u; out[2] = 0x51u; out[3] = 0x49u; out[4] = 0x46u;
        return;
    case '3':
        out[0] = 0x21u; out[1] = 0x41u; out[2] = 0x45u; out[3] = 0x4Bu; out[4] = 0x31u;
        return;
    case '4':
        out[0] = 0x18u; out[1] = 0x14u; out[2] = 0x12u; out[3] = 0x7Fu; out[4] = 0x10u;
        return;
    case '5':
        out[0] = 0x27u; out[1] = 0x45u; out[2] = 0x45u; out[3] = 0x45u; out[4] = 0x39u;
        return;
    case '6':
        out[0] = 0x3Cu; out[1] = 0x4Au; out[2] = 0x49u; out[3] = 0x49u; out[4] = 0x30u;
        return;
    case '7':
        out[0] = 0x01u; out[1] = 0x71u; out[2] = 0x09u; out[3] = 0x05u; out[4] = 0x03u;
        return;
    case '8':
        out[0] = 0x36u; out[1] = 0x49u; out[2] = 0x49u; out[3] = 0x49u; out[4] = 0x36u;
        return;
    case '9':
        out[0] = 0x06u; out[1] = 0x49u; out[2] = 0x49u; out[3] = 0x29u; out[4] = 0x1Eu;
        return;
    case 'A':
        out[0] = 0x7Eu; out[1] = 0x11u; out[2] = 0x11u; out[3] = 0x11u; out[4] = 0x7Eu;
        return;
    case 'B':
        out[0] = 0x7Fu; out[1] = 0x49u; out[2] = 0x49u; out[3] = 0x49u; out[4] = 0x36u;
        return;
    case 'C':
        out[0] = 0x3Eu; out[1] = 0x41u; out[2] = 0x41u; out[3] = 0x41u; out[4] = 0x22u;
        return;
    case 'D':
        out[0] = 0x7Fu; out[1] = 0x41u; out[2] = 0x41u; out[3] = 0x22u; out[4] = 0x1Cu;
        return;
    case 'E':
        out[0] = 0x7Fu; out[1] = 0x49u; out[2] = 0x49u; out[3] = 0x49u; out[4] = 0x41u;
        return;
    case 'F':
        out[0] = 0x7Fu; out[1] = 0x09u; out[2] = 0x09u; out[3] = 0x09u; out[4] = 0x01u;
        return;
    case 'G':
        out[0] = 0x3Eu; out[1] = 0x41u; out[2] = 0x49u; out[3] = 0x49u; out[4] = 0x7Au;
        return;
    case 'H':
        out[0] = 0x7Fu; out[1] = 0x08u; out[2] = 0x08u; out[3] = 0x08u; out[4] = 0x7Fu;
        return;
    case 'I':
        out[0] = 0x00u; out[1] = 0x41u; out[2] = 0x7Fu; out[3] = 0x41u; out[4] = 0x00u;
        return;
    case 'J':
        out[0] = 0x20u; out[1] = 0x40u; out[2] = 0x41u; out[3] = 0x3Fu; out[4] = 0x01u;
        return;
    case 'K':
        out[0] = 0x7Fu; out[1] = 0x08u; out[2] = 0x14u; out[3] = 0x22u; out[4] = 0x41u;
        return;
    case 'L':
        out[0] = 0x7Fu; out[1] = 0x40u; out[2] = 0x40u; out[3] = 0x40u; out[4] = 0x40u;
        return;
    case 'M':
        out[0] = 0x7Fu; out[1] = 0x02u; out[2] = 0x0Cu; out[3] = 0x02u; out[4] = 0x7Fu;
        return;
    case 'N':
        out[0] = 0x7Fu; out[1] = 0x04u; out[2] = 0x08u; out[3] = 0x10u; out[4] = 0x7Fu;
        return;
    case 'O':
        out[0] = 0x3Eu; out[1] = 0x41u; out[2] = 0x41u; out[3] = 0x41u; out[4] = 0x3Eu;
        return;
    case 'P':
        out[0] = 0x7Fu; out[1] = 0x09u; out[2] = 0x09u; out[3] = 0x09u; out[4] = 0x06u;
        return;
    case 'Q':
        out[0] = 0x3Eu; out[1] = 0x41u; out[2] = 0x51u; out[3] = 0x21u; out[4] = 0x5Eu;
        return;
    case 'R':
        out[0] = 0x7Fu; out[1] = 0x09u; out[2] = 0x19u; out[3] = 0x29u; out[4] = 0x46u;
        return;
    case 'S':
        out[0] = 0x46u; out[1] = 0x49u; out[2] = 0x49u; out[3] = 0x49u; out[4] = 0x31u;
        return;
    case 'T':
        out[0] = 0x01u; out[1] = 0x01u; out[2] = 0x7Fu; out[3] = 0x01u; out[4] = 0x01u;
        return;
    case 'U':
        out[0] = 0x3Fu; out[1] = 0x40u; out[2] = 0x40u; out[3] = 0x40u; out[4] = 0x3Fu;
        return;
    case 'V':
        out[0] = 0x1Fu; out[1] = 0x20u; out[2] = 0x40u; out[3] = 0x20u; out[4] = 0x1Fu;
        return;
    case 'W':
        out[0] = 0x3Fu; out[1] = 0x40u; out[2] = 0x38u; out[3] = 0x40u; out[4] = 0x3Fu;
        return;
    case 'X':
        out[0] = 0x63u; out[1] = 0x14u; out[2] = 0x08u; out[3] = 0x14u; out[4] = 0x63u;
        return;
    case 'Y':
        out[0] = 0x07u; out[1] = 0x08u; out[2] = 0x70u; out[3] = 0x08u; out[4] = 0x07u;
        return;
    case 'Z':
        out[0] = 0x61u; out[1] = 0x51u; out[2] = 0x49u; out[3] = 0x45u; out[4] = 0x43u;
        return;
    case ':':
        out[0] = 0x00u; out[1] = 0x36u; out[2] = 0x36u; out[3] = 0x00u; out[4] = 0x00u;
        return;
    case '.':
        out[0] = 0x00u; out[1] = 0x60u; out[2] = 0x60u; out[3] = 0x00u; out[4] = 0x00u;
        return;
    case '/':
        out[0] = 0x20u; out[1] = 0x10u; out[2] = 0x08u; out[3] = 0x04u; out[4] = 0x02u;
        return;
    case '-':
        out[0] = 0x08u; out[1] = 0x08u; out[2] = 0x08u; out[3] = 0x08u; out[4] = 0x08u;
        return;
    case '_':
        out[0] = 0x40u; out[1] = 0x40u; out[2] = 0x40u; out[3] = 0x40u; out[4] = 0x40u;
        return;
    case ' ':
        return;
    default:
        out[0] = 0x3Eu; out[1] = 0x22u; out[2] = 0x22u; out[3] = 0x22u; out[4] = 0x3Eu;
        return;
    }
}

static bool oled_set_page_address(fw_oled_t *oled, uint8_t page)
{
    uint8_t col_offset = 0u;
    if (oled == 0)
    {
        return false;
    }
    if (oled->controller == FW_OLED_CONTROLLER_SH1106)
    {
        col_offset = 2u;
    }

    if (!write_cmd(oled, (uint8_t)(0xB0u + (page & 0x0Fu))))
    {
        return false;
    }
    if (!write_cmd(oled, (uint8_t)(0x00u + (col_offset & 0x0Fu))))
    {
        return false;
    }
    if (!write_cmd(oled, (uint8_t)(0x10u + ((col_offset >> 4) & 0x0Fu))))
    {
        return false;
    }
    return true;
}

static bool oled_clear_all_pages(fw_oled_t *oled)
{
    uint8_t zeros[128];
    uint8_t page;
    (void)memset(zeros, 0, sizeof(zeros));
    for (page = 0u; page < 8u; ++page)
    {
        if (!oled_set_page_address(oled, page))
        {
            return false;
        }
        if (!write_page_data(oled, zeros))
        {
            return false;
        }
    }
    return true;
}

static bool oled_send_init_sequence(fw_oled_t *oled, fw_oled_controller_t controller)
{
    static const uint8_t ssd1306_seq[] = {
        0xAEu, 0xD5u, 0x80u, 0xA8u, 0x3Fu, 0xD3u, 0x00u, 0x40u,
        0xA1u, 0xC8u, 0xDAu, 0x12u, 0x81u, 0x7Fu, 0xD9u, 0xF1u,
        0xDBu, 0x20u, 0xA4u, 0xA6u, 0x20u, 0x02u, 0x8Du, 0x14u, 0xAFu
    };
    static const uint8_t sh1106_seq[] = {
        0xAEu, 0xD5u, 0x80u, 0xA8u, 0x3Fu, 0xD3u, 0x00u, 0x40u,
        0xA1u, 0xC8u, 0xDAu, 0x12u, 0x81u, 0x7Fu, 0xD9u, 0x22u,
        0xDBu, 0x20u, 0xA4u, 0xA6u, 0xAFu
    };
    const uint8_t *seq = ssd1306_seq;
    uint16_t seq_len = (uint16_t)sizeof(ssd1306_seq);
    uint16_t i;

    if (controller == FW_OLED_CONTROLLER_SH1106)
    {
        seq = sh1106_seq;
        seq_len = (uint16_t)sizeof(sh1106_seq);
    }

    for (i = 0u; i < seq_len; ++i)
    {
        if (!write_cmd(oled, seq[i]))
        {
            return false;
        }
    }
    return true;
}

void FwOled_InitState(
    fw_oled_t *oled,
    fw_oled_i2c_write_fn i2c_write,
    void *user_data)
{
    if (oled == 0)
    {
        return;
    }
    (void)memset(oled, 0, sizeof(*oled));
    oled->i2c_write = i2c_write;
    oled->user_data = user_data;
    oled->controller = FW_OLED_CONTROLLER_SSD1306;
    oled->ready = false;
}

bool FwOled_ProbeAndInit(fw_oled_t *oled, const uint8_t *candidate_addresses, uint8_t candidate_count)
{
    static const fw_oled_controller_t controllers[] = {
        FW_OLED_CONTROLLER_SSD1306,
        FW_OLED_CONTROLLER_SH1106,
    };
    uint8_t idx;
    uint8_t c_idx;

    if (oled == 0 || candidate_addresses == 0 || candidate_count == 0u)
    {
        return false;
    }

    oled->ready = false;
    for (idx = 0u; idx < candidate_count; ++idx)
    {
        for (c_idx = 0u; c_idx < (uint8_t)(sizeof(controllers) / sizeof(controllers[0])); ++c_idx)
        {
            oled->address_7bit = candidate_addresses[idx];
            oled->controller = controllers[c_idx];
            if (!oled_send_init_sequence(oled, oled->controller))
            {
                continue;
            }
            if (!oled_clear_all_pages(oled))
            {
                continue;
            }
            oled->ready = true;
            return true;
        }
    }
    return false;
}

bool FwOled_RenderLines(
    fw_oled_t *oled,
    const char lines[FW_LCD_LINE_COUNT][FW_LCD_LINE_CHARS + 1u])
{
    uint8_t page;
    if (oled == 0 || lines == 0 || !oled->ready)
    {
        return false;
    }

    for (page = 0u; page < FW_LCD_LINE_COUNT; ++page)
    {
        uint8_t data[128];
        uint8_t col = 0u;
        uint8_t i;

        (void)memset(data, 0, sizeof(data));
        for (i = 0u; i < FW_LCD_LINE_CHARS; ++i)
        {
            uint8_t glyph[5];
            uint8_t ch = (uint8_t)lines[page][i];
            if (ch == '\0')
            {
                ch = ' ';
            }
            glyph_5x7(ch, glyph);
            if (col < 123u)
            {
                data[col++] = glyph[0];
                data[col++] = glyph[1];
                data[col++] = glyph[2];
                data[col++] = glyph[3];
                data[col++] = glyph[4];
                data[col++] = 0x00u;
            }
        }

        if (!oled_set_page_address(oled, page))
        {
            oled->ready = false;
            return false;
        }
        if (!write_page_data(oled, data))
        {
            oled->ready = false;
            return false;
        }
    }
    return true;
}

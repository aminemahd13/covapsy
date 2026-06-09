/*
 * sh1106.h
 *
 * Driver for the SH1106 128x64 OLED (silkscreen "TF051"). I2C address 0x3C.
 *
 * SH1106 differs from SSD1306:
 *   - 132 columns of RAM, only middle 128 visible, so offset column index by +2.
 *   - No horizontal addressing mode. Page-based writes: set page address,
 *     set column low+high, then stream a row of 128 bytes.
 *
 * Pins:
 *   I2C1_SDA = PB7  (AF4), via Arduino A4 / D4 socket
 *   I2C1_SCL = PA15 (AF4), jumper to Hat SCL trace (R6 SCL pad)
 *
 * Reference: Covapsy-prof/Software/programmes_capteurs_afficheur_AX12/test_OLED_TF051.py
 *            (luma.oled.device.sh1106 at address 0x3C).
 */
#ifndef SH1106_H_
#define SH1106_H_

#include <stdbool.h>
#include <stdint.h>

#define SH1106_W        128
#define SH1106_H        64
#define SH1106_PAGES    (SH1106_H / 8)
#define SH1106_FB_BYTES (SH1106_W * SH1106_PAGES)
#define SH1106_COL_OFF  2u           /* columns 2..129 are visible */

/* True if init succeeded (display ack'd at 0x3C). */
bool sh1106_init(void);

/* Clear the RAM framebuffer (does not flush). */
void sh1106_clear(void);

/* Set cursor in character cells, col 0..20, row 0..7. */
void sh1106_set_cursor(uint8_t col, uint8_t row);

/* Print string at cursor, wraps to next line on overflow. */
void sh1106_puts(const char *s);

/* Clear a single row then print. */
void sh1106_print_row(uint8_t row, const char *s);

/* Push the top 4 pages (status area). */
bool sh1106_show(void);

/* Push all 8 pages. Used at init to clear power-on RAM garbage. */
bool sh1106_show_full(void);

/* Push one page (0..SH1106_PAGES-1). Spreads a full refresh across loop
 * iterations (~12 ms/page) instead of blocking ~100 ms at once. */
bool sh1106_show_page(uint8_t page);

/* Non-blocking DMA refresh.
 * Streams the whole framebuffer via DMA for the per-page data bursts; only
 * the page-address setup (~0.6 ms) is polled, not the ~12 ms/page transfer.
 * Usage:
 *     if (sh1106_refresh_idle()) { ...rebuild rows...; sh1106_refresh_begin(); }
 *     sh1106_refresh_poll();   // call every loop pass
 */
void sh1106_refresh_begin(void);   /* start pushing the current framebuffer */
void sh1106_refresh_poll(void);    /* advance one non-blocking step per call */
bool sh1106_refresh_idle(void);    /* true when no frame is being pushed */

#endif /* SH1106_H_ */

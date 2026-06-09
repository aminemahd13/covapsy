/*
 * cmd_parser.c
 *
 * One-byte RX from LPUART1 into a 256-byte ring buffer. cmd_parser_pop()
 * parses LF-terminated lines.
 */
#include "cmd_parser.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

#define RX_RING_SIZE   256u   /* must be power of two */
#define LINE_MAX_LEN    32u

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head;   /* written by ISR */
static volatile uint16_t rx_tail;   /* read by main */

static uint8_t  rx_byte;            /* one-byte RX target */

static char    line_buf[LINE_MAX_LEN];
static uint8_t line_len;

/* HAL RX-complete callback. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        uint16_t next = (rx_head + 1u) & (RX_RING_SIZE - 1u);
        if (next != rx_tail) {
            rx_ring[rx_head] = rx_byte;
            rx_head = next;
        }
        /* Re-arm RX. */
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

/* Clear error flags and re-arm RX after a UART error (overrun, noise, framing).
 * Without this the first error stops all command reception. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF
                                   | UART_CLEAR_FEF  | UART_CLEAR_PEF);
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

void cmd_parser_init(void)
{
    rx_head = rx_tail = 0;
    line_len = 0;
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

/* Re-arm RX if it fell out of interrupt mode. Call every main-loop pass. */
void cmd_parser_rx_ensure(void)
{
    if (huart1.RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

static bool parse_line(const char *s, uint8_t n, cmd_t *out)
{
    if (n == 0) return false;
    char c = s[0];
    if (c == 'X' || c == 'x') {
        out->kind = CMD_STOP;
        out->value = 0.0f;
        return true;
    }
    /* "CAL": ESC throttle-range calibration. */
    if (n >= 3 && (s[0] == 'C' || s[0] == 'c')
              && (s[1] == 'A' || s[1] == 'a')
              && (s[2] == 'L' || s[2] == 'l')) {
        out->kind = CMD_CAL;
        out->value = 0.0f;
        return true;
    }
    /* "REV": brake-release-reverse sequence. */
    if (n >= 3 && (s[0] == 'R' || s[0] == 'r')
              && (s[1] == 'E' || s[1] == 'e')
              && (s[2] == 'V' || s[2] == 'v')) {
        out->kind = CMD_REV;
        out->value = 0.0f;
        return true;
    }
    /* "DPING": Dynamixel ping diagnostic. */
    if (n >= 5 && (s[0] == 'D' || s[0] == 'd')
              && (s[1] == 'P' || s[1] == 'p')
              && (s[2] == 'I' || s[2] == 'i')
              && (s[3] == 'N' || s[3] == 'n')
              && (s[4] == 'G' || s[4] == 'g')) {
        out->kind = CMD_DPING;
        out->value = 0.0f;
        return true;
    }
    /* "DSCAN": Dynamixel baud-rate and ID broadcast sweep. */
    if (n >= 5 && (s[0] == 'D' || s[0] == 'd')
              && (s[1] == 'S' || s[1] == 's')
              && (s[2] == 'C' || s[2] == 'c')
              && (s[3] == 'A' || s[3] == 'a')
              && (s[4] == 'N' || s[4] == 'n')) {
        out->kind = CMD_DSCAN;
        out->value = 0.0f;
        return true;
    }
    /* "DIR<us>": raw steering pulse in us, bypasses the +-18 deg clamp.
     * Checked before the single-char block. */
    if (n >= 4 && (s[0] == 'D' || s[0] == 'd')
              && (s[1] == 'I' || s[1] == 'i')
              && (s[2] == 'R' || s[2] == 'r')) {
        char buf[LINE_MAX_LEN + 1];
        if (n > LINE_MAX_LEN) n = LINE_MAX_LEN;
        memcpy(buf, s + 3, n - 3);
        buf[n - 3] = '\0';
        char *end = NULL;
        float v = strtof(buf, &end);
        if (end == buf) return false;
        out->kind = CMD_DIRPWM;
        out->value = v;
        return true;
    }
    if ((c == 'S' || c == 's' || c == 'V' || c == 'v' || c == 'P' || c == 'p') && n >= 2) {
        char buf[LINE_MAX_LEN + 1];
        if (n > LINE_MAX_LEN) n = LINE_MAX_LEN;
        memcpy(buf, s + 1, n - 1);
        buf[n - 1] = '\0';
        char *end = NULL;
        float v = strtof(buf, &end);
        if (end == buf) return false;        /* no digits parsed */
        switch (c) {
            case 'S': case 's': out->kind = CMD_STEER; break;
            case 'V': case 'v': out->kind = CMD_SPEED; break;
            case 'P': case 'p': out->kind = CMD_PWM;   break;
            default: return false;
        }
        out->value = v;
        return true;
    }
    /* "WD0" disables watchdog, "WD1" re-enables. */
    if (n >= 3 && (s[0] == 'W' || s[0] == 'w') && (s[1] == 'D' || s[1] == 'd')) {
        out->kind  = CMD_WD;
        out->value = (s[2] == '0') ? 0.0f : 1.0f;
        return true;
    }
    return false;
}

bool cmd_parser_pop(cmd_t *out)
{
    /* Drain ring; emit at most one parsed command per call. */
    while (rx_tail != rx_head) {
        uint8_t b = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1u) & (RX_RING_SIZE - 1u);

        if (b == '\r') continue;             /* ignore CR */
        if (b == '\n') {
            bool ok = parse_line(line_buf, line_len, out);
            line_len = 0;
            if (ok) return true;
            continue;                        /* dropped junk line */
        }
        if (line_len < LINE_MAX_LEN - 1) {
            line_buf[line_len++] = (char)b;
        } else {
            /* Overflow: drop the line and resync at next '\n'. */
            line_len = 0;
        }
    }
    return false;
}

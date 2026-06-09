/*
 * cmd_parser.h
 *
 * ASCII line-based command parser for Pi to STM32 over LPUART1 (VCP).
 *
 * Protocol (case-sensitive, '\n' terminated, '\r' tolerated):
 *   S<float>\n   set steering, degrees, clamped [-18, +18]
 *   V<float>\n   set speed,    m/s,     clamped [-V_MAX_HARD, +V_MAX_SOFT]
 *   X\n          stop (speed = 0, steering = 0)
 *
 * Any other line is dropped.
 *
 * UART RX interrupt fills a single-byte FIFO; main loop drains it line-by-line via cmd_parser_pop().
 */
#ifndef CMD_PARSER_H_
#define CMD_PARSER_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    CMD_NONE  = 0,
    CMD_STEER = 1,
    CMD_SPEED = 2,
    CMD_STOP  = 3,
    CMD_CAL   = 4,   /* "CAL\n" run ESC throttle-range calibration */
    CMD_PWM   = 5,   /* "P<us>\n" raw ESC pulse width in us, bypasses speed mapping */
    CMD_WD    = 6,   /* "WD0\n"/"WD1\n" disable/enable watchdog */
    CMD_REV   = 7,   /* "REV\n" brake-release-reverse sequence */
    CMD_DPING = 8,   /* "DPING\n" Robotis ping to Dynamixel ID 1, dump response */
    CMD_DSCAN = 9,   /* "DSCAN\n" sweep baud rates with broadcast ping */
    CMD_DIRPWM = 10, /* "DIR<us>\n" raw steering pulse (us), bypasses the +-18deg clamp */
} cmd_kind_t;

typedef struct {
    cmd_kind_t kind;
    float      value;   // degrees for STEER, m/s for SPEED, 0 for STOP
} cmd_t;

/* Init parser and arm UART RX in interrupt mode. */
void cmd_parser_init(void);

/* Drain RX FIFO. Returns true and fills *out on a complete command, else false. Non-blocking. */
bool cmd_parser_pop(cmd_t *out);

/* Re-arm RX interrupt after an unhandled UART error. Call once per main-loop pass. */
void cmd_parser_rx_ensure(void);

#endif /* CMD_PARSER_H_ */

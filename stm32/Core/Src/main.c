/*
 * main.c
 *
 * COVAPSY new_car STM32 motor-test firmware.
 *
 * - PWM from moteurs.c: TIM1_CH1/PA8 ESC, TIM1_CH4/PA11 steering servo.
 * - LPUART1 (PA2/PA3) via the Nucleo ST-LINK VCP, /dev/ttyACM0 on the Pi.
 * - ASCII line protocol decoded by cmd_parser.c.
 * - Watchdog: 250 ms of silence sets neutral output.
 * - TX boot banner, heartbeats, and per-command ACK.
 * - LED LD2 on PB8 toggles every 250 ms.
 */
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"
#include "moteurs.h"
#include "cmd_parser.h"
#include "sh1106.h"
#include "dynamixel.h"
#include "adc.h"
#include "bno055.h"
#include "buzzer.h"
#include "encoder.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define CMD_WATCHDOG_MS  250u
#define LED_PERIOD_MS    250u
#define HB_PERIOD_MS     500u
#define OLED_PERIOD_MS   200u
#define ESC_ARM_SILENCE_MS 2000u   /* hold PWM line low 2s at boot, then step to 1500us so the ESC sees a signal-arrives-at-neutral edge */

/* Rear IR (Sharp GP2xx on X_IR_G, PA1/ADC1_IN2). Raw rises as an obstacle
 * nears; >= this means something within ~25 cm behind. Tune against the
 * live `rear=` telemetry value. */
#define REAR_IR_BLOCK_RAW 1200u

/* Wheel-encoder calibration: opto-fork pulses per metre. Placeholder; calibrate
 * by pushing the car 1.00 m and reading the `tot=` delta. Only scales the
 * reported SPEED in m/s; stuck detection uses raw pulses/s. */
#define ENC_PULSES_PER_M 200u

#define LED_GPIO_PORT    GPIOB
#define LED_GPIO_PIN     GPIO_PIN_8        /* Nucleo-32 G431KB user LED (LD2) */

/* ESC pulse-width endpoints (us) for calibration. Hobbywing WP-1060 re-records
 * its throttle range from a max/min/neutral sweep. */
#define PWM_PROP_MAX_US  2000u
#define PWM_PROP_MIN_US  1000u
#define PWM_PROP_MID_US  1500u
#define CAL_PHASE_MS     4000u             /* hold each phase 4 s */

/* Raw steering-pulse (DIR<us>) clamp for servo centering. Wider than the
 * +-18 deg (1100..1660 us) driving range to allow the full servo span when
 * finding a mis-mounted center. */
#define DIR_CAL_MIN_US   1000u
#define DIR_CAL_MAX_US   2000u

void SystemClock_Config(void);

static void dbg_puts(const char *s)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)s, strlen(s), 50);
}

static void dbg_send_boot(void)
{
    dbg_puts("BOOT new_car v0.2 LPUART1 PA2/PA3\n");
}

static void dbg_send_ok(const cmd_t *c)
{
    char buf[40];
    const char *kind = "?";
    switch (c->kind) {
        case CMD_STEER: kind = "S"; break;
        case CMD_SPEED: kind = "V"; break;
        case CMD_PWM:   kind = "P"; break;
        case CMD_WD:    kind = "WD"; break;
        case CMD_STOP:  kind = "X"; break;
        case CMD_CAL:   kind = "CAL"; break;
        case CMD_REV:   kind = "REV"; break;
        case CMD_DPING: kind = "DPING"; break;
        case CMD_DSCAN: kind = "DSCAN"; break;
        case CMD_DIRPWM: kind = "DIR"; break;
        default: break;
    }
    snprintf(buf, sizeof buf, "OK %s %+.2f\n", kind, c->value);
    dbg_puts(buf);
}

static void dbg_send_hb(uint32_t ms, bool oled_ok,
                        uint16_t rear_raw, int rear_blocked,
                        int imu_ok, int heading_deg,
                        uint32_t wheel_pps, uint32_t speed_mm_s)
{
    uint32_t ccr_prop = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint32_t ccr_dir  = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    char buf[144];
    snprintf(buf, sizeof buf,
             "HB %lu prop=%lu us dir=%lu us oled=%s rear=%u rblk=%d head=%d "
             "enc=%lu spd=%lu tot=%lu\n",
             (unsigned long)ms, (unsigned long)ccr_prop, (unsigned long)ccr_dir,
             oled_ok ? "yes" : "no",
             (unsigned)rear_raw, rear_blocked, imu_ok ? heading_deg : -1,
             (unsigned long)wheel_pps, (unsigned long)speed_mm_s,
             (unsigned long)encoder_count());
    dbg_puts(buf);
}

/* Pad a string with trailing spaces to exactly 21 chars (OLED row width) so a
 * shorter value never leaves stale glyphs from the previous frame. */
static void pad21(char *s)
{
    size_t n = strlen(s);
    if (n >= 21u) { s[21] = '\0'; return; }
    while (n < 21u) s[n++] = ' ';
    s[n] = '\0';
}

static void led_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin   = LED_GPIO_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &g);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
}

/* Set ESC pulse width in us directly via TIM1_CH1 CCR. Used by calibration. */
static void prop_set_us(uint32_t us)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, us);
}

/* Hobbywing WP-1060 forward-to-reverse double-tap. The ESC reverses only in a
 * Forward/Brake/Reverse running mode and only after: stop, reverse-tap (brake),
 * neutral, reverse again. Reads the wheel encoder across the reverse run to
 * report whether the wheels turned (0 pulses means reverse is disabled).
 * Steering is left untouched. */
static void esc_reverse_sequence(void)
{
    char b[48];

    /* 1) Brake tap while still moving forward; registers a discrete brake event. */
    dbg_puts("REV_BRAKE 1150us 300ms\n");
    prop_set_us(1150);
    HAL_Delay(300);

    /* 2) Short neutral blip. */
    dbg_puts("REV_NEUTRAL 1500us 150ms\n");
    prop_set_us(1500);
    HAL_Delay(150);

    /* 3) Reverse input. Measure wheel motion via the encoder. */
    uint32_t enc0 = encoder_count();
    dbg_puts("REV_RUN 1250us 3000ms\n");
    prop_set_us(1250);
    HAL_Delay(3000);
    uint32_t moved = encoder_count() - enc0;

    /* 4) Back to neutral. */
    prop_set_us(1500);
    HAL_Delay(500);

    snprintf(b, sizeof b, "REV_RUN moved=%lu pulses\n", (unsigned long)moved);
    dbg_puts(b);
    dbg_puts(moved ? "REV_DONE (wheels turned)\n"
                   : "REV_DONE (NO motion -> enable REVERSE in the ESC running mode!)\n");
}

/* ESC throttle-range calibration sweep. ESC must be in learn-range mode
 * (Hobbywing WP-1060: hold SET while powering on until the LED flashes, then
 * release). Steering held centered throughout. */
static void esc_calibrate(void)
{
    dbg_puts("CAL_START hold ESC SET button while powering it on\n");
    set_direction_degres(0.0f);

    dbg_puts("CAL_PHASE_MAX 2000us 4s\n");
    prop_set_us(PWM_PROP_MAX_US);
    HAL_Delay(CAL_PHASE_MS);

    dbg_puts("CAL_PHASE_MIN 1000us 4s\n");
    prop_set_us(PWM_PROP_MIN_US);
    HAL_Delay(CAL_PHASE_MS);

    dbg_puts("CAL_PHASE_MID 1500us 2s\n");
    prop_set_us(PWM_PROP_MID_US);
    HAL_Delay(2000u);

    dbg_puts("CAL_DONE\n");
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    led_init();
    MX_TIM1_Init();
    MX_LPUART1_UART_Init();
    i2c1_init();
    i2c1_dma_init();        /* DMA1_Ch1 for non-blocking OLED page bursts */

    Propulsion_init();
    Direction_init();
    /* ESC arming: start with PWM low (CCR=0 holds the pin low all period, no
     * pulse). After ESC_ARM_SILENCE_MS, step to the neutral pulse width. Many
     * ESCs (Hobbywing WP-1060) need this signal-arrives-at-neutral edge to arm. */
    prop_set_us(0u);
    set_direction_degres(0.0f);

    /* Dynamixel XL430 on X_AX12 (USART1 half-duplex, PA9 AF7). Steering
     * commands drive both this and the TIM1_CH4 PWM line. */
    dxl_init();
    dbg_puts("DXL init done\n");

    cmd_parser_init();

    dbg_send_boot();

    /* Let the Mezzanine I2C devices (OLED, BNO055) finish power-on before the
     * scan; the BNO055 needs ~650 ms after VDD. */
    HAL_Delay(700);

    /* i2c1_init() performs a 9-clock bus recovery. */

    /* Debounced peripheral I2C scan: probe each address 5x, report it only if
     * it ACKs >=4 times. Filters false-positives (expect OLED 0x3C, IMU if
     * present). */
    dbg_puts("I2C scan (peripheral x5): ");
    {
        char hexbuf[12];
        bool any = false;
        for (uint8_t a = 0x08; a <= 0x77; a++) {
            int acks = 0;
            for (int t = 0; t < 5; t++) {
                uint8_t dummy = 0x00;
                if (i2c1_write(a, &dummy, 1)) acks++;
            }
            if (acks >= 4) {
                snprintf(hexbuf, sizeof hexbuf, "0x%02X ", a);
                dbg_puts(hexbuf);
                any = true;
            }
        }
        if (!any) dbg_puts("(none)");
        dbg_puts("\n");
    }

    /* Give the SH1106 a generous post-power-on settling time and retry once. */
    HAL_Delay(250);
    bool oled_ok = sh1106_init();
    if (!oled_ok) {
        dbg_puts("OLED init #1 failed, retrying in 200 ms\n");
        HAL_Delay(200);
        oled_ok = sh1106_init();
    }
    if (oled_ok) {
        sh1106_clear();
        sh1106_show_full();     /* blank the screen until status is drawn */
        dbg_puts("OLED ready\n");
    } else {
        dbg_puts("OLED init failed (no ACK at 0x3C)\n");
    }

    /* Rear IR on ADC1_IN2 / PA1 (Sharp GP2xx via X_IR_G) for reverse gating. */
    dbg_puts("ADC init...\n");
    adc_rear_ir_init();
    dbg_puts("ADC ok\n");

    /* Buzzer on TIM4_CH1 / PB6. */
    dbg_puts("BUZ init...\n");
    buzzer_init();
    buzzer_song_start(HAL_GetTick());   /* ~3 s startup song, non-blocking */
    dbg_puts("BUZ ok\n");

    /* BNO055 IMU on I2C1 @ 0x28 (shares the OLED bus). Heading for telemetry
     * and OLED only; does not influence driving. */
    dbg_puts("IMU init...\n");
    bool imu_ok = bno055_init();
    dbg_puts(imu_ok ? "IMU ready (BNO055 NDOF)\n" : "IMU init failed\n");

    /* Optical wheel encoder (FOURCHE) on PA0 / TIM2: speed and stuck detection. */
    encoder_init();
    dbg_puts("ENC ready (PA0/TIM2)\n");

    uint32_t now             = HAL_GetTick();
    uint32_t last_cmd_ms     = now;
    uint32_t last_led_ms     = now;
    uint32_t last_hb_ms      = now;
    uint32_t last_oled_ms    = now;
    int      watchdog_armed  = 0;       /* edge-triggered stale flag */
    int      watchdog_enabled = 1;       /* WD0 disables, WD1 re-enables */
    int      esc_armed_neutral = 0;     /* set true after the boot-silence window */
    uint32_t last_adc_ms     = now;
    uint32_t last_imu_ms     = now;
    uint16_t rear_ir_raw     = 0;
    int      rear_blocked    = 0;
    int      imu_heading     = 0;
    uint32_t last_enc_ms     = now;
    uint32_t last_enc_count  = 0;
    uint32_t wheel_pps       = 0;
    uint32_t speed_mm_s      = 0;

    for (;;)
    {
        now = HAL_GetTick();

        buzzer_song_update(now);   /* advance the non-blocking startup melody */

        /* Boot-silence to neutral: PWM line low until ESC_ARM_SILENCE_MS, then
         * snap to the neutral pulse for a clean signal-arrives-at-neutral edge. */
        if (!esc_armed_neutral && now >= ESC_ARM_SILENCE_MS) {
            set_vitesse_m_s(0.0f);
            esc_armed_neutral = 1;
            dbg_puts("ESC_ARM neutral on\n");
            last_cmd_ms = now;
        }

        cmd_parser_rx_ensure();   /* restart RX if a UART error abandoned it */

        cmd_t c;
        if (cmd_parser_pop(&c))
        {
            switch (c.kind)
            {
                case CMD_STEER:
                    set_direction_degres(c.value);     /* X_DIR PWM steering servo */
                    break;
                case CMD_SPEED:
                    set_vitesse_m_s(c.value);
                    break;
                case CMD_PWM:
                    /* Raw pulse width, bypasses the speed-to-us mapping. 0 means
                     * no pulse (pin stays low). */
                    prop_set_us((uint32_t)c.value);
                    break;
                case CMD_DIRPWM: {
                    /* Raw steering pulse (us), bypasses the +-18 deg clamp for
                     * centering a mis-mounted servo. Clamped to a safe span. */
                    uint32_t us = (uint32_t)c.value;
                    if (us < DIR_CAL_MIN_US) us = DIR_CAL_MIN_US;
                    if (us > DIR_CAL_MAX_US) us = DIR_CAL_MAX_US;
                    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, us);
                    break;
                }
                case CMD_WD:
                    watchdog_enabled = (c.value != 0.0f);
                    {
                        char b[32];
                        snprintf(b, sizeof b, "WD %s\n", watchdog_enabled ? "on" : "off");
                        dbg_puts(b);
                    }
                    break;
                case CMD_STOP:
                    set_vitesse_m_s(0.0f);
                    set_direction_degres(0.0f);
                    break;
                case CMD_CAL:
                    dbg_send_ok(&c);
                    esc_calibrate();
                    /* Refresh timestamp so the watchdog does not fire immediately. */
                    last_cmd_ms = HAL_GetTick();
                    continue;
                case CMD_REV:
                    dbg_send_ok(&c);
                    esc_reverse_sequence();
                    last_cmd_ms = HAL_GetTick();
                    continue;
                case CMD_DPING: {
                    dbg_send_ok(&c);
                    uint8_t resp[20];
                    uint16_t got = dxl_ping(DXL_ID, resp, sizeof resp);
                    if (got >= 11 && resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == 0xFD) {
                        dbg_puts("DXL_PING_OK servo alive\n");
                    } else if (got > 0) {
                        dbg_puts("DXL_PING garbled response\n");
                    } else {
                        dbg_puts("DXL_PING no response (timeout)\n");
                    }
                    last_cmd_ms = HAL_GetTick();
                    continue;
                }
                case CMD_DSCAN:
                    dbg_send_ok(&c);
                    dxl_scan();
                    last_cmd_ms = HAL_GetTick();
                    continue;
                default:
                    break;
            }
            dbg_send_ok(&c);
            last_cmd_ms = now;
            watchdog_armed = 0;
        }

        if (esc_armed_neutral && watchdog_enabled && (now - last_cmd_ms) > CMD_WATCHDOG_MS)
        {
            set_vitesse_m_s(0.0f);
            set_direction_degres(0.0f);
            if (!watchdog_armed) {
                dbg_puts("WD stale -> neutral\n");
                watchdog_armed = 1;
            }
        }

        /* Rear IR sample (~20 Hz): raw value and blocked flag for reverse gating. */
        if ((now - last_adc_ms) >= 50u) {
            rear_ir_raw  = adc_rear_ir_raw();
            rear_blocked = (rear_ir_raw >= REAR_IR_BLOCK_RAW);
            last_adc_ms  = now;
        }

        /* IMU heading (~10 Hz). Keep the previous value on a read failure. */
        if (imu_ok && (now - last_imu_ms) >= 100u) {
            int h;
            if (bno055_read_heading_deg(&h)) imu_heading = h;
            last_imu_ms = now;
        }

        /* Wheel encoder (4 Hz): pulse-count delta to pulses/s to speed (mm/s). */
        if ((now - last_enc_ms) >= 250u) {
            uint32_t c  = encoder_count();
            uint32_t d  = c - last_enc_count;       /* uint32 wrap is harmless */
            uint32_t dt = now - last_enc_ms;
            wheel_pps  = (dt > 0u) ? (d * 1000u / dt) : 0u;
            speed_mm_s = wheel_pps * 1000u / ENC_PULSES_PER_M;
            last_enc_count = c;
            last_enc_ms    = now;
        }

        if ((now - last_led_ms) >= LED_PERIOD_MS) {
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
            last_led_ms = now;
        }

        if ((now - last_hb_ms) >= HB_PERIOD_MS) {
            dbg_send_hb(now, oled_ok, rear_ir_raw, rear_blocked, imu_ok, imu_heading,
                        wheel_pps, speed_mm_s);
            last_hb_ms = now;
        }

        /* Stream the OLED frame via DMA, one page per loop pass. The ~12 ms data
         * burst per page runs on DMA1_Ch1; only the page-address setup is polled,
         * so the control loop is not blocked on the display. */
        if (oled_ok) sh1106_refresh_poll();

        /* OLED at ~3 Hz: THROTTLE % and STEERING deg as labelled bar gauges with
         * status below. Rows are rebuilt only once the previous frame has finished
         * streaming (tear-free); the DMA push above then streams the new frame. */
        if (oled_ok && sh1106_refresh_idle() && (now - last_oled_ms) >= 300u) {
            uint32_t ccr_prop = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
            uint32_t ccr_dir  = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);

            /* Steering pulse to degrees (center DIR_MILIEU; + = LEFT). Uses the
             * moteurs.h constants so it tracks calibration (incl. the reversed
             * GAUCHE/DROITE sign for this servo). */
            int s = ((int)ccr_dir - DIR_MILIEU) * (2 * (int)DIR_ANGLE_MAX)
                    / (DIR_BUTEE_GAUCHE - DIR_BUTEE_DROITE);
            if (s > 18) s = 18; else if (s < -18) s = -18;

            /* Throttle pulse to signed % past the ESC dead zone (+ = FWD). */
            int t;
            if ((int)ccr_prop >= PROP_POINT_MORT)
                t = ((int)ccr_prop - PROP_POINT_MORT) * 100 / (PROP_MAX - PROP_POINT_MORT);
            else if ((int)ccr_prop <= PROP_POINT_MORT_NEG)
                t = -(((int)PROP_POINT_MORT_NEG - (int)ccr_prop) * 100 / (PROP_MAX - PROP_POINT_MORT));
            else
                t = 0;
            if (t > 100) t = 100; else if (t < -100) t = -100;

            int at = (t < 0) ? -t : t;
            int as = (s < 0) ? -s : s;
            uint32_t link_s = (now - last_cmd_ms) / 1000u;
            if (link_s > 99u) link_s = 99u;

            char line[24];
            char bar[19];

            /* Row 0: throttle headline. */
            snprintf(line, sizeof line, "THROTTLE %4s %+4d%%",
                     (t > 0) ? "FWD" : (t < 0) ? "REV" : "IDLE", t);
            pad21(line); sh1106_print_row(0, line);

            /* Row 1: throttle bar, seam at center, FWD fills right, REV left. */
            for (int i = 0; i < 18; i++) bar[i] = ' ';
            bar[18] = '\0';
            bar[8] = '|';
            {
                int n = (at * 9 + 50) / 100;                 /* 0..9 cells */
                if (t > 0)      for (int i = 0; i < n; i++) bar[9 + i] = '#';
                else if (t < 0) for (int i = 0; i < n; i++) bar[8 - i] = '#';
            }
            snprintf(line, sizeof line, "[%s]", bar);
            pad21(line); sh1106_print_row(1, line);

            /* Row 2: steering headline. */
            snprintf(line, sizeof line, "STEERING %5s %+3dd",
                     (s > 0) ? "LEFT" : (s < 0) ? "RIGHT" : "AHEAD", s);
            pad21(line); sh1106_print_row(2, line);

            /* Row 3: steering bar, pivot at center, LEFT '<' / RIGHT '>'. */
            for (int i = 0; i < 18; i++) bar[i] = ' ';
            bar[18] = '\0';
            bar[9] = '|';
            {
                int n = (as * 9 + 9) / 18;                   /* 0..9 cells (2 deg each) */
                if (s > 0)      { for (int i = 0; i < n; i++) bar[8 - i]  = '#'; if (n > 0) bar[8 - (n - 1)] = '<'; }
                else if (s < 0) { for (int i = 0; i < n; i++) bar[10 + i] = '#'; if (n > 0) bar[10 + (n - 1)] = '>'; }
            }
            snprintf(line, sizeof line, "[%s]", bar);
            pad21(line); sh1106_print_row(3, line);

            sh1106_print_row(4, "");                          /* spacer */

            /* Row 5: rear obstacle + IMU heading. */
            if (imu_ok)
                snprintf(line, sizeof line, "REAR %-6s HDG %3d",
                         rear_blocked ? "BLOCK" : "clear", imu_heading);
            else
                snprintf(line, sizeof line, "REAR %-6s HDG  --",
                         rear_blocked ? "BLOCK" : "clear");
            pad21(line); sh1106_print_row(5, line);

            /* Row 6: link age + watchdog. */
            if (watchdog_armed)
                snprintf(line, sizeof line, "LINK %-2lus    WD STALE", (unsigned long)link_s);
            else
                snprintf(line, sizeof line, "LINK %-2lus       WD ok", (unsigned long)link_s);
            pad21(line); sh1106_print_row(6, line);

            /* Row 7: real measured wheel speed (from the encoder). */
            snprintf(line, sizeof line, "SPEED %lu.%02lu m/s",
                     (unsigned long)(speed_mm_s / 1000u),
                     (unsigned long)((speed_mm_s % 1000u) / 10u));
            pad21(line); sh1106_print_row(7, line);

            sh1106_refresh_begin();  /* stream the new frame via DMA, one page per pass */
            last_oled_ms = now;
        }
    }
}

/* Copied verbatim from the professor's project (HSI -> PLL -> 170 MHz SYSCLK). */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    RCC_OscInitStruct.OscillatorType       = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState             = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue  = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState         = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource        = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM             = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN             = 85;
    RCC_OscInitStruct.PLL.PLLP             = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ             = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR             = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; }
#endif

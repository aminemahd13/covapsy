/*
 * dynamixel.c
 *
 * Dynamixel Protocol 2.0 minimal master, USART1 single-wire half-duplex.
 *
 * Packet layout (Robotis Protocol 2.0):
 *   FF FF FD 00 | ID | LEN_L LEN_H | INST | PARAMS... | CRC_L CRC_H
 *   - LEN counts INST + PARAMS + CRC bytes
 *   - INST 0x03 = WRITE
 *   - CRC-16/IBM polynomial 0x8005, init 0x0000, over header through last param
 *
 * WRITE only, no byte-stuffing (cannot occur for writes to addr 11/64/116).
 */
#include "dynamixel.h"
#include "main.h"
#include "moteurs.h"     /* for DIR_ANGLE_MAX */
#include "usart.h"       /* huart1 (LPUART1 = Pi VCP) for diagnostics */

#include <stdio.h>
#include <string.h>

#define DXL_ADDR_OPERATING_MODE  11
#define DXL_ADDR_TORQUE_ENABLE   64
#define DXL_ADDR_GOAL_POSITION  116

#define DXL_INST_WRITE         0x03

static UART_HandleTypeDef hdxl;

/* Hex-dump a labelled buffer to the Pi VCP (LPUART1). */
static void dxl_dump(const char *label, const uint8_t *data, uint16_t n)
{
    HAL_UART_Transmit(&huart1, (const uint8_t *)label, strlen(label), 30);
    char b[6];
    for (uint16_t i = 0; i < n; i++) {
        snprintf(b, sizeof b, " %02X", data[i]);
        HAL_UART_Transmit(&huart1, (const uint8_t *)b, 3, 30);
    }
    HAL_UART_Transmit(&huart1, (const uint8_t *)"\n", 1, 30);
}

static uint16_t dxl_crc16(const uint8_t *data, uint16_t n)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < n; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x8005u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Transmit on PA9 with RX muted. The board echoes TX onto RX
 * (TX_DIR, R5, DATA, RX_DIR, PA10), so disable RE while sending; caller
 * re-enables it to hear the servo reply. */
static bool dxl_tx(const uint8_t *pkt, uint16_t n)
{
    CLEAR_BIT(USART1->CR1, USART_CR1_RE);
    return HAL_UART_Transmit(&hdxl, (uint8_t *)pkt, n, 50) == HAL_OK;
}

static bool dxl_write_n(uint8_t id, uint16_t addr, const uint8_t *data, uint8_t n)
{
    uint8_t pkt[24];
    uint16_t len = 1 + 2 + n + 2;        /* INST + addr + data + CRC */
    if (10 + n + 2 > sizeof pkt) return false;

    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFD; pkt[3] = 0x00;
    pkt[4] = id;
    pkt[5] = (uint8_t)(len & 0xFF);
    pkt[6] = (uint8_t)(len >> 8);
    pkt[7] = DXL_INST_WRITE;
    pkt[8] = (uint8_t)(addr & 0xFF);
    pkt[9] = (uint8_t)(addr >> 8);
    memcpy(&pkt[10], data, n);

    uint16_t crc = dxl_crc16(pkt, 10 + n);
    pkt[10 + n + 0] = (uint8_t)(crc & 0xFF);
    pkt[10 + n + 1] = (uint8_t)(crc >> 8);

    dxl_dump("DXL_TX:", pkt, 10 + n + 2);
    return dxl_tx(pkt, 10 + n + 2);
}

uint16_t dxl_ping(uint8_t id, uint8_t *resp, uint16_t resp_max)
{
    /* Ping packet: header(4) + id + len(2)=3 + inst=0x01 + crc(2) = 10 bytes */
    uint8_t pkt[10];
    pkt[0] = 0xFF; pkt[1] = 0xFF; pkt[2] = 0xFD; pkt[3] = 0x00;
    pkt[4] = id;
    pkt[5] = 0x03; pkt[6] = 0x00;
    pkt[7] = 0x01;                  /* PING instruction */
    uint16_t crc = dxl_crc16(pkt, 8);
    pkt[8] = (uint8_t)(crc & 0xFF);
    pkt[9] = (uint8_t)(crc >> 8);

    dxl_dump("DXL_TX(ping):", pkt, 10);

    if (!dxl_tx(pkt, 10)) return 0;

    /* Listen on PA10 (RX_DIR off the DATA line). Un-mute RX and clear stale
     * flag/echo first. PING reply = 14 bytes
     * (Header4 ID Len2 Err Model2 FwVer Crc2). */
    __HAL_UART_CLEAR_OREFLAG(&hdxl);
    __HAL_UART_SEND_REQ(&hdxl, UART_RXDATA_FLUSH_REQUEST);
    SET_BIT(USART1->CR1, USART_CR1_RE);
    uint16_t got = 0;
    uint32_t deadline = HAL_GetTick() + 80;     /* 80 ms timeout */
    while (got < resp_max && HAL_GetTick() < deadline) {
        uint8_t b;
        if (HAL_UART_Receive(&hdxl, &b, 1, 10) == HAL_OK) {
            resp[got++] = b;
        }
    }
    if (got > 0) dxl_dump("DXL_RX:", resp, got);
    else         dxl_dump("DXL_RX:", (const uint8_t *)"", 0);
    return got;
}

bool dxl_torque_off(void)
{
    uint8_t v = 0;
    return dxl_write_n(DXL_ID, DXL_ADDR_TORQUE_ENABLE, &v, 1);
}

bool dxl_torque_on(void)
{
    uint8_t v = 1;
    return dxl_write_n(DXL_ID, DXL_ADDR_TORQUE_ENABLE, &v, 1);
}

bool dxl_set_goal_position(int32_t position)
{
    if (position < 0) position = 0;
    if (position > 4095) position = 4095;
    uint8_t b[4];
    b[0] = (uint8_t)(position      );
    b[1] = (uint8_t)(position >>  8);
    b[2] = (uint8_t)(position >> 16);
    b[3] = (uint8_t)(position >> 24);
    return dxl_write_n(DXL_ID, DXL_ADDR_GOAL_POSITION, b, 4);
}

void dxl_set_steering_degres(float angle_degre)
{
    if (angle_degre < -DIR_ANGLE_MAX) angle_degre = -DIR_ANGLE_MAX;
    if (angle_degre > +DIR_ANGLE_MAX) angle_degre = +DIR_ANGLE_MAX;
    int32_t pos = DXL_POS_CENTER + (int32_t)(angle_degre * DXL_POS_PER_DEG);
    dxl_set_goal_position(pos);
}

void dxl_set_baud(uint32_t baud)
{
    HAL_UART_DeInit(&hdxl);
    hdxl.Init.BaudRate = baud;
    HAL_UART_Init(&hdxl);          /* full-duplex 2-wire, not single-wire */
    HAL_Delay(5);
}

void dxl_scan(void)
{
    static const uint32_t bauds[] = {57600u, 115200u, 1000000u, 9600u, 1000000u / 4u};
    char line[64];
    for (uint8_t i = 0; i < sizeof bauds / sizeof bauds[0]; i++) {
        snprintf(line, sizeof line, "DXL_SCAN baud=%lu\n", (unsigned long)bauds[i]);
        HAL_UART_Transmit(&huart1, (const uint8_t *)line, strlen(line), 30);
        dxl_set_baud(bauds[i]);

        uint8_t resp[24];
        uint16_t got = dxl_ping(0xFE, resp, sizeof resp);          /* broadcast */
        if (got == 0) {
            uint16_t got2 = dxl_ping(0x01, resp, sizeof resp);     /* id=1 fallback */
            if (got2) {
                snprintf(line, sizeof line, "DXL_SCAN HIT id=1 baud=%lu len=%u\n",
                         (unsigned long)bauds[i], got2);
                HAL_UART_Transmit(&huart1, (const uint8_t *)line, strlen(line), 30);
            }
        } else {
            snprintf(line, sizeof line, "DXL_SCAN HIT broadcast baud=%lu len=%u\n",
                     (unsigned long)bauds[i], got);
            HAL_UART_Transmit(&huart1, (const uint8_t *)line, strlen(line), 30);
        }
    }
    /* Restore working baud. */
    dxl_set_baud(DXL_BAUD);
}

void dxl_init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* Servo on a 2-wire half-duplex bus:
     *   PA9  (USART1_TX) to TX_DIR to R5 (1k) to DATA   (R5 isolates TX)
     *   PA10 (USART1_RX) from RX_DIR from DATA          (servo reply)
     * Use full-duplex USART1, not single-wire. PA9 push-pull drives DATA
     * through R5; PA10 reads DATA directly. dxl_tx() mutes RX during a send. */
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_9;                  /* TX to TX_DIR to R5 to DATA */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin       = GPIO_PIN_10;                 /* RX from RX_DIR from DATA (direct) */
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &g);

    hdxl.Instance              = USART1;
    hdxl.Init.BaudRate         = DXL_BAUD;
    hdxl.Init.WordLength       = UART_WORDLENGTH_8B;
    hdxl.Init.StopBits         = UART_STOPBITS_1;
    hdxl.Init.Parity           = UART_PARITY_NONE;
    hdxl.Init.Mode             = UART_MODE_TX_RX;
    hdxl.Init.HwFlowCtl        = UART_HWCONTROL_NONE;
    hdxl.Init.OverSampling     = UART_OVERSAMPLING_16;
    hdxl.Init.OneBitSampling   = UART_ONE_BIT_SAMPLE_DISABLE;
    hdxl.Init.ClockPrescaler   = UART_PRESCALER_DIV1;
    hdxl.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hdxl) != HAL_OK) {      /* full-duplex 2-wire */
        Error_Handler();
    }

    /* Servo needs 150 ms after power-on before it answers. */
    HAL_Delay(150);

    /* Operating Mode (addr 11) changes only with torque off. Set
     * position-control mode (value 3) then re-enable torque. */
    dxl_torque_off();
    HAL_Delay(20);
    uint8_t mode = 3;
    dxl_write_n(DXL_ID, DXL_ADDR_OPERATING_MODE, &mode, 1);
    HAL_Delay(20);
    dxl_torque_on();
    HAL_Delay(20);

    /* Centre. */
    dxl_set_goal_position(DXL_POS_CENTER);
}

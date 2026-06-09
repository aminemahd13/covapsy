/*
 * i2c.c: bare-metal I2C1 master, polling, 100 kHz standard mode. CMSIS only.
 *
 * Pins on NUCLEO-G431KB (from silkscreen):
 *   PB7  = SDA (AF4 open-drain), Arduino socket D4
 *   PA15 = SCL (AF4 open-drain), Arduino socket D5
 *   PB6 is the Hat BUZZER_MCU line, not I2C SCL.
 *
 * Clock source: HSI16 (16 MHz). TIMINGR set for 100 kHz SCL.
 */
#include "i2c.h"
#include "stm32g4xx.h"

#define I2C_TIMEOUT_LOOPS  300000u

static bool wait_flag_set(volatile uint32_t *reg, uint32_t mask)
{
    for (uint32_t i = 0; i < I2C_TIMEOUT_LOOPS; i++) {
        if (*reg & mask) return true;
    }
    return false;
}

/* I2C bus recovery: toggle SCL up to 9 times until a slave releases SDA, then
 * STOP. Bit-banged open-drain GPIO before the peripheral takes the pins.
 * SDA = PB7, SCL = PA15. */
static void i2c1_bus_recover(void)
{
    /* Open-drain + pull-up on both lines. */
    GPIOB->OTYPER |= (1u << 7);
    GPIOB->PUPDR   = (GPIOB->PUPDR & ~(3u << (7 * 2)))  | (1u << (7 * 2));
    GPIOA->OTYPER |= (1u << 15);
    GPIOA->PUPDR   = (GPIOA->PUPDR & ~(3u << (15 * 2))) | (1u << (15 * 2));

    /* Release both lines (input, pulled high). */
    GPIOB->MODER &= ~(3u << (7 * 2));
    GPIOA->MODER &= ~(3u << (15 * 2));
    for (volatile uint32_t d = 0; d < 3000; d++) { }

    /* Clock SCL until SDA goes high, max 9 pulses. */
    for (int i = 0; i < 9; i++) {
        if (GPIOB->IDR & (1u << 7)) break;                 /* SDA high, free */
        GPIOA->BSRR  = (1u << (15 + 16));                  /* SCL low (ODR=0) */
        GPIOA->MODER = (GPIOA->MODER & ~(3u << (15 * 2))) | (1u << (15 * 2));
        for (volatile uint32_t d = 0; d < 3000; d++) { }
        GPIOA->MODER &= ~(3u << (15 * 2));                 /* SCL release high */
        for (volatile uint32_t d = 0; d < 3000; d++) { }
    }

    /* Issue a STOP: with SCL high, drive SDA low then release it high. */
    GPIOA->MODER &= ~(3u << (15 * 2));                      /* SCL high */
    GPIOB->BSRR   = (1u << (7 + 16));                       /* SDA low */
    GPIOB->MODER  = (GPIOB->MODER & ~(3u << (7 * 2))) | (1u << (7 * 2));
    for (volatile uint32_t d = 0; d < 3000; d++) { }
    GPIOB->MODER &= ~(3u << (7 * 2));                       /* SDA release high */
    for (volatile uint32_t d = 0; d < 3000; d++) { }
}

void i2c1_init(void)
{
    /* GPIO clocks. */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;

    /* Free any slave holding the bus before configuring it. */
    i2c1_bus_recover();

    /* PB7 = AF4 (I2C1_SDA), open-drain, pull-up, low speed. */
    GPIOB->MODER   = (GPIOB->MODER   & ~(3u << (7 * 2))) | (2u << (7 * 2));
    GPIOB->OTYPER  |= (1u << 7);
    GPIOB->OSPEEDR &= ~(3u << (7 * 2));
    GPIOB->PUPDR   = (GPIOB->PUPDR   & ~(3u << (7 * 2))) | (1u << (7 * 2));
    GPIOB->AFR[0]  = (GPIOB->AFR[0]  & ~(0xFu << (7 * 4))) | (4u << (7 * 4));

    /* PA15 = AF4 (I2C1_SCL), open-drain, pull-up. */
    GPIOA->MODER   = (GPIOA->MODER   & ~(3u << (15 * 2))) | (2u << (15 * 2));
    GPIOA->OTYPER  |= (1u << 15);
    GPIOA->OSPEEDR &= ~(3u << (15 * 2));
    GPIOA->PUPDR   = (GPIOA->PUPDR   & ~(3u << (15 * 2))) | (1u << (15 * 2));
    GPIOA->AFR[1]  = (GPIOA->AFR[1]  & ~(0xFu << ((15 - 8) * 4))) | (4u << ((15 - 8) * 4));

    /* Route I2C1 clock to HSI16 via RCC->CCIPR. */
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_I2C1SEL_Msk) | (2u << RCC_CCIPR_I2C1SEL_Pos);

    /* Enable I2C1 peripheral clock. */
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C1EN;

    /* Program TIMINGR for 100 kHz standard mode at HSI16 (16 MHz): PRESC=1,
     * SCLDEL=4, SDADEL=2, SCLH=39, SCLL=39. Bus is marginal above 100 kHz
     * (weak pull-ups). */
    I2C1->CR1     = 0;
    I2C1->TIMINGR = (1u << 28) | (4u << 20) | (2u << 16) | (39u << 8) | 39u;
    I2C1->CR1     = I2C_CR1_PE;
}

void i2c1_reset(void)
{
    /* Toggle the RCC reset bit and clear all status flags. */
    I2C1->CR1 = 0;                                     /* disable */
    RCC->APB1RSTR1 |=  RCC_APB1RSTR1_I2C1RST;
    for (volatile uint32_t i = 0; i < 50; i++) { }
    RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_I2C1RST;
    I2C1->TIMINGR = (1u << 28) | (4u << 20) | (2u << 16) | (39u << 8) | 39u;  /* 100 kHz */
    I2C1->CR1 = I2C_CR1_PE;
}

bool i2c1_write(uint8_t addr_7bit, const uint8_t *data, uint16_t n)
{
    if (n == 0 || n > 255) return false;

    /* Clear any stale flags from a prior aborted transaction. */
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;

    /* Wait for the bus to be free (bounded, ~8 ms tops). */
    for (uint32_t i = 0; i < I2C_TIMEOUT_LOOPS; i++) {
        if (!(I2C1->ISR & I2C_ISR_BUSY)) break;
    }

    /* Set CR2: 7-bit address, write, NBYTES = n, AUTOEND, START. */
    uint32_t cr2 = ((uint32_t)addr_7bit << 1) & I2C_CR2_SADD_Msk;
    cr2 |= ((uint32_t)n << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
    cr2 |= I2C_CR2_AUTOEND | I2C_CR2_START;
    I2C1->CR2 = cr2;

    for (uint16_t i = 0; i < n; i++) {
        if (!wait_flag_set(&I2C1->ISR, I2C_ISR_TXIS | I2C_ISR_NACKF)) {
            return false;
        }
        if (I2C1->ISR & I2C_ISR_NACKF) {
            I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
            return false;
        }
        I2C1->TXDR = data[i];
    }

    if (!wait_flag_set(&I2C1->ISR, I2C_ISR_STOPF | I2C_ISR_NACKF)) {
        return false;
    }
    bool nack = (I2C1->ISR & I2C_ISR_NACKF) != 0;
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    return !nack;
}

bool i2c1_read(uint8_t addr_7bit, uint8_t reg, uint8_t *buf, uint16_t n)
{
    if (n == 0 || n > 255) return false;

    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    for (uint32_t i = 0; i < I2C_TIMEOUT_LOOPS; i++) {
        if (!(I2C1->ISR & I2C_ISR_BUSY)) break;
    }

    /* Phase 1: write register pointer (1 byte), software-end for repeated START
     * (no AUTOEND, no RELOAD). */
    uint32_t cr2 = ((uint32_t)addr_7bit << 1) & I2C_CR2_SADD_Msk;
    cr2 |= (1u << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
    cr2 |= I2C_CR2_START;                     /* write direction */
    I2C1->CR2 = cr2;

    if (!wait_flag_set(&I2C1->ISR, I2C_ISR_TXIS | I2C_ISR_NACKF)) return false;
    if (I2C1->ISR & I2C_ISR_NACKF) {
        I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
        return false;
    }
    I2C1->TXDR = reg;
    if (!wait_flag_set(&I2C1->ISR, I2C_ISR_TC | I2C_ISR_NACKF)) return false;
    if (I2C1->ISR & I2C_ISR_NACKF) {
        I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
        return false;
    }

    /* Phase 2: repeated START, read n bytes, AUTOEND. */
    cr2 = ((uint32_t)addr_7bit << 1) & I2C_CR2_SADD_Msk;
    cr2 |= ((uint32_t)n << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
    cr2 |= I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | I2C_CR2_START;
    I2C1->CR2 = cr2;

    for (uint16_t i = 0; i < n; i++) {
        if (!wait_flag_set(&I2C1->ISR, I2C_ISR_RXNE | I2C_ISR_NACKF)) return false;
        if (I2C1->ISR & I2C_ISR_NACKF) {
            I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
            return false;
        }
        buf[i] = (uint8_t)I2C1->RXDR;
    }

    if (!wait_flag_set(&I2C1->ISR, I2C_ISR_STOPF | I2C_ISR_NACKF)) return false;
    bool nack = (I2C1->ISR & I2C_ISR_NACKF) != 0;
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF;
    return !nack;
}

/* Non-blocking DMA write (DMA1_Channel1, DMAMUX1 req I2C1_TX).
 * DMA1_Channel1 feeds I2C1->TXDR on each TXIS; caller polls for completion.
 * MEM to PERIPH, 8-bit, memory-incrementing. */

static volatile bool dma_active = false;

void i2c1_dma_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMAMUX1EN;

    /* Route DMAMUX1_Channel0 to the I2C1_TX request. Request ID 17 = I2C1_TX
     * on STM32G4. */
    DMAMUX1_Channel0->CCR = (DMAMUX1_Channel0->CCR & ~DMAMUX_CxCR_DMAREQ_ID)
                          | (17u & DMAMUX_CxCR_DMAREQ_ID);

    /* Static channel setup: read-from-memory, memory-increment, 8-bit (PSIZE/
     * MSIZE = 00), high priority. CPAR fixed at TXDR; CMAR/CNDTR/EN set per xfer. */
    DMA1_Channel1->CCR  = DMA_CCR_DIR | DMA_CCR_MINC | (2u << DMA_CCR_PL_Pos);
    DMA1_Channel1->CPAR = (uint32_t)&I2C1->TXDR;
}

void i2c1_dma_abort(void)
{
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    I2C1->CR1 &= ~I2C_CR1_TXDMAEN;
    /* If the transfer stalled mid-bus, force a STOP to release SDA/SCL. */
    if (I2C1->ISR & I2C_ISR_BUSY) {
        I2C1->CR2 |= I2C_CR2_STOP;
        for (uint32_t i = 0; i < I2C_TIMEOUT_LOOPS; i++) {
            if (I2C1->ISR & I2C_ISR_STOPF) break;
        }
    }
    I2C1->ISR  = I2C_ISR_TXE;          /* flush a byte DMA may have pre-loaded */
    I2C1->ICR  = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;
    DMA1->IFCR = DMA_IFCR_CGIF1;        /* clear all channel-1 DMA flags */
    dma_active = false;
}

/* Hard bus recovery for a jam (slave holding SDA low). Stops any DMA, then
 * re-runs i2c1_init() (9-clock SDA-release sequence plus reprogram). */
void i2c1_recover(void)
{
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    I2C1->CR1 &= ~I2C_CR1_TXDMAEN;
    dma_active = false;
    i2c1_init();
}

bool i2c1_write_dma_start(uint8_t addr_7bit, const uint8_t *data, uint16_t n)
{
    if (n == 0 || n > 255 || dma_active) return false;

    /* Wait for the bus to be free (bounded). If still held, hard-recover. */
    bool busy = true;
    for (uint32_t i = 0; i < I2C_TIMEOUT_LOOPS; i++) {
        if (!(I2C1->ISR & I2C_ISR_BUSY)) { busy = false; break; }
    }
    if (busy) i2c1_recover();
    I2C1->ICR = I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF;

    /* Arm the DMA channel for this buffer. */
    DMA1_Channel1->CCR  &= ~DMA_CCR_EN;
    DMA1->IFCR           = DMA_IFCR_CGIF1;
    DMA1_Channel1->CMAR  = (uint32_t)data;
    DMA1_Channel1->CNDTR = n;
    DMA1_Channel1->CCR  |= DMA_CCR_EN;

    /* Enable TX DMA requests, then START a 7-bit write of n bytes with AUTOEND
     * (hardware issues STOP after the last byte, then STOPF). */
    I2C1->CR1 |= I2C_CR1_TXDMAEN;
    uint32_t cr2 = ((uint32_t)addr_7bit << 1) & I2C_CR2_SADD_Msk;
    cr2 |= ((uint32_t)n << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES_Msk;
    cr2 |= I2C_CR2_AUTOEND | I2C_CR2_START;
    I2C1->CR2 = cr2;

    dma_active = true;
    return true;
}

int i2c1_write_dma_poll(void)
{
    if (!dma_active) return 1;

    uint32_t isr = I2C1->ISR;
    if (isr & I2C_ISR_NACKF) {          /* no ACK: abort and report error */
        i2c1_dma_abort();
        return -1;
    }
    if (isr & I2C_ISR_STOPF) {          /* AUTOEND fired: transfer complete */
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        I2C1->CR1 &= ~I2C_CR1_TXDMAEN;
        I2C1->ICR  = I2C_ICR_STOPCF;
        DMA1->IFCR = DMA_IFCR_CGIF1;
        dma_active = false;
        return 1;
    }
    return 0;                            /* still streaming */
}

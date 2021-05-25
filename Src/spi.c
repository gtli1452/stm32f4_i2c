#include "spi.h"
#include "stm32f4xx.h"

/* PC3 is _CS */
#define _CS_IDLE GPIOC->ODR |= (1 << 3)
#define _CS_ACK GPIOC->ODR &= ~(1 << 3)

/* PC1 is SDO */
#define SDO_HIGH GPIOC->ODR |= (1 << 1)
#define SDO_LOW GPIOC->ODR &= ~(1 << 1)

/* PC15 is SCLK */
#define SCLK_HIGH GPIOC->ODR |= (1 << 15)
#define SCLK_LOW GPIOC->ODR &= ~(1 << 15)

/* PC13 is SDI */
#define SDI_READ GPIOC->IDR &(1 << 13)

/* PC14 is _RST */
#define _RST_IDLE GPIOC->ODR |= (1 << 14)
#define _RST_ACK GPIOC->ODR &= ~(1 << 14)

/* PA1 is _BUSY*/
#define _BUSY_READ ((GPIOA->IDR & (1 << 1)) == 0 ? 1 : 0)

/* tBUSW for /BUSY minimum SCLK cycle requirements */
#define tBUSW_REST 745
#define tBUSW_NOP 4
#define tBUSW_VALID_ADDR 4
#define tBUSW_DAC 19
#define tBUSW_VIL_VIH 22

volatile spi_packet_t sdo, sdi;

/* Delay time */
static void nop(uint16_t cnt)
{
    for (int i = 0; i < cnt; i++)
        __nop();
}

/*
 * The SCLK pin continues to operate for as long as the BUSY pin state remains
 * active (active low). This period of time is defined by the parameter tBUSW
 * and is defined in ADATE320 datasheet.
 */
static void release_busy(uint16_t sclk_cnt)
{
    for (int i = 0; i < sclk_cnt; i++) {
        SCLK_LOW;
        nop(0);
        SCLK_HIGH;
        nop(0);
    }
    SCLK_LOW;
}

static void release_busy_pin (uint8_t address)
{
    if (address <= 0x0F)
        release_busy(tBUSW_VIL_VIH);
    else if (address == 0x12)
        release_busy(tBUSW_REST);
    else
        release_busy(tBUSW_VALID_ADDR);
}

/* Hardware reset to ADATE320 */
void ate_hw_reset(void)
{
    _RST_ACK;
    nop(0);

    _RST_IDLE;
    nop(0);
    
    release_busy(tBUSW_REST);
}

uint8_t write_spi(spi_packet_t sdo)
{
    uint8_t addr = sdo.spi.address;
    
    _CS_ACK;
    nop(0);

    for (int i = 0; i < 26; i++) {
        SCLK_LOW;
        nop(0);

        if (sdo.spi_word & 0x02000000)
            SDO_HIGH;
        else
            SDO_LOW;
        nop(0);

        SCLK_HIGH;
        nop(0);

        sdo.spi_word <<= 1;
    }

    SCLK_LOW;
    nop(0);

    _CS_IDLE;

    release_busy_pin(addr);

    return 0;
}

uint8_t read_spi(volatile spi_packet_t *sdi)
{
    uint32_t tmp = 0;
    _CS_ACK;
    nop(0);

    for (int i = 0; i < 26; i++) {
        SCLK_LOW;
        nop(0);

        SCLK_HIGH;
        nop(0);

        if (SDI_READ)
            tmp += (1 << (25 - i));
    }

    SCLK_LOW;
    nop(0);

    _CS_IDLE;

    release_busy(tBUSW_VALID_ADDR);
    
    sdi->spi_word = tmp;

    return 0;
}

#include "i2c.h"
#include "stm32f4xx.h"

/* this handler deals with master read and master write only */

/* PB0 */
#define SCL_HIGH GPIOB->ODR |= GPIO_PIN_0
#define SCL_LOW GPIOB->ODR &= ~GPIO_PIN_0

/* PB15 */
#define SDA_HIGH GPIOB->ODR |= GPIO_PIN_15
#define SDA_LOW GPIOB->ODR &= ~GPIO_PIN_15

/* Configure PB15 input or output */
#define SET_SDA_INPUT GPIOB->MODER &= ~(3UL << (2 * 15))
#define SET_SDA_OUTPUT GPIOB->MODER |= (1UL << (2 * 15))

/* Get PB15 input */
#define SDA GPIOB->IDR &GPIO_PIN_15

#define I2C_HALF_PERIOD_CNT_100K 155
#define START_SET_TIME_CNT_100K 120
#define START_HOLD_TIME_CNT_100K 120
#define STOP_SET_TIME_CNT_100K 30

#define I2C_HALF_PERIOD_CNT_400K 36
#define START_SET_TIME_CNT_400K 36
#define START_HOLD_TIME_CNT_400K 36
#define STOP_SET_TIME_CNT_400K 36

volatile i2c_packet_t i2c;
volatile uint32_t gI2cTimeout;

static inline void half_period(void)
{
    /*
     * __asm("NOP") ~= 35.6ns when system clock is 168MHz
     * i = 6, 1.7MHz; i = 36, 400kHz; i = 155, 100kHz;
     */
    for (int i = 0; i < I2C_HALF_PERIOD_CNT_400K; i++)
        __asm("NOP");
}

static inline void repeat_start_set_time(void)
{
    for (int i = 0; i < START_SET_TIME_CNT_400K; i++)
        __asm("NOP");
}

static inline void repeat_start_hold_time(void)
{
    for (int i = 0; i < START_HOLD_TIME_CNT_400K; i++)
        __asm("NOP");
}

static inline void stop_set_time(void)
{
    for (int i = 0; i < STOP_SET_TIME_CNT_400K; i++)
        __asm("NOP");
}

static inline void nop_test(int cnt)
{
    for (int i = 0; i < cnt; i++)
        __asm("NOP");
}

/* SM transmit byte */
uint8_t spike = 0;
void I2cTxByte(uint8_t I2cTxByte)
{
    uint8_t tmp = I2cTxByte;

    for (int i = 0; i < 8; i++) {
        SCL_LOW;
        if (tmp & 0x80)
            SDA_HIGH;
        else
            SDA_LOW;
        half_period();

        SCL_HIGH;
        if (!spike && i == 2) {
            nop_test(18);
            SDA_HIGH;
            SDA_LOW;
            nop_test(18);
            spike = 1;
        } else
            half_period();

        tmp = tmp << 1;
    }
}

/* SM receive byte */
uint8_t I2cRxByte(void)
{
    uint8_t bI2cRxByte = 0;

    SCL_LOW;
    SET_SDA_INPUT;  // set SDA input to get data
    half_period();

    for (int i = 7; i >= 0; i--) {
        SCL_LOW;
        half_period();

        SCL_HIGH;
        if (SDA) /* get bit */
            bI2cRxByte = bI2cRxByte | (1 << i);
        half_period();
    }

    return bI2cRxByte;
}

/* slaver ack */
uint8_t SlaveAck(void)
{
    uint8_t nack = 0;

    SCL_LOW;
    SET_SDA_INPUT;  // set SDA input to get slave ack
    half_period();

    SCL_HIGH;
    half_period();

    SCL_LOW;
    if (SDA)
        nack = 0x01;
    else
        nack = 0x00;
    SDA_HIGH;
    SET_SDA_OUTPUT;
    half_period();

    return nack;
}

/* master ack */
void MasterAck(void)
{
    SCL_LOW;
    SDA_LOW;
    SET_SDA_OUTPUT;
    half_period();

    SCL_HIGH;
    half_period();
}

/* master nack */
void MasterNAck(void)
{
    SCL_LOW;
    SDA_HIGH;
    SET_SDA_OUTPUT;
    half_period();

    SCL_HIGH;
    half_period();
}

/* master stop */
void MasterStop(void)
{
    SCL_LOW;
    half_period();

    SDA_LOW;
    half_period();

    SCL_HIGH;
    stop_set_time();

    SDA_HIGH;
}

/* master start */
void MasterStart(void)
{
    SDA_HIGH;
    half_period();

    SCL_HIGH;
    repeat_start_set_time();

    SDA_LOW;
    repeat_start_hold_time();

    SCL_LOW;
    half_period();
}

/* write data to IC */
uint8_t write_i2c(uint8_t DeviceAddr,
                  uint8_t RegAddr,
                  uint8_t RegAmount,
                  uint8_t *pData)
{
    /* Master write register address*/
    MasterStart();

    I2cTxByte(DeviceAddr << 1);
    if (SlaveAck()) {
        MasterStop();  // Slave NACK, master stop
        return 0x01;
    }

    if (RegAmount == 0) {
        MasterStop();
        return 0x00;
    }

    I2cTxByte(RegAddr);
    if (SlaveAck()) {
        MasterStop();
        return 0x01;
    }

    /* writing data */
    for (int i = 0; i < RegAmount; i++) {
        I2cTxByte(*pData);
        if (SlaveAck()) {
            MasterStop();
            return 0x01;
        }
        pData++;
    }

    MasterStop();

    SDA_HIGH;
    SCL_HIGH;

    return 0x00;
}

/* read data from IC */
uint8_t read_i2c(uint8_t DeviceAddr,
                 uint8_t RegAddr,
                 uint8_t RegAmount,
                 uint8_t *pData)
{
    /* Master write register address*/
    MasterStart();

    I2cTxByte(DeviceAddr << 1);
    if (SlaveAck()) {
        MasterStop();
        return 0x01;
    }

    I2cTxByte(RegAddr);
    if (SlaveAck()) {
        MasterStop();
        return 0x01;
    }

    /* Master read register data*/
    MasterStart();

    I2cTxByte((DeviceAddr << 1) | 0x01);
    if (SlaveAck()) {
        MasterStop();
        return 0x01;
    }

    /* reading data */
    for (int i = 0; i < RegAmount; i++) {
        *pData = I2cRxByte();
        if (i == RegAmount - 1)
            MasterNAck();
        else
            MasterAck();
        pData++;
    }

    MasterStop();

    SDA_HIGH;
    SCL_HIGH;

    return 0x00;
}

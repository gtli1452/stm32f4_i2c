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
#define SDA GPIOB->IDR & GPIO_PIN_15

volatile i2c_packet_t i2c;
volatile uint32_t gI2cTimeout;

static void nop(void)
{
    /*
     * __nop() ~= 35.6ns when system clock is 168MHz
     * i = 6, 1.7MHz; i = 38, 400kHz;
     */
    for (int i = 0; i < 38; i++)
        __nop();
}

/* SM transmit byte */
void I2cTxByte(uint8_t I2cTxByte)
{
    uint8_t tmp = I2cTxByte;

    for (int i = 0; i < 8; i++) {
        SCL_LOW;
        if ((tmp & 0x80) == 0x80)
            SDA_HIGH;
        else
            SDA_LOW;
        nop();

        SCL_HIGH;
        nop();

        tmp = tmp << 1;
    }
}

/* SM receive byte */
uint8_t I2cRxByte(void)
{
    uint8_t bI2cRxByte = 0;

    SCL_LOW;
    SET_SDA_INPUT;  // set SDA input to get data
    nop();

    for (int i = 7; i >= 0; i--) {
        SCL_LOW;
        nop();

        SCL_HIGH;
        if (SDA) /* get bit */
            bI2cRxByte = bI2cRxByte | (1 << i);
        nop();
    }

    return bI2cRxByte;
}

/* slaver ack */
uint8_t SlaveAck(void)
{
    uint8_t nack = 0;

    SCL_LOW;
    SET_SDA_INPUT;  // set SDA input to get slave ack
    nop();

    SCL_HIGH;
    nop();

    SCL_LOW;
    if (SDA)
        nack = 0x01;
    else
        nack = 0x00;
    SDA_HIGH;
    SET_SDA_OUTPUT;
    nop();

    return nack;
}

/* master ack */
void MasterAck(void)
{
    SCL_LOW;
    SDA_LOW;
    SET_SDA_OUTPUT;
    nop();

    SCL_HIGH;
    nop();
}

/* master nack */
void MasterNAck(void)
{
    SCL_LOW;
    SDA_HIGH;
    SET_SDA_OUTPUT;
    nop();

    SCL_HIGH;
    nop();
}

/* master stop */
void MasterStop(void)
{
    SCL_LOW;
    nop();

    SDA_LOW;
    nop();

    SCL_HIGH;
    nop();

    SDA_HIGH;
}

/* master start */
void MasterStart(void)
{
    SDA_HIGH;
    nop();

    SCL_HIGH;
    nop();

    SDA_LOW;
    nop();

    SCL_LOW;
    nop();
}

/* write data to IC */
uint8_t write_i2c(uint8_t DeviceAddr,
                uint8_t RegAddr,
                uint8_t RegAmount,
                uint8_t *pData)
{
    /* Master write register address*/
    MasterStart();

    I2cTxByte(DeviceAddr);
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

    I2cTxByte(DeviceAddr);
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

    I2cTxByte(DeviceAddr | 0x01);
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

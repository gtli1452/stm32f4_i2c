#include "i2c.h"
#include "stm32f4xx.h"

volatile I2C_transaction_TypeDef i2c;
volatile uint32_t gHasExeOnce;
volatile uint32_t gI2cTimeout;

/* this handler deals with master read and master write only */

#define SCL_HIGH GPIOB->ODR |= (1 << 0)  // pb0
#define SCL_LOW GPIOB->ODR &= ~(1 << 0)
#define SDA_HIGH GPIOB->ODR |= (1 << 1)  // pb1
#define SDA_LOW GPIOB->ODR &= ~(1 << 1)
#define SET_SDA_INPUT GPIOB->MODER &= ~(3UL << (2 * 1))  // set PB1 input
#define SET_SDA_OUTPUT GPIOB->MODER |= (1UL << (2 * 1))  // set PB1 output
#define SDA GPIOB->IDR & 0x02

void nop(void)
{
    /*
     * __nop() ~= 35.6ns when system clock is 168MHz
     * i = 6, 1.7MHz; i = 38, 400kHz;
     */
    for (int i = 0; i < 38; i++)
        __nop();
}

void NopCycle(int max)
{
    for (int i = 0; i < max; i++)
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
    SET_SDA_INPUT; // set SDA input to get slave ack
    nop();

    SCL_HIGH;
    nop();

    SCL_LOW;
    if (SDA) {
        SDA_HIGH;
        nack = 0x01;
    } else {
        SDA_LOW;
        nack = 0x00;
    }
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
void MasterNAck()
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
uint8_t WriteIc(uint8_t DeviceAddr,
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
uint8_t ReadIc(uint8_t DeviceAddr,
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

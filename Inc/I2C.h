#ifndef __I2C_H 
#define __I2C_H

#ifndef __stdint_h
    #include "stdint.h"
#endif

typedef struct I2C_PACKET {
    uint8_t state;
    uint8_t device_addr;
    uint8_t reg_addr;
    uint8_t RW;
    uint32_t data_length;
    uint8_t data[256];
} i2c_packet_t;

extern volatile i2c_packet_t i2c;

uint8_t write_i2c(uint8_t device_addr,
                  uint8_t reg_addr,
                  uint8_t reg_amount,
                  uint8_t *pData);
                  
uint8_t read_i2c(uint8_t device_addr,
                 uint8_t reg_addr,
                 uint8_t reg_amount,
                 uint8_t *pData);

void I2C_IRQHandler(void);

#endif

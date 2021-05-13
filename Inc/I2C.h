#ifndef __stdint_h
	#include "stdint.h"
#endif

typedef struct {
    uint8_t state;
    uint8_t device_addr;
    uint8_t reg_addr;
    uint8_t RW;
    uint32_t data_length;
    uint8_t data[256];
} I2C_transaction_TypeDef;

extern uint8_t WriteIc(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t RegAmount, uint8_t* pData);
extern uint8_t ReadIc(uint8_t DeviceAddr, uint8_t RegAddr, uint8_t RegAmount, uint8_t* pData);

#define coreClock 48000000
#define I2CONSET_I2EN       (0x1<<6)  /* I2C Control Set Register */
#define I2CONSET_AA         (0x1<<2)
#define I2CONSET_SI         (0x1<<3)
#define I2CONSET_STO        (0x1<<4)
#define I2CONSET_STA        (0x1<<5)

#define I2CONCLR_AAC        (0x1<<2)  /* I2C Control clear Register */
#define I2CONCLR_SIC        (0x1<<3)
#define I2CONCLR_STAC       (0x1<<5)
#define I2CONCLR_I2ENC      (0x1<<6)

extern void I2C_IRQHandler(void);
extern void I2cInitial(void);
extern uint32_t I2CEngine(void);
extern volatile I2C_transaction_TypeDef i2c;

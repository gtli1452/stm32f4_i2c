#ifndef __stdint_h
	#include "stdint.h"
#endif



typedef struct
{
	uint8_t U8I2cState;
	uint8_t U8DeviceAddress;
	uint8_t U8Index;
	uint32_t U32RW;
	uint32_t U32DataLength;
	uint32_t U32DataCounter;
	uint8_t U8Data[256];
	uint8_t caseitem;
}I2C_transaction_TypeDef;

extern uint8_t WriteIc(uint8_t U8DeviceAddress, uint8_t U8IndexAddress, uint8_t U8RegisterAmount, uint8_t* pU8Data);
extern uint8_t ReadIc(uint8_t U8DeviceAddress, uint8_t U8IndexAddress, uint8_t U8RegisterAmount, uint8_t* pU8Data);

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

extern void I2C_IRQHandler( void );
extern void I2cInitial(void);
extern uint32_t I2CEngine( void );
extern volatile I2C_transaction_TypeDef I2C_transaction;


#ifndef __UART_H 
#define __UART_H

#ifndef __stdint_h
	#include "stdint.h"
#endif

/* State machine definition */
#define IDLE         0
#define SET_I2C_ADDR 1
#define WRITE_I2C    2
#define READ_I2C     3
#define WRITE_SPI    4
#define READ_SPI     5
#define SET_IO       6

void init_uart(void);

void write_uart(uint8_t *pBuffer, uint32_t length);

#endif /* end __UART_H */

#include "stdint.h"

#ifndef __UART_H 
#define __UART_H


extern void UART_IRQHandler(void);
extern void UartInitial(void);
void UARTSend(uint8_t *pBuffer, uint32_t Length);

#define IDLE               0
#define SET_ADDRESS        1
#define WRITE_I2C          2
#define READ_I2C           3
#define G2194_FPGA_TEST    4
#define WRITE_READ_I2C     6
#define MODE_SELECTION    10



#endif /* end __UART_H */

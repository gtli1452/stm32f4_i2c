#ifndef __TIMER7_H 
#define __TIMER7_H

#define UART_TIMEOUT_MODE      0
#define I2C_TIMEOUT_MODE       1

void EnableTimer7(void);
void DisableTimer7(void);
void ResetTimer7(void);
void InitialTimer7(void);

#endif

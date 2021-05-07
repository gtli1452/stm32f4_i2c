#ifndef __TIMER7_H 
#define __TIMER7_H

#ifndef __stdint_h
	#include "stdint.h"
#endif

#define UART_TIMEOUT_MODE      0
#define I2C_TIMEOUT_MODE       1

void EnableTimer7(uint32_t mode);
void DisableTimer7(void);
void ResetTimer7(void);
void InitialTimer7(void);

#endif

#ifndef __TIMER7_H 
#define __TIMER7_H

#ifndef __stdint_h
	#include "stdint.h"
#endif

#define UART_TIMEOUT_MODE      0
#define I2C_TIMEOUT_MODE       1

void enable_timer7(uint32_t mode);
void disable_timer7(void);
void reset_timer7(void);
void init_timer7(void);

#endif

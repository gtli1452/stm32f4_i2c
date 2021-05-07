#include "TIMER7.h"
#include "UART.h"
#include "stm32f4xx.h"

extern volatile uint32_t gExecution;
extern volatile uint32_t gI2cTimeout;
extern volatile uint32_t rx_count;

volatile uint32_t timeOutMode;

void TIM7_IRQHandler(void)
{
    if (TIM7->SR) {
        TIM7->SR &= ~(0x0001); /* clear interrupt flag */
        switch (timeOutMode) {
        case UART_TIMEOUT_MODE:
            gExecution = IDLE;
            break;
        case I2C_TIMEOUT_MODE:
            gI2cTimeout = 1;
            break;
        default:
            break;
        }
    }

    DisableTimer7();
}

/******************************************************************************
** Function name:		EnableTimer16B0
**
** Descriptions:		Enable timer
**
** parameters:		None
** Returned value:	None
**
******************************************************************************/
void EnableTimer7(void)
{
    TIM7->CR1 = 1;
    return;
}

/******************************************************************************
** Function name:		DisableTimer16B0
**
** Descriptions:		Disable timer
**
** parameters:		None
** Returned value:	None
**
******************************************************************************/
void DisableTimer7(void)
{
    TIM7->CR1 = 0;
    return;
}

/******************************************************************************
** Function name:		ResetTimer16B0
**
** Descriptions:		Reset timer
**
** parameters:		None
** Returned value:	None
**
******************************************************************************/
void ResetTimer7(void)
{
    TIM7->CNT = 0;
    return;
}

/******************************************************************************

** Function name:		init_timer
**
** Descriptions:		Initialize timer, set timer interval, reset timer,
**				install timer interrupt handler
**
** parameters:		timer number and timer interval
** Returned value:	None
**
******************************************************************************/
void InitialTimer7(void)
{
    SCB->AIRCR = 0x05AF0000 | 0x400;
    RCC->APB1ENR |= (1 << 5);  // enable timer7 clock
    TIM7->PSC = 83999;         // 84MHz/(83999+1) = 1000 Hz
    TIM7->ARR = 10;            // 10ms
    TIM7->CNT = 0;             // reset counter
    TIM7->DIER |= (1 << 0);
    TIM7->DIER |= (1 << 6);
    NVIC->IP[55] = 0x80;
    NVIC->ISER[1] |= (1 << (55 - 32));
    return;
}

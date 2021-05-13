#include "timer7.h"
#include "stm32f4xx.h"
#include "uart.h"

extern volatile uint32_t state_machine;
extern volatile uint32_t gI2cTimeout;
extern volatile uint32_t rx_count;

static volatile uint32_t timeout_mode;

void TIM7_IRQHandler(void)
{
    if (TIM7->SR) {
        TIM7->SR &= ~(0x0001); /* clear interrupt flag */
        switch (timeout_mode) {
        case UART_TIMEOUT_MODE:
            state_machine = IDLE;
            break;
        case I2C_TIMEOUT_MODE:
            gI2cTimeout = 1;
            break;
        default:
            break;
        }
    }

    disable_timer7();
}

void enable_timer7(uint32_t mode)
{
    TIM7->CR1 = 1;
    timeout_mode = mode;
}

void disable_timer7(void)
{
    TIM7->CR1 = 0;
}

void reset_timer7(void)
{
    TIM7->CNT = 0;
}

void init_timer7(void)
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
}

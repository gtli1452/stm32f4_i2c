#include "uart.h"
#include "i2c.h"
#include "timer7.h"
#include "stm32f4xx.h"

#define rx_data USART1->DR
#define tx_data USART1->DR

extern volatile uint32_t state_machine;

static volatile uint32_t rx_count;
static volatile uint8_t ap_cmd;

void USART1_IRQHandler(void)
{
    /* read interrupt */
    while (USART1->SR & USART_SR_RXNE) {
        USART1->SR &= ~USART_SR_RXNE; /* clear interrupt */
        if (rx_count == 0) {
            ap_cmd = rx_data;
            rx_count++;
            EnableTimer7(UART_TIMEOUT_MODE);
        } else {
            switch (ap_cmd) {
            case 'a':  // write to IC registers
                if (rx_count == 1)
                    i2c.reg_addr = rx_data;
                else if (rx_count == 2)
                    i2c.data_length = rx_data;
                else {
                    i2c.data[rx_count - 3] = rx_data;
                    if (rx_count == (i2c.data_length + 2))
                        state_machine = WRITE_I2C;
                }
                break;
            case 'b':  // read the IC internal registers
                if (rx_count == 1)
                    i2c.reg_addr = rx_data;
                else {
                    i2c.data_length = rx_data;
                    state_machine = READ_I2C;
                }
                break;
            case 'e':  // set device address
                if (rx_count == 1) {
                    i2c.device_addr = rx_data;
                    state_machine = SET_ADDRESS;
                }
                break;
            default:
                break;
            }
            rx_count++;
        }
    }

    /* received all data */
    if (state_machine != IDLE) {
        ap_cmd = 0;
        rx_count = 0;
        DisableTimer7();
    } else {

    }
}

void UartInitial(void)
{
    /* Enable GPIOA clock */
    RCC->AHB1ENR |= (1UL << 0);

    /* set PA.9 to TX */
    GPIOA->MODER &= ~(3UL << 2 * 9);  // PA.9(TX) is alternative function
    GPIOA->MODER |= (2UL << 2 * 9);
    GPIOA->OTYPER &= ~(1UL << 9);     // PA.9 is push pull
    GPIOA->PUPDR &= ~(3UL << 2 * 9);  // PA.9 is Pull up
    GPIOA->PUPDR |= (1UL << 2 * 9);
    GPIOA->OSPEEDR &= ~(3UL << 2 * 9);  // PA.9 is 50MHz High Speed
    GPIOA->OSPEEDR |= (3UL << 2 * 9);

    /* set PA.10 RX */
    GPIOA->MODER &= ~(3UL << 2 * 10);  // PA.10(RX) is alternative function
    GPIOA->MODER |= (2UL << 2 * 10);
    GPIOA->OTYPER &= ~(1UL << 10);     // PA.10 is push pull
    GPIOA->PUPDR &= ~(3UL << 2 * 10);  // PA.10 is Pull up
    GPIOA->PUPDR |= (1UL << 2 * 10);
    GPIOA->OSPEEDR &= ~(3UL << 2 * 10);  // PA.10 is 25MHz High Speed
    GPIOA->OSPEEDR |= (3UL << 2 * 10);

    GPIOA->AFR[1] &= ~(0xfUL << 4 * 1);  // PA.9 is TX of USART1
    GPIOA->AFR[1] |= (7UL << 4 * 1);
    GPIOA->AFR[1] &= ~(0xfUL << 4 * 2);  // PA.10 is RX of USART1
    GPIOA->AFR[1] |= (7UL << 4 * 2);

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock	(1UL<<4)
    USART1->CR1 |= (USART_CR1_TE |         // Transmitter enable
                    USART_CR1_RE |         // Receiver enable
                    USART_CR1_RXNEIE);

    USART1->BRR = 0x2D9;  // baurd rate : 115200 CLK = 84Mhz

    USART1->CR1 |= USART_CR1_UE;  // USART enable
    NVIC_EnableIRQ(USART1_IRQn);
}

void UARTSend(uint8_t *pBuffer, uint32_t Length)
{
    while (Length != 0) {
        while (!(USART1->SR & USART_SR_TXE))
            ;
        tx_data = *pBuffer;
        pBuffer++;
        Length--;
    }
    return;
}

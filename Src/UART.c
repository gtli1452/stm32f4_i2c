#include "stm32f4xx.h"
#include "UART.h"
#include "I2C.h"
#include "TIMER7.h"

volatile uint32_t gUartReceiveCounter;
volatile uint32_t gExecution;
volatile uint8_t gCommand;
volatile uint32_t gTimer7TimeOutMode;

extern volatile uint32_t gHasExeOnce;
extern volatile uint32_t gI2cTimeout;
extern volatile uint32_t counter;

#define SCL_HIGH GPIOB->ODR |= (1<<0)
#define SCL_LOW GPIOB->ODR &= ~(1<<0)
#define SDA_HIGH GPIOB->ODR |= (1<<1)
#define SDA_LOW GPIOB->ODR &= ~(1<<1)

void USART1_IRQHandler(void)
{
	volatile uint32_t status;

	while(((status = USART1->SR)&USART_SR_RXNE)==USART_SR_RXNE)             /* read interrupt                   */
	{
		USART1->SR &= ~USART_SR_RXNE;//clear interrupt
		if(gUartReceiveCounter ==0)
		{				     
			gCommand = USART1->DR;
			gUartReceiveCounter++;
			EnableTimer7();
			gTimer7TimeOutMode = UART_TIMEOUT_MODE;
		}
		else
		{
			switch(gCommand)
			{
				case 'a' :  //write to IC registers
									SDA_HIGH;
									SCL_HIGH;
									if(gUartReceiveCounter == 1)  
										I2C_transaction.Index = USART1->DR;
									else if(gUartReceiveCounter == 2)  
										I2C_transaction.DataLength = USART1->DR; 
									else if(gUartReceiveCounter>2)
									{
										I2C_transaction.Data[gUartReceiveCounter-3] = USART1->DR;
										if(gUartReceiveCounter==(I2C_transaction.DataLength+2))
								    {
											gExecution= WRITE_I2C;
											DisableTimer7();
								    }																				
									}
					break;
					case 'b' : 	//read the IC internal registers

									if(gUartReceiveCounter == 1)  
										I2C_transaction.Index = USART1->DR;
									else if(gUartReceiveCounter == 2)
									{
											I2C_transaction.DataLength = USART1->DR;
											gExecution= READ_I2C;
											DisableTimer7();
									}
					break;		
									
						 case 'e' : //Set Device address	         
									if(gUartReceiveCounter == 1)
									{	
										I2C_transaction.DeviceAddress = USART1->DR;
										gExecution= SET_ADDRESS;
										DisableTimer7();
									}
						  break;
									
						case 'x' : //writeread to IC registers
							if(gUartReceiveCounter == 1)  
								I2C_transaction.Index = USART1->DR;
              else if(gUartReceiveCounter == 2)  
								I2C_transaction.DataLength = USART1->DR;
              else if(gUartReceiveCounter>2)
							{
								I2C_transaction.Data[gUartReceiveCounter-3] = USART1->DR;
							
									gExecution= WRITE_READ_I2C;
									DisableTimer7();								
							}						
						break;
							
            default:	 
						break;							
 			}	
				gUartReceiveCounter++;			
		}
	}
}	

void UartInitial(void)
{
	RCC->AHB1ENR |= (1UL << 0);				  		// Enable GPIOA clock 
	GPIOA->MODER &= ~(3UL << 2*9);         	// PA.9(TX) is alternative function  
	GPIOA->MODER |= (2UL << 2*9);
	GPIOA->OTYPER &= ~(1UL<<9);							// PA.9 is push pull
	GPIOA->PUPDR &= ~(3UL << 2*9);      		// PA.9 is Pull up
	GPIOA->PUPDR |= (1UL << 2*9);
	GPIOA->OSPEEDR &= ~(3UL << 2*9);       	// PA.9 is 50MHz High Speed 
	GPIOA->OSPEEDR |= (3UL << 2*9);         
	
	GPIOA->MODER &= ~(3UL << 2*10);        	// PA.10(RX) is alternative function  
	GPIOA->MODER |= (2UL << 2*10);
	GPIOA->OTYPER &= ~(1UL<<10);				  	// PA.10 is push pull
	GPIOA->PUPDR &= ~(3UL << 2*10);      		// PA.10 is Pull up
	GPIOA->PUPDR |= (1UL << 2*10);      
	GPIOA->OSPEEDR  &= ~(3UL << 2*10);    // PA.10 is 25MHz High Speed 
	GPIOA->OSPEEDR  |= (3UL << 2*10);   
	
	GPIOA->AFR[1] &= ~(0xfUL << 4*1);				// PA.9 is TX of USART1 spec reference p.61
	GPIOA->AFR[1] |=  (7UL << 4*1);
	GPIOA->AFR[1] &= ~(0xfUL << 4*2);				// PA.10 is RX of USART1
	GPIOA->AFR[1] |= (7UL << 4*2);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   // Enable USART1 clock	(1UL<<4)
	USART1->CR1 |= (USART_CR1_TE | 					// Trasmitter enable
	                USART_CR1_RE |					// Receiver enable
									USART_CR1_RXNEIE 									
									);						

	USART1->BRR = 0x2D9;	// baurd rate : 115200 CLK = 84Mhz

	USART1->CR1 |= USART_CR1_UE;						// USART enable	
	NVIC_EnableIRQ(USART1_IRQn);
  
}

void UARTSend(uint8_t *pBuffer, uint32_t Length)
{	
  while(Length != 0 )
  {	
    while ( !(USART1->SR & USART_SR_TXE));
		USART1->DR = *pBuffer;
    pBuffer++;
    Length--;
  }
  return;
}



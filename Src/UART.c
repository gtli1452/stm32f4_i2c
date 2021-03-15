#include "stm32f4xx.h"
#include "UART.h"
#include "I2C.h"
#include "TIMER7.h"

volatile uint32_t gU32UartReceiveCounter;
volatile uint32_t gU32Execution;
volatile uint8_t gU8ModeSelection;
volatile uint8_t gU8Command;
volatile uint32_t gU32Timer7TimeOutMode;

extern volatile uint32_t gU32HasExeOnce;
extern volatile uint32_t gU32I2cTimeout;
volatile uint32_t gU32Timer7TimeOutMode;
extern volatile uint32_t counter;
volatile uint8_t dataflag;
volatile uint8_t indexflag;
volatile uint8_t ackflag;
volatile uint8_t ackflag2;

#define SCL_HIGH GPIOB->ODR |= (1<<0)
#define SCL_LOW GPIOB->ODR &= ~(1<<0)
#define SDA_HIGH GPIOB->ODR |= (1<<1)
#define SDA_LOW GPIOB->ODR &= ~(1<<1)

void USART1_IRQHandler(void)
{
	volatile uint32_t status;
	//uint8_t temp = 0x55;
	//UARTSend(&temp, 1);
	while(((status = USART1->SR)&USART_SR_RXNE)==USART_SR_RXNE)             /* read interrupt                   */
	{
		USART1->SR &= ~USART_SR_RXNE;//clear interrupt
		if(gU32UartReceiveCounter ==0)
		{				     
			gU8Command = USART1->DR;
			gU32UartReceiveCounter++;
			EnableTimer7();
			gU32Timer7TimeOutMode = UART_TIMEOUT_MODE;
		}
		else
		{
			switch(gU8Command)
			{
				case 'a' :  //write to IC registers
									SDA_HIGH;
									SCL_HIGH;
									dataflag=1;
									indexflag=1;
									ackflag=1;
									ackflag2=1;
									if(gU32UartReceiveCounter == 1)  
										I2C_transaction.U8Index = USART1->DR;
									else if(gU32UartReceiveCounter == 2)  
										I2C_transaction.U32DataLength = USART1->DR; 
									else if(gU32UartReceiveCounter>2)
									{
										I2C_transaction.U8Data[gU32UartReceiveCounter-3] = USART1->DR;
										if(gU32UartReceiveCounter==(I2C_transaction.U32DataLength+2))
								    {
											gU32Execution= WRITE_I2C;
											DisableTimer7();
								    }																				
									}
					break;
					case 'b' : 	//read the IC internal registers

									if(gU32UartReceiveCounter == 1)  
										I2C_transaction.U8Index = USART1->DR;
									else if(gU32UartReceiveCounter == 2)
									{
 										//if(gU32UartReceiveCounter==(I2C_transaction.U32DataLength+2))
 										//{
											I2C_transaction.U32DataLength = USART1->DR;
											gU32Execution= READ_I2C;
											DisableTimer7();
 										//}
									}
					break;		
									
						 case 'e' : //Set Device address	         
									if(gU32UartReceiveCounter == 1)
									{	
										I2C_transaction.U8DeviceAddress = USART1->DR;
										gU32Execution= SET_ADDRESS;
										DisableTimer7();
									}
						  break;
									
						case 'x' : //writeread to IC registers
							if(gU32UartReceiveCounter == 1)  
								I2C_transaction.U8Index = USART1->DR;
              else if(gU32UartReceiveCounter == 2)  
								I2C_transaction.U32DataLength = USART1->DR;
              else if(gU32UartReceiveCounter>2)
							{
								I2C_transaction.U8Data[gU32UartReceiveCounter-3] = USART1->DR;
							
									gU32Execution= WRITE_READ_I2C;
									DisableTimer7();								
							}						
						break;
							
            default:	 
						break;							
 			}	
				gU32UartReceiveCounter++;			
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

void UARTSend(uint8_t *pU8Buffer, uint32_t U32Length)
{	
  while(U32Length != 0 )
  {	
    while ( !(USART1->SR & USART_SR_TXE));
		USART1->DR = *pU8Buffer;
    pU8Buffer++;
    U32Length--;
  }
  return;
}



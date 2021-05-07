#include "I2C.h"
#include "stm32f4xx.h"

volatile I2C_transaction_TypeDef I2C_transaction;
volatile uint32_t gHasExeOnce;
volatile uint32_t gI2cTimeout;
volatile uint32_t testflag;

/* this handler deals with master read and master write only */
	
#define SCL_HIGH GPIOB->ODR |= (1<<0)  //pb0
#define SCL_LOW  GPIOB->ODR &= ~(1<<0) 
#define SDA_HIGH GPIOB->ODR |= (1<<1) //pb1
#define SDA_LOW  GPIOB->ODR &= ~(1<<1)
#define SDA GPIOB->IDR & 0x02

void nop(void) 
{
	int i;
	// i = 6, 1.7MHz; i = 38, 400kHz;
	for(i = 0; i < 38;i++) // __nop() ~= 35.6ns when system clock is 168MHz
			__nop();
	
	// for 3.4MHz
	/*
			__nop();
			__nop();
			__nop();
			__nop();
			__nop();
	*/
}
void NopCycle(int max)
{
	for (int i = 0; i < max; i++)
	{
		__nop();
	}
}


/* SM transmit byte */ 
void I2cTxByte(uint8_t I2cTxByte)
{
	uint8_t i = 0;
	uint8_t a = 0;
	
	a = I2cTxByte;
	
	for(i = 0; i < 8; i++)
	{		
		SCL_LOW;
		
		if((a & 0x80) == 0x80)
			SDA_HIGH;
		else
			SDA_LOW;
		a = a << 1;
		nop();
		SCL_HIGH;
		
		nop();

		/*spike
		if(testflag==1)
		{
			if(i==4)
			{
				GPIOB->OTYPER=0x02;
				GPIOB->PUPDR = 0x05;
				//SDA_LOW;
				GPIOB->ODR = 0x01;
				//__nop();
				GPIOB->ODR = 0x03;
				//SDA_HIGH;
				GPIOB->OTYPER=0x03;
		}*/	
	}
}

/* SM receive byte */      
uint8_t I2cRxByte(void)
{
		uint8_t bI2cRxByte = 0;
		
		SDA_HIGH;
		GPIOB->MODER &= ~((3UL << 2*1)); // set PB1 input to receive data
		
		SCL_LOW; // bit8
		nop();
		if(SDA)
				bI2cRxByte = bI2cRxByte | 0x80;
		SCL_HIGH;	
		nop();
		
		SCL_LOW; // bit7	
		nop();
		if(SDA)
				bI2cRxByte = bI2cRxByte | 0x40;
		SCL_HIGH;	
		nop();
		
		SCL_LOW; // bit6
		nop();
		if(SDA)
				bI2cRxByte = bI2cRxByte | 0x20;
		SCL_HIGH;	
		nop();
		
		SCL_LOW; // bit5
		nop();
		if(SDA)
				bI2cRxByte = bI2cRxByte | 0x10;
		SCL_HIGH;	
		nop();
		
		SCL_LOW; // bit4
		nop();
		if(SDA)
				bI2cRxByte = bI2cRxByte | 0x8;
		SCL_HIGH;	
		nop();
		
		SCL_LOW; // bit3
		nop();
		if(SDA)
			bI2cRxByte = bI2cRxByte | 0x4;
		SCL_HIGH;	
		nop();
		
		SCL_LOW; // bit2	
		nop();
		if(SDA)
			bI2cRxByte = bI2cRxByte | 0x2;
		SCL_HIGH;	
		nop();
		
		SCL_LOW; // bit1
		nop();
		if(SDA)
			bI2cRxByte = bI2cRxByte | 0x1;
		SCL_HIGH;	
		nop();
		
		SDA_HIGH;
		GPIOB->MODER  |= (1UL << 2*1); // set PB1 output

		return bI2cRxByte;
}

/* slaver ack */
uint8_t SlaveAck(void)
{
		SCL_LOW;
		//SDA_HIGH;
		GPIOB->MODER &= ~((3UL << 2*1)); // set PB1 input
		nop();

		SCL_HIGH;
		nop();

		if(SDA)
		{
				GPIOB->MODER  |= (1UL << 2*1); // set PB1 output
				SDA_HIGH;
				SCL_LOW;
				nop();
				return 0x01;
		}
		else
		{
				GPIOB->MODER  |= (1UL << 2*1); // set PB1 output
				SDA_LOW;
				SCL_LOW;
				nop();
				return 0x00;
		}
}

/* master ack */
void MasterAck(void)
{
		SCL_LOW;
		__nop();
		__nop();
		__nop();
		__nop();

		SDA_LOW;
		nop();
		SCL_HIGH;
		nop();
}

/* master stop*/
void MasterStop(void)
{
		SCL_LOW;
		nop();
		SDA_LOW;	
		nop();
		SCL_HIGH;
		//nop();
		NopCycle(15);
		SDA_HIGH;
}

void MasterStart(void)
{
		SDA_HIGH;
		nop();
		
		SCL_HIGH;
		nop();

		SDA_LOW;
		nop();
	
		SCL_LOW;
		nop();
}

/* write data to IC */
uint8_t WriteIc(uint8_t DeviceAddress, uint8_t IndexAddress, uint8_t RegisterAmount, uint8_t* pData)
{
		uint8_t Temp1;
		//start
		MasterStart();

		//device address
		testflag=1;
		I2cTxByte(DeviceAddress);
		testflag=0;

		if(SlaveAck())
		{
			// Slave NACK, master stop
			MasterStop();
			return 0x01;
		}

		if(RegisterAmount==0)
		{
			//stop
			MasterStop();
			return 0x00;
		}

		//index Address
		I2cTxByte(IndexAddress);

		if(SlaveAck())
		{
			//stop
			MasterStop();
			return 0x01;
		}

		//writing data
		for(Temp1=0; Temp1<RegisterAmount; Temp1++)
		{
				I2cTxByte(*pData);
				
				if(SlaveAck())
				{
						//stop
						MasterStop();
						return 0x01;
				}
				pData++;
		}      
		 
		//stop
		MasterStop();

		SDA_HIGH;
		SCL_HIGH;

		return 0x00;
}

/* Read data to IC */
uint8_t ReadIc(uint8_t DeviceAddress, uint8_t IndexAddress, uint8_t RegisterAmount, uint8_t* pData)
{
		uint8_t Temp1;
		
		//start
		MasterStart();
	
		//device address
		I2cTxByte(DeviceAddress);
		if(SlaveAck())
		{
			//stop
			MasterStop();
			return 0x01;
		}	 
		 //index Address	
		I2cTxByte(IndexAddress);

		if(SlaveAck())
		{
			//stop
			MasterStop();
			return 0x01;
		}

		//re-start
		MasterStart();

		//device address
		I2cTxByte(DeviceAddress | 0x01);
		
		if(SlaveAck())
		{
			//stop
			MasterStop();
			return 0x01;
		}
		
		for(Temp1=0; Temp1<(RegisterAmount-1); Temp1++)
		{
			 *pData = I2cRxByte();
			 MasterAck();
			 pData++;          
		}   
		 
		//the last byte
		*pData = I2cRxByte();      
		
		//No Ack
/*		SCL_LOW;
		nop();
		SDA_HIGH;
		nop();
		SCL_HIGH;
		nop();
		SCL_LOW;
		nop();
		SDA_LOW;
		nop();
*/		
		//Master no ack
		SCL_LOW;
		SDA_HIGH;
		nop();
		
		SCL_HIGH;
		nop();
		
		//stop
		MasterStop();
		
		SDA_HIGH;
		SCL_HIGH;
		
		return 0x00;
}

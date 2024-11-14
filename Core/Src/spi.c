/***********************************************************************************										 
SPIģʽ1��SPIģʽ2ͨ��w5500.h �ļ���
#define MODE_SPI  1     // ѡ��SPI1  
#define MODE_SPI  2     // ѡ��SPI2
������ѡ��

***********************************************************************************/


#include "stm32h7xx_hal.h"
//#include "config.h"
//#include "socket.h"
//#include "w5500.h"
#include "spi.h"

//#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define SPI_BUFFER_SIZE 255

extern  SPI_HandleTypeDef hspi1;


// Connected to Data Flash
void WIZ_CS(uint8_t val)
{	
/*	if (val == LOW) 
	{  
			HAL_GPIO_WritePin(GPIOF,WIZ_SCS,GPIO_PIN_RESET);
	}
	else if (val == HIGH)
	{
			HAL_GPIO_WritePin(GPIOF,WIZ_SCS,GPIO_PIN_SET);
	}*/
}


uint8_t SPI_SendByte(SPI_HandleTypeDef *hspi,uint8_t TxData)
{
	 uint8_t i,Rxdata;

//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		
   HAL_SPI_TransmitReceive(hspi,&TxData,&Rxdata,1, 5000); 
	 //HAL_Delay(1);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return Rxdata;          		    //�����յ�������		
}

uint8_t SPI_SendByteS(SPI_HandleTypeDef *hspi,uint8_t *TxData,uint8_t len)
{
	 uint8_t Rxdata[255];
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	
   HAL_SPI_TransmitReceive(hspi,TxData,Rxdata,len, 5000);
	
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	 return 1;          		    //�����յ�������		
}

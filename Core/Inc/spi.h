/***********************************************************************************							 
SPIģʽ1��SPIģʽ2ͨ��w5500.h �ļ���
#define MODE_SPI  1     // ѡ��SPI1  
#define MODE_SPI  2     // ѡ��SPI2
������ѡ��

***********************************************************************************/

#ifndef __SPI_H
#define __SPI_H

#include "stm32h7xx_hal.h"

void WIZ_SPI_Init(void);
void WIZ_CS(uint8_t val);
uint8_t SPI_SendByte(SPI_HandleTypeDef *hspi,uint8_t byte);
uint8_t SPI_SendByteS(SPI_HandleTypeDef *hspi,uint8_t *TxData,uint8_t len);
#endif


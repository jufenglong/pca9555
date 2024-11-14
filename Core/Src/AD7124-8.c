#include "AD7124-8.h" 
#include "spi.h"

//////////////////////////////////////////////////////////////////////////////////	 
#define temp_gain 0.000004967053532600403
//extern SPI_HandleTypeDef hspi1;

extern AD7124REGSTRUCT   AD7124REG;
volatile	AD7124Struct AD7124={0};

void AD7124Init(SPI_HandleTypeDef *hspi,uint8_t AD7124_channel)
{
	volatile int i,itemp=0;
	
	AD7124_Reset(hspi,AD7124_channel);
	//////寄存器初始化,地址，长度（几个字节）
	AD7124REG.STATUS.adr=0X00;  
	AD7124REG.STATUS.len=1; 
	
	AD7124REG.ADC_CONTROL.adr=0X01;  
	AD7124REG.ADC_CONTROL.len=2; 
	
	AD7124REG.DATA.adr=0X02;  
	AD7124REG.DATA.len=3; 
	
//	AD7124REG.IO_CONTROL_1.adr=0X03;  
//	AD7124REG.IO_CONTROL_1.len=3; 
//	AD7124REG.IO_CONTROL_1.value.all = 0x003F40;
	
	AD7124REG.IO_CONTROL_2.adr=0X04;  
	AD7124REG.IO_CONTROL_2.len=2; 
	AD7124REG.IO_CONTROL_1.value.all = 0x00;

	AD7124REG.ID.adr=0X05;  
	AD7124REG.ID.len=1; 
	AD7124REG.ERR.adr=0X06;  
	AD7124REG.ERR.len=3; 
	AD7124REG.ERR_EN.adr=0X07;  
	AD7124REG.ERR_EN.len=3; 
	AD7124REG.MCLK_COUNT.adr=0X08;  
	AD7124REG.MCLK_COUNT.len=1; 
	
	for(i=0; i<17; i++)
	{
			AD7124REG.CHANNEL[i].adr=i+0x09;
			AD7124REG.CHANNEL[i].len=2;
	}
	for(i=0; i<8; i++)
	{
			AD7124REG.CONFIG[i].adr=i+0x19;
			AD7124REG.CONFIG[i].len=2;
	}	
	for(i=0; i<8; i++)
	{
			AD7124REG.FLITER[i].adr=i+0x21;
			AD7124REG.FLITER[i].len=3;
	}		
		for(i=0; i<8; i++)
	{
			AD7124REG.Offset[i].adr=i+0x29;
			AD7124REG.Offset[i].len=3;
	}		
		for(i=0; i<8; i++)
	{
			AD7124REG.GAIN[i].adr=i+0x31;
			AD7124REG.GAIN[i].len=3;
	}		
	///////////
	HAL_Delay(100);
	AD7124REG.ID.value = AD7124ReadREG(hspi,AD7124REG.ID.adr, AD7124REG.ID.len,AD7124_channel);
	//ADC控制寄存器
/*	AD7124REG.ADC_CONTROL.value.bit.DOUT_RDY_DEL=0;
	AD7124REG.ADC_CONTROL.value.bit.CONT_READ=0;
	AD7124REG.ADC_CONTROL.value.bit.DATA_STATUS=1;
	AD7124REG.ADC_CONTROL.value.bit.CS_EN=1;//置1时， DOUT/RDY引脚在SCLK无效沿之后继续用作DOUT引脚。当CS变为高电平时，该引脚变为RDY引脚。
	AD7124REG.ADC_CONTROL.value.bit.REF_EN=1;//内部基准输出使能
	AD7124REG.ADC_CONTROL.value.bit.POWER_MODE=3;//0:低功耗，1：中功率，23：全功率
	AD7124REG.ADC_CONTROL.value.bit.Mode=0;//0:	连续转换；1：单次转换；2待机模式，3：关断模式，4：空闲模式，5：内部零电平校准，6：内部满偏校准，7：系统零电平校准，8：系统满偏校准
	AD7124.InitERR += AD7124WriteREG(hspi,AD7124REG.ADC_CONTROL.adr,AD7124REG.ADC_CONTROL.value.all, AD7124REG.ADC_CONTROL.len,AD7124_channel);
 	
	AD7124REG.IO_CONTROL_1.adr=0X03;  
	AD7124REG.IO_CONTROL_1.len=3; 
	AD7124REG.IO_CONTROL_1.value.all = 0x000700 ;
	AD7124.InitERR += AD7124WriteREG(hspi,AD7124REG.IO_CONTROL_1.adr,AD7124REG.IO_CONTROL_1.value.all, AD7124REG.IO_CONTROL_1.len,AD7124_channel);
 	
	//通道寄存器0
	AD7124REG.CHANNEL[0].value.bit.Enable=1;
  AD7124REG.CHANNEL[0].value.bit.Setup=0;
	AD7124REG.CHANNEL[0].value.bit.AINP=1;
	AD7124REG.CHANNEL[0].value.bit.AINM=2;//0X11-AVSS
	AD7124.InitERR += AD7124WriteREG(hspi,AD7124REG.CHANNEL[0].adr,AD7124REG.CHANNEL[0].value.all, AD7124REG.CHANNEL[0].len,AD7124_channel);

	AD7124REG.CONFIG[0].value.bit.Bipolar=1;//0:
	AD7124REG.CONFIG[0].value.bit.Burnout=0;
	AD7124REG.CONFIG[0].value.bit.REF_BUFP=0;//1：基准缓冲打开
	AD7124REG.CONFIG[0].value.bit.REF_BUFP=0;//1:基准缓冲打开
	AD7124REG.CONFIG[0].value.bit.AIN_BUFP=0;
	AD7124REG.CONFIG[0].value.bit.AIN_BUFM=0;
	AD7124REG.CONFIG[0].value.bit.REF_SEL=2;//0:REFIN1+-; 2:internal
	AD7124REG.CONFIG[0].value.bit.PGA=5;
	AD7124.InitERR += AD7124WriteREG(hspi,AD7124REG.CONFIG[0].adr,AD7124REG.CONFIG[0].value.all, AD7124REG.CONFIG[0].len,AD7124_channel); 
*/
}	

uint16_t AD7124ReadID(SPI_HandleTypeDef *hspi)
{
  uint16_t ID;   
  ID=  AD7124ReadREG(hspi,0x05,1,0); 
  return ID;
}



int32_t AD7124_Reset(SPI_HandleTypeDef *hspi,uint8_t AD7124_channel)
{
	int32_t i = 0;
	uint32_t value;
	uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	if(hspi->Instance==SPI1)
    {
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
      
		}
		else if(hspi->Instance==SPI2)
		{
			 switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI3){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI4){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
			}
		}
		
		for(i=0;i<8;i++)
		{
			 value = SPI_SendByte(hspi,wrBuf[i]);
		}
		//HAL_Delay(2);
		if(hspi->Instance==SPI1){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break; 
					}
				}
				else if(hspi->Instance==SPI2){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
			}
					}
				else if(hspi->Instance==SPI3){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				}
					}
				else if(hspi->Instance==SPI4){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	  
					break;
				default:
					//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
			}
			}
		}

//读寄存器
uint32_t AD7124ReadREG(SPI_HandleTypeDef *hspi,uint8_t adr,uint8_t len,uint8_t AD7124_channel)
{
	uint32_t value,temp=0;  
	uint8_t readdata,i;
  value=0; readdata=0;
 
	readdata = 0X04<<4;
	readdata |= adr;
	
	if(hspi->Instance==SPI1)
    {
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
      
		}
		else if(hspi->Instance==SPI2)
		{
			 switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI3){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI4){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
			}
		}

  value = SPI_SendByte(hspi,readdata);

	for(i=0;i<len;i++)
	{
			if(i>0)	value<<=8;
			value |=  SPI_SendByte(hspi,0xff); 
	}
	
	//HAL_Delay(2);
 	if(hspi->Instance==SPI1){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break; 
					}
				}
				else if(hspi->Instance==SPI2){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
			}
					}
				else if(hspi->Instance==SPI3){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				}
					}
				else if(hspi->Instance==SPI4){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
			}
		}
		
  return value;
}


//写寄存器
unsigned char AD7124WriteREG(SPI_HandleTypeDef *hspi,uint8_t adr,uint32_t data, uint8_t len,uint8_t AD7124_channel)
{
		uint8_t i=0; uint8_t uctemp=0;
		uint32_t u32temp=0;
  	
		if(hspi->Instance==SPI1)
    {
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
      
		}
		else if(hspi->Instance==SPI2)
		{
			 switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI3){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI4){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
			}
		}
			//--写
			SPI_SendByte(hspi,adr);
			for(i=0;i<len;i++)
			{
					uctemp = data >> 8*(len-i-1);
		 
					SPI_SendByte(hspi,uctemp); 		
			}
			//--读
			u32temp = AD7124ReadREG(hspi,adr,len,AD7124_channel);
			if(u32temp==data)
			{
				if(hspi->Instance==SPI1){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break; 
					}
				}
				else if(hspi->Instance==SPI2){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
			}
					}
				else if(hspi->Instance==SPI3){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				}
					}
				else if(hspi->Instance==SPI4){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
			}
				
			}
			return 0;
		}
			else
			{
					return 1;
			}
 
}



uint32_t AD7124_RDATA(SPI_HandleTypeDef *hspi,uint8_t AD7124_channel)
{
  volatile uint32_t value;  
	volatile uint8_t readdata,i;
  value=0; readdata=0;
 
		if(hspi->Instance==SPI1)
    {
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
      
		}
		else if(hspi->Instance==SPI2)
		{
			 switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI3){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);	
					break;
			}
		}
		else if(hspi->Instance==SPI4){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);	
					break;
			}
		}
//	while(1)
//	{
//			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
//			{
//					break;
//			}		
//	}
  SPI_SendByte(hspi,0X42); 

	for(i=0;i<3;i++)
	{
			if(i>0)	value<<=8;
			value |=  SPI_SendByte(hspi, 0xff); 

	}
  
  if(hspi->Instance==SPI1){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	
					break; 
					}
				}
				else if(hspi->Instance==SPI2){
			switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
			}
					}
				else if(hspi->Instance==SPI3){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);	
					break;
				}
					}
				else if(hspi->Instance==SPI4){
						switch(AD7124_channel)
			{
				case 0: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
				case 1: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);	
					break;
				case 2: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);	
					break;
				case 3: 
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);	  
					break;
				default:
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);	
					break;
			}
		}
	
  return value;
}

void scan_ad7124_channel(SPI_HandleTypeDef *hspi,uint8_t AD7124_scan_index,int *get_ad_data)
{
	uint8_t i;
	switch(AD7124_scan_index)
				{
					case 0:
						get_ad_data[0] = AD7124.CODE = AD7124_RDATA(hspi,AD7124_scan_index);
						break;
					case 1:	
						get_ad_data[1] = AD7124.CODE = AD7124_RDATA(hspi,AD7124_scan_index);
						break;
					case 2:				
						get_ad_data[2] = AD7124.CODE = AD7124_RDATA(hspi,AD7124_scan_index);
						break;	
					case 3:		
						get_ad_data[3] = AD7124.CODE = AD7124_RDATA(hspi,AD7124_scan_index);				
						break;	
					default:
						//AD7124_scan_index	 = 0;
						break;
				}			
}

double AD7124_CodeValue(unsigned char pol, unsigned int code, float vref, float gain)
{
		double dftemp=0;
		//单极性
		if(pol==0)
		{
					dftemp=code*(vref/16777216);
		}
		//双极性
		else
		{
				if(code&0x800000)
				{
								code|=0XFF000000;
				}
				dftemp=code*(vref/16777216);
		}
		dftemp=dftemp*gain;
		return dftemp;

}

float AD7124_tempValue(unsigned int code)
{
		float dftemp=0;
		
//		if(code >= 0x800000)
				{
					//code = (code-0x800000);
					dftemp = (float)code*temp_gain -20;//13584.0)-272.5
				}
/*		else if(code < 0x800000)
		{
					code = (0x800000-code);
					dftemp=-((float)code/13584.0)-272.5;
		}*/
				
		
		return dftemp;
}




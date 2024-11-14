#ifndef __AD71248_H
#define __AD71248_H

#include "main.h"

 typedef struct
 {
	 			//--0X00  STATUS
			struct
			{
						union 
						{
								struct
								{
										uint8_t  CH_ACTIVE	    :4;
										uint8_t 	PRO_FLAG      :1;
										uint8_t 	         	    :1;
										uint8_t  ERR_FLAG    :1;
										uint8_t  	RDY				 :1;
								}bit;
								uint8_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}STATUS; 
			//--0X01  ADC_CONTROL¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint16_t  CLK_SEL	  :2;
										uint16_t 	Mode      :4;
										uint16_t 	POWER_MODE :2;
										uint16_t  REF_EN    :1;
										uint16_t  CS_EN			 :1;
										uint16_t  DATA_STATUS	 :1;
										uint16_t  CONT_READ	 :1;
										uint16_t  DOUT_RDY_DEL	 :1;
									  uint16_t   	 :3;
								}bit;
								uint16_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}ADC_CONTROL; 	 
 			//--0X02  DATA Êý¾Ý¼Ä´æÆ÷
			struct
			{
					uint32_t value;
					uint8_t adr;
					uint8_t len;
			}DATA;
			//--0X03  IO_CONTROL_1 Êý¾Ý¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint16_t  IOUT0_CH	:4;
										uint16_t 	IOUT1_CH  :4;
										uint16_t 	IOUT0 :3;
										uint16_t  IOUT1    :3;
										uint16_t   			 :1;
										uint16_t  PDSW		 :1;
										uint16_t  GPIO_CTRL1	 :1;
										uint16_t  GPIO_CTRL2	 :1;
										uint16_t  GPIO_CTRL3	 :1;
										uint16_t  GPIO_CTRL4	 :1;
										uint16_t  GPIO_DAT1	 :1;
										uint16_t  GPIO_DAT2	 :1;
										uint16_t  GPIO_DAT3	 :1;
										uint16_t  GPIO_DAT4	 :1;									
								}bit;
								uint16_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}IO_CONTROL_1; 	
			//--0X04  IO_CONTROL_2 Êý¾Ý¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint16_t  VBIAS0	:1;
										uint16_t  VBIAS1	:1;
										uint16_t  VBIAS2	:1;
										uint16_t  VBIAS3	:1;
										uint16_t  VBIAS4	:1;
										uint16_t  VBIAS5	:1;
										uint16_t  VBIAS6	:1;
										uint16_t  VBIAS7	:1;
										uint16_t  VBIAS8	:1;
										uint16_t  VBIAS9	:1;
										uint16_t  VBIAS10	:1;
										uint16_t  VBIAS11	:1;
										uint16_t  VBIAS12	:1;
										uint16_t  VBIAS13	:1;
										uint16_t  VBIAS14	:1;
										uint16_t  VBIAS15	:1;
								}bit;
								uint16_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}IO_CONTROL_2; 
 			//--0X05  ID
			struct
			{
					uint8_t value;
					uint8_t adr;
					uint8_t len;
			}ID;
			//--0X06  ERR ´íÎó¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint32_t   	:1;
										uint32_t  MM_CRC_ERR	:1;
										uint32_t  SPI_CRC_ERR	:1;
										uint32_t  SPI_WRITE_ERR	:1;
										uint32_t  SPI_READ_ERR	:1;
										uint32_t  SPI_SCLK_CNT_ERR	:1;
										uint32_t  SPI_IGNORE_ERR	:1;
										uint32_t  ALDO_PSM_ERR 	:1;
										uint32_t   	:1;
										uint32_t  DLDO_PSM_ERR	:1;
										uint32_t   	:1;
										uint32_t  REF_DET_ERR	:1;
										uint32_t  AINM_UV_ERR 	:1;
										uint32_t  AINM_OV_ERR	:1;
										uint32_t  AINP_UV_ERR 	:1;
										uint32_t  AINP_OV_ERR	:1;
										uint32_t  ADC_SAT_ERR	:1;
										uint32_t  ADC_CONV_ERR 	:1;
										uint32_t  ADC_CAL_ERR 	:1;
										uint32_t  LDO_CAP_ERR 	:1;
										uint32_t   	:4;
								}bit;
								uint32_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}ERR;  
			//--0X07  ERROR_EN ´íÎó¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint32_t   	:1;
										uint32_t  MM_CRC_ERR_EN	:1;
										uint32_t  SPI_CRC_ERR_EN	:1;
										uint32_t  SPI_WRITE_ERR_EN	:1;
										uint32_t  SPI_READ_ERR_EN	:1;
										uint32_t  SPI_SCLK_CNT_ERR_EN	:1;
										uint32_t  SPI_IGNORE_ERR_EN	:1;
										uint32_t  ALDO_PSM_ERR_EN 	:1;
										uint32_t  ALDO_PSM_TRIP_TEST_EN 	:1;
										uint32_t  DLDO_PSM_ERR_EN	:1;
										uint32_t  DLDO_PSM_TRIP_TEST_EN 	:1;
										uint32_t  REF_DET_ERR_EN	:1;
										uint32_t  AINM_UV_ERR_EN 	:1;
										uint32_t  AINM_OV_ERR_EN	:1;
										uint32_t  AINP_UV_ERR_EN 	:1;
										uint32_t  AINP_OV_ERR_EN	:1;
										uint32_t  ADC_SAT_ERR_EN	:1;
										uint32_t  ADC_CONV_ERR_EN 	:1;
										uint32_t  ADC_CAL_ERR_EN 	:1;
										uint32_t  LDO_CAP_CHK 	:2;
										uint32_t  LDO_CAP_CHK_TEST_EN 	:1;
										uint32_t  MCLK_CNT_EN 	:1;
										uint32_t    	:1;
								}bit;
								uint32_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}ERR_EN;  
			//--0X08  MCLK_COUNT¼Ä´æÆ÷
			struct
			{
					uint8_t value;
					uint8_t adr;
					uint8_t len;
			}MCLK_COUNT;
			//--0X09~0X18 CHANNEL Í¨µÀ¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint16_t  AINM 	:5;
										uint16_t  AINP 	:5;
										uint16_t   	:2;
										uint16_t  Setup  	:3;
										uint16_t  Enable 	:1;
								}bit;
								uint16_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}CHANNEL[15];
			//--0X19~0X20 CONFIG Í¨µÀ¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint16_t  PGA 	:3;
										uint16_t  REF_SEL 	:2;
										uint16_t  AIN_BUFM 	:1;
										uint16_t  AIN_BUFP 	:1;
										uint16_t  REF_BUFM 	:1;
										uint16_t  REF_BUFP 	:1;
										uint16_t  Burnout  	:2;
										uint16_t  Bipolar  	:1;
										uint16_t     	:4;
								}bit;
								uint16_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}CONFIG[8];
			//--0X21~0X28 FLITER Í¨µÀ¼Ä´æÆ÷
			struct
			{
						union 
						{
								struct
								{
										uint32_t  FS 	:11;
										uint32_t    	:5;
										uint32_t  SINGLE_CYCLE 	:1;
										uint32_t  POST_FILTER 	:3;
										uint32_t  REJ60 	:1;
										uint32_t  Filter 	:3;
								}bit;
								uint32_t all;
						}value;
						uint8_t adr;
						uint8_t len;
			}FLITER[8];
			//--0X29~0X30 OFFSET Í¨µÀ¼Ä´æÆ÷
			struct
			{
					uint32_t value;
					uint8_t adr;
					uint8_t len;
			}Offset[8];
			struct
			{
					uint32_t value;
					uint8_t adr;
					uint8_t len;
			}GAIN[8];
			
 }AD7124REGSTRUCT;
extern   AD7124REGSTRUCT   AD7124REG;
 
  typedef struct 
{
		unsigned  int InitERR;
		unsigned  int CODE;
		double   fvalue;
}AD7124Struct;
extern  volatile AD7124Struct AD7124;
 
float AD7124_tempValue(unsigned int code);

void scan_ad7124_channel(SPI_HandleTypeDef *hspi,uint8_t AD7124_scan_index,int *get_ad_data);

void AD7124Init(SPI_HandleTypeDef *hspi,uint8_t AD7124_channel);
uint16_t AD7124ReadID(SPI_HandleTypeDef *hspi);
int32_t AD7124_Reset(SPI_HandleTypeDef *hspi,uint8_t AD7124_channel);
uint32_t AD7124ReadREG(SPI_HandleTypeDef *hspi,uint8_t adr,uint8_t len,uint8_t AD7124_channel);
unsigned char AD7124WriteREG(SPI_HandleTypeDef *hspi,uint8_t adr,uint32_t data, uint8_t len,uint8_t AD7124_channel);
double AD7124_CodeValue(unsigned char pol, unsigned int code, float vref, float gain);
uint32_t AD7124_RDATA(SPI_HandleTypeDef *hspi,uint8_t AD7124_channel);
#endif

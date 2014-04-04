/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			SST25VF016B.h
** Descriptions:		SST25VF016Bͷ���� 
**
**------------------------------------------------------------------------------------------------------
** Created by:			AVRman
** Created date:		2011-1-26
** Version:				1.0
** Descriptions:		The original version
**
**------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:	
** Version:
** Descriptions:		
********************************************************************************************************/

#ifndef __SST25VF016B_H 
#define __SST25VF016B_H

/* Includes ------------------------------------------------------------------*/
#include "LPC17xx.h"
#include "hal.h"


#if HAL_USE_SSP_SPI || defined(__DOXYGEN__)
/* Private typedef -----------------------------------------------------------*/
/* �������SST25VF016B��������� */
typedef enum ERTYPE{Sec1,Sec8,Sec16,Chip} ErType;  
typedef enum IDTYPE{Manu_ID,Dev_ID,Jedec_ID} idtype;

/* Private define ------------------------------------------------------------*/
#define MAX_ADDR		0x1FFFFF	/* ����оƬ�ڲ�����ַ */
#define	SEC_MAX     	511         /* ������������� */
#define SEC_SIZE		0x1000      /* ������С	*/


/* bit definitions for register SSPSR. */
#define SSPSR_RNE       2
#define SSPSR_BSY       4

/* bit-frequency = PCLK / CPSR, PCLK=72MHz, must be even	*/
#define SPI_SPEED_12MHz		6	/* => 12MHz */
#define SPI_SPEED_18MHz		4	/* => 18MHz */
#define SPI_SPEED_400kHz  180	/* => 400kHz */

#define ERROR				1
#define ENABLE      		0

/* Private function prototypes -----------------------------------------------*/
void SPI_FLASH_Init(SSPSPIDriver *spip, const SSPSPIConfig *config);
uint8_t SSTF016B_RD(uint32_t Dst, uint8_t* RcvBufPt ,uint32_t NByte);
uint8_t SSTF016B_RdID(idtype IDType,uint32_t* RcvbufPt);
uint8_t SSTF016B_WR(uint32_t Dst, uint8_t* SndbufPt,uint32_t NByte);

uint8_t SSTF016B_Erase(uint32_t sec1, uint32_t sec2);
void SPI_FLASH_Test(void);

#endif
#endif
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			SST25VF016B.c
** Descriptions:		SST25VF016B���������� 
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

/* Includes ------------------------------------------------------------------*/
#include "hal.h"
#include "SST25VF016B.h"


#define SPI_FLASH_CS_LOW()    	 sspspiSelect(&SPID1)
#define SPI_FLASH_CS_HIGH()  	 sspspiUnselect(&SPID1)



static SSPSPIConfig spicfg = {
  NULL,
  LPC_GPIO0,
  16,
  7,
  0x40 
};

/*******************************************************************************
* Function Name  : LPC17xx_SPI_SetSpeed
* Description    : Set a SSP0 clock speed to desired value
* Input          : - speed: SPI Speed
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void LPC17xx_SPI_SetSpeed (uint8_t speed)
{
	speed &= 0xFE;
	if ( speed < 2  ) {
		speed = 2 ;
	}
	LPC_SSP0->CPSR = speed;
}
uint32_t LPC17xx_SPI_GetSpeed (uint8_t speed)
{
	speed &= 0xFE;
	if ( speed < 2  ) {
		speed = 2 ;
	}
	return speed;
}

/*******************************************************************************
* Function Name  : SPI_FLASH_Init
* Description    : ��ʼ������SSI�Ĺܽ�
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void SPI_FLASH_Init(void)
{
	uint32_t speed = LPC17xx_SPI_GetSpeed(SPI_SPEED_12MHz);
	LOG_PRINT("speed=0x%x", speed);
	spicfg.cpsr = speed;
	sspspiStart(&SPID1, &spicfg);

#if 0
	volatile uint32_t dummy;
	dummy = dummy;                                   /* avoid warning */
	/* drain SPI RX FIFO */
	while( LPC_SSP0->SR & ( 1 << SSPSR_RNE ) )
	{
		dummy = LPC_SSP0->DR;
	}
#endif
}


/*****************************************************************************
* Function Name  : Flash_WriteByte
* Description    : ͨ��Ӳ��SPI����һ���ֽڵ�SST25VF016B
* Input          : - data: ���͵�����
* Output         : None
* Return         : SST25VF016B ���ص�����
* Attention		 : None
*******************************************************************************/
void Flash_WriteByte(uint8_t data)		
{
	sspspiSend(&SPID1,1, &data);    /* Polled method.       */
}

uint8_t Flash_ReadByte(void)		
{
	uint8_t r_buf;
	sspspiReceive(&SPID1, 1, &r_buf); 
	return r_buf;
}



/* ���º�������ֲʱ�����޸� */
/*******************************************************************************
* Function Name  : SSTF016B_RD
* Description    : SST25VF016B�Ķ�����,��ѡ���ID�Ͷ����ݲ���
* Input          : - Dst: Ŀ���ַ,��Χ 0x0 - MAX_ADDR��MAX_ADDR = 0x1FFFFF��
*                  - RcvBufPt: ���ջ����ָ��
*                  - NByte: Ҫ��ȡ�������ֽ���	
* Output         : �����ɹ��򷵻�OK,ʧ���򷵻�ERROR
* Return         : SST25VF016B ���ص�����
* Attention		 : ��ĳ������,ĳһ��ڲ�����Ч,���ڸ���ڲ�������Invalid���ò�����������
*******************************************************************************/
uint8_t SSTF016B_RD(uint32_t Dst, uint8_t* RcvBufPt ,uint32_t NByte)
{
	uint32_t i = 0;
	if ((Dst+NByte > MAX_ADDR)||(NByte == 0))	return (ERROR);	 /*	�����ڲ��� */
	
    SPI_FLASH_CS_LOW();
	Flash_WriteByte(0x0B); 						/* ���Ͷ����� */
	Flash_WriteByte(((Dst & 0xFFFFFF) >> 16));	/* ���͵�ַ��Ϣ:�õ�ַ��3���ֽ����	*/
	Flash_WriteByte(((Dst & 0xFFFF) >> 8));
	Flash_WriteByte(Dst & 0xFF);
	Flash_WriteByte(0xFF);						/* ����һ�����ֽ��Զ�ȡ����	*/
#if __SSP_SPI__
	sspspiReceive(&SPID1, NByte, RcvBufPt); 
#else
	for (i = 0; i < NByte; i++)		
	{
       RcvBufPt[i] = Flash_ReadByte();		
	}
#endif
    SPI_FLASH_CS_HIGH();
	return (ENABLE);
}

/*******************************************************************************
* Function Name  : SSTF016B_RdID
* Description    : SST25VF016B�Ķ�ID����,��ѡ���ID�Ͷ����ݲ���
* Input          : - IDType: ID���͡��û�����Jedec_ID,Dev_ID,Manu_ID������ѡ��
* Output         : - RcvbufPt: �洢ID������ָ��
* Return         : �����ɹ��򷵻�OK,ʧ���򷵻�ERROR
* Attention		 : ������Ĳ���������Ҫ���򷵻�ERROR
*******************************************************************************/
uint8_t SSTF016B_RdID(idtype IDType,uint32_t* RcvbufPt)
{
	uint32_t temp = 0;
              
	if (IDType == Jedec_ID)
	{
		SPI_FLASH_CS_LOW();	
				
		Flash_WriteByte(0x9F);		 		         /* ���Ͷ�JEDEC ID����(9Fh)	*/
		sspspiReceive(&SPID1, 3, RcvbufPt); 
        SPI_FLASH_CS_HIGH();

		return (ENABLE);
	}
	
	if ((IDType == Manu_ID) || (IDType == Dev_ID) )
	{
	    SPI_FLASH_CS_LOW();	
		
		Flash_WriteByte(0x90);				/* ���Ͷ�ID���� (90h or ABh) */
    	Flash_WriteByte(0x00);				/* ���͵�ַ	*/
		Flash_WriteByte(0x00);				/* ���͵�ַ	*/
		Flash_WriteByte(IDType);		    /* ���͵�ַ - ����00H����01H */
		temp = Flash_ReadByte();	    /* ���ջ�ȡ�������ֽ� */

        SPI_FLASH_CS_HIGH();

		*RcvbufPt = temp;
		return (ENABLE);
	}
	else
	{
		return (ERROR);	
	}
}


/*******************************************************************************
* Function Name  : SSTF016B_WR
* Description    : SST25VF016B��д��������д1���Ͷ�����ݵ�ָ����ַ
* Input          : - Dst: Ŀ���ַ,��Χ 0x0 - MAX_ADDR��MAX_ADDR = 0x1FFFFF��
*                  - SndbufPt: ���ͻ�����ָ��
*                  - NByte: Ҫд�������ֽ���
* Output         : None
* Return         : �����ɹ��򷵻�OK,ʧ���򷵻�ERROR
* Attention		 : ��ĳ������,ĳһ��ڲ�����Ч,���ڸ���ڲ�������Invalid���ò�����������
*******************************************************************************/
uint8_t SSTF016B_WR(uint32_t Dst, uint8_t* SndbufPt,uint32_t NByte)
{
	uint8_t temp = 0,StatRgVal = 0;
	uint32_t i = 0;
	if (( (Dst+NByte-1 > MAX_ADDR)||(NByte == 0) ))
	{
		return (ERROR);	 /*	�����ڲ��� */
	}

	
	SPI_FLASH_CS_LOW();				 
	Flash_WriteByte(0x05);							    /* ���Ͷ�״̬�Ĵ�������	*/
	temp = Flash_ReadByte();					    /* ������õ�״̬�Ĵ���ֵ */
	SPI_FLASH_CS_HIGH();								

	SPI_FLASH_CS_LOW();				
	Flash_WriteByte(0x50);							    /* ʹ״̬�Ĵ�����д	*/
	SPI_FLASH_CS_HIGH();	
		
	SPI_FLASH_CS_LOW();				
	Flash_WriteByte(0x01);							    /* ����д״̬�Ĵ���ָ�� */
	Flash_WriteByte(0);								    /* ��0BPxλ��ʹFlashоƬȫ����д */
	SPI_FLASH_CS_HIGH();			
	
	for(i = 0; i < NByte; i++)
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* ����дʹ������ */
		SPI_FLASH_CS_HIGH();			

		SPI_FLASH_CS_LOW();					
		Flash_WriteByte(0x02); 						    /* �����ֽ�������д����	*/
		Flash_WriteByte((((Dst+i) & 0xFFFFFF) >> 16));  /* ����3���ֽڵĵ�ַ��Ϣ */
		Flash_WriteByte((((Dst+i) & 0xFFFF) >> 8));
		Flash_WriteByte((Dst+i) & 0xFF);
		Flash_WriteByte(SndbufPt[i]);					/* ���ͱ���д������	*/
		SPI_FLASH_CS_HIGH();			

		do
		{
		  	SPI_FLASH_CS_LOW();					 
			Flash_WriteByte(0x05);					    /* ���Ͷ�״̬�Ĵ������� */
			StatRgVal = Flash_ReadByte();			/* ������õ�״̬�Ĵ���ֵ */
			SPI_FLASH_CS_HIGH();							
  		}
		while (StatRgVal == 0x03 );					          /* һֱ�ȴ���ֱ��оƬ����	*/

	}

	SPI_FLASH_CS_LOW();					
	Flash_WriteByte(0x06);							    /* ����дʹ������ */
	SPI_FLASH_CS_HIGH();			

	SPI_FLASH_CS_LOW();					
	Flash_WriteByte(0x50);							    /* ʹ״̬�Ĵ�����д	*/
	SPI_FLASH_CS_HIGH();
			
	SPI_FLASH_CS_LOW();				
	Flash_WriteByte(0x01);							    /* ����д״̬�Ĵ���ָ�� */
	Flash_WriteByte(temp);						     	/* �ָ�״̬�Ĵ���������Ϣ */
	SPI_FLASH_CS_HIGH();

	return (ENABLE);		
}


/*******************************************************************************
* Function Name  : SSTF016B_Erase
* Description    : ����ָ����������ѡȡ���Ч���㷨����
* Input          : - sec1: ��ʼ������,��Χ(0~511)
*                  - sec2: ��ֹ������,��Χ(0~511)
* Output         : None
* Return         : �����ɹ��򷵻�OK,ʧ���򷵻�ERROR
* Attention		 : None
*******************************************************************************/
uint8_t SSTF016B_Erase(uint32_t sec1, uint32_t sec2)
{
	uint8_t  temp1 = 0,temp2 = 0,StatRgVal = 0;
    uint32_t SecnHdAddr = 0;	  			
	uint32_t no_SecsToEr = 0;				   			    /* Ҫ������������Ŀ */
	uint32_t CurSecToEr = 0;	  						    /* ��ǰҪ������������ */
	
	/*  �����ڲ��� */
	if ((sec1 > SEC_MAX)||(sec2 > SEC_MAX))	
	{
		return (ERROR);	
	}
   	
   	SPI_FLASH_CS_LOW();			 
	Flash_WriteByte(0x05);								/* ���Ͷ�״̬�Ĵ�������	*/
	temp1 = Flash_ReadByte();						/* ������õ�״̬�Ĵ���ֵ */
	SPI_FLASH_CS_HIGH();								

	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x50);								/* ʹ״̬�Ĵ�����д	*/
	SPI_FLASH_CS_HIGH();			

	SPI_FLASH_CS_LOW();								  	
	Flash_WriteByte(0x01);								/* ����д״̬�Ĵ���ָ��	*/
	Flash_WriteByte(0);									/* ��0BPxλ��ʹFlashоƬȫ����д */
	SPI_FLASH_CS_HIGH();
	
	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x06);								/* ����дʹ������ */
	SPI_FLASH_CS_HIGH();			

	/* ����û��������ʼ�����Ŵ�����ֹ�����ţ������ڲ��������� */
	if (sec1 > sec2)
	{
	   temp2 = sec1;
	   sec1  = sec2;
	   sec2  = temp2;
	} 
	/* ����ֹ���������������������� */
	if (sec1 == sec2)	
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* ����дʹ������ */
		SPI_FLASH_CS_HIGH();			

	    SecnHdAddr = SEC_SIZE * sec1;				          /* ������������ʼ��ַ	*/
	    SPI_FLASH_CS_LOW();	
    	Flash_WriteByte(0x20);							  /* ������������ָ�� */
	    Flash_WriteByte(((SecnHdAddr & 0xFFFFFF) >> 16)); /* ����3���ֽڵĵ�ַ��Ϣ */
   		Flash_WriteByte(((SecnHdAddr & 0xFFFF) >> 8));
   		Flash_WriteByte(SecnHdAddr & 0xFF);
  		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* ���Ͷ�״̬�Ĵ������� */
			StatRgVal = Flash_ReadByte();			  /* ������õ�״̬�Ĵ���ֵ	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);				              /* һֱ�ȴ���ֱ��оƬ����	*/
		return (ENABLE);			
	}
	
    /* ������ʼ��������ֹ��������������ٵĲ������� */	
	
	if (sec2 - sec1 == SEC_MAX)	
	{
		SPI_FLASH_CS_LOW();			
		Flash_WriteByte(0x60);							  /* ����оƬ����ָ��(60h or C7h) */
		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* ���Ͷ�״̬�Ĵ������� */
			StatRgVal = Flash_ReadByte();			  /* ������õ�״̬�Ĵ���ֵ	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);					          /* һֱ�ȴ���ֱ��оƬ����	*/
   		return (ENABLE);
	}
	
	no_SecsToEr = sec2 - sec1 +1;					          /* ��ȡҪ������������Ŀ */
	CurSecToEr  = sec1;								          /* ����ʼ������ʼ����	*/
	
	/* ����������֮��ļ���������ȡ8���������㷨 */
	while (no_SecsToEr >= 8)
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* ����дʹ������ */
		SPI_FLASH_CS_HIGH();			

	    SecnHdAddr = SEC_SIZE * CurSecToEr;			          /* ������������ʼ��ַ */
	    SPI_FLASH_CS_LOW();	
	    Flash_WriteByte(0x52);							  /* ����32KB����ָ�� */
	    Flash_WriteByte(((SecnHdAddr & 0xFFFFFF) >> 16)); /* ����3���ֽڵĵ�ַ��Ϣ */
   		Flash_WriteByte(((SecnHdAddr & 0xFFFF) >> 8));
   		Flash_WriteByte(SecnHdAddr & 0xFF);
  		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* ���Ͷ�״̬�Ĵ������� */
			StatRgVal = Flash_ReadByte();			  /* ������õ�״̬�Ĵ���ֵ	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);					          /* һֱ�ȴ���ֱ��оƬ����	*/
		CurSecToEr  += 8;
		no_SecsToEr -=  8;
	}
	/* �������������㷨����ʣ������� */
	while (no_SecsToEr >= 1)
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* ����дʹ������ */
		SPI_FLASH_CS_HIGH();			

	    SecnHdAddr = SEC_SIZE * CurSecToEr;			          /* ������������ʼ��ַ */
	    SPI_FLASH_CS_LOW();	
    	Flash_WriteByte(0x20);							  /* ������������ָ�� */
	    Flash_WriteByte(((SecnHdAddr & 0xFFFFFF) >> 16)); /* ����3���ֽڵĵ�ַ��Ϣ */
   		Flash_WriteByte(((SecnHdAddr & 0xFFFF) >> 8));
   		Flash_WriteByte(SecnHdAddr & 0xFF);
  		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* ���Ͷ�״̬�Ĵ������� */
			StatRgVal = Flash_ReadByte();			  /* ������õ�״̬�Ĵ���ֵ	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);					          /* һֱ�ȴ���ֱ��оƬ���� */
		CurSecToEr  += 1;
		no_SecsToEr -=  1;
	}
    /* ��������,�ָ�״̬�Ĵ�����Ϣ */
	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x06);								  /* ����дʹ������ */
	SPI_FLASH_CS_HIGH();			

	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x50);								  /* ʹ״̬�Ĵ�����д */
	SPI_FLASH_CS_HIGH();			
	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x01);								  /* ����д״̬�Ĵ���ָ�� */
	Flash_WriteByte(temp1);								  /* �ָ�״̬�Ĵ���������Ϣ */
	SPI_FLASH_CS_HIGH();    
	return (ENABLE);
}


/*******************************************************************************
* Function Name  : SPI_FLASH_Test
* Description    : ��ȡSST25VF016B ID 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void SPI_FLASH_Test(void)
{
    uint32_t  ChipID = 0;
  	SSTF016B_RdID(Jedec_ID, &ChipID);                                   /*  �������е��˴�ʱ, ��Watch��
	                                                                        �鿴ChipID��ֵ�Ƿ�0xBF2541  */
    ChipID &= ~0xff000000;						                        /*  ��������24λ����            */
	if (ChipID != 0xBF2541) {											/*  ID����ȷ������ѭ��          */
           while(1) {
		   }
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


/****************************************Copyright (c)**************************************************                         
**
**                                 http://www.powermcu.com
**
**--------------File Info-------------------------------------------------------------------------------
** File name:			SST25VF016B.c
** Descriptions:		SST25VF016B操作函数库 
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
* Description    : 初始化控制SSI的管脚
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
* Description    : 通过硬件SPI发送一个字节到SST25VF016B
* Input          : - data: 发送的数据
* Output         : None
* Return         : SST25VF016B 返回的数据
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



/* 以下函数在移植时无需修改 */
/*******************************************************************************
* Function Name  : SSTF016B_RD
* Description    : SST25VF016B的读函数,可选择读ID和读数据操作
* Input          : - Dst: 目标地址,范围 0x0 - MAX_ADDR（MAX_ADDR = 0x1FFFFF）
*                  - RcvBufPt: 接收缓存的指针
*                  - NByte: 要读取的数据字节数	
* Output         : 操作成功则返回OK,失败则返回ERROR
* Return         : SST25VF016B 返回的数据
* Attention		 : 若某功能下,某一入口参数无效,可在该入口参数处填Invalid，该参数将被忽略
*******************************************************************************/
uint8_t SSTF016B_RD(uint32_t Dst, uint8_t* RcvBufPt ,uint32_t NByte)
{
	uint32_t i = 0;
	if ((Dst+NByte > MAX_ADDR)||(NByte == 0))	return (ERROR);	 /*	检查入口参数 */
	
    SPI_FLASH_CS_LOW();
	Flash_WriteByte(0x0B); 						/* 发送读命令 */
	Flash_WriteByte(((Dst & 0xFFFFFF) >> 16));	/* 发送地址信息:该地址由3个字节组成	*/
	Flash_WriteByte(((Dst & 0xFFFF) >> 8));
	Flash_WriteByte(Dst & 0xFF);
	Flash_WriteByte(0xFF);						/* 发送一个哑字节以读取数据	*/
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
* Description    : SST25VF016B的读ID函数,可选择读ID和读数据操作
* Input          : - IDType: ID类型。用户可在Jedec_ID,Dev_ID,Manu_ID三者里选择
* Output         : - RcvbufPt: 存储ID变量的指针
* Return         : 操作成功则返回OK,失败则返回ERROR
* Attention		 : 若填入的参数不符合要求，则返回ERROR
*******************************************************************************/
uint8_t SSTF016B_RdID(idtype IDType,uint32_t* RcvbufPt)
{
	uint32_t temp = 0;
              
	if (IDType == Jedec_ID)
	{
		SPI_FLASH_CS_LOW();	
				
		Flash_WriteByte(0x9F);		 		         /* 发送读JEDEC ID命令(9Fh)	*/
		sspspiReceive(&SPID1, 3, RcvbufPt); 
        SPI_FLASH_CS_HIGH();

		return (ENABLE);
	}
	
	if ((IDType == Manu_ID) || (IDType == Dev_ID) )
	{
	    SPI_FLASH_CS_LOW();	
		
		Flash_WriteByte(0x90);				/* 发送读ID命令 (90h or ABh) */
    	Flash_WriteByte(0x00);				/* 发送地址	*/
		Flash_WriteByte(0x00);				/* 发送地址	*/
		Flash_WriteByte(IDType);		    /* 发送地址 - 不是00H就是01H */
		temp = Flash_ReadByte();	    /* 接收获取的数据字节 */

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
* Description    : SST25VF016B的写函数，可写1个和多个数据到指定地址
* Input          : - Dst: 目标地址,范围 0x0 - MAX_ADDR（MAX_ADDR = 0x1FFFFF）
*                  - SndbufPt: 发送缓存区指针
*                  - NByte: 要写的数据字节数
* Output         : None
* Return         : 操作成功则返回OK,失败则返回ERROR
* Attention		 : 若某功能下,某一入口参数无效,可在该入口参数处填Invalid，该参数将被忽略
*******************************************************************************/
uint8_t SSTF016B_WR(uint32_t Dst, uint8_t* SndbufPt,uint32_t NByte)
{
	uint8_t temp = 0,StatRgVal = 0;
	uint32_t i = 0;
	if (( (Dst+NByte-1 > MAX_ADDR)||(NByte == 0) ))
	{
		return (ERROR);	 /*	检查入口参数 */
	}

	
	SPI_FLASH_CS_LOW();				 
	Flash_WriteByte(0x05);							    /* 发送读状态寄存器命令	*/
	temp = Flash_ReadByte();					    /* 保存读得的状态寄存器值 */
	SPI_FLASH_CS_HIGH();								

	SPI_FLASH_CS_LOW();				
	Flash_WriteByte(0x50);							    /* 使状态寄存器可写	*/
	SPI_FLASH_CS_HIGH();	
		
	SPI_FLASH_CS_LOW();				
	Flash_WriteByte(0x01);							    /* 发送写状态寄存器指令 */
	Flash_WriteByte(0);								    /* 清0BPx位，使Flash芯片全区可写 */
	SPI_FLASH_CS_HIGH();			
	
	for(i = 0; i < NByte; i++)
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* 发送写使能命令 */
		SPI_FLASH_CS_HIGH();			

		SPI_FLASH_CS_LOW();					
		Flash_WriteByte(0x02); 						    /* 发送字节数据烧写命令	*/
		Flash_WriteByte((((Dst+i) & 0xFFFFFF) >> 16));  /* 发送3个字节的地址信息 */
		Flash_WriteByte((((Dst+i) & 0xFFFF) >> 8));
		Flash_WriteByte((Dst+i) & 0xFF);
		Flash_WriteByte(SndbufPt[i]);					/* 发送被烧写的数据	*/
		SPI_FLASH_CS_HIGH();			

		do
		{
		  	SPI_FLASH_CS_LOW();					 
			Flash_WriteByte(0x05);					    /* 发送读状态寄存器命令 */
			StatRgVal = Flash_ReadByte();			/* 保存读得的状态寄存器值 */
			SPI_FLASH_CS_HIGH();							
  		}
		while (StatRgVal == 0x03 );					          /* 一直等待，直到芯片空闲	*/

	}

	SPI_FLASH_CS_LOW();					
	Flash_WriteByte(0x06);							    /* 发送写使能命令 */
	SPI_FLASH_CS_HIGH();			

	SPI_FLASH_CS_LOW();					
	Flash_WriteByte(0x50);							    /* 使状态寄存器可写	*/
	SPI_FLASH_CS_HIGH();
			
	SPI_FLASH_CS_LOW();				
	Flash_WriteByte(0x01);							    /* 发送写状态寄存器指令 */
	Flash_WriteByte(temp);						     	/* 恢复状态寄存器设置信息 */
	SPI_FLASH_CS_HIGH();

	return (ENABLE);		
}


/*******************************************************************************
* Function Name  : SSTF016B_Erase
* Description    : 根据指定的扇区号选取最高效的算法擦除
* Input          : - sec1: 起始扇区号,范围(0~511)
*                  - sec2: 终止扇区号,范围(0~511)
* Output         : None
* Return         : 操作成功则返回OK,失败则返回ERROR
* Attention		 : None
*******************************************************************************/
uint8_t SSTF016B_Erase(uint32_t sec1, uint32_t sec2)
{
	uint8_t  temp1 = 0,temp2 = 0,StatRgVal = 0;
    uint32_t SecnHdAddr = 0;	  			
	uint32_t no_SecsToEr = 0;				   			    /* 要擦除的扇区数目 */
	uint32_t CurSecToEr = 0;	  						    /* 当前要擦除的扇区号 */
	
	/*  检查入口参数 */
	if ((sec1 > SEC_MAX)||(sec2 > SEC_MAX))	
	{
		return (ERROR);	
	}
   	
   	SPI_FLASH_CS_LOW();			 
	Flash_WriteByte(0x05);								/* 发送读状态寄存器命令	*/
	temp1 = Flash_ReadByte();						/* 保存读得的状态寄存器值 */
	SPI_FLASH_CS_HIGH();								

	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x50);								/* 使状态寄存器可写	*/
	SPI_FLASH_CS_HIGH();			

	SPI_FLASH_CS_LOW();								  	
	Flash_WriteByte(0x01);								/* 发送写状态寄存器指令	*/
	Flash_WriteByte(0);									/* 清0BPx位，使Flash芯片全区可写 */
	SPI_FLASH_CS_HIGH();
	
	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x06);								/* 发送写使能命令 */
	SPI_FLASH_CS_HIGH();			

	/* 如果用户输入的起始扇区号大于终止扇区号，则在内部作出调整 */
	if (sec1 > sec2)
	{
	   temp2 = sec1;
	   sec1  = sec2;
	   sec2  = temp2;
	} 
	/* 若起止扇区号相等则擦除单个扇区 */
	if (sec1 == sec2)	
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* 发送写使能命令 */
		SPI_FLASH_CS_HIGH();			

	    SecnHdAddr = SEC_SIZE * sec1;				          /* 计算扇区的起始地址	*/
	    SPI_FLASH_CS_LOW();	
    	Flash_WriteByte(0x20);							  /* 发送扇区擦除指令 */
	    Flash_WriteByte(((SecnHdAddr & 0xFFFFFF) >> 16)); /* 发送3个字节的地址信息 */
   		Flash_WriteByte(((SecnHdAddr & 0xFFFF) >> 8));
   		Flash_WriteByte(SecnHdAddr & 0xFF);
  		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* 发送读状态寄存器命令 */
			StatRgVal = Flash_ReadByte();			  /* 保存读得的状态寄存器值	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);				              /* 一直等待，直到芯片空闲	*/
		return (ENABLE);			
	}
	
    /* 根据起始扇区和终止扇区间距调用最快速的擦除功能 */	
	
	if (sec2 - sec1 == SEC_MAX)	
	{
		SPI_FLASH_CS_LOW();			
		Flash_WriteByte(0x60);							  /* 发送芯片擦除指令(60h or C7h) */
		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* 发送读状态寄存器命令 */
			StatRgVal = Flash_ReadByte();			  /* 保存读得的状态寄存器值	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);					          /* 一直等待，直到芯片空闲	*/
   		return (ENABLE);
	}
	
	no_SecsToEr = sec2 - sec1 +1;					          /* 获取要擦除的扇区数目 */
	CurSecToEr  = sec1;								          /* 从起始扇区开始擦除	*/
	
	/* 若两个扇区之间的间隔够大，则采取8扇区擦除算法 */
	while (no_SecsToEr >= 8)
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* 发送写使能命令 */
		SPI_FLASH_CS_HIGH();			

	    SecnHdAddr = SEC_SIZE * CurSecToEr;			          /* 计算扇区的起始地址 */
	    SPI_FLASH_CS_LOW();	
	    Flash_WriteByte(0x52);							  /* 发送32KB擦除指令 */
	    Flash_WriteByte(((SecnHdAddr & 0xFFFFFF) >> 16)); /* 发送3个字节的地址信息 */
   		Flash_WriteByte(((SecnHdAddr & 0xFFFF) >> 8));
   		Flash_WriteByte(SecnHdAddr & 0xFF);
  		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* 发送读状态寄存器命令 */
			StatRgVal = Flash_ReadByte();			  /* 保存读得的状态寄存器值	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);					          /* 一直等待，直到芯片空闲	*/
		CurSecToEr  += 8;
		no_SecsToEr -=  8;
	}
	/* 采用扇区擦除算法擦除剩余的扇区 */
	while (no_SecsToEr >= 1)
	{
		SPI_FLASH_CS_LOW();				
		Flash_WriteByte(0x06);						    /* 发送写使能命令 */
		SPI_FLASH_CS_HIGH();			

	    SecnHdAddr = SEC_SIZE * CurSecToEr;			          /* 计算扇区的起始地址 */
	    SPI_FLASH_CS_LOW();	
    	Flash_WriteByte(0x20);							  /* 发送扇区擦除指令 */
	    Flash_WriteByte(((SecnHdAddr & 0xFFFFFF) >> 16)); /* 发送3个字节的地址信息 */
   		Flash_WriteByte(((SecnHdAddr & 0xFFFF) >> 8));
   		Flash_WriteByte(SecnHdAddr & 0xFF);
  		SPI_FLASH_CS_HIGH();			
		do
		{
		  	SPI_FLASH_CS_LOW();			 
			Flash_WriteByte(0x05);						  /* 发送读状态寄存器命令 */
			StatRgVal = Flash_ReadByte();			  /* 保存读得的状态寄存器值	*/
			SPI_FLASH_CS_HIGH();								
  		}
		while (StatRgVal == 0x03);					          /* 一直等待，直到芯片空闲 */
		CurSecToEr  += 1;
		no_SecsToEr -=  1;
	}
    /* 擦除结束,恢复状态寄存器信息 */
	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x06);								  /* 发送写使能命令 */
	SPI_FLASH_CS_HIGH();			

	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x50);								  /* 使状态寄存器可写 */
	SPI_FLASH_CS_HIGH();			
	SPI_FLASH_CS_LOW();			
	Flash_WriteByte(0x01);								  /* 发送写状态寄存器指令 */
	Flash_WriteByte(temp1);								  /* 恢复状态寄存器设置信息 */
	SPI_FLASH_CS_HIGH();    
	return (ENABLE);
}


/*******************************************************************************
* Function Name  : SPI_FLASH_Test
* Description    : 读取SST25VF016B ID 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void SPI_FLASH_Test(void)
{
    uint32_t  ChipID = 0;
  	SSTF016B_RdID(Jedec_ID, &ChipID);                                   /*  单步运行到此处时, 在Watch里
	                                                                        查看ChipID的值是否0xBF2541  */
    ChipID &= ~0xff000000;						                        /*  仅保留低24位数据            */
	if (ChipID != 0xBF2541) {											/*  ID不正确进入死循环          */
           while(1) {
		   }
    }
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


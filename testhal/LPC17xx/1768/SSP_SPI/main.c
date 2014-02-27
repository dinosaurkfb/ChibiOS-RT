
/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include <string.h>
#include "lpc_types.h"
#include "SST25VF016B.h"
#include "FlashDriver.h"

/* Private variables ---------------------------------------------------------*/
uint8_t  ReadBuffer[256];
uint8_t  WriteBuffer[256];

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
int main(void)
{
    uint16_t  i;
    uint32_t  ChipID = 0;	

  halInit();
  chSysInit();

  print_info();
	
	SPI_FLASH_Init();
	SPI_FLASH_Test();


	df_write_open(0);
	df_read_open(0);
	SSTF016B_Erase(0, 0);
	
	for( i = 0; i < 256; i++ )
	{
	    WriteBuffer[i] = i;
	}

	df_write( WriteBuffer, sizeof(WriteBuffer) );

	df_read( ReadBuffer, sizeof(ReadBuffer) );

	/* Matching data */
	if( memcmp( WriteBuffer, ReadBuffer, sizeof(WriteBuffer) ) == 0 )  
	{
		LOG_PRINT("HY-LPC1788-SDK SPI Flash SST25VF016B OK");
	}
	else
	{
		LOG_PRINT("HY-LPC1788-SDK SPI Flash SST25VF016B False");
	} 
	while (TRUE) { 
		chThdSleepMilliseconds(1000); 
		LOG_PRINT("*** Alive seconds.\n" ); 
	}
}

void print_info() {
	/*
   * Test procedure.
   */
  LOG_PRINT("\n*** ChibiOS/RT IRQ-STORM long duration test\n");
  LOG_PRINT("***\n");
  LOG_PRINT("*** Kernel:       %u\n", CH_KERNEL_VERSION);
#ifdef CH_COMPILER_NAME
  LOG_PRINT("*** Compiler:     %u\n", CH_COMPILER_NAME);
#endif
  LOG_PRINT("*** Architecture: %u\n", CH_ARCHITECTURE_NAME);
#ifdef CH_CORE_VARIANT_NAME
  LOG_PRINT("*** Core Variant: %u\n", CH_CORE_VARIANT_NAME);
#endif
#ifdef CH_PORT_INFO
  LOG_PRINT("*** Port Info:    %u\n", CH_PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  LOG_PRINT("*** Platform:     %u\n", PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  LOG_PRINT("*** Test Board:   %u\n", BOARD_NAME);
#endif
  LOG_PRINT("***\n");
  LOG_PRINT("*** System Clock: %u\n", LPC17xx_CCLK);
}


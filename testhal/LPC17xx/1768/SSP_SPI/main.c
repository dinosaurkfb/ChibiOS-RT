
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
#include "update.h"

#include <string.h>
#include "lpc_types.h"
#include "SST25VF016B.h"
#include "FlashDriver.h"

uint8_t  ReadBuffer[256];
uint8_t  WriteBuffer[256];


int main(void)
{ 
	uint16_t  i; 
	halInit(); 
	chSysInit();

	LOG_PRINT("SSP SPI Test Running.\n");
    updateThreadStart();

	
	SPI_FLASH_Init();

	uint32_t  ChipID = 0;
  	SSTF016B_RdID(Jedec_ID, &ChipID);                                     
	                                                                     
    ChipID &= ~0xff000000;						                        
	if (ChipID != 0x4125BF) {										
		LOG_PRINT(" Err: SSTF016B ID =0x%x", ChipID);
       while(1) {
		chThdSleepMilliseconds(1000); 
		}
    }else{
		LOG_PRINT("Read id OK: SSTF016B ID =0x%x.\n", ChipID);
	}


	SSTF016B_Erase(0, 1);
	
	for( i = 0; i < 256; i++ )
	{
	    WriteBuffer[i] = i;
	}

	df_write_open(0);
	df_write( WriteBuffer, sizeof(WriteBuffer) );
	df_write_close(); 

	LOG_PRINT("HY-LPC1788-SDK SPI Flash Write Date:.\n");
	memdump(WriteBuffer, sizeof(WriteBuffer));


	df_read_open(0);
	df_read( ReadBuffer, sizeof(ReadBuffer) );

	LOG_PRINT("HY-LPC1788-SDK SPI Flash Read Date:.\n");
	memdump(WriteBuffer, sizeof(WriteBuffer));

	/* Matching data */
	if( memcmp(WriteBuffer, ReadBuffer, sizeof(WriteBuffer) ) == 0 )  
	{
		LOG_PRINT("HY-LPC1788-SDK SPI Flash SST25VF016B OK.\n");
	}
	else
	{
		LOG_PRINT("HY-LPC1788-SDK SPI Flash SST25VF016B False.\n");
		for(i=0; i<256; i++ ) {
			if(ReadBuffer[i] != WriteBuffer[i]) {
				LOG_PRINT("%03d %03d %03d.\n", i, ReadBuffer[i], WriteBuffer[i]);
			}
		}
	} 

	while (TRUE) { 
		chThdSleepMilliseconds(1000); 
		LOG_PRINT("*** Alive seconds.\n" ); 
	}
}



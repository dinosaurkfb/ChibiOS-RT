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
#include "test_hal.h"
#include "lpc_types.h"

#include "string.h"
#include "devices_lib.h"
#include "FlashDriver.h"


/* buffers depth */
#define SPI_TEST_SIZE 			256

static uint8_t r_buf[SPI_TEST_SIZE];
static uint8_t w_buf[SPI_TEST_SIZE];

/* SPI interface config */
static SSPSPIConfig spicfg = {
  NULL,
  LPC_GPIO0,
  16,
  7,
  1000000  //speed
};


/* read flash id test */
static void read_flash_id_setup(void) {
	SPI_FLASH_Init(&SPID1, &spicfg);
	LOG_PRINT("\tspi start.\n");
}

static void read_flash_id_exe(void) {
	uint32_t  ChipID = 0;
  	SSTF016B_RdID(Jedec_ID, &ChipID);                                     
	                                                                     
    ChipID &= ~0xff000000;						                        
	LOG_PRINT("\tSSTF016B ID =0x%x.\n", ChipID);
    test_assert(1, 
			ChipID == 0x4125BF, 
			"\tErr: Read SSTF016B ID");
}

ROMCONST testcase_t read_flash_id = {
  "read flash id",
  read_flash_id_setup,
  NULL,
  read_flash_id_exe
};

/* flash_write_byte test */
static void write_flash_setup(void) {
  uint32_t i = 0;
  SSTF016B_Erase(0, 1);
  for (i = 0; i < SPI_TEST_SIZE; i++) {
    w_buf[i] = i;
  }
}

static void write_flash_exe(void) {
	/* write data begin 0 */
	df_write_open(0); 
	df_write(w_buf, SPI_TEST_SIZE);
	df_write_close(); 
}

ROMCONST testcase_t write_flash = {
  "write flash",
  write_flash_setup,
  NULL,
  write_flash_exe
};


/* flash_read_byte test */
static void read_flash_setup(void) {
  uint32_t i = 0;
  for (i = 0; i < SPI_TEST_SIZE; i++) {
    r_buf[i] = 0;
  }
}

static void read_flash_exe(void) {
	/* read data begin 0 */
	df_read_open(0); 
	df_read(r_buf, SPI_TEST_SIZE);
	df_read_close(); 
}

ROMCONST testcase_t read_flash = {
  "read flash",
  read_flash_setup,
  NULL,
  read_flash_exe
};



/* test consistency between read data and written data  */
static void write_read_cmp_setup(void) {
}

static void write_read_cmp_exe(void) {
	int i;
	/* Matching data */
	int ret = memcmp(w_buf, r_buf, SPI_TEST_SIZE);
	if( ret == 0 )  {
		LOG_PRINT("\tLPC1768 SPI Flash SST25VF016B Test OK.\n");
	} else {
		LOG_PRINT("\tERR: LPC1768 SPI Flash SST25VF016B False.\n");
		for(i=0; i<SPI_TEST_SIZE; i++ ) {
			if(r_buf[i] != w_buf[i]) {
				LOG_PRINT("\t%03d: %03d != %03d.\n", i, r_buf[i], w_buf[i]);
			}
		}
	} 
    test_assert(1, ret == 0, "Data not matched.");
}

ROMCONST testcase_t write_read_flash_cmp = {
  "write read compare",
  write_read_cmp_setup,
  NULL,
  write_read_cmp_exe
};


/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_spi[] = {
  &read_flash_id,
  &write_flash,
  &read_flash,
  &write_read_flash_cmp,
  NULL
};

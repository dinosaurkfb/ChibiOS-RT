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

#include "string.h"
#include "iCAN.h"


#define MAX_WRITE_TIMES  		16
#define ICAN_DATA_STAET 		256
#define MAX_ICAN_DATA_LEN 		(ICAN_DATA_STAET + MAX_WRITE_TIMES)
#define ICAN_TEST_LEN 			448

uint8_t  r_buf[ICAN_TEST_LEN];
uint8_t  w_buf[ICAN_TEST_LEN];

iCANMiddleware iCan1;
iCANMiddleware iCan2;

static const CANConfig cancfg = {
  0x500f9 //40kb
};



static void ican_test_setup(void) {
	int i;

	/* Initialization can1 and ican1 */
	canStart(&CAND1, &cancfg);
	iCanInit(&iCan1, &CAND1, 0x01);

	/* Initialization can2 and ican2 */
	canStart(&CAND2, &cancfg);
	iCanInit(&iCan2, &CAND2, 0x02);

	for(i=0; i<ICAN_TEST_LEN; i++) {
		w_buf[i] = i;
	} 
}


/* ican1 test */
static void ican_test_exe(void) {
	int j;
	iCANID s_id;
	iCANID ican_id;
	s_id.ucSrcAddr = 0x01;
    s_id.ucDestAddr = 0x02;
    s_id.ucAck =  V_NO_ACK;
    s_id.ucFuncId = FUNC_WRITE;
    s_id.ucResourceId = 0xa5;

	for (j = ICAN_DATA_STAET; j < MAX_ICAN_DATA_LEN; j++) { 
		chThdSleepMilliseconds(500);
		/* Tell sender to send 'j' bytes to me. */ 
		LOG_PRINT("\tTimes:%d.\n",j-ICAN_DATA_STAET+1);
		LOG_PRINT("\tiCan wirte bytes:%d.\n",j);

		/* ican1 send data to ican2 */ 
		test_assert(j, j==iCanWriteBus(&iCan1, &s_id, w_buf, j),  "\tiCAN write fail");

		while(1) {
		  int ret = iCanReadBus(&iCan2, &ican_id, r_buf, ICAN_TEST_LEN);
		  if(ret > 0) { 
			  test_assert(j, 
					  j==ret,
					  "\tReturn bytes doesn't match.\n");

			  test_assert(j, 
					  !memcmp(w_buf, r_buf, j), 
					  "\tData not matched.");
		   } 
		   LOG_PRINT("\tiCan read bytes :%d, And data matched.\n\n",j); 
		   break;
		 }
	}
	
  
}

static void ican_test_td(void) {
	iCanClose(&iCan1);
	iCanClose(&iCan2);
}

ROMCONST testcase_t ican_test = {
  "ican_test",
  ican_test_setup,
  ican_test_td,
  ican_test_exe
};



/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_ican[] = {
  &ican_test,
  NULL
};


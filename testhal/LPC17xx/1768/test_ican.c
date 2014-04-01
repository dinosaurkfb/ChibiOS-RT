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


#define ICAN_TEST_LEN  		128

uint8_t  r1_buf[ICAN_TEST_LEN];
uint8_t  r2_buf[ICAN_TEST_LEN];
uint8_t  w_buf[ICAN_TEST_LEN];

iCANMiddleware iCan1;
iCANMiddleware iCan2;

static const CANConfig cancfg = {
  0x500f9 //40kb
};



static WORKING_AREA(ican1_wa, 512);
static WORKING_AREA(ican2_wa, 512);

/*
 * If ican1 read data, Then send the data to ican2
 */
static msg_t ican1_read(void * p) {
	(void)p;
	iCANID ican_id; 
	LOG_PRINT("ICAN1 Test read.\n");
	while(!chThdShouldTerminate()) {
	  int ret = iCanReadBus(&iCan1, &ican_id, r1_buf, ICAN_TEST_LEN);
	  if(ret > 0) {
		  iCanPrintPacket(&ican_id, r1_buf, ret);
		  chThdSleepMilliseconds(500);
		  ican_id.ucSrcAddr = 0x01;
		  ican_id.ucDestAddr = 0x02;
		  iCanWriteBus(&iCan1, &ican_id, r1_buf, ret);
	  }
	} 
	return 0;
}

/*
 * If ican2 read data, Then send the data to ican1
 */
static msg_t ican2_read(void * p) {
	(void)p;
	iCANID ican_id; 
	LOG_PRINT("ICAN2 Test read.\n");
	while(!chThdShouldTerminate()) {
	  int ret = iCanReadBus(&iCan2, &ican_id, r2_buf, ICAN_TEST_LEN);
	  if(ret > 0) {
		  iCanPrintPacket(&ican_id, r2_buf, ret);
		  chThdSleepMilliseconds(500);
		  ican_id.ucSrcAddr = 0x02;
		  ican_id.ucDestAddr = 0x01;
		  iCanWriteBus(&iCan2, &ican_id, r2_buf, ret);
	  }
	} 
	return 0;
}


/*
 * Initialization can1 and ican1
 */
static void ican1_setup(void) {
	int i;
	canStart(&CAND1, &cancfg);
	iCanInit(&iCan1, &CAND1, 0x01);
	for(i=0; i<ICAN_TEST_LEN; i++) {
		w_buf[i] = i;
	}
}


/* ican1 test */
static void ican1_exe(void) {
	iCANID s_id;
	s_id.ucSrcAddr = 0x01;
    s_id.ucDestAddr = 0x02;
    s_id.ucAck =  V_NO_ACK;
    s_id.ucFuncId = FUNC_WRITE;
    s_id.ucResourceId = 0xa5;

	LOG_PRINT("ICAN Test write.\n");
	/* ican1 send data to ican2 */
	iCanWriteBus(&iCan1, &s_id, w_buf, ICAN_TEST_LEN);

	chThdCreateStatic(ican1_wa, sizeof(ican1_wa), NORMALPRIO + 7, ican1_read, NULL);
}

ROMCONST testcase_t ican1_test = {
  "ican1_test",
  ican1_setup,
  NULL,
  ican1_exe
};


/*
 * Initialization can2 and ican2
 */
static void ican2_setup(void) {
	canStart(&CAND2, &cancfg);
	iCanInit(&iCan2, &CAND2, 0x02);
}


/* ican2 test */
static void ican2_exe(void) {
	chThdCreateStatic(ican2_wa, sizeof(ican2_wa), NORMALPRIO + 7, ican2_read, NULL);
}

ROMCONST testcase_t ican2_test = {
  "ican2_test",
  ican2_setup,
  NULL,
  ican2_exe
};



/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_ican[] = {
  &ican1_test,
  &ican2_test,
  NULL
};


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


#define CAN_TEST_LEN  		128



static const CANConfig cancfg = {
  0x500f9 //40kb
};



static WORKING_AREA(can1_wa, 512);
static WORKING_AREA(can2_wa, 512);

/* Print can frame */
void printMsg(CANRxFrame *msg) {
	int i;
	LOG_PRINT("\n\nID:0x%x", msg->EID);
	LOG_PRINT(" BUF:");
	for(i=0; i<msg->DLC; i++){
		LOG_PRINT("%02x ",msg->data8[i]);
	}
}

/*
 * If can1 read data, Then send the data to can2
 */
static msg_t can1_read(void * p) {
	CANRxFrame rxmsg;
	msg_t ret;

	(void)p;
	chRegSetThreadName("receiver1");
	while(!chThdShouldTerminate()) {
	  while(1) {
		ret = canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, 100);
		  /* Process message.*/
		if(ret == RDY_OK) {
			printMsg(&rxmsg);
			chThdSleepMilliseconds(500);
			rxmsg.EID = 0x01234567;
			canTransmit(&CAND1, CAN_ANY_MAILBOX, (const CANTxFrame *)&rxmsg, MS2ST(100));
			continue;
		}
		break;
	  }
	chThdSleepMilliseconds(20);
	}
	return 0;
}

/*
 * If can2 read data, Then send the data to can1
 */
static msg_t can2_read(void * p) {
	CANRxFrame rxmsg;
	msg_t ret;

	(void)p;
	chRegSetThreadName("receiver1");
	while(!chThdShouldTerminate()) {
	  while(1) {
		ret = canReceive(&CAND2, CAN_ANY_MAILBOX, &rxmsg, 100);
		  /* Process message.*/
		if(ret == RDY_OK) {
			printMsg(&rxmsg);
			chThdSleepMilliseconds(500);
			rxmsg.EID = 0x00043210;
			canTransmit(&CAND2, CAN_ANY_MAILBOX, (const CANTxFrame *)&rxmsg, MS2ST(100));
			continue;
		}
		break;
	  }
	chThdSleepMilliseconds(20);
	}
	return 0;
}


/*
 * Initialization can1 and can1
 */
static void can1_setup(void) {
	canStart(&CAND1, &cancfg);
}


/* can1 test */
static void can1_exe(void) {
	CANTxFrame txmsg;

	chRegSetThreadName("transmitter");
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.EID = 0x01234567;
	txmsg.DLC = 8;
	txmsg.data32[0] = 0xaaAAaaAA;
	txmsg.data32[1] = 0xaaFFa0FF;

	canTransmit(&CAND2, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
	chThdCreateStatic(can1_wa, sizeof(can1_wa), NORMALPRIO + 7, can1_read, NULL);
}

ROMCONST testcase_t can1_test = {
  "can1_test",
  can1_setup,
  NULL,
  can1_exe
};


/*
 * Initialization can2 and can2
 */
static void can2_setup(void) {
	canStart(&CAND2, &cancfg);
}


/* can2 test */
static void can2_exe(void) {
	chThdCreateStatic(can2_wa, sizeof(can2_wa), NORMALPRIO + 7, can2_read, NULL);
}

ROMCONST testcase_t can2_test = {
  "can2_test",
  can2_setup,
  NULL,
  can2_exe
};



/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_can[] = {
  &can1_test,
  &can2_test,
  NULL
};


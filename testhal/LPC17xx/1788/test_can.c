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

#include "update.h"
/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
#define MAX_WRITE_TIMES 8

#define BUF_SIZE  256

static Thread *s_senderp;

/** @brief Driver configuration.*/
static const CANConfig cancfg = {
  0x500f9 //40kb
};


CANTxFrame txmsg;
CANRxFrame s_rxmsg;

static WORKING_AREA(waCan2RxThread, 128);
static msg_t Can2Sender(void *arg) {
  (void)arg;
  chRegSetThreadName("Can2Sender");

	chRegSetThreadName("transmitter");
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.EID = 0x01234567;

  uint32_t i = 0;
  Thread * serial_receiver = NULL;
  uint32_t sbytes = 0;
  while (TRUE) {
    /* Wait msg from serial_receiver */
    serial_receiver = chMsgWait();
    /* Read the msg which stand for bytes the receiver expect to receive. */
    sbytes = chMsgGet(serial_receiver);
	txmsg.DLC = sbytes;
    /* Call chMsgRelease to release receiver so it can continue to receive */
    chMsgRelease(serial_receiver, 0);
    for (i = 0; i < sbytes; i++) {
	  txmsg.data8[i] = i;
    }
	LOG_PRINT("\tTimes:%d.\n",sbytes);
	LOG_PRINT("\tCan wirte bytes:%d.\n",sbytes);
	canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_INFINITE);

    ToggleLED(6);
  }

  return RDY_OK;
}


static void can1_to_can2_setup(void) {
  canStart(&CAND1, &cancfg);
  canStart(&CAND2, &cancfg);
  s_senderp = chThdCreateStatic(waCan2RxThread, sizeof waCan2RxThread,
                                NORMALPRIO - 20, Can2Sender, NULL);
}

static void can1_to_can2_td(void) {
}

/* SERIAL3_T1 serialPolledDelay test */
static void can1_to_can2_exe(void) {
  /*
   * Tell sender thread to send some bytes and then read those bytes and
   * examine the count of bytes and compare read and written data.
   */
  uint32_t j = 0;

  LOG_PRINT("\n\tConnect CAN1 and CAN2 together to pass the test or not"
            " to fail the test.\n\n");
  chThdSleepSeconds(1);
  for (j = 1; j <= MAX_WRITE_TIMES; j++) {
    /* Tell sender to send 'j' bytes to me. */
    chMsgSend(s_senderp, (msg_t)j);
    test_assert(j,
                RDY_OK == canReceive(&CAND2, CAN_ANY_MAILBOX, &s_rxmsg, TIME_INFINITE),
                "Return bytes doesn't match");
    test_assert(j,
                !memcmp(s_rxmsg.data8, txmsg.data8, j),
                "Data not matched.");
    chThdSleepMilliseconds(500);
	LOG_PRINT("\tCan read bytes :%d, And data matched.\n\n",j);
  }
}

ROMCONST testcase_t can1_to_can2 = {
  "can1_to_can2",
  can1_to_can2_setup,
  can1_to_can2_td,
  can1_to_can2_exe
};



static WORKING_AREA(waCan1RxThread, 128);
static msg_t Can1Sender(void *arg) {
  (void)arg;
  chRegSetThreadName("Can1Sender");

	chRegSetThreadName("transmitter");
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.EID = 0x01234567;

  uint32_t i = 0;
  Thread * serial_receiver = NULL;
  uint32_t sbytes = 0;
  while (TRUE) {
    /* Wait msg from serial_receiver */
    serial_receiver = chMsgWait();
    /* Read the msg which stand for bytes the receiver expect to receive. */
    sbytes = chMsgGet(serial_receiver);
	txmsg.DLC = sbytes;
    /* Call chMsgRelease to release receiver so it can continue to receive */
    chMsgRelease(serial_receiver, 0);
    for (i = 0; i < sbytes; i++) {
	  txmsg.data8[i] = i;
    }
	LOG_PRINT("\tTimes:%d.\n",sbytes);
	LOG_PRINT("\tCan wirte bytes:%d.\n",sbytes);
	canTransmit(&CAND2, CAN_ANY_MAILBOX, &txmsg, TIME_INFINITE);

    ToggleLED(6);
  }

  return RDY_OK;
}


static void can2_to_can1_setup(void) {
//  canStart(&CAND1, &cancfg);
//  canStart(&CAND2, &cancfg);
  s_senderp = chThdCreateStatic(waCan1RxThread, sizeof waCan1RxThread,
                                NORMALPRIO - 20, Can1Sender, NULL);
}

static void can2_to_can1_td(void) {
}

/* SERIAL3_T1 serialPolledDelay test */
static void can2_to_can1_exe(void) {
  /*
   * Tell sender thread to send some bytes and then read those bytes and
   * examine the count of bytes and compare read and written data.
   */
  uint32_t j = 0;

  LOG_PRINT("\n\tConnect CAN1 and CAN2 together to pass the test or not"
            " to fail the test.\n\n");
  chThdSleepSeconds(1);
  for (j = 1; j <= MAX_WRITE_TIMES; j++) {
    /* Tell sender to send 'j' bytes to me. */
    chMsgSend(s_senderp, (msg_t)j);
    test_assert(j,
                RDY_OK == canReceive(&CAND1, CAN_ANY_MAILBOX, &s_rxmsg, TIME_INFINITE),
                "Return bytes doesn't match");
    test_assert(j,
                !memcmp(s_rxmsg.data8, txmsg.data8, j),
                "Data not matched.");
    chThdSleepMilliseconds(500);
	LOG_PRINT("\tCan read bytes :%d, And data matched.\n\n",j);
  }
}

ROMCONST testcase_t can2_to_can1 = {
  "can2_to_can1",
  can2_to_can1_setup,
  can2_to_can1_td,
  can2_to_can1_exe
};

static void can_timeout_setup(void) {
 	canStart(&CAND1, &cancfg);
}

static volatile systime_t s_start_st = 0;
static void can_timeout_exe(void) { 
    s_start_st = chTimeNow();
	msg_t ret = canReceive(&CAND1, CAN_ANY_MAILBOX, &s_rxmsg, MS2ST(200));
    test_assert(1,
			    ret == RDY_TIMEOUT,
                "\tcanReceive do not timeout.");
    test_assert_time_window(1, s_start_st + MS2ST(200) - 1, s_start_st + MS2ST(200) + 1);
}

ROMCONST testcase_t can_timeout_test = {
  "can_timeout_test",
  can_timeout_setup,
  NULL,
  can_timeout_exe
};

/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_can[] = {
  &can_timeout_test,
  &can1_to_can2,
  &can2_to_can1,
  NULL
};


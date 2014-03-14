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

#include "ch.h"
#include "hal.h"

struct can_instance {
  CANDriver     *canp;
};

static const struct can_instance can1 = {&CAND1};
static const struct can_instance can2 = {&CAND2};

/*
 * Internal loopback mode, 40KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the LPC reference manual.
 */
static const CANConfig cancfg = {
  0x500f9 //40kb
};

/*
 * Receiver thread.
 */
void printMsg(CANRxFrame *msg) {
	int i;
	LOG_PRINT("\n\nID:0x%x", msg->EID);
	LOG_PRINT(" BUF:");
	for(i=0; i<msg->DLC; i++){
		LOG_PRINT("%02x ",msg->data8[i]);
	}
}

static WORKING_AREA(can_rx1_wa, 256);
static WORKING_AREA(can_rx2_wa, 256);
static msg_t can_rx(void *p) {
  struct can_instance *cip = p;
  CANRxFrame rxmsg;
  msg_t ret;

  (void)p;
  chRegSetThreadName("receiver");
  while(!chThdShouldTerminate()) {
	  while(1) {
		ret = canReceive(cip->canp, CAN_ANY_MAILBOX, &rxmsg, 100);
		  /* Process message.*/
		if(ret == RDY_OK) {
			printMsg(&rxmsg);
			continue;
		}
		break;
	  }
    chThdSleepMilliseconds(20);
  }
  return 0;
}


/*
 * Transmitter thread.
 */
static WORKING_AREA(can_tx_wa, 256);
static msg_t can_tx(void * p) {
  CANTxFrame txmsg;

  (void)p;
  chRegSetThreadName("transmitter");
  txmsg.IDE = CAN_IDE_EXT;
  txmsg.RTR = CAN_RTR_DATA;

  while (!chThdShouldTerminate()) {
    txmsg.EID = 0x01234567;
    txmsg.DLC = 8;
    txmsg.data32[0] = 0xaaAAaaAA;
    txmsg.data32[1] = 0xaaFFa0FF;

    canTransmit(&CAND2, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));

#if 0
    txmsg.EID = 0x0123123;
    txmsg.DLC = 4;
    txmsg.data32[0] = 0x99999999;
    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, MS2ST(100));
#endif
    chThdSleepMilliseconds(100);
  }
  return 0;
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

   chThdSleepMilliseconds(1000);
    /*
   * Test procedure.
   */
  LOG_PRINT("\n*** ChibiOS/RT CAN Test\n");
  LOG_PRINT("***\n");

  /*
   * Activates the CAN drivers 1 and 2.
   */
 // canStart(&CAND1, &cancfg);
  canStart(&CAND2, &cancfg);

  /*
   * Starting the transmitter and receiver threads.
   */
  LOG_PRINT("Test Start\n");
//  chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7, can_rx, (void *)&can1);

  chThdCreateStatic(can_rx2_wa, sizeof(can_rx2_wa), NORMALPRIO + 7, can_rx, (void *)&can2);

  chThdCreateStatic(can_tx_wa, sizeof(can_tx_wa), NORMALPRIO + 7, can_tx, NULL);

  LOG_PRINT("Test Running\n");
  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (TRUE) {
    chThdSleepMilliseconds(1000);
  }
  return 0;
}

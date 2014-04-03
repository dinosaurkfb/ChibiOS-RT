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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "test_hal.h"
#include "devices_lib.h"
#include "update.h"

/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
/* EEPROM I2C interface config */
static const I2CConfig eeprom_i2ccfg = {
    I2C_STANDARD_MODE,
    400000
};

extern ROMCONST testcase_t * ROMCONST pattern_i2c[];
extern ROMCONST testcase_t * ROMCONST pattern_gpt[];
extern ROMCONST testcase_t * ROMCONST pattern_ext[];
extern ROMCONST testcase_t * ROMCONST pattern_serial[];
extern ROMCONST testcase_t * ROMCONST pattern_spi[];
extern ROMCONST testcase_t * ROMCONST pattern_can[];
extern ROMCONST testcase_t * ROMCONST pattern_ican[];

/*
 * Array of all the test patterns.
 */
static ROMCONST testcase_t * ROMCONST *patterns[] = {
  pattern_i2c,
  pattern_gpt,
  pattern_ext,
  pattern_serial,
  pattern_spi,
  pattern_can,
  pattern_ican,
  NULL
};

/*===========================================================================*/
/* Generic demo code.                                                        */
/*===========================================================================*/


/*===========================================================================*/
/* Test related code.                                                        */
/*===========================================================================*/
/** @brief Driver configuration.*/
static const SerialConfig uart2_config = {
  38400,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};


/* Print can frame */
void printMsg(CANRxFrame *msg) {
	int i;
	LOG_PRINT("\n\nID:0x%x", msg->EID);
	LOG_PRINT(" BUF:");
	for(i=0; i<msg->DLC; i++){
		LOG_PRINT("%02x ",msg->data8[i]);
	}
}

static uint8_t r_buf[256];

/*
 * Application entry point.
 */
int main(void) {
  msg_t result;
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  CANRxFrame rxmsg;
  CANTxFrame txmsg;

  updateThreadStart();
  EEPROMInit(&I2CD1, &eeprom_i2ccfg);

  result = TestThread(&SD1, &patterns);
  LOG_PRINT("TestThread return %d\n", result);
  size_t ret;
  sdStart(&SD3, &uart2_config);
  while(TRUE) {
    if(RDY_OK == canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, MS2ST(10))) {
      printMsg(&rxmsg);
      txmsg.IDE = rxmsg.IDE;
      txmsg.RTR = rxmsg.RTR;
      txmsg.EID = rxmsg.EID;
      txmsg.DLC = rxmsg.DLC;
      memcpy(txmsg.data8, rxmsg.data8, rxmsg.DLC);
      canTransmit(&CAND1, CAN_ANY_MAILBOX, (const CANTxFrame *)&txmsg, MS2ST(20));
    }
    ret = sdReadTimeout(&SD3, r_buf, sizeof(r_buf), MS2ST(10));
    if (ret > 0) {
      memdump(r_buf, ret);
      sdWriteTimeout(&SD3, r_buf, sizeof(r_buf), MS2ST(10));
    }
  }
}




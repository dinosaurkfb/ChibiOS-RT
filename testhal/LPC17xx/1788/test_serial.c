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
#define MAX_WRITE_TIMES 100

#define BUF_SIZE  256
static uint8_t w_buf[BUF_SIZE];
static uint8_t r_buf[BUF_SIZE];

static Thread *s_senderp;

/** @brief Driver configuration.*/
static const SerialConfig uart2_config = {
  38400,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};

static WORKING_AREA(waSerial3RxThread, 128);
static msg_t Serial3Sender(void *arg) {
  (void)arg;
  chRegSetThreadName("Serial3Sender");
  uint32_t i = 0;
  Thread * serial_receiver = NULL;
  uint32_t sbytes = 0;
  while (TRUE) {
    /* Wait msg from serial_receiver */
    serial_receiver = chMsgWait();
    /* Read the msg which stand for bytes the receiver expect to receive. */
    sbytes = chMsgGet(serial_receiver);
    /* Call chMsgRelease to release receiver so it can continue to receive */
    chMsgRelease(serial_receiver, 0);
    for (i = 0; i < sbytes; i++) {
      w_buf[i] = i + sbytes;
    }
    chSequentialStreamWrite((BaseSequentialStream  *)&SD3, w_buf, sbytes);
    ToggleLED(6);
  }

  return RDY_OK;
}

static void serial3_t1_setup(void) {
  sdStart(&SD3, &uart2_config);
}

static void serial3_t1_td(void) {
  sdStop(&SD3);
}

/* SERIAL3_T1 serialPolledDelay test */
static void serial3_t1_exe(void) {
  uint32_t j = 0;
  /*
   * Write data and then examine the return value.
   */
  uint32_t i = 0;
  for (j = 1; j < MAX_WRITE_TIMES; j++) {
    for (i = 0; i < BUF_SIZE; i++) {
      w_buf[i] = i + j;
    }
    test_assert(j,
                j == chSequentialStreamWrite((BaseSequentialStream  *)&SD3, w_buf, j),
                "Write failed");
  }
}

ROMCONST testcase_t serial3_t1 = {
  "serial3_t1",
  serial3_t1_setup,
  serial3_t1_td,
  serial3_t1_exe
};

static void serial3_t2_setup(void) {
  sdStart(&SD3, &uart2_config);
  s_senderp = chThdCreateStatic(waSerial3RxThread, sizeof waSerial3RxThread,
                                NORMALPRIO - 20, Serial3Sender, NULL);
}

static void serial3_t2_td(void) {
  sdStop(&SD3);
}

/* SERIAL3_T1 serialPolledDelay test */
static void serial3_t2_exe(void) {
  /*
   * Tell sender thread to send some bytes and then read those bytes and
   * examine the count of bytes and compare read and written data.
   */
  uint32_t j = 0;
  BaseSequentialStream *bssp = (BaseSequentialStream  *)&SD3;

  LOG_PRINT("\nConnect RxD and TxD of UART2 together to pass the test or not"
            " to fail the test.\n\n");
  chThdSleepSeconds(3);
  for (j = 1; j < MAX_WRITE_TIMES; j++) {
    /* Tell sender to send 'j' bytes to me. */
    chMsgSend(s_senderp, (msg_t)j);
    test_assert(j,
                j == chSequentialStreamRead(bssp, r_buf, j),
                "Return bytes doesn't match");
    test_assert(j+MAX_WRITE_TIMES,
                !memcmp(w_buf, r_buf, j),
                "Data not matched.");
  }
}

ROMCONST testcase_t serial3_t2 = {
  "serial3_t2",
  serial3_t2_setup,
  serial3_t2_td,
  serial3_t2_exe
};

/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_serial[] = {
  &serial3_t1,
  &serial3_t2,
  NULL
};


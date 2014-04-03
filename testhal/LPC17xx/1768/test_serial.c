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
#define READ_TIMEOUT 100

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
    sdWrite(&SD3, w_buf, sbytes);
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
                j == sdWrite(&SD3, w_buf, j),
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
  chThdTerminate(s_senderp);
  sdStop(&SD3);
}

/* SERIAL3_T1 serialPolledDelay test */
static void serial3_t2_exe(void) {
  /*
   * Tell sender thread to send some bytes and then read those bytes and
   * examine the count of bytes and compare read and written data.
   */
  uint32_t j = 0;

  LOG_PRINT("\nConnect RxD and TxD of UART2 together to pass the test or not"
            " to fail the test.\n\n");
  chThdSleepSeconds(3);
  for (j = 1; j < MAX_WRITE_TIMES; j++) {
    /* Tell sender to send 'j' bytes to me. */
    chMsgSend(s_senderp, (msg_t)j);
    test_assert(j,
                j == sdRead(&SD3, r_buf, j),
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

static void serial3_t3_setup(void) {
  sdStart(&SD3, &uart2_config);
}

static void serial3_t3_td(void) {
  sdStop(&SD3);
}

/* SERIAL3_t3 serialPolledDelay test */
static void serial3_t3_exe(void) {
  int32_t j = 0;
  /*
   * Write data and then examine the return value.
   */
  test_assert(0, TRUE == sdGetWouldBlock(&SD3), "Serial buffer not empty.");
  for (j = 1; j <= SERIAL_BUFFERS_SIZE; j++) {
    test_assert(j,
                Q_OK == sdPut(&SD3, j),
                "sdPut byte failed");
    test_assert(j + SERIAL_BUFFERS_SIZE,
                j == sdGet(&SD3),
                "sdGet byte failed");
  }
}


ROMCONST testcase_t serial3_t3 = {
  "serial3_t3",
  serial3_t3_setup,
  serial3_t3_td,
  serial3_t3_exe
};

static void serial3_t4_setup(void) {
  sdStart(&SD3, &uart2_config);
}

static void serial3_t4_td(void) {
  sdStop(&SD3);
}

/* SERIAL3_t4 serialPolledDelay test */
static void serial3_t4_exe(void) {
  int32_t j = 0;
  /*
   * Keep putting bytes until the buffer is full.
   */
  test_assert(0, TRUE == sdGetWouldBlock(&SD3), "Serial buffer not empty.");
  while (TRUE) {
    j++;
    if (!sdPutWouldBlock(&SD3)) {
      sdPut(&SD3, j);
    } else {
      test_assert(j, TRUE == sdPutWouldBlock(&SD3), "Serial buffer not full.");
      break;
    }
  }
}

ROMCONST testcase_t serial3_t4 = {
  "serial3_t4",
  serial3_t4_setup,
  serial3_t4_td,
  serial3_t4_exe
};

static void serial3_t5_setup(void) {
  sdStart(&SD3, &uart2_config);
}

static void serial3_t5_td(void) {
  sdStop(&SD3);
}

/* SERIAL3_t5 serialPolledDelay test */
static void serial3_t5_exe(void) {
  /*
   * When sender is not present, wait for bytes to test timeout.
   */
  systime_t start_st = chTimeNow();
  systime_t t_o = MS2ST(READ_TIMEOUT);
  sdGetTimeout(&SD3, t_o);
  test_assert_time_window(1, start_st + t_o - 1, start_st + t_o + 1);

  start_st = chTimeNow();
  sdReadTimeout(&SD3, r_buf, sizeof(r_buf), t_o);
  test_assert_time_window(2, start_st + t_o - 1, start_st + t_o + 1);

  /*
   * sdAsynchronousRead should return immediately when no data in buffer.
   */
  start_st = chTimeNow();
  sdAsynchronousRead(&SD3, r_buf, sizeof(r_buf));
  test_assert_time_window(2, start_st, start_st + 1);
}

ROMCONST testcase_t serial3_t5 = {
  "serial3_t5",
  serial3_t5_setup,
  serial3_t5_td,
  serial3_t5_exe
};

static void serial3_t6_setup(void) {
  sdStart(&SD3, &uart2_config);
  s_senderp = chThdCreateStatic(waSerial3RxThread, sizeof waSerial3RxThread,
                                NORMALPRIO - 20, Serial3Sender, NULL);
}

static void serial3_t6_td(void) {
  chThdTerminate(s_senderp);
  sdStop(&SD3);
}

/* SERIAL3_T1 serialPolledDelay test */
static void serial3_t6_exe(void) {
  /*
   * Tell sender thread to send some bytes and then read those bytes and
   * examine the count of bytes and compare read and written data.
   */
  uint32_t j = 0;
  systime_t t_o = MS2ST(READ_TIMEOUT);
  systime_t start_st, end_t;

  LOG_PRINT("\nConnect RxD and TxD of UART2 together to pass the test or not"
            " to fail the test.\n\n");
  chThdSleepSeconds(1);
  for (j = 1; j < MAX_WRITE_TIMES; j++) {
    /*
     * Tell sender to send 'j' bytes to me, but read '2*j' bytes to test timeout
     * and received bytes.
     */
    chMsgSend(s_senderp, (msg_t)j);
    t_o = MS2ST(j);
    start_st = chTimeNow();
    test_assert(j,
                j == sdReadTimeout(&SD3, r_buf, j * 2, t_o),
                "Return bytes doesn't match");

    test_assert_time_window(j + MAX_WRITE_TIMES,
                            start_st + t_o - (t_o/2 + 1),
                            start_st + t_o + (t_o/2 + 1));

    end_t = chTimeNow();
    /* LOG_PRINT("start=%d, now=%d, t_o=%d, dt=%d\n", start_st, end_t, t_o, end_t - start_st); */
  }
}

ROMCONST testcase_t serial3_t6 = {
  "serial3_t6",
  serial3_t6_setup,
  serial3_t6_td,
  serial3_t6_exe
};

/**
 * @brief   Test sequence for eeprom.
 */
ROMCONST testcase_t * ROMCONST pattern_serial[] = {
  &serial3_t1,
  &serial3_t2,
  &serial3_t3,
  &serial3_t4,
  &serial3_t5,
  &serial3_t6,
  NULL
};


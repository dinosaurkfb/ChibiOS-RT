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

#include "string.h"

#include "update.h"
/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
#define NO_TEST  TRUE
#if ENABLE_IAP
static WORKING_AREA(waUpdaterThread, 128);
static msg_t UpdaterThread(void *arg) {
  (void)arg;
  LOG_PRINT("*** Thread updater.\n");
  chRegSetThreadName("updater");

  /* Work loop.*/
  while (TRUE) {
    /* Waiting for a update packet.*/
    uart0_scan();
  }
  return RDY_OK;
}
#endif /* #if ENABLE_IAP */

#define BUF_SIZE  16
uint8_t w_buf[BUF_SIZE];
uint8_t r_buf2[BUF_SIZE];
uint8_t r_buf3[BUF_SIZE];

/** @brief Driver configuration.*/
static const SerialConfig uart1_config = {
  38400,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};
/** @brief Driver configuration.*/
static const SerialConfig uart2_config = {
  38400,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};

static WORKING_AREA(waSerial2RxThread, 128);
static msg_t Serial2Receiver(void *arg) {
  (void)arg;
  size_t ret = 0;

  LOG_PRINT("*** Thread Serial2Receiver started.\n");
  chRegSetThreadName("Serial2Receiver");

  /* Work loop.*/
  while (TRUE) {
    ret = chSequentialStreamRead((BaseSequentialStream  *)&SD2, r_buf2, BUF_SIZE);
    if (ret > 0) {
      LOG_PRINT("Serial2 ret=%d\n", ret);
      memdump(r_buf2, ret);
      ToggleLED(3);
    }
  }
  return RDY_OK;
}

static WORKING_AREA(waSerial3RxThread, 128);
static msg_t Serial3Receiver(void *arg) {
  (void)arg;
  size_t ret = 0;
  LOG_PRINT("*** Thread Serial3Receiver started.\n");
  chRegSetThreadName("Serial3Receiver");

  /* Work loop.*/
  while (TRUE) {
    ret = chSequentialStreamRead((BaseSequentialStream  *)&SD3, r_buf3, BUF_SIZE);
    if (ret > 0) {
      LOG_PRINT("Serial3 ret=%d\n", ret);
      memdump(r_buf3, ret);
      ToggleLED(4);
    }
  }
  return RDY_OK;
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

  /*
   * Test procedure.
   */
  LOG_PRINT("\n*** ChibiOS/RT IRQ-STORM long duration test\n");
  LOG_PRINT("***\n");
  LOG_PRINT("*** Kernel:       %s\n", CH_KERNEL_VERSION);
#ifdef CH_COMPILER_NAME
  LOG_PRINT("*** Compiler:     %s\n", CH_COMPILER_NAME);
#endif
  LOG_PRINT("*** Architecture: %s\n", CH_ARCHITECTURE_NAME);
#ifdef CH_CORE_VARIANT_NAME
  LOG_PRINT("*** Core Variant: %s\n", CH_CORE_VARIANT_NAME);
#endif
#ifdef CH_PORT_INFO
  LOG_PRINT("*** Port Info:    %s\n", CH_PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  LOG_PRINT("*** Platform:     %s\n", PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  LOG_PRINT("*** Test Board:   %s\n", BOARD_NAME);
#endif
  LOG_PRINT("***\n");
  LOG_PRINT("*** System Clock: %u\n", LPC17xx_CCLK);
  LOG_PRINT("*** Peripheral Clock: %u\n", LPC17xx_PCLK);
  /*
   * Normal main() thread activity, nothing in this test.
   */
#ifdef NO_TEST
  uint32_t s = 0;
#endif

#ifdef ENABLE_IAP
  chThdCreateStatic(waUpdaterThread, sizeof waUpdaterThread,
		    NORMALPRIO - 20, UpdaterThread, NULL);
  chThdSleepMilliseconds(20);
#endif /* #ifdef ENABLE_IAP */

  sdStart(&SD2, &uart1_config);
  sdStart(&SD3, &uart2_config);

  size_t ret = 0;
  LOG_PRINT("\n*** Serial 2, 3 read and write test\n");
  chThdCreateStatic(waSerial3RxThread, sizeof waSerial3RxThread,
		    NORMALPRIO - 20, Serial3Receiver, NULL);
  chThdCreateStatic(waSerial2RxThread, sizeof waSerial2RxThread,
		    NORMALPRIO - 20, Serial2Receiver, NULL);
  uint32_t i = 0;
  while (TRUE) {
    for (i = 0; i < BUF_SIZE; i++) {
      w_buf[i] = i + s;
    }
    ret = chSequentialStreamWrite((BaseSequentialStream  *)&SD2, w_buf, BUF_SIZE);
    if (ret > 0) {
      ToggleLED(1);
    }
    ret = chSequentialStreamWrite((BaseSequentialStream  *)&SD3, w_buf, BUF_SIZE);
    if (ret > 0) {
      ToggleLED(2);
      s++;
    }
    chThdSleepMilliseconds(500);
  }
  return 0;
}

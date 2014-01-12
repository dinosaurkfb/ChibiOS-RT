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
#include "chprintf.h"

/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* Generic demo code.                                                        */
/*===========================================================================*/

static void print(char *p) {

  while (*p) {
    chSequentialStreamPut(&SD1, *p++);
  }
}

static void println(char *p) {

  while (*p) {
    chSequentialStreamPut(&SD1, *p++);
  }
  chSequentialStreamWrite(&SD1, (uint8_t *)"\r\n", 2);
}

static void printn(uint32_t n) {
  char buf[16], *p;

  if (!n)
    chSequentialStreamPut(&SD1, '0');
  else {
    p = buf;
    while (n)
      *p++ = (n % 10) + '0', n /= 10;
    while (p > buf)
      chSequentialStreamPut(&SD1, *--p);
  }
}

/** @brief Driver default configuration.*/
static const SerialConfig my_config = {
  115200,
  LCR_WL8 | LCR_STOP1 | LCR_NOPARITY,
  FCR_TRIGGER0
};


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
   * Prepares the Serial driver 2 and GPT drivers 1 and 2.
   */
  sdStart(&SD1, &my_config);          /* Default is 38400-8-N-1. But here use my_config. */

  /*
   * Test procedure.
   */
  println("");
  println("*** ChibiOS/RT IRQ-STORM long duration test");
  println("***");
  print("*** Kernel:       ");
  println(CH_KERNEL_VERSION);
#ifdef CH_COMPILER_NAME
  print("*** Compiler:     ");
  println(CH_COMPILER_NAME);
#endif
  print("*** Architecture: ");
  println(CH_ARCHITECTURE_NAME);
#ifdef CH_CORE_VARIANT_NAME
  print("*** Core Variant: ");
  println(CH_CORE_VARIANT_NAME);
#endif
#ifdef CH_PORT_INFO
  print("*** Port Info:    ");
  println(CH_PORT_INFO);
#endif
#ifdef PLATFORM_NAME
  print("*** Platform:     ");
  println(PLATFORM_NAME);
#endif
#ifdef BOARD_NAME
  print("*** Test Board:   ");
  println(BOARD_NAME);
#endif
  println("***");
  print("*** System Clock: ");
  printn(LPC17xx_CCLK);
  println("");
  println("Test Complete");
  //  chprintf("test chprintf");
  /*
   * Normal main() thread activity, nothing in this test.
   */
  while (TRUE) {
    chThdSleepMilliseconds(1000);
    println("*** Alive.   ");
  }
  return 0;
}

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

/*===========================================================================*/
/* Configurable settings.                                                    */
/*===========================================================================*/
#define TEST_EXT TRUE

#define NO_TEST  TRUE

#if (TEST_EXT == TRUE)
#undef NO_TEST
#endif

/*===========================================================================*/
/* Generic demo code.                                                        */
/*===========================================================================*/

#if (TEST_EXT == TRUE) 
/* Triggered when button is released. LED5 is set to ON or OFF alternately.*/
static void extcb2(EXTDriver *extp, expchannel_t channel) {
  static bool status = TRUE;
  if (status == FALSE) {
    LEDON(5);
    status = TRUE;
  } else {
    LEDOFF(5);
    status = FALSE;
  }
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART, extcb2},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};
#endif

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


#if (TEST_EXT == TRUE)
  /*
   * Activates the EXT driver 1.
   */
  extStart(&EXTD1, &extcfg);
#endif

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
  /*
   * Normal main() thread activity, nothing in this test.
   */
#ifdef NO_TEST
  uint32_t s = 0;
#endif

#if (TEST_EXT == TRUE)
  LOG_PRINT("\n*** EXT Test with KEY2 and LD5.\n");
#endif

  while (TRUE) {
#ifdef NO_TEST
    chThdSleepMilliseconds(1000);
    ++s;
    LOG_PRINT("*** Alive %u seconds.\n", s);
#endif

#if (TEST_EXT == TRUE)
    LOG_PRINT("EXT Channel 2 enabled for 10 seconds, "
	      "Press KEY2, LD5 will switch ON and OFF.\n");
    chThdSleepMilliseconds(10000);
    extChannelDisable(&EXTD1, 2);
    LOG_PRINT("EXT Channel 2 disabled for 10 seconds, "
	      "No effect when pressing KEY2.\n\n");
    chThdSleepMilliseconds(10000);
    extChannelEnable(&EXTD1, 2);
#endif
  }
  return 0;
}
